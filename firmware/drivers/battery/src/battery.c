#include "battery/battery.h"
#include "board/pinmap.h"

#include <stdatomic.h>
#include <stdlib.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "battery";

/** Divider midpoint reads `vbat / 2`; multiply back to recover terminal
 * voltage. */
#define BAT_DIVIDER_RATIO 2.0f

/** Samples per pass. 16 → stable median, <2 ms total. */
#define SAMPLES_PER_UPDATE 16

/** EMA coefficient. α=0.1 → ~10 s time constant at 1 Hz updates, smooths BLE TX
 * dips. */
#define EMA_ALPHA 0.1f

/** Resample period. LiPo terminal voltage drifts on the order of mV/min under
 * normal load. */
#define UPDATE_PERIOD_MS 1000

/** Compile-time gate for per-sample telemetry. Defaults off; override with
 * `-DBATTERY_DIAG_LOG=1` during bring-up or bench testing. */
#ifndef BATTERY_DIAG_LOG
#define BATTERY_DIAG_LOG 0
#endif

#if BATTERY_DIAG_LOG
#define BAT_DIAG_LOGI(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define BAT_DIAG_LOGI(...) ((void)0)
#endif

/**
 * @brief One OCV-SoC curve point for a 1S LiPo (LiCoO₂, 300 mAh cell).
 *
 * Curve must be sorted by descending voltage; `voltage_to_percent()`
 * linearly interpolates between adjacent points.
 *
 * Upper plateau (100-15 %) uses 5 % steps from a generic LiPo profile
 * (±4 % vs Battery18650Stats reference table). Steep tail (10-0 %) uses
 * 1 % steps because the curve drops ~50 mV / %. Floor 3.20 V is set so
 * the SGM6029 buck (~30 mV dropout) still delivers ~3.17 V to VDD,
 * above the brownout detector (level 7 ≈ 3.10 V).
 */
typedef struct {
  float voltage;
  int percent;
} lipo_point_t;

static const lipo_point_t LIPO_CURVE[] = {
    {4.20f, 100}, {4.15f, 95}, {4.11f, 90}, {4.08f, 85}, {4.02f, 80},
    {3.98f, 75},  {3.95f, 70}, {3.91f, 65}, {3.87f, 60}, {3.85f, 55},
    {3.84f, 50},  {3.82f, 45}, {3.80f, 40}, {3.79f, 35}, {3.77f, 30},
    {3.75f, 25},  {3.73f, 20}, {3.71f, 15}, {3.70f, 10}, {3.65f, 9},
    {3.60f, 8},   {3.55f, 7},  {3.50f, 6},  {3.45f, 5},  {3.40f, 4},
    {3.35f, 3},   {3.30f, 2},  {3.20f, 0},
};
#define LIPO_CURVE_LEN (sizeof(LIPO_CURVE) / sizeof(LIPO_CURVE[0]))

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali = NULL;

/* Atomics so the consumer task never sees a torn float / int. */
static _Atomic float s_voltage = 0.0f;
static _Atomic int s_percent = 0;

/* ---- Forward declarations ----------------------------------------- */

static int compare_int(const void *a, const void *b);
static int voltage_to_percent(float v);
static float sample_and_smooth(void);
static void battery_task(void *arg);

static int compare_int(const void *a, const void *b) {
  int ai = *(const int *)a;
  int bi = *(const int *)b;
  return (ai > bi) - (ai < bi);
}

/**
 * @brief Linearly interpolate @p v through #LIPO_CURVE.
 *        Out-of-range voltages clamp to 0 or 100.
 */
static int voltage_to_percent(float v) {
  if (v >= LIPO_CURVE[0].voltage)
    return 100;
  if (v <= LIPO_CURVE[LIPO_CURVE_LEN - 1].voltage)
    return 0;

  for (size_t i = 0; i < LIPO_CURVE_LEN - 1; i++) {
    float v_high = LIPO_CURVE[i].voltage;
    float v_low = LIPO_CURVE[i + 1].voltage;
    if (v >= v_low) {
      int p_high = LIPO_CURVE[i].percent;
      int p_low = LIPO_CURVE[i + 1].percent;
      float t = (v - v_low) / (v_high - v_low);
      return (int)(p_low + t * (p_high - p_low) + 0.5f);
    }
  }
  return 0;
}

/**
 * @brief Median-filter @ref SAMPLES_PER_UPDATE ADC reads, fold into EMA.
 *
 * The middle-50 % trim rejects load transients and high-Z glitches.
 *
 * @return Updated smoothed voltage in volts.
 */
static float sample_and_smooth(void) {
  int samples[SAMPLES_PER_UPDATE];
  for (int i = 0; i < SAMPLES_PER_UPDATE; i++) {
    if (adc_oneshot_read(s_adc, ADC_BAT_CHANNEL, &samples[i]) != ESP_OK) {
      samples[i] = 0;
    }
  }

  qsort(samples, SAMPLES_PER_UPDATE, sizeof(int), compare_int);
  int sum = 0;
  int q1 = SAMPLES_PER_UPDATE / 4;
  int q3 = SAMPLES_PER_UPDATE - q1;
  for (int i = q1; i < q3; i++)
    sum += samples[i];
  int trimmed_raw = sum / (q3 - q1);

  int vadc_mv;
  if (s_cali != NULL) {
    if (adc_cali_raw_to_voltage(s_cali, trimmed_raw, &vadc_mv) != ESP_OK) {
      vadc_mv = (trimmed_raw * 3300) / 4095;
    }
  } else {
    vadc_mv = (trimmed_raw * 3300) / 4095;
  }
  float v_new = (vadc_mv / 1000.0f) * BAT_DIVIDER_RATIO;

  float v_prev = atomic_load(&s_voltage);
  float v_smoothed;
  if (v_prev <= 0.01f) {
    /* First post-boot sample — seed the filter to avoid a 0-V ramp. */
    v_smoothed = v_new;
  } else {
    v_smoothed = EMA_ALPHA * v_new + (1.0f - EMA_ALPHA) * v_prev;
  }
  atomic_store(&s_voltage, v_smoothed);

  BAT_DIAG_LOGI(
      "adc_raw=%d  vadc=%dmV  v_inst=%.3fV  v_ema=%.3fV  (prev=%.3fV)",
      trimmed_raw, vadc_mv, v_new, v_smoothed, v_prev);

  return v_smoothed;
}

static void battery_task(void *arg) {
  /* Subscribe to TWDT so the 1 Hz ADC loop is monitored alongside the
   * other tasks; if the ADC stalls or qsort hangs we want to know. */
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  while (1) {
    esp_task_wdt_reset();
    float v = sample_and_smooth();
    int pct = voltage_to_percent(v);
    atomic_store(&s_percent, pct);
    BAT_DIAG_LOGI("battery: %.3fV  %d%%", v, pct);
    vTaskDelay(pdMS_TO_TICKS(UPDATE_PERIOD_MS));
  }
}

esp_err_t battery_init(void) {
  adc_oneshot_unit_init_cfg_t init = {.unit_id = ADC_BAT_UNIT};
  esp_err_t err = adc_oneshot_new_unit(&init, &s_adc);
  if (err != ESP_OK)
    return err;

  adc_oneshot_chan_cfg_t cfg = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  err = adc_oneshot_config_channel(s_adc, ADC_BAT_CHANNEL, &cfg);
  if (err != ESP_OK)
    return err;

  adc_cali_curve_fitting_config_t cali_cfg = {
      .unit_id = ADC_BAT_UNIT,
      .chan = ADC_BAT_CHANNEL,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali) == ESP_OK) {
    ESP_LOGI(TAG, "ADC curve-fitting calibration enabled");
  } else {
    ESP_LOGW(TAG, "ADC calibration unavailable, using raw scaling");
    s_cali = NULL;
  }

  /* Prime the cache so the first battery_get_voltage() call returns a
   * real reading rather than 0 if it lands before the task ticks. */
  float v = sample_and_smooth();
  int pct = voltage_to_percent(v);
  atomic_store(&s_percent, pct);
  ESP_LOGI(TAG, "init: first read %.3fV  %d%%  cali=%s", v, pct,
           s_cali != NULL ? "curve-fit" : "raw");

  BaseType_t ok = xTaskCreate(battery_task, "battery", 3072, NULL, 4, NULL);
  if (ok != pdPASS)
    return ESP_ERR_NO_MEM;

  return ESP_OK;
}

float battery_get_voltage(void) { return atomic_load(&s_voltage); }

int battery_get_percent(void) { return atomic_load(&s_percent); }
