#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BNO08x.hpp"
#include "imu_profile.h"

#include <atomic>

extern "C" {
#include "battery/battery.h"
#include "ble/ble.h"
#include "board/pinmap.h"
#include "button/button.h"
#include "cal/cal.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "power/power.h"
#include "preshot/preshot.h"
#include "shot_detect/shot_detect.h"
#include "stability/stability.h"
#include "supervisor/supervisor.h"
}

static const char *TAG = "main";

/** 4 × 500 ms = 2 s long-press → deep sleep. */
static constexpr int DEEP_SLEEP_HOLD_TICKS = 4;

/** Status-task period. */
static constexpr uint32_t STATUS_PERIOD_MS = 500;

/** IMU stream is treated as dead after this long with no sample. The
 *  vendor `data_available()` timeout is 3 s, so 5 s clears one stalled
 *  poll cycle without false positives. */
static constexpr uint32_t IMU_STALL_TIMEOUT_MS = 5000;

/** Continuous status-task ticks of fresh IMU data before the boot is
 *  declared healthy and the supervisor fault budget is cleared.
 *  60 × 500 ms = 30 s. */
static constexpr int IMU_HEALTHY_TICKS = 60;

/** status-task ticks after boot by which imu_task must have produced
 *  its first sample; past this with no data the bring-up is treated as
 *  hung. 40 × 500 ms = 20 s — bring-up normally completes in ~3 s. */
static constexpr int IMU_BRINGUP_TIMEOUT_TICKS = 40;

/** Inactivity auto-sleep: with no device motion for this long — per the
 *  BNO085 stability classifier — the device deep-sleeps, in or out of a
 *  session. Flip @ref AUTO_SLEEP_ENABLED to false for battery testing. */
static constexpr bool AUTO_SLEEP_ENABLED = true;
static constexpr uint32_t AUTO_SLEEP_TIMEOUT_MS = 5 * 60 * 1000;

/**
 * @brief Set by imu_task after `initialize()` returns, read by
 * status_task on the long-press shutdown path. Nullptr during the
 * brief window between status_task starting and imu_task finishing
 * init.
 */
static std::atomic<BNO08x *> g_imu_ref{nullptr};

/**
 * @brief Cooperative stop handshake for imu_task.
 *
 * The long-press shutdown runs on status_task, but imu_task is higher
 * priority and would keep driving the BNO08x and SPI bus all through
 * the teardown. status_task raises @ref g_imu_stop and waits for
 * imu_task to acknowledge via @ref g_imu_stopped and suspend itself at
 * a safe point — its loop top, holding no vendor lock.
 */
static std::atomic<bool> g_imu_stop{false};
static std::atomic<bool> g_imu_stopped{false};

/**
 * @brief esp_timer timestamp (ms, wraps at ~49 days) of the last IMU
 * sample imu_task produced; 0 until the first sample. status_task
 * watches this to catch a wedged BNO08x — see @ref IMU_STALL_TIMEOUT_MS.
 *
 * A 32-bit value is natively lock-free on this core; unsigned
 * subtraction keeps the gap correct across the wrap.
 */
static std::atomic<uint32_t> g_last_sample_ms{0};

/**
 * @brief esp_timer timestamp (ms) of the most recent BNO085
 * stability-classifier report that read MOTION. Written by imu_task on
 * each fresh MOTION sample, watched by status_task for the inactivity
 * auto-sleep timeout.
 */
static std::atomic<uint32_t> g_last_motion_ms{0};

/** DCD re-save interval — see @ref maybe_save_calibration. Tunable. */
static constexpr int64_t CAL_SAVE_INTERVAL_US = 5LL * 24 * 60 * 60 * 1000000;

/** Magic marking @ref g_cal_save valid; a mismatch means a cold boot. */
static constexpr uint32_t CAL_SAVE_MAGIC = 0xCA1B5A7Eu;

/**
 * @brief Calibration-save scheduler state, kept in RTC memory.
 *
 * RTC_NOINIT survives deep sleep and esp_restart() but is garbage on a
 * cold power-on — the magic check turns that into a forced first save.
 * `last_save_us` is an `esp_timer` timestamp; esp_timer is continuous
 * across deep sleep, so the re-save interval spans sleep cycles.
 */
struct cal_save_state_t {
  uint32_t magic;
  int64_t last_save_us;
};
RTC_NOINIT_ATTR static cal_save_state_t g_cal_save;

/* ---- Forward declarations -----------------------------------------
 *
 * Static functions defined further down. Listed here so any reorder of
 * definitions stays compile-safe — the compiler sees every prototype
 * before the first use. ma_push / ma_value are intentionally omitted:
 * both depend on the moving_avg_t struct defined alongside them, so
 * the cluster stays together. */

static void shutdown_bno_and_sleep();
static bool apply_imu_profile(BNO08x &imu, const imu_report_profile_t &p);
static void status_task(void *arg);
static void maybe_save_calibration(BNO08x &imu, BNO08xAccuracy accel_acc,
                                   BNO08xAccuracy gyro_acc);
static void imu_task(void *arg);
static esp_err_t init_nvs(void);

/**
 * @brief Shutdown sequence requested by long-press on the user button.
 *
 * Suspends imu_task so it stops driving the BNO08x and SPI, disables the
 * BNO08x reports, puts the sensor into software sleep, then enters deep
 * sleep. DCD persistence is handled separately on a periodic schedule
 * (@ref maybe_save_calibration), so this path does not touch it.
 */
static void shutdown_bno_and_sleep() {
  BNO08x *imu = g_imu_ref.load(std::memory_order_acquire);
  if (imu == nullptr) {
    ESP_LOGW(TAG, "imu not initialised; sleeping without BNO shutdown");
    power_enter_deep_sleep();
    return;
  }

  /* Cooperative stop is a courtesy, not a correctness requirement: the
   * vendor BNO08x driver serialises every SH-2 command through sh2_HAL_
   * lock and every cached-report read through data_lock, both routed
   * via its own internal SPI task — so disable_all_reports() and sleep()
   * below cannot collide with imu_task's data_available / .get() calls
   * regardless of where imu_task happens to be. Stopping it just keeps
   * it from pushing fresh BLE notifications during the teardown. */
  g_imu_stop.store(true, std::memory_order_release);
  for (int i = 0; i < 100 && !g_imu_stopped.load(std::memory_order_acquire);
       i++) {
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  if (!g_imu_stopped.load(std::memory_order_acquire)) {
    ESP_LOGW(TAG, "imu_task did not suspend in 500 ms; proceeding anyway");
  }

  imu->disable_all_reports();
  imu->sleep();

  power_enter_deep_sleep();
}

/**
 * @brief Slow Core-0 housekeeping: battery, button long-press, BLE
 * battery notify forwarding.
 */
static void status_task(void *arg) {
  int btn_hold_ticks = 0;
  int boot_ticks = 0;
  int imu_healthy_ticks = 0;
  bool imu_healthy_marked = false;
  int stack_log_tick = 0;

  /* Subscribe to TWDT — a wedged status_task is otherwise invisible to
   * the system watchdog (which only watches the idle task by default). */
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

  while (true) {
    esp_task_wdt_reset();

    bool pressed = button_is_pressed();
    btn_hold_ticks = pressed ? btn_hold_ticks + 1 : 0;

    if (btn_hold_ticks >= DEEP_SLEEP_HOLD_TICKS) {
      ESP_LOGI(TAG, "button held %d ms — entering shutdown",
               btn_hold_ticks * (int)STATUS_PERIOD_MS);
      shutdown_bno_and_sleep();
    }

    /* Inactivity auto-sleep — once the device has sat motionless (set
     * down or forgotten) for AUTO_SLEEP_TIMEOUT_MS, deep-sleep it
     * regardless of session or connection state. */
    uint32_t sleep_now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (AUTO_SLEEP_ENABLED &&
        sleep_now_ms - g_last_motion_ms.load(std::memory_order_relaxed) >=
            AUTO_SLEEP_TIMEOUT_MS) {
      ESP_LOGI(TAG, "no motion for %lu s — auto-sleep",
               (unsigned long)(AUTO_SLEEP_TIMEOUT_MS / 1000));
      shutdown_bno_and_sleep();
    }

    /* IMU supervisor. A hung bring-up or a wedged BNO08x is routed to
     * the fault supervisor, which reboots up to a bounded number of
     * times. This block runs even once that budget is spent: the
     * fault-reboot escalation is then skipped (it could not reboot
     * anyway), but the healthy-run check still runs — so a transient
     * fault self-heals, clearing the budget without a power cycle. */
    {
      uint32_t last = g_last_sample_ms.load(std::memory_order_relaxed);
      uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
      bool imu_fresh = (last != 0) && (now_ms - last < IMU_STALL_TIMEOUT_MS);
      bool shutting_down = g_imu_stop.load(std::memory_order_acquire);

      if (boot_ticks < IMU_BRINGUP_TIMEOUT_TICKS) {
        boot_ticks++;
      }

      /* Fault-reboot escalation — only while the budget has room and no
       * shutdown is in progress. Once the budget is spent the call is a
       * no-op, so skip it and its per-tick log noise. */
      if (!supervisor_reboot_budget_exhausted() && !shutting_down) {
        if (last == 0) {
          /* No sample yet — reboot if bring-up has overrun its window. */
          if (boot_ticks >= IMU_BRINGUP_TIMEOUT_TICKS) {
            supervisor_fault_reboot("IMU produced no data after bring-up");
          }
        } else if (!imu_fresh) {
          /* Samples flowed and then stopped — a wedged BNO08x. */
          supervisor_fault_reboot("IMU stream stalled");
        }
      }

      /* A long uninterrupted run means any earlier fault was transient
       * and has recovered — clear the reboot budget, once. Runs in the
       * degraded state too: that is what lets degraded mode self-heal. */
      if (!imu_healthy_marked) {
        if (imu_fresh) {
          if (++imu_healthy_ticks >= IMU_HEALTHY_TICKS) {
            supervisor_mark_healthy();
            imu_healthy_marked = true;
          }
        } else {
          imu_healthy_ticks = 0;
        }
      }
    }

    float vbat = battery_get_voltage();
    int pct = battery_get_percent();
    ESP_LOGI(TAG, "vbat=%.3fV (%d%%)  btn=%s", vbat, pct,
             pressed ? "DOWN" : "up");

    if (pct >= 0) {
      ble_set_battery((uint8_t)pct);
    }

    /* Stack high-water mark — smallest free-byte count since boot. Logged
     * every 20 ticks (10 s) so we can right-size status_task's 4 KB stack. */
    if (++stack_log_tick >= 20) {
      ESP_LOGI(TAG, "stack_hwm: status=%u",
               (unsigned)uxTaskGetStackHighWaterMark(NULL));
      stack_log_tick = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(STATUS_PERIOD_MS));
  }
}

/**
 * @brief Moving-average window for the gyro/linear-accel decimation
 * filter. At the 200 Hz internal rate M=4 gives f_-3dB ~ 22 Hz (well
 * clear of the 6-12 Hz tremor band), ~6 dB white-noise SNR gain, and a
 * 7.5 ms group delay. Conservative default: raising it improves SNR and
 * anti-aliasing at the cost of attenuating the top of the tremor band.
 */
static constexpr int MA_WINDOW = 4;

/**
 * @brief Fixed-window moving average over one signal axis.
 *
 * `ma_push` writes each 200 Hz sample into a ring buffer and keeps a
 * running sum (add incoming, subtract evicted); `ma_value` returns that
 * sum divided by the window. Both are O(1) regardless of MA_WINDOW.
 * Sampling `ma_value` once per 50 Hz rv_game tick decimates the filtered
 * 200 Hz signal by 4 — see imu_task.
 */
struct moving_avg_t {
  float buf[MA_WINDOW];
  float sum;
  int idx;
  int count;
};

static void ma_push(moving_avg_t &m, float v) {
  /* Slide the running sum: drop the sample being evicted, add the new
   * one. buf[idx] is the oldest slot (0.0f until the window first
   * fills), so this stays correct through the fill transient. */
  m.sum += v - m.buf[m.idx];
  m.buf[m.idx] = v;
  m.idx = (m.idx + 1) % MA_WINDOW;
  if (m.count < MA_WINDOW) {
    m.count++;
  }
}

static float ma_value(const moving_avg_t &m) {
  return m.count > 0 ? m.sum / (float)m.count : 0.0f;
}

/**
 * @brief Persist the BNO085 DCD on a ~5-day schedule.
 *
 * The BNO085 refines calibration in RAM and reloads it from its own
 * flash on every reset; a periodic Save DCD keeps that flash copy fresh
 * so each cold start begins well-calibrated. Gated on accel and gyro
 * accuracy HIGH — a Save DCD of a poor calibration would overwrite a
 * better stored one (see cal.h) — and skipped during a BLE session,
 * where dynamic calibration is frozen and nothing new accrues.
 *
 * Called every rv_game tick; the actual save runs at most once per
 * @ref CAL_SAVE_INTERVAL_US, plus once right after a cold boot.
 */
static void maybe_save_calibration(BNO08x &imu, BNO08xAccuracy accel_acc,
                                   BNO08xAccuracy gyro_acc) {
  if (accel_acc != BNO08xAccuracy::HIGH || gyro_acc != BNO08xAccuracy::HIGH ||
      ble_session_active()) {
    return;
  }

  int64_t now_us = esp_timer_get_time();
  bool rtc_valid = (g_cal_save.magic == CAL_SAVE_MAGIC);
  /* esp_timer is continuous across deep sleep, so the interval spans
   * sleep cycles. A cold power-on leaves the RTC block garbage (caught
   * by rtc_valid) and forces the first save. */
  if (rtc_valid && now_us - g_cal_save.last_save_us < CAL_SAVE_INTERVAL_US) {
    return;
  }

  bool ok = imu.dynamic_calibration_save();
  g_cal_save.magic = CAL_SAVE_MAGIC;
  g_cal_save.last_save_us = now_us;
  if (!ok) {
    ESP_LOGW("imu", "periodic DCD save failed");
    return;
  }

  cal_state_t cs;
  if (cal_load(&cs) != ESP_OK) {
    return;
  }
  cs.dcd_ever_saved = true;
  cs.dcd_save_count++;
  cal_store(&cs);
  ble_set_calibration_state(cs.dcd_ever_saved, cs.dcd_save_count);
  ESP_LOGI("imu", "DCD saved (count=%lu)", (unsigned long)cs.dcd_save_count);
}

/**
 * @brief Drive the BNO085 reports to match @p p.
 *
 * Enables each report at its profile period, or disables it when the period
 * is 0. Enabling an already-enabled report at a new period re-rates it (SH-2
 * Set Feature). The target accelerometer report is enabled before the other
 * accel report is disabled, so the accel sensor — and thus its dynamic
 * calibration — never gaps across a session switch. cal_gyro and the
 * stability classifier run in both profiles and are only re-rated, never
 * dropped, so the housekeeping path (cal_gyro) and the auto-sleep motion
 * source (stability) never gap either.
 *
 * @return true if every report op succeeded.
 */
static bool apply_imu_profile(BNO08x &imu, const imu_report_profile_t &p) {
  bool ok = true;

  ok &= p.stability_us ? imu.rpt.stability_classifier.enable(p.stability_us)
                       : imu.rpt.stability_classifier.disable();
  ok &= p.cal_gyro_us ? imu.rpt.cal_gyro.enable(p.cal_gyro_us)
                      : imu.rpt.cal_gyro.disable();

  /* Enable the target accel report before disabling the other one. */
  if (p.accel_us) {
    ok &= imu.rpt.accelerometer.enable(p.accel_us);
  }
  if (p.linear_accel_us) {
    ok &= imu.rpt.linear_accelerometer.enable(p.linear_accel_us);
  }
  if (!p.accel_us) {
    ok &= imu.rpt.accelerometer.disable();
  }
  if (!p.linear_accel_us) {
    ok &= imu.rpt.linear_accelerometer.disable();
  }

  /* Fusion engine last — heaviest, nothing else depends on it. */
  ok &= p.rv_game_us ? imu.rpt.rv_game.enable(p.rv_game_us)
                     : imu.rpt.rv_game.disable();

  return ok;
}

/**
 * @brief IMU sample consumer, pinned to Core 0. The vendor driver owns
 * SPI, the INT ISR, and SH-2 service on its own tasks; this loop
 * oversamples gyro + linear-accel at 200 Hz, moving-average filters and
 * decimates them, and forwards a fused 50 Hz sample to BLE.
 *
 * Must stay on Core 0. Pinning it to Core 1 was tried and hangs the
 * BNO08x report-enable sequence during bring-up — the vendor driver is
 * only stable with its SPI bus and ISRs created from Core 0.
 */
static void imu_task(void *arg) {
  ESP_LOGI("imu", "imu_task started on core %d", xPortGetCoreID());

  /* SCLK 3 MHz is the BNO085 datasheet ceiling. */
  bno08x_config_t cfg(SPI2_HOST, PIN_BNO_MOSI, PIN_BNO_MISO, PIN_BNO_SCK,
                      PIN_BNO_CS, PIN_BNO_INT, PIN_BNO_RST, 3000000, true);

  static BNO08x imu(cfg);
  if (!imu.initialize()) {
    /* Reboots unless the budget is spent; if it returns, run degraded. */
    supervisor_fault_reboot("BNO08x initialize() failed");
    ESP_LOGE("imu", "IMU bring-up abandoned; running without IMU");
    vTaskDelete(nullptr);
    return;
  }
  g_imu_ref.store(&imu, std::memory_order_release);

  /* ACCEL|GYRO only; MAG is unreliable around archery hardware. The
   * 0x07 mask (incl. MAG) makes BNO085 firmware 3.2.6.13 silently fall
   * back to 0x05 with gyro stripped — see
   * project_bno085_gyro_accuracy_quirk.md. */
  auto cal_cfg = static_cast<BNO08xCalSel>(
      static_cast<uint8_t>(BNO08xCalSel::accelerometer) |
      static_cast<uint8_t>(BNO08xCalSel::gyro));
  if (!imu.dynamic_calibration_enable(cal_cfg)) {
    ESP_LOGE("imu", "dynamic_calibration_enable failed");
  } else {
    ESP_LOGI("imu", "dynamic_calibration_enable(accel|gyro) OK");
  }

  /* The BLE frame is emitted at 50 Hz — the rv_game report at that rate
   * is the decimation clock. gyro + linear-accel are oversampled at
   * 200 Hz and run through a moving-average filter; sampling that filter
   * once per rv_game tick decimates by 4. The 200 Hz gyro/accel samples
   * also feed the on-device V4 shot detector. rv_game stays
   * 50 Hz native — a quaternion cannot be component-wise averaged. */
  /* Boot in the lean idle profile — no session is active yet. The full
   * streaming profile is applied on SESSION_START (see the session-edge
   * block below). The 50/200 Hz periods now live in imu_profile.h. */
  if (!apply_imu_profile(imu, imu_profile_for(false))) {
    supervisor_fault_reboot("BNO08x report enable failed");
    ESP_LOGE("imu", "IMU bring-up abandoned; running without IMU");
    vTaskDelete(nullptr);
    return;
  }
  ESP_LOGI("imu",
           "BNO085 idle profile up: cal_gyro+linAccel @ 10 Hz, "
           "stability @ 1 Hz (full stream starts on session); MA(%d)",
           MA_WINDOW);

  /* Zero-initialised: the 200 Hz path (pre-shot buffer) reads q before
   * the 50 Hz rv_game path first assigns it. */
  bno08x_quat_t q = {};
  moving_avg_t gyro_ma[3] = {};
  moving_avg_t accel_ma[3] = {};
  BNO08xAccuracy gyro_acc = BNO08xAccuracy::UNDEFINED;
  BNO08xAccuracy accel_acc = BNO08xAccuracy::UNDEFINED;
  int sample_count = 0;
  int last_logged = 0;
  bool session_prev = false;
  float sd_gyro[3] = {0.0f, 0.0f, 0.0f};

  preshot_init();
  shot_detect_init();
  stability_init();

  /* Subscribe to TWDT — without this, a wedged imu_task is only caught
   * by status_task's stream-stall watchdog, which depends on imu_task
   * having produced at least one sample. TWDT closes that gap. */
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

  while (true) {
    /* Reset at the top of the loop. With no IMU data the vendor's
     * data_available() returns false after a 3 s timeout; resetting here
     * keeps us well under the 5 s TWDT default. */
    esp_task_wdt_reset();

    if (g_imu_stop.load(std::memory_order_acquire)) {
      ESP_LOGI("imu", "stop requested — suspending imu_task");
      g_imu_stopped.store(true, std::memory_order_release);
      /* Unsubscribe before suspend, otherwise the dog starves and panics
       * the device just to bring it down for deep sleep. */
      esp_task_wdt_delete(NULL);
      vTaskSuspend(nullptr);
    }

    /* Freeze BNO085 dynamic calibration for a session's duration — a
     * mid-session recalibration step would perturb the analysed signal.
     * Resumed when the session ends. */
    bool session_now = ble_session_active();
    if (session_now != session_prev) {
      /* Switch the report profile to match the new mode, then freeze/resume
       * dynamic calibration. A failed switch is logged but not fatal — keep
       * running the current reports rather than reboot a live session. */
      if (!apply_imu_profile(imu, imu_profile_for(session_now))) {
        ESP_LOGW("imu", "profile switch failed (session=%d)", session_now);
      }
      if (session_now) {
        imu.dynamic_calibration_disable(cal_cfg);
        ESP_LOGI("imu", "session active — full stream, dynamic cal frozen");
      } else {
        imu.dynamic_calibration_enable(cal_cfg);
        ESP_LOGI("imu", "session ended — idle profile, dynamic cal resumed");
      }
      session_prev = session_now;
    }

    if (!imu.data_available()) {
      continue;
    }

    /* 200 Hz path — push every gyro/accel sample through the decimation
     * filter; the V4 shot detector and the pre-shot buffer tap the same
     * pre-decimation samples here. */
    if (imu.rpt.cal_gyro.has_new_data()) {
      bno08x_gyro_t g = imu.rpt.cal_gyro.get();
      ma_push(gyro_ma[0], g.x);
      ma_push(gyro_ma[1], g.y);
      ma_push(gyro_ma[2], g.z);
      gyro_acc = g.accuracy;
      sd_gyro[0] = g.x;
      sd_gyro[1] = g.y;
      sd_gyro[2] = g.z;

      /* Profile-independent housekeeping on the cal-gyro tick (200 Hz in a
       * session, 10 Hz idle): feed the supervisor stream-stall watchdog,
       * refresh sensor-health, and run the periodic DCD save. None of these
       * may depend on the 50 Hz rv_game frame, which is off in idle. */
      g_last_sample_ms.store((uint32_t)(esp_timer_get_time() / 1000),
                             std::memory_order_relaxed);
      ble_set_imu_health((uint8_t)accel_acc, (uint8_t)gyro_acc);
      maybe_save_calibration(imu, accel_acc, gyro_acc);
    }

    /* Linear acceleration runs in both profiles: 200 Hz DSP/stream input in a
     * session, 10 Hz keep-alive in idle. Its fused-output accuracy is what the
     * device-health card shows in both modes (HIGH once fusion has converged,
     * unlike the raw accel-cal status), and sampling it keeps ACCEL dynamic
     * calibration refining. The 200 Hz DSP below is session-only. */
    if (imu.rpt.linear_accelerometer.has_new_data()) {
      bno08x_accel_t a = imu.rpt.linear_accelerometer.get();
      accel_acc = a.accuracy;

      /* Decimation filter, pre-shot ring, and the V4 shot detector are
       * meaningful only on the 200 Hz session stream — gate them on the
       * session so the 10 Hz idle keep-alive cannot pollute the ring or emit
       * a stray shot event.
       * `q` is the most recent Game-RV quaternion (50 Hz native),
       * intentionally stale 3 of 4 ticks — the preshot ring keeps the raw
       * 200 Hz accel/gyro for post-shot release-transient analysis; the
       * shot-review aim trace sources the smooth quaternion stream from the
       * 50 Hz session chunks instead, not from this blob. */
      if (session_now) {
        ma_push(accel_ma[0], a.x);
        ma_push(accel_ma[1], a.y);
        ma_push(accel_ma[2], a.z);

        const uint64_t now_us = esp_timer_get_time();
        const float sd_accel[3] = {a.x, a.y, a.z};
        const float sd_quat[4] = {q.i, q.j, q.k, q.real};
        preshot_write((uint32_t)(now_us / 1000), sd_gyro, sd_accel, sd_quat);

        shot_event_t shot;
        if (shot_detect_tick(now_us, sd_gyro, sd_accel, &shot)) {
          uint16_t peak_centi = shot.peak_mps2 > 655.0f
                                    ? 65535u
                                    : (uint16_t)(shot.peak_mps2 * 100.0f);
          ble_emit_shot_event((uint8_t)shot.type, shot.onset_ms, peak_centi);
          if (shot.type == SHOT_EVT_PROVISIONAL) {
            preshot_mark_shot(shot.onset_ms);
          }
        }
      }
    }

    /* Inactivity auto-sleep: the BNO085 stability classifier reports
     * when the device is in motion; status_task deep-sleeps a device
     * that has stayed out of motion for AUTO_SLEEP_TIMEOUT_MS. */
    if (imu.rpt.stability_classifier.has_new_data() &&
        imu.rpt.stability_classifier.get_stability() ==
            BNO08xStability::MOTION) {
      g_last_motion_ms.store((uint32_t)(esp_timer_get_time() / 1000),
                             std::memory_order_relaxed);
    }

    /* 50 Hz path — the rv_game tick is the decimation clock: emit one
     * frame from the current filtered gyro/accel. */
    if (!imu.rpt.rv_game.has_new_data()) {
      continue;
    }
    q = imu.rpt.rv_game.get_quat();
    sample_count++;
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    ble_imu_sample_t s = {
        .timestamp_ms = now_ms,
        .quat_i = q.i,
        .quat_j = q.j,
        .quat_k = q.k,
        .quat_w = q.real,
        .gyro_rad_x = ma_value(gyro_ma[0]),
        .gyro_rad_y = ma_value(gyro_ma[1]),
        .gyro_rad_z = ma_value(gyro_ma[2]),
        .lin_accel_x = ma_value(accel_ma[0]),
        .lin_accel_y = ma_value(accel_ma[1]),
        .lin_accel_z = ma_value(accel_ma[2]),
    };
    if (ble_send_sample(&s) != ESP_OK) {
      ESP_LOGD("imu", "ble_send_sample dropped");
    }

    /* Hold-stability score, computed on the 50 Hz decimated stream; its
     * output — smoothed score + raw RMS components — is notified on the
     * metrics characteristic every frame (50 Hz), 1:1 with the IMU
     * frame so the tuning UI can align the two streams sample-for-sample. */
    const float st_gyro[3] = {ma_value(gyro_ma[0]), ma_value(gyro_ma[1]),
                              ma_value(gyro_ma[2])};
    const float st_accel[3] = {ma_value(accel_ma[0]), ma_value(accel_ma[1]),
                               ma_value(accel_ma[2])};
    const float st_quat[4] = {q.i, q.j, q.k, q.real};
    stability_output_t st;
    stability_tick(st_gyro, st_accel, st_quat, &st);
    ble_metrics_t m = {
        .timestamp_ms = now_ms,
        .score = st.score,
        .instability = st.instability,
        .rms_omega = st.rms_omega,
        .rms_accel_lf = st.rms_accel_lf,
        .rms_disp = st.rms_disp,
    };
    if (ble_send_metrics(&m) != ESP_OK) {
      ESP_LOGD("imu", "ble_send_metrics dropped");
    }

    if (sample_count - last_logged >= 100) {
      ESP_LOGI("imu",
               "[fusion=%s gyro=%s accel=%s] "
               "GRV i=%+.3f j=%+.3f k=%+.3f r=%+.3f | "
               "gyro x=%+.3f y=%+.3f z=%+.3f rad/s | "
               "lin x=%+.3f y=%+.3f z=%+.3f m/s2 | "
               "stack_hwm=%u | cnt=%d",
               BNO08xAccuracy_to_str(q.accuracy),
               BNO08xAccuracy_to_str(gyro_acc),
               BNO08xAccuracy_to_str(accel_acc), q.i, q.j, q.k, q.real,
               s.gyro_rad_x, s.gyro_rad_y, s.gyro_rad_z, s.lin_accel_x,
               s.lin_accel_y, s.lin_accel_z,
               (unsigned)uxTaskGetStackHighWaterMark(NULL), sample_count);
      last_logged = sample_count;
    }
  }
}

/**
 * @brief Initialise the default NVS partition.
 *
 * NVS backs both the BLE host (NimBLE PHY calibration, future bonding)
 * and the calibration driver (cal.c), so it is a shared dependency
 * brought up here once rather than hidden inside any one component.
 */
static esp_err_t init_nvs(void) {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  return err;
}

extern "C" void app_main(void) {
  if (power_woke_from_button()) {
    ESP_LOGI(TAG, "wake from deep sleep (button)");
    /* Release the holds we set on entry; otherwise BNO RSTN stays low
     * and the BNO08x driver's hard_reset() during init never sees the
     * line rise. Also detach PIN_BTN from the RTC IO mux so the
     * downstream button_init() can re-own it as a digital input. */
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis(PIN_BNO_RST);
    rtc_gpio_deinit(PIN_BNO_INT);
    rtc_gpio_deinit(PIN_BTN);
  } else {
    ESP_LOGI(TAG, "StabilityTracker firmware bring-up");
  }

  supervisor_init();

  ESP_ERROR_CHECK(init_nvs());
  ESP_ERROR_CHECK(battery_init());
  ESP_ERROR_CHECK(button_init());
  ESP_ERROR_CHECK(ble_init());
  ESP_ERROR_CHECK(cal_init());

  /* Seed the sensor-health characteristic with the persisted DCD-save
   * state; imu_task then refreshes it — live accuracy every tick, the
   * save count on each periodic save — once it is running. */
  cal_state_t cs;
  if (cal_load(&cs) == ESP_OK) {
    ble_set_calibration_state(cs.dcd_ever_saved, cs.dcd_save_count);
  }

  /* A core task that cannot be created leaves the device unable to do
   * its job — abort (and reboot) rather than run half-alive, matching
   * the ESP_ERROR_CHECK policy on the init calls above. */
  BaseType_t s = xTaskCreatePinnedToCore(status_task, "status", 4096, nullptr,
                                         5, nullptr, 0);
  ESP_ERROR_CHECK(s == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

  /* imu_task is always spawned, even with the reboot budget spent.
   * "Budget spent" only stops the reboot escalation — supervisor_fault_
   * reboot() is already a no-op then, so a fresh bring-up cannot
   * reboot-loop. Retrying the IMU lets a transient fault self-heal: a
   * sustained healthy run clears the budget (status_task calls
   * supervisor_mark_healthy). A genuinely dead BNO08x just fails
   * bring-up and the device runs without it, as before. */
  if (supervisor_reboot_budget_exhausted()) {
    ESP_LOGW(TAG, "fault-reboot budget spent — retrying IMU; "
                  "a healthy run clears it");
  }
  /* 6144 B: imu_task runs float DSP (shot detect + stability) and a
   * wide ESP_LOGI; 4096 was tight. Precautionary — confirm against
   * uxTaskGetStackHighWaterMark on hardware. */
  BaseType_t i =
      xTaskCreatePinnedToCore(imu_task, "imu", 6144, nullptr, 6, nullptr, 0);
  ESP_ERROR_CHECK(i == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
