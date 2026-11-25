#include "ble/ble.h"
#include "ble_priv.h"

#include <math.h>
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "os/os_mbuf.h"
#include "preshot/preshot.h"
#include "services/bas/ble_svc_bas.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "shot_detect/shot_detect.h"
#include "stability/stability.h"

/**
 * @file ble_gatt.c
 * @brief Custom GATT service from `docs/ble/sensor-protocol.md`.
 *
 * Service 5d3ace00-…, characteristics:
 *   - 5d3ace04-… NOTIFY        IMU stream v3 (25 B @ 50 Hz)
 *   - 5d3ace02-… READ          Device info
 *   - 5d3ace03-… WRITE         Control (session, shot transfer)
 *   - 5d3ace05-… READ          Sensor health (8 B)
 *   - 5d3ace06-… NOTIFY        Shot events (9 B)
 *   - 5d3ace07-… NOTIFY        Stability metrics (26 B)
 *   - 5d3ace08-… NOTIFY        Preshot transfer (16 / 28 / 8 B)
 *   - 5d3ace09-… READ + WRITE  Shot-detect tuning (24 B)
 *   - 5d3ace0a-… READ + WRITE  Stability-score tuning (50 B)
 *
 * Plus Battery Service 0x180F via NimBLE's built-in `bas` module.
 *
 * @note `BLE_UUID128_INIT` byte order is little-endian — the human-
 *       readable LSB lands at slot [0], the MSB at [15].
 */

static const char *TAG = "ble_gatt";

static const ble_uuid128_t st_svc_uuid =
    BLE_UUID128_INIT(0xf1, 0xc7, 0xe9, 0xb3, 0xa8, 0xf6, 0x2c, 0x8d, 0x1a, 0x4b,
                     0x9f, 0x7e, 0x00, 0xce, 0x3a, 0x5d);
static const ble_uuid128_t st_chr_imu_uuid =
    BLE_UUID128_INIT(0xf1, 0xc7, 0xe9, 0xb3, 0xa8, 0xf6, 0x2c, 0x8d, 0x1a, 0x4b,
                     0x9f, 0x7e, 0x04, 0xce, 0x3a, 0x5d);
static const ble_uuid128_t st_chr_devinfo_uuid =
    BLE_UUID128_INIT(0xf1, 0xc7, 0xe9, 0xb3, 0xa8, 0xf6, 0x2c, 0x8d, 0x1a, 0x4b,
                     0x9f, 0x7e, 0x02, 0xce, 0x3a, 0x5d);
static const ble_uuid128_t st_chr_control_uuid =
    BLE_UUID128_INIT(0xf1, 0xc7, 0xe9, 0xb3, 0xa8, 0xf6, 0x2c, 0x8d, 0x1a, 0x4b,
                     0x9f, 0x7e, 0x03, 0xce, 0x3a, 0x5d);
static const ble_uuid128_t st_chr_health_uuid =
    BLE_UUID128_INIT(0xf1, 0xc7, 0xe9, 0xb3, 0xa8, 0xf6, 0x2c, 0x8d, 0x1a, 0x4b,
                     0x9f, 0x7e, 0x05, 0xce, 0x3a, 0x5d);
static const ble_uuid128_t st_chr_shot_uuid =
    BLE_UUID128_INIT(0xf1, 0xc7, 0xe9, 0xb3, 0xa8, 0xf6, 0x2c, 0x8d, 0x1a, 0x4b,
                     0x9f, 0x7e, 0x06, 0xce, 0x3a, 0x5d);
static const ble_uuid128_t st_chr_metrics_uuid =
    BLE_UUID128_INIT(0xf1, 0xc7, 0xe9, 0xb3, 0xa8, 0xf6, 0x2c, 0x8d, 0x1a, 0x4b,
                     0x9f, 0x7e, 0x07, 0xce, 0x3a, 0x5d);
static const ble_uuid128_t st_chr_preshot_uuid =
    BLE_UUID128_INIT(0xf1, 0xc7, 0xe9, 0xb3, 0xa8, 0xf6, 0x2c, 0x8d, 0x1a, 0x4b,
                     0x9f, 0x7e, 0x08, 0xce, 0x3a, 0x5d);
static const ble_uuid128_t st_chr_shot_tuning_uuid =
    BLE_UUID128_INIT(0xf1, 0xc7, 0xe9, 0xb3, 0xa8, 0xf6, 0x2c, 0x8d, 0x1a, 0x4b,
                     0x9f, 0x7e, 0x09, 0xce, 0x3a, 0x5d);
static const ble_uuid128_t st_chr_stability_tuning_uuid =
    BLE_UUID128_INIT(0xf1, 0xc7, 0xe9, 0xb3, 0xa8, 0xf6, 0x2c, 0x8d, 0x1a, 0x4b,
                     0x9f, 0x7e, 0x0a, 0xce, 0x3a, 0x5d);

static uint16_t imu_chr_val_handle;
static uint16_t devinfo_chr_val_handle;
static uint16_t control_chr_val_handle;
static uint16_t health_chr_val_handle;
static uint16_t shot_chr_val_handle;
static uint16_t metrics_chr_val_handle;
static uint16_t preshot_chr_val_handle;
static uint16_t shot_tuning_chr_val_handle;
static uint16_t stability_tuning_chr_val_handle;

/* Written by the NimBLE host task (subscribe/disconnect events), read
 * by imu_task on every sample — atomic to avoid a cross-task race. */
static _Atomic uint16_t imu_subscriber_conn = BLE_HS_CONN_HANDLE_NONE;

/* Same cross-task contract for the shot-event characteristic. */
static _Atomic uint16_t shot_subscriber_conn = BLE_HS_CONN_HANDLE_NONE;

/* Same cross-task contract for the stability-metrics characteristic. */
static _Atomic uint16_t metrics_subscriber_conn = BLE_HS_CONN_HANDLE_NONE;

/* Same cross-task contract for the preshot-transfer characteristic. */
static _Atomic uint16_t preshot_subscriber_conn = BLE_HS_CONN_HANDLE_NONE;

/** Wraparound IMU sequence counter; mobile detects gaps via (cur-prev)%256. */
static uint8_t imu_seq;

/** Wraparound shot-event sequence counter; same gap-detection contract. */
static uint8_t shot_seq;

/** Wraparound metrics sequence counter; same gap-detection contract. */
static uint8_t metrics_seq;

/** Monotonic transfer counter; each transfer's header/sample/trailer
 *  frames share this value so mobile can disambiguate concurrent or
 *  re-requested transfers. */
static uint8_t preshot_transfer_seq;

/** Set in `gatt_svr_init`; the control handler signals it via
 *  `xTaskNotifyGive` on SHOT_TRANSFER_START. */
static TaskHandle_t s_preshot_transfer_task = NULL;

/* ---- Forward declarations -----------------------------------------
 *
 * Every static function defined later in this file. Listed up front so
 * any reorder of definitions stays compile-safe (the build error that
 * preceded this block was preshot_transfer_task being spawned before
 * its body was visible). Buffer-typed parameters use plain pointer
 * notation here; the definitions keep the explicit-size [N] form for
 * documentation. */

static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg);
static void encode_imu_frame(uint8_t *buf, const ble_imu_sample_t *s,
                             uint8_t seq);
static void encode_shot_tuning(uint8_t *buf, const shot_detect_config_t *cfg);
static bool decode_shot_tuning(const uint8_t *buf, size_t len,
                               shot_detect_config_t *out);
static void encode_stability_tuning(uint8_t *buf,
                                    const stability_config_t *cfg);
static bool decode_stability_tuning(const uint8_t *buf, size_t len,
                                    stability_config_t *out);
static void encode_shot_frame(uint8_t *buf, uint8_t event_type,
                              uint32_t onset_ms, uint16_t peak_centi,
                              uint8_t seq);
static void encode_metrics_frame(uint8_t *buf, const ble_metrics_t *m,
                                 uint8_t seq);
static void encode_preshot_header(uint8_t *buf, uint16_t total_samples,
                                  uint16_t onset_idx, uint32_t onset_ms,
                                  uint8_t seq);
static void encode_preshot_sample(uint8_t *buf, const preshot_sample_t *s,
                                  uint16_t chunk_idx);
static void encode_preshot_trailer(uint8_t *buf, uint16_t sent_count,
                                   uint8_t status, uint8_t seq);
static esp_err_t preshot_send_frame_once(const uint8_t *buf, size_t len);
static esp_err_t preshot_send_frame_blocking(const uint8_t *buf, size_t len);
static void preshot_send_trailer(uint16_t sent, uint8_t status, uint8_t seq);
static void preshot_run_transfer(uint8_t seq);
static void preshot_transfer_task(void *arg);

/** Control-characteristic opcodes — see `docs/ble/sensor-protocol.md`. */
#define BLE_CTRL_OP_SESSION_STOP 0x00
#define BLE_CTRL_OP_SESSION_START 0x01
#define BLE_CTRL_OP_SHOT_TRANSFER_START 0x10

/* Sizes of the hardware-tuning payloads; the encode/decode helpers
 * that own these layouts live further down in the file, but the GATT
 * handler above needs the buffer sizes — hoist them here. */
#define BLE_TUNING_SCHEMA_VERSION 0x01
#define BLE_SHOT_TUNING_BYTES 24
#define BLE_STABILITY_TUNING_BYTES 50

/* Session state — set by the control characteristic on the NimBLE host
 * task, read by ble_send_sample and imu_task; atomic for the cross-task
 * access. The IMU stream only flows while a session is active. */
static _Atomic bool session_active = false;

/**
 * @brief Device-info characteristic payload.
 *
 * Layout per `docs/ble/sensor-protocol.md`:
 *   - [0..3]   firmware_version_packed (BE, major.minor.patch.build)
 *   - [4..9]   hardware_id (BLE MAC, BE)
 *   - [10]     serial_number (UTF-8, null-terminated; empty for now)
 *
 * v1.4.0.0 adds the hardware-tuning characteristics 5d3ace09-… and
 * 5d3ace0a-… (RAM-only; mobile is the persistent source of truth and
 * re-pushes its saved config on each connect).
 * v1.3.0.0 added the preshot bulk-transfer characteristic (5d3ace08-…),
 * the SHOT_TRANSFER_START control opcode, EMA-RMS-based stability
 * metrics, proactive ATT MTU exchange, and TWDT registration of the
 * core tasks. The IMU wire-format v3 (25-B frame) is unchanged from
 * v1.2.
 */
static uint8_t s_devinfo[11] = {
    0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/** Sensor-health characteristic schema version, byte [0]. */
#define BLE_HEALTH_SCHEMA_VERSION 0x02
#define BLE_HEALTH_PAYLOAD_BYTES 8

/**
 * @brief Sensor-health characteristic state.
 *
 * Each field is individually atomic: the GATT read handler assembles
 * the 8-byte payload from them and the setters (imu_task, app_main)
 * update them, with no cross-task lock — a single-field torn read is
 * impossible and the payload carries no cross-field invariant.
 *
 * Wire layout (`docs/ble/sensor-protocol.md`):
 *   - [0]    schema version
 *   - [1]    flags: bit0 imu_online, bit1 dcd_ever_saved
 *   - [2]    live accel accuracy (0-3; 0xFF = no report yet)
 *   - [3]    live gyro accuracy  (0-3; 0xFF = no report yet)
 *   - [4..7] dcd_save_count (uint32 LE)
 */
static _Atomic uint8_t s_health_accel_acc = 0xFF;
static _Atomic uint8_t s_health_gyro_acc = 0xFF;
static _Atomic bool s_health_imu_online = false;
static _Atomic bool s_health_dcd_ever_saved = false;
static _Atomic uint32_t s_health_dcd_save_count = 0;

static uint8_t s_battery_last_notified = 0xFF; /**< 0xFF = "never sent". */
static int64_t s_battery_last_notify_us = 0;
#define BATTERY_NOTIFY_THROTTLE_US (5 * 1000 * 1000LL)

/** Compile-time gate for BAS-notify decision logs. Defaults off; override
 * with `-DBATTERY_DIAG_LOG=1` to mirror battery.c's bring-up tracing. */
#ifndef BATTERY_DIAG_LOG
#define BATTERY_DIAG_LOG 0
#endif

#if BATTERY_DIAG_LOG
#define BAT_DIAG_LOGI(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define BAT_DIAG_LOGI(...) ((void)0)
#endif

static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (attr_handle == devinfo_chr_val_handle) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
      int rc = os_mbuf_append(ctxt->om, s_devinfo, sizeof(s_devinfo));
      return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
  }

  if (attr_handle == control_chr_val_handle) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
      uint8_t opcode = 0;
      uint16_t copied = 0;
      int rc = ble_hs_mbuf_to_flat(ctxt->om, &opcode, sizeof(opcode), &copied);
      if (rc != 0 || copied < 1) {
        ESP_LOGW(TAG, "control write: empty payload");
        return 0;
      }
      switch (opcode) {
      case BLE_CTRL_OP_SESSION_START:
        atomic_store(&session_active, true);
        ESP_LOGI(TAG, "control: SESSION_START");
        break;
      case BLE_CTRL_OP_SESSION_STOP:
        atomic_store(&session_active, false);
        ESP_LOGI(TAG, "control: SESSION_STOP");
        break;
      case BLE_CTRL_OP_SHOT_TRANSFER_START:
        ESP_LOGI(TAG, "control: SHOT_TRANSFER_START");
        /* Hand off to the transfer task — the host task must not block
         * on PSRAM alloc / BLE-notify pacing inside this callback. */
        if (s_preshot_transfer_task != NULL) {
          xTaskNotifyGive(s_preshot_transfer_task);
        }
        break;
      default:
        ESP_LOGW(TAG, "control: unknown opcode 0x%02x (ignored)", opcode);
        break;
      }
      return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
  }

  if (attr_handle == health_chr_val_handle) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
      uint8_t buf[BLE_HEALTH_PAYLOAD_BYTES];
      buf[0] = BLE_HEALTH_SCHEMA_VERSION;
      buf[1] = (atomic_load(&s_health_imu_online) ? 0x01 : 0x00) |
               (atomic_load(&s_health_dcd_ever_saved) ? 0x02 : 0x00);
      buf[2] = atomic_load(&s_health_accel_acc);
      buf[3] = atomic_load(&s_health_gyro_acc);
      uint32_t count = atomic_load(&s_health_dcd_save_count);
      memcpy(&buf[4], &count, sizeof(count));
      int rc = os_mbuf_append(ctxt->om, buf, sizeof(buf));
      return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
  }

  if (attr_handle == shot_tuning_chr_val_handle) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
      shot_detect_config_t cfg;
      shot_detect_get_config(&cfg);
      uint8_t buf[BLE_SHOT_TUNING_BYTES];
      encode_shot_tuning(buf, &cfg);
      int rc = os_mbuf_append(ctxt->om, buf, sizeof(buf));
      return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
      uint8_t buf[BLE_SHOT_TUNING_BYTES];
      uint16_t copied = 0;
      int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &copied);
      if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
      }
      shot_detect_config_t next;
      if (!decode_shot_tuning(buf, copied, &next)) {
        ESP_LOGW(TAG, "shot tuning: bad payload (%u B)", (unsigned)copied);
        return BLE_ATT_ERR_INVALID_PDU;
      }
      shot_detect_set_config(&next);
      return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
  }

  if (attr_handle == stability_tuning_chr_val_handle) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
      stability_config_t cfg;
      stability_get_config(&cfg);
      uint8_t buf[BLE_STABILITY_TUNING_BYTES];
      encode_stability_tuning(buf, &cfg);
      int rc = os_mbuf_append(ctxt->om, buf, sizeof(buf));
      return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
      uint8_t buf[BLE_STABILITY_TUNING_BYTES];
      uint16_t copied = 0;
      int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &copied);
      if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
      }
      stability_config_t next;
      if (!decode_stability_tuning(buf, copied, &next)) {
        ESP_LOGW(TAG, "stability tuning: bad payload (%u B)", (unsigned)copied);
        return BLE_ATT_ERR_INVALID_PDU;
      }
      stability_set_config(&next);
      return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
  }

  return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &st_svc_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = &st_chr_imu_uuid.u,
                    .access_cb = gatt_access_cb,
                    .flags = BLE_GATT_CHR_F_NOTIFY,
                    .val_handle = &imu_chr_val_handle,
                },
                {
                    .uuid = &st_chr_devinfo_uuid.u,
                    .access_cb = gatt_access_cb,
                    .flags = BLE_GATT_CHR_F_READ,
                    .val_handle = &devinfo_chr_val_handle,
                },
                {
                    .uuid = &st_chr_control_uuid.u,
                    .access_cb = gatt_access_cb,
                    .flags = BLE_GATT_CHR_F_WRITE,
                    .val_handle = &control_chr_val_handle,
                },
                {
                    .uuid = &st_chr_health_uuid.u,
                    .access_cb = gatt_access_cb,
                    .flags = BLE_GATT_CHR_F_READ,
                    .val_handle = &health_chr_val_handle,
                },
                {
                    .uuid = &st_chr_shot_uuid.u,
                    .access_cb = gatt_access_cb,
                    .flags = BLE_GATT_CHR_F_NOTIFY,
                    .val_handle = &shot_chr_val_handle,
                },
                {
                    .uuid = &st_chr_metrics_uuid.u,
                    .access_cb = gatt_access_cb,
                    .flags = BLE_GATT_CHR_F_NOTIFY,
                    .val_handle = &metrics_chr_val_handle,
                },
                {
                    .uuid = &st_chr_preshot_uuid.u,
                    .access_cb = gatt_access_cb,
                    .flags = BLE_GATT_CHR_F_NOTIFY,
                    .val_handle = &preshot_chr_val_handle,
                },
                {
                    .uuid = &st_chr_shot_tuning_uuid.u,
                    .access_cb = gatt_access_cb,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                    .val_handle = &shot_tuning_chr_val_handle,
                },
                {
                    .uuid = &st_chr_stability_tuning_uuid.u,
                    .access_cb = gatt_access_cb,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                    .val_handle = &stability_tuning_chr_val_handle,
                },
                {0},
            },
    },
    {0},
};

int gatt_svr_init(void) {
  ble_svc_gap_init();
  ble_svc_gatt_init();

  ble_svc_bas_init();
  int rc = ble_svc_bas_battery_level_set(100);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_svc_bas_battery_level_set: rc=%d", rc);
    return rc;
  }

  rc = ble_gatts_count_cfg(gatt_svcs);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gatts_count_cfg: rc=%d", rc);
    return rc;
  }

  rc = ble_gatts_add_svcs(gatt_svcs);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gatts_add_svcs: rc=%d", rc);
    return rc;
  }

  /* Spawn the preshot-transfer task. The control handler signals it on
   * each SHOT_TRANSFER_START; idle cost is one task suspended on a
   * notify, 4 KB stack. */
  BaseType_t ok = xTaskCreate(preshot_transfer_task, "preshot_tx", 4096, NULL,
                              3, &s_preshot_transfer_task);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "preshot_transfer_task spawn failed");
    return -1;
  }

  ESP_LOGI(TAG, "GATT services registered (custom + BAS)");
  return 0;
}

uint16_t ble_gatt_imu_chr_handle(void) { return imu_chr_val_handle; }

void ble_gatt_set_imu_subscriber(uint16_t conn_handle) {
  atomic_store(&imu_subscriber_conn, conn_handle);
}

uint16_t ble_gatt_shot_chr_handle(void) { return shot_chr_val_handle; }

void ble_gatt_set_shot_subscriber(uint16_t conn_handle) {
  atomic_store(&shot_subscriber_conn, conn_handle);
}

uint16_t ble_gatt_metrics_chr_handle(void) { return metrics_chr_val_handle; }

void ble_gatt_set_metrics_subscriber(uint16_t conn_handle) {
  atomic_store(&metrics_subscriber_conn, conn_handle);
}

uint16_t ble_gatt_preshot_chr_handle(void) { return preshot_chr_val_handle; }

void ble_gatt_set_preshot_subscriber(uint16_t conn_handle) {
  atomic_store(&preshot_subscriber_conn, conn_handle);
}

bool ble_session_active(void) { return atomic_load(&session_active); }

void ble_gatt_clear_session(void) { atomic_store(&session_active, false); }

/** Saturating cast for Q-format conversions. */
static inline int16_t clamp_to_int16(float v) {
  if (v > 32767.0f)
    return 32767;
  if (v < -32768.0f)
    return -32768;
  return (int16_t)v;
}

#define BLE_IMU_FRAME_BYTES 25

/**
 * @brief Encode an IMU sample into the 25-byte v3 little-endian frame.
 *
 * Layout (per `docs/ble/sensor-protocol.md`):
 *   - [0..3]   uint32 timestamp_ms
 *   - [4..11]  int16 quat_x/y/z/w   (Q1.14 — divide by 16384)
 *   - [12..17] int16 gyro_x/y/z     (Q12.4 deg/s — divide by 16)
 *   - [18..23] int16 accel_x/y/z    (Q8.8 m/s² — divide by 256)
 *   - [24]     uint8 seq
 */
static void encode_imu_frame(uint8_t *buf, const ble_imu_sample_t *s,
                             uint8_t seq) {
  memcpy(&buf[0], &s->timestamp_ms, 4);

  int16_t qi = clamp_to_int16(s->quat_i * 16384.0f);
  int16_t qj = clamp_to_int16(s->quat_j * 16384.0f);
  int16_t qk = clamp_to_int16(s->quat_k * 16384.0f);
  int16_t qw = clamp_to_int16(s->quat_w * 16384.0f);
  memcpy(&buf[4], &qi, 2);
  memcpy(&buf[6], &qj, 2);
  memcpy(&buf[8], &qk, 2);
  memcpy(&buf[10], &qw, 2);

  /* rad/s → deg/s, then Q12.4. Range ±2048 dps saturates rather than wraps. */
  static const float RAD_TO_DEG_Q4 = 16.0f * 180.0f / (float)M_PI;
  int16_t gx = clamp_to_int16(s->gyro_rad_x * RAD_TO_DEG_Q4);
  int16_t gy = clamp_to_int16(s->gyro_rad_y * RAD_TO_DEG_Q4);
  int16_t gz = clamp_to_int16(s->gyro_rad_z * RAD_TO_DEG_Q4);
  memcpy(&buf[12], &gx, 2);
  memcpy(&buf[14], &gy, 2);
  memcpy(&buf[16], &gz, 2);

  /* BNO085 removes gravity on-chip. Q8.8 → ±128 m/s² range,
   * ~3.9 mg resolution; archery release peaks ~3-6 m/s². */
  int16_t ax = clamp_to_int16(s->lin_accel_x * 256.0f);
  int16_t ay = clamp_to_int16(s->lin_accel_y * 256.0f);
  int16_t az = clamp_to_int16(s->lin_accel_z * 256.0f);
  memcpy(&buf[18], &ax, 2);
  memcpy(&buf[20], &ay, 2);
  memcpy(&buf[22], &az, 2);

  buf[24] = seq;
}

esp_err_t ble_send_sample(const ble_imu_sample_t *sample) {
  if (sample == NULL)
    return ESP_ERR_INVALID_ARG;

  /* Session-gated: nothing goes on air until the central has both
   * subscribed and opened a session with SESSION_START. */
  if (!atomic_load(&session_active)) {
    return ESP_OK;
  }

  uint16_t conn = atomic_load(&imu_subscriber_conn);
  if (conn == BLE_HS_CONN_HANDLE_NONE) {
    return ESP_OK;
  }

  /* Bump seq on every emit attempt, including local drops — the mobile
   * parser uses gaps to count loss% regardless of who lost the frame. */
  uint8_t seq_now = imu_seq++;

  uint8_t buf[BLE_IMU_FRAME_BYTES];
  encode_imu_frame(buf, sample, seq_now);

  struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, sizeof(buf));
  if (om == NULL) {
    ESP_LOGW(TAG, "mbuf alloc failed; dropping sample");
    return ESP_FAIL;
  }

  /* ble_gatts_notify_custom takes ownership of the mbuf either way. */
  int rc = ble_gatts_notify_custom(conn, imu_chr_val_handle, om);
  if (rc != 0) {
    ESP_LOGD(TAG, "ble_gatts_notify_custom: rc=%d", rc);
    return ESP_FAIL;
  }
  return ESP_OK;
}

#define BLE_SHOT_FRAME_BYTES 9
#define BLE_SHOT_SCHEMA_VERSION 0x01

/**
 * @brief Encode a shot-detection event into the 9-byte little-endian frame.
 *
 * Layout (per `docs/ble/sensor-protocol.md`):
 *   - [0]    schema version
 *   - [1]    event type (0 provisional, 1 confirmed, 2 rejected)
 *   - [2..5] uint32 onset_ms
 *   - [6..7] uint16 peak envelope, centi-m/s²
 *   - [8]    uint8 seq
 */
static void encode_shot_frame(uint8_t *buf, uint8_t event_type,
                              uint32_t onset_ms, uint16_t peak_centi,
                              uint8_t seq) {
  buf[0] = BLE_SHOT_SCHEMA_VERSION;
  buf[1] = event_type;
  memcpy(&buf[2], &onset_ms, 4);
  memcpy(&buf[6], &peak_centi, 2);
  buf[8] = seq;
}

void ble_emit_shot_event(uint8_t event_type, uint32_t onset_ms,
                         uint16_t peak_centi) {
  /* Not session-gated — a shot event is rare and worth delivering to any
   * subscribed central regardless of IMU-stream session state. */
  uint16_t conn = atomic_load(&shot_subscriber_conn);
  if (conn == BLE_HS_CONN_HANDLE_NONE) {
    return;
  }

  uint8_t buf[BLE_SHOT_FRAME_BYTES];
  encode_shot_frame(buf, event_type, onset_ms, peak_centi, shot_seq++);

  struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, sizeof(buf));
  if (om == NULL) {
    ESP_LOGW(TAG, "shot event: mbuf alloc failed; dropping");
    return;
  }

  /* ble_gatts_notify_custom takes ownership of the mbuf either way. */
  int rc = ble_gatts_notify_custom(conn, shot_chr_val_handle, om);
  if (rc != 0) {
    ESP_LOGD(TAG, "shot event notify: rc=%d", rc);
  }
}

#define BLE_METRICS_FRAME_BYTES 26
#define BLE_METRICS_SCHEMA_VERSION 0x01

/**
 * @brief Encode a stability-metrics frame into the 26-byte LE frame.
 *
 * Layout (per `docs/ble/sensor-protocol.md`):
 *   - [0]      schema version
 *   - [1]      uint8 seq
 *   - [2..5]   uint32 timestamp_ms
 *   - [6..9]   float32 score
 *   - [10..13] float32 instability
 *   - [14..17] float32 rms_omega
 *   - [18..21] float32 rms_accel_lf
 *   - [22..25] float32 rms_disp
 */
static void encode_metrics_frame(uint8_t *buf, const ble_metrics_t *m,
                                 uint8_t seq) {
  buf[0] = BLE_METRICS_SCHEMA_VERSION;
  buf[1] = seq;
  memcpy(&buf[2], &m->timestamp_ms, 4);
  memcpy(&buf[6], &m->score, 4);
  memcpy(&buf[10], &m->instability, 4);
  memcpy(&buf[14], &m->rms_omega, 4);
  memcpy(&buf[18], &m->rms_accel_lf, 4);
  memcpy(&buf[22], &m->rms_disp, 4);
}

/* ============================================================
 * Hardware-tuning characteristics
 *
 * Two READ + WRITE custom characteristics carry the runtime config of
 * the firmware's DSP services. Layouts (`docs/ble/sensor-protocol.md`):
 *
 *   5d3ace09 — shot detection, 24 B:
 *     [0]     schema version (0x01)
 *     [1]     reserved (must be 0)
 *     [2..5]  float32 hold_gyro_threshold_rad_sq
 *     [6..9]  float32 accel_trigger_ratio
 *    [10..13] float32 accel_trigger_floor_ms2
 *    [14..17] float32 gyro_trigger_ratio
 *    [18..21] float32 gyro_trigger_floor_rad_s
 *    [22..23] uint16  min_hold_ms (LE, max 5000)
 *
 *   5d3ace0a — stability score, 50 B:
 *     [0]     schema version (0x01)
 *     [1]     reserved (must be 0)
 *     [2..5]  float32 w_omega
 *     [6..9]  float32 w_accel_lf
 *    [10..13] float32 w_disp
 *    [14..17] float32 map_k
 *    [18..21] float32 baseline_omega
 *    [22..25] float32 baseline_accel_lf
 *    [26..29] float32 baseline_disp
 *    [30..33] float32 alpha_omega
 *    [34..37] float32 alpha_accel_lf
 *    [38..41] float32 alpha_disp
 *    [42..45] float32 alpha_qref
 *    [46..49] float32 smoothing_alpha
 *
 * RAM-only: a reboot reverts to compile-time defaults. The mobile app
 * is the persistent source of truth and re-pushes its saved config on
 * each connect.
 *
 * The size and schema-version constants are hoisted near the top of
 * the file so the GATT handler can size its buffers without a forward
 * #define dance.
 * ============================================================ */

static void encode_shot_tuning(uint8_t *buf, const shot_detect_config_t *cfg) {
  buf[0] = BLE_TUNING_SCHEMA_VERSION;
  buf[1] = 0;
  memcpy(&buf[2], &cfg->hold_gyro_threshold_rad_sq, 4);
  memcpy(&buf[6], &cfg->accel_trigger_ratio, 4);
  memcpy(&buf[10], &cfg->accel_trigger_floor_ms2, 4);
  memcpy(&buf[14], &cfg->gyro_trigger_ratio, 4);
  memcpy(&buf[18], &cfg->gyro_trigger_floor_rad_s, 4);
  uint16_t min_hold =
      cfg->min_hold_ms > 0xFFFFu ? 0xFFFFu : (uint16_t)cfg->min_hold_ms;
  memcpy(&buf[22], &min_hold, 2);
}

static bool decode_shot_tuning(const uint8_t *buf, size_t len,
                               shot_detect_config_t *out) {
  if (len != BLE_SHOT_TUNING_BYTES) {
    return false;
  }
  if (buf[0] != BLE_TUNING_SCHEMA_VERSION) {
    return false;
  }
  memcpy(&out->hold_gyro_threshold_rad_sq, &buf[2], 4);
  memcpy(&out->accel_trigger_ratio, &buf[6], 4);
  memcpy(&out->accel_trigger_floor_ms2, &buf[10], 4);
  memcpy(&out->gyro_trigger_ratio, &buf[14], 4);
  memcpy(&out->gyro_trigger_floor_rad_s, &buf[18], 4);
  uint16_t min_hold = 0;
  memcpy(&min_hold, &buf[22], 2);
  out->min_hold_ms = min_hold;
  return true;
}

static void encode_stability_tuning(uint8_t *buf,
                                    const stability_config_t *cfg) {
  buf[0] = BLE_TUNING_SCHEMA_VERSION;
  buf[1] = 0;
  memcpy(&buf[2], &cfg->w_omega, 4);
  memcpy(&buf[6], &cfg->w_accel_lf, 4);
  memcpy(&buf[10], &cfg->w_disp, 4);
  memcpy(&buf[14], &cfg->map_k, 4);
  memcpy(&buf[18], &cfg->baseline_omega, 4);
  memcpy(&buf[22], &cfg->baseline_accel_lf, 4);
  memcpy(&buf[26], &cfg->baseline_disp, 4);
  memcpy(&buf[30], &cfg->alpha_omega, 4);
  memcpy(&buf[34], &cfg->alpha_accel_lf, 4);
  memcpy(&buf[38], &cfg->alpha_disp, 4);
  memcpy(&buf[42], &cfg->alpha_qref, 4);
  memcpy(&buf[46], &cfg->smoothing_alpha, 4);
}

static bool decode_stability_tuning(const uint8_t *buf, size_t len,
                                    stability_config_t *out) {
  if (len != BLE_STABILITY_TUNING_BYTES) {
    return false;
  }
  if (buf[0] != BLE_TUNING_SCHEMA_VERSION) {
    return false;
  }
  memcpy(&out->w_omega, &buf[2], 4);
  memcpy(&out->w_accel_lf, &buf[6], 4);
  memcpy(&out->w_disp, &buf[10], 4);
  memcpy(&out->map_k, &buf[14], 4);
  memcpy(&out->baseline_omega, &buf[18], 4);
  memcpy(&out->baseline_accel_lf, &buf[22], 4);
  memcpy(&out->baseline_disp, &buf[26], 4);
  memcpy(&out->alpha_omega, &buf[30], 4);
  memcpy(&out->alpha_accel_lf, &buf[34], 4);
  memcpy(&out->alpha_disp, &buf[38], 4);
  memcpy(&out->alpha_qref, &buf[42], 4);
  memcpy(&out->smoothing_alpha, &buf[46], 4);
  return true;
}

esp_err_t ble_send_metrics(const ble_metrics_t *m) {
  if (m == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  /* Session-gated like the IMU stream — an idle connection costs no
   * metrics traffic. */
  if (!atomic_load(&session_active)) {
    return ESP_OK;
  }
  uint16_t conn = atomic_load(&metrics_subscriber_conn);
  if (conn == BLE_HS_CONN_HANDLE_NONE) {
    return ESP_OK;
  }

  uint8_t buf[BLE_METRICS_FRAME_BYTES];
  encode_metrics_frame(buf, m, metrics_seq++);

  struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, sizeof(buf));
  if (om == NULL) {
    ESP_LOGW(TAG, "metrics: mbuf alloc failed; dropping");
    return ESP_FAIL;
  }

  /* ble_gatts_notify_custom takes ownership of the mbuf either way. */
  int rc = ble_gatts_notify_custom(conn, metrics_chr_val_handle, om);
  if (rc != 0) {
    ESP_LOGD(TAG, "metrics notify: rc=%d", rc);
    return ESP_FAIL;
  }
  return ESP_OK;
}

/* ============================================================
 * Preshot bulk transfer
 *
 * On SHOT_TRANSFER_START (control opcode 0x10), s_preshot_transfer_
 * task wakes, waits for preshot_window_ready, copies the
 * pre-release + post-release window from PSRAM, and streams it on
 * the preshot characteristic as one header frame, N sample frames,
 * and a trailer frame. Every frame in a single transfer carries the
 * same transfer_seq so a mobile receiver can pair them and discard
 * stragglers from a previous transfer.
 *
 * Pacing: each notify allocates an mbuf; the pool is small (~12),
 * so a tight loop would starve it. preshot_send_frame_blocking
 * retries with 5 ms sleeps, which auto-paces the burst to the
 * BLE link's actual rate (~200-300 notifies/s on a 15 ms
 * connection interval). The 4 s + 2 s window at 200 Hz = 1200
 * samples → ~5 s total transfer time at that rate.
 * ============================================================ */

#define BLE_PRESHOT_SCHEMA_VERSION 0x01
#define BLE_PRESHOT_TYPE_HEADER 0x00
#define BLE_PRESHOT_TYPE_SAMPLE 0x01
#define BLE_PRESHOT_TYPE_TRAILER 0x02

#define BLE_PRESHOT_STATUS_OK 0x00
#define BLE_PRESHOT_STATUS_TRUNCATED 0x01
#define BLE_PRESHOT_STATUS_ABORTED 0x02

#define BLE_PRESHOT_HEADER_BYTES 16
#define BLE_PRESHOT_SAMPLE_BYTES 28
#define BLE_PRESHOT_TRAILER_BYTES 8

/** Sample-buffer capacity. preshot enforces 4 s pre + 2 s post at
 *  200 Hz = 1200 samples; 1500 gives headroom against future window
 *  growth. PSRAM-allocated, so 1500 × 44 B ≈ 66 KB is fine. */
#define BLE_PRESHOT_MAX_SAMPLES 1500

/** Max 50-ms ticks to wait for the post-release window to fill after
 *  a transfer request (10 s ceiling; expected ≤2.5 s). */
#define BLE_PRESHOT_WINDOW_TIMEOUT_TICKS 200

/**
 * @brief Inter-frame pacing (ms) for the bulk stream while a session is live.
 *
 * A live session's 50 Hz IMU + metrics notifications share the NimBLE mbuf
 * pool; an unpaced transfer holds every freed block and starves them to 0 (a
 * bigger pool can't fix it — the transfer fills whatever exists). Yielding
 * ~6 ms per frame (> the ~3 ms link drain) caps the transfer to ~1-2 in-flight
 * blocks so imu_task always finds one. ~1200 frames -> ~7 s. Skipped when no
 * session is active (no live stream to protect -> full-rate transfer).
 */
#define BLE_PRESHOT_SESSION_PACING_MS 6

/**
 * @brief Encode the transfer-header frame.
 *
 * Layout (per `docs/ble/sensor-protocol.md`):
 *   - [0]      schema version
 *   - [1]      frame type (HEADER)
 *   - [2..3]   total_samples (uint16 LE)
 *   - [4..5]   onset_idx (uint16 LE) — sample index of the release
 *   - [6..9]   onset_ms (uint32 LE) — release timestamp
 *   - [10]     transfer_seq
 *   - [11..15] reserved
 */
static void encode_preshot_header(uint8_t *buf, uint16_t total_samples,
                                  uint16_t onset_idx, uint32_t onset_ms,
                                  uint8_t seq) {
  buf[0] = BLE_PRESHOT_SCHEMA_VERSION;
  buf[1] = BLE_PRESHOT_TYPE_HEADER;
  memcpy(&buf[2], &total_samples, 2);
  memcpy(&buf[4], &onset_idx, 2);
  memcpy(&buf[6], &onset_ms, 4);
  buf[10] = seq;
  memset(&buf[11], 0, 5);
}

/**
 * @brief Encode one preshot sample. Same Q-format scaling as the live
 *        IMU frame so mobile decoders can reuse one helper.
 *
 * Layout:
 *   - [0]      schema version
 *   - [1]      frame type (SAMPLE)
 *   - [2..3]   chunk_idx (uint16 LE) — sample's index in this transfer
 *   - [4..7]   uint32 t_ms
 *   - [8..13]  int16 gyro_x/y/z   (Q12.4 deg/s, divide by 16)
 *   - [14..19] int16 accel_x/y/z  (Q8.8 m/s², divide by 256)
 *   - [20..27] int16 quat_i/j/k/w (Q1.14, divide by 16384)
 */
static void encode_preshot_sample(uint8_t *buf, const preshot_sample_t *s,
                                  uint16_t chunk_idx) {
  buf[0] = BLE_PRESHOT_SCHEMA_VERSION;
  buf[1] = BLE_PRESHOT_TYPE_SAMPLE;
  memcpy(&buf[2], &chunk_idx, 2);
  memcpy(&buf[4], &s->t_ms, 4);

  static const float RAD_TO_DEG_Q4 = 16.0f * 180.0f / (float)M_PI;
  int16_t gx = clamp_to_int16(s->gyro[0] * RAD_TO_DEG_Q4);
  int16_t gy = clamp_to_int16(s->gyro[1] * RAD_TO_DEG_Q4);
  int16_t gz = clamp_to_int16(s->gyro[2] * RAD_TO_DEG_Q4);
  memcpy(&buf[8], &gx, 2);
  memcpy(&buf[10], &gy, 2);
  memcpy(&buf[12], &gz, 2);

  int16_t ax = clamp_to_int16(s->accel[0] * 256.0f);
  int16_t ay = clamp_to_int16(s->accel[1] * 256.0f);
  int16_t az = clamp_to_int16(s->accel[2] * 256.0f);
  memcpy(&buf[14], &ax, 2);
  memcpy(&buf[16], &ay, 2);
  memcpy(&buf[18], &az, 2);

  int16_t qi = clamp_to_int16(s->quat[0] * 16384.0f);
  int16_t qj = clamp_to_int16(s->quat[1] * 16384.0f);
  int16_t qk = clamp_to_int16(s->quat[2] * 16384.0f);
  int16_t qw = clamp_to_int16(s->quat[3] * 16384.0f);
  memcpy(&buf[20], &qi, 2);
  memcpy(&buf[22], &qj, 2);
  memcpy(&buf[24], &qk, 2);
  memcpy(&buf[26], &qw, 2);
}

/**
 * @brief Encode the transfer-trailer frame.
 *
 * Layout:
 *   - [0]    schema version
 *   - [1]    frame type (TRAILER)
 *   - [2..3] uint16 sent_count
 *   - [4]    status (0=OK, 1=truncated, 2=aborted)
 *   - [5]    transfer_seq
 *   - [6..7] reserved
 */
static void encode_preshot_trailer(uint8_t *buf, uint16_t sent_count,
                                   uint8_t status, uint8_t seq) {
  buf[0] = BLE_PRESHOT_SCHEMA_VERSION;
  buf[1] = BLE_PRESHOT_TYPE_TRAILER;
  memcpy(&buf[2], &sent_count, 2);
  buf[4] = status;
  buf[5] = seq;
  buf[6] = 0;
  buf[7] = 0;
}

/** @brief One non-blocking send attempt; ESP_FAIL on mbuf-pool starve. */
static esp_err_t preshot_send_frame_once(const uint8_t *buf, size_t len) {
  uint16_t conn = atomic_load(&preshot_subscriber_conn);
  if (conn == BLE_HS_CONN_HANDLE_NONE) {
    return ESP_FAIL;
  }
  struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
  if (om == NULL) {
    return ESP_FAIL;
  }
  /* ble_gatts_notify_custom takes ownership of the mbuf either way. */
  int rc = ble_gatts_notify_custom(conn, preshot_chr_val_handle, om);
  return rc == 0 ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Send a preshot frame, retrying on mbuf-pool exhaustion.
 *
 * Each retry yields 5 ms — long enough for NimBLE to drain a couple
 * of frames off the controller. ~40 retries × 5 ms = 200 ms ceiling;
 * if the link cannot pull frames within that, the subscriber is
 * effectively gone and we bail out.
 */
static esp_err_t preshot_send_frame_blocking(const uint8_t *buf, size_t len) {
  for (int attempt = 0; attempt < 40; attempt++) {
    if (preshot_send_frame_once(buf, len) == ESP_OK) {
      return ESP_OK;
    }
    /* If the subscriber dropped out (disconnect cleared the handle),
     * fail fast rather than spinning until the timeout. */
    if (atomic_load(&preshot_subscriber_conn) == BLE_HS_CONN_HANDLE_NONE) {
      return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  return ESP_FAIL;
}

/** @brief Send a trailer-only frame; used for abort paths. */
static void preshot_send_trailer(uint16_t sent, uint8_t status, uint8_t seq) {
  uint8_t tbuf[BLE_PRESHOT_TRAILER_BYTES];
  encode_preshot_trailer(tbuf, sent, status, seq);
  preshot_send_frame_blocking(tbuf, sizeof(tbuf));
}

/** @brief Run one full transfer — header + samples + trailer. */
static void preshot_run_transfer(uint8_t seq) {
  /* Wait for the post-release portion of the window to fill (the
   * mark may land before the 2 s post-release span has been written). */
  int wait = 0;
  while (!preshot_window_ready() && wait < BLE_PRESHOT_WINDOW_TIMEOUT_TICKS) {
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(50));
    wait++;
  }
  if (!preshot_window_ready()) {
    preshot_send_trailer(0, BLE_PRESHOT_STATUS_ABORTED, seq);
    ESP_LOGW(TAG, "preshot transfer: window not ready in time (seq=%u)", seq);
    return;
  }

  /* PSRAM-allocate the working snapshot; ring stays writable for
   * imu_task while we stream this copy out. */
  preshot_sample_t *win =
      heap_caps_malloc(BLE_PRESHOT_MAX_SAMPLES * sizeof(preshot_sample_t),
                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (win == NULL) {
    preshot_send_trailer(0, BLE_PRESHOT_STATUS_ABORTED, seq);
    ESP_LOGE(TAG, "preshot transfer: PSRAM alloc failed (seq=%u)", seq);
    return;
  }

  uint32_t onset_idx = 0;
  uint32_t n = preshot_copy(win, BLE_PRESHOT_MAX_SAMPLES, &onset_idx);
  if (n == 0) {
    heap_caps_free(win);
    preshot_send_trailer(0, BLE_PRESHOT_STATUS_ABORTED, seq);
    ESP_LOGW(TAG, "preshot transfer: empty window (seq=%u)", seq);
    return;
  }

  uint32_t onset_ms = preshot_marked_onset_ms();
  ESP_LOGI(TAG, "preshot transfer start: seq=%u samples=%lu onset_idx=%lu", seq,
           (unsigned long)n, (unsigned long)onset_idx);

  uint8_t hbuf[BLE_PRESHOT_HEADER_BYTES];
  encode_preshot_header(hbuf, (uint16_t)n, (uint16_t)onset_idx, onset_ms, seq);
  if (preshot_send_frame_blocking(hbuf, sizeof(hbuf)) != ESP_OK) {
    heap_caps_free(win);
    ESP_LOGW(TAG, "preshot transfer: header send failed (seq=%u)", seq);
    return;
  }

  uint32_t sent = 0;
  bool aborted = false;
  for (uint32_t i = 0; i < n; i++) {
    esp_task_wdt_reset();
    uint8_t sbuf[BLE_PRESHOT_SAMPLE_BYTES];
    encode_preshot_sample(sbuf, &win[i], (uint16_t)i);
    if (preshot_send_frame_blocking(sbuf, sizeof(sbuf)) != ESP_OK) {
      aborted = true;
      break;
    }
    sent++;

    /* Pace only while a live session shares the mbuf pool — see
     * BLE_PRESHOT_SESSION_PACING_MS. Without this the bulk stream starves the
     * 50 Hz IMU/metrics notifications to 0 regardless of pool size. */
    if (ble_session_active()) {
      vTaskDelay(pdMS_TO_TICKS(BLE_PRESHOT_SESSION_PACING_MS));
    }
  }
  heap_caps_free(win);

  uint8_t status = aborted ? BLE_PRESHOT_STATUS_ABORTED
                           : (sent < n ? BLE_PRESHOT_STATUS_TRUNCATED
                                       : BLE_PRESHOT_STATUS_OK);
  preshot_send_trailer((uint16_t)sent, status, seq);

  ESP_LOGI(TAG, "preshot transfer end: seq=%u sent=%lu/%lu status=%u", seq,
           (unsigned long)sent, (unsigned long)n, status);
}

/**
 * @brief Transfer-task loop. Idles on task notify; the control handler
 *        signals it on SHOT_TRANSFER_START.
 */
static void preshot_transfer_task(void *arg) {
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  while (1) {
    /* Wake at least every 2 s to feed the dog even with no transfer
     * pending. 2 s << 5 s TWDT default — comfortable margin. */
    uint32_t got = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
    esp_task_wdt_reset();
    if (got == 0) {
      continue;
    }
    uint8_t seq = preshot_transfer_seq++;
    preshot_run_transfer(seq);
  }
}

void ble_gatt_set_device_info_mac(const uint8_t mac_le[6]) {
  /* NimBLE returns LE; spec wants BE on the wire. */
  for (int i = 0; i < 6; i++) {
    s_devinfo[4 + i] = mac_le[5 - i];
  }
}

void ble_set_calibration_state(bool dcd_ever_saved, uint32_t dcd_save_count) {
  atomic_store(&s_health_dcd_ever_saved, dcd_ever_saved);
  atomic_store(&s_health_dcd_save_count, dcd_save_count);
}

void ble_set_imu_health(uint8_t accel_acc, uint8_t gyro_acc) {
  atomic_store(&s_health_accel_acc, accel_acc);
  atomic_store(&s_health_gyro_acc, gyro_acc);
  atomic_store(&s_health_imu_online, true);
}

esp_err_t ble_set_battery(uint8_t percent) {
  if (percent > 100)
    percent = 100;

  if (percent == s_battery_last_notified) {
    BAT_DIAG_LOGI("bat: %d%% unchanged, skip", percent);
    return ESP_OK;
  }

  int64_t now_us = esp_timer_get_time();
  bool first = (s_battery_last_notified == 0xFF);
  if (!first &&
      (now_us - s_battery_last_notify_us) < BATTERY_NOTIFY_THROTTLE_US) {
    BAT_DIAG_LOGI(
        "bat: %d%% (was %d%%) throttled, %llds until next notify", percent,
        s_battery_last_notified,
        (BATTERY_NOTIFY_THROTTLE_US - (now_us - s_battery_last_notify_us)) /
            1000000LL);
    return ESP_OK;
  }

  int rc = ble_svc_bas_battery_level_set(percent);
  if (rc != 0) {
    ESP_LOGW(TAG, "ble_svc_bas_battery_level_set: rc=%d", rc);
    return ESP_FAIL;
  }
  BAT_DIAG_LOGI("bat: notified %d%% (was %d%%, first=%s)", percent,
                s_battery_last_notified, first ? "yes" : "no");
  s_battery_last_notified = percent;
  s_battery_last_notify_us = now_us;
  return ESP_OK;
}
