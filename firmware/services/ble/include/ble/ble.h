#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file ble.h
 * @brief NimBLE host wrapper — advertising, IMU notify, battery service.
 *
 * Full wire-format spec: `docs/ble/sensor-protocol.md`. Producer side
 * is `ble_gatt.c` (this component); consumer side is
 * `mobile/.../sensor_protocol.dart`. The IMU v3 frame in short:
 *   - Q1.14 quaternion (i, j, k, w)
 *   - Q12.4 gyro in deg/s
 *   - Q8.8 linear accel in m/s²
 *   - sequence counter
 */

/**
 * @brief Initialise the NimBLE host stack, register GATT services, and
 *        start advertising.
 *
 * NVS must already be initialised — NimBLE stores PHY calibration and
 * (future) bonding records there. Must be called once at app startup.
 */
esp_err_t ble_init(void);

/**
 * @brief One IMU sample as the firmware sees it before encoding.
 *
 * Quaternion fields are unit-vector floats in `[-1, +1]`. Gyro is in
 * rad/s (the encoder converts to deg/s for the wire). Linear accel is
 * in m/s² with gravity removed (BNO085 `linear_accelerometer` report).
 */
typedef struct {
  uint32_t timestamp_ms;
  float quat_i;
  float quat_j;
  float quat_k;
  float quat_w;
  float gyro_rad_x;
  float gyro_rad_y;
  float gyro_rad_z;
  float lin_accel_x;
  float lin_accel_y;
  float lin_accel_z;
} ble_imu_sample_t;

/**
 * @brief Encode @p sample as a v3 frame and notify on the IMU stream.
 *
 * No-op when no session is active or no central is subscribed (safe to
 * call every sample).
 *
 * @return ESP_OK on success, or when no session is active / nobody is
 *         subscribed.
 *         ESP_FAIL when the NimBLE notify queue / mbuf pool is
 *         exhausted — the current (newest) sample is dropped while
 *         frames already queued inside NimBLE are kept. Non-blocking;
 *         the caller should drop and continue.
 */
esp_err_t ble_send_sample(const ble_imu_sample_t *sample);

/**
 * @brief Whether an IMU session is currently active.
 *
 * A session is opened by the central writing SESSION_START to the
 * control characteristic and closed by SESSION_STOP or disconnect. The
 * IMU stream flows only while a session is active; imu_task also uses
 * this to freeze BNO085 dynamic calibration for the session's duration.
 */
bool ble_session_active(void);

/**
 * @brief Update the Battery Service level (0-100).
 *
 * The built-in BAS only emits a notify when the value changes; on top
 * of that this wrapper throttles to one notify every 5 s. Safe to
 * call every poll cycle.
 *
 * @return ESP_OK if the value was accepted, ESP_FAIL on internal BAS error.
 */
esp_err_t ble_set_battery(uint8_t percent);

/**
 * @brief Publish the persisted DCD-save state on the sensor-health
 *        characteristic.
 *
 * Called at startup with the NVS-loaded state, and again on each
 * periodic DCD save so a mobile read always sees the current count.
 */
void ble_set_calibration_state(bool dcd_ever_saved, uint32_t dcd_save_count);

/**
 * @brief Publish the live IMU accuracy on the sensor-health
 *        characteristic.
 *
 * Called by imu_task. Accuracy values use BNO085 semantics (0-3), or
 * 0xFF before the first report. The first call also marks the IMU
 * online — until then the characteristic reports it offline.
 */
void ble_set_imu_health(uint8_t accel_acc, uint8_t gyro_acc);

/**
 * @brief Notify a shot-detection event on the shot characteristic.
 *
 * Called by imu_task whenever the shot detector produces an event. A
 * no-op when no central is subscribed (safe to call unconditionally).
 * Not session-gated — a shot event is rare and worth delivering even
 * without an open IMU-stream session.
 *
 * @param event_type 0 provisional, 1 confirmed, 2 rejected.
 * @param onset_ms    Release-onset timestamp, on the same monotonic ms
 *                    clock as the IMU frame's `timestamp_ms`.
 * @param peak_centi  Peak release envelope in centi-m/s² (m/s² × 100);
 *                    0 for a provisional event.
 */
void ble_emit_shot_event(uint8_t event_type, uint32_t onset_ms,
                         uint16_t peak_centi);

/**
 * @brief One stability-metrics frame as the firmware computes it.
 *
 * The 0..100 hold-stability score plus the raw component RMS values and
 * the composite instability index — the latter four let the app's
 * tuning UI see what the score is built from.
 */
typedef struct {
  uint32_t timestamp_ms;
  float score;        /**< smoothed hold-stability score, 0..100 */
  float instability;  /**< composite instability index I */
  float rms_omega;    /**< rate-term RMS (rad/s) */
  float rms_accel_lf; /**< sway-term RMS (m/s²) */
  float rms_disp;     /**< drift-term RMS (rad) */
} ble_metrics_t;

/**
 * @brief Notify @p m on the stability-metrics characteristic.
 *
 * Called by imu_task at 50 Hz (once per IMU frame). Session-gated like
 * the IMU stream — a no-op when no session is active or no central is
 * subscribed.
 *
 * @return ESP_OK on success or when nothing is sent; ESP_FAIL when the
 *         NimBLE mbuf pool is exhausted (the frame is dropped).
 */
esp_err_t ble_send_metrics(const ble_metrics_t *m);

/**
 * @brief Hardware-tuning characteristics.
 *
 * Two READ + WRITE custom characteristics carry the runtime config of
 * the firmware's DSP services so the mobile tuning UI can adjust them
 * without a reflash. Layouts and UUIDs live in
 * `docs/ble/sensor-protocol.md`. RAM-only — a reboot reverts to
 * compile-time defaults; the mobile app is the persistent source of
 * truth and re-pushes its saved config on each connect.
 *
 *   - `5d3ace09-…` shot-detect tuning, 24 B
 *   - `5d3ace0a-…` stability-score tuning, 50 B
 *
 * The GATT handlers translate between the binary wire form and
 * @ref shot_detect_set_config / @ref stability_set_config. Reads
 * always reflect the current effective config, including any clamps
 * applied by the setters.
 */

#ifdef __cplusplus
}
#endif
