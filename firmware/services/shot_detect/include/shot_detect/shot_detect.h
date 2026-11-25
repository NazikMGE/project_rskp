#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file shot_detect.h
 * @brief Archery release (shot) detector — variant V4.
 *
 * A dual-channel transient detector on the 200 Hz IMU stream. A release
 * is caught on either an accel hand-shock (a 10 Hz-highpass envelope) or
 * a gyro bow-rotation transient — whichever crosses its adaptive
 * threshold first — gated by a sustained aiming hold and confirmed by a
 * post-onset ring-down check. O(1) per sample — imu_task ticks it
 * synchronously, so it carries no task or queue of its own.
 *
 * Two-stage events: a @ref SHOT_EVT_PROVISIONAL is produced within
 * ~20 ms of the onset, then ~230 ms later a @ref SHOT_EVT_CONFIRMED or
 * @ref SHOT_EVT_REJECTED for the same shot — both carry the same
 * @ref shot_event_t::onset_ms so a consumer can pair them.
 */

/** @brief Lifecycle stage a @ref shot_event_t reports. */
typedef enum {
  SHOT_EVT_PROVISIONAL = 0, /**< Trigger fired — fast, low-latency marker. */
  SHOT_EVT_CONFIRMED = 1,   /**< Ring-down validated — a real shot. */
  SHOT_EVT_REJECTED = 2,    /**< No ring-down — the trigger was not a shot. */
} shot_event_type_t;

/**
 * @brief One detector event.
 *
 * The provisional event and the confirmed/rejected event for the same
 * shot share @ref onset_ms; a consumer pairs them on that field.
 */
typedef struct {
  shot_event_type_t type;
  uint32_t onset_ms; /**< Release-onset timestamp (esp_timer ms), group-delay
                          compensated; same clock as the IMU frame. */
  float peak_mps2;   /**< Peak release envelope in the post-onset window;
                          0 for a provisional event. */
} shot_event_t;

/**
 * @brief Runtime-tunable detector parameters.
 *
 * Six knobs the hardware-tuning UI can push over BLE; everything else
 * (sample rate, filter coefficients, debounce counts, ring-down
 * weights) stays fixed in firmware. Defaults match the original
 * `SD_*` `#define`s in `shot_detect.c` so the shipped behaviour is
 * preserved when the central never pushes a config.
 */
typedef struct {
  /** `(rad/s)^2` — gyro mean-square hold gate. Default 0.030. */
  float hold_gyro_threshold_rad_sq;
  /** Accel trigger threshold = max(floor, baseline * K). Default 6.0. */
  float accel_trigger_ratio;
  /** `m/s^2` — accel envelope floor. Default 1.5. */
  float accel_trigger_floor_ms2;
  /** Gyro trigger threshold = max(floor, gyro_base * K). Default 8.0. */
  float gyro_trigger_ratio;
  /** `rad/s` — gyro envelope floor. Default 0.5. */
  float gyro_trigger_floor_rad_s;
  /** Minimum hold time (ms) before a release may be triggered. Default 500. */
  uint32_t min_hold_ms;
} shot_detect_config_t;

/**
 * @brief Reset the detector to its idle state.
 *
 * Call once before the first @ref shot_detect_tick. The runtime config
 * is initialised to compile-time defaults; @ref shot_detect_set_config
 * can override them later (e.g. from a BLE tuning write).
 */
void shot_detect_init(void);

/** @brief Copy the current tunable config into @p cfg. */
void shot_detect_get_config(shot_detect_config_t *cfg);

/**
 * @brief Replace the tunable config (e.g. from the tuning UI).
 *
 * Applied atomically; values are clamped to safe ranges so a bad
 * BLE write cannot stall the detector. NULL is a no-op.
 */
void shot_detect_set_config(const shot_detect_config_t *cfg);

/**
 * @brief Feed one 200 Hz IMU sample through the detector.
 *
 * Call once per linear-accel sample from imu_task. O(1), no allocation,
 * never blocks. At most one event is produced per call.
 *
 * @param t_us       Sample timestamp, esp_timer microseconds.
 * @param gyro_rad   Calibrated gyro [x,y,z] in rad/s — the hold gate and
 *                   the bow-rotation trigger channel.
 * @param lin_accel  Linear accel [x,y,z] in m/s² (gravity removed).
 * @param out_event  Non-NULL; filled only when the return value is true.
 * @return true when @p out_event holds a new event this tick.
 */
bool shot_detect_tick(uint64_t t_us, const float gyro_rad[3],
                      const float lin_accel[3], shot_event_t *out_event);

#ifdef __cplusplus
}
#endif
