#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @file imu_profile.h
 * @brief BNO085 report-rate profiles selected by training-session state.
 *
 * A "profile" is the set of report periods the IMU streams in a given power
 * mode. The device runs the lean idle profile whenever no session is active
 * (standby / connected-idle) and the full session profile only during a
 * training session. A period of 0 means that report is disabled.
 *
 * Pure data + selector, no ESP/driver dependencies, so it is unit-testable on
 * the host (see firmware/test/host/imu_profile_test.c).
 */

/** Per-report periods in microseconds; 0 = report disabled in this profile. */
typedef struct {
  uint32_t rv_game_us;  /**< Game Rotation Vector (sensor-fusion engine). */
  uint32_t cal_gyro_us; /**< Calibrated gyroscope. */
  uint32_t
      linear_accel_us;   /**< Linear acceleration (gravity removed; fusion). */
  uint32_t accel_us;     /**< Calibrated accelerometer (no fusion). */
  uint32_t stability_us; /**< Stability classifier (motion -> auto-sleep). */
} imu_report_profile_t;

/**
 * @brief Select the report profile for the current session state.
 *
 * Session: full streaming — 50 Hz Game-RV (decimation clock) + dual 200 Hz
 * gyro/linear-accel (the DSP and BLE-stream inputs) + 1 Hz stability.
 * Idle: lean keep-alive — 10 Hz cal-gyro + 10 Hz linear-accel hold the fused
 * accel accuracy the health card shows and keep ACCEL|GYRO dynamic calibration
 * refining; 1 Hz stability drives auto-sleep. The 50 Hz Game-RV frame and the
 * 200 Hz DSP rates are off, but fusion still runs at the lean 10 Hz — a
 * deliberate power trade for a consistent accuracy reading and a reachable
 * DCD-save gate.
 */
static inline imu_report_profile_t imu_profile_for(bool session_active) {
  if (session_active) {
    imu_report_profile_t p = {
        .rv_game_us = 20000,     /* 50 Hz  */
        .cal_gyro_us = 5000,     /* 200 Hz */
        .linear_accel_us = 5000, /* 200 Hz */
        .accel_us = 0,           /* off    */
        .stability_us = 1000000, /* 1 Hz   */
    };
    return p;
  }
  imu_report_profile_t p = {
      .rv_game_us = 0,           /* off: no 50 Hz frame in idle */
      .cal_gyro_us = 100000,     /* 10 Hz */
      .linear_accel_us = 100000, /* 10 Hz fused accel: accuracy + cal */
      .accel_us = 0,             /* off: linear accel replaces it */
      .stability_us = 1000000,   /* 1 Hz */
  };
  return p;
}
