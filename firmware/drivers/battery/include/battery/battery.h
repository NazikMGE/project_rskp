#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file battery.h
 * @brief 1S LiPo monitor on an external 220k/220k divider feeding D5.
 *
 * `battery_init()` spawns a 1 Hz background sampler that updates a
 * pair of cached, smoothed values. Callers read those via the getters
 * below — the ADC is never touched on the caller's thread.
 */

/**
 * @brief Initialise ADC + calibration and start the sampling task.
 *
 * Safe to call once at boot.
 */
esp_err_t battery_init(void);

/**
 * @brief Latest smoothed terminal voltage, in volts.
 *
 * Stable to ~5 mV under steady load; tracks slow drift.
 */
float battery_get_voltage(void);

/**
 * @brief Latest smoothed charge percentage in [0, 100].
 *
 * Mapped through a non-linear 1S-LiPo discharge curve; updates ~1 Hz.
 */
int battery_get_percent(void);

#ifdef __cplusplus
}
#endif
