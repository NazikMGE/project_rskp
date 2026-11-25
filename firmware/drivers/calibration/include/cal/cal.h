#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file cal.h
 * @brief Persisted bookkeeping for BNO085 dynamic-calibration saves.
 *
 * The BNO085 holds Dynamic Calibration Data (DCD) in its internal FRS
 * flash. Each save costs one flash write, and writing a partial
 * calibration overwrites a previously-better one. This module persists
 * a small record on the ESP32 NVS — the save count and accuracy at the
 * last save — written by the periodic-save path and read out over the
 * BLE calibration characteristic.
 */

/** State persisted across reboots. */
typedef struct {
  /** True after the first successful DCD save. */
  bool dcd_ever_saved;
  /** Cumulative successful save count. */
  uint32_t dcd_save_count;
} cal_state_t;

/**
 * @brief Verify the NVS namespace is reachable.
 *
 * Call once at boot, after `nvs_flash_init()`.
 */
esp_err_t cal_init(void);

/**
 * @brief Load the persisted state.
 *
 * Missing keys take their defaults — `dcd_ever_saved = false`,
 * `dcd_save_count = 0`.
 */
esp_err_t cal_load(cal_state_t *out);

/** @brief Persist @p state and commit. */
esp_err_t cal_store(const cal_state_t *state);

#ifdef __cplusplus
}
#endif
