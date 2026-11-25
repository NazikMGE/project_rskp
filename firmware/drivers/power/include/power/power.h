#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file power.h
 * @brief ESP32 deep-sleep entry and wake-cause detection.
 */

/**
 * @brief Enter ESP32 deep sleep with EXT0 wake on the user button.
 *
 * Spin-waits for button release, drives the BNO085 `RSTN` line low and
 * holds that level across the sleep period, then arms EXT0 wake on
 * `PIN_BTN` and stops the CPU. Never returns; on wake the chip resets
 * and re-runs `app_main`.
 *
 * @note The caller owns BNO085 shutdown ordering (disable reports,
 *       persist DCD if required, request software sleep). This
 *       function only handles the host-side teardown.
 */
void power_enter_deep_sleep(void);

/**
 * @brief True if the most recent reset was an EXT0 wake from deep
 *        sleep (user pressed the button).
 *
 * Returns false on cold boot or any non-EXT0 reset cause.
 */
bool power_woke_from_button(void);

#ifdef __cplusplus
}
#endif
