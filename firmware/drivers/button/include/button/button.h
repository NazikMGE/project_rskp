#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file button.h
 * @brief Single tactile button on `PIN_BTN`, active-low.
 */

/** @brief Configure `PIN_BTN` as input with internal pull-up. */
esp_err_t button_init(void);

/** @brief True while the user is holding the button down. */
bool button_is_pressed(void);

#ifdef __cplusplus
}
#endif
