#include "button/button.h"
#include "board/pinmap.h"

#include "driver/gpio.h"

esp_err_t button_init(void) {
  gpio_config_t cfg = {
      .pin_bit_mask = (1ULL << PIN_BTN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  return gpio_config(&cfg);
}

bool button_is_pressed(void) { return gpio_get_level(PIN_BTN) == 0; }
