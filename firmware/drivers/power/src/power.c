#include "power/power.h"

#include "board/pinmap.h"
#include "button/button.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "power";

void power_enter_deep_sleep(void) {
  ESP_LOGI(TAG, "deep sleep — waiting for button release");

  while (button_is_pressed()) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  vTaskDelay(pdMS_TO_TICKS(200));

  /* gpio_hold_en + gpio_deep_sleep_hold_en are both required: without
   * the deep-sleep variant the digital domain shutdown releases the
   * hold and RSTN floats, booting the BNO085 mid-sleep. */
  gpio_set_level(PIN_BNO_RST, 0);
  gpio_hold_en(PIN_BNO_RST);
  gpio_deep_sleep_hold_en();

  /* HINT is a BNO output. With RSTN asserted the chip tri-states it,
   * so we add an RTC pull-up to keep the ESP input pad out of the
   * crowbar region across deep sleep. */
  rtc_gpio_pullup_en(PIN_BNO_INT);
  rtc_gpio_pulldown_dis(PIN_BNO_INT);

  rtc_gpio_pullup_en(PIN_BTN);
  rtc_gpio_pulldown_dis(PIN_BTN);
  esp_sleep_enable_ext0_wakeup(PIN_BTN, 0);

  esp_deep_sleep_start();
}

bool power_woke_from_button(void) {
  uint64_t causes = esp_sleep_get_wakeup_causes();
  return (causes & (1ULL << ESP_SLEEP_WAKEUP_EXT0)) != 0;
}
