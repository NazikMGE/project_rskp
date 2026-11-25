#include "supervisor/supervisor.h"

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "supervisor";

/**
 * @brief RTC-memory magic marking @ref s_fault_reboots as valid.
 *
 * RTC slow memory is retained across `esp_restart()` and deep sleep but
 * holds garbage after a cold power-on; the magic tells the two apart.
 */
#define SUPERVISOR_RTC_MAGIC 0x5B1A7C01U

/** Console-flush slack so the final log line reaches the UART. */
#define SUPERVISOR_FLUSH_MS 50

RTC_NOINIT_ATTR static uint32_t s_rtc_magic;
RTC_NOINIT_ATTR static uint32_t s_fault_reboots;

/** @brief True for reset reasons that signal a firmware fault. */
static bool reset_reason_is_fault(esp_reset_reason_t reason) {
  switch (reason) {
  case ESP_RST_PANIC:
  case ESP_RST_INT_WDT:
  case ESP_RST_TASK_WDT:
  case ESP_RST_WDT:
    return true;
  default:
    return false;
  }
}

void supervisor_init(void) {
  esp_reset_reason_t reason = esp_reset_reason();
  bool carried_over = (s_rtc_magic == SUPERVISOR_RTC_MAGIC);

  if (!carried_over || reason == ESP_RST_POWERON) {
    /* Cold boot, or RTC memory was not retained — full budget. */
    s_rtc_magic = SUPERVISOR_RTC_MAGIC;
    s_fault_reboots = 0;
  } else if (reset_reason_is_fault(reason)) {
    /* A panic or watchdog reset that no supervisor_fault_reboot()
     * accounted for — charge it so a crash loop is bounded too. */
    s_fault_reboots++;
  }

  if (s_fault_reboots > SUPERVISOR_MAX_FAULT_REBOOTS) {
    s_fault_reboots = SUPERVISOR_MAX_FAULT_REBOOTS;
  }

  ESP_LOGI(TAG, "init: reset_reason=%d fault_reboots=%u/%u", (int)reason,
           (unsigned)s_fault_reboots, (unsigned)SUPERVISOR_MAX_FAULT_REBOOTS);
  if (supervisor_reboot_budget_exhausted()) {
    ESP_LOGE(TAG, "fault-reboot budget spent — device runs degraded");
  }
}

bool supervisor_reboot_budget_exhausted(void) {
  return s_fault_reboots >= SUPERVISOR_MAX_FAULT_REBOOTS;
}

void supervisor_mark_healthy(void) {
  if (s_fault_reboots != 0) {
    ESP_LOGI(TAG, "healthy run confirmed — clearing fault counter (was %u)",
             (unsigned)s_fault_reboots);
    s_fault_reboots = 0;
  }
}

void supervisor_fault_reboot(const char *reason) {
  const char *r = (reason != NULL) ? reason : "(unspecified)";

  if (supervisor_reboot_budget_exhausted()) {
    ESP_LOGE(TAG, "FAULT: %s — reboot budget spent, staying up degraded", r);
    return;
  }

  s_fault_reboots++;
  ESP_LOGE(TAG, "FAULT: %s — reboot %u/%u", r, (unsigned)s_fault_reboots,
           (unsigned)SUPERVISOR_MAX_FAULT_REBOOTS);
  vTaskDelay(pdMS_TO_TICKS(SUPERVISOR_FLUSH_MS));
  esp_restart();
}
