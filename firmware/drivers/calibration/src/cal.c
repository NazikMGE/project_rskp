#include "cal/cal.h"

#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "cal";

#define CAL_NS "cal"
#define KEY_EVER_SAVED "dcd_saved"
#define KEY_SAVE_COUNT "dcd_count"

esp_err_t cal_init(void) {
  nvs_handle_t h;
  esp_err_t err = nvs_open(CAL_NS, NVS_READWRITE, &h);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_open: %s", esp_err_to_name(err));
    return err;
  }
  nvs_close(h);
  return ESP_OK;
}

esp_err_t cal_load(cal_state_t *out) {
  out->dcd_ever_saved = false;
  out->dcd_save_count = 0;

  nvs_handle_t h;
  esp_err_t err = nvs_open(CAL_NS, NVS_READONLY, &h);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    return ESP_OK;
  }
  if (err != ESP_OK) {
    return err;
  }

  uint8_t saved = 0;
  if (nvs_get_u8(h, KEY_EVER_SAVED, &saved) == ESP_OK) {
    out->dcd_ever_saved = (saved != 0);
  }
  uint32_t count = 0;
  if (nvs_get_u32(h, KEY_SAVE_COUNT, &count) == ESP_OK) {
    out->dcd_save_count = count;
  }

  nvs_close(h);
  return ESP_OK;
}

esp_err_t cal_store(const cal_state_t *state) {
  nvs_handle_t h;
  esp_err_t err = nvs_open(CAL_NS, NVS_READWRITE, &h);
  if (err != ESP_OK) {
    return err;
  }

  err = nvs_set_u8(h, KEY_EVER_SAVED, state->dcd_ever_saved ? 1 : 0);
  if (err == ESP_OK) {
    err = nvs_set_u32(h, KEY_SAVE_COUNT, state->dcd_save_count);
  }
  if (err == ESP_OK) {
    err = nvs_commit(h);
  }
  nvs_close(h);
  return err;
}
