#include "ble/ble.h"
#include "ble_priv.h"

#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "ble";

/**
 * @brief Custom service UUID `5d3ace00-7e9f-4b1a-8d2c-f6a8b3e9c7f1`.
 *
 * `BLE_UUID128_INIT` takes raw bytes in little-endian — rightmost UUID
 * byte goes to slot [0], leftmost to slot [15].
 */
static const ble_uuid128_t st_service_uuid =
    BLE_UUID128_INIT(0xf1, 0xc7, 0xe9, 0xb3, 0xa8, 0xf6, 0x2c, 0x8d, 0x1a, 0x4b,
                     0x9f, 0x7e, 0x00, 0xce, 0x3a, 0x5d);

/** Adv interval, 100 ms (NimBLE units are 0.625 ms per LSB). */
#define ADV_INTERVAL_MS 100
#define ADV_INTERVAL_UNITS ((ADV_INTERVAL_MS * 1000) / 625)

/** Minimum ATT MTU for the 25-B v3 IMU frame: 25 B payload + 3-B ATT
 *  notification header. Below this NimBLE silently truncates the
 *  notification — see `docs/ble/sensor-protocol.md`. */
#define IMU_FRAME_MIN_MTU 28

/** Preferred ATT MTU advertised to centrals. 247 is the ESP32-S3
 *  NimBLE practical ceiling: large enough for the 25-B IMU frame, the
 *  26-B metrics frame, and the upcoming preshot-transfer frames in a
 *  single notify, without straining the controller's L2CAP buffers.
 *  Centrals that ignore the preferred MTU stay at the 23-B default;
 *  the subscribe-time warning still catches that. */
#define BLE_PREFERRED_MTU 247

/** `ST-XXXX\0` — 8 bytes. */
static char s_device_name[8];

/** Address type chosen by `ble_hs_id_infer_auto` at sync. */
static uint8_t s_own_addr_type;

/* ---- Forward declarations ----------------------------------------- */

static void start_advertising(void);
static void request_fast_conn_params(uint16_t conn_handle);
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);
static void on_sync(void);
static void on_reset(int reason);
static void host_task(void *arg);

/**
 * @brief Start a connectable undirected advertisement.
 *
 * Primary packet carries the device name; scan response carries the
 * 128-bit service UUID. Splitting fits both inside the 31-B adv
 * payload limit while still surfacing a quick name in scan lists.
 */
static void start_advertising(void) {
  struct ble_hs_adv_fields adv_fields = {0};
  adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  adv_fields.name = (uint8_t *)s_device_name;
  adv_fields.name_len = strlen(s_device_name);
  adv_fields.name_is_complete = 1;

  int rc = ble_gap_adv_set_fields(&adv_fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_adv_set_fields: rc=%d", rc);
    return;
  }

  struct ble_hs_adv_fields rsp_fields = {0};
  rsp_fields.uuids128 = (ble_uuid128_t *)&st_service_uuid;
  rsp_fields.num_uuids128 = 1;
  rsp_fields.uuids128_is_complete = 1;

  rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields: rc=%d", rc);
    return;
  }

  struct ble_gap_adv_params adv_params = {
      .conn_mode = BLE_GAP_CONN_MODE_UND,
      .disc_mode = BLE_GAP_DISC_MODE_GEN,
      .itvl_min = ADV_INTERVAL_UNITS,
      .itvl_max = ADV_INTERVAL_UNITS,
  };
  rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                         ble_gap_event_cb, NULL);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_adv_start: rc=%d", rc);
    return;
  }

  ESP_LOGI(TAG, "advertising as '%s' @ %d ms interval", s_device_name,
           ADV_INTERVAL_MS);
}

/**
 * @brief Ask the central for a fast connection interval.
 *
 * The 50 Hz IMU stream wants a short interval; left to the central it
 * often lands at 30-50 ms, adding latency and backing up the notify
 * queue. Request 15-30 ms (15 ms is the iOS-compatible floor). The
 * central may reject — this is best-effort, so only the rc is logged.
 */
static void request_fast_conn_params(uint16_t conn_handle) {
  struct ble_gap_upd_params params = {
      .itvl_min = 12, /* 15 ms (1.25 ms units) */
      .itvl_max = 24, /* 30 ms */
      .latency = 0,
      .supervision_timeout = 400, /* 4 s (10 ms units) */
      .min_ce_len = 0,
      .max_ce_len = 0,
  };
  int rc = ble_gap_update_params(conn_handle, &params);
  if (rc != 0) {
    ESP_LOGW(TAG, "ble_gap_update_params: rc=%d", rc);
  }
}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    ESP_LOGI(TAG, "connect: status=%d handle=%d", event->connect.status,
             event->connect.conn_handle);
    if (event->connect.status == 0) {
      request_fast_conn_params(event->connect.conn_handle);
      /* Proactively initiate the MTU exchange — relying on the central
       * to ask first is fragile; a peer that forgets leaves us truncating
       * 25-B frames on the 23-B default. Best-effort, log on failure. */
      int rc = ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);
      if (rc != 0) {
        ESP_LOGW(TAG, "ble_gattc_exchange_mtu: rc=%d", rc);
      }
    } else {
      start_advertising();
    }
    break;
  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI(TAG, "disconnect: reason=%d", event->disconnect.reason);
    /* Drop every subscriber tied to this conn; otherwise a notify
     * targets a dead handle and accumulates errors. End the session
     * too — it must not outlive the connection. */
    ble_gatt_set_imu_subscriber(BLE_HS_CONN_HANDLE_NONE);
    ble_gatt_set_shot_subscriber(BLE_HS_CONN_HANDLE_NONE);
    ble_gatt_set_metrics_subscriber(BLE_HS_CONN_HANDLE_NONE);
    ble_gatt_set_preshot_subscriber(BLE_HS_CONN_HANDLE_NONE);
    ble_gatt_clear_session();
    start_advertising();
    break;
  case BLE_GAP_EVENT_ADV_COMPLETE:
    ESP_LOGI(TAG, "adv complete; reason=%d", event->adv_complete.reason);
    start_advertising();
    break;
  case BLE_GAP_EVENT_SUBSCRIBE:
    ESP_LOGI(TAG, "subscribe: handle=%d cur_notify=%d",
             event->subscribe.attr_handle, event->subscribe.cur_notify);
    if (event->subscribe.attr_handle == ble_gatt_imu_chr_handle()) {
      if (event->subscribe.cur_notify) {
        uint16_t mtu = ble_att_mtu(event->subscribe.conn_handle);
        if (mtu < IMU_FRAME_MIN_MTU) {
          ESP_LOGW(TAG,
                   "IMU subscribed with ATT MTU=%u < %d — 25-B frames will "
                   "be truncated; central must negotiate a larger MTU",
                   mtu, IMU_FRAME_MIN_MTU);
        }
      }
      ble_gatt_set_imu_subscriber(event->subscribe.cur_notify
                                      ? event->subscribe.conn_handle
                                      : BLE_HS_CONN_HANDLE_NONE);
    }
    if (event->subscribe.attr_handle == ble_gatt_shot_chr_handle()) {
      ble_gatt_set_shot_subscriber(event->subscribe.cur_notify
                                       ? event->subscribe.conn_handle
                                       : BLE_HS_CONN_HANDLE_NONE);
    }
    if (event->subscribe.attr_handle == ble_gatt_metrics_chr_handle()) {
      ble_gatt_set_metrics_subscriber(event->subscribe.cur_notify
                                          ? event->subscribe.conn_handle
                                          : BLE_HS_CONN_HANDLE_NONE);
    }
    if (event->subscribe.attr_handle == ble_gatt_preshot_chr_handle()) {
      ble_gatt_set_preshot_subscriber(event->subscribe.cur_notify
                                          ? event->subscribe.conn_handle
                                          : BLE_HS_CONN_HANDLE_NONE);
    }
    break;
  case BLE_GAP_EVENT_MTU:
    ESP_LOGI(TAG, "mtu: handle=%d mtu=%d", event->mtu.conn_handle,
             event->mtu.value);
    break;
  default:
    break;
  }
  return 0;
}

/**
 * @brief NimBLE host-sync callback. Derives the adv name from the BLE
 *        MAC and starts advertising.
 */
static void on_sync(void) {
  int rc = ble_hs_util_ensure_addr(0);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_hs_util_ensure_addr: rc=%d", rc);
    return;
  }

  rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_hs_id_infer_auto: rc=%d", rc);
    return;
  }

  uint8_t addr[6];
  rc = ble_hs_id_copy_addr(s_own_addr_type, addr, NULL);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_hs_id_copy_addr: rc=%d", rc);
    return;
  }

  /* `ST-XXXX` from the high two bytes of the MAC. NimBLE returns the
   * address little-endian, so addr[1]:addr[0] are the printed-MSB bytes. */
  snprintf(s_device_name, sizeof(s_device_name), "ST-%02X%02X", addr[1],
           addr[0]);

  ble_gatt_set_device_info_mac(addr);

  rc = ble_svc_gap_device_name_set(s_device_name);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_svc_gap_device_name_set: rc=%d", rc);
    return;
  }

  ESP_LOGI(TAG, "addr=%02x:%02x:%02x:%02x:%02x:%02x type=%u", addr[5], addr[4],
           addr[3], addr[2], addr[1], addr[0], s_own_addr_type);

  start_advertising();
}

static void on_reset(int reason) {
  ESP_LOGW(TAG, "host reset; reason=%d", reason);
}

static void host_task(void *arg) {
  ESP_LOGI(TAG, "nimble host task running");
  nimble_port_run();
  nimble_port_freertos_deinit();
}

esp_err_t ble_init(void) {
  /* NVS — which NimBLE uses for PHY calibration and (future) bonding —
   * is brought up by the caller before ble_init; see app_main. */
  esp_err_t err = nimble_port_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nimble_port_init: %s", esp_err_to_name(err));
    return err;
  }

  /* Bump the preferred ATT MTU from the 23-B default. NimBLE answers the
   * peer's MTU-exchange with this value; combined with the proactive
   * exchange we kick off on connect, this gets the link to a usable MTU
   * before the first notify rather than relying on the central. */
  int mtu_rc = ble_att_set_preferred_mtu(BLE_PREFERRED_MTU);
  if (mtu_rc != 0) {
    ESP_LOGW(TAG, "ble_att_set_preferred_mtu(%d): rc=%d", BLE_PREFERRED_MTU,
             mtu_rc);
  }

  /* +9 dBm is the chip ceiling without external PA. The dominant
   * link-budget factor is still the external FPC antenna — without it
   * seated, TX-power gains barely register. */
  esp_err_t pwr_err =
      esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
  if (pwr_err != ESP_OK) {
    ESP_LOGW(TAG, "esp_ble_tx_power_set: %s", esp_err_to_name(pwr_err));
  } else {
    ESP_LOGI(TAG, "BLE TX power set to +9 dBm");
  }

  ble_hs_cfg.reset_cb = on_reset;
  ble_hs_cfg.sync_cb = on_sync;

  int rc = gatt_svr_init();
  if (rc != 0) {
    ESP_LOGE(TAG, "gatt_svr_init: rc=%d", rc);
    return ESP_FAIL;
  }

  nimble_port_freertos_init(host_task);
  ESP_LOGI(TAG, "ble_init OK; NimBLE host task spawned");
  return ESP_OK;
}
