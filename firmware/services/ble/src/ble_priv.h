#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file ble_priv.h
 * @brief Internal declarations shared between `ble.c` and `ble_gatt.c`.
 *        Public consumers go through `include/ble/ble.h`.
 */

/**
 * @brief Register GATT services: SIG GAP+GATT, Battery (0x180F),
 *        and the StabilityTracker custom service (`5d3ace00-…`) with
 *        IMU stream / device info / control / sensor health /
 *        shot events / stability metrics / preshot transfer /
 *        hardware-tuning characteristics.
 *
 * Call once after `nimble_port_init()`, before
 * `nimble_port_freertos_init()` spawns the host task.
 */
int gatt_svr_init(void);

/**
 * @brief Value-attribute handle of the IMU stream characteristic,
 *        assigned by NimBLE inside `gatt_svr_init()`.
 *
 * Returned by this getter so `ble.c` can filter
 * `BLE_GAP_EVENT_SUBSCRIBE` events to only this handle.
 *
 * @return Handle, or 0 before `gatt_svr_init()` has run.
 */
uint16_t ble_gatt_imu_chr_handle(void);

/**
 * @brief Track the connection currently subscribed to IMU notify.
 *
 * @param conn_handle Active subscriber's handle, or
 *        `BLE_HS_CONN_HANDLE_NONE` when nobody is subscribed.
 */
void ble_gatt_set_imu_subscriber(uint16_t conn_handle);

/**
 * @brief Value-attribute handle of the shot-event characteristic,
 *        assigned by NimBLE inside `gatt_svr_init()`.
 *
 * @return Handle, or 0 before `gatt_svr_init()` has run.
 */
uint16_t ble_gatt_shot_chr_handle(void);

/**
 * @brief Track the connection currently subscribed to shot-event notify.
 *
 * @param conn_handle Active subscriber's handle, or
 *        `BLE_HS_CONN_HANDLE_NONE` when nobody is subscribed.
 */
void ble_gatt_set_shot_subscriber(uint16_t conn_handle);

/**
 * @brief Value-attribute handle of the stability-metrics characteristic.
 * @return Handle, or 0 before `gatt_svr_init()` has run.
 */
uint16_t ble_gatt_metrics_chr_handle(void);

/**
 * @brief Track the connection currently subscribed to metrics notify.
 *
 * @param conn_handle Active subscriber's handle, or
 *        `BLE_HS_CONN_HANDLE_NONE` when nobody is subscribed.
 */
void ble_gatt_set_metrics_subscriber(uint16_t conn_handle);

/**
 * @brief Value-attribute handle of the preshot-transfer characteristic,
 *        assigned by NimBLE inside `gatt_svr_init()`.
 * @return Handle, or 0 before `gatt_svr_init()` has run.
 */
uint16_t ble_gatt_preshot_chr_handle(void);

/**
 * @brief Track the connection currently subscribed to preshot-transfer
 *        notify.
 *
 * @param conn_handle Active subscriber's handle, or
 *        `BLE_HS_CONN_HANDLE_NONE` when nobody is subscribed.
 */
void ble_gatt_set_preshot_subscriber(uint16_t conn_handle);

/**
 * @brief Inject the local BLE MAC into the device-info characteristic.
 *
 * Called once from `ble.c`'s `on_sync`. NimBLE hands the MAC in
 * little-endian; the setter reverses it for the wire (big-endian).
 */
void ble_gatt_set_device_info_mac(const uint8_t mac_le[6]);

/**
 * @brief End any active session. Called from `ble.c` on disconnect so a
 *        session never outlives its connection.
 */
void ble_gatt_clear_session(void);

#ifdef __cplusplus
}
#endif
