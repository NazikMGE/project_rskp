# StabilityTracker BLE service component.
#
# Wraps NimBLE host stack into a small public API (ble.h) so the
# rest of firmware doesn't import NimBLE headers directly. Phase 1
# milestone: advertise `ST-XXXX` with the custom service UUID so
# the mobile app can discover the device. GATT services + IMU
# stream notify follow in subsequent milestones.
idf_component_register(
    SRCS
        "src/ble.c"
        "src/ble_gatt.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    REQUIRES esp_common
    PRIV_REQUIRES
        bt
        esp_hw_support
        esp_system
        freertos
        log
        preshot
        # ble_gatt.c decodes the tuning characteristics into
        # shot_detect_config_t / stability_config_t and forwards through
        # the services' own set_config entry points.
        shot_detect
        stability
)
