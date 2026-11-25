idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        log
        freertos
        esp_driver_gpio
        esp_system
        nvs_flash
        battery
        ble
        esp32_BNO08x
        board
        button
        calibration
        power
        preshot
        shot_detect
        stability
        supervisor
)
