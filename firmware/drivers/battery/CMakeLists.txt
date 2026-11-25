idf_component_register(
    SRCS "src/battery.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common
    PRIV_REQUIRES esp_adc esp_driver_gpio esp_system freertos log board
)
