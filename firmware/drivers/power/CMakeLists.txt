idf_component_register(
    SRCS "src/power.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_gpio esp_hw_support freertos log button board
)
