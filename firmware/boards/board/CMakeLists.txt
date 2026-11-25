# XIAO ESP32-S3 Plus board support — pin assignments and other
# board-specific constants. Header-only "interface" component: no
# sources, just a public include dir. Consumers get GPIO_NUM_x and
# ADC_CHANNEL_x types transitively through REQUIRES.
idf_component_register(
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_adc
)
