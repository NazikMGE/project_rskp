idf_component_register(
    SRCS "src/cal.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common
    PRIV_REQUIRES nvs_flash log
)
