idf_component_register(
    SRCS "src/supervisor.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common
    PRIV_REQUIRES log freertos
)
