idf_component_register(
    SRCS "sensor_status.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common
    PRIV_REQUIRES esp_timer freertos log
)
