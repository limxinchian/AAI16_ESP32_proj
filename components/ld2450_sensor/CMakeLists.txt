idf_component_register(SRCS "ld2450_sensor.c" "ld2450_parser.c"
    INCLUDE_DIRS "include" REQUIRES esp_common
    PRIV_REQUIRES esp_driver_uart esp_timer freertos)
