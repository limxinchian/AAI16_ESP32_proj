idf_component_register(SRCS "max6675_sensor.c" INCLUDE_DIRS "include"
    REQUIRES esp_common PRIV_REQUIRES spi_bus_manager freertos log)
