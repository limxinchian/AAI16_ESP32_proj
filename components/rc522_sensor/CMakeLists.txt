idf_component_register(SRCS "rc522_sensor.c" INCLUDE_DIRS "include"
    REQUIRES esp_common PRIV_REQUIRES spi_bus_manager esp_driver_gpio esp_timer freertos)
