
idf_component_register(
    SRCS "gy511_sensor.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES
        i2c_bus_manager
        esp_driver_i2c
        log
)
