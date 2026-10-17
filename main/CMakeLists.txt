
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        dht22_moisture_sensor
        wifi_manager
        mqtt_manager
        i2c_bus_manager
        esp_timer
        gy511_sensor
        sensor_status
)
