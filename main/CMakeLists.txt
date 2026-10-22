idf_component_register(
    SRCS "main.c" "rc522_reader.c" "ld2450_reader.c" "hcsr04p_reader.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        dht22_moisture_sensor
        wifi_manager
        mqtt_manager
        i2c_bus_manager
        esp_timer
        gy511_sensor
        sensor_status
        spi_bus_manager
        rc522_sensor
        ld2450_sensor
        hcsr04p_sensor
)
