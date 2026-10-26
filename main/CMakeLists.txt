idf_component_register(
    SRCS "max6675_reader.c" "bme280_reader.c" "max6675_reader.c" "main.c" "rc522_reader.c" "ld2450_reader.c"
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
        bme280_sensor
        max6675_sensor
)
