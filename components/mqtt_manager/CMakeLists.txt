
idf_component_register(
    SRCS "mqtt_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES
        mqtt
        esp_event
        esp_netif
        freertos
        json
        sensor_status
        ld2450_sensor
    PRIV_REQUIRES esp_timer esp_hw_support
)
