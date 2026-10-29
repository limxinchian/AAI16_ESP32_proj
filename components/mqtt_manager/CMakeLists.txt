
idf_component_register(
    SRCS "mqtt_manager.c" "config_transactions.c"
    INCLUDE_DIRS "include"
    REQUIRES
        mqtt
        esp_event
        esp_netif
        freertos
        sensor_status
        ld2450_sensor
        hcsr04p_sensor
    PRIV_REQUIRES aai16_protocol esp_timer esp_hw_support mhz1911a_sensor motion_ml model_upload
)
