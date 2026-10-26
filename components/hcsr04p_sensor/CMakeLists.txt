idf_component_register(
    SRCS "hcsr04p_sensor.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_mcpwm esp_driver_gpio esp_timer
)
