idf_component_register(
    SRCS "plat_bus.c" "plat_storage.c" "plat_io.c" "plat_net.c" "plat_fill.c" "plat_ml.cc"
    INCLUDE_DIRS "include"
    REQUIRES node_core freertos
    PRIV_REQUIRES ml_core esp_driver_i2c esp_driver_uart esp_driver_spi esp_driver_gpio esp_driver_ledc
                  nvs_flash esp_partition esp_timer esp_wifi esp_netif esp_event heap log esp_rom
                  espressif__mqtt espressif__esp-tflite-micro
)
