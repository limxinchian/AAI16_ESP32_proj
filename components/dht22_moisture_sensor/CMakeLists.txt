idf_component_register(
    SRCS "dht22.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_rmt
    PRIV_REQUIRES esp_rom
)