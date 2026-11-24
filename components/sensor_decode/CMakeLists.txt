# Builds for esp32s3 with ESP-IDF 6.1; the logic is host-tested under tests/ (see README.md, Testing).
idf_component_register(
    SRCS "sensor_decode.c"
    INCLUDE_DIRS "include"
    REQUIRES aai16_protocol config_manager
)
