# Builds for esp32s3 with ESP-IDF 6.1 (clean build verified); logic is host-tested under tests/ - see STATUS.md.
idf_component_register(
    SRCS "sensor_decode.c"
    INCLUDE_DIRS "include"
    REQUIRES aai16_protocol config_manager
)
