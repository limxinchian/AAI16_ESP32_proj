# Builds for esp32s3 with ESP-IDF 6.1 (clean build verified); logic is host-tested under tests/ - see STATUS.md.
idf_component_register(
    SRCS "node_core.c" "node_topics.c"
    INCLUDE_DIRS "include"
    REQUIRES aai16_protocol config_manager sensor_decode sensor_runtime ml_core node_services
)
