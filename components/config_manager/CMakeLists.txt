# Builds for esp32s3 with ESP-IDF 6.1; the logic is host-tested under tests/ (see README.md, Testing).
idf_component_register(
    SRCS "config_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES aai16_protocol
)
