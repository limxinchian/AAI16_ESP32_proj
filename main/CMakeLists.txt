idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES node_core platform_esp nvs_flash
)
