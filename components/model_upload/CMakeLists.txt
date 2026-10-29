idf_component_register(SRCS "upload_store.cc" "model_upload.cc"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    PRIV_REQUIRES aai16_protocol motion_ml esp_partition esp_timer heap mbedtls freertos)
