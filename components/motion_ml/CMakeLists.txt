set(model_dir "${CMAKE_CURRENT_LIST_DIR}/../../models/motion_model_a")
set(generator "${CMAKE_CURRENT_LIST_DIR}/../../tools/ml_inference/generate_bundle.py")
set(bundle "${CMAKE_CURRENT_BINARY_DIR}/model_a_bundle.h")
idf_component_register(SRCS "motion_core.c" "motion_engine.cc" "motion_ml.cc" "model_source.cc"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "." "${CMAKE_CURRENT_BINARY_DIR}"
    PRIV_REQUIRES esp-tflite-micro esp_timer esp_hw_support mbedtls
    REQUIRES freertos)
add_custom_command(OUTPUT "${bundle}"
    COMMAND ${PYTHON} "${generator}" --model-dir "${model_dir}" --output "${bundle}"
    DEPENDS "${generator}" "${model_dir}/model_int8.tflite"
        "${model_dir}/manifest.json" "${model_dir}/input_contract.json"
        "${model_dir}/quantization.json" "${model_dir}/golden_vectors.json"
    VERBATIM)
target_sources(${COMPONENT_LIB} PRIVATE "${bundle}")
# Match NumPy's ordered float32 operations: no reassociation or contraction.
target_compile_options(${COMPONENT_LIB} PRIVATE -fno-fast-math -ffp-contract=off)
