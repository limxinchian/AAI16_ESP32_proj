set(MOTION_BOOT_MODEL "A" CACHE STRING "Provider development selection (A or B); Phase 5B worker always boots A")
set_property(CACHE MOTION_BOOT_MODEL PROPERTY STRINGS A B)
if(NOT MOTION_BOOT_MODEL MATCHES "^(A|B)$")
    message(FATAL_ERROR "MOTION_BOOT_MODEL must be A or B")
endif()
set(generator "${CMAKE_CURRENT_LIST_DIR}/../../tools/ml_inference/generate_bundle.py")
idf_component_register(SRCS "motion_core.c" "motion_engine.cc" "motion_ml.cc" "model_source.cc" "motion_runtime.cc" "motion_candidate.cc" "model_runtime.cc"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "." "${CMAKE_CURRENT_BINARY_DIR}"
    PRIV_REQUIRES esp-tflite-micro esp_timer esp_hw_support mbedtls
    REQUIRES freertos aai16_protocol)
# ESP_NN is PRIVATE to the pinned TFLM target. Compile the single reporter on
# that target so its flags and sdkconfig are the same as the actual FC wrapper.
# Do not infer the kernel path from the motion/MQTT/upload translation units.
idf_component_get_property(tflm_lib espressif__esp-tflite-micro COMPONENT_LIB)
idf_component_get_property(protocol_lib aai16_protocol COMPONENT_LIB)
target_sources(${tflm_lib} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/motion_backend.c")
target_include_directories(${tflm_lib} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include")
target_link_libraries(${tflm_lib} PRIVATE ${protocol_lib})
foreach(family a b)
    set(model_dir "${CMAKE_CURRENT_LIST_DIR}/../../models/motion_model_${family}")
    set(bundle "${CMAKE_CURRENT_BINARY_DIR}/model_${family}_bundle.h")
    add_custom_command(OUTPUT "${bundle}"
        COMMAND ${PYTHON} "${generator}" --model-dir "${model_dir}" --output "${bundle}" --model ${family}
        DEPENDS "${generator}" "${model_dir}/model_int8.tflite"
            "${model_dir}/manifest.json" "${model_dir}/input_contract.json"
            "${model_dir}/quantization.json" "${model_dir}/golden_vectors.json"
        VERBATIM)
    target_sources(${COMPONENT_LIB} PRIVATE "${bundle}")
endforeach()
target_compile_definitions(${COMPONENT_LIB} PRIVATE MOTION_BOOT_MODEL_B=$<STREQUAL:${MOTION_BOOT_MODEL},B>)
message(STATUS "Motion provider development selection: ${MOTION_BOOT_MODEL}; Phase 5B worker boot policy: bundled A")
# Match NumPy's ordered float32 operations: no reassociation or contraction.
target_compile_options(${COMPONENT_LIB} PRIVATE -fno-fast-math -ffp-contract=off)
