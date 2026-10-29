#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif
#include "motion_backend.h"

aai16_Backend motion_inference_backend(void)
{
    // Match kernels/esp_nn/fully_connected.cc, then esp_nn.h's selection.
    // ESP_NN=1 includes both ANSI and optimized ESP-NN implementations.
#if defined(ESP_NN) && ESP_NN
#if defined(CONFIG_NN_OPTIMIZED)
    return aai16_Backend_BACKEND_ESP_NN_OPTIMIZED;
#else
    return aai16_Backend_BACKEND_ESP_NN_ANSI;
#endif
#else
    return aai16_Backend_BACKEND_TFLM_REFERENCE;
#endif
}

const char *motion_inference_backend_name(void)
{
    switch (motion_inference_backend()) {
    case aai16_Backend_BACKEND_TFLM_REFERENCE: return "BACKEND_TFLM_REFERENCE";
    case aai16_Backend_BACKEND_ESP_NN_ANSI: return "BACKEND_ESP_NN_ANSI";
    case aai16_Backend_BACKEND_ESP_NN_OPTIMIZED: return "BACKEND_ESP_NN_OPTIMIZED";
    default: return "BACKEND_UNSPECIFIED";
    }
}
