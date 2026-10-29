#pragma once
#include "aai16.pb.h"

#ifdef __cplusplus
extern "C" {
#endif
// Compiled alongside the TFLM kernel wrapper, not its callers: ESP_NN is a
// private flag of that component. This is the build's execution path, not a
// runtime selector or a claim that every optimized kernel avoids fallback.
aai16_Backend motion_inference_backend(void);
const char *motion_inference_backend_name(void);
#ifdef __cplusplus
}
#endif
