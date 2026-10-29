#pragma once
#include <cstddef>
#include <cstdint>
#include "motion_core.h"

namespace motion {
enum class ModelFamily { A, B };
constexpr uint32_t kMotionCompatibilityProfile = 1;
// Provider owns immutable, 16-byte-aligned bytes for the interpreter lifetime.
// Flash mapping/storage and activation policy stay outside the inference engine.
struct ModelView {
    const uint8_t *bytes;
    size_t length;
    const uint8_t *sha256;
    const char *version;
    float input_scale, output_scale;
    int input_zero_point, output_zero_point;
};
struct GoldenVector {
    float raw[MOTION_SAMPLES][MOTION_AXES];
    int8_t input[MOTION_INPUTS];
    int8_t output[2];
    int prediction;
};
// All referenced storage, including version and golden data, must remain alive
// while a consumer uses this descriptor. Providers do not own an interpreter.
// Profile 1 is the existing 10 Hz, 20-sample, stride-10 REST/MOVING pipeline.
struct ModelDescriptor {
    ModelFamily family;
    uint32_t compatibility_profile;
    ModelView view;
    const GoldenVector *golden;
    size_t golden_count;
};
const ModelDescriptor *bundled_model(ModelFamily family); // nullptr for unknown family
const ModelDescriptor &startup_model(); // Development-time choice; not runtime switching.
const char *model_family_name(ModelFamily family);
class Engine;
// Generic validation usable by a future provider; runs raw windows through the
// production preprocessing, encoding, interpreter and decoding. Fails closed.
bool model_self_test(Engine &engine, const ModelDescriptor &model, size_t &passed);
}
