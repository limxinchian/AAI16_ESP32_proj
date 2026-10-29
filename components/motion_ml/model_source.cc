#include "motion_engine.h"
#include "motion_core.h"
#include "model_a_bundle.h"
#include "model_b_bundle.h"
#include <cstring>
#include "tensorflow/lite/micro/micro_log.h"

namespace motion {
const ModelDescriptor *bundled_model(ModelFamily family)
{
    switch (family) {
    case ModelFamily::A: return &motion_bundle_a::descriptor;
    case ModelFamily::B: return &motion_bundle_b::descriptor;
    default: return nullptr;
    }
}
const ModelDescriptor &startup_model()
{
#if MOTION_BOOT_MODEL_B
    return *bundled_model(ModelFamily::B);
#else
    return *bundled_model(ModelFamily::A);
#endif
}
const char *model_family_name(ModelFamily family)
{
    switch (family) {
    case ModelFamily::A: return "A";
    case ModelFamily::B: return "B";
    default: return "UNKNOWN";
    }
}
bool model_self_test(Engine &engine, const ModelDescriptor &model, size_t &passed)
{
    passed = 0;
    if (!engine.ready() || model.compatibility_profile != kMotionCompatibilityProfile ||
        !model.golden || !model.golden_count || model.golden_count > 8 ||
        engine.input_scale() != model.view.input_scale || engine.input_zero_point() != model.view.input_zero_point ||
        engine.output_scale() != model.view.output_scale || engine.output_zero_point() != model.view.output_zero_point)
        return false;
    for (size_t index = 0; index < model.golden_count; ++index) {
        const auto &golden = model.golden[index];
        float features[60], logits[2];
        int8_t input[60], output[2];
        uint32_t clipped;
        if (!motion_preprocess(golden.raw, features) ||
            !motion_quantize(features, engine.input_scale(), engine.input_zero_point(), input, &clipped) ||
            std::memcmp(input, golden.input, 60) || !engine.invoke(input, output) ||
            std::memcmp(output, golden.output, 2) ||
            motion_decode(output, engine.output_scale(), engine.output_zero_point(), logits) != golden.prediction) {
            MicroPrintf("Model %s golden vector %u failed (preprocessing, quantization, Invoke or output); runtime=%s",
                model_family_name(model.family), (unsigned)index, engine.error());
            return false;
        }
        ++passed;
    }
    return true;
}
}
