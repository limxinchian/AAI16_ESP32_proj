#include "motion_engine.h"
#include "motion_core.h"
#include "model_a_bundle.h"
#include <cstring>
#include "tensorflow/lite/micro/micro_log.h"

namespace motion {
ModelView bundled_model()
{
    return {motion_bundle::model, sizeof(motion_bundle::model), motion_bundle::sha256,
        motion_bundle::version, motion_bundle::input_scale, motion_bundle::output_scale,
        motion_bundle::input_zero_point, motion_bundle::output_zero_point};
}
bool bundled_self_test(Engine &engine)
{
    unsigned index = 0;
    for (const auto &golden : motion_bundle::golden) {
        float features[60], logits[2];
        int8_t input[60], output[2];
        uint32_t clipped;
        if (!motion_preprocess(golden.raw, features) ||
            !motion_quantize(features, engine.input_scale(), engine.input_zero_point(), input, &clipped) ||
            std::memcmp(input, golden.input, 60) || !engine.invoke(input, output) ||
            std::memcmp(output, golden.output, 2) ||
            motion_decode(output, engine.output_scale(), engine.output_zero_point(), logits) != golden.prediction) {
            MicroPrintf("Model A golden vector %u failed (preprocessing, quantization, Invoke or output); runtime=%s",
                index, engine.error());
            return false;
        }
        ++index;
    }
    return true;
}
}
