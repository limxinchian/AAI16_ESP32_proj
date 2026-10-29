#include "motion_engine.h"
#include "motion_backend.h"
#include "motion_core.h"
#include "mbedtls/sha256.h"
#include <cassert>
#include <cstdio>
#include <cstring>

int main(int argc, char **argv)
{
    assert(argc == 2 || argc == 4);
#if ESP_NN
    assert(motion_inference_backend() == aai16_Backend_BACKEND_ESP_NN_ANSI);
#else
    assert(motion_inference_backend() == aai16_Backend_BACKEND_TFLM_REFERENCE);
#endif
    assert(!std::strcmp(argv[1], "a") || !std::strcmp(argv[1], "b"));
    const auto family = !std::strcmp(argv[1], "a") ? motion::ModelFamily::A : motion::ModelFamily::B;
    const auto &descriptor = *motion::bundled_model(family);
    assert(!motion::bundled_model(static_cast<motion::ModelFamily>(99)));
    assert(!std::strcmp(motion::model_family_name(static_cast<motion::ModelFamily>(99)), "UNKNOWN"));
#if MOTION_BOOT_MODEL_B
    assert(motion::startup_model().family == motion::ModelFamily::B);
#else
    assert(motion::startup_model().family == motion::ModelFamily::A);
#endif
    assert(descriptor.golden_count == 4);
    assert(!std::strcmp(descriptor.view.version, family == motion::ModelFamily::A ?
        "aai16-motion-model-a-0.1.0" : "aai16-motion-model-b-0.1.0"));
    alignas(16) uint8_t arena[16384];
    motion::Engine engine;
    auto model = descriptor.view;
    size_t passed = 99;
    assert(!motion::model_self_test(engine, descriptor, passed) && passed == 0);
    int8_t input[60] = {}, output[2];
    assert(!engine.invoke(input, output));
    assert(!engine.init(model, nullptr, 16384));
    assert(!engine.init(model, arena + 1, 16383));
    alignas(16) uint8_t corrupt[65536];
    assert(model.length <= sizeof(corrupt));
    std::memcpy(corrupt, model.bytes, model.length);
    corrupt[100] ^= 1;
    auto wrong = model;
    wrong.bytes = corrupt;
    assert(!engine.init(wrong, arena, sizeof(arena)));
    assert(!std::strcmp(engine.error(), "sha256_mismatch"));
    uint8_t digest[32];
    std::memset(corrupt, 0, sizeof(corrupt));
    assert(!mbedtls_sha256(corrupt, model.length, digest, 0));
    wrong.sha256 = digest;
    assert(!engine.init(wrong, arena, sizeof(arena)));
    assert(!std::strcmp(engine.error(), "invalid_flatbuffer"));
    wrong = model;
    wrong.input_scale *= 2;
    assert(!engine.init(wrong, arena, sizeof(arena)));
    assert(!std::strcmp(engine.error(), "tensor_contract"));
    wrong = model;
    wrong.output_zero_point++;
    assert(!engine.init(wrong, arena, sizeof(arena)));
    assert(!std::strcmp(engine.error(), "tensor_contract"));
    // Supplying the other model's output quantization must never be accepted.
    const auto &other = *motion::bundled_model(family == motion::ModelFamily::A ? motion::ModelFamily::B : motion::ModelFamily::A);
    wrong = model;
    wrong.output_scale = other.view.output_scale;
    wrong.output_zero_point = other.view.output_zero_point;
    assert(!engine.init(wrong, arena, sizeof(arena)));
    assert(!std::strcmp(engine.error(), "tensor_contract"));
    assert(!engine.init(model, arena, 1024));
    assert(engine.init(model, arena, sizeof(arena)));
    assert(motion::model_self_test(engine, descriptor, passed) && passed == 4);
    assert(!motion::model_self_test(engine, other, passed) && passed == 0);
    auto invalid = descriptor;
    invalid.golden_count = 0;
    assert(!motion::model_self_test(engine, invalid, passed) && passed == 0);
    invalid = descriptor; invalid.golden = nullptr;
    assert(!motion::model_self_test(engine, invalid, passed));
    invalid = descriptor; invalid.compatibility_profile++;
    assert(!motion::model_self_test(engine, invalid, passed));
    motion::GoldenVector mutated = descriptor.golden[0];
    invalid = descriptor; invalid.golden = &mutated; invalid.golden_count = 1;
    mutated.input[0] ^= 1;
    assert(!motion::model_self_test(engine, invalid, passed) && passed == 0);
    mutated = descriptor.golden[0]; mutated.output[0] ^= 1;
    assert(!motion::model_self_test(engine, invalid, passed) && passed == 0);
    mutated = descriptor.golden[0]; mutated.prediction ^= 1;
    assert(!motion::model_self_test(engine, invalid, passed) && passed == 0);
    assert(motion::model_self_test(engine, descriptor, passed) && passed == 4);
    size_t used = engine.arena_used();
    // Repeated Invoke must preserve the arena allocation and outputs.
    for (unsigned i = 0; i < 10000; ++i) {
        const auto &g = descriptor.golden[i % descriptor.golden_count];
        assert(engine.invoke(g.input, output));
        assert(!std::memcmp(output, g.output, 2));
        assert(engine.arena_used() == used);
    }
    std::printf("TFLM Model %s: 4/4 golden vectors exact; 10000 stable Invoke calls; host arena=%zu (not MCU RAM)\n",
        motion::model_family_name(family), used);
    if (argc == 4) {
        FILE *in = std::fopen(argv[2], "rb"), *out = std::fopen(argv[3], "wb");
        assert(in && out);
        unsigned vectors = 0;
        while (std::fread(input, 1, sizeof(input), in) == sizeof(input)) {
            assert(engine.invoke(input, output));
            assert(std::fwrite(output, 1, 2, out) == 2);
            vectors++;
        }
        std::fclose(in); std::fclose(out);
        std::printf("TFLM: executed %u Python reference inputs\n", vectors);
    }
}
