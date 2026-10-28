#include "motion_engine.h"
#include "motion_core.h"
#include "model_a_bundle.h"
#include "mbedtls/sha256.h"
#include <cassert>
#include <cstdio>
#include <cstring>

int main(int argc, char **argv)
{
    alignas(16) uint8_t arena[16384];
    motion::Engine engine;
    auto model = motion::bundled_model();
    int8_t input[60] = {}, output[2];
    assert(!engine.invoke(input, output));
    assert(!engine.init(model, nullptr, 16384));
    assert(!engine.init(model, arena + 1, 16383));
    alignas(16) uint8_t corrupt[sizeof(motion_bundle::model)];
    std::memcpy(corrupt, model.bytes, model.length);
    corrupt[100] ^= 1;
    auto wrong = model;
    wrong.bytes = corrupt;
    assert(!engine.init(wrong, arena, sizeof(arena)));
    assert(!std::strcmp(engine.error(), "sha256_mismatch"));
    uint8_t digest[32];
    std::memset(corrupt, 0, sizeof(corrupt));
    assert(!mbedtls_sha256(corrupt, sizeof(corrupt), digest, 0));
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
    assert(!engine.init(model, arena, 1024));
    assert(engine.init(model, arena, sizeof(arena)));
    assert(motion::bundled_self_test(engine));
    size_t used = engine.arena_used();
    // Repeated Invoke must preserve the arena allocation and outputs.
    for (unsigned i = 0; i < 10000; ++i) {
        const auto &g = motion_bundle::golden[i % 4];
        assert(engine.invoke(g.input, output));
        assert(!std::memcmp(output, g.output, 2));
        assert(engine.arena_used() == used);
    }
    std::printf("TFLM: 4/4 golden vectors exact; 10000 stable Invoke calls; host arena=%zu (not MCU RAM)\n", used);
    if (argc == 3) {
        FILE *in = std::fopen(argv[1], "rb"), *out = std::fopen(argv[2], "wb");
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
