// Laptop-only compatibility runner. Uses the unmodified production Engine and
// installed esp-tflite-micro kernels; never loads a model onto the ESP32.
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include "mbedtls/sha256.h"
#include "motion_engine.h"

int main(int argc, char **argv) {
    if (argc != 8) {
        std::fprintf(stderr, "usage: runner model inputs outputs input_scale input_zp output_scale output_zp\n");
        return 2;
    }
    alignas(16) static uint8_t model[65536];
    alignas(16) static uint8_t arena[16384];
    std::ifstream file(argv[1], std::ios::binary | std::ios::ate);
    if (!file || file.tellg() <= 0 || file.tellg() > sizeof(model)) return 3;
    const size_t length = static_cast<size_t>(file.tellg());
    file.seekg(0);
    if (!file.read(reinterpret_cast<char *>(model), length)) return 3;
    uint8_t digest[32];
    if (mbedtls_sha256(model, length, digest, 0) != 0) return 4;
    motion::ModelView view{model, length, digest, "host-file-validation",
        std::strtof(argv[4], nullptr), std::strtof(argv[6], nullptr),
        std::atoi(argv[5]), std::atoi(argv[7])};
    motion::Engine engine;
    if (!engine.init(view, arena, sizeof(arena))) {
        std::fprintf(stderr, "Engine init failed: %s\n", engine.error());
        return 5;
    }
    std::ifstream inputs(argv[2], std::ios::binary);
    std::ofstream outputs(argv[3], std::ios::binary | std::ios::trunc);
    if (!inputs || !outputs) return 6;
    size_t count = 0;
    int8_t input[60], output[2];
    while (inputs.read(reinterpret_cast<char *>(input), sizeof(input))) {
        if (!engine.invoke(input, output)) return 7;
        if (!outputs.write(reinterpret_cast<const char *>(output), sizeof(output))) return 8;
        ++count;
    }
    if (!inputs.eof() || inputs.gcount() != 0 || count == 0) return 9;
    outputs.close();
    if (!outputs) return 8;
    std::printf("Production Engine completed %zu vectors\n", count);
    return 0;
}
