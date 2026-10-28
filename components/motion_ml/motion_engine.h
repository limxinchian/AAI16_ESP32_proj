#pragma once
#include <cstddef>
#include <cstdint>
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

namespace motion {
// Provider owns immutable, 16-byte-aligned bytes for the interpreter lifetime.
// A later flash-mapping provider can supply this view without changing Engine.
// Loading/switching/upload policy deliberately does not belong to this engine.
struct ModelView {
    const uint8_t *bytes;
    size_t length;
    const uint8_t *sha256;
    const char *version;
    float input_scale, output_scale;
    int input_zero_point, output_zero_point;
};
class Engine {
public:
    Engine() = default;
    ~Engine();
    Engine(const Engine &) = delete;
    Engine &operator=(const Engine &) = delete;
    bool init(const ModelView &view, uint8_t *arena, size_t arena_size);
    bool invoke(const int8_t input[60], int8_t output[2]);
    const char *error() const { return error_; }
    size_t arena_used() const { return interpreter_ ? interpreter_->arena_used_bytes() : 0; }
    float input_scale() const { return input_->params.scale; }
    int input_zero_point() const { return input_->params.zero_point; }
    float output_scale() const { return output_->params.scale; }
    int output_zero_point() const { return output_->params.zero_point; }
private:
    bool fail(const char *reason);
    tflite::MicroMutableOpResolver<1> resolver_;
    bool registered_ = false;
    alignas(tflite::MicroInterpreter) uint8_t storage_[sizeof(tflite::MicroInterpreter)];
    tflite::MicroInterpreter *interpreter_ = nullptr;
    TfLiteTensor *input_ = nullptr, *output_ = nullptr;
    const char *error_ = "not_initialized";
};
ModelView bundled_model();
bool bundled_self_test(Engine &engine);
}
