#pragma once
#include <cstddef>
#include <cstdint>
#include "model_provider.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

namespace motion {
class Engine {
public:
    Engine() = default;
    ~Engine();
    Engine(const Engine &) = delete;
    Engine &operator=(const Engine &) = delete;
    bool init(const ModelView &view, uint8_t *arena, size_t arena_size);
    bool invoke(const int8_t input[60], int8_t output[2]);
    void reset() { fail("not_initialized"); }
    bool ready() const { return interpreter_ && input_ && output_; }
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
}
