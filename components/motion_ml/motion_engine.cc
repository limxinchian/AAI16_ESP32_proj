#include "motion_engine.h"
#include <cmath>
#include <cstring>
#include <new>
#include "mbedtls/sha256.h"
#include "tensorflow/lite/schema/schema_generated.h"

namespace motion {
Engine::~Engine() { if (interpreter_) interpreter_->~MicroInterpreter(); }
bool Engine::fail(const char *reason)
{
    if (interpreter_) interpreter_->~MicroInterpreter();
    interpreter_ = nullptr;
    input_ = output_ = nullptr;
    error_ = reason;
    return false;
}
static bool tensor_matches(const TfLiteTensor *t, int width, float scale, int zp)
{
    return t && t->type == kTfLiteInt8 && t->dims && t->dims->size == 2 &&
        t->dims->data[0] == 1 && t->dims->data[1] == width && t->bytes == (size_t)width &&
        t->data.int8 && std::isfinite(t->params.scale) && t->params.scale > 0 &&
        t->params.scale == scale && t->params.zero_point == zp;
}
bool Engine::init(const ModelView &view, uint8_t *arena, size_t size)
{
    fail("initializing");
    if (!view.bytes || view.length < 8 || !view.sha256 || !view.version ||
        !arena || size < 1024 || reinterpret_cast<uintptr_t>(arena) % 16 ||
        reinterpret_cast<uintptr_t>(view.bytes) % 16) return fail("model_or_arena_invalid");
    uint8_t digest[32];
    if (mbedtls_sha256(view.bytes, view.length, digest, 0) != 0 ||
        std::memcmp(digest, view.sha256, 32)) return fail("sha256_mismatch");
    flatbuffers::Verifier verifier(view.bytes, view.length);
    if (!tflite::VerifyModelBuffer(verifier)) return fail("invalid_flatbuffer");
    const tflite::Model *model = tflite::GetModel(view.bytes);
    if (model->version() != TFLITE_SCHEMA_VERSION) return fail("schema_version");
    if (!model->subgraphs() || model->subgraphs()->size() != 1) return fail("subgraph_count");
    const auto *graph = model->subgraphs()->Get(0);
    if (!graph->inputs() || !graph->outputs() || graph->inputs()->size() != 1 ||
        graph->outputs()->size() != 1 || !graph->operators() ||
        graph->operators()->size() != 2) return fail("graph_contract");
    for (unsigned i = 0; i < 2; ++i) {
        const auto *op = graph->operators()->Get(i);
        if (!model->operator_codes() || op->opcode_index() >= model->operator_codes()->size())
            return fail("opcode_index");
        const auto *code = model->operator_codes()->Get(op->opcode_index());
        const auto *options = op->builtin_options_as_FullyConnectedOptions();
        if (code->builtin_code() != tflite::BuiltinOperator_FULLY_CONNECTED || code->version() != 4 ||
            !options || options->fused_activation_function() !=
                (i == 0 ? tflite::ActivationFunctionType_RELU : tflite::ActivationFunctionType_NONE) ||
            options->weights_format() != tflite::FullyConnectedOptionsWeightsFormat_DEFAULT)
            return fail("fully_connected_contract");
    }
    if (!registered_) {
        if (resolver_.AddFullyConnected() != kTfLiteOk) return fail("operator_registration");
        registered_ = true;
    }
    interpreter_ = new (storage_) tflite::MicroInterpreter(model, resolver_, arena, size);
    if (interpreter_->AllocateTensors() != kTfLiteOk) return fail("allocate_tensors");
    input_ = interpreter_->input(0);
    output_ = interpreter_->output(0);
    if (!tensor_matches(input_, 60, view.input_scale, view.input_zero_point) ||
        !tensor_matches(output_, 2, view.output_scale, view.output_zero_point)) return fail("tensor_contract");
    error_ = "ok";
    return true;
}
bool Engine::invoke(const int8_t input[60], int8_t output[2])
{
    if (!interpreter_ || !input || !output) { error_ = "not_initialized"; return false; }
    std::memcpy(input_->data.int8, input, 60);
    if (interpreter_->Invoke() != kTfLiteOk) { error_ = "invoke_failed"; return false; }
    std::memcpy(output, output_->data.int8, 2);
    error_ = "ok";
    return true;
}
}
