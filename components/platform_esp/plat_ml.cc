/* Inference runtime: real esp-tflite-micro (with ESP-NN kernels when enabled at build time).
 * Only FULLY_CONNECTED and SOFTMAX are registered: those are the only ops the two trained models use
 * (see report.json under ml/artifacts for each task). A model needing other ops is rejected at load time, never run.
 * NOT hardware-verified. */
#include <math.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/schema/schema_generated.h"

extern "C" {
#include "platform_esp.h"
}

static const char *TAG = "plat_ml";

namespace {

struct Loaded {
    uint8_t *model = nullptr;
    uint8_t *arena = nullptr;
    tflite::MicroInterpreter *interp = nullptr;
    uint32_t n_in = 0, n_out = 0;
};

Loaded g_cur;
tflite::MicroMutableOpResolver<2> *g_resolver = nullptr;

void release(Loaded &l)
{
    delete l.interp;
    heap_caps_free(l.arena);
    heap_caps_free(l.model);
    l = Loaded();
}

bool same_quant(float a, float b) { return fabsf(a - b) <= 1e-6f * fmaxf(1.0f, fabsf(b)); }

aai16_ModelReason reject(Loaded &l, const char *why)
{
    ESP_LOGE(TAG, "model rejected: %s", why);
    release(l);
    return aai16_ModelReason_MODEL_REASON_RUNTIME_REJECTED;
}

} // namespace

extern "C" aai16_ModelReason plat_ml_load(void *ctx, const ml_package_t *pkg)
{
    (void)ctx;
    Loaded n;
    const aai16_ModelContract &c = pkg->contract;

    /* The flatbuffer must be 16-byte aligned; the package stores it at an arbitrary flash offset, so copy it. */
    n.model = static_cast<uint8_t *>(heap_caps_aligned_alloc(16, pkg->tflite_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    n.arena = static_cast<uint8_t *>(heap_caps_aligned_alloc(16, c.tensor_arena_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!n.model || !n.arena) return reject(n, "out of memory for model/arena");
    memcpy(n.model, pkg->tflite, pkg->tflite_len);

    const tflite::Model *m = tflite::GetModel(n.model);
    if (m->version() != TFLITE_SCHEMA_VERSION) return reject(n, "unsupported flatbuffer schema version");

    if (!g_resolver) {
        g_resolver = new tflite::MicroMutableOpResolver<2>();
        if (g_resolver->AddFullyConnected() != kTfLiteOk || g_resolver->AddSoftmax() != kTfLiteOk) {
            return reject(n, "op registration failed");
        }
    }
    n.interp = new tflite::MicroInterpreter(m, *g_resolver, n.arena, c.tensor_arena_bytes);
    if (n.interp->AllocateTensors() != kTfLiteOk) return reject(n, "AllocateTensors failed (unsupported op or arena too small)");

    TfLiteTensor *in = n.interp->input(0), *out = n.interp->output(0);
    if (n.interp->inputs_size() != 1 || n.interp->outputs_size() != 1) return reject(n, "expected one input and one output");
    if (in->type != kTfLiteInt8 || out->type != kTfLiteInt8) return reject(n, "tensors are not int8");
    if (in->dims->size != 2 || in->dims->data[0] != 1 || (uint32_t)in->dims->data[1] != c.feature_count) return reject(n, "input shape != contract");
    const uint32_t n_out_expected = c.output_count ? c.output_count : c.labels_count;
    if (out->dims->size != 2 || out->dims->data[0] != 1 || (uint32_t)out->dims->data[1] != n_out_expected) return reject(n, "output shape != contract");
    if (!same_quant(in->params.scale, c.input_scale) || in->params.zero_point != c.input_zero_point ||
        !same_quant(out->params.scale, c.output_scale) || out->params.zero_point != c.output_zero_point) {
        return reject(n, "quantization parameters differ from the contract");
    }
    n.n_in = c.feature_count;
    n.n_out = n_out_expected;

    ESP_LOGI(TAG, "model loaded: %u B flatbuffer, arena used %u of %u B", (unsigned)pkg->tflite_len,
             (unsigned)n.interp->arena_used_bytes(), (unsigned)c.tensor_arena_bytes);
    release(g_cur);
    g_cur = n;
    return aai16_ModelReason_MODEL_REASON_NONE;
}

extern "C" bool plat_ml_invoke(void *ctx, const int8_t *in, uint32_t n_in, int8_t *out, uint32_t n_out, uint32_t *micros)
{
    (void)ctx;
    if (!g_cur.interp || n_in != g_cur.n_in || n_out != g_cur.n_out) return false;
    memcpy(g_cur.interp->input(0)->data.int8, in, n_in);
    const int64_t t0 = esp_timer_get_time();
    if (g_cur.interp->Invoke() != kTfLiteOk) return false;
    *micros = (uint32_t)(esp_timer_get_time() - t0);
    memcpy(out, g_cur.interp->output(0)->data.int8, n_out);
    return true;
}

extern "C" void plat_ml_fill(node_platform_t *pf)
{
    pf->ml_load = plat_ml_load;
    pf->ml_invoke = plat_ml_invoke;
}
