#include "motion_ml.h"
#include "motion_engine.h"
#include <cstring>
#include <inttypes.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

namespace {
constexpr unsigned kQueueLength = 32;
constexpr size_t kArenaBytes = 16 * 1024;
constexpr unsigned kStackBytes = 6144;
constexpr UBaseType_t kPriority = 3;
constexpr BaseType_t kCore = 1;
struct QueuedSample { motion_sample_t sample; uint32_t epoch; };
StaticQueue_t queue_control;
alignas(16) uint8_t queue_storage[kQueueLength * sizeof(QueuedSample)];
QueueHandle_t queue;
portMUX_TYPE ingress_lock = portMUX_INITIALIZER_UNLOCKED;
struct Control {
    uint32_t epoch, drops, interval_ms, generation;
    bool enabled, available;
} control = {0, 0, 500, 0, true, false};
uint32_t boot_id;
motion_result_callback_t on_result;
TaskHandle_t worker;

Control snapshot()
{
    portENTER_CRITICAL(&ingress_lock);
    Control value = control;
    portEXIT_CRITICAL(&ingress_lock);
    return value;
}

void emit(motion_result_t &result, motion_status_t status)
{
    static motion_status_t last_status = MOTION_INIT_FAILED;
    static int64_t next_status_us = 0, heartbeat_us = 0;
    result.status = status;
    result.uptime_us = esp_timer_get_time();
    if (status != MOTION_PREDICTION && result.uptime_us < next_status_us) return;
    if (status != MOTION_PREDICTION && status == last_status && result.uptime_us < heartbeat_us) return;
    next_status_us = result.uptime_us + 1000000;
    heartbeat_us = result.uptime_us + 60000000;
    last_status = status;
    if (status == MOTION_PREDICTION) {
        ESP_LOGI("ML", "model=A state=%s inference_us=%" PRId64 " samples=%" PRIu32 "..%" PRIu32
            " clipped=%" PRIu32 " total=%" PRIu32,
            result.class_id == 0 ? "REST" : "MOVING", result.inference_us,
            result.first_sequence, result.last_sequence, result.clipped, result.clipped_total);
    } else {
        result.class_id = -1;
        result.inference_us = 0;
        result.clipped = 0;
        result.first_sequence = result.last_sequence = 0;
        result.window_start_us = result.window_end_us = 0;
        std::memset(result.output_int8, 0, sizeof(result.output_int8));
        std::memset(result.logits, 0, sizeof(result.logits));
        ESP_LOGI("ML", "model=A ready=%d status=%s interval_ms=%" PRIu32,
            result.model_ready, motion_status_name(status), result.interval_ms);
    }
    if (on_result) on_result(&result);
}

void run(void *)
{
    // Lifetime is the worker's lifetime. Allocations occur only during startup.
    static motion::Engine engine;
    static motion_window_t window;
    motion::ModelView model = motion::bundled_model();
    auto *arena = static_cast<uint8_t *>(heap_caps_aligned_alloc(16, kArenaBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    motion_result_t result = {};
    std::strncpy(result.model_version, model.version, sizeof(result.model_version) - 1);
    result.boot_id = boot_id;
    result.class_id = -1;
    result.model_ready = arena && engine.init(model, arena, kArenaBytes);
    const char *init_error = arena ? engine.error() : "arena_allocation";
    if (result.model_ready) {
        result.arena_used_bytes = engine.arena_used();
        result.model_ready = motion::bundled_self_test(engine);
        if (!result.model_ready) init_error = "golden_vector_mismatch";
        ESP_LOGI("ML", "Model A startup golden vectors: %s", result.model_ready ? "4/4 PASS" : "FAIL");
    }
    if (!result.model_ready) {
        ESP_LOGE("ML", "initialization failed: %s; live inference disabled",
            init_error);
        // Engine/arena remain owned here, even on failure; no dangling tensors.
    }
    ESP_LOGI("ML", "arena_reserved=%u arena_used=%u internal_free=%u internal_min=%u ESP_NN=%d core=%d stack=%u",
        (unsigned)kArenaBytes, (unsigned)result.arena_used_bytes,
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        1, // Pinned esp-tflite-micro 1.4.1 compiles its wrappers with -DESP_NN.
        (int)kCore, kStackBytes);
    uint32_t epoch = snapshot().epoch, previous_drops = 0;
    int64_t last_valid_us = -1, next_health_us = 0;
    motion_status_t status = MOTION_COLLECTING;
    while (true) {
        QueuedSample entry;
        bool received = xQueueReceive(queue, &entry, pdMS_TO_TICKS(100)) == pdTRUE;
        Control state = snapshot();
        result.interval_ms = state.interval_ms;
        result.generation = state.generation;
        result.queue_drops = state.drops;
        if (epoch != state.epoch) {
            motion_window_reset(&window);
            last_valid_us = -1;
            status = state.drops != previous_drops ? MOTION_QUEUE_OVERFLOW : MOTION_COLLECTING;
            epoch = state.epoch;
        }
        previous_drops = state.drops;
        if (!result.model_ready) status = result.invoke_failures ? MOTION_INVOKE_FAILED : MOTION_INIT_FAILED;
        else if (!state.available) status = MOTION_SENSOR_UNAVAILABLE;
        else if (!state.enabled) status = MOTION_SENSOR_DISABLED;
        else if (state.interval_ms != 100) status = MOTION_WAIT_100MS;
        else if (received && entry.epoch == epoch) {
            const auto &s = entry.sample;
            int64_t now = esp_timer_get_time();
            if (s.boot_id != boot_id || s.generation != state.generation || s.acquired_us < 0 || s.acquired_us > now ||
                now - s.acquired_us > 150000) {
                motion_window_reset(&window);
                status = MOTION_DISCONTINUITY;
            } else {
                bool full = motion_window_push(&window, &s, &status);
                if (window.count) last_valid_us = s.acquired_us;
                if (full) {
                    float raw[20][3], features[60];
                    int8_t input[60];
                    for (unsigned i = 0; i < 20; ++i) std::memcpy(raw[i], window.samples[i].xyz, sizeof(raw[i]));
                    bool encoded = motion_preprocess(raw, features) &&
                        motion_quantize(features, engine.input_scale(), engine.input_zero_point(), input, &result.clipped);
                    if (!encoded) {
                        motion_window_reset(&window);
                        status = MOTION_INVALID_SAMPLE;
                    } else {
                        result.clipped_total += result.clipped;
                        int64_t start = esp_timer_get_time();
                        bool ok = engine.invoke(input, result.output_int8);
                        result.inference_us = esp_timer_get_time() - start;
                        if (!ok) {
                            ++result.invoke_failures;
                            result.model_ready = false; // Fail closed until restart.
                            motion_window_reset(&window);
                            status = MOTION_INVOKE_FAILED;
                            ESP_LOGE("ML", "Invoke failed: %s", engine.error());
                        } else {
                            result.class_id = motion_decode(result.output_int8, engine.output_scale(),
                                engine.output_zero_point(), result.logits);
                            result.first_sequence = window.samples[0].sequence;
                            result.last_sequence = window.samples[19].sequence;
                            result.window_start_us = window.samples[0].acquired_us;
                            result.window_end_us = window.samples[19].acquired_us;
                            motion_window_advance(&window);
                            // Configuration/overflow may race with Invoke. Cancel
                            // that result before admitting it to the callback.
                            if (snapshot().epoch == epoch) status = MOTION_PREDICTION;
                            else { motion_window_reset(&window); status = MOTION_DISCONTINUITY; }
                        }
                    }
                }
            }
        } else if (!received && result.model_ready && state.enabled && state.interval_ms == 100 &&
                   (last_valid_us < 0 || esp_timer_get_time() - last_valid_us > 150000)) {
            if (window.count) motion_window_reset(&window);
            status = MOTION_NO_DATA;
        } else if (status == MOTION_PREDICTION) {
            status = MOTION_COLLECTING; // Never re-emit an old prediction.
        }
        result.window_resets = window.resets;
        emit(result, status);
        if (status == MOTION_PREDICTION) status = MOTION_COLLECTING;
        if (esp_timer_get_time() >= next_health_us) {
            next_health_us = esp_timer_get_time() + 60000000;
            ESP_LOGI("ML", "health drops=%" PRIu32 " resets=%" PRIu32 " failures=%" PRIu32
                " stack_min=%u internal_free=%u internal_min=%u",
                result.queue_drops, window.resets, result.invoke_failures,
                (unsigned)uxTaskGetStackHighWaterMark(nullptr),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        }
    }
}
}

extern "C" esp_err_t motion_ml_start(uint32_t identity, motion_result_callback_t callback)
{
    if (queue) return ESP_ERR_INVALID_STATE;
    boot_id = identity;
    on_result = callback;
    queue = xQueueCreateStatic(kQueueLength, sizeof(QueuedSample), queue_storage, &queue_control);
    if (!queue) return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCore(run, "motion_ml", kStackBytes, nullptr, kPriority, &worker, kCore) != pdPASS) {
        vQueueDelete(queue);
        queue = nullptr;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

extern "C" bool motion_ml_submit(const motion_sample_t *sample)
{
    if (!queue || !sample) return false;
    QueuedSample entry = {*sample, snapshot().epoch};
    if (xQueueSend(queue, &entry, 0) == pdTRUE) return true;
    portENTER_CRITICAL(&ingress_lock);
    ++control.drops;
    ++control.epoch; // Invalidate ALL older queued/in-flight windows on loss.
    portEXIT_CRITICAL(&ingress_lock);
    return false;
}

extern "C" void motion_ml_configure(uint32_t interval_ms, uint32_t generation, bool enabled)
{
    portENTER_CRITICAL(&ingress_lock);
    control.interval_ms = interval_ms;
    control.generation = generation;
    control.enabled = enabled;
    ++control.epoch;
    portEXIT_CRITICAL(&ingress_lock);
}

extern "C" void motion_ml_set_sensor_available(bool available)
{
    portENTER_CRITICAL(&ingress_lock);
    control.available = available;
    ++control.epoch;
    portEXIT_CRITICAL(&ingress_lock);
}
