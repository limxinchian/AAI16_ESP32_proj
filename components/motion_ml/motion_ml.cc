#include "motion_ml.h"
#include "motion_backend.h"
#include "model_runtime.h"
#include "motion_candidate.h"
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
} control = {0, 0, 100, 0, true, false};
uint32_t boot_id;
motion_result_callback_t on_result;
TaskHandle_t worker;
bool model_ready;
motion_model_state_t published_model;
motion_model_command_t model_command;
motion_model_reply_t model_reply;
bool command_pending, reply_pending;

void *allocate_model(size_t n) { return heap_caps_aligned_alloc(16,n,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT); }
void release_model(void *p) { heap_caps_free(p); }
void update_model(motion::ModelRuntime &runtime, motion_result_t &result)
{
    const auto &s=runtime.state();
    result.model_identity=s.active; result.model_epoch=s.epoch;
    std::strncpy(result.model_version,s.active.version,sizeof(result.model_version)-1);
    result.model_ready=s.ready;
    result.arena_used_bytes=s.ready?runtime.engine().arena_used():0;
    portENTER_CRITICAL(&ingress_lock);
    published_model=s; model_ready=s.ready;
    portEXIT_CRITICAL(&ingress_lock);
}

Control snapshot()
{
    portENTER_CRITICAL(&ingress_lock);
    Control value = control;
    portEXIT_CRITICAL(&ingress_lock);
    return value;
}

void emit(motion_result_t &result, motion_status_t status, const motion::ModelDescriptor &model)
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
        ESP_LOGI("ML", "model=%s state=%s epoch=%" PRIu32 " source=%" PRIu32 " inference_us=%" PRId64 " samples=%" PRIu32 "..%" PRIu32
            " clipped=%" PRIu32 " total=%" PRIu32,
            motion::model_family_name(model.family), result.class_id == 0 ? "REST" : "MOVING", result.model_epoch, result.model_identity.source, result.inference_us,
            result.first_sequence, result.last_sequence, result.clipped, result.clipped_total);
    } else {
        result.class_id = -1;
        result.inference_us = 0;
        result.clipped = 0;
        result.first_sequence = result.last_sequence = 0;
        result.window_start_us = result.window_end_us = 0;
        std::memset(result.output_int8, 0, sizeof(result.output_int8));
        std::memset(result.logits, 0, sizeof(result.logits));
        ESP_LOGI("ML", "model=%s ready=%d status=%s interval_ms=%" PRIu32,
            motion::model_family_name(model.family), result.model_ready, motion_status_name(status), result.interval_ms);
    }
    if (on_result) on_result(&result);
}

void run(void *)
{
    // Runtime/arenas and immutable RAM providers belong exclusively to this task.
    static motion::ModelRuntime runtime(allocate_model,release_model);
    static motion_window_t window;
    runtime.start(); // Every boot selects safe bundled A, including development builds.
    const motion::ModelDescriptor &model = runtime.model();
    const char *family = motion::model_family_name(model.family);
    ESP_LOGI("ML", "initializing model=%s version=%s bytes=%u profile=%" PRIu32,
        family, model.view.version, (unsigned)model.view.length, model.compatibility_profile);
    ESP_LOGI("ML", "model=%s input_scale=%.10g input_zero_point=%d output_scale=%.10g output_zero_point=%d",
        family, (double)model.view.input_scale, model.view.input_zero_point,
        (double)model.view.output_scale, model.view.output_zero_point);
    motion_result_t result = {};
    std::strncpy(result.model_version, model.view.version, sizeof(result.model_version) - 1);
    result.boot_id = boot_id;
    result.class_id = -1;
    update_model(runtime,result);
    ESP_LOGI("ML", "boot policy=bundled_A activation=volatile golden+trial=%s detail=%s",
        result.model_ready?"PASS":"FAIL",runtime.state().detail);
    if (!result.model_ready) {
        ESP_LOGE("ML", "initialization failed: %s; live inference disabled",
            runtime.state().detail);
        // Engine/arena remain owned here, even on failure; no dangling tensors.
    }
    ESP_LOGI("ML", "arena_reserved=%u arena_used=%u internal_free=%u internal_min=%u backend=%s backend_id=%u core=%d stack=%u",
        (unsigned)kArenaBytes, (unsigned)result.arena_used_bytes,
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        motion_inference_backend_name(), (unsigned)motion_inference_backend(),
        (int)kCore, kStackBytes);
    uint32_t epoch = snapshot().epoch, previous_drops = 0;
    int64_t last_valid_us = -1, next_health_us = 0, model_boundary_us=0;
    motion_status_t status = MOTION_COLLECTING;
    while (true) {
        // The only handoff point: no interpreter Invoke or input window access
        // is in progress. MQTT and sensor tasks never touch the runtime.
        portENTER_CRITICAL(&ingress_lock);
        bool change=command_pending;
        motion_model_command_t command=model_command;
        portEXIT_CRITICAL(&ingress_lock);
        if(change) {
            const auto before=runtime.state().epoch;
            int64_t start=esp_timer_get_time();
            auto reply=runtime.change(command);
            if(before!=runtime.state().epoch) {
                model_boundary_us=esp_timer_get_time();
                portENTER_CRITICAL(&ingress_lock);
                ++control.epoch;
                portEXIT_CRITICAL(&ingress_lock);
                motion_window_reset(&window); last_valid_us=-1;
                status=MOTION_COLLECTING;
            }
            update_model(runtime,result);
            ESP_LOGI("ML", "activation result=%d active=%s source=%" PRIu32 " epoch=%" PRIu32 " checkpoint_us=%" PRId64 " detail=%s",
                int(reply.result),result.model_version,result.model_identity.source,result.model_epoch,
                esp_timer_get_time()-start,reply.detail);
            portENTER_CRITICAL(&ingress_lock);
            model_reply=reply; reply_pending=true; command_pending=false;
            portEXIT_CRITICAL(&ingress_lock);
        }
        auto &engine=runtime.engine();
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
            if (s.boot_id != boot_id || s.generation != state.generation || s.acquired_us <= model_boundary_us || s.acquired_us > now ||
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
                        motion_runtime_enter();
                        int64_t start = esp_timer_get_time();
                        bool ok = engine.invoke(input, result.output_int8);
                        result.inference_us = esp_timer_get_time() - start;
                        motion_runtime_leave();
                        if (!ok) {
                            portENTER_CRITICAL(&ingress_lock);
                            model_ready = false;
                            portEXIT_CRITICAL(&ingress_lock);
                        }
                        if (!ok) {
                            ++result.invoke_failures;
                            runtime.recover_invoke();
                            model_boundary_us=esp_timer_get_time();
                            update_model(runtime,result);
                            portENTER_CRITICAL(&ingress_lock);
                            ++control.epoch;
                            portEXIT_CRITICAL(&ingress_lock);
                            motion_window_reset(&window);
                            status = MOTION_INVOKE_FAILED;
                            ESP_LOGE("ML", "Invoke failed; active=%s epoch=%" PRIu32 " ready=%d detail=%s",
                                result.model_version,result.model_epoch,result.model_ready,runtime.state().detail);
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
        emit(result, status, runtime.model());
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

extern "C" bool motion_ml_model_ready(void)
{
    portENTER_CRITICAL(&ingress_lock);
    bool ready = model_ready;
    portEXIT_CRITICAL(&ingress_lock);
    return ready;
}

extern "C" void motion_ml_model_state(motion_model_state_t *state)
{
    portENTER_CRITICAL(&ingress_lock);
    *state=published_model;
    portEXIT_CRITICAL(&ingress_lock);
}
extern "C" bool motion_ml_request_model(const motion_model_command_t *command)
{
    portENTER_CRITICAL(&ingress_lock);
    bool accepted=queue && !command_pending && !reply_pending;
    if(accepted) { model_command=*command; command_pending=true; }
    portEXIT_CRITICAL(&ingress_lock);
    return accepted;
}
extern "C" bool motion_ml_take_model_reply(motion_model_reply_t *reply)
{
    portENTER_CRITICAL(&ingress_lock);
    bool ready=reply_pending;
    if(ready) { *reply=model_reply; reply_pending=false; }
    portEXIT_CRITICAL(&ingress_lock);
    return ready;
}
