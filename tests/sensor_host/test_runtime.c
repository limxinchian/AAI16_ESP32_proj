// Six real sensor application loops with cooperative host scheduling.
// pthreads provide separate C stacks; only one simulated task runs at a time.
// This tests dispatch interleavings, not ESP32 SMP timing or the MQTT transport.
#include "platform.h"
#include "../rc522_host/platform.h"
#include <assert.h>
#include <pthread.h>
#include <string.h>

#include "../../main/main.c"
#include "../../main/rc522_reader.c"
#include "../../main/ld2450_reader.c"
#include "../../main/hcsr04p_reader.c"
#include "../../main/mhz1911a_reader.c"

typedef struct {
    pthread_t thread;
    bool exists, finished, running, notify_wait, notified;
    uint32_t notification;
    int64_t wake_us;
    void (*entry)(void *);
} sim_task_t;
enum { SIM_TASKS = 6 };
static sim_task_t tasks[SIM_TASKS]; // app_main/DHT22, GY511, RC522, LD2450, HC-SR04P, MH-Z1911A
static bool co2_active, co2_task_failure;
static pthread_mutex_t scheduler_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t scheduler_changed = PTHREAD_COND_INITIALIZER;
static bool stopping;
static int running = -1;
static int64_t base_us, end_us = 8200000;
static bool gy_missing, dht_missing, task_failure, slow_read;
static bool slow_dht_read, preempt_on_notify, gy_read_error;
static bool rc_active, rc_missing, rc_task_failure, rc_slow_init, rc_slow_read, rc_read_error;
static unsigned rc_inits, rc_count, rc_tx;
static int64_t rc_times[1024], rc_event_times[16];
static const char *rc_scenario;
static bool radar_active, radar_missing, radar_errors, radar_init_failure, radar_task_failure;
static unsigned radar_inits, radar_receives, radar_valid, radar_deinits;
static unsigned radar_snapshots;
static unsigned radar_discards, radar_commits;
static uint32_t radar_interval_ms = 1000;
static bool radar_enabled = true, radar_discard_failure, radar_resume_silence;
static int64_t radar_last_handoff = -1;
static bool radar_disabled_errors, radar_health_saved;
static sensor_status_t radar_disabled_health;
static int64_t radar_resumed_at = -1, radar_first_delay_us;
static bool radar_resume_fragment;
static bool radar_resume_noise;
static unsigned radar_resume_reads;
static unsigned diagnostics_paused, dht_inits, gy_inits;
static bool ultra_active, ultra_missing, ultra_invalid, ultra_resume;
static bool ultra_init_failure, ultra_task_failure, ultra_overrun;
static unsigned ultra_inits, ultra_reads, wifi_starts, mqtt_starts;
static unsigned ultra_snapshots;
static bool ultra_enabled = true, ultra_read_active, ultra_health_saved;
static sensor_status_t ultra_disabled_health;
static int64_t ultra_first_delay_us;
static bool ultra_recover_on_enable;
static unsigned ultra_active_notifications;
static int64_t ultra_times[1024];
static unsigned notify_busy, diagnostics, dht_count, gy_count, dht_tx, gy_tx;
static int64_t dht_times[128], gy_times[512];
static mqtt_config_update_t queue[4];
static unsigned queued, taken;
typedef struct { int64_t at_us; mqtt_config_update_t update; const char *rejection; } input_t;
static input_t inputs[16];
static unsigned input_count, next_input;
typedef struct {
    mqtt_config_update_t update;
    bool applied;
    int64_t at_us;
    const char *reason;
    unsigned read_count;
    unsigned event_count;
    sensor_state_t state;
} ack_t;
static ack_t acks[32];
static unsigned ack_count;

static int task_index(void)
{
    for (int i = 0; i < SIM_TASKS; ++i) {
        if (tasks[i].exists && pthread_equal(tasks[i].thread, pthread_self())) return i;
    }
    assert(false); return -1;
}

static void await_turn(sim_task_t *task)
{
    while (!task->running && !stopping) pthread_cond_wait(&scheduler_changed, &scheduler_lock);
    if (stopping) {
        pthread_mutex_unlock(&scheduler_lock);
        pthread_exit(NULL);
    }
}

static void yield_until(int64_t wake_us, bool notify_wait)
{
    pthread_mutex_lock(&scheduler_lock);
    sim_task_t *task = &tasks[task_index()];
    assert(wake_us > test_now_us || (preempt_on_notify && task_index() == 0 && wake_us == test_now_us));
    task->wake_us = wake_us;
    task->notify_wait = notify_wait;
    task->running = false;
    running = -1;
    pthread_cond_broadcast(&scheduler_changed);
    await_turn(task);
    task->notify_wait = false;
    pthread_mutex_unlock(&scheduler_lock);
}

static void *task_entry(void *arg)
{
    sim_task_t *task = arg;
    pthread_mutex_lock(&scheduler_lock);
    await_turn(task);
    pthread_mutex_unlock(&scheduler_lock);
    if (task == &tasks[0]) app_main();
    else task->entry(NULL);
    assert(false); // Both platform loops must remain alive.
    return NULL;
}

void vTaskDelay(TickType_t ticks) { yield_until(test_now_us + ticks * 10000LL, false); }
void vTaskDelayUntil(TickType_t *last_wake, TickType_t period)
{
    *last_wake += period;
    TickType_t remaining = *last_wake - xTaskGetTickCount();
    vTaskDelay(remaining);
}
TickType_t xTaskGetTickCount(void) { return (TickType_t)(test_now_us / 10000); }
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task) { return 1024; }
BaseType_t xTaskCreate(void (*entry)(void *), const char *name, unsigned stack,
                      void *arg, unsigned priority, TaskHandle_t *handle)
{
    assert(task_index() == 0 && stack == 4096);
    int index = !strcmp(name, "gy511_reader") ? 1 : !strcmp(name, "rc522_reader") ? 2 :
                !strcmp(name, "ld2450_reader") ? 3 : !strcmp(name, "hcsr04p_reader") ? 4 : 5;
    assert(index == 1 ? priority == 4 : priority == 3);
    assert(handle && (index != 5 || !strcmp(name, "mhz1911a_reader")));
    if (index == 1 ? task_failure : index == 2 ? rc_task_failure : index == 3 ?
        (!radar_active || radar_task_failure) : index == 4 ?
        (!ultra_active || ultra_task_failure) : co2_task_failure) return pdFALSE;
    pthread_mutex_lock(&scheduler_lock);
    tasks[index].exists = true;
    tasks[index].entry = entry;
    tasks[index].wake_us = test_now_us;
    if (handle) *handle = &tasks[index];
    assert(pthread_create(&tasks[index].thread, NULL, task_entry, &tasks[index]) == 0);
    pthread_mutex_unlock(&scheduler_lock);
    return pdPASS;
}
void vTaskDelete(TaskHandle_t task)
{
    assert(!task && task_index() >= 2 && task_index() < SIM_TASKS);
    pthread_mutex_lock(&scheduler_lock);
    int index = task_index();
    tasks[index].finished = true; tasks[index].running = false; running = -1;
    pthread_cond_broadcast(&scheduler_changed);
    pthread_mutex_unlock(&scheduler_lock);
    pthread_exit(NULL);
}
BaseType_t xTaskNotify(TaskHandle_t handle, uint32_t value, eNotifyAction action)
{
    assert(task_index() == 0 && (handle == &tasks[1] || handle == &tasks[2] || handle == &tasks[3] || handle == &tasks[4] || handle == &tasks[5]) && action == eSetValueWithoutOverwrite);
    sim_task_t *task = handle;
    assert(!task->finished); // Never notify a deleted initialization-failure task.
    if (task->notified) { notify_busy++; return pdFALSE; }
    task->notified = true;
    task->notification = value;
    if (handle == &tasks[4] && ultra_read_active) ultra_active_notifications++;
    if (task->notify_wait) task->wake_us = test_now_us;
    // Exercise a higher-priority reader committing before app_main removes the
    // FIFO head, as can happen on the target (including across its two cores).
    if (preempt_on_notify && task->notify_wait) yield_until(test_now_us, false);
    return pdPASS;
}
BaseType_t xTaskNotifyWait(uint32_t clear_entry, uint32_t clear_exit,
                           uint32_t *value, TickType_t wait)
{
    assert(task_index() != 0 && clear_entry == 0 && clear_exit == UINT32_MAX);
    sim_task_t *task = &tasks[task_index()];
    if (!task->notified && wait) yield_until(test_now_us + wait * 10000LL, true);
    if (!task->notified) return pdFALSE;
    *value = task->notification;
    task->notified = false;
    return pdTRUE;
}

esp_err_t wifi_manager_start(void) { wifi_starts++; return ESP_OK; }
esp_err_t hcsr04p_sensor_init(void)
{
    assert(task_index() == 4);
    ultra_inits++;
    if (ultra_first_delay_us) vTaskDelay(pdMS_TO_TICKS(ultra_first_delay_us / 1000));
    return ultra_init_failure ? ESP_ERR_NO_MEM : ESP_OK;
}
esp_err_t mqtt_manager_update_hcsr04p_snapshot(const hcsr04p_reading_t *reading, int64_t acquired_us)
{
    assert(task_index() == 4 && acquired_us == test_now_us && ultra_enabled && !ultra_read_active);
    assert(reading->distance_cm == 100 && reading->echo_us == 5830.9f);
    ultra_snapshots++;
    // Transport unavailability must not change any reader's health/schedule.
    return ESP_ERR_INVALID_STATE;
}
void mqtt_manager_commit_hcsr04p_enabled(bool enabled)
{
    assert(task_index() == 4 && !ultra_read_active);
    if (enabled && !ultra_enabled && ultra_recover_on_enable) ultra_missing = false;
    ultra_enabled = enabled;
}
esp_err_t hcsr04p_sensor_read(hcsr04p_reading_t *out)
{
    assert(task_index() == 4 && ultra_reads < sizeof(ultra_times) / sizeof(*ultra_times) && ultra_inits == 1 && ultra_enabled);
    sensor_status_t status;
    assert(sensor_status_get(SENSOR_HCSR04P, &status) && status.initialized && status.state != SENSOR_DISABLED);
    if (ultra_reads == 0) assert(status.read_successes == 0 && status.last_success_us == -1);
    if (ultra_resume && ultra_reads == 3) {
        assert(status.consecutive_failures == 3 && status.state == SENSOR_DEGRADED);
    }
    ultra_times[ultra_reads++] = test_now_us - base_us;
    // Release the scheduler throughout the worst-case capture wait.
    ultra_read_active = true;
    vTaskDelay(ultra_overrun && ultra_reads == 1 ? 35 : 4);
    ultra_read_active = false;
    if (ultra_missing || (ultra_resume && ultra_reads <= 3)) return ESP_ERR_TIMEOUT;
    if (ultra_invalid) return ESP_ERR_INVALID_RESPONSE;
    *out = (hcsr04p_reading_t){.distance_cm = 100, .echo_us = 5830.9f};
    return ESP_OK;
}
esp_err_t ld2450_sensor_init(void)
{ radar_inits++; return radar_init_failure ? ESP_ERR_NO_MEM : ESP_OK; }
esp_err_t ld2450_sensor_deinit(void) { radar_deinits++; return ESP_OK; }
esp_err_t ld2450_sensor_discard_pending(void)
{
    assert(task_index() == 3 && !radar_enabled);
    radar_discards++;
    return radar_discard_failure ? ESP_FAIL : ESP_OK;
}
esp_err_t ld2450_sensor_receive(ld2450_report_t *report, unsigned *frames)
{
    assert(task_index() == 3); radar_receives++;
    sensor_status_t health; assert(sensor_status_get(SENSOR_LD2450, &health));
    if (radar_health_saved) {
        assert(health.state == SENSOR_DISABLED && !radar_enabled);
        assert(health.read_successes == radar_disabled_health.read_successes &&
               health.read_failures == radar_disabled_health.read_failures &&
               health.last_success_us == radar_disabled_health.last_success_us &&
               health.last_error == radar_disabled_health.last_error &&
               health.consecutive_failures == radar_disabled_health.consecutive_failures);
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Real application services configuration while UART blocks.
    *frames = 0;
    if (radar_missing || test_now_us - base_us < radar_first_delay_us) return ESP_OK;
    if (radar_resume_silence && radar_discards) return ESP_OK;
    if (radar_resume_noise && radar_discards) return ESP_ERR_INVALID_RESPONSE;
    if (radar_disabled_errors && !radar_enabled) return ESP_FAIL;
    if (radar_errors && radar_valid) return ESP_FAIL;
    if (radar_resume_fragment && radar_discards && ++radar_resume_reads == 1) return ESP_ERR_INVALID_RESPONSE;
    *report = (ld2450_report_t){0}; // No-target reports still prove communication.
    *frames = 1; radar_valid++;
    return radar_resume_fragment && radar_resume_reads == 2 ? ESP_ERR_INVALID_RESPONSE : ESP_OK;
}
void ld2450_sensor_get_diagnostics(ld2450_diagnostics_t *d)
{ *d = (ld2450_diagnostics_t){.frames = radar_valid}; }
esp_err_t mqtt_manager_update_ld2450_snapshot(const ld2450_report_t *report, int64_t received_us)
{
    assert(task_index() == 3 && report->count == 0 && received_us == test_now_us);
    assert(radar_enabled && sensor_status_can_sample(SENSOR_LD2450));
    if (radar_resumed_at >= 0) assert(received_us > radar_resumed_at);
    radar_snapshots++; radar_last_handoff = received_us;
    return ESP_OK;
}
esp_err_t mqtt_manager_commit_ld2450_config(const mqtt_config_update_t *update)
{
    assert(task_index() == 3 && update->sensor == SENSOR_LD2450);
    if (update->operation == MQTT_CONFIG_ENABLED) {
        if (update->enabled && !radar_enabled) assert(radar_discards && !radar_discard_failure);
        radar_enabled = update->enabled;
    } else {
        assert(radar_enabled && update->interval_ms >= 500 && update->interval_ms <= 10000);
        radar_interval_ms = update->interval_ms;
    }
    radar_commits++;
    return ESP_OK;
}
esp_err_t spi_bus_manager_init(void) { return rc_active ? ESP_OK : ESP_FAIL; }
esp_err_t rc522_sensor_init(void)
{
    rc_inits++;
    if (rc_slow_init) vTaskDelay(pdMS_TO_TICKS(2500));
    return rc_missing ? ESP_FAIL : ESP_OK;
}
esp_err_t rc522_sensor_poll(rc522_uid_t *uid, bool *present)
{
    assert(task_index() == 2 && rc_count < sizeof(rc_times) / sizeof(*rc_times) && sensor_status_can_sample(SENSOR_RC522));
    int64_t now = test_now_us - base_us;
    rc_times[rc_count++] = now;
    if (rc_slow_read && now == 2200000) vTaskDelay(pdMS_TO_TICKS(700));
    if (rc_read_error && now == 2200000) return ESP_ERR_TIMEOUT;
    *uid = (rc522_uid_t){.length = 4, .bytes = {0x5a, 0x2d, 0xfd, 0x03}};
    *present = strcmp(rc_scenario, "rc_idle") != 0;
    if (!strcmp(rc_scenario, "rc_remove") && now >= 4500000 && now <= 6000000) *present = false;
    if (!strcmp(rc_scenario, "rc_partial_idle") && (now == 2700000 || now == 3200000 || now == 4800000)) *present = false;
    if (!strcmp(rc_scenario, "rc_different") && now >= 4000000) uid->bytes[0] = 0xff;
    return ESP_OK;
}
esp_err_t mqtt_manager_publish_rc522_event(const uint8_t *uid, uint8_t length)
{
    assert(task_index() == 2 && sensor_status_can_sample(SENSOR_RC522));
    assert(length == 4 && rc_tx < 16);
    rc_event_times[rc_tx++] = test_now_us - base_us;
    return ESP_OK;
}
esp_err_t mqtt_manager_init(void) { mqtt_starts++; return ESP_OK; }
esp_err_t i2c_bus_manager_init(void) { return ESP_OK; }
void i2c_bus_manager_scan(void) {}
esp_err_t gy511_sensor_init(void) { gy_inits++; return gy_missing ? ESP_FAIL : ESP_OK; }
esp_err_t gy511_sensor_deinit(void) { return ESP_OK; }
esp_err_t dht22_init(gpio_num_t pin) { dht_inits++; return dht_missing ? ESP_ERR_NO_MEM : ESP_OK; }
esp_err_t dht22_read(dht22_data_t *data)
{
    assert(task_index() == 0 && dht_count < 64);
    assert(sensor_status_can_sample(SENSOR_DHT22));
    dht_times[dht_count++] = test_now_us - base_us;
    if (slow_dht_read && dht_count == 1) vTaskDelay(pdMS_TO_TICKS(700));
    *data = (dht22_data_t){25, 50}; return ESP_OK;
}
esp_err_t gy511_sensor_read(gy511_data_t *data)
{
    assert(task_index() == 1 && gy_count < sizeof(gy_times) / sizeof(*gy_times));
    assert(sensor_status_can_sample(SENSOR_GY511));
    gy_times[gy_count++] = test_now_us - base_us;
    if (slow_read && test_now_us - base_us == 2100000) vTaskDelay(pdMS_TO_TICKS(700));
    if (gy_read_error && gy_count == 5) return ESP_ERR_TIMEOUT;
    *data = (gy511_data_t){1, 2, 3, 10, 20, 30}; return ESP_OK;
}
esp_err_t mqtt_manager_publish_dht22(float temperature, float humidity)
{
    assert(temperature == 25 && humidity == 50 && sensor_status_can_sample(SENSOR_DHT22));
    dht_tx++; return ESP_OK;
}
esp_err_t mqtt_manager_publish_gy511(float ax, float ay, float az, float mx, float my, float mz)
{
    assert(ax == 1 && ay == 2 && az == 3 && mx == 10 && my == 20 && mz == 30);
    assert(sensor_status_can_sample(SENSOR_GY511));
    gy_tx++; return ESP_OK;
}
bool mqtt_manager_peek_config_update(mqtt_config_update_t *update)
{
    assert(task_index() == 0);
    if (!queued) return false;
    *update = queue[0]; return true;
}
bool mqtt_manager_take_config_update(mqtt_config_update_t *update)
{
    if (!mqtt_manager_peek_config_update(update)) return false;
    queued--; taken++;
    memmove(queue, queue + 1, queued * sizeof(queue[0]));
    return true;
}
static void co2_check_ack(const mqtt_config_update_t *update, const sensor_status_t *status);
esp_err_t mqtt_manager_publish_config_ack(const mqtt_config_update_t *update)
{
    // In particular, the dispatcher must NEVER ACK a forwarded GY-511 update.
    assert(task_index() == (update->sensor == SENSOR_GY511 ? 1 : update->sensor == SENSOR_RC522 ? 2 : update->sensor == SENSOR_LD2450 ? 3 : update->sensor == SENSOR_HCSR04P ? 4 : update->sensor == SENSOR_MHZ1911A ? 5 : 0));
    if (update->sensor == SENSOR_LD2450) {
        assert(radar_commits > 0);
        assert(update->operation == MQTT_CONFIG_ENABLED ? radar_enabled == update->enabled : radar_interval_ms == update->interval_ms);
    }
    sensor_status_t status;
    assert(sensor_status_get(update->sensor, &status));
    if (update->sensor == SENSOR_MHZ1911A) co2_check_ack(update, &status);
    if (update->sensor == SENSOR_HCSR04P) {
        assert(!ultra_read_active);
        if (update->operation == MQTT_CONFIG_ENABLED) {
            assert(ultra_enabled == update->enabled);
            if (ultra_health_saved) {
                assert(status.read_successes == ultra_disabled_health.read_successes &&
                       status.read_failures == ultra_disabled_health.read_failures &&
                       status.last_success_us == ultra_disabled_health.last_success_us &&
                       status.last_error == ultra_disabled_health.last_error &&
                       status.consecutive_failures == ultra_disabled_health.consecutive_failures);
            }
            ultra_health_saved = !update->enabled;
            if (ultra_health_saved) ultra_disabled_health = status;
        }
    }
    if (update->operation == MQTT_CONFIG_ENABLED) {
        // State must already be committed when the ACK API is called.
        assert(update->enabled ? (update->sensor == SENSOR_MHZ1911A ? status.state != SENSOR_DISABLED :
            sensor_status_can_sample(update->sensor)) : status.state == SENSOR_DISABLED);
        if (update->sensor == SENSOR_LD2450) {
            if (update->enabled && radar_health_saved) {
                assert(status.read_successes == radar_disabled_health.read_successes &&
                       status.read_failures == radar_disabled_health.read_failures &&
                       status.last_success_us == radar_disabled_health.last_success_us &&
                       status.last_error == radar_disabled_health.last_error &&
                       status.consecutive_failures == radar_disabled_health.consecutive_failures);
                assert(status.state == (status.error_stage == SENSOR_STAGE_READ ? SENSOR_DEGRADED : SENSOR_READY));
                radar_health_saved = false; radar_resumed_at = test_now_us;
            } else if (!update->enabled) {
                radar_health_saved = true; radar_disabled_health = status;
            }
        }
    }
    assert(ack_count < 32);
    acks[ack_count++] = (ack_t){.update = *update, .applied = true, .at_us = test_now_us - base_us,
        .read_count = update->sensor == SENSOR_DHT22 ? dht_count : update->sensor == SENSOR_GY511 ? gy_count : update->sensor == SENSOR_HCSR04P ? ultra_reads : rc_count,
        .event_count = rc_tx, .state = status.state};
    return ESP_OK;
}
esp_err_t mqtt_manager_reject_config(const mqtt_config_update_t *update, const char *reason)
{
    sensor_status_t status;
    assert(sensor_status_get(update->sensor, &status));
    bool failed_resume = update->sensor == SENSOR_LD2450 && radar_discard_failure &&
                         update->operation == MQTT_CONFIG_ENABLED && update->enabled;
    assert(!strcmp(reason, status.state == SENSOR_DISABLED && !failed_resume ? "sensor_disabled" : "sensor_unavailable"));
    assert(!sensor_status_can_sample(update->sensor));
    assert(ack_count < 32);
    acks[ack_count++] = (ack_t){.update = *update, .applied = false, .at_us = test_now_us - base_us,
        .reason = reason, .read_count = update->sensor == SENSOR_DHT22 ? dht_count : update->sensor == SENSOR_GY511 ? gy_count : rc_count,
        .event_count = rc_tx,
        .state = status.state};
    return ESP_OK;
}
void mqtt_manager_log_diagnostics(void)
{
    diagnostics++;
    sensor_status_t dht, gy;
    assert(sensor_status_get(SENSOR_DHT22, &dht) && sensor_status_get(SENSOR_GY511, &gy));
    if (dht.state == SENSOR_DISABLED && gy.state == SENSOR_DISABLED) diagnostics_paused++;
}

static void input(int64_t at_us, sensor_id_t sensor, uint32_t interval)
{
    assert(input_count < 16);
    inputs[input_count++] = (input_t){.at_us = at_us, .update = {.sensor = sensor, .interval_ms = interval}};
}

static void gate_input(int64_t at_us, sensor_id_t sensor, bool enabled)
{
    assert(input_count < 16);
    inputs[input_count++] = (input_t){.at_us = at_us,
        .update = {.sensor = sensor, .operation = MQTT_CONFIG_ENABLED, .enabled = enabled}};
}

static void verify_commands(void)
{
    sensor_status_t rc522;
    assert(sensor_status_get(SENSOR_RC522, &rc522));
    if (!rc_active) assert(rc522.state == SENSOR_UNAVAILABLE);
    assert(!queued && taken == input_count && ack_count == input_count);
    assert(dht_inits == 1 && gy_inits == 1); // Enable never reinitializes either driver.
    assert(dht_tx == dht_count && gy_tx == gy_count - (gy_read_error ? 1U : 0U) && diagnostics > 0);
    // FIFO application/ACK order within each sensor, including rejections at
    // the owner. Cross-sensor ACK order need not match the original FIFO.
    for (int sensor = SENSOR_DHT22; sensor <= SENSOR_MHZ1911A; ++sensor) {
        unsigned cursor = 0;
        for (unsigned i = 0; i < input_count; ++i) {
            const input_t *in = &inputs[i];
            if (in->update.sensor != (sensor_id_t)sensor) continue;
            while (cursor < ack_count && acks[cursor].update.sensor != (sensor_id_t)sensor) cursor++;
            assert(cursor < ack_count);
            const ack_t *ack = &acks[cursor++];
            assert(ack->update.operation == in->update.operation);
            if (in->update.operation == MQTT_CONFIG_INTERVAL) assert(ack->update.interval_ms == in->update.interval_ms);
            else assert(ack->update.enabled == in->update.enabled);
            assert(ack->at_us >= in->at_us && ack->applied == (in->rejection == NULL));
            if (in->rejection) assert(ack->reason && !strcmp(ack->reason, in->rejection));
        }
    }
}

static void simulate(void)
{
    pthread_mutex_lock(&scheduler_lock);
    tasks[0].exists = true;
    tasks[0].wake_us = base_us;
    assert(pthread_create(&tasks[0].thread, NULL, task_entry, &tasks[0]) == 0);
    while (true) {
        while (running >= 0) pthread_cond_wait(&scheduler_changed, &scheduler_lock);
        int64_t next_us = base_us + end_us;
        for (int i = 0; i < SIM_TASKS; ++i) {
            if (tasks[i].exists && !tasks[i].finished && tasks[i].wake_us < next_us) next_us = tasks[i].wake_us;
        }
        if (next_input < input_count && base_us + inputs[next_input].at_us < next_us) {
            next_us = base_us + inputs[next_input].at_us;
        }
        if (next_us >= base_us + end_us) break;
        test_now_us = next_us;
        while (next_input < input_count && base_us + inputs[next_input].at_us <= test_now_us) {
            assert(queued < 4); // Queue-full validation is covered by the actual MQTT tests.
            queue[queued++] = inputs[next_input++].update;
        }
        const int priority_order[] = {1, 2, 3, 4, 5, 0};
        for (unsigned p = 0; p < SIM_TASKS; ++p) {
            int i = priority_order[p];
            if (tasks[i].exists && !tasks[i].finished && tasks[i].wake_us <= test_now_us) {
                tasks[i].running = true; running = i;
                pthread_cond_broadcast(&scheduler_changed);
                break;
            }
        }
    }
    stopping = true;
    pthread_cond_broadcast(&scheduler_changed);
    pthread_mutex_unlock(&scheduler_lock);
    for (int i = 0; i < SIM_TASKS; ++i) if (tasks[i].exists) pthread_join(tasks[i].thread, NULL);
}

#include "../mhz1911a_host/test_runtime.inc"
#include "../mhz1911a_host/test_config_runtime.inc"

static void test_gates(const char *scenario)
{
    bool gy_pause = !strcmp(scenario, "gate_gy") || !strcmp(scenario, "gate_wrap");
    bool dht_pause = !strcmp(scenario, "gate_dht");
    bool both = !strcmp(scenario, "gate_both");
    bool burst = !strcmp(scenario, "gate_burst") || !strcmp(scenario, "gate_preempt");
    preempt_on_notify = !strcmp(scenario, "gate_preempt");
    slow_read = !strcmp(scenario, "gate_read");
    slow_dht_read = !strcmp(scenario, "gate_dht_read");
    gy_missing = !strcmp(scenario, "gate_gy_unavailable");
    task_failure = !strcmp(scenario, "gate_task_unavailable");
    dht_missing = !strcmp(scenario, "gate_dht_unavailable");
    gy_read_error = !strcmp(scenario, "gate_degraded");
    if (!strcmp(scenario, "gate_wrap")) base_us = ((int64_t)UINT32_MAX - 250) * 10000;
    test_now_us = base_us;
    if (gy_pause || dht_pause) {
        sensor_id_t sensor = gy_pause ? SENSOR_GY511 : SENSOR_DHT22;
        input(2300000, sensor, gy_pause ? 1000 : 5000);
        gate_input(3300000, sensor, false);
        gate_input(3500000, sensor, false); // Idempotent disable.
        input(3700000, sensor, 2000);
        inputs[input_count - 1].rejection = "sensor_disabled";
        gate_input(4300000, sensor, true);
        gate_input(4400000, sensor, true); // Must not postpone the deadline.
        if (dht_pause) end_us = 15000000;
    } else if (both) {
        gate_input(2300000, SENSOR_DHT22, false);
        gate_input(2300000, SENSOR_GY511, false);
        gate_input(10000000, SENSOR_GY511, false);
        gate_input(10000000, SENSOR_DHT22, false);
        input(20000000, SENSOR_GY511, 1000);
        inputs[input_count - 1].rejection = "sensor_disabled";
        input(21000000, SENSOR_DHT22, 5000);
        inputs[input_count - 1].rejection = "sensor_disabled";
        gate_input(122000000, SENSOR_DHT22, true);
        gate_input(122000000, SENSOR_GY511, true);
        end_us = 124200000; // Cross two summary deadlines with no acquisitions.
    } else if (burst) {
        gate_input(2300000, SENSOR_GY511, false);
        input(2300000, SENSOR_GY511, 1000);
        inputs[input_count - 1].rejection = "sensor_disabled";
        gate_input(2300000, SENSOR_DHT22, false);
        gate_input(2300000, SENSOR_GY511, true);
        input(2800000, SENSOR_GY511, 2000);
        gate_input(2800000, SENSOR_GY511, false);
        gate_input(2800000, SENSOR_DHT22, true);
        gate_input(2800000, SENSOR_GY511, true);
    } else if (gy_missing || task_failure || dht_missing) {
        sensor_id_t sensor = dht_missing ? SENSOR_DHT22 : SENSOR_GY511;
        gate_input(0, sensor, true); // Includes commands arriving before failed startup.
        gate_input(2300000, sensor, false);
        input(2400000, sensor, 5000);
        for (unsigned i = 0; i < input_count; ++i) inputs[i].rejection = "sensor_unavailable";
    } else {
        assert(slow_read || slow_dht_read || gy_read_error);
        sensor_id_t sensor = slow_dht_read ? SENSOR_DHT22 : SENSOR_GY511;
        gate_input(2300000, sensor, false);
        gate_input(3300000, sensor, true);
        gate_input(3400000, sensor, true);
    }
    simulate();
    verify_commands();

    // No reads between a committed disable and the following committed enable,
    // even for duplicate requests. Already submitted telemetry is not retracted.
    for (int sensor = SENSOR_DHT22; sensor <= SENSOR_GY511; ++sensor) {
        bool enabled = true;
        unsigned paused_count = 0;
        for (unsigned i = 0; i < ack_count; ++i) {
            const ack_t *ack = &acks[i];
            if (!ack->applied || ack->update.sensor != (sensor_id_t)sensor ||
                ack->update.operation != MQTT_CONFIG_ENABLED) continue;
            if (!enabled) assert(ack->read_count == paused_count);
            if (!ack->update.enabled) paused_count = ack->read_count;
            enabled = ack->update.enabled;
        }
        if (!enabled) assert((sensor == SENSOR_DHT22 ? dht_count : gy_count) == paused_count);
    }
    if (gy_pause) {
        assert(gy_count == 9);
        for (unsigned i = 0; i < 5; ++i) assert(gy_times[i] == 100000 + i * 500000LL);
        assert(gy_times[5] == 3100000);
        for (unsigned i = 6; i < gy_count; ++i) assert(gy_times[i] == 5300000 + (i - 6) * 1000000LL);
    } else if (dht_pause) {
        assert(dht_count == 3 && dht_times[0] == 2000000 && dht_times[1] == 9300000 && dht_times[2] == 14300000);
    } else if (both) {
        assert(dht_count == 2 && dht_times[0] == 2000000 && dht_times[1] == 124000000);
        assert(gy_count == 9 && gy_times[5] == 122500000 && gy_times[8] == 124000000);
        assert(diagnostics_paused > 1100 && test_log_calls >= 6);
    } else if (burst) {
        if (!preempt_on_notify) assert(notify_busy > 0);
        for (int sensor = SENSOR_DHT22; sensor <= SENSOR_GY511; ++sensor) {
            const ack_t *last_enable = NULL;
            for (unsigned i = 0; i < ack_count; ++i) {
                if (acks[i].update.sensor == (sensor_id_t)sensor && acks[i].applied &&
                    acks[i].update.operation == MQTT_CONFIG_ENABLED && acks[i].update.enabled) last_enable = &acks[i];
            }
            const int64_t *times = sensor == SENSOR_DHT22 ? dht_times : gy_times;
            unsigned count = sensor == SENSOR_DHT22 ? dht_count : gy_count;
            assert(last_enable && count >= last_enable->read_count + 2);
            for (unsigned i = last_enable->read_count; i < count; ++i) {
                assert(times[i] == last_enable->at_us + (i - last_enable->read_count + 1) * 2000000LL);
            }
        }
    } else if (gy_missing || task_failure || dht_missing) {
        sensor_status_t status;
        assert(sensor_status_get(dht_missing ? SENSOR_DHT22 : SENSOR_GY511, &status));
        assert(status.state == SENSOR_UNAVAILABLE);
        assert(dht_missing ? !dht_count : !gy_count);
    } else if (slow_dht_read) {
        assert(acks[0].at_us == 2700000); // Commit follows the in-flight read and publication.
        assert(dht_count == 3 && dht_times[0] == 2000000 && dht_times[1] == 5300000 && dht_times[2] == 7300000);
    } else {
        assert(gy_count == 14);
        for (unsigned i = 5; i < gy_count; ++i) assert(gy_times[i] == 3800000 + (i - 5) * 500000LL);
        if (slow_read) assert(acks[0].at_us == 2800000);
        if (gy_read_error) {
            assert(acks[0].state == SENSOR_DISABLED && acks[1].state == SENSOR_DEGRADED);
            sensor_status_t status;
            assert(sensor_status_get(SENSOR_GY511, &status) && status.state == SENSOR_READY);
            assert(status.read_failures == 1 && status.consecutive_failures == 0);
        }
    }
    // The unrelated sensor keeps its default schedule throughout each pause.
    if (!dht_pause && !both && !burst && !slow_dht_read && !dht_missing) {
        assert(dht_count == 4);
        for (unsigned i = 0; i < dht_count; ++i) assert(dht_times[i] == 2000000 + i * 2000000LL);
    }
    if (dht_pause || slow_dht_read || dht_missing) {
        assert(gy_count == (dht_pause ? 30U : 17U));
        for (unsigned i = 0; i < gy_count; ++i) assert(gy_times[i] == 100000 + i * 500000LL);
    }
    printf("PASS: enable/disable scenario %s\n", scenario);
}

static void test_rc522_config(const char *scenario)
{
    rc_scenario = scenario;
    rc_active = strcmp(scenario, "rc_bus_failure") != 0;
    rc_missing = !strcmp(scenario, "rc_unavailable");
    rc_task_failure = !strcmp(scenario, "rc_task_failure");
    rc_slow_init = !strcmp(scenario, "rc_uninitialized");
    rc_slow_read = !strcmp(scenario, "rc_during_read");
    rc_read_error = !strcmp(scenario, "rc_degraded");
    preempt_on_notify = !strcmp(scenario, "rc_preempt");
    bool unavailable = !rc_active || rc_missing || rc_task_failure;
    bool gates = !strcmp(scenario, "rc_gates") || !strcmp(scenario, "rc_remove") ||
                 !strcmp(scenario, "rc_different") || !strcmp(scenario, "rc_partial_idle") ||
                 !strcmp(scenario, "rc_wrap");
    bool burst = !strcmp(scenario, "rc_burst") || preempt_on_notify;
    bool all_config = !strcmp(scenario, "rc_all_config");
    if (!strcmp(scenario, "rc_wrap")) base_us = ((int64_t)UINT32_MAX - 250) * 10000;
    test_now_us = base_us;
    if (unavailable) {
        gate_input(0, SENSOR_RC522, true);
        gate_input(2300000, SENSOR_RC522, false);
        input(2400000, SENSOR_RC522, 500);
        for (unsigned i = 0; i < input_count; ++i) inputs[i].rejection = "sensor_unavailable";
    } else if (rc_slow_init) {
        input(2000000, SENSOR_RC522, 500);
        inputs[input_count - 1].rejection = "sensor_unavailable";
        gate_input(3000000, SENSOR_RC522, true);
    } else if (gates) {
        input(2300000, SENSOR_RC522, 500);
        gate_input(3300000, SENSOR_RC522, false);
        gate_input(3500000, SENSOR_RC522, false);
        input(3700000, SENSOR_RC522, 1000);
        inputs[input_count - 1].rejection = "sensor_disabled";
        gate_input(4300000, SENSOR_RC522, true);
        gate_input(4400000, SENSOR_RC522, true);
    } else if (burst) {
        gate_input(2300000, SENSOR_RC522, false);
        input(2300000, SENSOR_RC522, 500);
        inputs[input_count - 1].rejection = "sensor_disabled";
        gate_input(2300000, SENSOR_RC522, true);
        input(2300000, SENSOR_RC522, 1000);
        input(2800000, SENSOR_RC522, 2000);
        gate_input(2800000, SENSOR_RC522, false);
        gate_input(2800000, SENSOR_RC522, true);
        input(2800000, SENSOR_RC522, 500);
    } else if (rc_slow_read || rc_read_error) {
        gate_input(2300000, SENSOR_RC522, false);
        gate_input(3300000, SENSOR_RC522, true);
        gate_input(3400000, SENSOR_RC522, true);
    } else if (!strcmp(scenario, "rc_paused")) {
        gate_input(2300000, SENSOR_RC522, false);
        gate_input(5000000, SENSOR_RC522, false);
    } else if (all_config) {
        input(2300000, SENSOR_RC522, 500);
        input(2300000, SENSOR_DHT22, 5000);
        input(2300000, SENSOR_GY511, 1000);
        gate_input(2300000, SENSOR_RC522, false);
        gate_input(3100000, SENSOR_RC522, true);
        gate_input(3500000, SENSOR_GY511, false);
        gate_input(4000000, SENSOR_DHT22, false);
        gate_input(4300000, SENSOR_GY511, true);
        gate_input(4500000, SENSOR_DHT22, true);
        input(4700000, SENSOR_RC522, 200);
    } else if (!strcmp(scenario, "rc_flood")) {
        for (unsigned i = 0; i < 16; ++i) {
            if (i % 2) gate_input(2300000 + i * 100000, SENSOR_RC522, true);
            else input(2300000 + i * 100000, SENSOR_RC522, 200);
        }
    } else if (!strcmp(scenario, "rc_interval")) {
        input(2300000, SENSOR_RC522, 500);
        input(3400000, SENSOR_RC522, 1000);
        input(5400000, SENSOR_RC522, 200);
    } else if (!strcmp(scenario, "rc_rounding")) {
        input(2300000, SENSOR_RC522, 201);
    } else if (!strcmp(scenario, "rc_shorten")) {
        input(2300000, SENSOR_RC522, 2000);
        input(2600000, SENSOR_RC522, 200);
    } else assert(!strcmp(scenario, "rc_default") || !strcmp(scenario, "rc_idle"));

    simulate();
    verify_commands();
    assert(rc_inits == (!rc_active || rc_task_failure ? 0U : 1U));
    sensor_status_t status;
    assert(sensor_status_get(SENSOR_RC522, &status));
    if (unavailable) {
        assert(status.state == SENSOR_UNAVAILABLE && !rc_count && !rc_tx);
        assert(!rc522_reader_task_handle());
    } else {
        assert(rc522_reader_task_handle() == &tasks[2]);
        assert(status.read_failures == (rc_read_error ? 1U : 0U));
        bool enabled = true;
        unsigned paused_reads = 0, paused_events = 0;
        for (unsigned i = 0; i < ack_count; ++i) {
            const ack_t *ack = &acks[i];
            if (!ack->applied || ack->update.sensor != SENSOR_RC522 ||
                ack->update.operation != MQTT_CONFIG_ENABLED) continue;
            if (!enabled) {
                assert(ack->read_count == paused_reads && ack->event_count == paused_events);
            }
            if (!ack->update.enabled) {
                paused_reads = ack->read_count;
                paused_events = ack->event_count;
            }
            enabled = ack->update.enabled;
        }
        if (!enabled) assert(rc_count == paused_reads && rc_tx == paused_events && status.state == SENSOR_DISABLED);
        else assert(status.state == SENSOR_READY);
        unsigned expected_events = !strcmp(scenario, "rc_idle") ? 0 :
            (!strcmp(scenario, "rc_remove") || !strcmp(scenario, "rc_different")) ? 2 : 1;
        assert(rc_tx == expected_events); // Pause/resume alone never fabricates a presentation.
        if (!strcmp(scenario, "rc_idle")) assert(!status.read_successes && status.last_success_us == -1);
        if (!strcmp(scenario, "rc_remove")) assert(rc_event_times[1] == 6300000);
        if (!strcmp(scenario, "rc_different")) assert(rc_event_times[1] == 4800000);
        if (gates) {
            for (unsigned i = 0; i < 12; ++i) assert(rc_times[i] == i * 200000LL);
            assert(rc_times[12] == 2700000 && rc_times[13] == 3200000);
            for (unsigned i = 14; i < rc_count; ++i) assert(rc_times[i] == 4800000 + (i - 14) * 500000LL);
        } else if (!strcmp(scenario, "rc_interval")) {
            assert(rc_times[12] == 2700000 && rc_times[13] == 3200000 &&
                   rc_times[14] == 4200000 && rc_times[15] == 5200000 && rc_times[16] == 5400000);
            for (unsigned i = 17; i < rc_count; ++i) assert(rc_times[i] - rc_times[i - 1] == 200000);
        } else if (!strcmp(scenario, "rc_rounding")) {
            for (unsigned i = 12; i < rc_count; ++i) assert(rc_times[i] == 2410000 + (i - 12) * 210000LL);
        } else if (!strcmp(scenario, "rc_shorten")) {
            assert(acks[1].at_us == 2600000 && rc_times[12] == 2600000);
            for (unsigned i = 13; i < rc_count; ++i) assert(rc_times[i] - rc_times[i - 1] == 200000);
        } else if (burst) {
            if (!preempt_on_notify) assert(notify_busy > 0);
            const ack_t *last = &acks[ack_count - 1];
            assert(last->update.interval_ms == 500 && rc_count > last->read_count + 1);
            for (unsigned i = last->read_count + 1; i < rc_count; ++i) assert(rc_times[i] - rc_times[i - 1] == 500000);
        } else if (rc_slow_read || rc_read_error) {
            assert(acks[0].at_us == (rc_slow_read ? 2900000 : 2300000));
            assert(rc_times[12] == 3500000);
            if (rc_read_error) assert(acks[1].state == SENSOR_DEGRADED && status.consecutive_failures == 0);
        } else if (rc_slow_init) {
            assert(!acks[0].applied && acks[0].at_us == 2000000);
            for (unsigned i = 0; i < rc_count; ++i) assert(rc_times[i] == 2500000 + i * 200000LL);
        } else if (!all_config) {
            for (unsigned i = 0; i < rc_count; ++i) assert(rc_times[i] == i * 200000LL);
        }
    }
    if (!all_config) {
        assert(dht_count == 4 && gy_count == 17);
        for (unsigned i = 0; i < dht_count; ++i) assert(dht_times[i] == 2000000 + i * 2000000LL);
        for (unsigned i = 0; i < gy_count; ++i) assert(gy_times[i] == 100000 + i * 500000LL);
    } else assert(dht_count > 0 && gy_count > 0 && rc_count > 0);
    printf("PASS: RC522 runtime configuration scenario %s\n", scenario);
}

static void test_radar(const char *scenario)
{
    radar_active = rc_active = true;
    rc_scenario = "rc_default";
    radar_missing = !strcmp(scenario, "radar_missing");
    radar_errors = !strcmp(scenario, "radar_errors");
    radar_init_failure = !strcmp(scenario, "radar_init");
    radar_task_failure = !strcmp(scenario, "radar_task");
    input(2350000, SENSOR_DHT22, 5000);
    input(2350000, SENSOR_GY511, 1000);
    input(2350000, SENSOR_RC522, 500);
    simulate();
    verify_commands();
    assert(dht_count == 2 && dht_times[0] == 2000000 && dht_times[1] == 7000000);
    assert(gy_count == 11 && rc_count == 24 && rc_tx == 1);
    for (unsigned i = 0; i < gy_count; ++i) {
        assert(gy_times[i] == (i < 5 ? 100000 + i * 500000LL : 3100000 + (i - 5) * 1000000LL));
    }
    for (unsigned i = 0; i < rc_count; ++i) {
        assert(rc_times[i] == (i < 13 ? i * 200000LL : 2900000 + (i - 13) * 500000LL));
    }
    // All three settings commit inside the radar's 2.31..2.41s receive wait.
    for (unsigned i = 0; i < ack_count; ++i) assert(acks[i].at_us == 2400000 && acks[i].applied);
    assert(diagnostics == dht_count && radar_inits == (radar_task_failure ? 0U : 1U));
    sensor_status_t s;
    assert(sensor_status_get(SENSOR_LD2450, &s));
    if (radar_task_failure || radar_init_failure || radar_missing) {
        assert(s.state == SENSOR_UNAVAILABLE && !s.initialized && !radar_valid);
        assert(s.error_stage == (radar_task_failure ? SENSOR_STAGE_TASK : SENSOR_STAGE_INIT));
        assert(radar_deinits == (radar_missing ? 1U : 0U));
    } else if (radar_errors) {
        assert(s.state == SENSOR_DEGRADED && s.read_successes == 1 && s.read_failures > 50);
    } else assert(s.state == SENSOR_READY && s.read_successes > 70 && !s.read_failures);
    for (sensor_id_t id = SENSOR_DHT22; id <= SENSOR_RC522; ++id) {
        assert(sensor_status_get(id, &s) && s.state == SENSOR_READY && !s.read_failures);
    }
    printf("PASS: LD2450 four-sensor isolation scenario %s\n", scenario);
}

static void test_radar_config(const char *scenario)
{
    radar_active = rc_active = true;
    rc_scenario = "rc_default";
    radar_missing = !strcmp(scenario, "radar_cfg_missing");
    radar_init_failure = !strcmp(scenario, "radar_cfg_init");
    radar_task_failure = !strcmp(scenario, "radar_cfg_task");
    radar_discard_failure = !strcmp(scenario, "radar_cfg_flush");
    radar_resume_silence = !strcmp(scenario, "radar_cfg_silence");
    radar_errors = !strcmp(scenario, "radar_cfg_history");
    radar_disabled_errors = !strcmp(scenario, "radar_cfg_gate");
    radar_resume_fragment = !strcmp(scenario, "radar_cfg_fragment");
    radar_resume_noise = !strcmp(scenario, "radar_cfg_noise");
    bool unavailable = radar_missing || radar_init_failure || radar_task_failure;
    bool interval = !strcmp(scenario, "radar_cfg_interval");
    bool burst = !strcmp(scenario, "radar_cfg_burst");
    bool paused = !strcmp(scenario, "radar_cfg_disabled");
    if (!strcmp(scenario, "radar_cfg_wrap")) base_us = ((int64_t)UINT32_MAX - 30) * 10000;
    test_now_us = base_us;
    if (!strcmp(scenario, "radar_cfg_uninitialized")) {
        radar_first_delay_us = 4000000;
        gate_input(0, SENSOR_LD2450, true);
        inputs[input_count-1].rejection = "sensor_unavailable";
    }
    input(2350000, SENSOR_DHT22, 5000);
    input(2350000, SENSOR_GY511, 1000);
    input(2350000, SENSOR_RC522, 500);
    if (unavailable) {
        input(2750000, SENSOR_LD2450, 2000); inputs[input_count-1].rejection = "sensor_unavailable";
        gate_input(5500000, SENSOR_LD2450, false); inputs[input_count-1].rejection = "sensor_unavailable";
        gate_input(6000000, SENSOR_LD2450, true); inputs[input_count-1].rejection = "sensor_unavailable";
    } else if (radar_first_delay_us) {
        input(4500000, SENSOR_LD2450, 500);
    } else if (interval) {
        input(2750000, SENSOR_LD2450, 2000);
        input(4500000, SENSOR_LD2450, 500);
        input(6000000, SENSOR_LD2450, 10000);
    } else if (burst) {
        gate_input(3000000, SENSOR_LD2450, false);
        input(3000000, SENSOR_LD2450, 500); inputs[input_count-1].rejection = "sensor_disabled";
        gate_input(3000000, SENSOR_LD2450, true);
        input(3000000, SENSOR_LD2450, 10000);
    } else {
        input(2750000, SENSOR_LD2450, 2000);
        gate_input(3000000, SENSOR_LD2450, false);
        input(3200000, SENSOR_LD2450, 500); inputs[input_count-1].rejection = "sensor_disabled";
        gate_input(3500000, SENSOR_LD2450, false); // Repeated disable preserves history.
        if (!paused) {
            gate_input(5500000, SENSOR_LD2450, true);
            if (radar_discard_failure) inputs[input_count-1].rejection = "sensor_unavailable";
            gate_input(5800000, SENSOR_LD2450, true); // Idempotent enable doesn't postpone timeout.
            if (radar_discard_failure) inputs[input_count-1].rejection = "sensor_unavailable";
        }
    }
    simulate(); verify_commands();
    assert(dht_count == 2 && gy_count == 11 && rc_count == 24 && rc_tx == 1);
    assert(radar_inits == (radar_task_failure ? 0U : 1U));
    sensor_status_t s; assert(sensor_status_get(SENSOR_LD2450, &s));
    if (unavailable) {
        assert(s.state == SENSOR_UNAVAILABLE && !radar_commits && !radar_snapshots);
        assert(ld2450_reader_task_handle() == NULL && radar_discards == 0);
    } else {
        assert(ld2450_reader_task_handle() == &tasks[3] && radar_deinits == 0);
        assert(radar_receives > 70); // Disabled radar still drains at the same cadence.
        if (paused || radar_discard_failure) {
            assert(s.state == SENSOR_DISABLED && !radar_enabled && radar_health_saved);
            assert(s.read_failures == 0 && s.last_success_us == radar_disabled_health.last_success_us);
            assert(radar_snapshots == s.read_successes && radar_interval_ms == 2000);
            assert(radar_discards == (paused ? 0U : 2U));
        } else if (radar_resume_silence || radar_errors || radar_resume_noise) {
            assert(s.state == SENSOR_DEGRADED && s.read_failures > 0 && radar_discards == 1);
            assert(s.last_success_us < radar_resumed_at && radar_last_handoff == s.last_success_us);
            if (radar_resume_silence) assert(s.read_failures == 1); // New 2 s grace after enable.
            if (radar_resume_noise) assert(s.read_failures > 4); // Resync grace cannot reopen after timeout.
        } else {
            assert(s.state == SENSOR_READY && !s.read_failures && radar_enabled);
            assert(radar_interval_ms == (interval || burst ? 10000U : radar_first_delay_us ? 500U : 2000U));
            assert(radar_discards == (interval || radar_first_delay_us ? 0U : 1U));
        }
        if (burst) assert(notify_busy > 0);
        for (unsigned i = 0; i < ack_count; ++i) {
            if (acks[i].update.sensor == SENSOR_LD2450 && acks[i].applied) {
                // Read begins at N*110 ms; commit is between batches, never
                // within the blocking receive or by the MQTT/application callback.
                assert(acks[i].at_us % 110000 == 0);
            }
        }
    }
    for (sensor_id_t id=SENSOR_DHT22; id<=SENSOR_RC522; ++id) {
        assert(sensor_status_get(id,&s) && s.state==SENSOR_READY && !s.read_failures);
    }
    printf("PASS: LD2450 owner configuration, continuous draining and isolation %s\n", scenario);
}

#include "test_hcsr04p_config.c"

int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *scenario = argv[1];
    if (!strncmp(scenario, "co2_config_", 11)) { test_co2_controls(scenario); return 0; }
    if (!strncmp(scenario, "co2_", 4)) { test_co2_isolation(scenario); return 0; }
    if (!strncmp(scenario, "ultra_cfg_", 10)) { test_hcsr04p_config(scenario); return 0; }
    if (!strncmp(scenario, "ultra_", 6)) {
        ultra_active = radar_active = rc_active = true;
        rc_scenario = "rc_default";
        ultra_missing = !strcmp(scenario, "ultra_missing");
        ultra_invalid = !strcmp(scenario, "ultra_invalid");
        ultra_resume = !strcmp(scenario, "ultra_resume");
        ultra_init_failure = !strcmp(scenario, "ultra_init");
        ultra_task_failure = !strcmp(scenario, "ultra_task");
        ultra_overrun = !strcmp(scenario, "ultra_overrun");
        end_us = 8200000;
        if (!strcmp(scenario, "ultra_wrap")) base_us = ((int64_t)UINT32_MAX - 30) * 10000;
        test_now_us = base_us;
        input(2300000, SENSOR_GY511, 1000);
        input(3400000, SENSOR_GY511, 500);
        input(4500000, SENSOR_RC522, 300);
        input(5500000, SENSOR_LD2450, 2000);
        simulate();
        verify_commands();
        assert(wifi_starts == 1 && mqtt_starts == 1);
        assert(dht_count == 4 && diagnostics == dht_count && gy_count > 10 && rc_count > 20);
        assert(radar_snapshots > 50 && radar_inits == 1);
        for (sensor_id_t id = SENSOR_DHT22; id <= SENSOR_LD2450; ++id) {
            sensor_status_t status;
            assert(sensor_status_get(id, &status) && status.state == SENSOR_READY && !status.read_failures);
        }
        sensor_status_t status;
        assert(sensor_status_get(SENSOR_HCSR04P, &status));
        assert(ultra_snapshots == status.read_successes);
        assert(ultra_inits == (ultra_task_failure ? 0U : 1U));
        if (ultra_init_failure || ultra_task_failure) {
            assert(!ultra_reads && status.state == SENSOR_UNAVAILABLE && !status.initialized);
            assert(status.error_stage == (ultra_init_failure ? SENSOR_STAGE_INIT : SENSOR_STAGE_TASK));
            assert(!status.read_failures && status.last_success_us == -1);
        } else {
            assert(ultra_reads >= 39);
            for (unsigned i = 1; i < ultra_reads; ++i) {
                assert(ultra_times[i] - ultra_times[i-1] == (ultra_overrun && i == 1 ? 550000 : 200000));
            }
            if (ultra_missing || ultra_invalid) {
                assert(status.state == SENSOR_DEGRADED && status.read_failures >= 39);
                assert(status.consecutive_failures == status.read_failures && !status.read_successes);
                assert(status.last_error == (ultra_missing ? ESP_ERR_TIMEOUT : ESP_ERR_INVALID_RESPONSE));
                assert(status.last_success_us == -1);
            } else {
                assert(status.state == SENSOR_READY && !status.consecutive_failures);
                assert(status.read_failures == (ultra_resume ? 3U : 0U) && status.read_successes >= 36);
            }
        }
        printf("PASS: HC-SR04P scheduling, configuration and five-sensor isolation %s\n", scenario);
        return 0;
    }
    if (!strncmp(scenario, "radar_cfg_", 10)) { test_radar_config(scenario); return 0; }
    if (!strncmp(scenario, "radar_", 6)) { test_radar(scenario); return 0; }
    if (!strncmp(scenario, "rc_", 3)) { test_rc522_config(scenario); return 0; }
    if (!strncmp(scenario, "gate_", 5)) { test_gates(scenario); return 0; }
    bool changes = !strcmp(scenario, "changes") || !strcmp(scenario, "mixed") ||
                   !strcmp(scenario, "dht_unavailable");
    bool burst = !strcmp(scenario, "burst");
    bool shorten = !strcmp(scenario, "shorten");
    bool rounding = !strcmp(scenario, "rounding");
    gy_missing = !strcmp(scenario, "gy_unavailable");
    task_failure = !strcmp(scenario, "gy_task_failure");
    dht_missing = !strcmp(scenario, "dht_unavailable");
    slow_read = !strcmp(scenario, "during_read");
    if (!strcmp(scenario, "tick_wrap")) base_us = ((int64_t)UINT32_MAX - 30) * 10000;
    test_now_us = base_us;
    if (changes) {
        input(2300000, SENSOR_GY511, 1000);
        input(3400000, SENSOR_GY511, 2000);
        if (!strcmp(scenario, "mixed")) { input(4500000, SENSOR_DHT22, 5000); end_us = 10200000; }
        input(5400000, SENSOR_GY511, 500);
    } else if (burst) {
        input(2300000, SENSOR_GY511, 1000);
        input(2300000, SENSOR_GY511, 2000);
        input(2300000, SENSOR_DHT22, 5000);
        input(2300000, SENSOR_GY511, 500);
    } else if (gy_missing || task_failure) {
        input(0, SENSOR_GY511, 1000); // Pending even before initialization/task failure.
        inputs[input_count - 1].rejection = "sensor_unavailable";
        input(2300000, SENSOR_DHT22, 5000);
    } else if (slow_read) {
        input(2300000, SENSOR_GY511, 1000);
    } else if (shorten) {
        input(2300000, SENSOR_GY511, 10000);
        input(2600000, SENSOR_GY511, 500);
    } else if (rounding) {
        input(2300000, SENSOR_GY511, 501);
    }
    simulate();
    verify_commands();
    if (dht_missing) assert(dht_count == 0);
    else if (!strcmp(scenario, "mixed")) {
        assert(dht_count == 3 && dht_times[0] == 2000000 && dht_times[1] == 4000000 && dht_times[2] == 9000000);
    } else if (burst || gy_missing || task_failure) {
        assert(dht_count == 2 && dht_times[0] == 2000000 && dht_times[1] == 7000000);
    } else {
        assert(dht_count == 4);
        for (unsigned i = 0; i < dht_count; ++i) assert(dht_times[i] == 2000000 + i * 2000000LL);
    }
    if (gy_missing || task_failure) {
        sensor_status_t status;
        assert(!gy_count && sensor_status_get(SENSOR_GY511, &status));
        assert(status.state == SENSOR_UNAVAILABLE);
    } else {
        for (unsigned i = 0; i < 5; ++i) assert(gy_times[i] == 100000 + i * 500000LL);
        if (changes) {
            assert(gy_times[5] == 3100000 && gy_times[6] == 5100000);
            for (unsigned i = 7; i < gy_count; ++i) assert(gy_times[i] == 5600000 + (i - 7) * 500000LL);
        } else if (slow_read) {
            assert(acks[0].at_us == 2800000); // Receipt during I2C cannot ACK application early.
            for (unsigned i = 5; i < gy_count; ++i) assert(gy_times[i] == 3100000 + (i - 5) * 1000000LL);
        } else if (rounding) {
            for (unsigned i = 5; i < gy_count; ++i) assert(gy_times[i] == 2610000 + (i - 5) * 510000LL);
        } else {
            for (unsigned i = 5; i < gy_count; ++i) assert(gy_times[i] == 100000 + i * 500000LL);
        }
    }
    if (burst) assert(notify_busy > 0);
    if (shorten) assert(acks[1].at_us == 2600000); // Does not sleep for the old 10 s period.
    printf("PASS: interleaved readers/configuration scenario %s\n", scenario);
    return 0;
}
