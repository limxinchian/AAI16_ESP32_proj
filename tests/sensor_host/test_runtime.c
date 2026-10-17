// Two real application loops with deterministic, cooperative host scheduling.
// pthreads provide separate C stacks; only one simulated task runs at a time.
// This tests dispatch interleavings, not ESP32 SMP timing or the MQTT transport.
#include "platform.h"
#include <assert.h>
#include <pthread.h>
#include <string.h>

static int quiet_printf(const char *format, ...) { return 0; }
#define printf quiet_printf
#include "../../main/main.c"
#undef printf

typedef struct {
    pthread_t thread;
    bool exists, running, notify_wait, notified;
    uint32_t notification;
    int64_t wake_us;
    void (*entry)(void *);
} sim_task_t;
static sim_task_t tasks[2]; // app_main, GY-511 reader
static pthread_mutex_t scheduler_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t scheduler_changed = PTHREAD_COND_INITIALIZER;
static bool stopping;
static int running = -1;
static int64_t base_us, end_us = 8200000;
static bool gy_missing, dht_missing, task_failure, slow_read;
static bool slow_dht_read, preempt_on_notify, gy_read_error;
static unsigned diagnostics_paused, dht_inits, gy_inits;
static unsigned notify_busy, diagnostics, dht_count, gy_count, dht_tx, gy_tx;
static int64_t dht_times[64], gy_times[64];
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
    sensor_state_t state;
} ack_t;
static ack_t acks[32];
static unsigned ack_count;

static int task_index(void)
{
    for (int i = 0; i < 2; ++i) {
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
    assert(task_index() == 0 && stack == 4096 && priority == 4 && handle);
    assert(!strcmp(name, "gy511_reader"));
    if (task_failure) return pdFALSE;
    pthread_mutex_lock(&scheduler_lock);
    tasks[1].exists = true;
    tasks[1].entry = entry;
    tasks[1].wake_us = test_now_us;
    *handle = &tasks[1];
    assert(pthread_create(&tasks[1].thread, NULL, task_entry, &tasks[1]) == 0);
    pthread_mutex_unlock(&scheduler_lock);
    return pdPASS;
}
BaseType_t xTaskNotify(TaskHandle_t handle, uint32_t value, eNotifyAction action)
{
    assert(task_index() == 0 && handle == &tasks[1] && action == eSetValueWithoutOverwrite);
    sim_task_t *task = handle;
    if (task->notified) { notify_busy++; return pdFALSE; }
    task->notified = true;
    task->notification = value;
    if (task->notify_wait) task->wake_us = test_now_us;
    // Exercise a higher-priority reader committing before app_main removes the
    // FIFO head, as can happen on the target (including across its two cores).
    if (preempt_on_notify && task->notify_wait) yield_until(test_now_us, false);
    return pdPASS;
}
BaseType_t xTaskNotifyWait(uint32_t clear_entry, uint32_t clear_exit,
                           uint32_t *value, TickType_t wait)
{
    assert(task_index() == 1 && clear_entry == 0 && clear_exit == UINT32_MAX);
    sim_task_t *task = &tasks[1];
    if (!task->notified && wait) yield_until(test_now_us + wait * 10000LL, true);
    if (!task->notified) return pdFALSE;
    *value = task->notification;
    task->notified = false;
    return pdTRUE;
}

esp_err_t wifi_manager_start(void) { return ESP_OK; }
esp_err_t mqtt_manager_init(void) { return ESP_OK; }
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
    assert(task_index() == 1 && gy_count < 64);
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
esp_err_t mqtt_manager_publish_config_ack(const mqtt_config_update_t *update)
{
    // In particular, the dispatcher must NEVER ACK a forwarded GY-511 update.
    assert(task_index() == (update->sensor == SENSOR_GY511 ? 1 : 0));
    sensor_status_t status;
    assert(sensor_status_get(update->sensor, &status));
    if (update->operation == MQTT_CONFIG_ENABLED) {
        // State must already be committed when the ACK API is called.
        assert(update->enabled ? sensor_status_can_sample(update->sensor) : status.state == SENSOR_DISABLED);
    }
    assert(ack_count < 32);
    acks[ack_count++] = (ack_t){.update = *update, .applied = true, .at_us = test_now_us - base_us,
        .read_count = update->sensor == SENSOR_DHT22 ? dht_count : gy_count, .state = status.state};
    return ESP_OK;
}
esp_err_t mqtt_manager_reject_config(const mqtt_config_update_t *update, const char *reason)
{
    sensor_status_t status;
    assert(sensor_status_get(update->sensor, &status));
    assert(!strcmp(reason, status.state == SENSOR_DISABLED ? "sensor_disabled" : "sensor_unavailable"));
    assert(!sensor_status_can_sample(update->sensor));
    assert(ack_count < 32);
    acks[ack_count++] = (ack_t){.update = *update, .applied = false, .at_us = test_now_us - base_us,
        .reason = reason, .read_count = update->sensor == SENSOR_DHT22 ? dht_count : gy_count,
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
    assert(!queued && taken == input_count && ack_count == input_count);
    assert(dht_inits == 1 && gy_inits == 1); // Enable never reinitializes either driver.
    assert(dht_tx == dht_count && gy_tx == gy_count - (gy_read_error ? 1U : 0U) && diagnostics > 0);
    // FIFO application/ACK order within each sensor, including rejections at
    // the owner. Cross-sensor ACK order need not match the original FIFO.
    for (int sensor = SENSOR_DHT22; sensor <= SENSOR_GY511; ++sensor) {
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
        for (int i = 0; i < 2; ++i) {
            if (tasks[i].exists && tasks[i].wake_us < next_us) next_us = tasks[i].wake_us;
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
        for (int i = 1; i >= 0; --i) { // GY-511 is higher priority than app_main.
            if (tasks[i].exists && tasks[i].wake_us <= test_now_us) {
                tasks[i].running = true; running = i;
                pthread_cond_broadcast(&scheduler_changed);
                break;
            }
        }
    }
    stopping = true;
    pthread_cond_broadcast(&scheduler_changed);
    pthread_mutex_unlock(&scheduler_lock);
    for (int i = 0; i < 2; ++i) if (tasks[i].exists) pthread_join(tasks[i].thread, NULL);
}

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

int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *scenario = argv[1];
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
