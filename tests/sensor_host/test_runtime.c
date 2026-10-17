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
static unsigned notify_busy, diagnostics, dht_count, gy_count, dht_tx, gy_tx;
static int64_t dht_times[64], gy_times[64];
static mqtt_interval_update_t queue[4];
static unsigned queued, taken;
typedef struct { int64_t at_us; mqtt_interval_update_t update; } input_t;
static input_t inputs[16];
static unsigned input_count, next_input;
typedef struct { mqtt_interval_update_t update; bool applied; int64_t at_us; } ack_t;
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
    assert(wake_us > test_now_us);
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
esp_err_t gy511_sensor_init(void) { return gy_missing ? ESP_FAIL : ESP_OK; }
esp_err_t gy511_sensor_deinit(void) { return ESP_OK; }
esp_err_t dht22_init(gpio_num_t pin) { return dht_missing ? ESP_ERR_NO_MEM : ESP_OK; }
esp_err_t dht22_read(dht22_data_t *data)
{
    assert(task_index() == 0 && dht_count < 64);
    dht_times[dht_count++] = test_now_us - base_us;
    *data = (dht22_data_t){25, 50}; return ESP_OK;
}
esp_err_t gy511_sensor_read(gy511_data_t *data)
{
    assert(task_index() == 1 && gy_count < 64);
    gy_times[gy_count++] = test_now_us - base_us;
    if (slow_read && test_now_us - base_us == 2100000) vTaskDelay(pdMS_TO_TICKS(700));
    *data = (gy511_data_t){1, 2, 3, 10, 20, 30}; return ESP_OK;
}
esp_err_t mqtt_manager_publish_dht22(float temperature, float humidity)
{
    assert(temperature == 25 && humidity == 50); dht_tx++; return ESP_OK;
}
esp_err_t mqtt_manager_publish_gy511(float ax, float ay, float az, float mx, float my, float mz)
{
    assert(ax == 1 && ay == 2 && az == 3 && mx == 10 && my == 20 && mz == 30);
    gy_tx++; return ESP_OK;
}
bool mqtt_manager_peek_interval_update(mqtt_interval_update_t *update)
{
    assert(task_index() == 0);
    if (!queued) return false;
    *update = queue[0]; return true;
}
bool mqtt_manager_take_interval_update(mqtt_interval_update_t *update)
{
    if (!mqtt_manager_peek_interval_update(update)) return false;
    queued--; taken++;
    memmove(queue, queue + 1, queued * sizeof(queue[0]));
    return true;
}
esp_err_t mqtt_manager_publish_config_ack(sensor_id_t sensor, uint32_t interval)
{
    // In particular, the dispatcher must NEVER ACK a forwarded GY-511 update.
    assert(task_index() == (sensor == SENSOR_GY511 ? 1 : 0));
    assert(ack_count < 32);
    acks[ack_count++] = (ack_t){{sensor, interval}, true, test_now_us - base_us};
    return ESP_OK;
}
esp_err_t mqtt_manager_reject_config(const mqtt_interval_update_t *update, const char *reason)
{
    assert(!strcmp(reason, "sensor_unavailable") && !sensor_status_can_sample(update->sensor));
    assert(ack_count < 32);
    acks[ack_count++] = (ack_t){*update, false, test_now_us - base_us};
    return ESP_OK;
}
void mqtt_manager_log_diagnostics(void) { diagnostics++; }

static void input(int64_t at_us, sensor_id_t sensor, uint32_t interval)
{
    assert(input_count < 16);
    inputs[input_count++] = (input_t){at_us, {sensor, interval}};
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

int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *scenario = argv[1];
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
    assert(!queued && taken == input_count && ack_count == input_count);
    assert(dht_tx == dht_count && gy_tx == gy_count && diagnostics > 0);
    // Per-sensor ACK order/identity matches every command, even across busy handoffs.
    for (int sensor = SENSOR_DHT22; sensor <= SENSOR_GY511; ++sensor) {
        unsigned cursor = 0;
        for (unsigned i = 0; i < input_count; ++i) {
            if (inputs[i].update.sensor != (sensor_id_t)sensor) continue;
            while (cursor < ack_count && acks[cursor].update.sensor != (sensor_id_t)sensor) cursor++;
            assert(cursor < ack_count && acks[cursor].update.interval_ms == inputs[i].update.interval_ms);
            assert(acks[cursor].at_us >= inputs[i].at_us);
            assert(acks[cursor].applied == !(sensor == SENSOR_GY511 && (gy_missing || task_failure)));
            cursor++;
        }
    }
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
