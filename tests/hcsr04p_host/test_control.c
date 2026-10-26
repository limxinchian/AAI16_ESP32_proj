// Reuse the driver suite's register/ISR/queue doubles, exercising the real
// reader and real MCPWM driver together. The original driver suite still runs.
#define main driver_regression_main
#include "test_driver.c"
#undef main
#include <setjmp.h>
#include "../../main/hcsr04p_reader.c"

static jmp_buf control_done;
static void (*control_reader)(void *);
static bool notification_pending, mqtt_enabled = true, timeout_case;
static uint32_t notification;
static unsigned control_stage, acknowledgements, snapshots;
static sensor_status_t disabled_status;
static unsigned init_steps;
static int64_t control_origin;

BaseType_t xTaskCreate(void (*fn)(void *), const char *name, unsigned stack,
    void *arg, unsigned priority, TaskHandle_t *handle)
{
    assert(!strcmp(name, "hcsr04p_reader") && stack == 4096 && priority == 3 && !arg && handle);
    control_reader = fn; *handle = (void *)4; return pdPASS;
}
void vTaskDelete(TaskHandle_t task) { assert(false); }
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task) { return 2000; }

static void notify_control(uint32_t control)
{
    assert(!notification_pending);
    notification_pending = true; notification = control;
}
static void disable_during_echo(void)
{
    if (triggers != 2) return;
    assert(s_armed && !high_trig && mqtt_enabled);
    init_steps = step;
    notify_control(CONFIG_ENABLED_FLAG); // Request arrives during active capture.
    assert(sensor_status_can_sample(SENSOR_HCSR04P)); // Not committed by requester.
    capture_wait_hook = NULL;
}

BaseType_t xTaskNotifyWait(uint32_t entry, uint32_t exit, uint32_t *value, TickType_t wait)
{
    assert(!entry && exit == UINT32_MAX);
    if (!notification_pending && wait) {
        int64_t wake = test_now_us + wait * 10000LL;
        // Change actual acquisition to 100 ms while idle. Following the second
        // ECHO, pause until 300 ms; a repeated enable at 350 ms cannot postpone.
        const int64_t at[] = {50000, 300000, 350000, 450000};
        int64_t next = control_origin + at[control_stage];
        if (next <= wake) {
            test_now_us = next;
            if (control_stage == 3) longjmp(control_done, 1);
            if (control_stage == 1) {
                assert(!mqtt_enabled && !s_armed && triggers == 2 && !high_trig);
                unsigned old_sends = sends;
                edge(MCPWM_CAP_EDGE_POS, 100); edge(MCPWM_CAP_EDGE_NEG, 80100);
                assert(!queued && sends == old_sends); // Late callbacks while disabled.
                // Also verify the existing read entry drains an old completion
                // rather than using it for the first post-enable measurement.
                queued = true; queued_event = (echo_event_t){.ticks = 40000, .error = ESP_OK};
                mode = VALID; pulse_ticks = 80000;
            }
            notify_control(control_stage == 0 ? 100 : CONFIG_ENABLED_FLAG | 1);
            control_stage++;
        } else test_now_us = wake;
    }
    if (!notification_pending) return pdFALSE;
    *value = notification; notification_pending = false; return pdTRUE;
}

void mqtt_manager_commit_hcsr04p_enabled(bool enabled)
{
    assert(!s_armed && !high_trig && step == init_steps); // No capture reinstallation.
    mqtt_enabled = enabled;
}
esp_err_t mqtt_manager_update_hcsr04p_snapshot(const hcsr04p_reading_t *reading, int64_t acquired_us)
{
    assert(mqtt_enabled && !s_armed && acquired_us == test_now_us);
    assert(reading->echo_us == 1000 && fabsf(reading->distance_cm - 17.15f) < 0.001f);
    if (triggers == 3) assert(s_last_trigger_us >= control_origin + 400000 && s_last_trigger_us < control_origin + 410000);
    snapshots++;
    return ESP_OK;
}
esp_err_t mqtt_manager_publish_config_ack(const mqtt_config_update_t *update)
{
    assert(update->sensor == SENSOR_HCSR04P && !s_armed && !high_trig);
    sensor_status_t status;
    assert(sensor_status_get(SENSOR_HCSR04P, &status));
    if (update->operation == MQTT_CONFIG_ENABLED) {
        assert(mqtt_enabled == update->enabled && triggers == 2);
        if (!update->enabled) {
            assert(status.state == SENSOR_DISABLED);
            assert(status.read_successes + status.read_failures == 2);
            disabled_status = status;
        } else {
            assert(status.read_successes == disabled_status.read_successes &&
                   status.read_failures == disabled_status.read_failures &&
                   status.last_success_us == disabled_status.last_success_us &&
                   status.consecutive_failures == disabled_status.consecutive_failures);
            assert(status.state == (timeout_case ? SENSOR_DEGRADED : SENSOR_READY));
        }
    } else assert(update->interval_ms == 100);
    acknowledgements++;
    return ESP_OK;
}
esp_err_t mqtt_manager_reject_config(const mqtt_config_update_t *update, const char *reason)
{ assert(false); return ESP_FAIL; }

int main(int argc, char **argv)
{
    assert(argc == 2);
    timeout_case = !strcmp(argv[1], "timeout");
    reset(); control_origin = test_now_us;
    mode = timeout_case ? MISSING : VALID;
    capture_wait_hook = disable_during_echo;
    hcsr04p_reader_start();
    if (!setjmp(control_done)) control_reader(NULL);
    assert(triggers == 3 && acknowledgements == 4 && step == init_steps);
    assert(snapshots == (timeout_case ? 1U : 3U));
    assert(!s_armed && !high_trig && !queued && mqtt_enabled);
    assert(timer_enabled && timer_started && channel_enabled);
    sensor_status_t status;
    assert(sensor_status_get(SENSOR_HCSR04P, &status) && status.state == SENSOR_READY);
    assert(status.read_failures == (timeout_case ? 2U : 0U) && !status.consecutive_failures);
    printf("PASS: real capture/reader disable during %s ECHO, late ISR, stale queue, retained interval and recovery\n", argv[1]);
    return 0;
}
