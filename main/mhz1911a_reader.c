#include "mhz1911a_reader.h"
#include "mhz1911a_sensor.h"
#include "sensor_status.h"
#include "mqtt_manager.h"
#include "sensor_config.h"
#include <inttypes.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CO2_HEALTH_US 60000000LL
#define CO2_UNAVAILABLE_AFTER 3

static bool s_reader_started;
static TaskHandle_t s_co2_task;
static bool s_co2_control_ready;
static portMUX_TYPE s_co2_control_lock = portMUX_INITIALIZER_UNLOCKED;

static void mhz1911a_reader_task(void *arg)
{
    (void)arg;
    esp_err_t err = mhz1911a_sensor_init();
    if (err != ESP_OK) {
        sensor_status_record_init(SENSOR_MHZ1911A, err);
        ESP_LOGE("SENSOR", "mhz1911a | ERROR | UART initialization=%s; other sensors continue", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    // Publish a live control owner only after UART initialization succeeds.
    // An init-failure task may delete itself on the other core; app_main must
    // never notify its stale handle or leave a command without an owner ACK.
    portENTER_CRITICAL(&s_co2_control_lock);
    s_co2_control_ready = true;
    portEXIT_CRITICAL(&s_co2_control_lock);
    const int64_t tick_us = portTICK_PERIOD_MS * 1000LL;
    uint32_t interval_ms = MHZ1911A_SAMPLE_MS;
    int64_t period_us = interval_ms * 1000LL;
    const int64_t warmup_until = esp_timer_get_time() + MHZ1911A_WARMUP_MS * 1000LL;
    int64_t next_due = warmup_until;
    int64_t next_health = next_due, previous_start = -1;
    int64_t max_read = 0, max_late = 0, max_jitter = 0;
    uint32_t overruns = 0;
    bool enabled = true, pending_update = false;
    uint32_t control = 0;
    ESP_LOGI("SENSOR", "mhz1911a | HEALTH | state=UNINITIALIZED warmup=%u ms UART2 TX=15 RX=16 9600/8N1 period=%u ms",
             MHZ1911A_WARMUP_MS, MHZ1911A_SAMPLE_MS);
    ESP_LOGI("SENSOR", "mhz1911a | HEALTH | UART method tested on one module; manufacturer confirmation pending; early readings may stabilize");
    while (true) {
        // A notification received during a bounded UART operation is committed
        // only here, after that transaction and its success-only handoff finish.
        for (unsigned commands = 0; commands < 4; ++commands) {
            if (!pending_update && xTaskNotifyWait(0, UINT32_MAX, &control, 0) != pdTRUE) break;
            pending_update = false;
            mqtt_config_update_t update = {.sensor = SENSOR_MHZ1911A};
            bool gate = (control & CONFIG_ENABLED_FLAG) != 0;
            update.operation = gate ? MQTT_CONFIG_ENABLED : MQTT_CONFIG_INTERVAL;
            if (gate) update.enabled = (control & 1U) != 0;
            else update.interval_ms = control;
            // This live probe owner accepts controls during original warm-up
            // and READ failures; generic initialized-only policy is unsuitable.
            if (!gate && !enabled) {
                (void)mqtt_manager_reject_config(&update, "sensor_disabled");
                continue;
            }
            if (gate && sensor_status_set_probe_enabled(SENSOR_MHZ1911A, update.enabled) != ESP_OK) {
                (void)mqtt_manager_reject_config(&update, "sensor_unavailable");
                continue;
            }
            if (!gate || enabled != update.enabled) {
                if (gate) enabled = update.enabled;
                else interval_ms = update.interval_ms;
                period_us = interval_ms * 1000LL;
                mqtt_manager_commit_mhz1911a_config(enabled);
                next_due = esp_timer_get_time() + period_us;
                if (next_due < warmup_until) next_due = warmup_until;
                previous_start = -1;
            }
            if (gate) ESP_LOGI("SENSOR", "mhz1911a | CONFIG | enabled=%s", enabled ? "true" : "false");
            else ESP_LOGI("SENSOR", "mhz1911a | CONFIG | interval_ms=%" PRIu32, interval_ms);
            // Same bounded ACK queue as all other sensors; never ACK on receipt.
            (void)mqtt_manager_publish_config_ack(&update);
        }
        int64_t now = esp_timer_get_time();
        if (enabled && now >= next_due) {
            int64_t start = esp_timer_get_time();
            int64_t late = start - next_due;
            if (late > max_late) max_late = late;
            if (previous_start >= 0) {
                int64_t jitter = start - previous_start - period_us;
                if (jitter > max_jitter) max_jitter = jitter;
            }
            previous_start = start;
            next_due = start + period_us;
            mhz1911a_reading_t reading;
            err = mhz1911a_sensor_read(&reading); // Sole owner; requests cannot overlap.
            int64_t completed = esp_timer_get_time();
            int64_t read_us = completed - start;
            if (read_us > max_read) max_read = read_us;
            sensor_status_record_probe(SENSOR_MHZ1911A, err, completed, CO2_UNAVAILABLE_AFTER);
            if (err == ESP_OK) {
                // Fixed-size handoff only; the shared publisher owns all SDK waits.
                // MQTT submission/delivery outcomes do not change acquisition health.
                (void)mqtt_manager_update_mhz1911a_snapshot(reading.co2_ppm, completed);
                ESP_LOGI("SENSOR", "mhz1911a | DATA | co2=%u ppm read=%.1f ms", (unsigned)reading.co2_ppm, read_us / 1000.0);
            }
            now = esp_timer_get_time();
            if (now >= next_due) {
                ++overruns;
                next_due = now + period_us; // Skip missed periods; no catch-up burst.
            }
        }
        // The registry throttles failure/recovery logs. Keep cumulative detail
        // visible once a minute, including while disabled or disconnected.
        if (now >= next_health) {
            next_health = now + CO2_HEALTH_US;
            sensor_status_t status;
            mhz1911a_diagnostics_t d;
            mhz1911a_sensor_get_diagnostics(&d);
            if (sensor_status_get(SENSOR_MHZ1911A, &status)) {
                ESP_LOGI("SENSOR", "mhz1911a | HEALTH | state=%s ok_total=%" PRIu32 " failed_total=%" PRIu32
                         " consecutive=%" PRIu32 " period=%" PRIu32 " ms enabled=%s timeouts_total=%" PRIu32 " checksum_failed_total=%" PRIu32,
                         sensor_status_state_name(status.state), status.read_successes, status.read_failures,
                         status.consecutive_failures, interval_ms, enabled ? "true" : "false", d.timeouts, d.checksum_failures);
            }
            ESP_LOGI("SENSOR", "mhz1911a | HEALTH | invalid_total=%" PRIu32 " partial_total=%" PRIu32 " uart_errors_total=%" PRIu32
                     " rx_overflow_total=%" PRIu32 " event_overflow_total=%" PRIu32 " tx_failed_total=%" PRIu32 " api_errors_total=%" PRIu32,
                     d.invalid_frames, d.partial_responses, d.uart_errors, d.rx_overflows, d.event_overflows, d.tx_failures, d.api_errors);
            ESP_LOGI("SENSOR", "mhz1911a | HEALTH | requests_total=%" PRIu32 " discarded_bytes_total=%" PRIu32 " max_read=%.1f ms"
                     " max_late=%.1f ms max_jitter=%.1f ms overruns_total=%" PRIu32 " stack_min=%u B",
                     d.requests, d.discarded_bytes, max_read / 1000.0, max_late / 1000.0, max_jitter / 1000.0,
                     overruns, (unsigned)uxTaskGetStackHighWaterMark(NULL));
        }
        now = esp_timer_get_time();
        int64_t wake = next_health;
        if (enabled && next_due < wake) wake = next_due;
        if (wake > now) {
            // Notification wakes warm-up/idle/disabled waits without polling.
            pending_update = xTaskNotifyWait(0, UINT32_MAX, &control,
                (TickType_t)((wake - now + tick_us - 1) / tick_us)) == pdTRUE;
        }
    }
}

void mhz1911a_reader_start(void)
{
    if (s_reader_started) return;
    s_reader_started = true;
    sensor_status_configure(SENSOR_MHZ1911A, "mhz1911a", true);
    // Matches RC522/LD2450/HC-SR04P. Stack/priority remain provisional until
    // continuous hardware testing; no control queue or MQTT task is introduced.
    if (xTaskCreate(mhz1911a_reader_task, "mhz1911a_reader", 4096, NULL, 3, &s_co2_task) != pdPASS) {
        sensor_status_record_unavailable(SENSOR_MHZ1911A, SENSOR_STAGE_TASK, ESP_ERR_NO_MEM);
        ESP_LOGE("SENSOR", "mhz1911a | ERROR | Reader task creation failed; other sensors continue");
    }
}

TaskHandle_t mhz1911a_reader_task_handle(void)
{
    // app_main calls only after start returns, so xTaskCreate has finished
    // assigning s_co2_task. A ready owner remains alive through all read faults.
    portENTER_CRITICAL(&s_co2_control_lock);
    TaskHandle_t task = s_co2_control_ready ? s_co2_task : NULL;
    portEXIT_CRITICAL(&s_co2_control_lock);
    return task;
}
