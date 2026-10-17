
#include <stdio.h>
#include <stdint.h>

#include "wifi_manager.h"
#include "mqtt_manager.h"

#include "i2c_bus_manager.h"
#include "gy511_sensor.h"
#include "sensor_status.h"
#include "rc522_reader.h"
#include "ld2450_reader.h"
#include "sensor_config.h"

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dht22.h"

#define DHT22_GPIO GPIO_NUM_6
#define SAMPLE_INTERVAL_MS 2000

#define SUMMARY_EVERY 30
#define JITTER_LIMIT_US 50000LL

#define GY511_SAMPLE_MS 500
// Serial display only; every successful read is still published.
#define GY511_LOG_INTERVAL_MS 10000

static TaskHandle_t s_gy511_task;

static void gy511_reader_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(100));
    TickType_t next_due = xTaskGetTickCount();
    TickType_t last_sample = next_due;
    uint32_t active_interval_ms = GY511_SAMPLE_MS;
    TickType_t period = pdMS_TO_TICKS(GY511_SAMPLE_MS);
    uint32_t control = 0;
    bool pending_update = false;
    int64_t next_log_us = 0;
    uint32_t reads = 0, ok = 0, overruns = 0;
    int64_t max_read_us = 0;
    esp_err_t last_status = ESP_OK;

    while (1) {
        // Notification index 0 belongs exclusively to sensor configuration.
        // Bound work under a command flood; only this task commits its schedule.
        for (unsigned commands = 0; commands < 4; ++commands) {
            if (!pending_update &&
                xTaskNotifyWait(0, UINT32_MAX, &control, 0) != pdTRUE) break;
            pending_update = false;
            mqtt_config_update_t update = {.sensor = SENSOR_GY511};
            if (control & CONFIG_ENABLED_FLAG) {
                update.operation = MQTT_CONFIG_ENABLED;
                update.enabled = (control & 1) != 0;
            } else {
                update.operation = MQTT_CONFIG_INTERVAL;
                update.interval_ms = control;
            }
            // Validate availability/disabled policy HERE, after all older
            // commands have committed, not against a dispatcher snapshot.
            sensor_status_t status;
            if (!config_can_apply(&update, &status)) continue;
            if (update.operation == MQTT_CONFIG_ENABLED) {
                if (sensor_status_set_enabled(SENSOR_GY511, update.enabled) != ESP_OK) {
                    (void)mqtt_manager_reject_config(&update, "sensor_unavailable");
                    continue;
                }
                if (update.enabled && status.state == SENSOR_DISABLED) {
                    // Fresh anchor: no catch-up reads, and repeated enables do
                    // not keep moving the deadline. Retain the configured period.
                    last_sample = xTaskGetTickCount();
                    next_due = last_sample + period;
                }
                printf("\n[CONFIG] GY511 enabled applied: %s\n", update.enabled ? "true" : "false");
            } else {
                active_interval_ms = update.interval_ms;
                // Round up to the RTOS tick (10 ms in this project).
                period = pdMS_TO_TICKS(active_interval_ms + portTICK_PERIOD_MS - 1);
                // Repeated commands cannot postpone acquisition indefinitely.
                next_due = last_sample + period;
                TickType_t now = xTaskGetTickCount();
                if ((int32_t)(next_due - now) < 0) next_due = now;
                printf("\n[CONFIG] GY511 interval applied: %lu ms\n",
                       (unsigned long)active_interval_ms);
            }
            (void)mqtt_manager_publish_config_ack(&update);
        }

        if (!sensor_status_can_sample(SENSOR_GY511)) {
            // Pausing keeps the reader alive and wakeable by either operation.
            pending_update = xTaskNotifyWait(0, UINT32_MAX, &control, portMAX_DELAY) == pdTRUE;
            continue;
        }

        TickType_t now = xTaskGetTickCount();
        int32_t remaining = (int32_t)(next_due - now);
        if (remaining > 0) {
            // A notification wakes the reader without waiting for the old period.
            pending_update = xTaskNotifyWait(0, UINT32_MAX, &control,
                                             (TickType_t)remaining) == pdTRUE;
            continue;
        }
        last_sample = now;
        gy511_data_t reading = {0};
        int64_t start_us = esp_timer_get_time();
        esp_err_t err = gy511_sensor_read(&reading);
        int64_t completed_us = esp_timer_get_time();
        int64_t read_us = completed_us - start_us;
        sensor_status_record_read(SENSOR_GY511, err, completed_us);
        if (read_us > max_read_us) max_read_us = read_us;
        reads++;
        if (err == ESP_OK) {
            ok++;
            // Zero-wait handoff. Failures appear in MQTT diagnostics.
            (void)mqtt_manager_publish_gy511(
                reading.accel_x_g, reading.accel_y_g, reading.accel_z_g,
                reading.mag_x_ut, reading.mag_y_ut, reading.mag_z_ut);
        }
        int64_t now_us = esp_timer_get_time();
        if (now_us >= next_log_us || (err != ESP_OK && err != last_status)) {
            next_log_us = now_us + (int64_t)GY511_LOG_INTERVAL_MS * 1000;
            if (err == ESP_OK) {
                printf("\n[GY511 #%04lu] Acceleration (g) : X:%+7.3f | Y:%+7.3f | Z:%+7.3f\n"
                       "              Magnetic (uT)   : X:%+7.2f | Y:%+7.2f | Z:%+7.2f\n",
                       (unsigned long)reads, reading.accel_x_g, reading.accel_y_g,
                       reading.accel_z_g, reading.mag_x_ut, reading.mag_y_ut,
                       reading.mag_z_ut);
            } else {
                printf("\n[GY511 #%04lu] Reading failed: %s\n",
                       (unsigned long)reads, esp_err_to_name(err));
            }
            printf("              Period          : %lu ms | Read: %.1f ms | Status: %s\n"
                   "              Totals          : OK:%lu | Failed:%lu | Overruns:%lu\n"
                   "              Resources       : Stack min:%u B | Max read:%.1f ms | Uptime:%lld ms\n",
                   (unsigned long)active_interval_ms, read_us / 1000.0, err == ESP_OK ? "OK" : "FAIL",
                   (unsigned long)ok, (unsigned long)(reads - ok),
                   (unsigned long)overruns, (unsigned)uxTaskGetStackHighWaterMark(NULL),
                   max_read_us / 1000.0, (long long)(now_us / 1000));
        }
        last_status = err;
        if ((TickType_t)(xTaskGetTickCount() - next_due) >= period) {
            // Skip missed periods instead of spinning to catch up after a stall.
            overruns++;
            next_due = xTaskGetTickCount();
        }
        // Preserve vTaskDelayUntil's phase and full-period pause after an overrun.
        next_due += period;
    }
}

void app_main(void)
{
    printf("\n=== AAI16 | DHT22 TIMING TEST ===\n");
    printf("Target interval: %d ms\n", SAMPLE_INTERVAL_MS);
    printf("Jitter target: +/- 50 ms\n");

    sensor_status_configure(SENSOR_DHT22, "dht22", true);
    sensor_status_configure(SENSOR_GY511, "gy511", true);

    esp_err_t err = dht22_init(DHT22_GPIO);
    sensor_status_record_init(SENSOR_DHT22, err);

    if (err != ESP_OK)
    {
        printf("DHT22 initialization failed: %s; continuing platform startup.\n",
               esp_err_to_name(err));
    }
    else
    {
        printf("Sensor initialized successfully.\n");
    }

    // Start Wi-Fi without blocking the DHT22 test.
    esp_err_t wifi_err = wifi_manager_start();

    if (wifi_err != ESP_OK)
    {
        printf("Wi-Fi startup failed: %s\n",
               esp_err_to_name(wifi_err));
    }

    // Initialize MQTT.
    // MQTT retries automatically while Wi-Fi obtains an IP.
    esp_err_t mqtt_err = mqtt_manager_init();

    if (mqtt_err != ESP_OK)
    {
        printf("MQTT initialization failed: %s\n",
               esp_err_to_name(mqtt_err));
    }

    // Initialize and scan the shared I2C bus.
    esp_err_t i2c_err = i2c_bus_manager_init();

    if (i2c_err == ESP_OK)
    {
        i2c_bus_manager_scan();

        esp_err_t gy511_err = gy511_sensor_init();
        sensor_status_record_init(SENSOR_GY511, gy511_err);

        if (gy511_err == ESP_OK)
        {

            BaseType_t task_result = xTaskCreate(
                gy511_reader_task,
                "gy511_reader",
                4096, // Task stack size (bytes in ESP-IDF)
                NULL,
                4, // Task priority
                &s_gy511_task);

            if (task_result != pdPASS)
            {
                sensor_status_record_unavailable(
                    SENSOR_GY511, SENSOR_STAGE_TASK, ESP_ERR_NO_MEM);
                printf("Failed to create GY-511 task.\n");
                // No reader exists, so its device handles can be released safely.
                esp_err_t cleanup_err = gy511_sensor_deinit();
                if (cleanup_err != ESP_OK) {
                    printf("GY-511 cleanup failed: %s\n", esp_err_to_name(cleanup_err));
                }
            }
        }
        else
        {
            printf("GY-511 initialization failed: %s\n",
                   esp_err_to_name(gy511_err));
        }
    }
    else
    {
        sensor_status_record_unavailable(SENSOR_GY511, SENSOR_STAGE_DEPENDENCY, i2c_err);
    }

    rc522_reader_start();
    ld2450_reader_start();

    vTaskDelay(pdMS_TO_TICKS(2000));

    uint32_t total = 0;
    uint32_t success = 0;
    uint32_t failed = 0;
    uint32_t jitter_violations = 0;

    int64_t previous_start_us = -1;
    int64_t min_interval_us = INT64_MAX;
    int64_t max_interval_us = 0;
    int64_t worst_jitter_us = 0;

    int64_t test_start_us = esp_timer_get_time();
    int64_t next_due_us = test_start_us;
    int64_t last_sample_start_us = test_start_us;
    uint32_t active_interval_ms = SAMPLE_INTERVAL_MS;

    while (1)
    {
        // Check for validated MQTT configuration updates.
        mqtt_config_update_t update;

        // Bound work even if the broker continuously sends commands.
        for (unsigned commands = 0; commands < 4 &&
             mqtt_manager_peek_config_update(&update); ++commands)
        {
            if (update.sensor == SENSOR_GY511 || update.sensor == SENSOR_RC522 ||
                update.sensor == SENSOR_LD2450) {
                TaskHandle_t reader = update.sensor == SENSOR_GY511 ?
                    s_gy511_task : update.sensor == SENSOR_RC522 ?
                    rc522_reader_task_handle() : ld2450_reader_task_handle();
                if (reader == NULL) {
                    mqtt_manager_take_config_update(&update);
                    (void)mqtt_manager_reject_config(&update, "sensor_unavailable");
                    continue;
                }
                // No overwrite: keep the FIFO head pending if the reader still
                // has a notification. No ACK until that reader commits it.
                uint32_t control = update.operation == MQTT_CONFIG_ENABLED ?
                    CONFIG_ENABLED_FLAG | (uint32_t)update.enabled : update.interval_ms;
                if (xTaskNotify(reader, control,
                                eSetValueWithoutOverwrite) != pdPASS) break;
                mqtt_manager_take_config_update(&update);
                continue;
            }

            mqtt_manager_take_config_update(&update);
            if (update.sensor != SENSOR_DHT22) {
                (void)mqtt_manager_reject_config(&update, "unsupported_sensor");
                continue;
            }
            sensor_status_t status;
            if (!config_can_apply(&update, &status)) continue;
            if (update.operation == MQTT_CONFIG_ENABLED) {
                if (sensor_status_set_enabled(SENSOR_DHT22, update.enabled) != ESP_OK) {
                    (void)mqtt_manager_reject_config(&update, "sensor_unavailable");
                    continue;
                }
                if (update.enabled && status.state == SENSOR_DISABLED) {
                    last_sample_start_us = esp_timer_get_time();
                    next_due_us = last_sample_start_us + (int64_t)active_interval_ms * 1000LL;
                    previous_start_us = -1; // Exclude the deliberate pause from jitter.
                }
                printf("\n[CONFIG] DHT22 enabled applied: %s\n", update.enabled ? "true" : "false");
                (void)mqtt_manager_publish_config_ack(&update);
                continue;
            }

            // Apply the new sampling interval.
            active_interval_ms = update.interval_ms;

            // Anchor to acquisition, so repeated commands cannot keep pushing
            // the next read into the future. Never sample faster than 2000 ms.
            next_due_us = last_sample_start_us + (int64_t)active_interval_ms * 1000LL;

            // Do not count the transition as normal jitter.
            previous_start_us = -1;

            printf(
                "\n[CONFIG] DHT22 interval applied: %lu ms\n",
                (unsigned long)active_interval_ms);

            // Send application-level ACK over MQTT.
            esp_err_t ack_err =
                mqtt_manager_publish_config_ack(&update);

            if (ack_err != ESP_OK)
            {
                printf("[CONFIG] ACK queue failed: %s\n",
                       esp_err_to_name(ack_err));
            }
        }

        if (!sensor_status_can_sample(SENSOR_DHT22)) {
            mqtt_manager_log_diagnostics();
            sensor_status_log_summary();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int64_t remaining_us =
            next_due_us - esp_timer_get_time();

        if (remaining_us > 0)
        {
            uint32_t wait_ms =
                (uint32_t)((remaining_us + 999) / 1000);

            // Check for new configuration at least every ~100 ms.
            if (wait_ms > 100)
            {
                wait_ms = 100;
            }

            TickType_t ticks = pdMS_TO_TICKS(wait_ms);
            if (ticks == 0)
                ticks = 1;

            vTaskDelay(ticks);
            continue;
        }

        int64_t start_us = esp_timer_get_time();
        last_sample_start_us = start_us;

        next_due_us = start_us + (int64_t)active_interval_ms * 1000LL;

        int64_t interval_us = -1;
        int64_t jitter_us = 0;

        if (previous_start_us >= 0)
        {
            interval_us = start_us - previous_start_us;
            jitter_us = interval_us - (int64_t)active_interval_ms * 1000LL;

            if (interval_us < min_interval_us)
                min_interval_us = interval_us;

            if (interval_us > max_interval_us)
                max_interval_us = interval_us;

            int64_t abs_jitter =
                jitter_us < 0 ? -jitter_us : jitter_us;

            if (abs_jitter > worst_jitter_us)
                worst_jitter_us = abs_jitter;

            if (abs_jitter > JITTER_LIMIT_US)
                jitter_violations++;
        }

        previous_start_us = start_us;

        dht22_data_t data = {0};

        err = dht22_read(&data);

        int64_t completed_us = esp_timer_get_time();
        int64_t read_us = completed_us - start_us;
        sensor_status_record_read(SENSOR_DHT22, err, completed_us);

        // Publish real DHT22 data after a successful reading.
        if (err == ESP_OK)
        {
            (void)mqtt_manager_publish_dht22(
                data.temperature_c, data.humidity_pct);
        }

        total++;

        if (err == ESP_OK)
            success++;
        else
            failed++;

        // Print one readable record per acquisition. Missing measurements and
        // the first/transition interval are shown as "--", not invented zeros.
        char temperature[16] = "--", humidity[16] = "--";
        char interval[24] = "--", jitter[24] = "--";
        if (err == ESP_OK) {
            snprintf(temperature, sizeof(temperature), "%.1f", data.temperature_c);
            snprintf(humidity, sizeof(humidity), "%.1f", data.humidity_pct);
        }
        if (interval_us >= 0) {
            snprintf(interval, sizeof(interval), "%.1f", interval_us / 1000.0);
            snprintf(jitter, sizeof(jitter), "%+.1f", jitter_us / 1000.0);
        }
        printf("\n[DHT22 #%03lu] Temperature  : %5s C  | Humidity : %5s %%\n"
               "             Interval     : %8s ms | Jitter   : %7s ms\n"
               "             Read duration: %8.1f ms | Status   : %s\n",
               (unsigned long)total, temperature, humidity, interval, jitter,
               read_us / 1000.0, err == ESP_OK ? "OK" : esp_err_to_name(err));

        // Log after acquisition, so UART output is not placed ahead of a due read.
        mqtt_manager_log_diagnostics();
        sensor_status_log_summary();

        // Print cumulative summary every 30 samples.
        if (total % SUMMARY_EVERY == 0)
        {

            double elapsed_s =
                (esp_timer_get_time() - test_start_us) / 1000000.0;

            double rate = 100.0 * success / total;

            printf("\n========== DHT22 TEST SUMMARY ==========\n");
            printf("%-22s : %9.1f s\n",
                   "Elapsed", elapsed_s);
            printf("%-22s : %9lu\n",
                   "Total readings", (unsigned long)total);
            printf("%-22s : %9lu\n",
                   "Successful", (unsigned long)success);
            printf("%-22s : %9lu\n",
                   "Failed", (unsigned long)failed);
            printf("%-22s : %8.2f %%\n",
                   "Success rate", rate);
            printf("%-22s : %9u B\n", "Stack minimum free",
                   (unsigned)uxTaskGetStackHighWaterMark(NULL));

            if (min_interval_us != INT64_MAX)
            {
                printf("%-22s : %9.1f ms\n",
                       "Min interval",
                       min_interval_us / 1000.0);
                printf("%-22s : %9.1f ms\n",
                       "Max interval",
                       max_interval_us / 1000.0);
                printf("%-22s : %9.1f ms\n",
                       "Worst jitter",
                       worst_jitter_us / 1000.0);
                printf("%-22s : %9lu\n",
                       "Jitter violations",
                       (unsigned long)jitter_violations);
            }

            printf("========================================\n");
        }

    }
}
