
#include <stdio.h>
#include <stdint.h>

#include "wifi_manager.h"
#include "mqtt_manager.h"

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dht22.h"

#define DHT22_GPIO GPIO_NUM_6
#define SAMPLE_INTERVAL_MS 2000

#define SUMMARY_EVERY 30
#define JITTER_LIMIT_US 50000LL


void app_main(void)
{
    printf("\n=== AAI16 | DHT22 TIMING TEST ===\n");
    printf("Target interval: %d ms\n", SAMPLE_INTERVAL_MS);
    printf("Jitter target: +/- 50 ms\n");

    esp_err_t err = dht22_init(DHT22_GPIO);

    if (err != ESP_OK) {
        printf("Initialization failed: %s\n",
               esp_err_to_name(err));
        return;
    }

    printf("Sensor initialized successfully.\n");

    
    // Start Wi-Fi without blocking the DHT22 test.
    esp_err_t wifi_err = wifi_manager_start();

    if (wifi_err != ESP_OK) {
        printf("Wi-Fi startup failed: %s\n",
               esp_err_to_name(wifi_err));
    }

    
    // Initialize MQTT.
    // MQTT connection begins after Wi-Fi obtains an IP.
    esp_err_t mqtt_err = mqtt_manager_init();

    if (mqtt_err != ESP_OK) {
        printf("MQTT initialization failed: %s\n",
            esp_err_to_name(mqtt_err));
    }

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
    uint32_t active_interval_ms = SAMPLE_INTERVAL_MS;

    while (1) {
        
        // Check for validated MQTT configuration updates.
        uint32_t requested_ms;

        while (mqtt_manager_take_interval_update(
                   &requested_ms)) {

            // Apply the new sampling interval.
            active_interval_ms = requested_ms;

            // Start a fresh timing cycle for the new setting.
            next_due_us = esp_timer_get_time()
                + (int64_t)active_interval_ms * 1000LL;

            // Do not count the transition as normal jitter.
            previous_start_us = -1;

            printf(
                "\n[CONFIG] DHT22 interval applied: %lu ms\n",
                (unsigned long)active_interval_ms
            );

            // Send application-level ACK over MQTT.
            esp_err_t ack_err =
                mqtt_manager_publish_config_ack(
                    active_interval_ms
                );

            if (ack_err != ESP_OK) {
                printf("[CONFIG] ACK queue failed: %s\n",
                       esp_err_to_name(ack_err));
            }
        }

        int64_t remaining_us =
            next_due_us - esp_timer_get_time();

        if (remaining_us > 0) {
            uint32_t wait_ms =
                (uint32_t)((remaining_us + 999) / 1000);
            
            // Check for new configuration at least every ~100 ms.
            if (wait_ms > 100) {
                wait_ms = 100;
            }

            TickType_t ticks = pdMS_TO_TICKS(wait_ms);
            if (ticks == 0) ticks = 1;

            vTaskDelay(ticks);
            continue;
        }

        int64_t start_us = esp_timer_get_time();

        next_due_us = start_us
        + (int64_t)active_interval_ms * 1000LL;

        int64_t interval_us = -1;
        int64_t jitter_us = 0;

        if (previous_start_us >= 0) {
            interval_us = start_us - previous_start_us;
            jitter_us = interval_us
            - (int64_t)active_interval_ms * 1000LL;

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

        int64_t read_us =
            esp_timer_get_time() - start_us;

        
        // Publish real DHT22 data after a successful reading.
        if (err == ESP_OK) {
            esp_err_t publish_err = mqtt_manager_publish_dht22(
                data.temperature_c,
                data.humidity_pct
            );

            if (publish_err != ESP_OK &&
                publish_err != ESP_ERR_INVALID_STATE) {
                printf("MQTT queue failed: %s\n",
                    esp_err_to_name(publish_err));
            }
        }

        total++;

        if (err == ESP_OK)
            success++;
        else
            failed++;

        
        // Print each sensor reading with clearly labelled values.
        if (err == ESP_OK) {
            printf(
                "\n[DHT22 #%03lu] Temperature  : %5.1f C   | Humidity  : %5.1f %%\n",
                (unsigned long)total,
                data.temperature_c,
                data.humidity_pct
            );
        } else {
            printf(
                "\n[DHT22 #%03lu] Temperature  : %5s C   | Humidity  : %5s %%\n",
                (unsigned long)total,
                "--",
                "--"
            );
        }

        if (interval_us >= 0) {
            printf(
                "             Interval     : %6.1f ms | Jitter    : %+6.1f ms\n",
                interval_us / 1000.0,
                jitter_us / 1000.0
            );
        } else {
            printf(
                "             Interval     : %6s ms | Jitter    : %6s ms\n",
                "--",
                "--"
            );
        }

        printf(
            "             Read duration: %6.1f ms | Status    : %s\n",
            read_us / 1000.0,
            err == ESP_OK ? "OK" : "FAIL"
        );

        if (err != ESP_OK) {
            printf(
                "             Error        : %s\n",
                esp_err_to_name(err)
            );
        }

        printf("\n");


        // Print cumulative summary every 30 samples.
        if (total % SUMMARY_EVERY == 0) {

            double elapsed_s =
                (esp_timer_get_time() - test_start_us)
                / 1000000.0;

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

            if (total > 1) {
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

        fflush(stdout);
    }
}
