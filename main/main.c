
#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "dht22.h"

#define DHT22_GPIO GPIO_NUM_6
#define DHT22_INTERVAL_MS 2000

void app_main(void)
{
    printf("AAI16 ESP32-S3 Project Started!\n");

    esp_err_t err = dht22_init(DHT22_GPIO);

    if (err != ESP_OK) {
        printf("DHT22 initialization failed: %s\n",
               esp_err_to_name(err));
        return;
    }

    printf("DHT22 initialized. Starting readings...\n");

    // Allow the sensor to stabilize after power-up.
    vTaskDelay(pdMS_TO_TICKS(2000));

    uint32_t success_count = 0;
    uint32_t failure_count = 0;

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        dht22_data_t data = {0};

        err = dht22_read(&data);

        if (err == ESP_OK) {
            success_count++;

            printf("Temperature: %.1f C | Humidity: %.1f %%\n",
                   data.temperature_c,
                   data.humidity_pct);

        } else {
            failure_count++;

            printf("DHT22 Read Failed: %s\n",
                   esp_err_to_name(err));
        }

        printf("Success: %lu | Failed: %lu\n\n",
               (unsigned long)success_count,
               (unsigned long)failure_count);

        fflush(stdout);

        // Maintain an approximately 2-second sampling period.
        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(DHT22_INTERVAL_MS)
        );
    }
}
