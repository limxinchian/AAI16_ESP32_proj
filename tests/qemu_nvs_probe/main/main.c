#include <stdio.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    nvs_handle_t h;
    uint32_t boots = 0;
    esp_err_t oe = nvs_open("probe", NVS_READWRITE, &h);
    ESP_LOGI("probe", "nvs_open: %s", esp_err_to_name(oe));
    esp_err_t ge = nvs_get_u32(h, "boots", &boots);
    ESP_LOGI("probe", "PROBE_READ boots=%u (%s)", (unsigned)boots, esp_err_to_name(ge));
    boots++;
    ESP_ERROR_CHECK(nvs_set_u32(h, "boots", boots));
    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);
    ESP_LOGI("probe", "PROBE_WROTE boots=%u", (unsigned)boots);
    vTaskDelay(pdMS_TO_TICKS(500));
    if (boots < 3) esp_restart();
    ESP_LOGI("probe", "PROBE_DONE");
}
