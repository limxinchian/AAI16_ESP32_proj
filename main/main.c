/* AAI16 configurable edge node - application entry point.
 *
 * One task ("node") owns the whole node_t: it drains a queue filled by the MQTT and console tasks and
 * calls node_tick() every millisecond. Nothing else touches the node (see components/node_core). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "platform_esp.h"
#include "sdkconfig.h"

static const char *TAG = "aai16";

static node_t *s_node;
static QueueHandle_t s_inbox;

static void node_task(void *arg)
{
    (void)arg;
    plat_msg_t m;
    for (;;) {
        while (xQueueReceive(s_inbox, &m, 0) == pdTRUE) {
            if (m.kind == PLAT_IN_REPL) {
                char out[320];
                node_repl_line(s_node, (const char *)m.data, out, sizeof(out));
                printf("%s\n", out);
            } else {
                node_on_message(s_node, (node_in_t)m.kind, m.from, m.data, m.len);
            }
            free(m.data);
        }
        node_tick(s_node);
        vTaskDelay(1); /* 1 ms tick (CONFIG_FREERTOS_HZ=1000) */
    }
}

/* Reads console lines without needing a blocking UART driver: poll getchar(), sleep when idle. */
static void console_task(void *arg)
{
    (void)arg;
    static char line[2 * 4096 + 16]; /* NOT on the task stack: it is larger than the 6 KB stack */
    size_t n = 0;
    for (;;) {
        const int c = getchar();
        if (c == EOF) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (n > 0) {
                line[n] = '\0';
                plat_msg_t m = {.kind = PLAT_IN_REPL, .data = (uint8_t *)strdup(line), .len = n};
                if (!m.data || xQueueSend(s_inbox, &m, 0) != pdTRUE) free(m.data);
                n = 0;
            }
        } else if (n < sizeof(line) - 1) {
            line[n++] = (char)c;
        }
    }
}

void app_main(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* Destructive: the stored configuration and model metadata are lost. Say so loudly. */
        ESP_LOGW(TAG, "nvs_flash_init returned %s: ERASING the NVS partition", esp_err_to_name(e));
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_LOGI(TAG, "NVS ready: %s", esp_err_to_name(e));
    ESP_ERROR_CHECK(e);

    s_node = heap_caps_calloc(1, sizeof(node_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_inbox = xQueueCreate(8, sizeof(plat_msg_t));
    if (!s_node || !s_inbox) {
        ESP_LOGE(TAG, "out of memory for the node (%u bytes needed in PSRAM)", (unsigned)sizeof(node_t));
        return;
    }

    static node_platform_t pf; /* kept alive: node_t copies it, but plat_fill's callbacks are static */
    plat_fill(&pf, CONFIG_AAI16_NODE_ID);
    node_init(s_node, &pf);
    node_boot(s_node);
    ESP_LOGI(TAG, "node '%s' booted: config generation %u, model %s", CONFIG_AAI16_NODE_ID,
             (unsigned)s_node->cm.committed.generation, s_node->model_active ? "restored" : "none");

    if (CONFIG_AAI16_WIFI_SSID[0]) {
        if (!plat_net_start(CONFIG_AAI16_NODE_ID, CONFIG_AAI16_WIFI_SSID, CONFIG_AAI16_WIFI_PASSWORD,
                            CONFIG_AAI16_MQTT_URI, s_inbox)) {
            ESP_LOGE(TAG, "network start failed; continuing with the serial console only");
        }
    } else {
        ESP_LOGW(TAG, "no Wi-Fi SSID configured (menuconfig > AAI16 node): serial console only");
    }

    xTaskCreatePinnedToCore(node_task, "node", 12288, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(console_task, "console", 6144, NULL, 3, NULL, 0);
}
