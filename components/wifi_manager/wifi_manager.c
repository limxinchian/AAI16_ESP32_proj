
#include "wifi_manager.h"
#include "wifi_credentials.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#define WIFI_MAX_RETRY 5

static const char *TAG = "AAI16_WIFI";
static int retry_count = 0;

static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_START) {

        ESP_LOGI(TAG, "Connecting to Wi-Fi...");
        esp_wifi_connect();
    }

    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {

        wifi_event_sta_disconnected_t *event =
            (wifi_event_sta_disconnected_t *)event_data;

        ESP_LOGW(TAG, "Disconnected, reason: %d",
                 event->reason);

        if (retry_count < WIFI_MAX_RETRY) {
            retry_count++;

            ESP_LOGI(TAG, "Retry %d/%d",
                     retry_count, WIFI_MAX_RETRY);

            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "Wi-Fi connection failed.");
        }
    }

    else if (event_base == IP_EVENT &&
             event_id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t *event =
            (ip_event_got_ip_t *)event_data;

        retry_count = 0;

        ESP_LOGI(TAG, "Connected to Wi-Fi!");
        ESP_LOGI(TAG, "IP Address: " IPSTR,
                 IP2STR(&event->ip_info.ip));
    }
}

esp_err_t wifi_manager_start(void)
{
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("esp_netif_handlers", ESP_LOG_WARN);
    // Initialize non-volatile storage.
    esp_err_t err = nvs_flash_init();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS initialization failed.");
        return err;
    }

    // Initialize TCP/IP and event handling.
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *netif =
        esp_netif_create_default_wifi_sta();

    if (netif == NULL) {
        return ESP_FAIL;
    }

    // Initialize Wi-Fi driver.
    wifi_init_config_t init_config =
        WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&init_config));

    // Keep this initial Wi-Fi configuration in RAM.
    ESP_ERROR_CHECK(
        esp_wifi_set_storage(WIFI_STORAGE_RAM)
    );

    // Register event handlers.
    ESP_ERROR_CHECK(
        esp_event_handler_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &wifi_event_handler,
            NULL
        )
    );

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &wifi_event_handler,
            NULL
        )
    );

    // Configure ESP32 as Wi-Fi Station.
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(
        esp_wifi_set_config(
            WIFI_IF_STA,
            &wifi_config
        )
    );

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi started. Waiting for IP...");

    return ESP_OK;
}
