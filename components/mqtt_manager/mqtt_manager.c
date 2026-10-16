
#include "mqtt_manager.h"
#include "mqtt_credentials.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "mqtt_client.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "cJSON.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

// MQTT topics and configuration limits.
#define CONFIG_SET_TOPIC "aai16/config/set"
#define CONFIG_ACK_TOPIC "aai16/config/ack"
#define DHT22_DATA_TOPIC "aai16/sensors/dht22/data"

#define CONFIG_QUEUE_SIZE 4
#define MQTT_CONNECTED_BIT BIT0

static const char *TAG = "AAI16_MQTT";

static esp_mqtt_client_handle_t s_client = NULL;
static bool s_started = false;

static QueueHandle_t s_config_queue = NULL;
static EventGroupHandle_t s_mqtt_events = NULL;


// --------------------------------------------------
// Send rejected configuration ACK
// --------------------------------------------------

static void mqtt_send_rejected_ack(const char *reason)
{
    if (s_client == NULL || reason == NULL) {
        return;
    }

    char payload[192];

    int len = snprintf(
        payload,
        sizeof(payload),
        "{\"sensor\":\"dht22\","
        "\"status\":\"rejected\","
        "\"reason\":\"%s\"}",
        reason
    );

    if (len < 0 || len >= (int)sizeof(payload)) {
        ESP_LOGE(TAG, "Rejected ACK payload too long.");
        return;
    }

    int msg_id = esp_mqtt_client_enqueue(
        s_client,
        CONFIG_ACK_TOPIC,
        payload,
        len,
        1,      // QoS 1
        0,      // Retain OFF
        true
    );

    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to queue rejected ACK.");
    }
}


// --------------------------------------------------
// MQTT event handler
// --------------------------------------------------

static void mqtt_event_handler(
    void *arg,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data)
{
    switch ((esp_mqtt_event_id_t)event_id) {

    // ----------------------------------------------
    // MQTT connected
    // ----------------------------------------------

    case MQTT_EVENT_CONNECTED: {
        ESP_LOGI(TAG, "Connected to MQTT Broker!");

        xEventGroupSetBits(
            s_mqtt_events,
            MQTT_CONNECTED_BIT
        );

        // Subscribe to runtime configuration.
        int sub_id = esp_mqtt_client_subscribe(
            s_client,
            CONFIG_SET_TOPIC,
            1
        );

        if (sub_id >= 0) {
            ESP_LOGI(
                TAG,
                "Configuration subscription requested."
            );
        } else {
            ESP_LOGE(
                TAG,
                "Configuration subscription failed."
            );
        }

        break;
    }

    // ----------------------------------------------
    // MQTT subscription acknowledged by broker
    // ----------------------------------------------

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(
            TAG,
            "Configuration subscription acknowledged."
        );
        break;

    // ----------------------------------------------
    // Incoming MQTT configuration
    // ----------------------------------------------

    case MQTT_EVENT_DATA: {
        esp_mqtt_event_handle_t event =
            (esp_mqtt_event_handle_t)event_data;

        // Ignore continuation fragments.
        if (event->current_data_offset != 0) {
            break;
        }

        // Only process configuration topic.
        if (event->topic == NULL ||
            event->topic_len !=
                (int)strlen(CONFIG_SET_TOPIC) ||
            memcmp(
                event->topic,
                CONFIG_SET_TOPIC,
                event->topic_len
            ) != 0) {
            break;
        }

        // Ignore retained configuration commands.
        if (event->retain) {
            ESP_LOGW(
                TAG,
                "Retained configuration ignored."
            );
            break;
        }

        // Limit payload size to 256 bytes.
        if (event->data == NULL ||
            event->data_len <= 0 ||
            event->data_len > 256 ||
            event->data_len != event->total_data_len) {

            ESP_LOGW(
                TAG,
                "Invalid configuration payload size."
            );

            mqtt_send_rejected_ack(
                "invalid_payload_size_or_fragmented"
            );

            break;
        }

        // Copy MQTT payload into a null-terminated
        // string before parsing JSON.
        char json_text[257];

        memcpy(
            json_text,
            event->data,
            (size_t)event->data_len
        );

        json_text[event->data_len] = '\0';

        // Reject embedded null bytes.
        if (memchr(
                event->data,
                '\0',
                (size_t)event->data_len
            ) != NULL) {

            mqtt_send_rejected_ack("invalid_json");
            break;
        }

        // Parse the entire JSON message.
        cJSON *root = cJSON_ParseWithOpts(
            json_text,
            NULL,
            1
        );

        if (!cJSON_IsObject(root)) {
            ESP_LOGW(TAG, "Invalid JSON received.");

            mqtt_send_rejected_ack("invalid_json");

            cJSON_Delete(root);
            break;
        }

        cJSON *sensor =
            cJSON_GetObjectItemCaseSensitive(
                root,
                "sensor"
            );

        cJSON *interval =
            cJSON_GetObjectItemCaseSensitive(
                root,
                "interval_ms"
            );

        // ------------------------------------------
        // Validate sensor field
        // ------------------------------------------

        if (!cJSON_IsString(sensor) ||
            sensor->valuestring == NULL) {

            mqtt_send_rejected_ack(
                "missing_or_invalid_sensor"
            );

            cJSON_Delete(root);
            break;
        }

        // This prototype currently supports DHT22.
        if (strcmp(
                sensor->valuestring,
                "dht22"
            ) != 0) {

            ESP_LOGW(TAG, "Unsupported sensor.");

            mqtt_send_rejected_ack(
                "unsupported_sensor"
            );

            cJSON_Delete(root);
            break;
        }

        // ------------------------------------------
        // Validate interval field
        // ------------------------------------------

        if (!cJSON_IsNumber(interval)) {

            mqtt_send_rejected_ack(
                "missing_or_invalid_interval_ms"
            );

            cJSON_Delete(root);
            break;
        }

        double value = interval->valuedouble;

        // Allowed DHT22 interval: 2000-10000 ms.
        // The value must also be an integer.
        if (!(value >= 2000 && value <= 10000) ||
            value != (double)(uint32_t)value) {

            ESP_LOGW(
                TAG,
                "Invalid sampling interval."
            );

            mqtt_send_rejected_ack(
                "interval_ms must be an integer from 2000 to 10000"
            );

            cJSON_Delete(root);
            break;
        }

        uint32_t requested_ms = (uint32_t)value;

        cJSON_Delete(root);

        // ------------------------------------------
        // Send valid configuration to sensor loop
        // ------------------------------------------

        if (s_config_queue == NULL ||
            xQueueSend(
                s_config_queue,
                &requested_ms,
                0
            ) != pdTRUE) {

            ESP_LOGW(
                TAG,
                "Configuration queue full."
            );

            mqtt_send_rejected_ack("queue_full");
            break;
        }

        ESP_LOGI(
            TAG,
            "Valid configuration queued: %lu ms",
            (unsigned long)requested_ms
        );

        break;
    }

    // ----------------------------------------------
    // MQTT disconnected
    // ----------------------------------------------

    case MQTT_EVENT_DISCONNECTED:
        xEventGroupClearBits(
            s_mqtt_events,
            MQTT_CONNECTED_BIT
        );

        ESP_LOGW(TAG, "MQTT disconnected.");
        break;

    // ----------------------------------------------
    // MQTT error
    // ----------------------------------------------

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT connection error.");
        break;

    default:
        break;
    }
}


// --------------------------------------------------
// Start MQTT after Wi-Fi obtains an IP address
// --------------------------------------------------

static void ip_event_handler(
    void *arg,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data)
{
    if (!s_started && s_client != NULL) {

        ESP_LOGI(
            TAG,
            "Wi-Fi ready. Starting MQTT..."
        );

        esp_err_t err =
            esp_mqtt_client_start(s_client);

        if (err == ESP_OK) {
            s_started = true;
        } else {
            ESP_LOGE(
                TAG,
                "MQTT start failed: %s",
                esp_err_to_name(err)
            );
        }
    }
}


// --------------------------------------------------
// Initialize MQTT manager
// --------------------------------------------------

esp_err_t mqtt_manager_init(void)
{
    // Create configuration queue.
    s_config_queue = xQueueCreate(
        CONFIG_QUEUE_SIZE,
        sizeof(uint32_t)
    );

    if (s_config_queue == NULL) {
        ESP_LOGE(
            TAG,
            "Failed to create config queue."
        );

        return ESP_ERR_NO_MEM;
    }

    // Create MQTT connection event group.
    s_mqtt_events = xEventGroupCreate();

    if (s_mqtt_events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // MQTT broker and credentials.
    esp_mqtt_client_config_t config = {
        .broker.address.uri = MQTT_BROKER_URI,

        .credentials.username = MQTT_USERNAME,

        .credentials.authentication.password =
            MQTT_PASSWORD,

        .credentials.client_id = "aai16-esp32-s3",
    };

    s_client = esp_mqtt_client_init(&config);

    if (s_client == NULL) {
        ESP_LOGE(
            TAG,
            "MQTT initialization failed."
        );

        return ESP_FAIL;
    }

    // Register MQTT events.
    esp_err_t err = esp_mqtt_client_register_event(
        s_client,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL
    );

    if (err != ESP_OK) {
        return err;
    }

    // Register Wi-Fi IP event.
    return esp_event_handler_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        ip_event_handler,
        NULL
    );
}


// --------------------------------------------------
// Publish DHT22 temperature and humidity
// --------------------------------------------------

esp_err_t mqtt_manager_publish_dht22(
    float temperature_c,
    float humidity_pct)
{
    // Check MQTT connection.
    if (s_client == NULL ||
        s_mqtt_events == NULL ||
        !(xEventGroupGetBits(s_mqtt_events) &
          MQTT_CONNECTED_BIT)) {

        return ESP_ERR_INVALID_STATE;
    }

    char payload[128];

    int len = snprintf(
        payload,
        sizeof(payload),
        "{\"sensor\":\"dht22\","
        "\"temperature_c\":%.1f,"
        "\"humidity_pct\":%.1f}",
        temperature_c,
        humidity_pct
    );

    if (len < 0 || len >= (int)sizeof(payload)) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Add telemetry to MQTT outbox.
    int msg_id = esp_mqtt_client_enqueue(
        s_client,
        DHT22_DATA_TOPIC,
        payload,
        len,
        0,      // QoS 0
        0,      // Retain OFF
        true
    );

    return msg_id >= 0 ? ESP_OK : ESP_FAIL;
}


// --------------------------------------------------
// Retrieve validated configuration from queue
// --------------------------------------------------

bool mqtt_manager_take_interval_update(
    uint32_t *out_ms)
{
    if (s_config_queue == NULL || out_ms == NULL) {
        return false;
    }

    return xQueueReceive(
        s_config_queue,
        out_ms,
        0
    ) == pdTRUE;
}


// --------------------------------------------------
// Publish applied configuration ACK
// --------------------------------------------------

esp_err_t mqtt_manager_publish_config_ack(
    uint32_t interval_ms)
{
    // Check MQTT connection.
    if (s_client == NULL ||
        s_mqtt_events == NULL ||
        !(xEventGroupGetBits(s_mqtt_events) &
          MQTT_CONNECTED_BIT)) {

        return ESP_ERR_INVALID_STATE;
    }

    char payload[128];

    int len = snprintf(
        payload,
        sizeof(payload),
        "{\"sensor\":\"dht22\","
        "\"status\":\"applied\","
        "\"interval_ms\":%lu}",
        (unsigned long)interval_ms
    );

    if (len < 0 || len >= (int)sizeof(payload)) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Add ACK to MQTT outbox.
    int msg_id = esp_mqtt_client_enqueue(
        s_client,
        CONFIG_ACK_TOPIC,
        payload,
        len,
        1,      // QoS 1
        0,      // Retain OFF
        true
    );

    return msg_id >= 0 ? ESP_OK : ESP_FAIL;
}
