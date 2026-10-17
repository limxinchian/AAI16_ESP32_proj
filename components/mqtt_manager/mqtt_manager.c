
#include "mqtt_manager.h"
#include "mqtt_credentials.h"

#include <stdbool.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "mqtt_client.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_system.h"

#include "cJSON.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// MQTT topics and configuration limits.
#define CONFIG_SET_TOPIC "aai16/config/set"
#define CONFIG_ACK_TOPIC "aai16/config/ack"

#define DHT22_DATA_TOPIC "aai16/sensors/dht22/data"
#define GY511_DATA_TOPIC "aai16/sensors/gy511/data"

#define CONFIG_QUEUE_SIZE 4
#define MQTT_CONNECTED_BIT BIT0
#define TX_QUEUE_SIZE 8
#define TX_PAYLOAD_SIZE 320
#define TELEMETRY_MAX_AGE_MS 2000
#define OUTBOX_LIMIT_BYTES 4096
// Routine summaries: set to 10000 temporarily for a detailed hardware test.
// Connection/error events are logged immediately regardless of this interval.
#define DIAGNOSTIC_PERIOD_MS 60000
#define MQTT_NETWORK_TIMEOUT_MS 10000

static const char *TAG = "AAI16_MQTT";

static esp_mqtt_client_handle_t s_client = NULL;

static QueueHandle_t s_config_queue = NULL;
static EventGroupHandle_t s_mqtt_events = NULL;
static QueueHandle_t s_tx_queue = NULL;
static uint32_t s_boot_id;
static TaskHandle_t s_tx_task;

typedef enum {
    TX_DHT22,
    TX_GY511,
    TX_CONFIG_ACK,
    TX_KIND_COUNT
} tx_kind_t;

typedef struct {
    tx_kind_t kind;
    int len;
    int64_t created_ms;
    char payload[TX_PAYLOAD_SIZE];
} tx_message_t;

typedef struct {
    uint32_t attempts;
    uint32_t queued;
    uint32_t offline;
    uint32_t queue_full;
    uint32_t invalid;
    uint32_t stale;
    uint32_t outbox_full;
    uint32_t enqueue_failed;
    uint32_t enqueued;
    uint32_t publish_ok;
    uint32_t publish_failed;
    int64_t max_tx_call_us;
} tx_stats_t;

static tx_stats_t s_stats[TX_KIND_COUNT];
static uint32_t s_connections, s_disconnects, s_errors;
static uint32_t s_ack_pubacked, s_outbox_deleted;
static int s_outbox_peak, s_outbox_bytes;
static int64_t s_outbox_observed_ms, s_max_outbox_query_us;
static portMUX_TYPE s_stats_lock = portMUX_INITIALIZER_UNLOCKED;

// Only small counter operations occur under this spinlock. Never call MQTT,
// FreeRTOS queues or logging while holding it (MQTT callbacks hold an SDK lock).
static void count_event(uint32_t *counter)
{
    portENTER_CRITICAL(&s_stats_lock);
    ++*counter;
    portEXIT_CRITICAL(&s_stats_lock);
}

static uint32_t begin_attempt(tx_kind_t kind)
{
    portENTER_CRITICAL(&s_stats_lock);
    uint32_t seq = ++s_stats[kind].attempts;
    portEXIT_CRITICAL(&s_stats_lock);
    return seq;
}

static bool mqtt_connected(void)
{
    return s_mqtt_events != NULL &&
           (xEventGroupGetBits(s_mqtt_events) & MQTT_CONNECTED_BIT) != 0;
}

static esp_err_t queue_message(const tx_message_t *message)
{
    tx_stats_t *stats = &s_stats[message->kind];
    if (message->len < 0 || message->len >= (int)sizeof(message->payload)) {
        count_event(&stats->invalid);
        return ESP_ERR_INVALID_SIZE;
    }
    if (s_tx_queue == NULL || !mqtt_connected()) {
        count_event(&stats->offline);
        return ESP_ERR_INVALID_STATE;
    }
    // Zero wait: acquisition tasks never take the ESP-MQTT API mutex.
    if (xQueueSend(s_tx_queue, message, 0) != pdTRUE) {
        count_event(&stats->queue_full);
        return ESP_ERR_NO_MEM;
    }
    count_event(&stats->queued);
    return ESP_OK; // Local queue acceptance only; not an MQTT delivery receipt.
}

// The SDK getter also takes its API mutex: ONLY the publisher may call it.
static int update_outbox_snapshot(void)
{
    int64_t start_us = esp_timer_get_time();
    int bytes = esp_mqtt_client_get_outbox_size(s_client);
    int64_t end_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_stats_lock);
    s_outbox_bytes = bytes;
    s_outbox_observed_ms = end_us / 1000;
    if (bytes > s_outbox_peak) s_outbox_peak = bytes;
    if (end_us - start_us > s_max_outbox_query_us) {
        s_max_outbox_query_us = end_us - start_us;
    }
    portEXIT_CRITICAL(&s_stats_lock);
    return bytes;
}

static void mqtt_tx_task(void *arg)
{
    (void)arg;
    static const char *const topics[TX_KIND_COUNT] = {
        DHT22_DATA_TOPIC, GY511_DATA_TOPIC, CONFIG_ACK_TOPIC
    };
    tx_message_t message;
    int64_t next_failure_log_us = 0;
    while (true) {
        if (xQueueReceive(s_tx_queue, &message, pdMS_TO_TICKS(1000)) != pdTRUE) {
            update_outbox_snapshot();
            continue;
        }
        tx_stats_t *stats = &s_stats[message.kind];
        if (!mqtt_connected()) {
            count_event(&stats->offline);
            continue;
        }
        bool telemetry = message.kind != TX_CONFIG_ACK;
        if (telemetry && esp_timer_get_time() / 1000 - message.created_ms >
                         TELEMETRY_MAX_AGE_MS) {
            count_event(&stats->stale);
            continue;
        }
        int64_t start_us = esp_timer_get_time();
        // In the installed SDK, stored QoS 0 drains one item per MQTT loop,
        // followed by an idle read poll of up to 1 s. Our sensor rate exceeds
        // that drain rate. The application queue already decouples acquisition:
        // send QoS 0 here, without a second queue or outbox admission check.
        // enqueue(qos=0, store=false) is NOT an alternative: it sends nothing.
        int msg_id = telemetry
            ? esp_mqtt_client_publish(s_client, topics[message.kind],
                message.payload, message.len, 0, 0)
            : esp_mqtt_client_enqueue(s_client, topics[message.kind],
                message.payload, message.len, 1, 0, true);
        int64_t elapsed_us = esp_timer_get_time() - start_us;
        portENTER_CRITICAL(&s_stats_lock);
        if (elapsed_us > stats->max_tx_call_us) {
            stats->max_tx_call_us = elapsed_us;
        }
        if (telemetry) {
            // QoS 0 success is a local transport write, NOT a broker PUBACK.
            if (msg_id >= 0) stats->publish_ok++;
            else stats->publish_failed++;
        } else if (msg_id >= 0) {
            stats->enqueued++;
        } else if (msg_id == -2) {
            stats->outbox_full++;
        } else {
            stats->enqueue_failed++;
        }
        portEXIT_CRITICAL(&s_stats_lock);
        if (msg_id < 0 && esp_timer_get_time() >= next_failure_log_us) {
            ESP_LOGW(TAG, "%s %s failed (result=%d); see drop counters",
                     topics[message.kind], telemetry ? "publish" : "enqueue", msg_id);
            // First failure immediately; repeated failures at most every 10 s.
            next_failure_log_us = esp_timer_get_time() + 10000000LL;
        }
        update_outbox_snapshot();
    }
}

// Called only by app_main, even when the publisher is blocked on the SDK lock.
void mqtt_manager_log_diagnostics(void)
{
    static int64_t next_log_ms;
    int64_t now_ms = esp_timer_get_time() / 1000;
    if (now_ms < next_log_ms) {
        return;
    }
    next_log_ms = now_ms + DIAGNOSTIC_PERIOD_MS;
    tx_stats_t snapshot[TX_KIND_COUNT];
    uint32_t connections, disconnects, errors, pubacked, deleted;
    int peak, outbox_bytes;
    int64_t observed_ms, max_query_us;
    portENTER_CRITICAL(&s_stats_lock);
    memcpy(snapshot, s_stats, sizeof(snapshot));
    connections = s_connections;
    disconnects = s_disconnects;
    errors = s_errors;
    pubacked = s_ack_pubacked;
    deleted = s_outbox_deleted;
    peak = s_outbox_peak;
    outbox_bytes = s_outbox_bytes;
    observed_ms = s_outbox_observed_ms;
    max_query_us = s_max_outbox_query_us;
    portEXIT_CRITICAL(&s_stats_lock);
    ESP_LOGI(TAG, "MQTT summary | boot=%08" PRIx32 " uptime_ms=%" PRId64
             " connected=%d tx_pending=%u",
             s_boot_id, now_ms, mqtt_connected(),
             s_tx_queue ? (unsigned)uxQueueMessagesWaiting(s_tx_queue) : 0);
    ESP_LOGI(TAG, "  connects=%" PRIu32 " disconnects=%" PRIu32 " errors=%" PRIu32
             " ack_pubacked=%" PRIu32 " outbox_deleted=%" PRIu32,
             connections, disconnects, errors, pubacked, deleted);
    ESP_LOGI(TAG, "  outbox_observed_bytes=%d peak_observed=%d age_ms=%" PRId64
             " max_query_us=%" PRId64, outbox_bytes, peak,
             now_ms > observed_ms ? now_ms - observed_ms : 0, max_query_us);
    ESP_LOGI(TAG, "  heap_free=%" PRIu32 " heap_min=%" PRIu32 " tx_stack_free_min=%u B",
             esp_get_free_heap_size(),
             esp_get_minimum_free_heap_size(),
             s_tx_task ? (unsigned)uxTaskGetStackHighWaterMark(s_tx_task) : 0);
    static const char *const names[] = {"dht22", "gy511", "config_ack"};
    for (int i = 0; i < TX_KIND_COUNT; ++i) {
        const tx_stats_t *s = &snapshot[i];
        ESP_LOGI(TAG, "%s attempts=%" PRIu32 " queued=%" PRIu32
                 " publish_ok=%" PRIu32 " enqueued=%" PRIu32 " max_tx_call_us=%" PRId64,
                 names[i], s->attempts, s->queued, s->publish_ok, s->enqueued,
                 s->max_tx_call_us);
        ESP_LOGI(TAG, "  drops: offline=%" PRIu32 " queue_full=%" PRIu32
                 " invalid=%" PRIu32 " stale=%" PRIu32 " outbox_full=%" PRIu32
                 " enqueue_failed=%" PRIu32 " publish_failed=%" PRIu32,
                 s->offline,
                 s->queue_full, s->invalid, s->stale, s->outbox_full,
                 s->enqueue_failed, s->publish_failed);
    }
}


// --------------------------------------------------
// Send rejected configuration ACK
// --------------------------------------------------

static esp_err_t mqtt_send_rejected_ack(const char *reason)
{
    if (reason == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    begin_attempt(TX_CONFIG_ACK);
    tx_message_t message = {.kind = TX_CONFIG_ACK};
    message.len = snprintf(
        message.payload,
        sizeof(message.payload),
        "{\"sensor\":\"dht22\","
        "\"status\":\"rejected\","
        "\"reason\":\"%s\"}",
        reason
    );

    // Failures are counted and reported by the rate-limited diagnostics.
    return queue_message(&message);
}

esp_err_t mqtt_manager_reject_config_unavailable(void)
{
    // Current runtime configuration targets DHT22 only. Preserve the same
    // rejection schema and QoS 1 queue/outbox path as validation rejections.
    return mqtt_send_rejected_ack("sensor_unavailable");
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
        count_event(&s_connections);
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
        count_event(&s_disconnects);
        xEventGroupClearBits(
            s_mqtt_events,
            MQTT_CONNECTED_BIT
        );

        ESP_LOGW(TAG, "MQTT disconnected.");
        break;

    // ----------------------------------------------
    // MQTT error
    // ----------------------------------------------

    case MQTT_EVENT_ERROR: {
        count_event(&s_errors);
        esp_mqtt_event_handle_t event = event_data;
        if (event->error_handle != NULL) {
            ESP_LOGE(TAG, "MQTT error type=%d socket_errno=%d connack=%d",
                     event->error_handle->error_type,
                     event->error_handle->esp_transport_sock_errno,
                     event->error_handle->connect_return_code);
        }
        break;
    }

    // Only QoS 1 configuration ACKs generate this event. QoS 0 telemetry has
    // no broker receipt. PUBACK also does not prove any subscriber received it.
    case MQTT_EVENT_PUBLISHED:
        count_event(&s_ack_pubacked);
        break;

    case MQTT_EVENT_DELETED:
        count_event(&s_outbox_deleted);
        break;

    default:
        break;
    }
}


// --------------------------------------------------
// Initialize MQTT manager
// --------------------------------------------------

esp_err_t mqtt_manager_init(void)
{
    // Call once, from app_main, before starting sensor publisher tasks.
    if (s_client != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = ESP_ERR_NO_MEM;
    s_boot_id = esp_random();
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
        goto fail;
    }
    s_tx_queue = xQueueCreate(TX_QUEUE_SIZE, sizeof(tx_message_t));
    if (s_tx_queue == NULL) {
        goto fail;
    }

    // MQTT broker and credentials.
    esp_mqtt_client_config_t config = {
        .broker.address.uri = MQTT_BROKER_URI,

        .credentials.username = MQTT_USERNAME,

        .credentials.authentication.password =
            MQTT_PASSWORD,

        .credentials.client_id = "aai16-esp32-s3",
        .outbox.limit = OUTBOX_LIMIT_BYTES,
        .network.reconnect_timeout_ms = 5000,
        // Preserve the SDK's finite default per-transport-operation timeout.
        // The SDK API mutex has no public timed-lock option; only this worker
        // performs potentially blocking publication, never a sensor task.
        .network.timeout_ms = MQTT_NETWORK_TIMEOUT_MS,
        .network.disable_auto_reconnect = false,
    };

    s_client = esp_mqtt_client_init(&config);

    if (s_client == NULL) {
        ESP_LOGE(
            TAG,
            "MQTT initialization failed."
        );

        goto fail;
    }

    // Register MQTT events.
    err = esp_mqtt_client_register_event(
        s_client,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL
    );

    if (err != ESP_OK) {
        goto fail;
    }

    // Start once; ESP-MQTT retries if Wi-Fi is not ready yet. This avoids
    // losing GOT_IP between wifi_manager_start() and handler registration.
    err = esp_mqtt_client_start(s_client);
    if (err != ESP_OK) {
        goto fail;
    }
    if (xTaskCreate(mqtt_tx_task, "mqtt_tx", 4096, NULL, 3, &s_tx_task) != pdPASS) {
        esp_mqtt_client_stop(s_client);
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    ESP_LOGI(TAG, "Publisher ready: boot=%08" PRIx32
             " tx_slots=%d outbox_limit=%d diagnostics_ms=%d;"
             " QoS0 publish success is not a broker receipt",
             s_boot_id, TX_QUEUE_SIZE, OUTBOX_LIMIT_BYTES, DIAGNOSTIC_PERIOD_MS);
#if !CONFIG_MQTT_REPORT_DELETED_MESSAGES
    ESP_LOGW(TAG, "Outbox expiry events disabled in sdkconfig");
#endif
    return ESP_OK;

fail:
    if (s_client != NULL) {
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
    }
    if (s_tx_queue != NULL) {
        vQueueDelete(s_tx_queue);
        s_tx_queue = NULL;
    }
    if (s_mqtt_events != NULL) {
        vEventGroupDelete(s_mqtt_events);
        s_mqtt_events = NULL;
    }
    vQueueDelete(s_config_queue);
    s_config_queue = NULL;
    return err;
}


// --------------------------------------------------
// Publish DHT22 temperature and humidity
// --------------------------------------------------

esp_err_t mqtt_manager_publish_dht22(
    float temperature_c,
    float humidity_pct)
{
    uint32_t seq = begin_attempt(TX_DHT22);
    if (!isfinite(temperature_c) || !isfinite(humidity_pct)) {
        count_event(&s_stats[TX_DHT22].invalid);
        return ESP_ERR_INVALID_ARG;
    }
    tx_message_t message = {
        .kind = TX_DHT22,
        .created_ms = esp_timer_get_time() / 1000,
    };
    message.len = snprintf(
        message.payload,
        sizeof(message.payload),
        "{\"sensor\":\"dht22\","
        "\"boot_id\":\"%08" PRIx32 "\",\"seq\":%" PRIu32 ","
        "\"uptime_ms\":%" PRId64 ","
        "\"temperature_c\":%.1f,"
        "\"humidity_pct\":%.1f}",
        s_boot_id, seq, message.created_ms,
        temperature_c,
        humidity_pct
    );

    return queue_message(&message);
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
    begin_attempt(TX_CONFIG_ACK);
    tx_message_t message = {.kind = TX_CONFIG_ACK};
    message.len = snprintf(
        message.payload,
        sizeof(message.payload),
        "{\"sensor\":\"dht22\","
        "\"status\":\"applied\","
        "\"interval_ms\":%lu}",
        (unsigned long)interval_ms
    );

    return queue_message(&message);
}


esp_err_t mqtt_manager_publish_gy511(
    float accel_x_g,
    float accel_y_g,
    float accel_z_g,
    float mag_x_ut,
    float mag_y_ut,
    float mag_z_ut)
{
    uint32_t seq = begin_attempt(TX_GY511);
    if (!isfinite(accel_x_g) || !isfinite(accel_y_g) || !isfinite(accel_z_g) ||
        !isfinite(mag_x_ut) || !isfinite(mag_y_ut) || !isfinite(mag_z_ut)) {
        count_event(&s_stats[TX_GY511].invalid);
        return ESP_ERR_INVALID_ARG;
    }
    tx_message_t message = {
        .kind = TX_GY511,
        .created_ms = esp_timer_get_time() / 1000,
    };

    // Format GY-511 readings as JSON.
    message.len = snprintf(
        message.payload,
        sizeof(message.payload),
        "{\"sensor\":\"gy511\","
        "\"boot_id\":\"%08" PRIx32 "\",\"seq\":%" PRIu32 ","
        "\"uptime_ms\":%" PRId64 ","
        "\"acceleration_g\":{"
        "\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
        "\"magnetic_field_ut\":{"
        "\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}}",
        s_boot_id, seq, message.created_ms,
        accel_x_g,
        accel_y_g,
        accel_z_g,
        mag_x_ut,
        mag_y_ut,
        mag_z_ut
    );

    return queue_message(&message);
}
