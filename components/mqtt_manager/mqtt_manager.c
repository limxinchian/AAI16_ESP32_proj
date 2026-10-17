
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
#define RC522_EVENT_TOPIC "aai16/sensors/rc522/event"
#define LD2450_DATA_TOPIC "aai16/sensors/ld2450/data"

#define CONFIG_QUEUE_SIZE 4
#define MQTT_CONNECTED_BIT BIT0
#define TX_QUEUE_SIZE 8
#define TX_PAYLOAD_SIZE 320
#define TELEMETRY_MAX_AGE_MS 2000
// Three targets need more than the shared 320-byte queue payload. Keep that
// queue unchanged: serialize a single radar mailbox only in the publisher.
#define LD2450_PAYLOAD_SIZE 512
#define LD2450_PUBLISH_PERIOD_US 1000000LL
#define LD2450_MAX_AGE_US 2000000LL
// Keep the original 4096-byte allowance available to configuration ACKs/control
// traffic. Events may occupy at most an additional 1024 serialized bytes.
#define ACK_OUTBOX_RESERVE_BYTES 4096
#define EVENT_OUTBOX_BUDGET_BYTES 1024
#define OUTBOX_LIMIT_BYTES (ACK_OUTBOX_RESERVE_BYTES + EVENT_OUTBOX_BUDGET_BYTES)
#define EVENT_QUEUE_SIZE 8
#define EVENT_MAX_AGE_MS 60000
// Routine summaries: set to 10000 temporarily for a detailed hardware test.
// Connection/error events are logged immediately regardless of this interval.
#define DIAGNOSTIC_PERIOD_MS 60000
#define MQTT_NETWORK_TIMEOUT_MS 10000

typedef struct {
    sensor_id_t id;
    const char *name;
    uint32_t min_interval_ms;
    uint32_t max_interval_ms;
} config_rule_t;

// Application polling limits, independent of the sensors' internal data rates.
// Adding a configurable sensor requires a rule and an owning application route.
static const config_rule_t s_config_rules[] = {
    {SENSOR_DHT22, "dht22", 2000, 10000},
    {SENSOR_GY511, "gy511", 500, 10000},
    // Keep the verified 200 ms minimum; cap slow polling at 2 s because both
    // detection latency and the three-idle-poll removal filter scale with it.
    {SENSOR_RC522, "rc522", 200, 2000},
};

static const config_rule_t *config_rule_for_id(sensor_id_t id)
{
    for (unsigned i = 0; i < sizeof(s_config_rules) / sizeof(s_config_rules[0]); ++i) {
        if (s_config_rules[i].id == id) return &s_config_rules[i];
    }
    return NULL;
}

static const config_rule_t *config_rule_for_name(const char *name)
{
    for (unsigned i = 0; i < sizeof(s_config_rules) / sizeof(s_config_rules[0]); ++i) {
        if (strcmp(s_config_rules[i].name, name) == 0) return &s_config_rules[i];
    }
    return NULL;
}

static const char *TAG = "AAI16_MQTT";

static esp_mqtt_client_handle_t s_client = NULL;

static QueueHandle_t s_config_queue = NULL;
static EventGroupHandle_t s_mqtt_events = NULL;
static QueueHandle_t s_tx_queue = NULL;
static QueueHandle_t s_event_queue = NULL;
static uint32_t s_boot_id;
static TaskHandle_t s_tx_task;

typedef enum {
    TX_DHT22,
    TX_GY511,
    TX_CONFIG_ACK,
    TX_RC522_EVENT,
    TX_LD2450,
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
// SDK receipts contain packet IDs, not application kinds. Keep these totals
// explicitly shared; an RFID PUBACK must never be reported as a config ACK.
static uint32_t s_qos1_pubacked, s_outbox_deleted;
static uint32_t s_event_offline_buffered, s_event_outbox_waits;
static int s_outbox_peak, s_outbox_bytes;
static int64_t s_outbox_observed_ms, s_max_outbox_query_us;
static portMUX_TYPE s_stats_lock = portMUX_INITIALIZER_UNLOCKED;

// Protected by s_stats_lock. Only fixed-size copies/counters under the lock;
// the reader never waits for serialization, logging, queues or the MQTT SDK.
static struct {
    ld2450_report_t report;
    int64_t received_us;
    bool pending;
} s_radar;
static uint32_t s_radar_updates, s_radar_coalesced, s_radar_invalid, s_radar_deferred;
// Publisher-owned scheduling state, never touched by the UART reader.
static int64_t s_radar_next_publish_us, s_radar_next_failure_log_us;

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
    bool event = message->kind == TX_RC522_EVENT;
    QueueHandle_t queue = event ? s_event_queue : s_tx_queue;
    bool connected = mqtt_connected();
    if (queue == NULL || (!event && !connected)) {
        count_event(&stats->offline);
        return ESP_ERR_INVALID_STATE;
    }
    // Zero wait: acquisition tasks never take the ESP-MQTT API mutex.
    if (xQueueSend(queue, message, 0) != pdTRUE) {
        count_event(&stats->queue_full);
        return ESP_ERR_NO_MEM;
    }
    count_event(&stats->queued);
    if (event && !connected) count_event(&s_event_offline_buffered);
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

// Called only by the existing publisher. Keep the head in its queue until
// accepted/dropped, including across disconnects; no unbounded retry copies.
static void service_event_queue(void)
{
    tx_message_t message;
    if (xQueuePeek(s_event_queue, &message, 0) != pdTRUE) return;
    tx_stats_t *stats = &s_stats[message.kind];
    if (esp_timer_get_time() / 1000 - message.created_ms > EVENT_MAX_AGE_MS) {
        xQueueReceive(s_event_queue, &message, 0);
        count_event(&stats->stale);
        return;
    }
    // Existing telemetry/ACK work gets priority. Events never consume its slots.
    if (!mqtt_connected() || uxQueueMessagesWaiting(s_tx_queue)) return;
    int bytes = update_outbox_snapshot();
    // MQTT 3.1.1 PUBLISH: at most 5 fixed-header bytes + 2 topic-length bytes
    // + topic + 2 packet-ID bytes + payload. SDK admission checks payload only;
    // account for the complete packet here, conservatively, to protect ACKs.
    int packet_bound = message.len + (int)strlen(RC522_EVENT_TOPIC) + 9;
    if (bytes < 0 || bytes + packet_bound > EVENT_OUTBOX_BUDGET_BYTES) {
        count_event(&s_event_outbox_waits); // Deferred, not dropped.
        return;
    }
    // The SDK snapshot can wait on its API mutex. Recheck age and pending ACKs.
    if (esp_timer_get_time() / 1000 - message.created_ms > EVENT_MAX_AGE_MS) {
        xQueueReceive(s_event_queue, &message, 0);
        count_event(&stats->stale);
        return;
    }
    if (!mqtt_connected() || uxQueueMessagesWaiting(s_tx_queue)) return;
    int64_t start_us = esp_timer_get_time();
    int msg_id = esp_mqtt_client_enqueue(s_client, RC522_EVENT_TOPIC,
        message.payload, message.len, 1, 0, true);
    int64_t elapsed_us = esp_timer_get_time() - start_us;
    portENTER_CRITICAL(&s_stats_lock);
    if (elapsed_us > stats->max_tx_call_us) stats->max_tx_call_us = elapsed_us;
    if (msg_id >= 0) stats->enqueued++;
    else if (msg_id == -2) s_event_outbox_waits++;
    else stats->enqueue_failed++;
    portEXIT_CRITICAL(&s_stats_lock);
    // -2 is backpressure: retain the same event/sequence until room or local
    // expiry. Other enqueue failures drop this event and are counted above.
    if (msg_id != -2) xQueueReceive(s_event_queue, &message, 0);
    update_outbox_snapshot();
}

esp_err_t mqtt_manager_update_ld2450_snapshot(const ld2450_report_t *report,
                                             int64_t received_us)
{
    unsigned count = 0;
    if (report != NULL) {
        for (unsigned i = 0; i < LD2450_TARGETS; ++i) count += report->targets[i].present;
    }
    if (report == NULL || report->count != count || received_us < 0 ||
        received_us > esp_timer_get_time()) {
        count_event(&s_radar_invalid);
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_stats_lock);
    ++s_radar_updates;
    if (s_radar.pending) ++s_radar_coalesced;
    s_radar.report = *report;
    s_radar.received_us = received_us;
    s_radar.pending = true;
    portEXIT_CRITICAL(&s_stats_lock);
    return ESP_OK;
}

static int format_radar(char *payload, size_t capacity, const ld2450_report_t *report,
                        int64_t received_us, uint32_t seq)
{
    int len = snprintf(payload, capacity,
        "{\"sensor\":\"ld2450\",\"boot_id\":\"%08" PRIx32 "\",\"seq\":%" PRIu32
        ",\"uptime_ms\":%" PRId64 ",\"target_count\":%u,\"targets\":[",
        s_boot_id, seq, received_us / 1000, report->count);
    if (len < 0 || (size_t)len >= capacity) return -1;
    bool first = true;
    for (unsigned i = 0; i < LD2450_TARGETS; ++i) {
        const ld2450_target_t *t = &report->targets[i];
        if (!t->present) continue;
        unsigned distance = (unsigned)(sqrtf((float)t->x_mm * t->x_mm +
                                            (float)t->y_mm * t->y_mm) + 0.5f);
        int n = snprintf(payload + len, capacity - len,
            "%s{\"slot\":%u,\"x_mm\":%d,\"y_mm\":%d,\"distance_mm\":%u,"
            "\"speed_cm_s\":%d,\"resolution_mm\":%u}",
            first ? "" : ",", i + 1, t->x_mm, t->y_mm, distance,
            t->speed_cm_s, t->resolution_mm);
        if (n < 0 || (size_t)n >= capacity - len) return -1;
        len += n;
        first = false;
    }
    int n = snprintf(payload + len, capacity - len, "]}");
    return n < 0 || (size_t)n >= capacity - len ? -1 : len + n;
}

// Only the existing MQTT worker calls this. A blocked transport cannot hold
// the mailbox lock or stop UART reception. No replay/catch-up burst after stalls.
static void service_radar_snapshot(void)
{
    int64_t now = esp_timer_get_time();
    if (now < s_radar_next_publish_us) return;
    s_radar_next_publish_us = now + LD2450_PUBLISH_PERIOD_US;
    // Preserve all eight queue slots for their existing users. Defer radar if
    // any ACK/telemetry work is pending; the reader keeps replacing its snapshot.
    if (s_tx_queue != NULL && uxQueueMessagesWaiting(s_tx_queue)) {
        portENTER_CRITICAL(&s_stats_lock);
        if (s_radar.pending) ++s_radar_deferred;
        portEXIT_CRITICAL(&s_stats_lock);
        return;
    }
    ld2450_report_t report;
    int64_t received_us;
    portENTER_CRITICAL(&s_stats_lock);
    bool pending = s_radar.pending;
    report = s_radar.report;
    received_us = s_radar.received_us;
    s_radar.pending = false;
    portEXIT_CRITICAL(&s_stats_lock);
    if (!pending) return; // No fabricated initial data or repeated old samples.
    tx_stats_t *stats = &s_stats[TX_LD2450];
    uint32_t seq = begin_attempt(TX_LD2450);
    if (now - received_us >= LD2450_MAX_AGE_US) {
        count_event(&stats->stale);
        return;
    }
    if (!mqtt_connected()) {
        count_event(&stats->offline);
        return;
    }
    char payload[LD2450_PAYLOAD_SIZE];
    int len = format_radar(payload, sizeof(payload), &report, received_us, seq);
    if (len < 0) {
        count_event(&stats->invalid);
        return;
    }
    int64_t start_us = esp_timer_get_time();
    int result = esp_mqtt_client_publish(s_client, LD2450_DATA_TOPIC, payload, len, 0, 0);
    int64_t elapsed_us = esp_timer_get_time() - start_us;
    portENTER_CRITICAL(&s_stats_lock);
    if (elapsed_us > stats->max_tx_call_us) stats->max_tx_call_us = elapsed_us;
    if (result >= 0) ++stats->publish_ok;
    else ++stats->publish_failed;
    portEXIT_CRITICAL(&s_stats_lock);
    if (result < 0 && esp_timer_get_time() >= s_radar_next_failure_log_us) {
        ESP_LOGW(TAG, "%s publish failed (result=%d); see drop counters", LD2450_DATA_TOPIC, result);
        s_radar_next_failure_log_us = esp_timer_get_time() + 10000000LL;
    }
}

static void mqtt_tx_task(void *arg)
{
    (void)arg;
    static const char *const topics[TX_KIND_COUNT] = {
        DHT22_DATA_TOPIC, GY511_DATA_TOPIC, CONFIG_ACK_TOPIC, RC522_EVENT_TOPIC, LD2450_DATA_TOPIC
    };
    tx_message_t message;
    int64_t next_failure_log_us = 0;
    while (true) {
        service_radar_snapshot();
        if (xQueueReceive(s_tx_queue, &message, pdMS_TO_TICKS(1000)) != pdTRUE) {
            update_outbox_snapshot();
            service_event_queue();
            continue;
        }
        tx_stats_t *stats = &s_stats[message.kind];
        if (!mqtt_connected()) {
            count_event(&stats->offline);
            service_event_queue();
            continue;
        }
        bool telemetry = message.kind != TX_CONFIG_ACK;
        if (telemetry && esp_timer_get_time() / 1000 - message.created_ms >
                         TELEMETRY_MAX_AGE_MS) {
            count_event(&stats->stale);
            service_event_queue();
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
        service_event_queue();
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
    uint32_t connections, disconnects, errors, pubacked, deleted, event_offline, event_waits;
    uint32_t radar_updates, radar_coalesced, radar_invalid, radar_deferred;
    bool radar_pending;
    int peak, outbox_bytes;
    int64_t observed_ms, max_query_us;
    portENTER_CRITICAL(&s_stats_lock);
    memcpy(snapshot, s_stats, sizeof(snapshot));
    connections = s_connections;
    disconnects = s_disconnects;
    errors = s_errors;
    pubacked = s_qos1_pubacked;
    deleted = s_outbox_deleted;
    event_offline = s_event_offline_buffered;
    event_waits = s_event_outbox_waits;
    peak = s_outbox_peak;
    outbox_bytes = s_outbox_bytes;
    observed_ms = s_outbox_observed_ms;
    max_query_us = s_max_outbox_query_us;
    radar_updates = s_radar_updates;
    radar_coalesced = s_radar_coalesced;
    radar_invalid = s_radar_invalid;
    radar_deferred = s_radar_deferred;
    radar_pending = s_radar.pending;
    portEXIT_CRITICAL(&s_stats_lock);
    ESP_LOGI(TAG, "MQTT summary | boot=%08" PRIx32 " uptime_ms=%" PRId64
             " connected=%d tx_pending=%u",
             s_boot_id, now_ms, mqtt_connected(),
             s_tx_queue ? (unsigned)uxQueueMessagesWaiting(s_tx_queue) : 0);
    ESP_LOGI(TAG, "  connects=%" PRIu32 " disconnects=%" PRIu32 " errors=%" PRIu32
             " qos1_pubacked_total=%" PRIu32 " outbox_deleted=%" PRIu32,
             connections, disconnects, errors, pubacked, deleted);
    ESP_LOGI(TAG, "  outbox_observed_bytes=%d peak_observed=%d age_ms=%" PRId64
             " max_query_us=%" PRId64, outbox_bytes, peak,
             now_ms > observed_ms ? now_ms - observed_ms : 0, max_query_us);
    ESP_LOGI(TAG, "  heap_free=%" PRIu32 " heap_min=%" PRIu32 " tx_stack_free_min=%u B",
             esp_get_free_heap_size(),
             esp_get_minimum_free_heap_size(),
             s_tx_task ? (unsigned)uxTaskGetStackHighWaterMark(s_tx_task) : 0);
    ESP_LOGI(TAG, "  events_pending=%u offline_buffered=%" PRIu32 " outbox_waits=%" PRIu32
             " local_max_age_ms=%d ack_reserve_bytes=%d",
             s_event_queue ? (unsigned)uxQueueMessagesWaiting(s_event_queue) : 0,
             event_offline, event_waits, EVENT_MAX_AGE_MS, ACK_OUTBOX_RESERVE_BYTES);
    ESP_LOGI(TAG, "  radar_snapshot_updates=%" PRIu32 " coalesced=%" PRIu32
             " invalid_updates=%" PRIu32 " busy_deferrals=%" PRIu32 " pending=%d period_ms=1000 max_age_ms=2000",
             radar_updates, radar_coalesced, radar_invalid, radar_deferred, radar_pending);
    static const char *const names[] = {"dht22", "gy511", "config_ack", "rc522_event", "ld2450"};
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

static esp_err_t mqtt_send_rejected_ack(const char *sensor, const char *reason,
                                       const double *interval, const bool *enabled)
{
    if (reason == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    begin_attempt(TX_CONFIG_ACK);
    // Escape untrusted sensor IDs (including unsupported names). Malformed JSON
    // has no trustworthy target, so report JSON null rather than inventing one.
    char sensor_json[262] = "null", reason_json[128], interval_json[64] = "";
    char enabled_json[24] = "";
    cJSON sensor_value = {.type = cJSON_String, .valuestring = (char *)sensor};
    cJSON reason_value = {.type = cJSON_String, .valuestring = (char *)reason};
    if ((sensor != NULL && !cJSON_PrintPreallocated(&sensor_value, sensor_json,
                                                   sizeof(sensor_json), false)) ||
        !cJSON_PrintPreallocated(&reason_value, reason_json, sizeof(reason_json), false)) {
        count_event(&s_stats[TX_CONFIG_ACK].invalid);
        return ESP_ERR_INVALID_SIZE;
    }
    if (interval != NULL && isfinite(*interval)) {
        snprintf(interval_json, sizeof(interval_json), ",\"interval_ms\":%.17g", *interval);
    }
    if (enabled != NULL) {
        snprintf(enabled_json, sizeof(enabled_json), ",\"enabled\":%s", *enabled ? "true" : "false");
    }
    tx_message_t message = {.kind = TX_CONFIG_ACK};
    message.len = snprintf(
        message.payload,
        sizeof(message.payload),
        "{\"sensor\":%s,"
        "\"status\":\"rejected\","
        "\"reason\":%s%s%s}",
        sensor_json, reason_json, interval_json, enabled_json
    );

    // Failures are counted and reported by the rate-limited diagnostics.
    return queue_message(&message);
}

esp_err_t mqtt_manager_reject_config(const mqtt_config_update_t *update,
                                   const char *reason)
{
    if (update == NULL || (update->operation != MQTT_CONFIG_INTERVAL &&
                           update->operation != MQTT_CONFIG_ENABLED)) return ESP_ERR_INVALID_ARG;
    const config_rule_t *rule = config_rule_for_id(update->sensor);
    if (rule == NULL) return ESP_ERR_INVALID_ARG;
    double interval = update->operation == MQTT_CONFIG_INTERVAL ? update->interval_ms : 0;
    return mqtt_send_rejected_ack(rule->name, reason,
        update->operation == MQTT_CONFIG_INTERVAL ? &interval : NULL,
        update->operation == MQTT_CONFIG_ENABLED ? &update->enabled : NULL);
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
                NULL, "invalid_payload_size_or_fragmented", NULL, NULL
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

            mqtt_send_rejected_ack(NULL, "invalid_json", NULL, NULL);
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

            mqtt_send_rejected_ack(NULL, "invalid_json", NULL, NULL);

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

        cJSON *enabled = cJSON_GetObjectItemCaseSensitive(root, "enabled");
        unsigned sensor_fields = 0, operation_fields = 0;
        const cJSON *field;
        cJSON_ArrayForEach(field, root) {
            if (strcmp(field->string, "sensor") == 0) sensor_fields++;
            if (strcmp(field->string, "interval_ms") == 0 ||
                strcmp(field->string, "enabled") == 0) operation_fields++;
        }

        // ------------------------------------------
        // Validate sensor field
        // ------------------------------------------

        if (sensor_fields != 1 || !cJSON_IsString(sensor) ||
            sensor->valuestring == NULL) {

            mqtt_send_rejected_ack(
                NULL, "missing_or_invalid_sensor", NULL, NULL
            );

            cJSON_Delete(root);
            break;
        }

        const config_rule_t *rule = config_rule_for_name(sensor->valuestring);
        const double *requested_value = cJSON_IsNumber(interval) ? &interval->valuedouble : NULL;
        bool requested_enabled = cJSON_IsTrue(enabled);
        const bool *requested_gate = cJSON_IsBool(enabled) ? &requested_enabled : NULL;
        if (rule == NULL) {

            ESP_LOGW(TAG, "Unsupported sensor.");

            mqtt_send_rejected_ack(
                sensor->valuestring, "unsupported_sensor", requested_value, requested_gate
            );

            cJSON_Delete(root);
            break;
        }

        // ------------------------------------------
        // Exactly one operation, including rejecting duplicate operation keys.
        // ------------------------------------------

        if (operation_fields != 1) {
            mqtt_send_rejected_ack(rule->name, operation_fields == 0 ?
                "missing_configuration_operation" : "ambiguous_configuration",
                requested_value, requested_gate);
            cJSON_Delete(root);
            break;
        }

        mqtt_config_update_t update = {.sensor = rule->id};
        if (enabled != NULL) {
            if (!cJSON_IsBool(enabled)) {
                mqtt_send_rejected_ack(rule->name, "enabled must be a boolean", NULL, NULL);
                cJSON_Delete(root);
                break;
            }
            update.operation = MQTT_CONFIG_ENABLED;
            update.enabled = requested_enabled;
        } else {
            if (!cJSON_IsNumber(interval)) {
                mqtt_send_rejected_ack(rule->name, "missing_or_invalid_interval_ms", NULL, NULL);
                cJSON_Delete(root);
                break;
            }
            double value = interval->valuedouble;
            // Check bounds before converting, including non-finite JSON numbers.
            if (!(value >= rule->min_interval_ms && value <= rule->max_interval_ms) ||
                value != (double)(uint32_t)value) {
                char reason[96];
                snprintf(reason, sizeof(reason), "interval_ms must be an integer from %lu to %lu",
                         (unsigned long)rule->min_interval_ms, (unsigned long)rule->max_interval_ms);
                mqtt_send_rejected_ack(rule->name, reason, &value, NULL);
                cJSON_Delete(root);
                break;
            }
            update.operation = MQTT_CONFIG_INTERVAL;
            update.interval_ms = (uint32_t)value;
        }

        cJSON_Delete(root);

        // ------------------------------------------
        // Send valid configuration to sensor loop
        // ------------------------------------------

        if (s_config_queue == NULL ||
            xQueueSend(
                s_config_queue,
                &update,
                0
            ) != pdTRUE) {

            ESP_LOGW(
                TAG,
                "Configuration queue full."
            );

            mqtt_manager_reject_config(&update, "queue_full");
            break;
        }

        if (update.operation == MQTT_CONFIG_ENABLED) {
            ESP_LOGI(TAG, "%s configuration queued: enabled=%s",
                     rule->name, update.enabled ? "true" : "false");
        } else {
            ESP_LOGI(TAG, "%s configuration queued: %lu ms",
                     rule->name, (unsigned long)update.interval_ms);
        }

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

    // Shared total for config ACKs and RFID events. QoS 0 has no broker receipt;
    // PUBACK also does not prove that any subscriber received the publication.
    case MQTT_EVENT_PUBLISHED:
        count_event(&s_qos1_pubacked);
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
        sizeof(mqtt_config_update_t)
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
    s_event_queue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(tx_message_t));
    if (s_event_queue == NULL) {
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
    ESP_LOGI(TAG, "Event publisher: QoS1 retain=0 slots=%d local_max_age_ms=%d "
             "outbox_budget=%d ack_reserve=%d; PUBACK/SDK expiry counters are shared",
             EVENT_QUEUE_SIZE, EVENT_MAX_AGE_MS, EVENT_OUTBOX_BUDGET_BYTES, ACK_OUTBOX_RESERVE_BYTES);
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
    if (s_event_queue != NULL) {
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
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

esp_err_t mqtt_manager_publish_rc522_event(const uint8_t *uid, uint8_t uid_length)
{
    uint32_t seq = begin_attempt(TX_RC522_EVENT);
    if (uid == NULL || (uid_length != 4 && uid_length != 7 && uid_length != 10)) {
        count_event(&s_stats[TX_RC522_EVENT].invalid);
        return ESP_ERR_INVALID_ARG;
    }
    static const char hex[] = "0123456789ABCDEF";
    char uid_hex[21];
    for (unsigned i = 0; i < uid_length; ++i) {
        uid_hex[2 * i] = hex[uid[i] >> 4];
        uid_hex[2 * i + 1] = hex[uid[i] & 0x0f];
    }
    uid_hex[2 * uid_length] = '\0';
    tx_message_t message = {.kind = TX_RC522_EVENT, .created_ms = esp_timer_get_time() / 1000};
    message.len = snprintf(message.payload, sizeof(message.payload),
        "{\"sensor\":\"rc522\",\"event\":\"card_presented\",\"uid_hex\":\"%s\",\"uid_length\":%u,"
        "\"boot_id\":\"%08" PRIx32 "\",\"seq\":%" PRIu32 ",\"uptime_ms\":%" PRId64 "}",
        uid_hex, (unsigned)uid_length, s_boot_id, seq, message.created_ms);
    return queue_message(&message);
}


// --------------------------------------------------
// Retrieve validated configuration from queue
// --------------------------------------------------

bool mqtt_manager_peek_config_update(mqtt_config_update_t *out)
{
    return s_config_queue != NULL && out != NULL &&
           xQueuePeek(s_config_queue, out, 0) == pdTRUE;
}

bool mqtt_manager_take_config_update(mqtt_config_update_t *out)
{
    return s_config_queue != NULL && out != NULL &&
           xQueueReceive(s_config_queue, out, 0) == pdTRUE;
}


// --------------------------------------------------
// Publish applied configuration ACK
// --------------------------------------------------

esp_err_t mqtt_manager_publish_config_ack(const mqtt_config_update_t *update)
{
    if (update == NULL) return ESP_ERR_INVALID_ARG;
    const config_rule_t *rule = config_rule_for_id(update->sensor);
    if (rule == NULL) return ESP_ERR_INVALID_ARG;
    char value[64];
    if (update->operation == MQTT_CONFIG_ENABLED) {
        snprintf(value, sizeof(value), "\"enabled\":%s", update->enabled ? "true" : "false");
    } else if (update->operation == MQTT_CONFIG_INTERVAL) {
        snprintf(value, sizeof(value), "\"interval_ms\":%lu", (unsigned long)update->interval_ms);
    } else {
        return ESP_ERR_INVALID_ARG;
    }
    begin_attempt(TX_CONFIG_ACK);
    tx_message_t message = {.kind = TX_CONFIG_ACK};
    message.len = snprintf(
        message.payload,
        sizeof(message.payload),
        "{\"sensor\":\"%s\","
        "\"status\":\"applied\","
        "%s}",
        rule->name,
        value
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
