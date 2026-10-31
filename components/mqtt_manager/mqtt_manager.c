
#include "mqtt_manager.h"
#include "motion_core.h"
#include "motion_backend.h"
#include "mhz1911a_sensor.h"
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

#include "config_transactions.h"
#include "model_upload.h"

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
#define HCSR04P_DATA_TOPIC "aai16/sensors/hcsr04p/data"
#define MHZ1911A_DATA_TOPIC "aai16/sensors/mhz1911a/data"
#define BME280_DATA_TOPIC "aai16/sensors/bme280/data"
#define MAX6675_DATA_TOPIC "aai16/sensors/max6675/data"
#define MOTION_DATA_TOPIC "aai16/ml/motion/data"

#define CONFIG_QUEUE_SIZE 4
#define MQTT_CONNECTED_BIT BIT0
#define TX_QUEUE_SIZE 8
#define TX_PAYLOAD_SIZE 320
// ML uses protocol_version=1, boot_id, generation, and the ml_prediction body.
// Include both the two-byte body tag and its bounded two-byte length prefix.
_Static_assert(2 + 5 + 5 + 2 + 2 + aai16_MlPrediction_size < TX_PAYLOAD_SIZE,
               "ML envelope exceeds telemetry queue capacity");
#define TELEMETRY_MAX_AGE_MS 2000
// Three targets need more than the shared 320-byte queue payload. Keep that
// queue unchanged: serialize a single radar mailbox only in the publisher.
#define LD2450_PAYLOAD_SIZE 192
_Static_assert(36 + aai16_SensorTelemetry_size <= LD2450_PAYLOAD_SIZE, "Radar telemetry exceeds its payload buffer");
#define LD2450_DEFAULT_INTERVAL_MS 1000
#define LD2450_MAX_AGE_US 2000000LL
#define HCSR04P_PAYLOAD_SIZE 160
#define HCSR04P_PUBLISH_INTERVAL_US 1000000LL
#define HCSR04P_MAX_AGE_US (TELEMETRY_MAX_AGE_MS * 1000LL)
_Static_assert(HCSR04P_PAYLOAD_SIZE <= TX_PAYLOAD_SIZE, "Distance payload exceeds telemetry capacity");
#define MHZ1911A_PAYLOAD_SIZE 128
#define MHZ1911A_MAX_AGE_US (TELEMETRY_MAX_AGE_MS * 1000LL)
_Static_assert(MHZ1911A_PAYLOAD_SIZE <= TX_PAYLOAD_SIZE, "CO2 payload exceeds telemetry capacity");
_Static_assert(80 + aai16_Mhz1911aReading_size <= MHZ1911A_PAYLOAD_SIZE, "CO2 telemetry exceeds its payload buffer");
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
    {SENSOR_GY511, "gy511", 100, 10000},
    // Keep the verified 200 ms minimum; cap slow polling at 2 s because both
    // detection latency and the three-idle-poll removal filter scale with it.
    {SENSOR_RC522, "rc522", 200, 2000},
    // Publication period only: at most two replaceable radar samples/second.
    // UART acquisition and the 2 s freshness limit remain independent.
    {SENSOR_LD2450, "ld2450", 500, 10000},
    // Actual acquisition period; MQTT stays at its independent 1000 ms cadence.
    {SENSOR_HCSR04P, "hcsr04p", HCSR04P_MIN_INTERVAL_MS, HCSR04P_MAX_INTERVAL_MS},
    {SENSOR_MHZ1911A, "mhz1911a", MHZ1911A_MIN_INTERVAL_MS, MHZ1911A_MAX_INTERVAL_MS},
    // Forced-mode conversion takes ~8 ms; the room-climate signal is slow, so
    // 500 ms is a generous floor and 1000 ms is the reader's default.
    {SENSOR_BME280, "bme280", 500, 10000},
    // One read restarts the chip's ~0.22 s conversion; faster polling only
    // returns stale values, so 250 ms is the floor.
    {SENSOR_MAX6675, "max6675", 250, 10000},
};

static const config_rule_t *config_rule_for_id(sensor_id_t id)
{
    for (unsigned i = 0; i < sizeof(s_config_rules) / sizeof(s_config_rules[0]); ++i) {
        if (s_config_rules[i].id == id) return &s_config_rules[i];
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
static portMUX_TYPE s_config_lock = portMUX_INITIALIZER_UNLOCKED;
static config_transactions_t s_transactions;
static config_fragments_t s_fragments;
static uint32_t s_config_decode_rejected;
static TaskHandle_t s_tx_task;

typedef enum {
    TX_DHT22,
    TX_GY511,
    TX_CONFIG_ACK,
    TX_RC522_EVENT,
    TX_LD2450,
    TX_HCSR04P,
    TX_MHZ1911A,
    TX_BME280,
    TX_MAX6675,
    TX_MOTION,
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
    uint32_t cancelled;
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
    uint32_t generation; // Configuration generation when the report was handed over.
    bool pending;
} s_radar;
static uint32_t s_radar_updates, s_radar_coalesced, s_radar_invalid, s_radar_deferred;
// Shared policy/deadline also protected by s_stats_lock; committed by the reader.
static bool s_radar_enabled = true;
static uint32_t s_radar_interval_ms = LD2450_DEFAULT_INTERVAL_MS, s_radar_epoch;
static int64_t s_radar_next_publish_us;
static int64_t s_radar_next_failure_log_us; // Publisher only.

// Same bounded handoff as radar. Snapshot and counters use s_stats_lock;
// only the existing publisher owns the fixed publication deadline.
static struct {
    hcsr04p_reading_t reading;
    int64_t acquired_us;
    uint32_t generation; // Configuration generation when the sample was handed over.
    bool pending;
} s_hcsr04p;
static uint32_t s_hcsr04p_updates, s_hcsr04p_coalesced, s_hcsr04p_invalid, s_hcsr04p_deferred;
static int64_t s_hcsr04p_next_publish_us, s_hcsr04p_next_failure_log_us;
static bool s_hcsr04p_enabled = true; // Gate and epoch protected by s_stats_lock.
static uint32_t s_hcsr04p_epoch;

// Latest CO2 sample, with the same lock/ownership as the other mailboxes.
// The producer's configured acquisition cadence supplies publication cadence.
static struct {
    uint16_t co2_ppm;
    int64_t acquired_us;
    uint32_t generation; // Configuration generation when the sample was handed over.
    bool pending;
} s_mhz1911a;
static int64_t s_mhz1911a_last_update_us = -1; // Guard also covers already consumed samples.
static uint32_t s_mhz1911a_updates, s_mhz1911a_coalesced, s_mhz1911a_invalid;
static uint32_t s_mhz1911a_nonnew, s_mhz1911a_deferred;
static int64_t s_mhz1911a_next_failure_log_us; // Publisher only.
static bool s_mhz1911a_enabled = true; // Gate/epoch share the snapshot lock.
static uint32_t s_mhz1911a_epoch;

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

// The LD2450 parser yields sign/magnitude values, so every axis is within +-32767, and an
// unused slot is all zero. Anything else cannot be represented by the wire contract.
static bool radar_report_valid(const ld2450_report_t *report)
{
    unsigned count = 0;
    for (unsigned i = 0; i < LD2450_TARGETS; ++i) {
        const ld2450_target_t *t = &report->targets[i];
        count += t->present;
        if (t->x_mm < -32767 || t->y_mm < -32767 || t->speed_cm_s < -32767) return false;
        if (!t->present && (t->x_mm || t->y_mm || t->speed_cm_s || t->resolution_mm)) return false;
    }
    return report->count == count;
}

esp_err_t mqtt_manager_update_ld2450_snapshot(const ld2450_report_t *report,
                                             int64_t received_us)
{
    if (report == NULL || !radar_report_valid(report) || received_us < 0 ||
        received_us > esp_timer_get_time()) {
        count_event(&s_radar_invalid);
        return ESP_ERR_INVALID_ARG;
    }
    // A report keeps the generation it was acquired under, even if publication is
    // delayed. Separate short critical section: never nested with s_stats_lock.
    uint32_t generation;
    portENTER_CRITICAL(&s_config_lock);
    generation = s_transactions.generation[SENSOR_LD2450];
    portEXIT_CRITICAL(&s_config_lock);
    portENTER_CRITICAL(&s_stats_lock);
    if (!s_radar_enabled) {
        portEXIT_CRITICAL(&s_stats_lock);
        return ESP_ERR_INVALID_STATE;
    }
    ++s_radar_updates;
    if (s_radar.pending) ++s_radar_coalesced;
    s_radar.report = *report;
    s_radar.received_us = received_us;
    s_radar.generation = generation;
    s_radar.pending = true;
    portEXIT_CRITICAL(&s_stats_lock);
    return ESP_OK;
}

esp_err_t mqtt_manager_commit_ld2450_config(const mqtt_config_update_t *update)
{
    const config_rule_t *rule = config_rule_for_id(SENSOR_LD2450);
    if (update == NULL || update->sensor != SENSOR_LD2450 ||
        (update->operation != MQTT_CONFIG_ENABLED && update->operation != MQTT_CONFIG_INTERVAL) ||
        (update->operation == MQTT_CONFIG_INTERVAL &&
         (update->interval_ms < rule->min_interval_ms || update->interval_ms > rule->max_interval_ms))) {
        return ESP_ERR_INVALID_ARG;
    }
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_stats_lock);
    if (update->operation == MQTT_CONFIG_INTERVAL) {
        if (!s_radar_enabled) {
            portEXIT_CRITICAL(&s_stats_lock);
            return ESP_ERR_INVALID_STATE;
        }
        s_radar_interval_ms = update->interval_ms;
        s_radar_epoch++; // Cancel an old-period sample not yet admitted to SDK.
        s_radar_next_publish_us = now + (int64_t)s_radar_interval_ms * 1000;
    } else if (s_radar_enabled != update->enabled) {
        s_radar_enabled = update->enabled;
        s_radar.pending = false;
        s_radar_epoch++;
        s_radar_next_publish_us = now + (int64_t)s_radar_interval_ms * 1000;
    }
    portEXIT_CRITICAL(&s_stats_lock);
    return ESP_OK;
}

// Binary aai16.Envelope.sensor_telemetry.ld2450, never JSON. The old JSON "uptime_ms" was
// the acquisition time: it is acquired_us here, in microseconds; uptime_us is when this
// message was built. seq -> Envelope.sequence (consumed publication attempts).
// All three physical slots are always sent with a `present` flag (an unused slot is
// zeros), where the JSON listed only present targets plus "target_count"; the count is the
// number of present slots. The JSON "distance_mm" is not sent: it is derived from x and y
// (sqrt(x^2 + y^2)). Returns the encoded length, or -1 on failure.
static int encode_radar(uint8_t *data, size_t capacity, const ld2450_report_t *report,
                        int64_t acquired_us, int64_t uptime_us, uint32_t generation, uint32_t seq)
{
    if (acquired_us < 0 || uptime_us < acquired_us) return -1;
    aai16_Envelope e = aai16_Envelope_init_zero;
    e.has_protocol_version = e.has_boot_id = e.has_sequence = e.has_generation = true;
    e.protocol_version = AAI16_PROTOCOL_VERSION;
    e.boot_id = s_boot_id; e.sequence = seq; e.generation = generation;
    e.which_body = aai16_Envelope_sensor_telemetry_tag;
    aai16_SensorTelemetry *t = &e.body.sensor_telemetry;
    t->has_sensor = t->has_acquired_us = t->has_uptime_us = true;
    t->sensor = config_sensor_to_wire(SENSOR_LD2450);
    t->acquired_us = (uint64_t)acquired_us;
    t->uptime_us = (uint64_t)uptime_us;
    t->which_reading = aai16_SensorTelemetry_ld2450_tag;
    aai16_Ld2450Reading *r = &t->reading.ld2450;
    r->targets_count = LD2450_TARGETS;
    for (unsigned i = 0; i < LD2450_TARGETS; ++i) {
        const ld2450_target_t *src = &report->targets[i];
        aai16_RadarTarget *dst = &r->targets[i];
        dst->has_slot = dst->has_present = dst->has_x_mm = dst->has_y_mm = true;
        dst->has_speed_cm_s = dst->has_resolution_mm = true;
        dst->slot = i + 1;
        dst->present = src->present;
        dst->x_mm = src->x_mm;
        dst->y_mm = src->y_mm;
        dst->speed_cm_s = src->speed_cm_s;
        dst->resolution_mm = src->resolution_mm;
    }
    size_t length = 0;
    return aai16_protocol_encode(&e, data, capacity, &length) == aai16_ResultCode_RESULT_OK ? (int)length : -1;
}

// Only the existing MQTT worker calls this. A blocked transport cannot hold
// the mailbox lock or stop UART reception. No replay/catch-up burst after stalls.
static void service_radar_snapshot(void)
{
    int64_t now = esp_timer_get_time();
    bool busy = s_tx_queue != NULL && uxQueueMessagesWaiting(s_tx_queue);
    ld2450_report_t report;
    int64_t received_us;
    uint32_t epoch, generation;
    portENTER_CRITICAL(&s_stats_lock);
    if (!s_radar_enabled || now < s_radar_next_publish_us) {
        portEXIT_CRITICAL(&s_stats_lock);
        return;
    }
    s_radar_next_publish_us = now + (int64_t)s_radar_interval_ms * 1000;
    // Preserve all eight queue slots for their existing users. Defer radar if
    // any ACK/telemetry work is pending; the reader keeps replacing its snapshot.
    if (busy) {
        if (s_radar.pending) ++s_radar_deferred;
        portEXIT_CRITICAL(&s_stats_lock);
        return;
    }
    bool pending = s_radar.pending;
    epoch = s_radar_epoch;
    report = s_radar.report;
    received_us = s_radar.received_us;
    generation = s_radar.generation;
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
    uint8_t payload[LD2450_PAYLOAD_SIZE]; // Binary: length-delimited, never strlen().
    int len = encode_radar(payload, sizeof(payload), &report, received_us,
                           esp_timer_get_time(), generation, seq);
    if (len < 0) {
        count_event(&stats->invalid); // Serialization failure, separate from invalid updates.
        return;
    }
    int64_t start_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_stats_lock);
    bool admitted = s_radar_enabled && epoch == s_radar_epoch;
    if (!admitted) ++stats->cancelled;
    else if (start_us - received_us >= LD2450_MAX_AGE_US) {
        ++stats->stale;
        admitted = false;
    }
    portEXIT_CRITICAL(&s_stats_lock);
    if (!admitted) return;
    // Admission is the in-flight boundary. Never hold the spinlock across SDK
    // calls: a subsequent disable cannot cancel this already admitted send.
    int result = esp_mqtt_client_publish(s_client, LD2450_DATA_TOPIC, (const char *)payload, len, 0, 0);
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

esp_err_t mqtt_manager_update_hcsr04p_snapshot(const hcsr04p_reading_t *reading,
                                              int64_t acquired_us)
{
    // The driver validates pulse/range before handoff. Also bound this API's
    // inputs so non-finite numbers or invalid timestamps cannot reach the encoder.
    if (reading == NULL || !isfinite(reading->distance_cm) || !isfinite(reading->echo_us) ||
        reading->distance_cm < HCSR04P_MIN_CM || reading->distance_cm > HCSR04P_MAX_CM ||
        reading->echo_us <= 0 || reading->echo_us > HCSR04P_ECHO_TIMEOUT_MS * 1000 ||
        acquired_us < 0 || acquired_us > esp_timer_get_time()) {
        count_event(&s_hcsr04p_invalid);
        return ESP_ERR_INVALID_ARG;
    }
    // A reading keeps the generation it was acquired under, even if publication is
    // delayed. Separate short critical section: never nested with s_stats_lock.
    uint32_t generation;
    portENTER_CRITICAL(&s_config_lock);
    generation = s_transactions.generation[SENSOR_HCSR04P];
    portEXIT_CRITICAL(&s_config_lock);
    portENTER_CRITICAL(&s_stats_lock);
    if (!s_hcsr04p_enabled) {
        portEXIT_CRITICAL(&s_stats_lock);
        return ESP_ERR_INVALID_STATE;
    }
    ++s_hcsr04p_updates;
    if (s_hcsr04p.pending) ++s_hcsr04p_coalesced;
    s_hcsr04p.reading = *reading;
    s_hcsr04p.acquired_us = acquired_us;
    s_hcsr04p.generation = generation;
    s_hcsr04p.pending = true;
    portEXIT_CRITICAL(&s_stats_lock);
    return ESP_OK;
}

void mqtt_manager_commit_hcsr04p_enabled(bool enabled)
{
    portENTER_CRITICAL(&s_stats_lock);
    if (s_hcsr04p_enabled != enabled) {
        s_hcsr04p_enabled = enabled;
        s_hcsr04p.pending = false;
        ++s_hcsr04p_epoch;
    }
    portEXIT_CRITICAL(&s_stats_lock);
}

// Binary aai16.Envelope.sensor_telemetry.hcsr04p, never JSON. The old JSON
// "uptime_ms" was the acquisition time: it is acquired_us here, in microseconds;
// uptime_us is when this message was built. seq -> Envelope.sequence (consumed
// publication attempts). distance_cm and echo_us are IEEE-754 floats on the wire,
// not the 0.1-rounded decimal text. Returns the encoded length, or -1 on failure.
static int encode_hcsr04p(uint8_t *data, size_t capacity, const hcsr04p_reading_t *reading,
                          int64_t acquired_us, int64_t uptime_us, uint32_t generation, uint32_t seq)
{
    if (acquired_us < 0 || uptime_us < acquired_us) return -1;
    aai16_Envelope e = aai16_Envelope_init_zero;
    e.has_protocol_version = e.has_boot_id = e.has_sequence = e.has_generation = true;
    e.protocol_version = AAI16_PROTOCOL_VERSION;
    e.boot_id = s_boot_id; e.sequence = seq; e.generation = generation;
    e.which_body = aai16_Envelope_sensor_telemetry_tag;
    aai16_SensorTelemetry *t = &e.body.sensor_telemetry;
    t->has_sensor = t->has_acquired_us = t->has_uptime_us = true;
    t->sensor = config_sensor_to_wire(SENSOR_HCSR04P);
    t->acquired_us = (uint64_t)acquired_us;
    t->uptime_us = (uint64_t)uptime_us;
    t->which_reading = aai16_SensorTelemetry_hcsr04p_tag;
    t->reading.hcsr04p.has_distance_cm = t->reading.hcsr04p.has_echo_us = true;
    t->reading.hcsr04p.distance_cm = reading->distance_cm;
    t->reading.hcsr04p.echo_us = reading->echo_us;
    size_t length = 0;
    return aai16_protocol_encode(&e, data, capacity, &length) == aai16_ResultCode_RESULT_OK ? (int)length : -1;
}

// Publisher task only. Slow SDK calls hold a local copy, never the mailbox lock.
static void service_hcsr04p_snapshot(void)
{
    int64_t now = esp_timer_get_time();
    if (now < s_hcsr04p_next_publish_us) return;
    s_hcsr04p_next_publish_us = now + HCSR04P_PUBLISH_INTERVAL_US;
    // Preserve ACK/telemetry slots, event priority and all QoS 1 outbox capacity.
    bool busy = (s_tx_queue != NULL && uxQueueMessagesWaiting(s_tx_queue)) ||
                (s_event_queue != NULL && uxQueueMessagesWaiting(s_event_queue));
    hcsr04p_reading_t reading;
    int64_t acquired_us;
    uint32_t epoch, generation;
    portENTER_CRITICAL(&s_stats_lock);
    if (!s_hcsr04p_enabled) {
        portEXIT_CRITICAL(&s_stats_lock);
        return;
    }
    if (busy) {
        if (s_hcsr04p.pending) ++s_hcsr04p_deferred;
        portEXIT_CRITICAL(&s_stats_lock);
        return;
    }
    bool pending = s_hcsr04p.pending;
    epoch = s_hcsr04p_epoch;
    reading = s_hcsr04p.reading;
    acquired_us = s_hcsr04p.acquired_us;
    generation = s_hcsr04p.generation;
    s_hcsr04p.pending = false;
    portEXIT_CRITICAL(&s_stats_lock);
    if (!pending) return;
    tx_stats_t *stats = &s_stats[TX_HCSR04P];
    uint32_t seq = begin_attempt(TX_HCSR04P);
    if (now - acquired_us >= HCSR04P_MAX_AGE_US) {
        count_event(&stats->stale);
        return;
    }
    if (!mqtt_connected()) {
        count_event(&stats->offline);
        return;
    }
    char payload[HCSR04P_PAYLOAD_SIZE]; // Binary: length-delimited, never strlen().
    int len = encode_hcsr04p((uint8_t *)payload, sizeof(payload), &reading, acquired_us,
                             esp_timer_get_time(), generation, seq);
    if (len < 0) {
        count_event(&stats->invalid); // Serialization failure, separate from invalid updates.
        return;
    }
    // Serialization/preemption can age data; check again at SDK admission.
    int64_t start_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_stats_lock);
    bool admitted = s_hcsr04p_enabled && epoch == s_hcsr04p_epoch;
    if (!admitted) ++stats->cancelled;
    portEXIT_CRITICAL(&s_stats_lock);
    if (!admitted) return;
    if (start_us - acquired_us >= HCSR04P_MAX_AGE_US) {
        count_event(&stats->stale);
        return;
    }
    if (!mqtt_connected()) {
        count_event(&stats->offline);
        return;
    }
    int result = esp_mqtt_client_publish(s_client, HCSR04P_DATA_TOPIC, payload, len, 0, 0);
    int64_t elapsed_us = esp_timer_get_time() - start_us;
    portENTER_CRITICAL(&s_stats_lock);
    if (elapsed_us > stats->max_tx_call_us) stats->max_tx_call_us = elapsed_us;
    if (result >= 0) ++stats->publish_ok; // SDK success, never a broker/subscriber receipt.
    else ++stats->publish_failed;
    portEXIT_CRITICAL(&s_stats_lock);
    if (result < 0 && esp_timer_get_time() >= s_hcsr04p_next_failure_log_us) {
        ESP_LOGW(TAG, "%s publish failed (result=%d); see drop counters", HCSR04P_DATA_TOPIC, result);
        s_hcsr04p_next_failure_log_us = esp_timer_get_time() + 10000000LL;
    }
}

esp_err_t mqtt_manager_update_mhz1911a_snapshot(uint16_t co2_ppm, int64_t acquired_us)
{
    if (co2_ppm < MHZ1911A_MIN_PPM || co2_ppm > MHZ1911A_MAX_PPM ||
        acquired_us < 0 || acquired_us > esp_timer_get_time()) {
        count_event(&s_mhz1911a_invalid);
        return ESP_ERR_INVALID_ARG;
    }
    // A sample keeps the generation it was acquired under, even if publication is
    // delayed. Separate short critical section: never nested with s_stats_lock.
    uint32_t generation;
    portENTER_CRITICAL(&s_config_lock);
    generation = s_transactions.generation[SENSOR_MHZ1911A];
    portEXIT_CRITICAL(&s_config_lock);
    portENTER_CRITICAL(&s_stats_lock);
    if (!s_mhz1911a_enabled) {
        portEXIT_CRITICAL(&s_stats_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (acquired_us <= s_mhz1911a_last_update_us) {
        ++s_mhz1911a_nonnew;
        portEXIT_CRITICAL(&s_stats_lock);
        return ESP_ERR_INVALID_STATE;
    }
    ++s_mhz1911a_updates;
    if (s_mhz1911a.pending) ++s_mhz1911a_coalesced;
    s_mhz1911a.co2_ppm = co2_ppm;
    s_mhz1911a.acquired_us = acquired_us;
    s_mhz1911a.generation = generation;
    s_mhz1911a.pending = true;
    s_mhz1911a_last_update_us = acquired_us;
    portEXIT_CRITICAL(&s_stats_lock);
    return ESP_OK;
}

void mqtt_manager_commit_mhz1911a_config(bool enabled)
{
    portENTER_CRITICAL(&s_stats_lock);
    s_mhz1911a_enabled = enabled;
    s_mhz1911a.pending = false;
    ++s_mhz1911a_epoch;
    portEXIT_CRITICAL(&s_stats_lock);
}

// Binary aai16.Envelope.sensor_telemetry.mhz1911a, never JSON. The old JSON "uptime_ms"
// was the acquisition time: it is acquired_us here, in microseconds; uptime_us is when
// this message was built. seq -> Envelope.sequence (consumed publication attempts).
// co2_ppm is an unsigned integer, 400..5000. Returns the encoded length, or -1 on failure.
static int encode_mhz1911a(uint8_t *data, size_t capacity, uint16_t co2_ppm,
                           int64_t acquired_us, int64_t uptime_us, uint32_t generation, uint32_t seq)
{
    if (acquired_us < 0 || uptime_us < acquired_us) return -1;
    aai16_Envelope e = aai16_Envelope_init_zero;
    e.has_protocol_version = e.has_boot_id = e.has_sequence = e.has_generation = true;
    e.protocol_version = AAI16_PROTOCOL_VERSION;
    e.boot_id = s_boot_id; e.sequence = seq; e.generation = generation;
    e.which_body = aai16_Envelope_sensor_telemetry_tag;
    aai16_SensorTelemetry *t = &e.body.sensor_telemetry;
    t->has_sensor = t->has_acquired_us = t->has_uptime_us = true;
    t->sensor = config_sensor_to_wire(SENSOR_MHZ1911A);
    t->acquired_us = (uint64_t)acquired_us;
    t->uptime_us = (uint64_t)uptime_us;
    t->which_reading = aai16_SensorTelemetry_mhz1911a_tag;
    t->reading.mhz1911a.has_co2_ppm = true;
    t->reading.mhz1911a.co2_ppm = co2_ppm;
    size_t length = 0;
    return aai16_protocol_encode(&e, data, capacity, &length) == aai16_ResultCode_RESULT_OK ? (int)length : -1;
}

static void service_mhz1911a_snapshot(void)
{
    bool busy = (s_tx_queue != NULL && uxQueueMessagesWaiting(s_tx_queue)) ||
                (s_event_queue != NULL && uxQueueMessagesWaiting(s_event_queue));
    uint16_t co2_ppm;
    int64_t acquired_us;
    uint32_t epoch, generation;
    portENTER_CRITICAL(&s_stats_lock);
    if (!s_mhz1911a_enabled || !s_mhz1911a.pending || busy) {
        if (s_mhz1911a.pending) ++s_mhz1911a_deferred;
        portEXIT_CRITICAL(&s_stats_lock);
        return;
    }
    co2_ppm = s_mhz1911a.co2_ppm;
    epoch = s_mhz1911a_epoch;
    acquired_us = s_mhz1911a.acquired_us;
    generation = s_mhz1911a.generation;
    s_mhz1911a.pending = false; // Consume once, including offline/failed attempts.
    portEXIT_CRITICAL(&s_stats_lock);

    tx_stats_t *stats = &s_stats[TX_MHZ1911A];
    uint32_t seq = begin_attempt(TX_MHZ1911A);
    if (esp_timer_get_time() - acquired_us >= MHZ1911A_MAX_AGE_US) {
        count_event(&stats->stale);
        return;
    }
    if (!mqtt_connected()) {
        count_event(&stats->offline);
        return;
    }
    uint8_t payload[MHZ1911A_PAYLOAD_SIZE]; // Binary: length-delimited, never strlen().
    int len = encode_mhz1911a(payload, sizeof(payload), co2_ppm, acquired_us,
                              esp_timer_get_time(), generation, seq);
    if (len < 0) {
        count_event(&stats->invalid); // Serialization failure, separate from invalid updates.
        return;
    }
    int64_t start_us = esp_timer_get_time();
    if (start_us - acquired_us >= MHZ1911A_MAX_AGE_US) {
        count_event(&stats->stale);
        return;
    }
    if (!mqtt_connected()) {
        count_event(&stats->offline);
        return;
    }
    // Linearize admission against reader configuration without holding a
    // spinlock across an SDK call. Work admitted before a commit may finish;
    // copied data from an older epoch cannot gain admission after that commit.
    portENTER_CRITICAL(&s_stats_lock);
    bool admitted = s_mhz1911a_enabled && epoch == s_mhz1911a_epoch;
    if (!admitted) ++stats->cancelled;
    portEXIT_CRITICAL(&s_stats_lock);
    if (!admitted) return;
    // Only the shared publisher calls the SDK. Its local copy cannot be torn by
    // a concurrent reader update, and the reader never waits on this SDK mutex.
    int result = esp_mqtt_client_publish(s_client, MHZ1911A_DATA_TOPIC, (const char *)payload, len, 0, 0);
    int64_t elapsed_us = esp_timer_get_time() - start_us;
    portENTER_CRITICAL(&s_stats_lock);
    if (elapsed_us > stats->max_tx_call_us) stats->max_tx_call_us = elapsed_us;
    if (result >= 0) ++stats->publish_ok;
    else ++stats->publish_failed;
    portEXIT_CRITICAL(&s_stats_lock);
    if (result < 0 && esp_timer_get_time() >= s_mhz1911a_next_failure_log_us) {
        ESP_LOGW(TAG, "%s publish failed (result=%d); see drop counters", MHZ1911A_DATA_TOPIC, result);
        s_mhz1911a_next_failure_log_us = esp_timer_get_time() + 10000000LL;
    }
}

// Separate bounded mailbox keeps model traffic out of the sensor/config FIFO.
typedef struct { bool status; size_t length; uint8_t data[AAI16_FRAME_MAX]; } model_reply_t;
static model_reply_t s_model_replies[4];
static unsigned s_model_reply_head, s_model_reply_count;
static bool queue_model_reply(bool status, const uint8_t *data, size_t length)
{
    if (!data || !length || length > AAI16_FRAME_MAX) return false;
    portENTER_CRITICAL(&s_stats_lock);
    bool accepted = s_model_reply_count < 4;
    if (accepted) {
        model_reply_t *slot = &s_model_replies[(s_model_reply_head + s_model_reply_count) % 4];
        slot->status = status; slot->length = length;
        memcpy(slot->data, data, length);
        ++s_model_reply_count;
    }
    portEXIT_CRITICAL(&s_stats_lock);
    return accepted;
}
static void service_model_replies(void)
{
    if (!mqtt_connected()) return;
    model_reply_t reply;
    portENTER_CRITICAL(&s_stats_lock);
    bool pending = s_model_reply_count != 0;
    if (pending) reply = s_model_replies[s_model_reply_head];
    portEXIT_CRITICAL(&s_stats_lock);
    if (!pending) return;
    // Share the existing 1 KiB event allowance, preserving config ACK reserve.
    int used = esp_mqtt_client_get_outbox_size(s_client);
    if (used < 0 || (size_t)used + reply.length + 64 > EVENT_OUTBOX_BUDGET_BYTES) return;
    int id = esp_mqtt_client_enqueue(s_client, reply.status ? MODEL_STATUS_TOPIC : MODEL_ACK_TOPIC,
        (const char *)reply.data, reply.length, 1, 0, true);
    if (id < 0) return;
    portENTER_CRITICAL(&s_stats_lock);
    s_model_reply_head = (s_model_reply_head + 1) % 4;
    --s_model_reply_count;
    portEXIT_CRITICAL(&s_stats_lock);
}

static TickType_t publisher_wait_ticks(void)
{
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_stats_lock);
    int64_t remaining = s_radar_enabled ? s_radar_next_publish_us - now : 1000000;
    // A pending distance sample may be due before the radar deadline. With
    // no sample, the existing 1 s ceiling still discovers the next handoff.
    if (s_hcsr04p.pending && s_hcsr04p_next_publish_us - now < remaining) {
        remaining = s_hcsr04p_next_publish_us - now;
    }
    portEXIT_CRITICAL(&s_stats_lock);
    // The old fixed 1 s wait would prevent 500 ms publication on an idle queue.
    // Keep the existing 1 s ceiling and round up to at least one RTOS tick.
    if (remaining > 1000000) remaining = 1000000;
    uint32_t ms = remaining > 0 ? (uint32_t)((remaining + 999) / 1000) : 1;
    return pdMS_TO_TICKS(ms + portTICK_PERIOD_MS - 1);
}

static void mqtt_tx_task(void *arg)
{
    (void)arg;
    static const char *const topics[TX_KIND_COUNT] = {
        DHT22_DATA_TOPIC, GY511_DATA_TOPIC, CONFIG_ACK_TOPIC, RC522_EVENT_TOPIC, LD2450_DATA_TOPIC,
        HCSR04P_DATA_TOPIC, MHZ1911A_DATA_TOPIC, BME280_DATA_TOPIC, MAX6675_DATA_TOPIC, MOTION_DATA_TOPIC
    };
    tx_message_t message;
    int64_t next_failure_log_us = 0;
    while (true) {
        service_radar_snapshot();
        service_hcsr04p_snapshot();
        service_mhz1911a_snapshot();
        if (xQueueReceive(s_tx_queue, &message, publisher_wait_ticks()) != pdTRUE) {
            update_outbox_snapshot();
            service_model_replies();
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
        service_model_replies();
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
    uint32_t hcsr04p_updates, hcsr04p_coalesced, hcsr04p_invalid, hcsr04p_deferred;
    bool hcsr04p_pending, hcsr04p_enabled;
    uint32_t mhz1911a_updates, mhz1911a_coalesced, mhz1911a_invalid, mhz1911a_nonnew, mhz1911a_deferred;
    bool mhz1911a_pending, mhz1911a_enabled;
    bool radar_pending;
    bool radar_enabled;
    uint32_t radar_interval_ms;
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
    radar_enabled = s_radar_enabled;
    radar_interval_ms = s_radar_interval_ms;
    hcsr04p_updates = s_hcsr04p_updates;
    hcsr04p_coalesced = s_hcsr04p_coalesced;
    hcsr04p_invalid = s_hcsr04p_invalid;
    hcsr04p_deferred = s_hcsr04p_deferred;
    hcsr04p_pending = s_hcsr04p.pending;
    hcsr04p_enabled = s_hcsr04p_enabled;
    mhz1911a_updates = s_mhz1911a_updates;
    mhz1911a_coalesced = s_mhz1911a_coalesced;
    mhz1911a_invalid = s_mhz1911a_invalid;
    mhz1911a_nonnew = s_mhz1911a_nonnew;
    mhz1911a_deferred = s_mhz1911a_deferred;
    mhz1911a_pending = s_mhz1911a.pending;
    mhz1911a_enabled = s_mhz1911a_enabled;
    portEXIT_CRITICAL(&s_stats_lock);
    uint32_t config_invalid, fragment_invalid, fragment_expired;
    uint64_t config_high_water;
    portENTER_CRITICAL(&s_config_lock);
    config_invalid = s_config_decode_rejected;
    fragment_invalid = s_fragments.rejected; fragment_expired = s_fragments.expired;
    config_high_water = s_transactions.high_water_request;
    portEXIT_CRITICAL(&s_config_lock);
    ESP_LOGI(TAG, "  protobuf_config_invalid=%" PRIu32 " fragment_invalid=%" PRIu32
             " fragment_expired=%" PRIu32 " request_high_water=%" PRIu64,
             config_invalid, fragment_invalid, fragment_expired, config_high_water);
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
             " invalid_updates=%" PRIu32 " busy_deferrals=%" PRIu32 " pending=%d enabled=%d period_ms=%" PRIu32 " max_age_ms=2000",
             radar_updates, radar_coalesced, radar_invalid, radar_deferred, radar_pending, radar_enabled, radar_interval_ms);
    ESP_LOGI(TAG, "  hcsr04p_snapshot_updates=%" PRIu32 " coalesced=%" PRIu32
             " invalid_updates=%" PRIu32 " busy_deferrals=%" PRIu32 " pending=%d enabled=%d period_ms=1000 max_age_ms=2000",
             hcsr04p_updates, hcsr04p_coalesced, hcsr04p_invalid, hcsr04p_deferred, hcsr04p_pending, hcsr04p_enabled);
    ESP_LOGI(TAG, "  mhz1911a_snapshot_updates=%" PRIu32 " coalesced=%" PRIu32
             " invalid_updates=%" PRIu32 " nonnew_updates=%" PRIu32 " busy_deferrals=%" PRIu32 " pending=%d enabled=%d max_age_ms=2000",
             mhz1911a_updates, mhz1911a_coalesced, mhz1911a_invalid, mhz1911a_nonnew, mhz1911a_deferred, mhz1911a_pending, mhz1911a_enabled);
    static const char *const names[TX_KIND_COUNT] = {
        "dht22", "gy511", "config_ack", "rc522_event", "ld2450", "hcsr04p", "mhz1911a",
        "bme280", "max6675", "motion_ml"};
    for (int i = 0; i < TX_KIND_COUNT; ++i) {
        const tx_stats_t *s = &snapshot[i];
        ESP_LOGI(TAG, "%s attempts=%" PRIu32 " queued=%" PRIu32
                 " publish_ok=%" PRIu32 " enqueued=%" PRIu32 " max_tx_call_us=%" PRId64,
                 names[i], s->attempts, s->queued, s->publish_ok, s->enqueued,
                 s->max_tx_call_us);
        ESP_LOGI(TAG, "  drops: offline=%" PRIu32 " queue_full=%" PRIu32
                 " invalid=%" PRIu32 " stale=%" PRIu32 " outbox_full=%" PRIu32
                 " enqueue_failed=%" PRIu32 " publish_failed=%" PRIu32 " cancelled=%" PRIu32,
                 s->offline,
                 s->queue_full, s->invalid, s->stale, s->outbox_full,
                 s->enqueue_failed, s->publish_failed, s->cancelled);
    }
}


// --------------------------------------------------
// Send rejected configuration ACK
// --------------------------------------------------

#define CR(x) aai16_ResultCode_RESULT_##x
#define CP(x) aai16_Phase_PHASE_##x
// Full ACK bound: 36-byte conservative envelope overhead + 98-byte ConfigAck.
_Static_assert(36 + aai16_ConfigAck_size < TX_PAYLOAD_SIZE, "ACK exceeds existing TX slot");

static esp_err_t publish_receipt(const config_receipt_t *r)
{
    aai16_Envelope e = aai16_Envelope_init_zero;
    e.has_protocol_version = e.has_request_id = e.has_boot_id = e.has_generation = e.has_phase = true;
    e.protocol_version = AAI16_PROTOCOL_VERSION;
    e.request_id = r->update.request_id; e.boot_id = s_boot_id;
    e.generation = r->generation; e.phase = r->phase;
    e.which_body = aai16_Envelope_config_ack_tag;
    aai16_ConfigAck *ack = &e.body.config_ack;
    ack->has_result = true; ack->result = r->result;
    ack->sensor = config_sensor_to_wire(r->update.sensor);
    ack->has_sensor = ack->sensor != aai16_SensorId_SENSOR_UNSPECIFIED;
    if (ack->has_sensor) {
        ack->has_requested = true;
        config_requested(&r->update, &ack->requested);
    }
    begin_attempt(TX_CONFIG_ACK);
    tx_message_t message = {.kind = TX_CONFIG_ACK};
    size_t length = 0;
    if (aai16_protocol_encode(&e, (uint8_t *)message.payload, sizeof(message.payload), &length) != CR(OK)) {
        count_event(&s_stats[TX_CONFIG_ACK].invalid);
        return ESP_ERR_INVALID_ARG;
    }
    message.len = (int)length; // Binary: never strlen(), including explicit false.
    return queue_message(&message);
}

static esp_err_t finish_config(const mqtt_config_update_t *update, aai16_ResultCode result)
{
    if (!update) return ESP_ERR_INVALID_ARG;
    config_receipt_t receipt;
    portENTER_CRITICAL(&s_config_lock);
    bool finished = config_finish(&s_transactions, update, result, &receipt);
    portEXIT_CRITICAL(&s_config_lock);
    // Cache is committed before queue admission. Retrying the same request
    // recovers a dropped ACK without applying again or incrementing generation.
    return finished ? publish_receipt(&receipt) : ESP_ERR_INVALID_STATE;
}

esp_err_t mqtt_manager_reject_config(const mqtt_config_update_t *update, const char *reason)
{
    aai16_ResultCode result = CR(INTERNAL_ERROR);
    if (!reason) return ESP_ERR_INVALID_ARG;
    if (!strcmp(reason, "sensor_unavailable")) result = CR(SENSOR_UNAVAILABLE);
    else if (!strcmp(reason, "sensor_disabled")) result = CR(SENSOR_DISABLED);
    else if (!strcmp(reason, "unsupported_sensor")) result = CR(UNSUPPORTED_SENSOR);
    else if (!strcmp(reason, "queue_full")) result = CR(QUEUE_FULL);
    return finish_config(update, result);
}

bool mqtt_manager_stage_owner_config(const mqtt_config_update_t *update)
{
    if (!update) return false;
    portENTER_CRITICAL(&s_config_lock);
    bool ok = config_stage(&s_transactions, update);
    portEXIT_CRITICAL(&s_config_lock);
    return ok;
}
void mqtt_manager_cancel_owner_config(sensor_id_t sensor)
{
    if ((unsigned)sensor >= SENSOR_STATUS_CAPACITY) return;
    portENTER_CRITICAL(&s_config_lock);
    s_transactions.occupied[sensor] = false;
    portEXIT_CRITICAL(&s_config_lock);
}
bool mqtt_manager_take_owner_config(sensor_id_t sensor, mqtt_config_update_t *out)
{
    if (!out) return false;
    portENTER_CRITICAL(&s_config_lock);
    bool ok = config_take(&s_transactions, sensor, out);
    portEXIT_CRITICAL(&s_config_lock);
    return ok;
}
bool mqtt_manager_begin_owner_config(mqtt_config_update_t *update)
{
    if (!update) return false;
    config_receipt_t failure = {0};
    portENTER_CRITICAL(&s_config_lock);
    bool ok = config_begin(&s_transactions, update, &failure);
    portEXIT_CRITICAL(&s_config_lock);
    if (!ok && failure.update.request_id) (void)publish_receipt(&failure);
    return ok;
}

static void receive_config(const uint8_t *data, size_t length)
{
    aai16_Envelope envelope;
    uint64_t identity;
    aai16_ResultCode result = aai16_config_decode(data, length, &envelope, &identity);
    if (result != CR(OK)) {
        portENTER_CRITICAL(&s_config_lock);
        ++s_config_decode_rejected;
        portEXIT_CRITICAL(&s_config_lock);
        if (identity) {
            config_receipt_t rejected = {.update = {.request_id = identity, .sensor = SENSOR_STATUS_CAPACITY},
                .phase = CP(VALIDATE), .result = result};
            (void)publish_receipt(&rejected);
        }
        return; // No trusted request ID: diagnostic drop, never invent an ACK ID.
    }
    mqtt_config_update_t update;
    config_receipt_t reply;
    portENTER_CRITICAL(&s_config_lock);
    int admitted = config_admit(&s_transactions, &envelope, s_boot_id, &update, &reply);
    portEXIT_CRITICAL(&s_config_lock);
    if (admitted < 0) (void)publish_receipt(&reply);
    if (admitted > 0 && (s_config_queue == NULL || xQueueSend(s_config_queue, &update, 0) != pdTRUE))
        (void)mqtt_manager_reject_config(&update, "queue_full");
    // No successful VALIDATE/queued ACK: only the sensor owner can report OK.
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
        if (esp_mqtt_client_subscribe(s_client, MODEL_CONTROL_TOPIC, 1) < 0 ||
            esp_mqtt_client_subscribe(s_client, MODEL_CHUNK_TOPIC, 1) < 0)
            ESP_LOGE(TAG, "Model staging subscription failed");

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
        esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
        if (!event) break;
        model_upload_receive(event->topic, event->topic_len > 0 ? (size_t)event->topic_len : 0,
            event->data, event->data_len, event->current_data_offset, event->total_data_len,
            event->msg_id, event->retain);
        bool matches = event->topic && event->topic_len == (int)strlen(CONFIG_SET_TOPIC) &&
                       memcmp(event->topic, CONFIG_SET_TOPIC, strlen(CONFIG_SET_TOPIC)) == 0;
        uint8_t completed[AAI16_CONFIG_FRAME_MAX];
        int64_t now = esp_timer_get_time();
        portENTER_CRITICAL(&s_config_lock);
        size_t length = config_fragment_feed(&s_fragments, now, matches,
            event->topic != NULL && event->topic_len != 0, event->retain, event->msg_id,
            event->current_data_offset, event->total_data_len, event->data, event->data_len);
        if (length) memcpy(completed, s_fragments.data, length);
        portEXIT_CRITICAL(&s_config_lock);
        if (length) receive_config(completed, length);
        break;
    }

    // ----------------------------------------------
    // MQTT disconnected
    // ----------------------------------------------

    case MQTT_EVENT_DISCONNECTED:
        model_upload_disconnected();
        portENTER_CRITICAL(&s_config_lock);
        if (s_fragments.active) ++s_fragments.rejected;
        s_fragments.active = false;
        portEXIT_CRITICAL(&s_config_lock);
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
    memset(&s_transactions, 0, sizeof(s_transactions));
    memset(&s_fragments, 0, sizeof(s_fragments));
    s_config_decode_rejected = 0;
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

    // Address only (never credentials). A stale address baked into an old flash
    // image is a common cause of "Connection reset"; this makes it visible at boot.
    ESP_LOGI(TAG, "Broker: %s", MQTT_BROKER_URI);
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
    if (!model_upload_start(s_boot_id, queue_model_reply))
        ESP_LOGE(TAG, "Model staging worker unavailable; inference/configuration continue");
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
// Binary aai16.Envelope.sensor_telemetry.dht22, never JSON. seq -> Envelope.sequence
// (attempt count: rejected submissions still advance it), uptime_ms -> acquired_us and
// uptime_us (microseconds; the reading is published as it is acquired). generation is the
// sensor's committed configuration generation at submission.
// Physical limits of the DHT22 (datasheet): -40..80 C, 0..100 %RH. Anything outside is a
// failed read, not a measurement, so it is counted as invalid and never queued.
#define DHT22_TEMP_MIN_C (-40.0f)
#define DHT22_TEMP_MAX_C 80.0f
#define DHT22_HUMIDITY_MIN_PCT 0.0f
#define DHT22_HUMIDITY_MAX_PCT 100.0f
_Static_assert(36 + aai16_SensorTelemetry_size < TX_PAYLOAD_SIZE, "DHT22 telemetry exceeds TX slot");

esp_err_t mqtt_manager_publish_dht22(
    float temperature_c,
    float humidity_pct)
{
    uint32_t seq = begin_attempt(TX_DHT22);
    if (!isfinite(temperature_c) || !isfinite(humidity_pct) ||
        temperature_c < DHT22_TEMP_MIN_C || temperature_c > DHT22_TEMP_MAX_C ||
        humidity_pct < DHT22_HUMIDITY_MIN_PCT || humidity_pct > DHT22_HUMIDITY_MAX_PCT) {
        count_event(&s_stats[TX_DHT22].invalid);
        return ESP_ERR_INVALID_ARG;
    }
    int64_t now_us = esp_timer_get_time();
    uint32_t generation;
    portENTER_CRITICAL(&s_config_lock);
    generation = s_transactions.generation[SENSOR_DHT22];
    portEXIT_CRITICAL(&s_config_lock);

    aai16_Envelope e = aai16_Envelope_init_zero;
    e.has_protocol_version = e.has_boot_id = e.has_sequence = e.has_generation = true;
    e.protocol_version = AAI16_PROTOCOL_VERSION;
    e.boot_id = s_boot_id; e.sequence = seq; e.generation = generation;
    e.which_body = aai16_Envelope_sensor_telemetry_tag;
    aai16_SensorTelemetry *t = &e.body.sensor_telemetry;
    t->has_sensor = t->has_acquired_us = t->has_uptime_us = true;
    t->sensor = config_sensor_to_wire(SENSOR_DHT22);
    t->acquired_us = t->uptime_us = (uint64_t)now_us;
    t->which_reading = aai16_SensorTelemetry_dht22_tag;
    t->reading.dht22.has_temperature_c = t->reading.dht22.has_humidity_pct = true;
    t->reading.dht22.temperature_c = temperature_c;
    t->reading.dht22.humidity_pct = humidity_pct;

    tx_message_t message = {.kind = TX_DHT22, .created_ms = now_us / 1000};
    size_t length = 0;
    if (aai16_protocol_encode(&e, (uint8_t *)message.payload, sizeof(message.payload), &length) != CR(OK)) {
        count_event(&s_stats[TX_DHT22].invalid);
        return ESP_ERR_INVALID_ARG;
    }
    message.len = (int)length;
    return queue_message(&message);
}

// Card presentation: binary aai16.Envelope.sensor_telemetry.rc522, never JSON.
// The old JSON "event":"card_presented" is implied by the message type and topic;
// "uid_length" is the uid byte count. seq -> Envelope.sequence (attempt count, so
// rejected/dropped submissions still advance it), uptime_ms -> uptime_us.
// generation is the sensor's committed configuration generation at submission.
_Static_assert(36 + aai16_SensorTelemetry_size < TX_PAYLOAD_SIZE, "RC522 event exceeds TX slot");

esp_err_t mqtt_manager_publish_rc522_event(const uint8_t *uid, uint8_t uid_length)
{
    uint32_t seq = begin_attempt(TX_RC522_EVENT);
    if (uid == NULL || (uid_length != 4 && uid_length != 7 && uid_length != 10)) {
        count_event(&s_stats[TX_RC522_EVENT].invalid);
        return ESP_ERR_INVALID_ARG;
    }
    int64_t now_us = esp_timer_get_time();
    uint32_t generation;
    portENTER_CRITICAL(&s_config_lock);
    generation = s_transactions.generation[SENSOR_RC522];
    portEXIT_CRITICAL(&s_config_lock);

    aai16_Envelope e = aai16_Envelope_init_zero;
    e.has_protocol_version = e.has_boot_id = e.has_sequence = e.has_generation = true;
    e.protocol_version = AAI16_PROTOCOL_VERSION;
    e.boot_id = s_boot_id; e.sequence = seq; e.generation = generation;
    e.which_body = aai16_Envelope_sensor_telemetry_tag;
    aai16_SensorTelemetry *t = &e.body.sensor_telemetry;
    t->has_sensor = t->has_acquired_us = t->has_uptime_us = true;
    t->sensor = config_sensor_to_wire(SENSOR_RC522);
    t->acquired_us = t->uptime_us = (uint64_t)now_us;
    t->which_reading = aai16_SensorTelemetry_rc522_tag;
    t->reading.rc522.has_uid = true;
    t->reading.rc522.uid.size = uid_length;
    memcpy(t->reading.rc522.uid.bytes, uid, uid_length);

    tx_message_t message = {.kind = TX_RC522_EVENT, .created_ms = now_us / 1000};
    size_t length = 0;
    if (aai16_protocol_encode(&e, (uint8_t *)message.payload, sizeof(message.payload), &length) != CR(OK)) {
        count_event(&s_stats[TX_RC522_EVENT].invalid);
        return ESP_ERR_INVALID_ARG;
    }
    message.len = (int)length; // Binary payload: embedded NULs are not terminators.
    return queue_message(&message);
}


// --------------------------------------------------
// Retrieve validated configuration from queue
// --------------------------------------------------

bool mqtt_manager_peek_config_update(mqtt_config_update_t *out)
{
    // app_main polls at ~100 ms even if no complete configuration is queued.
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_config_lock);
    config_fragment_expire(&s_fragments, now);
    portEXIT_CRITICAL(&s_config_lock);
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
    return finish_config(update, CR(OK));
}

esp_err_t mqtt_manager_publish_gy511(
    float accel_x_g,
    float accel_y_g,
    float accel_z_g,
    float mag_x_ut,
    float mag_y_ut,
    float mag_z_ut)
{
    // Compatibility entry point for callers without acquisition metadata.
    return mqtt_manager_publish_gy511_at(accel_x_g, accel_y_g, accel_z_g,
        mag_x_ut, mag_y_ut, mag_z_ut, esp_timer_get_time(), 0);
}

uint32_t mqtt_manager_boot_id(void) { return s_boot_id; }

static aai16_ResultCode encode_motion(const motion_result_t *r, uint8_t *data,
                                    size_t capacity, size_t *length)
{
    aai16_Envelope e=aai16_Envelope_init_zero;
    e.has_protocol_version=e.has_boot_id=e.has_generation=true;
    e.protocol_version=AAI16_PROTOCOL_VERSION; e.boot_id=r->boot_id; e.generation=r->generation;
    e.which_body=aai16_Envelope_ml_prediction_tag;
    aai16_MlPrediction *p=&e.body.ml_prediction;
    p->has_model=p->has_model_epoch=p->has_status=p->has_model_ready=p->has_classification=p->has_interval_ms=p->has_uptime_us=true;
    p->model_epoch=r->model_epoch; p->status=(aai16_MotionStatus)(r->status+1);
    p->model_ready=r->model_ready; p->classification=aai16_MotionClass_CLASS_UNKNOWN;
    p->interval_ms=r->interval_ms; p->uptime_us=r->uptime_us;
    aai16_ModelIdentity *id=&p->model;
    id->has_family=id->has_version=id->has_sha256=id->has_source=true;
    id->family=(aai16_ModelFamily)r->model_identity.family; id->source=(aai16_ModelSource)r->model_identity.source;
    id->has_bank=id->source==aai16_ModelSource_SOURCE_STORED; id->bank=r->model_identity.bank;
    memcpy(id->version,r->model_identity.version,sizeof(id->version));
    id->sha256.size=32; memcpy(id->sha256.bytes,r->model_identity.sha256,32);
    p->has_backend=p->has_trial=true; p->backend=motion_inference_backend(); p->trial=false;
    p->has_clipped_total=p->has_queue_drops=p->has_window_resets=p->has_invoke_failures=true;
    p->clipped_total=r->clipped_total; p->queue_drops=r->queue_drops; p->window_resets=r->window_resets; p->invoke_failures=r->invoke_failures;
    p->has_arena_reserved_bytes=p->has_arena_used_bytes=true;
    p->arena_reserved_bytes=16384; p->arena_used_bytes=r->arena_used_bytes;
    if(r->status==MOTION_PREDICTION) {
        p->classification=r->class_id==0?aai16_MotionClass_CLASS_REST:aai16_MotionClass_CLASS_MOVING;
        p->has_first_sequence=p->has_last_sequence=p->has_window_start_us=p->has_window_end_us=p->has_inference_us=p->has_output_int8=p->has_clipped=true;
        p->first_sequence=r->first_sequence; p->last_sequence=r->last_sequence;
        p->window_start_us=r->window_start_us; p->window_end_us=r->window_end_us; p->inference_us=r->inference_us;
        p->output_int8.size=2; memcpy(p->output_int8.bytes,r->output_int8,2);
        p->logits_count=2; memcpy(p->logits,r->logits,sizeof(r->logits)); p->clipped=r->clipped;
    }
    return aai16_protocol_encode(&e,data,capacity,length);
}

esp_err_t mqtt_manager_publish_motion(const motion_result_t *r)
{
    (void)begin_attempt(TX_MOTION);
    if (!r ||
        (unsigned)r->status > MOTION_SENSOR_UNAVAILABLE ||
        (r->status == MOTION_PREDICTION && (!r->model_ready || r->class_id < 0 || r->class_id > 1)) ||
        r->uptime_us < 0 || r->uptime_us > esp_timer_get_time() || r->arena_used_bytes > UINT32_MAX) {
        count_event(&s_stats[TX_MOTION].invalid);
        return ESP_ERR_INVALID_ARG;
    }
    tx_message_t message = {.kind = TX_MOTION, .created_ms = r->uptime_us / 1000};
    // Fixed stack buffers and nanopb only. Identity/epoch are from this result,
    // never a later runtime snapshot. The shared queue copies the encoded bytes.
    size_t length = 0;
    if (encode_motion(r, (uint8_t *)message.payload, sizeof(message.payload), &length) != CR(OK)) {
        count_event(&s_stats[TX_MOTION].invalid);
        return ESP_ERR_INVALID_ARG;
    }
    message.len = (int)length; // Binary payload: embedded NULs are not terminators.
    return queue_message(&message);
}

esp_err_t mqtt_manager_publish_gy511_at(
    float accel_x_g, float accel_y_g, float accel_z_g,
    float mag_x_ut, float mag_y_ut, float mag_z_ut,
    int64_t acquired_us, int64_t jitter_us)
{
    uint32_t seq = begin_attempt(TX_GY511);
    if (!isfinite(accel_x_g) || !isfinite(accel_y_g) || !isfinite(accel_z_g) ||
        !isfinite(mag_x_ut) || !isfinite(mag_y_ut) || !isfinite(mag_z_ut) ||
        acquired_us < 0 || acquired_us > esp_timer_get_time()) {
        count_event(&s_stats[TX_GY511].invalid);
        return ESP_ERR_INVALID_ARG;
    }
    tx_message_t message = {
        .kind = TX_GY511,
        .created_ms = acquired_us / 1000,
    };

    // Format GY-511 readings as JSON.
    message.len = snprintf(
        message.payload,
        sizeof(message.payload),
        "{\"sensor\":\"gy511\","
        "\"boot_id\":\"%08" PRIx32 "\",\"seq\":%" PRIu32 ","
        "\"uptime_ms\":%" PRId64 ","
        "\"jitter_us\":%" PRId64 ","
        "\"acceleration_g\":{"
        "\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
        "\"magnetic_field_ut\":{"
        "\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}}",
        s_boot_id, seq, message.created_ms, jitter_us,
        accel_x_g,
        accel_y_g,
        accel_z_g,
        mag_x_ut,
        mag_y_ut,
        mag_z_ut
    );

    return queue_message(&message);
}


esp_err_t mqtt_manager_publish_bme280(
    float temperature_c,
    float pressure_hpa,
    float humidity_pct,
    bool has_humidity)
{
    uint32_t seq = begin_attempt(TX_BME280);
    if (!isfinite(temperature_c) || !isfinite(pressure_hpa) ||
        (has_humidity && !isfinite(humidity_pct))) {
        count_event(&s_stats[TX_BME280].invalid);
        return ESP_ERR_INVALID_ARG;
    }
    tx_message_t message = {
        .kind = TX_BME280,
        .created_ms = esp_timer_get_time() / 1000,
    };

    // A BMP280 has no humidity channel: publish null, never an invented value.
    char humidity[16] = "null";
    if (has_humidity) {
        snprintf(humidity, sizeof(humidity), "%.1f", humidity_pct);
    }
    message.len = snprintf(
        message.payload,
        sizeof(message.payload),
        "{\"sensor\":\"bme280\","
        "\"boot_id\":\"%08" PRIx32 "\",\"seq\":%" PRIu32 ","
        "\"uptime_ms\":%" PRId64 ","
        "\"temperature_c\":%.2f,"
        "\"pressure_hpa\":%.2f,"
        "\"humidity_pct\":%s}",
        s_boot_id, seq, message.created_ms,
        temperature_c,
        pressure_hpa,
        humidity
    );

    return queue_message(&message);
}


esp_err_t mqtt_manager_publish_max6675(float temperature_c)
{
    uint32_t seq = begin_attempt(TX_MAX6675);
    if (!isfinite(temperature_c)) {
        count_event(&s_stats[TX_MAX6675].invalid);
        return ESP_ERR_INVALID_ARG;
    }
    tx_message_t message = {
        .kind = TX_MAX6675,
        .created_ms = esp_timer_get_time() / 1000,
    };
    message.len = snprintf(
        message.payload,
        sizeof(message.payload),
        "{\"sensor\":\"max6675\","
        "\"boot_id\":\"%08" PRIx32 "\",\"seq\":%" PRIu32 ","
        "\"uptime_ms\":%" PRId64 ","
        "\"temperature_c\":%.2f}",
        s_boot_id, seq, message.created_ms,
        temperature_c
    );

    return queue_message(&message);
}
