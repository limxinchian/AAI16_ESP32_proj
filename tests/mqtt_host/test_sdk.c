#include "sdk_platform.h"
#include "mqtt_msg.h"
#include "mqtt_outbox.h"

enum { MQTT_STATE_CONNECTED, MQTT_STATE_DISCONNECTED };
struct sdk_client {
    struct { int outbox_limit; } *config;
    outbox_handle_t outbox;
    int state;
    struct {
        mqtt_connection_t connection;
        int pending_msg_type, pending_publish_qos;
        uint16_t pending_msg_id;
    } mqtt_state;
};
static const char *TAG = "SDK_TEST";
static int64_t now_ms;
static int writes, write_result, lock_depth;
#define MQTT_API_LOCK(client) (++lock_depth)
#define MQTT_API_UNLOCK(client) (--lock_depth)
int64_t platform_tick_get_ms(void) { return now_ms; }
static int esp_mqtt_write(esp_mqtt_client_handle_t client)
{ writes++; return write_result; }
static void esp_mqtt_abort_connection(esp_mqtt_client_handle_t client)
{ client->state = MQTT_STATE_DISCONNECTED; }

// Generated from the installed mqtt_client.c, not a reimplementation of the
// publish/enqueue/cleanup logic. See run.py for exact extraction boundaries.
#include "sdk_functions.inc"

static void clear_outbox(esp_mqtt_client_handle_t client)
{
    outbox_delete_all_items(client->outbox);
    assert(outbox_get_size(client->outbox) == 0);
}

int main(void)
{
    struct sdk_client client = {0};
    client.config = calloc(1, sizeof(*client.config));
    assert(client.config);
    client.config->outbox_limit = 4096;
    client.outbox = outbox_init();
    assert(client.outbox);
    client.state = MQTT_STATE_CONNECTED;
    client.mqtt_state.connection.information.protocol_ver = MQTT_PROTOCOL_V_3_1_1;
    assert(mqtt_msg_buffer_init(&client.mqtt_state.connection, 1024) == ESP_OK);
    const char *topic = "aai16/sensors/dht22/data";
    const char *payload = "{\"sensor\":\"dht22\",\"seq\":1}";
    int len = strlen(payload);

    // store=false on enqueue does NOT publish QoS 0.
    assert(esp_mqtt_client_enqueue(&client, topic, payload, len, 0, 0, false) == -1);
    assert(writes == 0 && outbox_get_size(client.outbox) == 0);
    // store=true copies and queues it. Normal send deletes it (not expiry).
    assert(esp_mqtt_client_enqueue(&client, topic, payload, len, 0, 0, true) == 0);
    assert(writes == 0 && outbox_get_size(client.outbox) > 0);
    sdk_drain_one_queued(&client);
    assert(writes == 1 && outbox_get_size(client.outbox) == 0);
    // Direct QoS 0 writes immediately, without an SDK outbox allocation.
    assert(esp_mqtt_client_publish(&client, topic, payload, len, 0, 0) == 0);
    assert(writes == 2 && outbox_get_size(client.outbox) == 0);
    client.state = MQTT_STATE_DISCONNECTED;
    assert(esp_mqtt_client_publish(&client, topic, payload, len, 0, 0) == -1);
    assert(writes == 2 && outbox_get_size(client.outbox) == 0);
    client.state = MQTT_STATE_CONNECTED;
    write_result = ESP_FAIL;
    assert(esp_mqtt_client_publish(&client, topic, payload, len, 0, 0) == -1);
    assert(client.state == MQTT_STATE_DISCONNECTED && outbox_get_size(client.outbox) == 0);
    write_result = ESP_OK;
    client.state = MQTT_STATE_CONNECTED;

    // QoS 1 ACKs are retained after transmission until PUBACK or expiry.
    char ack[] = "{\"sensor\":\"dht22\",\"status\":\"applied\",\"interval_ms\":5000}";
    int ack_id = esp_mqtt_client_enqueue(&client, "aai16/config/ack", ack, strlen(ack), 1, 0, true);
    assert(ack_id > 0);
    uint64_t ack_size = outbox_get_size(client.outbox);
    ack[0] = 'X'; // Caller can reuse its buffer; the outbox owns a copy.
    outbox_item_handle_t item = outbox_get(client.outbox, ack_id);
    size_t wire_len; uint16_t id; int type, qos;
    uint8_t *wire = outbox_item_get_data(item, &wire_len, &id, &type, &qos);
    assert(wire[wire_len - strlen(ack)] == '{');
    sdk_drain_one_queued(&client);
    assert(outbox_get_size(client.outbox) == ack_size);
    assert(outbox_item_get_pending(item) == TRANSMITTED);
    assert(outbox_delete(client.outbox, ack_id, MQTT_MSG_TYPE_PUBLISH) == ESP_OK);
    assert(outbox_get_size(client.outbox) == 0);
    // Expiry uses > timeout, and can delete QoS 0 entries with message id zero.
    assert(esp_mqtt_client_enqueue(&client, topic, payload, len, 0, 0, true) == 0);
    assert(outbox_delete_single_expired(client.outbox, 30000, 30000) == -1);
    assert(outbox_delete_single_expired(client.outbox, 30001, 30000) == 0);
    assert(outbox_get_size(client.outbox) == 0);

    // Deterministic 15-minute quiet-broker scenario: one queued send per
    // 1000 ms idle read poll, with a 500 ms GY-511 and 2000 ms DHT22 producer.
    // This models the observed SDK branch; it does not emulate network/RTOS timing.
    const char *topics[] = {"aai16/sensors/dht22/data", "aai16/sensors/gy511/data"};
    const char *payloads[] = {
        "{\"sensor\":\"dht22\",\"boot_id\":\"12345678\",\"seq\":1000,\"uptime_ms\":600000,\"temperature_c\":27.0,\"humidity_pct\":59.1}",
        "{\"sensor\":\"gy511\",\"boot_id\":\"12345678\",\"seq\":1000,\"uptime_ms\":600000,\"acceleration_g\":{\"x\":0.123,\"y\":0.123,\"z\":1.001},\"magnetic_field_ut\":{\"x\":12.34,\"y\":12.34,\"z\":12.34}}"
    };
    for (int direct = 0; direct <= 1; direct++) {
        clear_outbox(&client);
        writes = 0;
        int admitted = 0, rejected = 0, expired = 0;
        uint64_t peak = 0;
        for (now_ms = 0; now_ms < 900000; now_ms += 500) {
            for (int sensor = 0; sensor < 2; sensor++) {
                if (sensor == 0 && now_ms % 2000) continue;
                int size = strlen(payloads[sensor]);
                if (!direct && outbox_get_size(client.outbox) + size + 64 > 4096 - 1024) {
                    rejected++;
                } else {
                    int result = direct
                        ? esp_mqtt_client_publish(&client, topics[sensor], payloads[sensor], size, 0, 0)
                        : esp_mqtt_client_enqueue(&client, topics[sensor], payloads[sensor], size, 0, 0, true);
                    assert(result == 0); admitted++;
                }
            }
            if (outbox_get_size(client.outbox) > peak) peak = outbox_get_size(client.outbox);
            while (outbox_delete_single_expired(client.outbox, now_ms, 30000) >= 0) expired++;
            if (now_ms % SDK_IDLE_POLL_MS == 0) sdk_drain_one_queued(&client);
        }
        printf("SDK quiet-broker model (%s): admitted=%d rejected=%d writes=%d peak=%" PRIu64 " expired=%d\n",
               direct ? "direct QoS0" : "stored QoS0", admitted, rejected, writes, peak, expired);
        if (direct) assert(admitted == 2250 && writes == 2250 && rejected == 0 && peak == 0);
        else assert(rejected > 1000 && peak <= 3072 && peak > 2800 && writes == 900 && expired == 0);
    }
    // Outbox exhaustion is still enforced for ACKs; QoS 0 does not compete.
    int result;
    do { result = esp_mqtt_client_enqueue(&client, "aai16/config/ack", ack, strlen(ack), 1, 0, true); }
    while (result >= 0);
    assert(result == -2);
    uint64_t full_size = outbox_get_size(client.outbox);
    assert(full_size < 4096 + 64); // SDK admission excludes packet framing.
    assert(esp_mqtt_client_publish(&client, topic, payload, len, 0, 0) == 0);
    assert(outbox_get_size(client.outbox) == full_size && lock_depth == 0);
    mqtt_msg_buffer_destroy(&client.mqtt_state.connection);
    outbox_destroy(client.outbox);
    free(client.config);
    puts("PASS: installed SDK publish/enqueue semantics, QoS0 cleanup, ACK ownership/lifecycle, expiry, saturation model");
    return 0;
}
