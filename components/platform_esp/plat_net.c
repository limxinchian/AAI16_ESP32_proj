/* Wi-Fi station + esp-mqtt. Topics (all payloads are binary Protobuf):
 *   in  (this node):   aai16/<id>/in/{config,rules,bridge,model/chunk,model/control,command}
 *   out (this node):   aai16/<id>/out/{config_ack,rules_ack,bridge_ack,model_chunk_ack,model_status,inference,
 *                                       heartbeat,notification}   and   aai16/<id>/out/reading/<sensor_id>
 *   bridge input:      aai16/+/out/reading/+   (other nodes' readings; ignored unless a bridge source matches)
 *   rule -> node:      aai16/<target>/in/command
 * Incoming messages are reassembled and queued for the single node task. NOT hardware-verified. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "node_topics.h"
#include "platform_esp.h"

static const char *TAG = "plat_net";

static char s_id[16];
static char s_ssid[33], s_pass[65];
static QueueHandle_t s_inbox;
static esp_mqtt_client_handle_t s_client;
static volatile bool s_connected;
static bool s_mqtt_started;

/* ------------------------------------------------------------- inbound */
static struct {
    bool active;
    int kind;
    char from[16];
    uint8_t *buf;
    size_t total, got;
} s_asm;

static void on_data(esp_mqtt_event_handle_t ev)
{
    if (ev->current_data_offset == 0) { /* first fragment carries the topic */
        s_asm.active = false;
        free(s_asm.buf);
        s_asm.buf = NULL;
        char topic[96];
        if (ev->topic_len <= 0 || ev->topic_len >= (int)sizeof(topic)) return;
        memcpy(topic, ev->topic, (size_t)ev->topic_len);
        topic[ev->topic_len] = '\0';
        node_in_t kind;
        if (!node_topic_classify(s_id, topic, &kind, s_asm.from)) return;
        s_asm.kind = (int)kind;
        if (ev->total_data_len <= 0 || ev->total_data_len > PLAT_MAX_MESSAGE) {
            ESP_LOGW(TAG, "dropping %d-byte message on %s (limit %d)", ev->total_data_len, topic, PLAT_MAX_MESSAGE);
            return;
        }
        s_asm.buf = heap_caps_malloc((size_t)ev->total_data_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_asm.buf) { ESP_LOGE(TAG, "out of memory for message"); return; }
        s_asm.total = (size_t)ev->total_data_len;
        s_asm.got = 0;
        s_asm.active = true;
    }
    if (!s_asm.active) return;
    if (s_asm.got + (size_t)ev->data_len > s_asm.total) { s_asm.active = false; free(s_asm.buf); s_asm.buf = NULL; return; }
    memcpy(s_asm.buf + s_asm.got, ev->data, (size_t)ev->data_len);
    s_asm.got += (size_t)ev->data_len;
    if (s_asm.got == s_asm.total) {
        plat_msg_t m = {.kind = s_asm.kind, .data = s_asm.buf, .len = s_asm.total};
        strcpy(m.from, s_asm.from);
        s_asm.active = false;
        s_asm.buf = NULL;
        if (xQueueSend(s_inbox, &m, 0) != pdTRUE) {
            ESP_LOGW(TAG, "node inbox full: message dropped");
            free(m.data);
        }
    }
}

static void mqtt_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base;
    esp_mqtt_event_handle_t ev = data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED: {
        char t[64];
        snprintf(t, sizeof(t), "aai16/%s/in/#", s_id);
        esp_mqtt_client_subscribe(s_client, t, 1);
        esp_mqtt_client_subscribe(s_client, "aai16/+/out/reading/+", 0);
        s_connected = true;
        ESP_LOGI(TAG, "MQTT connected");
        break;
    }
    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        ESP_LOGW(TAG, "MQTT disconnected");
        break;
    case MQTT_EVENT_DATA:
        on_data(ev);
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------ outbound */
bool plat_net_publish(void *ctx, node_out_t kind, const char *target, const uint8_t *buf, size_t len)
{
    (void)ctx;
    if (!s_client) return false;
    char topic[80];
    int qos;
    if (!node_topic_build(kind, s_id, target, buf, len, topic, sizeof(topic), &qos)) return false;
    if (qos == 0 && !s_connected) return false; /* telemetry: drop rather than queue when the link is down */
    /* store must be true for every QoS: with store=false esp-mqtt does NOT send QoS 0 messages, it returns -1.
     * (Found on the real board: heartbeat/ACKs (QoS 1) arrived, readings (QoS 0) never did.) */
    return esp_mqtt_client_enqueue(s_client, topic, (const char *)buf, (int)len, qos, 0, true) >= 0;
}

/* ---------------------------------------------------------------- Wi-Fi */
static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        ESP_LOGW(TAG, "Wi-Fi disconnected, retrying");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "got IP");
        if (!s_mqtt_started && s_client) {
            esp_mqtt_client_start(s_client);
            s_mqtt_started = true;
        }
    }
}

bool plat_net_start(const char *node_id, const char *ssid, const char *password, const char *broker_uri, QueueHandle_t inbox)
{
    snprintf(s_id, sizeof(s_id), "%s", node_id);
    snprintf(s_ssid, sizeof(s_ssid), "%s", ssid);
    snprintf(s_pass, sizeof(s_pass), "%s", password);
    s_inbox = inbox;

    const esp_mqtt_client_config_t mc = {
        .broker.address.uri = broker_uri,
        .credentials.client_id = s_id,
        .session.keepalive = 30,
        .network.reconnect_timeout_ms = 5000,
        .buffer.size = 4096,
        .buffer.out_size = 2048,
        .task.stack_size = 6144,
    };
    s_client = esp_mqtt_client_init(&mc);
    if (!s_client) return false;
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event, NULL);

    if (esp_netif_init() != ESP_OK) return false;
    if (esp_event_loop_create_default() != ESP_OK) return false;
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wc = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&wc) != ESP_OK) return false;
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL);
    wifi_config_t cfg = {0};
    strlcpy((char *)cfg.sta.ssid, s_ssid, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, s_pass, sizeof(cfg.sta.password));
    cfg.sta.threshold.authmode = s_pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &cfg);
    return esp_wifi_start() == ESP_OK;
}
