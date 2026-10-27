// Host adapters for the installed SDK's MQTT packet/outbox and publish code.
// Network writes and locks are simulated; packet building and allocation/free
// are the real SDK implementations. No sockets, credentials or serial ports.
#pragma once
#include <assert.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_LOGD(tag, ...) do { (void)(tag); if (0) printf(__VA_ARGS__); } while (0)
#define ESP_LOGI ESP_LOGD
#define ESP_LOGW ESP_LOGD
#define ESP_LOGE ESP_LOGD
#define ESP_MEM_CHECK(tag, value, on_error) do { if (!(value)) { on_error; } } while (0)
#define heap_caps_malloc(size, caps) malloc(size)
#define heap_caps_calloc(count, size, caps) calloc(count, size)
#define MQTT_BUFFER_MEMORY 0
#define MQTT_OUTBOX_MEMORY 0
#define MQTT_MSG_ID_INCREMENTAL 1
#define MQTT_SKIP_PUBLISH_IF_DISCONNECTED 0
typedef enum { MQTT_PROTOCOL_V_3_1, MQTT_PROTOCOL_V_3_1_1, MQTT_PROTOCOL_V_5 } esp_mqtt_protocol_ver_t;
typedef struct { const char *filter; int qos; } esp_mqtt_topic_t;
typedef struct sdk_client *esp_mqtt_client_handle_t;
static inline int platform_random(int max) { return 1; }
int64_t platform_tick_get_ms(void);
