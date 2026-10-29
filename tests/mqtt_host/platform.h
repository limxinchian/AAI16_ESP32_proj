// Deterministic host doubles. These tests do not emulate FreeRTOS scheduling,
// ESP-MQTT internals, Wi-Fi, or hardware; the ESP-IDF build checks real APIs.
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_SIZE 0x104
#define ESP_ERR_TIMEOUT 0x107
#define ESP_ERR_INVALID_RESPONSE 0x108
#define ESP_ERR_INVALID_CRC 0x109
const char *esp_err_to_name(esp_err_t error);
#define ESP_EVENT_ANY_ID -1
#define CONFIG_MQTT_REPORT_DELETED_MESSAGES 1
#define BIT0 1
#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) (ms)
#define portTICK_PERIOD_MS 1
typedef uint32_t TickType_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
void test_enter_critical(void);
void test_exit_critical(void);
#define portENTER_CRITICAL(lock) ((void)(lock), test_enter_critical())
#define portEXIT_CRITICAL(lock) ((void)(lock), test_exit_critical())
// Keep format arguments compiled, without printing noisy test diagnostics.
#define ESP_LOGI(tag, ...) do { (void)(tag); if (0) printf(__VA_ARGS__); } while (0)
#define ESP_LOGW ESP_LOGI
#define ESP_LOGE ESP_LOGI

typedef struct fake_queue *QueueHandle_t;
typedef unsigned *EventGroupHandle_t;
typedef void *TaskHandle_t;
typedef const char *esp_event_base_t;
typedef void *esp_mqtt_client_handle_t;
typedef enum {
    MQTT_EVENT_CONNECTED, MQTT_EVENT_DISCONNECTED, MQTT_EVENT_SUBSCRIBED,
    MQTT_EVENT_DATA, MQTT_EVENT_ERROR, MQTT_EVENT_PUBLISHED, MQTT_EVENT_DELETED
} esp_mqtt_event_id_t;
typedef struct {
    int error_type, esp_transport_sock_errno, connect_return_code;
} fake_error_t;
typedef struct {
    int current_data_offset, topic_len, data_len, total_data_len, msg_id;
    char *topic, *data;
    bool retain;
    fake_error_t *error_handle;
} esp_mqtt_event_t, *esp_mqtt_event_handle_t;
typedef struct {
    struct { struct { const char *uri; } address; } broker;
    struct {
        const char *username, *client_id;
        struct { const char *password; } authentication;
    } credentials;
    struct { int limit; } outbox;
    struct { int reconnect_timeout_ms, timeout_ms; bool disable_auto_reconnect; } network;
} esp_mqtt_client_config_t;

QueueHandle_t xQueueCreate(unsigned capacity, unsigned item_size);
int xQueueSend(QueueHandle_t queue, const void *item, unsigned wait);
int xQueueReceive(QueueHandle_t queue, void *item, unsigned wait);
int xQueuePeek(QueueHandle_t queue, void *item, unsigned wait);
unsigned uxQueueMessagesWaiting(QueueHandle_t queue);
void vQueueDelete(QueueHandle_t queue);
EventGroupHandle_t xEventGroupCreate(void);
unsigned xEventGroupGetBits(EventGroupHandle_t group);
void xEventGroupSetBits(EventGroupHandle_t group, unsigned bits);
void xEventGroupClearBits(EventGroupHandle_t group, unsigned bits);
void vEventGroupDelete(EventGroupHandle_t group);
int xTaskCreate(void (*fn)(void *), const char *name, unsigned stack,
                void *arg, unsigned priority, TaskHandle_t *handle);
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task);
void vTaskDelay(TickType_t ticks);
int xTaskNotifyWait(uint32_t entry, uint32_t exit, uint32_t *value, TickType_t wait);
void vTaskDelete(TaskHandle_t task);
esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *config);
int esp_mqtt_client_register_event(esp_mqtt_client_handle_t client, int id,
    void (*handler)(void *, esp_event_base_t, int32_t, void *), void *arg);
int esp_mqtt_client_start(esp_mqtt_client_handle_t client);
int esp_mqtt_client_stop(esp_mqtt_client_handle_t client);
int esp_mqtt_client_destroy(esp_mqtt_client_handle_t client);
int esp_mqtt_client_subscribe(esp_mqtt_client_handle_t client, const char *topic, int qos);
int esp_mqtt_client_enqueue(esp_mqtt_client_handle_t client, const char *topic,
                           const char *data, int len, int qos, int retain, bool store);
int esp_mqtt_client_publish(esp_mqtt_client_handle_t client, const char *topic,
                           const char *data, int len, int qos, int retain);
int esp_mqtt_client_get_outbox_size(esp_mqtt_client_handle_t client);
int64_t esp_timer_get_time(void);
uint32_t esp_random(void);
uint32_t esp_get_free_heap_size(void);
uint32_t esp_get_minimum_free_heap_size(void);
