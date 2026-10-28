#pragma once
#include <cstddef>
#include <cstdint>
#include <cstdio>
using esp_err_t = int;
constexpr int ESP_OK=0, ESP_ERR_INVALID_STATE=1, ESP_ERR_NO_MEM=2;
using BaseType_t=int;
using UBaseType_t=unsigned;
using TickType_t=uint32_t;
using TaskHandle_t=void*;
using QueueHandle_t=void*;
struct StaticQueue_t { int unused; };
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(lock) ((void)(lock))
#define portEXIT_CRITICAL(lock) ((void)(lock))
#define pdMS_TO_TICKS(ms) ((ms)/10)
constexpr int pdTRUE=1, pdFALSE=0, pdPASS=1;
constexpr unsigned MALLOC_CAP_INTERNAL=1, MALLOC_CAP_8BIT=2;
int64_t esp_timer_get_time();
void *heap_caps_aligned_alloc(size_t,size_t,unsigned);
size_t heap_caps_get_free_size(unsigned);
size_t heap_caps_get_minimum_free_size(unsigned);
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t);
QueueHandle_t xQueueCreateStatic(unsigned,unsigned,uint8_t*,StaticQueue_t*);
void vQueueDelete(QueueHandle_t);
BaseType_t xQueueSend(QueueHandle_t,const void*,TickType_t);
BaseType_t xQueueReceive(QueueHandle_t,void*,TickType_t);
BaseType_t xTaskCreatePinnedToCore(void(*)(void*),const char*,unsigned,void*,unsigned,TaskHandle_t*,int);
void test_log(const char*,...);
#define ESP_LOGI(tag,...) test_log(__VA_ARGS__)
#define ESP_LOGE(tag,...) test_log(__VA_ARGS__)
