// Deterministic host doubles, not an RTOS/hardware emulator.
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_RESPONSE 0x108
#define ESP_ERR_INVALID_CRC 0x109
#define ESP_ERR_TIMEOUT 0x107
const char *esp_err_to_name(esp_err_t error);

typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
#define pdTRUE 1
#define pdPASS 1
#define pdFALSE 0
// Match this project's 100 Hz tick configuration.
#define pdMS_TO_TICKS(ms) ((ms) / 10)
#define portTICK_PERIOD_MS 10
#define portMAX_DELAY UINT32_MAX
typedef enum { eSetValueWithoutOverwrite } eNotifyAction;
BaseType_t xTaskNotify(TaskHandle_t task, uint32_t value, eNotifyAction action);
BaseType_t xTaskNotifyWait(uint32_t clear_entry, uint32_t clear_exit,
                           uint32_t *value, TickType_t wait);
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
void test_enter_critical(void);
void test_exit_critical(void);
#define portENTER_CRITICAL(lock) do { (void)(lock); test_enter_critical(); } while (0)
#define portEXIT_CRITICAL(lock) do { (void)(lock); test_exit_critical(); } while (0)
void vTaskDelay(TickType_t ticks);
void vTaskDelayUntil(TickType_t *last_wake, TickType_t period);
TickType_t xTaskGetTickCount(void);
BaseType_t xTaskCreate(void (*fn)(void *), const char *name, unsigned stack,
                      void *arg, unsigned priority, TaskHandle_t *handle);
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task);

extern int64_t test_now_us;
extern unsigned test_log_calls;
extern char test_last_log[512];
extern unsigned test_error_logs, test_recovery_logs, test_summary_logs, test_config_logs;
extern char test_last_level;
bool test_logs_contain(const char *text);
int64_t esp_timer_get_time(void);
void test_log(char level, const char *tag, const char *format, ...)
    __attribute__((format(printf, 3, 4)));
#define ESP_LOGI(tag, ...) test_log('I', tag, __VA_ARGS__)
#define ESP_LOGE(tag, ...) test_log('E', tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) test_log('W', tag, __VA_ARGS__)

typedef int gpio_num_t;
#define GPIO_NUM_6 6
typedef void *i2c_master_bus_handle_t;
typedef void *i2c_master_dev_handle_t;
#define I2C_ADDR_BIT_LEN_7 0
typedef struct {
    int dev_addr_length;
    unsigned device_address;
    unsigned scl_speed_hz;
} i2c_device_config_t;
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
    const i2c_device_config_t *config, i2c_master_dev_handle_t *device);
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t device);
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t device,
    const uint8_t *buffer, size_t length, int timeout_ms);
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t device,
    const uint8_t *reg, size_t reg_length, uint8_t *buffer, size_t length, int timeout_ms);
