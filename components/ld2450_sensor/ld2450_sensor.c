#include "ld2450_sensor.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define RADAR_UART UART_NUM_1
#define RX_BYTES 1024
#define EVENT_COUNT 16
#define BATCH_BYTES 256

static bool s_owned, s_initialized;
static QueueHandle_t s_events;
static ld2450_parser_t s_parser;
static ld2450_diagnostics_t s_diag;

esp_err_t ld2450_sensor_deinit(void)
{
    if (!s_owned) return ESP_OK;
    s_initialized = false;
    esp_err_t err = uart_driver_delete(RADAR_UART);
    if (err == ESP_OK) { s_owned = false; s_events = NULL; }
    return err;
}

esp_err_t ld2450_sensor_init(void)
{
    if (s_initialized) return ESP_OK;
    if (s_owned || uart_is_driver_installed(RADAR_UART)) return ESP_ERR_INVALID_STATE;
    esp_err_t err = uart_driver_install(RADAR_UART, RX_BYTES, 0, EVENT_COUNT, &s_events, 0);
    if (err != ESP_OK) return err;
    s_owned = true;
    const uart_config_t config = {
        .baud_rate = 256000, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    err = uart_param_config(RADAR_UART, &config);
    if (err == ESP_OK) err = uart_set_pin(RADAR_UART, 17, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        esp_err_t cleanup = ld2450_sensor_deinit();
        if (cleanup != ESP_OK) ESP_LOGE("SENSOR", "ld2450  | ERROR | UART cleanup failed: %s", esp_err_to_name(cleanup));
        return err;
    }
    s_parser = (ld2450_parser_t){0};
    s_diag = (ld2450_diagnostics_t){0};
    s_initialized = true;
    ESP_LOGI("SENSOR", "ld2450  | HEALTH | UART1 configured | TX=17 RX=18 | 256000 8N1 | receive only");
    return ESP_OK;
}

static bool check_event(const uart_event_t *event)
{
    switch (event->type) {
    case UART_FIFO_OVF:
    case UART_BUFFER_FULL:
        s_diag.rx_overflows++;
        return true;
    case UART_FRAME_ERR:
    case UART_PARITY_ERR:
    case UART_BREAK:
        s_diag.uart_errors++;
        return true;
    default:
        return false;
    }
}

static bool drain_events(void)
{
    unsigned count = uxQueueMessagesWaiting(s_events);
    bool fault = count >= EVENT_COUNT;
    // A full queue may have lost error notifications: fail closed.
    if (fault) s_diag.event_overflows++;
    if (count > EVENT_COUNT) count = EVENT_COUNT;
    uart_event_t event;
    for (unsigned i = 0; i < count && xQueueReceive(s_events, &event, 0) == pdTRUE; ++i) {
        fault |= check_event(&event);
    }
    return fault;
}

static esp_err_t discard_input(void)
{
    size_t buffered = 0;
    esp_err_t err = uart_get_buffered_data_len(RADAR_UART, &buffered);
    if (err == ESP_OK) s_diag.discarded_bytes += buffered;
    ld2450_parser_discard(&s_parser);
    esp_err_t flush = uart_flush_input(RADAR_UART);
    // No queue reset: a concurrent UART fault must remain visible next cycle.
    return err != ESP_OK ? err : flush;
}

esp_err_t ld2450_sensor_discard_pending(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    // Account for events accumulated while disabled without carrying those
    // old faults into the resumed health state. New events remain queued.
    (void)drain_events();
    return discard_input();
}

esp_err_t ld2450_sensor_receive(ld2450_report_t *latest, unsigned *frames)
{
    if (!latest || !frames) return ESP_ERR_INVALID_ARG;
    *frames = 0;
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    size_t buffered = 0;
    esp_err_t err = uart_get_buffered_data_len(RADAR_UART, &buffered);
    if (err != ESP_OK) {
        s_diag.uart_errors++;
        (void)discard_input();
        return err;
    }
    bool fault = drain_events();
    if (!buffered && !fault) {
        uart_event_t event;
        if (xQueueReceive(s_events, &event, pdMS_TO_TICKS(100)) == pdTRUE) fault = check_event(&event);
    }
    int64_t start_us = esp_timer_get_time();
    fault |= drain_events();
    err = uart_get_buffered_data_len(RADAR_UART, &buffered);
    if (buffered > s_diag.max_buffered_bytes) s_diag.max_buffered_bytes = buffered;
    if (err != ESP_OK) { s_diag.uart_errors++; fault = true; }
    if (fault) {
        esp_err_t cleanup = discard_input();
        if (err == ESP_OK) err = cleanup != ESP_OK ? cleanup : ESP_FAIL;
    } else {
        uint8_t data[BATCH_BYTES];
        int length = uart_read_bytes(RADAR_UART, data, sizeof(data), 0);
        if (length < 0 || drain_events()) {
            if (length < 0) s_diag.uart_errors++;
            else s_diag.discarded_bytes += length;
            esp_err_t cleanup = discard_input();
            err = cleanup != ESP_OK ? cleanup : ESP_FAIL;
        } else {
            uint32_t discarded = s_parser.discarded_bytes;
            s_diag.rx_bytes += length;
            *frames = ld2450_parser_feed(&s_parser, data, length, esp_timer_get_time(), latest);
            if (s_parser.discarded_bytes != discarded) err = ESP_ERR_INVALID_RESPONSE;
        }
    }
    int64_t elapsed = esp_timer_get_time() - start_us;
    if (elapsed > s_diag.max_processing_us) s_diag.max_processing_us = elapsed;
    return err;
}

void ld2450_sensor_get_diagnostics(ld2450_diagnostics_t *out)
{
    *out = s_diag; // Called only by the acquisition owner.
    out->frames = s_parser.frames;
    out->framing_errors = s_parser.framing_errors;
    out->partial_timeouts = s_parser.partial_timeouts;
    out->discarded_bytes += s_parser.discarded_bytes;
}
