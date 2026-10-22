#include "mhz1911a_sensor.h"
#include <stdbool.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define CO2_UART UART_NUM_2
#define CO2_EVENT_COUNT 8
#define CO2_QUIET_US 20000LL

// Only this measurement request is supported. No calibration/config commands.
static const uint8_t s_read_request[] = {0xff, 0x01, 0x86, 0, 0, 0, 0, 0, 0x79};
static bool s_owned, s_initialized;
static QueueHandle_t s_events;
static mhz1911a_diagnostics_t s_diag;

static TickType_t co2_ticks(int64_t us)
{
    const int64_t tick_us = portTICK_PERIOD_MS * 1000LL;
    return (TickType_t)((us + tick_us - 1) / tick_us);
}

esp_err_t mhz1911a_sensor_decode(const uint8_t *frame, size_t length,
                                mhz1911a_reading_t *out)
{
    if (!frame || !out) return ESP_ERR_INVALID_ARG;
    if (length != MHZ1911A_FRAME_BYTES || frame[0] != 0xff || frame[1] != 0x86)
        return ESP_ERR_INVALID_RESPONSE;
    uint8_t sum = 0;
    for (unsigned i = 1; i < MHZ1911A_FRAME_BYTES; ++i) sum = (uint8_t)(sum + frame[i]);
    if (sum) return ESP_ERR_INVALID_CRC;
    unsigned ppm = (unsigned)frame[2] * 256 + frame[3];
    if (ppm < MHZ1911A_MIN_PPM || ppm > MHZ1911A_MAX_PPM) return ESP_ERR_INVALID_RESPONSE;
    out->co2_ppm = ppm;
    return ESP_OK;
}

esp_err_t mhz1911a_sensor_deinit(void)
{
    if (!s_owned) return ESP_OK;
    s_initialized = false;
    esp_err_t err = uart_driver_delete(CO2_UART);
    if (err == ESP_OK) { s_owned = false; s_events = NULL; }
    return err;
}

esp_err_t mhz1911a_sensor_init(void)
{
    if (s_initialized) return ESP_OK;
    if (s_owned || uart_is_driver_installed(CO2_UART)) return ESP_ERR_INVALID_STATE;
    esp_err_t err = uart_driver_install(CO2_UART, 512, 0, CO2_EVENT_COUNT, &s_events, 0);
    if (err != ESP_OK) return err;
    s_owned = true;
    const uart_config_t config = {
        .baud_rate = 9600, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    err = uart_param_config(CO2_UART, &config);
    if (err == ESP_OK) err = uart_set_pin(CO2_UART, 15, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        esp_err_t cleanup = mhz1911a_sensor_deinit();
        if (cleanup != ESP_OK) ESP_LOGE("SENSOR", "mhz1911a | ERROR | UART cleanup=%s", esp_err_to_name(cleanup));
        return err;
    }
    s_diag = (mhz1911a_diagnostics_t){0};
    s_initialized = true;
    return ESP_OK;
}

static bool co2_events(void)
{
    unsigned count = uxQueueMessagesWaiting(s_events);
    bool fault = count >= CO2_EVENT_COUNT;
    if (fault) { ++s_diag.event_overflows; count = CO2_EVENT_COUNT; }
    uart_event_t event;
    for (unsigned i = 0; i < count && xQueueReceive(s_events, &event, 0) == pdTRUE; ++i) {
        switch (event.type) {
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL: ++s_diag.rx_overflows; fault = true; break;
        case UART_FRAME_ERR:
        case UART_PARITY_ERR:
        case UART_BREAK: ++s_diag.uart_errors; fault = true; break;
        default: break;
        }
    }
    return fault;
}

static esp_err_t co2_buffered(size_t *count)
{
    esp_err_t err = uart_get_buffered_data_len(CO2_UART, count);
    if (err != ESP_OK) ++s_diag.api_errors;
    return err;
}

static esp_err_t co2_discard(void)
{
    size_t count = 0;
    esp_err_t err = co2_buffered(&count);
    if (err == ESP_OK) s_diag.discarded_bytes += count;
    esp_err_t flush = uart_flush_input(CO2_UART);
    if (flush != ESP_OK) ++s_diag.api_errors;
    // Preserve queued faults; never reset the queue across a concurrent ISR.
    return err != ESP_OK ? err : flush;
}

static esp_err_t co2_exchange(mhz1911a_reading_t *out)
{
    bool fault = co2_events();
    esp_err_t err = co2_discard();
    fault |= co2_events();
    if (err != ESP_OK) return err;
    if (fault) return ESP_FAIL;
    err = uart_wait_tx_done(CO2_UART, co2_ticks(100000));
    if (err != ESP_OK) { ++s_diag.tx_failures; return err; }
    // Reject input still arriving at the request boundary. Buffered late replies
    // from a previous failed transaction must not count as a fresh measurement.
    vTaskDelay(co2_ticks(CO2_QUIET_US));
    size_t buffered = 0;
    err = co2_buffered(&buffered);
    if (err != ESP_OK) return err;
    if (co2_events()) return ESP_FAIL;
    if (buffered) return ESP_ERR_INVALID_RESPONSE;

    const int64_t deadline = esp_timer_get_time() + MHZ1911A_RESPONSE_TIMEOUT_MS * 1000LL;
    ++s_diag.requests;
    // One nonblocking FIFO enqueue on our exclusively owned UART. Short writes
    // fail this acquisition; never append a remainder or retry inside the read.
    int sent = uart_tx_chars(CO2_UART, (const char *)s_read_request, sizeof(s_read_request));
    err = uart_wait_tx_done(CO2_UART, co2_ticks(100000));
    if (sent != (int)sizeof(s_read_request) || err != ESP_OK) {
        ++s_diag.tx_failures;
        return err != ESP_OK ? err : ESP_FAIL;
    }
    uint8_t frame[MHZ1911A_FRAME_BYTES];
    size_t used = 0;
    int64_t complete_at = -1;
    while (esp_timer_get_time() < deadline) {
        if (co2_events()) return ESP_FAIL;
        err = co2_buffered(&buffered);
        if (err != ESP_OK) return err;
        if (buffered) {
            // One extra byte distinguishes an exact response from noise/echo/
            // duplicates. Do not seek a plausible subframe inside corrupt input.
            uint8_t bytes[MHZ1911A_FRAME_BYTES + 1];
            int count = uart_read_bytes(CO2_UART, bytes, sizeof(bytes), 0);
            if (count < 0) { ++s_diag.api_errors; return ESP_FAIL; }
            s_diag.rx_bytes += count;
            if (co2_events()) return ESP_FAIL;
            if (used + count > sizeof(frame)) return ESP_ERR_INVALID_RESPONSE;
            memcpy(frame + used, bytes, count);
            used += count;
            if (used == sizeof(frame) && complete_at < 0) complete_at = esp_timer_get_time();
        }
        if (complete_at >= 0 && esp_timer_get_time() - complete_at >= CO2_QUIET_US) {
            // Recheck the tail and events before returning a validated result.
            err = co2_buffered(&buffered);
            if (err != ESP_OK) return err;
            if (co2_events()) return ESP_FAIL;
            if (buffered) return ESP_ERR_INVALID_RESPONSE;
            return mhz1911a_sensor_decode(frame, used, out);
        }
        // One tick bounds polling and yields even for a noisy/stalled UART.
        vTaskDelay(1);
    }
    if (used) ++s_diag.partial_responses;
    return ESP_ERR_TIMEOUT;
}

esp_err_t mhz1911a_sensor_read(mhz1911a_reading_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    int64_t start = esp_timer_get_time();
    mhz1911a_reading_t reading;
    esp_err_t err = co2_exchange(&reading);
    if (err == ESP_OK) { *out = reading; ++s_diag.valid_frames; }
    else {
        s_diag.timeouts += err == ESP_ERR_TIMEOUT;
        s_diag.checksum_failures += err == ESP_ERR_INVALID_CRC;
        s_diag.invalid_frames += err == ESP_ERR_INVALID_RESPONSE;
        (void)co2_discard(); // Next acquisition also checks this boundary.
    }
    s_diag.last_read_us = esp_timer_get_time() - start;
    if (s_diag.last_read_us > s_diag.max_read_us) s_diag.max_read_us = s_diag.last_read_us;
    return err;
}

void mhz1911a_sensor_get_diagnostics(mhz1911a_diagnostics_t *out)
{
    if (out) *out = s_diag;
}
