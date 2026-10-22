#include "mhz_platform.h"
#include "mhz1911a_sensor.h"
#include "ld2450_sensor.h"
#include <assert.h>
#include <string.h>

// Adapted from the Phase 1A UART1 isolation and Phase 1B exchange tests.
// This is the user's single real capture; all timing/faults below are doubles.
static const uint8_t captured[] = {0xff,0x86,0x06,0xea,0x43,0,0,0,0x47};
static const char *scenario;
static bool fault_issued;
static unsigned tx_calls;
static int64_t sent_us = -1;
static uint8_t response[512];
static size_t response_length, released;
static struct uart_double {
    bool installed;
    uint8_t bytes[1024];
    size_t used;
    uart_event_t events[16];
    unsigned queued, installs, deletes, flushes;
    uart_config_t config;
    int tx, rx;
} ports[3];
static bool is(const char *name) { return !strcmp(scenario, name); }
static void checksum(void)
{
    uint8_t sum = 0;
    for (unsigned i = 1; i < 8; ++i) sum = (uint8_t)(sum + response[i]);
    response[8] = (uint8_t)(0 - sum);
}
static void append(int port, const uint8_t *bytes, size_t count)
{
    assert(ports[port].used + count <= sizeof(ports[port].bytes));
    memcpy(ports[port].bytes + ports[port].used, bytes, count); ports[port].used += count;
}
static void refresh(void)
{
    if (sent_us < 0) {
        if (is("boundary_noise") && test_now_us >= 20000 && !fault_issued) {
            append(2, captured, 9); fault_issued = true;
        }
        return;
    }
    int64_t age = test_now_us - sent_us;
    size_t ready = age >= 20000 ? response_length : 0;
    if (is("fragmented")) ready = age >= 150000 ? 9 : age >= 70000 ? 5 : age >= 20000 ? 2 : 0;
    if (is("late")) ready = age >= 2000000 ? response_length : 0;
    if (is("trailing")) ready = age >= 30000 ? response_length : age >= 20000 ? 9 : 0;
    if (ready > released) { append(2, response + released, ready - released); released = ready; }
    if (age >= 20000 && !fault_issued) {
        fault_issued = true;
        if (is("uart_error")) ports[2].events[ports[2].queued++].type = UART_FRAME_ERR;
        if (is("parity")) ports[2].events[ports[2].queued++].type = UART_PARITY_ERR;
        if (is("break")) ports[2].events[ports[2].queued++].type = UART_BREAK;
        if (is("overflow")) ports[2].events[ports[2].queued++].type = UART_FIFO_OVF;
        if (is("buffer_full")) ports[2].events[ports[2].queued++].type = UART_BUFFER_FULL;
        if (is("event_full")) for (unsigned i = 0; i < 8; ++i) ports[2].events[ports[2].queued++].type = UART_DATA;
    }
}
bool uart_is_driver_installed(uart_port_t p) { assert(p == 1 || p == 2); return ports[p].installed; }
esp_err_t uart_driver_install(uart_port_t p, int rx, int tx, int count, QueueHandle_t *q, int flags)
{
    assert(!ports[p].installed && !tx && !flags);
    assert(p == 2 ? rx == 512 && count == 8 : rx == 1024 && count == 16);
    ++ports[p].installs;
    if (p == 2 && is("install")) return ESP_ERR_NO_MEM;
    ports[p].installed = true; *q = &ports[p]; return ESP_OK;
}
esp_err_t uart_driver_delete(uart_port_t p)
{
    assert(ports[p].installed); ++ports[p].deletes;
    if (p == 2 && is("cleanup")) return ESP_FAIL;
    ports[p].installed = false; return ESP_OK;
}
esp_err_t uart_param_config(uart_port_t p, const uart_config_t *c)
{
    assert(ports[p].installed && c->data_bits == 8 && !c->parity && c->stop_bits == 1 && !c->flow_ctrl);
    assert(c->baud_rate == (p == 2 ? 9600 : 256000)); ports[p].config = *c;
    return p == 2 && is("config") ? ESP_FAIL : ESP_OK;
}
esp_err_t uart_set_pin(uart_port_t p, int tx, int rx, int rts, int cts)
{
    assert(rts == -1 && cts == -1 && tx == (p == 2 ? 15 : 17) && rx == (p == 2 ? 16 : 18));
    ports[p].tx = tx; ports[p].rx = rx; return p == 2 && is("pins") ? ESP_FAIL : ESP_OK;
}
esp_err_t uart_flush_input(uart_port_t p)
{
    ++ports[p].flushes;
    if (p == 2 && is("flush")) return ESP_FAIL;
    ports[p].used = 0; return ESP_OK;
}
esp_err_t uart_get_buffered_data_len(uart_port_t p, size_t *n)
{
    if (p == 2) refresh();
    if (p == 2 && is("length_error") && tx_calls) return ESP_FAIL;
    *n = ports[p].used; return ESP_OK;
}
int uart_tx_chars(uart_port_t p, const char *bytes, uint32_t length)
{
    const uint8_t expected[] = {255,1,134,0,0,0,0,0,121};
    assert(p == 2 && ports[p].installed && test_now_us >= 20000);
    assert(length == 9 && !memcmp(bytes, expected, 9));
    ++tx_calls; sent_us = test_now_us; released = 0; fault_issued = false;
    return is("short_tx") ? 4 : is("failed_tx") ? -1 : 9;
}
esp_err_t uart_wait_tx_done(uart_port_t p, TickType_t ticks)
{
    assert(p == 2 && ticks == 10);
    if ((is("tx_timeout") && tx_calls) || is("preflight_tx")) { test_now_us += ticks * 10000LL; return ESP_ERR_TIMEOUT; }
    return ESP_OK;
}
int uart_read_bytes(uart_port_t p, void *out, uint32_t capacity, TickType_t wait)
{
    assert(!wait);
    if (p == 2) {
        assert(capacity == 10 && tx_calls);
        if (is("read_error")) return -1;
        if (is("zero_read")) return 0;
    }
    size_t count = ports[p].used < capacity ? ports[p].used : capacity;
    memcpy(out, ports[p].bytes, count); ports[p].used -= count;
    memmove(ports[p].bytes, ports[p].bytes + count, ports[p].used);
    if (p == 2 && is("post_read_error")) ports[p].events[ports[p].queued++].type = UART_PARITY_ERR;
    return (int)count;
}
unsigned uxQueueMessagesWaiting(QueueHandle_t q) { if (q == &ports[2]) refresh(); return ((struct uart_double *)q)->queued; }
BaseType_t xQueueReceive(QueueHandle_t q, void *out, TickType_t wait)
{
    struct uart_double *p = q;
    if (uxQueueMessagesWaiting(q)) {
        *(uart_event_t *)out = p->events[0]; memmove(p->events, p->events + 1, --p->queued * sizeof(*p->events)); return pdTRUE;
    }
    test_now_us += wait * 10000LL; return pdFALSE;
}
void vTaskDelay(TickType_t ticks) { assert(ticks > 0 && ticks <= 2); test_now_us += ticks * 10000LL; }

static void decoder_tests(void)
{
    mhz1911a_reading_t reading = {.co2_ppm = 1234};
    assert(mhz1911a_sensor_decode(NULL, 9, &reading) == ESP_ERR_INVALID_ARG);
    assert(mhz1911a_sensor_decode(captured, 9, NULL) == ESP_ERR_INVALID_ARG);
    for (size_t length = 0; length < 9; ++length) {
        assert(mhz1911a_sensor_decode(captured, length, &reading) == ESP_ERR_INVALID_RESPONSE);
        assert(reading.co2_ppm == 1234);
    }
    assert(mhz1911a_sensor_decode(captured, 10, &reading) == ESP_ERR_INVALID_RESPONSE);
    assert(mhz1911a_sensor_decode(captured, 9, &reading) == ESP_OK && reading.co2_ppm == 1770);
}

int main(int argc, char **argv)
{
    assert(argc == 2); scenario = argv[1]; decoder_tests();
    memcpy(response, captured, 9); response_length = 9;
    if (is("header")) response[0] = 0xaa;
    if (is("command")) response[1] = 0x87;
    if (is("opaque")) { response[4] = 0xab; response[5] = 0xcd; response[6] = 0xef; response[7] = 1; }
    if (is("range_low")) { response[2] = 1; response[3] = 0x8f; }
    if (is("range_high")) { response[2] = 0x13; response[3] = 0x89; }
    if (is("min")) { response[2] = 1; response[3] = 0x90; }
    if (is("max")) { response[2] = 0x13; response[3] = 0x88; }
    checksum();
    if (is("checksum") || is("recovery")) response[8] ^= 1;
    if (is("partial")) response_length = 4;
    if (is("timeout") || is("stale")) response_length = 0;
    if (is("stale") || is("stale_fresh")) append(2, captured, 9);
    if (is("echo")) { const uint8_t echo[] = {255,1,134,0,0,0,0,0,121}; memcpy(response, echo, 9); }
    if (is("extra") || is("trailing")) { response[9] = 0; response_length = 10; }
    if (is("duplicate")) { memcpy(response + 9, response, 9); response_length = 18; }
    if (is("noise")) { memmove(response + 1, response, 9); response[0] = 0; response_length = 10; }
    if (is("flood")) { memset(response, 0xaa, sizeof(response)); response_length = sizeof(response); }
    if (is("owned")) ports[2].installed = true;
    if (is("preflight_error")) ports[2].events[ports[2].queued++].type = UART_FRAME_ERR;

    // Exercise the actual UART1 driver alongside UART2 in every scenario.
    assert(ld2450_sensor_init() == ESP_OK);
    const uint8_t idle[30] = {0xaa,0xff,3,0,[28]=0x55,[29]=0xcc};
    append(1, idle, 30);
    mhz1911a_reading_t reading = {.co2_ppm = 1234};
    assert(mhz1911a_sensor_read(&reading) == ESP_ERR_INVALID_STATE);
    assert(mhz1911a_sensor_read(NULL) == ESP_ERR_INVALID_ARG);
    esp_err_t init = mhz1911a_sensor_init();
    bool init_failed = is("owned") || is("install") || is("config") || is("pins");
    assert((init != ESP_OK) == init_failed);
    if (!init_failed) {
        assert(mhz1911a_sensor_init() == ESP_OK && ports[2].installs == 1);
        int64_t start = test_now_us;
        esp_err_t result = mhz1911a_sensor_read(&reading);
        bool good = is("valid") || is("fragmented") || is("min") || is("max") || is("opaque") ||
                    is("cleanup") || is("stale_fresh") || is("repeated");
        assert((result == ESP_OK) == good);
        assert(reading.co2_ppm == (good ? is("min") ? 400 : is("max") ? 5000 : 1770 : 1234));
        bool no_tx = is("flush") || is("preflight_tx") || is("preflight_error") || is("boundary_noise");
        assert(tx_calls == (no_tx ? 0U : 1U));
        assert(test_now_us - start <= 2120000);
        mhz1911a_diagnostics_t d; mhz1911a_sensor_get_diagnostics(&d);
        assert(d.valid_frames == (unsigned)good);
        if (is("checksum") || is("recovery")) assert(result == ESP_ERR_INVALID_CRC && d.checksum_failures == 1);
        if (is("timeout") || is("stale") || is("partial") || is("late") || is("zero_read")) assert(result == ESP_ERR_TIMEOUT && d.timeouts == 1);
        if (is("partial")) assert(d.partial_responses == 1);
        if (is("stale") || is("stale_fresh")) assert(d.discarded_bytes >= 9);
        if (is("recovery") || is("repeated")) {
            scenario = "valid"; memcpy(response, captured, 9);
            test_now_us += 5000000;
            assert(mhz1911a_sensor_read(&reading) == ESP_OK && reading.co2_ppm == 1770 && tx_calls == 2);
            mhz1911a_sensor_get_diagnostics(&d);
            assert(d.requests == 2 && d.valid_frames == (good ? 2U : 1U));
            assert(d.checksum_failures == (good ? 0U : 1U));
        }
    } else assert(!tx_calls);
    assert(ports[1].used == 30 && !ports[1].flushes && !ports[1].deletes);
    ld2450_report_t radar; unsigned frames = 0;
    assert(ld2450_sensor_receive(&radar, &frames) == ESP_OK && frames == 1);
    assert(ports[1].config.baud_rate == 256000 && ports[1].tx == 17 && ports[1].rx == 18);
    unsigned before = ports[2].deletes;
    esp_err_t cleanup = mhz1911a_sensor_deinit();
    if (is("owned") || is("install") || is("config") || is("pins")) assert(ports[2].deletes == before);
    else assert(ports[2].deletes == before + 1);
    if (is("cleanup")) {
        assert(cleanup == ESP_FAIL && mhz1911a_sensor_init() == ESP_ERR_INVALID_STATE);
        scenario = "valid"; assert(mhz1911a_sensor_deinit() == ESP_OK);
    } else assert(cleanup == ESP_OK);
    assert(ld2450_sensor_deinit() == ESP_OK);
    printf("PASS: MH-Z1911A driver %s and real UART1 isolation\n", argv[1]);
    return 0;
}
