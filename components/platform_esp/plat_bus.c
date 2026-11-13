/* Bus backend over ESP-IDF drivers. One cached configuration per bus type, matching the "one active
 * sensor per bus" rule: when a different profile's parameters arrive, the previous driver instance is
 * torn down and a new one created. NOT hardware-verified. */
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "platform_esp.h"

static const char *TAG = "plat_bus";

static uint32_t to_ms(uint32_t us) { return us ? (us + 999u) / 1000u : 1u; }

/* ------------------------------------------------------------------ I2C */
static struct {
    bool valid;
    aai16_I2cParams p;
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
} s_i2c;

static void i2c_teardown(void)
{
    if (s_i2c.valid) {
        i2c_master_bus_rm_device(s_i2c.dev);
        i2c_del_master_bus(s_i2c.bus);
        s_i2c.valid = false;
    }
}

static bool i2c_ensure(const aai16_I2cParams *p)
{
    if (s_i2c.valid && memcmp(&s_i2c.p, p, sizeof(*p)) == 0) return true;
    i2c_teardown();
    const i2c_master_bus_config_t bc = {
        .i2c_port = -1, .sda_io_num = (gpio_num_t)p->sda, .scl_io_num = (gpio_num_t)p->scl,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bc, &s_i2c.bus) != ESP_OK) { ESP_LOGE(TAG, "i2c bus init failed"); return false; }
    const i2c_device_config_t dc = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = (uint16_t)p->addr,
                                    .scl_speed_hz = p->clock_hz};
    if (i2c_master_bus_add_device(s_i2c.bus, &dc, &s_i2c.dev) != ESP_OK) {
        ESP_LOGE(TAG, "i2c add device 0x%02x failed", (unsigned)p->addr);
        i2c_del_master_bus(s_i2c.bus);
        return false;
    }
    s_i2c.p = *p;
    s_i2c.valid = true;
    return true;
}

static bus_status_t map_err(esp_err_t e) { return e == ESP_OK ? BUS_OK : (e == ESP_ERR_TIMEOUT ? BUS_TIMEOUT : BUS_ERROR); }

static bus_status_t b_i2c(void *c, const aai16_I2cParams *p, const uint8_t *tx, size_t txn, uint8_t *rx, size_t rxn, uint32_t to)
{
    (void)c;
    if (!i2c_ensure(p)) return BUS_ERROR;
    const int ms = (int)to_ms(to);
    if (txn && rxn) return map_err(i2c_master_transmit_receive(s_i2c.dev, tx, txn, rx, rxn, ms));
    if (txn) return map_err(i2c_master_transmit(s_i2c.dev, tx, txn, ms));
    if (rxn) return map_err(i2c_master_receive(s_i2c.dev, rx, rxn, ms));
    return BUS_OK;
}

/* ----------------------------------------------------------------- UART */
static struct { bool valid; aai16_UartParams p; } s_uart[UART_NUM_MAX];

static bool uart_ensure(const aai16_UartParams *p)
{
    if (p->port >= UART_NUM_MAX) return false;
    if (s_uart[p->port].valid && memcmp(&s_uart[p->port].p, p, sizeof(*p)) == 0) return true;
    if (s_uart[p->port].valid) uart_driver_delete((uart_port_t)p->port);
    s_uart[p->port].valid = false;
    const uart_config_t cfg = {.baud_rate = (int)p->baud, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
    if (uart_driver_install((uart_port_t)p->port, 1024, 0, 0, NULL, 0) != ESP_OK ||
        uart_param_config((uart_port_t)p->port, &cfg) != ESP_OK ||
        uart_set_pin((uart_port_t)p->port, (int)p->tx, (int)p->rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "uart%u init failed", (unsigned)p->port);
        return false;
    }
    s_uart[p->port].p = *p;
    s_uart[p->port].valid = true;
    return true;
}

static bus_status_t b_uart(void *c, const aai16_UartParams *p, const uint8_t *tx, size_t txn, uint8_t *rx, size_t rxn, uint32_t to)
{
    (void)c;
    if (!uart_ensure(p)) return BUS_ERROR;
    const uart_port_t port = (uart_port_t)p->port;
    if (txn) {
        uart_flush_input(port); /* command/reply: drop stale bytes first */
        if (uart_write_bytes(port, tx, txn) < 0) return BUS_ERROR;
        if (uart_wait_tx_done(port, pdMS_TO_TICKS(100)) != ESP_OK) return BUS_ERROR;
    }
    if (rxn) {
        const int got = uart_read_bytes(port, rx, rxn, pdMS_TO_TICKS(to_ms(to)));
        if (got < 0) return BUS_ERROR;
        if ((size_t)got < rxn) return BUS_TIMEOUT;
    }
    return BUS_OK;
}

static bus_status_t b_uart_sync(void *c, const aai16_UartParams *p, const uint8_t *pat, size_t patn, const uint8_t *tail,
                                size_t tailn, uint8_t *rx, size_t frame_len, uint32_t to)
{
    (void)c;
    if (!uart_ensure(p) || patn == 0 || patn > frame_len) return BUS_ERROR;
    const uart_port_t port = (uart_port_t)p->port;
    const int64_t deadline = esp_timer_get_time() + (int64_t)to;
    uint8_t win[16] = {0};
    size_t have = 0;
    for (;;) { /* slide a byte window until the header pattern is seen */
        const int64_t left = deadline - esp_timer_get_time();
        if (left <= 0) return BUS_TIMEOUT;
        uint8_t b;
        const int n = uart_read_bytes(port, &b, 1, pdMS_TO_TICKS(to_ms((uint32_t)left)));
        if (n < 0) return BUS_ERROR;
        if (n == 0) return BUS_TIMEOUT;
        if (have < patn) win[have++] = b;
        else { memmove(win, win + 1, patn - 1); win[patn - 1] = b; }
        if (have == patn && memcmp(win, pat, patn) == 0) break;
    }
    memcpy(rx, pat, patn);
    size_t got = patn;
    while (got < frame_len) {
        const int64_t left = deadline - esp_timer_get_time();
        if (left <= 0) return BUS_TIMEOUT;
        const int n = uart_read_bytes(port, rx + got, frame_len - got, pdMS_TO_TICKS(to_ms((uint32_t)left)));
        if (n < 0) return BUS_ERROR;
        if (n == 0) return BUS_TIMEOUT;
        got += (size_t)n;
    }
    if (tailn && (tailn > frame_len || memcmp(rx + frame_len - tailn, tail, tailn) != 0)) return BUS_BAD_FRAME;
    return BUS_OK;
}

/* ------------------------------------------------------------------ SPI */
static struct { bool valid; aai16_SpiParams p; spi_device_handle_t dev; } s_spi;

static void spi_teardown(void)
{
    if (s_spi.valid) {
        spi_bus_remove_device(s_spi.dev);
        spi_bus_free(SPI2_HOST);
        s_spi.valid = false;
    }
}

static bool spi_ensure(const aai16_SpiParams *p)
{
    if (s_spi.valid && memcmp(&s_spi.p, p, sizeof(*p)) == 0) return true;
    spi_teardown();
    const spi_bus_config_t bc = {.mosi_io_num = (int)p->mosi, .miso_io_num = (int)p->miso, .sclk_io_num = (int)p->sclk,
                                 .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 64};
    if (spi_bus_initialize(SPI2_HOST, &bc, SPI_DMA_CH_AUTO) != ESP_OK) { ESP_LOGE(TAG, "spi bus init failed"); return false; }
    const spi_device_interface_config_t dc = {.clock_speed_hz = (int)p->clock_hz, .mode = (uint8_t)p->mode,
                                              .spics_io_num = (int)p->cs, .queue_size = 1};
    if (spi_bus_add_device(SPI2_HOST, &dc, &s_spi.dev) != ESP_OK) { ESP_LOGE(TAG, "spi add device failed"); spi_bus_free(SPI2_HOST); return false; }
    s_spi.p = *p;
    s_spi.valid = true;
    return true;
}

/* Static (internal, DMA-capable) buffers: if a transaction ever completes late, it must not write into a
 * dead stack frame. The queued API with a timeout is used instead of polling, so a bus that never answers
 * cannot hang the node task. */
static uint8_t s_spi_tx[64], s_spi_rx[64];
static spi_transaction_t s_spi_tr;

static bus_status_t b_spi(void *c, const aai16_SpiParams *p, const uint8_t *tx, size_t txn, uint8_t *rx, size_t rxn, uint32_t to)
{
    (void)c;
    if (!spi_ensure(p)) return BUS_ERROR;
    const size_t n = txn > rxn ? txn : rxn;
    if (n == 0 || n > sizeof(s_spi_tx)) return BUS_ERROR;
    memset(s_spi_tx, 0, sizeof(s_spi_tx));
    memset(s_spi_rx, 0, sizeof(s_spi_rx));
    if (txn) memcpy(s_spi_tx, tx, txn);
    memset(&s_spi_tr, 0, sizeof(s_spi_tr));
    s_spi_tr.length = n * 8;
    s_spi_tr.tx_buffer = s_spi_tx;
    s_spi_tr.rx_buffer = s_spi_rx;
    const TickType_t wait = pdMS_TO_TICKS(to_ms(to));
    if (spi_device_queue_trans(s_spi.dev, &s_spi_tr, wait) != ESP_OK) return BUS_ERROR;
    spi_transaction_t *done = NULL;
    const esp_err_t e = spi_device_get_trans_result(s_spi.dev, &done, wait);
    if (e == ESP_ERR_TIMEOUT) {
        spi_teardown(); /* drop the wedged device; the next call re-creates it */
        return BUS_TIMEOUT;
    }
    if (e != ESP_OK) return BUS_ERROR;
    if (rxn) memcpy(rx, s_spi_rx, rxn);
    return BUS_OK;
}

/* ----------------------------------------------------------------- GPIO */
static void pin_out(uint32_t pin)
{
    gpio_reset_pin((gpio_num_t)pin);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
}

static void pin_in(uint32_t pin, bool pullup)
{
    gpio_reset_pin((gpio_num_t)pin);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)pin, pullup ? GPIO_PULLUP_ONLY : GPIO_FLOATING);
}

static void b_delay(void *c, uint32_t us)
{
    (void)c;
    if (us >= 2000) { vTaskDelay(pdMS_TO_TICKS(us / 1000)); us %= 1000; }
    if (us) esp_rom_delay_us(us);
}

static bus_status_t b_gpio_set(void *c, uint32_t pin, bool level)
{
    (void)c;
    pin_out(pin);
    gpio_set_level((gpio_num_t)pin, level);
    return BUS_OK;
}

static bus_status_t b_gpio_pulse(void *c, uint32_t pin, bool level, uint32_t width_us)
{
    pin_out(pin);
    gpio_set_level((gpio_num_t)pin, level);
    b_delay(c, width_us);
    gpio_set_level((gpio_num_t)pin, !level); /* release to the idle level (DHT22/HC-SR04 idle high/low respectively via profile) */
    return BUS_OK;
}

/* Poll until the pin equals `level`; returns elapsed us or -1 on timeout. */
static int64_t wait_level(uint32_t pin, int level, uint32_t timeout_us)
{
    const int64_t t0 = esp_timer_get_time();
    while (gpio_get_level((gpio_num_t)pin) != level) {
        if (esp_timer_get_time() - t0 > (int64_t)timeout_us) return -1;
    }
    return esp_timer_get_time() - t0;
}

static bus_status_t b_gpio_pulse_width(void *c, uint32_t pin, bool level, uint32_t timeout_us, uint32_t *w)
{
    (void)c;
    pin_in(pin, false);
    *w = 0;
    if (wait_level(pin, level, timeout_us) < 0) return BUS_TIMEOUT;            /* wait for the pulse to start */
    const int64_t t0 = esp_timer_get_time();
    if (wait_level(pin, !level, timeout_us) < 0) return BUS_TIMEOUT;           /* ...and to end */
    *w = (uint32_t)(esp_timer_get_time() - t0);
    return BUS_OK;
}

static portMUX_TYPE s_timing_mux = portMUX_INITIALIZER_UNLOCKED;

static bus_status_t b_gpio_bit_frame(void *c, uint32_t pin, uint32_t nbytes, uint32_t timeout_us, uint32_t threshold_us, uint8_t *out)
{
    (void)c;
    memset(out, 0, nbytes);
    pin_in(pin, true);
    bus_status_t st = BUS_OK;
    portENTER_CRITICAL(&s_timing_mux); /* ~5 ms of bit timing: interrupts off on this core */
    do {
        if (wait_level(pin, 0, timeout_us) < 0 || wait_level(pin, 1, timeout_us) < 0 || wait_level(pin, 0, timeout_us) < 0) {
            st = BUS_TIMEOUT; /* no response preamble */
            break;
        }
        for (uint32_t i = 0; i < nbytes * 8 && st == BUS_OK; i++) {
            if (wait_level(pin, 1, timeout_us) < 0) { st = BUS_TIMEOUT; break; }
            const int64_t t0 = esp_timer_get_time();
            if (wait_level(pin, 0, timeout_us) < 0) { st = BUS_TIMEOUT; break; }
            if ((esp_timer_get_time() - t0) > (int64_t)threshold_us) out[i / 8] |= (uint8_t)(0x80u >> (i % 8));
        }
    } while (0);
    portEXIT_CRITICAL(&s_timing_mux);
    return st;
}

static uint64_t b_now(void *c) { (void)c; return (uint64_t)esp_timer_get_time(); }

bus_backend_t plat_bus_backend(void)
{
    return (bus_backend_t){b_i2c, b_uart, b_uart_sync, b_spi, b_gpio_set, b_gpio_pulse, b_gpio_pulse_width,
                           b_gpio_bit_frame, b_delay, b_now, NULL};
}
