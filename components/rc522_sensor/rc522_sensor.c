#include "rc522_sensor.h"
#include "spi_bus_manager.h"

#include <stddef.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// NXP MFRC522 Rev. 3.9: SPI section 8.1.2 and register descriptions section 9.
enum {
    COMMAND = 0x01, COM_IRQ = 0x04, ERROR = 0x06, FIFO_DATA = 0x09,
    FIFO_LEVEL = 0x0a, CONTROL = 0x0c, BIT_FRAMING = 0x0d, COLL = 0x0e,
    MODE = 0x11, TX_MODE = 0x12, RX_MODE = 0x13, TX_CONTROL = 0x14,
    TX_ASK = 0x15, MOD_WIDTH = 0x24, T_MODE = 0x2a, T_PRESCALER = 0x2b,
    T_RELOAD_H = 0x2c, T_RELOAD_L = 0x2d, VERSION = 0x37,
};
enum { CMD_IDLE = 0x00, CMD_TRANSCEIVE = 0x0c, CMD_RESET = 0x0f };
typedef enum { RF_REPLY, RF_SILENCE, RF_COLLISION } rf_result_t;

static spi_bus_device_t s_device; // Static storage survives an SDK completion timeout.
static bool s_initialized, s_reset_configured;
static uint8_t s_version;
static int64_t s_deadline_us; // Only the acquisition owner accesses driver state.

#define TRY(call) do { esp_err_t result_ = (call); if (result_ != ESP_OK) return result_; } while (0)

static esp_err_t reg_transfer(uint8_t address, uint8_t value, uint8_t *read_value)
{
    if (esp_timer_get_time() >= s_deadline_us) return ESP_ERR_TIMEOUT;
    uint8_t tx[2] = {(uint8_t)((address << 1) | (read_value ? 0x80 : 0)), value};
    uint8_t rx[2];
    esp_err_t err = spi_bus_manager_transfer(&s_device, tx, rx, sizeof(tx));
    if (err == ESP_OK && read_value) *read_value = rx[1];
    return err;
}

static esp_err_t write_reg(uint8_t address, uint8_t value) { return reg_transfer(address, value, NULL); }
static esp_err_t read_reg(uint8_t address, uint8_t *value) { return reg_transfer(address, 0, value); }

// ISO14443A CRC: preset 0x6363, reflected polynomial 0x8408, low byte first.
static void crc_a(const uint8_t *data, size_t length, uint8_t out[2])
{
    uint16_t crc = 0x6363;
    for (size_t i = 0; i < length; ++i) {
        uint8_t byte = data[i] ^ (uint8_t)crc;
        byte ^= byte << 4;
        crc = (crc >> 8) ^ ((uint16_t)byte << 8) ^ ((uint16_t)byte << 3) ^ (byte >> 4);
    }
    out[0] = crc & 0xff;
    out[1] = crc >> 8;
}

static esp_err_t initialize_chip(void)
{
    const gpio_config_t reset = {.pin_bit_mask = UINT64_C(1) << 21, .mode = GPIO_MODE_OUTPUT};
    TRY(gpio_config(&reset));
    s_reset_configured = true;
    TRY(gpio_set_level(21, 0));
    vTaskDelay(pdMS_TO_TICKS(10));
    TRY(gpio_set_level(21, 1));
    vTaskDelay(pdMS_TO_TICKS(50));
    TRY(read_reg(VERSION, &s_version));
    if (s_version == 0 || s_version == 0xff) {
        ESP_LOGE("SENSOR", "rc522   | ERROR | VersionReg=0x%02X: invalid response; check power, SPI wiring and RST", s_version);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (s_version == 0x91 || s_version == 0x92) {
        ESP_LOGI("SENSOR", "rc522   | HEALTH | VersionReg=0x%02X (MFRC522 v%d.0)", s_version, s_version & 0x0f);
    } else {
        ESP_LOGW("SENSOR", "rc522   | ERROR | VersionReg=0x%02X (unrecognized revision; testing register communication)", s_version);
    }
    TRY(write_reg(COMMAND, CMD_RESET));
    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t value;
    do {
        TRY(read_reg(COMMAND, &value));
        if (!(value & 0x10)) break; // PowerDown clears after the oscillator restarts.
        vTaskDelay(1);
    } while (true); // Every register transfer checks the initialization deadline.
    TRY(read_reg(VERSION, &value));
    if (value != s_version) return ESP_ERR_INVALID_RESPONSE;

    const uint8_t settings[][2] = {
        {TX_MODE, 0x00}, {RX_MODE, 0x00}, {MOD_WIDTH, 0x26}, // 106 kbit/s, manual CRC.
        {T_MODE, 0x80}, {T_PRESCALER, 0xa9}, {T_RELOAD_H, 0x03}, {T_RELOAD_L, 0xe8},
        // Timer: 13.56 MHz / (2*169+1) = 40 kHz, 1000 ticks = 25 ms.
        {TX_ASK, 0x40}, {MODE, 0x3d}, {COLL, 0x00},
    };
    for (unsigned i = 0; i < sizeof(settings) / sizeof(settings[0]); ++i) {
        TRY(write_reg(settings[i][0], settings[i][1]));
        if (settings[i][0] == COLL) continue; // Collision position bits are read-only.
        TRY(read_reg(settings[i][0], &value));
        if (value != settings[i][1]) return ESP_ERR_INVALID_RESPONSE;
    }
    TRY(read_reg(TX_CONTROL, &value));
    TRY(write_reg(TX_CONTROL, value | 0x03));
    TRY(read_reg(TX_CONTROL, &value));
    if ((value & 0x03) != 0x03) return ESP_ERR_INVALID_RESPONSE;
    vTaskDelay(pdMS_TO_TICKS(10)); // ISO14443A field settling (>5 ms).
    return ESP_OK;
}

esp_err_t rc522_sensor_deinit(void)
{
    s_initialized = false;
    esp_err_t reset_err = s_reset_configured ? gpio_set_level(21, 0) : ESP_OK;
    esp_err_t err = spi_bus_manager_remove_device(&s_device);
    // If removal times out, retain the handle/descriptor. Never free memory
    // still owned by the SDK or tear down the shared bus to repair one reader.
    return err != ESP_OK ? err : reset_err;
}

esp_err_t rc522_sensor_init(void)
{
    if (s_initialized) return ESP_OK;
    if (s_device.handle) return ESP_ERR_INVALID_STATE;
    TRY(spi_bus_manager_add_device(&s_device, 14, 1000000, 0));
    s_deadline_us = esp_timer_get_time() + 300000;
    esp_err_t err = initialize_chip();
    if (err != ESP_OK) {
        esp_err_t cleanup = rc522_sensor_deinit();
        if (cleanup != ESP_OK) ESP_LOGW("SENSOR", "rc522   | ERROR | Cleanup incomplete: %s", esp_err_to_name(cleanup));
        return err;
    }
    s_initialized = true;
    ESP_LOGI("SENSOR", "rc522   | HEALTH | Ready | CS=14 RST=21 SPI=1 MHz | present a 13.56 MHz ISO14443A tag");
    return ESP_OK;
}

static esp_err_t exchange_inner(const uint8_t *tx, size_t tx_len, uint8_t tx_bits,
                                uint8_t align, uint8_t *rx, size_t capacity,
                                size_t *rx_len, uint8_t *last_bits, rf_result_t *rf)
{
    TRY(write_reg(COMMAND, CMD_IDLE));
    TRY(write_reg(COM_IRQ, 0x7f));
    TRY(write_reg(FIFO_LEVEL, 0x80));
    for (size_t i = 0; i < tx_len; ++i) TRY(write_reg(FIFO_DATA, tx[i]));
    uint8_t framing = (align << 4) | tx_bits;
    TRY(write_reg(BIT_FRAMING, framing));
    TRY(write_reg(COMMAND, CMD_TRANSCEIVE));
    TRY(write_reg(BIT_FRAMING, framing | 0x80));
    int64_t response_deadline = esp_timer_get_time() + 50000;
    uint8_t irq;
    while (true) {
        TRY(read_reg(COM_IRQ, &irq));
        if (irq & 0x32) break; // RxIRq, IdleIRq or ErrIRq.
        if (irq & 0x01) { *rf = RF_SILENCE; return ESP_OK; } // Hardware timer expired.
        if (esp_timer_get_time() >= response_deadline) return ESP_ERR_TIMEOUT;
        vTaskDelay(1); // Yield; never spin for a card or hold the SPI bus while waiting.
    }
    uint8_t error, count, control;
    TRY(read_reg(ERROR, &error));
    if (error & 0xd3) return ESP_ERR_INVALID_RESPONSE; // Write/temp/overflow/parity/protocol.
    if (error & 0x04) return ESP_ERR_INVALID_CRC;
    *rf = (error & 0x08) ? RF_COLLISION : RF_REPLY;
    TRY(read_reg(FIFO_LEVEL, &count));
    if (count > capacity) return ESP_ERR_INVALID_SIZE;
    TRY(read_reg(CONTROL, &control));
    *last_bits = control & 0x07;
    *rx_len = count;
    for (unsigned i = 0; i < count; ++i) TRY(read_reg(FIFO_DATA, &rx[i]));
    return ESP_OK;
}

static esp_err_t exchange(const uint8_t *tx, size_t tx_len, uint8_t tx_bits,
                          uint8_t align, uint8_t *rx, size_t capacity,
                          size_t *rx_len, uint8_t *last_bits, rf_result_t *rf)
{
    *rx_len = 0; *last_bits = 0;
    esp_err_t err = exchange_inner(tx, tx_len, tx_bits, align, rx, capacity, rx_len, last_bits, rf);
    esp_err_t stop = write_reg(COMMAND, CMD_IDLE);
    return err != ESP_OK ? err : stop;
}

static esp_err_t select_uid(rc522_uid_t *uid)
{
    for (unsigned level = 0; level < 3; ++level) {
        uint8_t select = 0x93 + level * 2;
        uint8_t part[5] = {0}; // Four UID/cascade bytes followed by BCC.
        unsigned known_bits = 0;
        bool complete = false;
        for (unsigned attempt = 0; attempt < 33; ++attempt) {
            unsigned whole = known_bits / 8, tail = known_bits % 8;
            uint8_t tx[7] = {select, (uint8_t)(((2 + whole) << 4) | tail)};
            memcpy(tx + 2, part, whole + (tail != 0));
            uint8_t rx[5], last_bits;
            size_t length;
            rf_result_t rf;
            TRY(exchange(tx, 2 + whole + (tail != 0), tail, tail,
                         rx, 5 - whole, &length, &last_bits, &rf));
            if (rf == RF_SILENCE) return ESP_ERR_INVALID_RESPONSE; // Card left during selection.
            if (length) {
                uint8_t mask = (1U << tail) - 1;
                part[whole] = (part[whole] & mask) | (rx[0] & ~mask);
                memcpy(part + whole + 1, rx + 1, length - 1);
            }
            if (rf == RF_COLLISION) {
                uint8_t collision;
                TRY(read_reg(COLL, &collision));
                unsigned position = collision & 0x1f;
                if (!position) position = 32;
                unsigned received_bits = length ? (length - 1) * 8 + (last_bits ? last_bits : 8) : 0;
                if ((collision & 0x20) || position <= known_bits ||
                    received_bits < tail + position - known_bits - 1) return ESP_ERR_INVALID_RESPONSE;
                // Deterministically choose the branch with a 1 at the collision.
                part[(position - 1) / 8] |= 1U << ((position - 1) % 8);
                known_bits = position;
                continue;
            }
            if (length != 5 - whole || last_bits != 0) return ESP_ERR_INVALID_RESPONSE;
            if ((part[0] ^ part[1] ^ part[2] ^ part[3]) != part[4]) return ESP_ERR_INVALID_CRC;
            complete = true;
            break;
        }
        if (!complete) return ESP_ERR_INVALID_RESPONSE;
        uint8_t frame[9] = {select, 0x70};
        memcpy(frame + 2, part, 5);
        crc_a(frame, 7, frame + 7);
        uint8_t sak[3], last_bits;
        size_t length;
        rf_result_t rf;
        TRY(exchange(frame, sizeof(frame), 0, 0, sak, sizeof(sak), &length, &last_bits, &rf));
        if (rf != RF_REPLY || length != 3 || last_bits) return ESP_ERR_INVALID_RESPONSE;
        uint8_t crc[2];
        crc_a(sak, 1, crc);
        if (memcmp(crc, sak + 1, 2)) return ESP_ERR_INVALID_CRC;
        bool cascade = (sak[0] & 0x04) != 0;
        // CL3 contains only UID data; its first byte may itself be 0x88.
        // A cascade tag has meaning only in CL1/CL2 (NXP AN10927, section 2).
        if ((level < 2 && cascade != (part[0] == 0x88)) ||
            (level == 2 && cascade)) return ESP_ERR_INVALID_RESPONSE;
        unsigned bytes = cascade ? 3 : 4;
        memcpy(uid->bytes + uid->length, part + (cascade ? 1 : 0), bytes);
        uid->length += bytes;
        if (!cascade) return ESP_OK;
    }
    return ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t halt_card(void)
{
    uint8_t frame[4] = {0x50, 0x00}, rx[3], last_bits;
    crc_a(frame, 2, frame + 2);
    size_t length;
    rf_result_t rf;
    TRY(exchange(frame, sizeof(frame), 0, 0, rx, sizeof(rx), &length, &last_bits, &rf));
    return rf == RF_SILENCE ? ESP_OK : ESP_ERR_INVALID_RESPONSE; // HLTA has no reply.
}

esp_err_t rc522_sensor_poll(rc522_uid_t *uid, bool *present)
{
    if (!uid || !present) return ESP_ERR_INVALID_ARG;
    *present = false;
    *uid = (rc522_uid_t){0};
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    s_deadline_us = esp_timer_get_time() + 300000;
    uint8_t value;
    TRY(read_reg(VERSION, &value));
    if (value != s_version || value == 0 || value == 0xff) return ESP_ERR_INVALID_RESPONSE;
    TRY(read_reg(MODE, &value));
    if (value != 0x3d) return ESP_ERR_INVALID_RESPONSE;
    TRY(read_reg(TX_CONTROL, &value));
    if ((value & 0x03) != 0x03) return ESP_ERR_INVALID_RESPONSE;
    uint8_t wake = 0x52, atqa[2], last_bits;
    size_t length;
    rf_result_t rf;
    TRY(exchange(&wake, 1, 7, 0, atqa, sizeof(atqa), &length, &last_bits, &rf));
    if (rf == RF_SILENCE) return ESP_OK;
    if (rf == RF_REPLY && (length != 2 || last_bits)) return ESP_ERR_INVALID_RESPONSE;
    // An ATQA collision also proves presence; the UID loop resolves a branch.
    rc522_uid_t selected = {0};
    esp_err_t err = select_uid(&selected);
    esp_err_t halt = halt_card(); // Also leave an already-selected card quiet after a bad SAK.
    if (err != ESP_OK) return err;
    if (halt != ESP_OK) return halt;
    *uid = selected;
    *present = true;
    return ESP_OK;
}
