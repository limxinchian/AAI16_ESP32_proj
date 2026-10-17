#include "platform.h"
#include <assert.h>
#include <string.h>
#include "../../components/rc522_sensor/rc522_sensor.c"

typedef struct {
    uint8_t tx[9], tx_len, tx_bits, align;
    uint8_t rx[6], rx_len, last_bits, error, collision, irq;
} frame_t;
static frame_t frames[40];
static unsigned frame_count, next_frame, transfers, fail_transfer, removed, adds;
static uint8_t regs[64], fifo[64];
static unsigned fifo_len, fifo_index;
static bool reset_stuck, bad_readback, fault_happened;
static esp_err_t add_error, gpio_error;
static unsigned transfer_delay_us;
static int token;

void vTaskDelay(TickType_t ticks) { assert(ticks); test_now_us += ticks * 10000LL; }
esp_err_t gpio_config(const gpio_config_t *config)
{
    assert(config->pin_bit_mask == (UINT64_C(1) << 21) && config->mode == GPIO_MODE_OUTPUT);
    return gpio_error;
}
esp_err_t gpio_set_level(int gpio, int level) { assert(gpio == 21 && (level == 0 || level == 1)); return ESP_OK; }
esp_err_t spi_bus_manager_add_device(spi_bus_device_t *device, int cs, int hz, uint8_t mode)
{
    assert(!device->handle && cs == 14 && hz == 1000000 && mode == 0);
    adds++;
    if (add_error) return add_error;
    device->handle = &token; return ESP_OK;
}
esp_err_t spi_bus_manager_remove_device(spi_bus_device_t *device)
{
    if (device->handle) removed++;
    device->handle = NULL; return ESP_OK;
}

static void respond(void)
{
    if (fault_happened) {
        // After an injected selection error the driver may still halt a card.
        const uint8_t stop[] = {0x50, 0x00, 0x57, 0xcd};
        assert(fifo_len == 4 && !memcmp(fifo, stop, 4));
        regs[COM_IRQ] = 1; fifo_len = fifo_index = 0;
        return;
    }
    assert(next_frame < frame_count);
    const frame_t *frame = &frames[next_frame++];
    if (fifo_len != frame->tx_len || memcmp(fifo, frame->tx, fifo_len)) {
        fprintf(stderr, "RF frame %u mismatch (transfer=%u injected=%u): got", next_frame, transfers, fail_transfer);
        for (unsigned i = 0; i < fifo_len; ++i) fprintf(stderr, " %02X", fifo[i]);
        fprintf(stderr, " expected");
        for (unsigned i = 0; i < frame->tx_len; ++i) fprintf(stderr, " %02X", frame->tx[i]);
        fprintf(stderr, "\n");
    }
    assert(fifo_len == frame->tx_len && !memcmp(fifo, frame->tx, fifo_len));
    assert((regs[BIT_FRAMING] & 7) == frame->tx_bits);
    assert(((regs[BIT_FRAMING] >> 4) & 7) == frame->align);
    regs[COM_IRQ] = frame->irq;
    regs[ERROR] = frame->error;
    regs[COLL] = frame->collision;
    regs[CONTROL] = frame->last_bits;
    memcpy(fifo, frame->rx, frame->rx_len);
    fifo_len = frame->rx_len; fifo_index = 0;
}

esp_err_t spi_bus_manager_transfer(spi_bus_device_t *device, const uint8_t *tx, uint8_t *rx, size_t length)
{
    assert(device->handle == &token && length == 2 && !(tx[0] & 1));
    test_now_us += transfer_delay_us;
    if (++transfers == fail_transfer) { fault_happened = true; return ESP_ERR_TIMEOUT; }
    unsigned reg = (tx[0] >> 1) & 0x3f;
    rx[0] = 0xa5; rx[1] = 0x5a; // The address phase must not be mistaken for data.
    if (tx[0] & 0x80) {
        assert(tx[1] == 0);
        if (reg == VERSION) assert(tx[0] == 0xee);
        if (reg == FIFO_DATA) { assert(fifo_index < fifo_len); rx[1] = fifo[fifo_index++]; }
        else if (reg == FIFO_LEVEL) rx[1] = fifo_len - fifo_index;
        else rx[1] = regs[reg];
        if (reg == MODE && bad_readback) rx[1] ^= 1;
    } else {
        if (reg == COMMAND && tx[1] == CMD_RESET) regs[COMMAND] = reset_stuck ? 0x10 : 0;
        else if (reg == COM_IRQ) { assert(tx[1] == 0x7f); regs[COM_IRQ] = 0; }
        else if (reg == FIFO_LEVEL) { assert(tx[1] == 0x80); fifo_len = fifo_index = 0; }
        else if (reg == FIFO_DATA) { assert(fifo_len < sizeof(fifo)); fifo[fifo_len++] = tx[1]; }
        else regs[reg] = tx[1];
        if (reg == BIT_FRAMING && (tx[1] & 0x80)) {
            assert(regs[COMMAND] == CMD_TRANSCEIVE); respond();
        }
    }
    return ESP_OK;
}

static void reset_test(uint8_t version)
{
    s_initialized = s_reset_configured = false;
    s_device = (spi_bus_device_t){0};
    memset(regs, 0, sizeof(regs));
    regs[VERSION] = version;
    regs[TX_CONTROL] = 0x80;
    memset(frames, 0, sizeof(frames));
    frame_count = next_frame = transfers = fail_transfer = removed = adds = 0;
    fifo_len = fifo_index = 0;
    reset_stuck = bad_readback = fault_happened = false;
    add_error = gpio_error = ESP_OK;
    transfer_delay_us = 0;
    test_now_us = 0;
}

// Independent bitwise CRC reference for synthetic PICC responses.
static void card_crc(uint8_t *data, size_t length)
{
    uint16_t crc = 0x6363;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1) ? 0x8408 : 0);
    }
    data[length] = crc & 0xff; data[length + 1] = crc >> 8;
}

static frame_t *add_frame(const uint8_t *tx, unsigned tx_len, const uint8_t *rx, unsigned rx_len)
{
    assert(frame_count < 40 && tx_len <= 9 && rx_len <= 6);
    frame_t *frame = &frames[frame_count++];
    *frame = (frame_t){.tx_len = tx_len, .rx_len = rx_len, .irq = 0x30};
    memcpy(frame->tx, tx, tx_len);
    if (rx_len) memcpy(frame->rx, rx, rx_len);
    return frame;
}

static void wake(bool present)
{
    const uint8_t tx[] = {0x52}, atqa[] = {0x04, 0x00};
    frame_t *frame = add_frame(tx, 1, atqa, present ? 2 : 0);
    frame->tx_bits = 7;
    if (!present) frame->irq = 0x01;
}

static void halt(void)
{
    const uint8_t tx[] = {0x50, 0x00, 0x57, 0xcd}; // CRC-A low byte first.
    add_frame(tx, sizeof(tx), NULL, 0)->irq = 0x01;
}

static void select_frame(uint8_t select, const uint8_t part[5], bool cascade)
{
    uint8_t tx[9] = {select, 0x70}, sak[3] = {cascade ? 0x04 : 0x08};
    memcpy(tx + 2, part, 5); card_crc(tx, 7); card_crc(sak, 1);
    add_frame(tx, 9, sak, 3);
}

static void card(const uint8_t *uid, unsigned length)
{
    wake(true);
    unsigned levels = (length - 1) / 3, offset = 0;
    for (unsigned level = 0; level < levels; ++level) {
        bool cascade = level + 1 < levels;
        uint8_t select = 0x93 + 2 * level, part[5] = {0}, tx[2] = {select, 0x20};
        if (cascade) part[0] = 0x88;
        unsigned bytes = cascade ? 3 : 4;
        memcpy(part + (cascade ? 1 : 0), uid + offset, bytes); offset += bytes;
        part[4] = part[0] ^ part[1] ^ part[2] ^ part[3];
        add_frame(tx, 2, part, 5);
        select_frame(select, part, cascade);
    }
    halt();
}

static void expect_poll(esp_err_t expected, bool expected_present, const uint8_t *bytes, unsigned length)
{
    bool present = true;
    rc522_uid_t uid = {.length = 99};
    esp_err_t result = rc522_sensor_poll(&uid, &present);
    assert(result == expected && present == expected_present);
    assert(uid.length == (expected_present ? length : 0));
    if (expected_present) assert(!memcmp(uid.bytes, bytes, length));
}

int main(void)
{
    const uint8_t versions[] = {0x91, 0x92, 0x90, 0x12, 0, 0xff};
    for (unsigned i = 0; i < sizeof(versions); ++i) {
        reset_test(versions[i]);
        bool valid = versions[i] != 0 && versions[i] != 0xff;
        assert(rc522_sensor_init() == (valid ? ESP_OK : ESP_ERR_INVALID_RESPONSE));
        assert(s_initialized == valid && removed == (valid ? 0U : 1U));
        if (valid) {
            assert(regs[T_MODE] == 0x80 && regs[T_PRESCALER] == 0xa9);
            assert(regs[T_RELOAD_H] == 3 && regs[T_RELOAD_L] == 0xe8 && regs[TX_CONTROL] == 0x83);
            assert(test_now_us == 120000 && rc522_sensor_init() == ESP_OK && adds == 1);
        }
        assert(rc522_sensor_deinit() == ESP_OK && !s_device.handle);
    }
    reset_test(0x92); bad_readback = true;
    assert(rc522_sensor_init() == ESP_ERR_INVALID_RESPONSE && removed == 1);
    reset_test(0x92); reset_stuck = true;
    assert(rc522_sensor_init() == ESP_ERR_TIMEOUT && test_now_us <= 310000 && removed == 1);
    reset_test(0x92); add_error = ESP_ERR_NO_MEM;
    assert(rc522_sensor_init() == add_error && !removed);
    reset_test(0x92); gpio_error = ESP_FAIL;
    assert(rc522_sensor_init() == gpio_error && removed == 1);
    reset_test(0x92); assert(rc522_sensor_init() == ESP_OK);
    unsigned init_transfers = transfers;
    for (unsigned fault = 1; fault <= init_transfers; ++fault) {
        reset_test(0x92); fail_transfer = fault;
        assert(rc522_sensor_init() == ESP_ERR_TIMEOUT && !s_initialized && removed == 1);
    }
    const uint8_t uid_bytes[] = {0x04, 0xa1, 0xb2, 0xc3, 0xd4, 0xe5, 0xf6, 0x17, 0x28, 0x39};
    for (unsigned length = 4; length <= 10; length += 3) {
        reset_test(0x92); assert(rc522_sensor_init() == ESP_OK);
        card(uid_bytes, length);
        expect_poll(ESP_OK, true, uid_bytes, length);
        assert(next_frame == frame_count && regs[COMMAND] == CMD_IDLE);
    }
    reset_test(0x92); assert(rc522_sensor_init() == ESP_OK);
    const uint8_t uid_cl3_88[] = {0x04, 1, 2, 3, 4, 5, 0x88, 7, 8, 9};
    card(uid_cl3_88, 10); // 0x88 is ordinary UID data at CL3, not a cascade tag.
    expect_poll(ESP_OK, true, uid_cl3_88, 10);
    reset_test(0x92); assert(rc522_sensor_init() == ESP_OK);
    wake(false); expect_poll(ESP_OK, false, NULL, 0); // Hardware RF timer, normal idle.
    unsigned poll_start = transfers;
    card(uid_bytes, 4); expect_poll(ESP_OK, true, uid_bytes, 4);
    unsigned poll_transfers = transfers - poll_start;
    // Every SPI failure during acquisition must return an error, never "idle" or a UID.
    for (unsigned fault = 1; fault <= poll_transfers; ++fault) {
        reset_test(0x92); assert(rc522_sensor_init() == ESP_OK);
        card(uid_bytes, 4); fail_transfer = transfers + fault;
        expect_poll(ESP_ERR_TIMEOUT, false, NULL, 0);
    }
    reset_test(0x92); assert(rc522_sensor_init() == ESP_OK);
    wake(false); frames[0].irq = 0; // Broken IRQ/timer: software timeout is a FAILURE.
    int64_t before = test_now_us;
    expect_poll(ESP_ERR_TIMEOUT, false, NULL, 0);
    assert(test_now_us - before == 50000);
    reset_test(0x92); assert(rc522_sensor_init() == ESP_OK); regs[VERSION] = 0xff;
    expect_poll(ESP_ERR_INVALID_RESPONSE, false, NULL, 0);
    reset_test(0x92); assert(rc522_sensor_init() == ESP_OK); regs[MODE] = 0;
    expect_poll(ESP_ERR_INVALID_RESPONSE, false, NULL, 0);
    reset_test(0x92); assert(rc522_sensor_init() == ESP_OK); regs[TX_CONTROL] &= ~3;
    expect_poll(ESP_ERR_INVALID_RESPONSE, false, NULL, 0);
    for (unsigned bad = 0; bad < 6; ++bad) {
        reset_test(0x92); assert(rc522_sensor_init() == ESP_OK); card(uid_bytes, 4);
        esp_err_t expected = ESP_ERR_INVALID_RESPONSE;
        if (bad == 0) { frames[1].rx[4] ^= 1; expected = ESP_ERR_INVALID_CRC; }
        if (bad == 1) frames[1].rx_len = 4;
        if (bad == 2) { frames[1].rx_len = 6; expected = ESP_ERR_INVALID_SIZE; }
        if (bad <= 2) { frames[2] = frames[3]; frame_count = 3; }
        if (bad == 3) { frames[2].rx[2] ^= 1; expected = ESP_ERR_INVALID_CRC; }
        if (bad == 4) { frames[2].rx[0] = 0x04; card_crc(frames[2].rx, 1); } // Invalid CT/SAK pair.
        if (bad == 5) { frames[0].rx_len = 1; frame_count = 1; }
        expect_poll(expected, false, NULL, 0);
        assert(next_frame == frame_count);
    }
    // Collisions at byte boundaries and within a byte, including CollPos=0 -> bit 32.
    const unsigned positions[] = {1, 8, 9, 32};
    const uint8_t selected_uid[] = {0x85, 0xa3, 0x42, 0x80};
    for (unsigned p = 0; p < 4; ++p) {
        reset_test(0x92); assert(rc522_sensor_init() == ESP_OK);
        unsigned position = positions[p], whole = position / 8, tail = position % 8;
        uint8_t part[5]; memcpy(part, selected_uid, 4);
        part[4] = part[0] ^ part[1] ^ part[2] ^ part[3];
        wake(true);
        const uint8_t initial[] = {0x93, 0x20};
        frame_t *collision = add_frame(initial, 2, part, (position + 7) / 8);
        collision->error = 8; collision->collision = position & 31;
        collision->last_bits = tail;
        collision->rx[(position - 1) / 8] &= (1U << ((position - 1) % 8)) - 1;
        uint8_t tx[7] = {0x93, (uint8_t)(((2 + whole) << 4) | tail)};
        memcpy(tx + 2, part, whole + (tail != 0));
        if (tail) tx[2 + whole] &= (1U << tail) - 1;
        frame_t *remaining = add_frame(tx, 2 + whole + (tail != 0), part + whole, 5 - whole);
        remaining->tx_bits = remaining->align = tail;
        if (tail) remaining->rx[0] &= ~((1U << tail) - 1); // Low bits must be preserved from TX.
        select_frame(0x93, part, false); halt();
        expect_poll(ESP_OK, true, selected_uid, 4);
        assert(next_frame == frame_count);
    }
    reset_test(0x92); assert(rc522_sensor_init() == ESP_OK); card(uid_bytes, 4);
    frames[1].error = 8; frames[1].collision = 0x20;
    frames[2] = frames[3]; frame_count = 3;
    expect_poll(ESP_ERR_INVALID_RESPONSE, false, NULL, 0);
    reset_test(0x92); assert(rc522_sensor_init() == ESP_OK); card(uid_bytes, 10);
    transfer_delay_us = 5000;
    before = test_now_us;
    expect_poll(ESP_ERR_TIMEOUT, false, NULL, 0);
    assert(test_now_us - before <= 305000);
    puts("PASS: SPI register framing, reset/version/readback, UID4/7/10, CRC/BCC, collision alignment, idle and deadlines");
    return 0;
}
