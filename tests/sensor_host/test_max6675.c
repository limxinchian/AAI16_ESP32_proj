#include "platform.h"
#include "../rc522_host/platform.h"
#include <assert.h>
#include <string.h>
#include "../../components/max6675_sensor/max6675_sensor.c"

static uint16_t frame;
static esp_err_t add_error, transfer_error, remove_error;
static unsigned transfers, adds, removes, delays;

void vTaskDelay(TickType_t ticks) { delays++; assert(ticks >= 25); }
esp_err_t spi_bus_manager_add_device(spi_bus_device_t *device, int cs_gpio,
                                     int clock_hz, uint8_t mode)
{
    // MAX6675: CS on GPIO10, well under its 4.3 MHz limit, mode 0.
    assert(cs_gpio == 10 && clock_hz == 1000000 && mode == 0);
    adds++;
    if (add_error) return add_error;
    device->handle = (spi_device_handle_t)1;
    return ESP_OK;
}
esp_err_t spi_bus_manager_transfer(spi_bus_device_t *device, const uint8_t *tx,
                                   uint8_t *rx, size_t length)
{
    assert(device->handle && length == 2 && tx[0] == 0 && tx[1] == 0);
    transfers++;
    if (transfer_error) return transfer_error;
    rx[0] = frame >> 8;
    rx[1] = frame & 0xff;
    return ESP_OK;
}
esp_err_t spi_bus_manager_remove_device(spi_bus_device_t *device)
{
    removes++;
    if (remove_error) return remove_error;
    device->handle = NULL;
    return ESP_OK;
}

int main(void)
{
    max6675_data_t d = {0};
    float t;

    // Frame decoding: bits 14..3 are 0.25 degC counts, bit 2 = open, bit 1 = ID, bit 15 = dummy.
    assert(decode(100 << 3, &t) == ESP_OK && t == 25.0f);
    assert(decode(0x0000, &t) == ESP_OK && t == 0.0f);
    assert(decode(1 << 3, &t) == ESP_OK && t == 0.25f);
    assert(decode(0x7FF8, &t) == ESP_OK && t == 1023.75f);
    assert(decode((100 << 3) | 1, &t) == ESP_OK && t == 25.0f);   // Tri-state bit ignored.
    assert(decode(0x0004, &t) == ESP_ERR_NOT_FOUND);              // Thermocouple open.
    assert(decode(0xFFFF, &t) == ESP_ERR_INVALID_RESPONSE);       // Floating line is NOT "open".
    assert(decode(0x8000 | (100 << 3), &t) == ESP_ERR_INVALID_RESPONSE);
    assert(decode((100 << 3) | BIT_ID, &t) == ESP_ERR_INVALID_RESPONSE);

    // Not initialized, bad arguments.
    assert(max6675_sensor_read(&d) == ESP_ERR_INVALID_STATE);
    assert(max6675_sensor_read(NULL) == ESP_ERR_INVALID_ARG);

    // Add-device failure: nothing to clean up.
    add_error = ESP_ERR_NO_MEM;
    assert(max6675_sensor_init() == ESP_ERR_NO_MEM && !s_device.handle && !s_initialized);
    add_error = 0;

    // Wiring fault at init (all ones): rejected, handle released, no delay skipped.
    frame = 0xFFFF; delays = 0;
    assert(max6675_sensor_init() == ESP_ERR_INVALID_RESPONSE && !s_device.handle && delays == 1);
    // SPI transfer failure at init is propagated and cleaned up.
    frame = 100 << 3; transfer_error = ESP_ERR_TIMEOUT;
    assert(max6675_sensor_init() == ESP_ERR_TIMEOUT && !s_device.handle && !s_initialized);
    transfer_error = 0;

    // Open thermocouple does not fail init (the chip answered); reads report it.
    frame = 0x0004;
    assert(max6675_sensor_init() == ESP_OK && s_initialized);
    assert(max6675_sensor_read(&d) == ESP_ERR_NOT_FOUND);
    assert(max6675_sensor_deinit() == ESP_OK && !s_initialized && !s_device.handle);

    // Healthy sensor: init is idempotent, reads decode, transport errors pass through.
    frame = 91 << 3; adds = 0;   // 22.75 degC
    assert(max6675_sensor_init() == ESP_OK && max6675_sensor_init() == ESP_OK && adds == 1);
    assert(max6675_sensor_read(&d) == ESP_OK && d.temperature_c == 22.75f);
    frame = 0xFFFF;
    assert(max6675_sensor_read(&d) == ESP_ERR_INVALID_RESPONSE);
    frame = 100 << 3; transfer_error = ESP_ERR_TIMEOUT;
    assert(max6675_sensor_read(&d) == ESP_ERR_TIMEOUT);
    transfer_error = 0;
    assert(max6675_sensor_read(&d) == ESP_OK && d.temperature_c == 25.0f);

    // Removal failure keeps the descriptor and blocks re-init until it succeeds.
    remove_error = ESP_ERR_INVALID_STATE;
    assert(max6675_sensor_deinit() == ESP_ERR_INVALID_STATE && s_device.handle);
    assert(max6675_sensor_init() == ESP_ERR_INVALID_STATE);
    assert(max6675_sensor_read(&d) == ESP_ERR_INVALID_STATE);
    remove_error = 0;
    assert(max6675_sensor_deinit() == ESP_OK && !s_device.handle);
    assert(max6675_sensor_deinit() == ESP_OK);

    puts("PASS: MAX6675 frame decode, open-thermocouple vs bad-frame, init cleanup, idempotence, removal failure");
    return 0;
}
