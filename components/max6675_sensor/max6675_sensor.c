#include "max6675_sensor.h"
#include "spi_bus_manager.h"

#include <stdbool.h>
#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Maxim MAX6675 datasheet: 16-bit read-only frame, MSB first.
//   bit 15      dummy sign bit, always 0
//   bits 14..3  12-bit temperature, 0.25 degC per count
//   bit 2       1 = thermocouple open
//   bit 1       device ID, always 0
//   bit 0       three-state, ignored
#define BIT_DUMMY   0x8000
#define BIT_OPEN    0x0004
#define BIT_ID      0x0002

#define MAX6675_CS_GPIO   10
#define MAX6675_CLOCK_HZ  1000000  // Datasheet maximum is 4.3 MHz.
// Mode 0: the chip changes SO on the falling edge of SCK, so the master must
// sample on the rising edge. (Mode 1 samples while SO is changing and shifts
// bits.) If readings look doubled or halved on hardware, try mode 1 here.
#define MAX6675_SPI_MODE  0
// Power-up conversion takes up to ~220 ms before the first valid result.
#define STARTUP_CONVERSION_MS 250

static spi_bus_device_t s_device; // Static storage survives an SDK completion timeout.
static bool s_initialized;
static const char *TAG = "MAX6675";

static esp_err_t read_frame(uint16_t *raw)
{
    const uint8_t tx[2] = {0, 0};
    uint8_t rx[2] = {0, 0};
    esp_err_t err = spi_bus_manager_transfer(&s_device, tx, rx, sizeof(rx));
    if (err == ESP_OK) *raw = (uint16_t)((rx[0] << 8) | rx[1]);
    return err;
}

// Framing is checked before the fault bit: a floating line often reads 0xFFFF
// (all ones) and must not be mistaken for an open thermocouple.
static esp_err_t decode(uint16_t raw, float *temperature_c)
{
    if (raw & (BIT_DUMMY | BIT_ID)) return ESP_ERR_INVALID_RESPONSE;
    if (raw & BIT_OPEN) return ESP_ERR_NOT_FOUND;
    *temperature_c = (float)(raw >> 3) * 0.25f;
    return ESP_OK;
}

esp_err_t max6675_sensor_deinit(void)
{
    s_initialized = false;
    // If removal times out, retain the handle/descriptor. Never free memory
    // still owned by the SDK or tear down the shared bus to repair one reader.
    return spi_bus_manager_remove_device(&s_device);
}

esp_err_t max6675_sensor_init(void)
{
    if (s_initialized) return ESP_OK;
    if (s_device.handle) return ESP_ERR_INVALID_STATE;
    esp_err_t err = spi_bus_manager_add_device(&s_device, MAX6675_CS_GPIO,
                                               MAX6675_CLOCK_HZ, MAX6675_SPI_MODE);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(STARTUP_CONVERSION_MS));

    uint16_t raw = 0;
    err = read_frame(&raw);
    float temperature_c;
    if (err == ESP_OK) {
        err = decode(raw, &temperature_c);
        if (err == ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "Chip responded but the thermocouple is open; check the probe leads");
            err = ESP_OK;
        } else if (err == ESP_ERR_INVALID_RESPONSE) {
            ESP_LOGE(TAG, "Bad frame 0x%04X; check SO/SCK/CS wiring and power", raw);
        } else if (raw == 0) {
            // Plausible for a real 0.00 degC, but also what a dead MISO line reads.
            ESP_LOGW(TAG, "Frame is exactly 0x0000; if this is not 0 degC, check SO wiring");
        }
    }
    if (err != ESP_OK) {
        esp_err_t cleanup_err = max6675_sensor_deinit();
        if (cleanup_err != ESP_OK) {
            ESP_LOGE(TAG, "Device cleanup failed: %s", esp_err_to_name(cleanup_err));
        }
        return err;
    }
    s_initialized = true;
    ESP_LOGI(TAG, "MAX6675 ready | CS=%d | %d Hz | mode %d",
             MAX6675_CS_GPIO, MAX6675_CLOCK_HZ, MAX6675_SPI_MODE);
    return ESP_OK;
}

esp_err_t max6675_sensor_read(max6675_data_t *data)
{
    if (data == NULL) return ESP_ERR_INVALID_ARG;
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    uint16_t raw;
    esp_err_t err = read_frame(&raw);
    if (err != ESP_OK) return err;
    data->raw = raw;
    return decode(raw, &data->temperature_c);
}
