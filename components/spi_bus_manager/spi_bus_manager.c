#include "spi_bus_manager.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

static bool s_initialized;

esp_err_t spi_bus_manager_init(void)
{
    if (s_initialized) return ESP_OK;
    const spi_bus_config_t config = {
        .mosi_io_num = 11, .miso_io_num = 13, .sclk_io_num = 12,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .data4_io_num = -1, .data5_io_num = -1,
        .data6_io_num = -1, .data7_io_num = -1,
        .max_transfer_sz = 4,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &config, SPI_DMA_DISABLED);
    if (err == ESP_OK) {
        s_initialized = true;
        ESP_LOGI("AAI16_SPI", "SPI2 ready | SCK=12 MOSI=11 MISO=13");
    }
    return err;
}

esp_err_t spi_bus_manager_add_device(spi_bus_device_t *device, int cs_gpio,
                                     int clock_hz, uint8_t mode)
{
    if (!device || device->handle || clock_hz <= 0 || mode > 3 || cs_gpio < 0 ||
        cs_gpio == 11 || cs_gpio == 12 || cs_gpio == 13) return ESP_ERR_INVALID_ARG;
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    const spi_device_interface_config_t config = {
        .clock_speed_hz = clock_hz, .mode = mode,
        .spics_io_num = cs_gpio, .queue_size = 1,
    };
    return spi_bus_add_device(SPI2_HOST, &config, &device->handle);
}

static esp_err_t finish(spi_bus_device_t *device, TickType_t wait)
{
    spi_transaction_t *done = NULL;
    esp_err_t err = spi_device_get_trans_result(device->handle, &done, wait);
    if (done == &device->transaction) device->pending = false;
    if (err == ESP_OK && done != &device->transaction) return ESP_ERR_INVALID_STATE;
    return err;
}

esp_err_t spi_bus_manager_transfer(spi_bus_device_t *device, const uint8_t *tx,
                                  uint8_t *rx, size_t length)
{
    if (!device || !tx || !length || length > 4) return ESP_ERR_INVALID_ARG;
    if (!device->handle) return ESP_ERR_INVALID_STATE;
    if (device->pending) {
        esp_err_t err = finish(device, 0);
        if (err != ESP_OK) return err;
    }
    device->transaction = (spi_transaction_t){
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = length * 8,
    };
    memcpy(device->transaction.tx_data, tx, length);
    esp_err_t err = spi_device_queue_trans(device->handle, &device->transaction, 0);
    if (err != ESP_OK) return err;
    device->pending = true;
    err = finish(device, pdMS_TO_TICKS(20));
    if (err == ESP_OK && rx) memcpy(rx, device->transaction.rx_data, length);
    return err;
}

esp_err_t spi_bus_manager_remove_device(spi_bus_device_t *device)
{
    if (!device) return ESP_ERR_INVALID_ARG;
    if (!device->handle) return ESP_OK;
    if (device->pending) {
        esp_err_t err = finish(device, 0);
        if (err != ESP_OK) return err;
    }
    esp_err_t err = spi_bus_remove_device(device->handle);
    if (err == ESP_OK) device->handle = NULL;
    return err;
}
