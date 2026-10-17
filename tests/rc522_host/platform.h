#pragma once
#include "../sensor_host/platform.h"

#define ESP_ERR_INVALID_SIZE 0x104
#define SPI2_HOST 1
#define SPI_DMA_DISABLED 0
#define SPI_TRANS_USE_TXDATA 1
#define SPI_TRANS_USE_RXDATA 2
#define GPIO_MODE_OUTPUT 1

typedef struct {
    uint64_t pin_bit_mask;
    int mode;
} gpio_config_t;
esp_err_t gpio_config(const gpio_config_t *config);
esp_err_t gpio_set_level(int gpio, int level);
void vTaskDelete(TaskHandle_t task);

typedef void *spi_device_handle_t;
typedef struct {
    int mosi_io_num, miso_io_num, sclk_io_num, quadwp_io_num, quadhd_io_num;
    int data4_io_num, data5_io_num, data6_io_num, data7_io_num, max_transfer_sz;
} spi_bus_config_t;
typedef struct { int clock_speed_hz, mode, spics_io_num, queue_size; } spi_device_interface_config_t;
typedef struct { unsigned flags; size_t length; uint8_t tx_data[4], rx_data[4]; } spi_transaction_t;
esp_err_t spi_bus_initialize(int host, const spi_bus_config_t *config, int dma);
esp_err_t spi_bus_add_device(int host, const spi_device_interface_config_t *config, spi_device_handle_t *handle);
esp_err_t spi_bus_remove_device(spi_device_handle_t handle);
esp_err_t spi_device_queue_trans(spi_device_handle_t handle, spi_transaction_t *transaction, TickType_t wait);
esp_err_t spi_device_get_trans_result(spi_device_handle_t handle, spi_transaction_t **transaction, TickType_t wait);
