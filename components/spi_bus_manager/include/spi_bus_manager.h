#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "driver/spi_master.h"

// One owner/task per device. Keep this object alive until remove succeeds:
// an SDK transaction can outlive our bounded wait. Do not use raw SDK transfers
// on its handle. These short transfers cover RC522 registers and MAX6675 words.
typedef struct {
    spi_device_handle_t handle;
    spi_transaction_t transaction;
    bool pending;
} spi_bus_device_t;

// Call once from app_main before starting SPI device workers; repeat calls from
// the same startup owner are harmless. SPI2: SCK12, MOSI11, MISO13; no DMA.
esp_err_t spi_bus_manager_init(void);
esp_err_t spi_bus_manager_add_device(spi_bus_device_t *device, int cs_gpio,
                                     int clock_hz, uint8_t mode);
// 1..4 bytes, full duplex, hardware CS for the entire transfer. Queue wait=0,
// completion wait=20 ms. A late result is discarded before any next transfer.
esp_err_t spi_bus_manager_transfer(spi_bus_device_t *device, const uint8_t *tx,
                                  uint8_t *rx, size_t length);
// Nonblocking; refuses removal while an SDK transaction is still pending.
// Never frees/reinitializes the shared bus or another device.
esp_err_t spi_bus_manager_remove_device(spi_bus_device_t *device);
