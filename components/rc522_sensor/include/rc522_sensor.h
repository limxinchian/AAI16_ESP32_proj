#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint8_t bytes[10];
    uint8_t length; // 4, 7 or 10 after successful selection and CRC/BCC checks.
} rc522_uid_t;

// Single reader/task owns all calls. Shared SPI bus must already be initialized.
// CS=GPIO14, RST=GPIO21, SPI mode 0 at 1 MHz; no IRQ connection is needed.
esp_err_t rc522_sensor_init(void);
esp_err_t rc522_sensor_deinit(void);
// ESP_OK + present=false is normal RF idle, NOT a transport/software timeout.
// WUPA allows a held, previously halted card to be detected on every poll.
// At most one UID is selected per poll; this is not a multi-card inventory API.
// A 300 ms software budget covers exchanges; an in-flight SPI wait can add 20 ms.
esp_err_t rc522_sensor_poll(rc522_uid_t *uid, bool *present);
