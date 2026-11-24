#pragma once
/* ESP-IDF implementation of node_platform_t (buses, GPIO outputs, flash storage, Wi-Fi/MQTT, TFLM).
 * Built for the ESP32-S3 and run on two boards; see docs/project_status.md for what has been verified and how. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "node_core.h"

/* Message handed from the MQTT/console tasks to the node task. `data` is heap memory owned by the
 * receiver (free with heap_caps_free / free). */
#define PLAT_IN_REPL 100
typedef struct {
    int kind; /* node_in_t, or PLAT_IN_REPL (data = NUL-terminated line) */
    char from[16];
    uint8_t *data;
    size_t len;
} plat_msg_t;

#define PLAT_MAX_MESSAGE 32768 /* >= aai16_ConfigApply_size */

/* Fills every hardware callback of the platform (bus, storage, outputs, runtime, clock, publish). */
void plat_fill(node_platform_t *pf, const char *node_id);

/* --- individual pieces (also used by the host-independent smoke tests) --- */
bus_backend_t plat_bus_backend(void);
void plat_storage_fill(node_platform_t *pf);
void plat_io_fill(node_platform_t *pf);
void plat_ml_fill(node_platform_t *pf);

/* Wi-Fi station + MQTT client. Incoming messages are queued on `inbox`. */
bool plat_net_start(const char *node_id, const char *ssid, const char *password, const char *broker_uri,
                    QueueHandle_t inbox);
bool plat_net_publish(void *ctx, node_out_t kind, const char *target, const uint8_t *buf, size_t len);
