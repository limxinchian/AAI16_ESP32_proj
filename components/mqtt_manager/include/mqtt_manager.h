
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Initialize once before any sensor tasks use this manager. Client retries
// automatically if Wi-Fi/broker is unavailable; no wait for a connection.
esp_err_t mqtt_manager_init(void);

// Publication functions copy into a bounded local queue with zero queue wait.
// ESP_OK means local queue acceptance, NOT ESP-MQTT enqueue or broker delivery.
// The worker publishes QoS 0 directly; only QoS 1 configuration ACKs use the
// SDK outbox. publish_ok = local write success; enqueued = ACK outbox acceptance.
// Neither proves subscriber receipt. See routine diagnostics for all drops.
// seq counts per-sensor API attempts (including skipped/failed submissions).
// uptime_ms is successful-read submission time, before either queue.

esp_err_t mqtt_manager_publish_dht22(
    float temperature_c,
    float humidity_pct
);

// Publish an ACK after applying a configuration.
esp_err_t mqtt_manager_publish_config_ack(
    uint32_t interval_ms
);

// Reject a validated DHT22 command when no active acquisition loop is available.
// Uses status="rejected", reason="sensor_unavailable", and the existing ACK path.
esp_err_t mqtt_manager_reject_config_unavailable(void);


esp_err_t mqtt_manager_publish_gy511(
    float accel_x_g,
    float accel_y_g,
    float accel_z_g,
    float mag_x_ut,
    float mag_y_ut,
    float mag_z_ut
);


// Retrieve a validated interval command, if available.
bool mqtt_manager_take_interval_update(uint32_t *out_ms);

// Call only from app_main. DIAGNOSTIC_PERIOD_MS in mqtt_manager.c defaults to
// 60000; use 10000 for detailed testing. Connection/errors log immediately.
void mqtt_manager_log_diagnostics(void);
