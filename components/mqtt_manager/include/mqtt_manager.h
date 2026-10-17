
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "sensor_status.h"
#include "ld2450_sensor.h"

typedef enum {
    MQTT_CONFIG_INTERVAL,
    MQTT_CONFIG_ENABLED,
} mqtt_config_operation_t;

typedef struct {
    sensor_id_t sensor;
    mqtt_config_operation_t operation;
    union {
        uint32_t interval_ms;
        bool enabled;
    };
} mqtt_config_update_t;

// Initialize once before any sensor tasks use this manager. Client retries
// automatically if Wi-Fi/broker is unavailable; no wait for a connection.
esp_err_t mqtt_manager_init(void);

// Publication functions copy into a bounded local queue with zero queue wait.
// ESP_OK means local queue acceptance, NOT ESP-MQTT enqueue or broker delivery.
// The worker publishes periodic samples at QoS 0; configuration ACKs and RFID
// events use the QoS 1 SDK outbox. publish_ok = local write success;
// enqueued = SDK outbox acceptance. PUBACK/SDK expiry totals cover both kinds.
// Neither proves subscriber receipt. See routine diagnostics for all drops.
// seq counts per-sensor API attempts (including skipped/failed submissions).
// uptime_ms is successful-read submission time, before either queue.

esp_err_t mqtt_manager_publish_dht22(
    float temperature_c,
    float humidity_pct
);

// Call once per presentation, not per poll. Copies 4/7/10-byte UIDs; uppercase
// uid_hex has no separators. Separate 8-event FIFO accepts while disconnected,
// drops newest when full, and expires waiting events after 60000 ms. No flash
// persistence. seq counts attempts, including rejected/dropped submissions.
// Publisher uses non-retained QoS 1 with a 1024-byte event outbox allowance,
// preserving 4096 bytes for ACKs/control. SDK expiry remains its existing 30 s
// default after enqueue. ESP_OK means local acceptance, not broker delivery.
// SDK retries can deliver duplicates: deduplicate by (sensor, boot_id, seq).
esp_err_t mqtt_manager_publish_rc522_event(const uint8_t *uid, uint8_t uid_length);

// Call only for a validated radar report, with its acquisition timestamp.
// Copies one latest snapshot under a short spinlock; no queue wait or SDK call.
// New reports replace pending ones, including offline. The existing publisher
// samples this mailbox at most once/second, at QoS 0, retain=0. Reports aged
// >=2 s are dropped, and a consumed report is never resent without an update.
// seq counts publication attempts (including offline/stale drops), not frames;
// uptime_ms is acquisition time. Pending ACK/telemetry queue work takes priority.
// ESP_OK means snapshot acceptance only; delivery failures never affect UART
// sensor status. This path uses neither the shared TX queue nor the QoS 1 outbox.
esp_err_t mqtt_manager_update_ld2450_snapshot(const ld2450_report_t *report,
                                             int64_t received_us);

// Publish only after the acquisition owner commits the requested operation.
esp_err_t mqtt_manager_publish_config_ack(const mqtt_config_update_t *update);

// Reject a validated command, echoing its sensor and requested operation/value.
esp_err_t mqtt_manager_reject_config(const mqtt_config_update_t *update,
                                   const char *reason);


esp_err_t mqtt_manager_publish_gy511(
    float accel_x_g,
    float accel_y_g,
    float accel_z_g,
    float mag_x_ut,
    float mag_y_ut,
    float mag_z_ut
);


// One shared four-entry FIFO. Only app_main may peek/remove commands. Peeking
// allows a busy reader's command to remain queued until its handoff succeeds.
// Both calls are nonblocking; the MQTT callback only appends to this queue.
// Exactly one operation per command. Owners reject interval changes while
// disabled with sensor_disabled; the previous interval is retained on resume.
// Disable commits between reads and pauses acquisition, not hardware power.
// Previously queued telemetry may still arrive afterward. Re-enable resets the
// deadline by one retained period; an already enabled sensor keeps its deadline.
// All three sensors start enabled with default intervals after each boot.
bool mqtt_manager_peek_config_update(mqtt_config_update_t *out);
bool mqtt_manager_take_config_update(mqtt_config_update_t *out);

// Call only from app_main. DIAGNOSTIC_PERIOD_MS in mqtt_manager.c defaults to
// 60000; use 10000 for detailed testing. Connection/errors log immediately.
void mqtt_manager_log_diagnostics(void);
