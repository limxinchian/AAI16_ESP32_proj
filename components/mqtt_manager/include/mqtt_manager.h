
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "sensor_status.h"
#include "ld2450_sensor.h"
#include "hcsr04p_sensor.h"
typedef struct motion_result motion_result_t;

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
// Stable identity assigned during init, also shared by local ML windows.
uint32_t mqtt_manager_boot_id(void);
// Temporary JSON adapter for a transport-independent bounded ML result.
// Uses the existing zero-wait, QoS0, non-retained telemetry path and diagnostics.
esp_err_t mqtt_manager_publish_motion(const motion_result_t *result);

// Publication functions copy into a bounded local queue with zero queue wait.
// ESP_OK means local queue acceptance, NOT ESP-MQTT enqueue or broker delivery.
// The worker publishes periodic samples at QoS 0; configuration ACKs and RFID
// events use the QoS 1 SDK outbox. publish_ok = local write success;
// enqueued = SDK outbox acceptance. PUBACK/SDK expiry totals cover both kinds.
// Neither proves subscriber receipt. See routine diagnostics for all drops.
// seq counts per-sensor API attempts (including skipped/failed submissions).
// uptime_ms is successful-read submission time unless documented otherwise.

esp_err_t mqtt_manager_publish_dht22(
    float temperature_c,
    float humidity_pct
);

// Same zero-wait QoS 0 path as DHT22/GY-511. has_humidity is false for a BMP280;
// humidity_pct is then ignored and published as JSON null.
esp_err_t mqtt_manager_publish_bme280(
    float temperature_c,
    float pressure_hpa,
    float humidity_pct,
    bool has_humidity
);

// Same zero-wait QoS 0 path as the other polled sensors. Thermocouple-open and
// framing faults are read failures in Sensor Status, never published as 0 degC.
esp_err_t mqtt_manager_publish_max6675(float temperature_c);

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
// samples this mailbox at the configured period (default 1000 ms), QoS 0,
// retain=0. Disabled updates are rejected with ESP_ERR_INVALID_STATE. Reports aged
// >=2 s are dropped, and a consumed report is never resent without an update.
// seq counts publication attempts (including offline/stale drops), not frames;
// uptime_ms is acquisition time. Pending ACK/telemetry queue work takes priority.
// ESP_OK means snapshot acceptance only; delivery failures never affect UART
// sensor status. This path uses neither the shared TX queue nor the QoS 1 outbox.
esp_err_t mqtt_manager_update_ld2450_snapshot(const ld2450_report_t *report,
                                             int64_t received_us);

// Reader task only: hand off an ESP_OK reading with its read-completion time.
// Copies one latest-valid snapshot under a short spinlock; no allocation, queue
// wait or SDK call. New successes replace unpublished data, including offline.
// The existing publisher consumes at most one snapshot per 1000 ms, QoS 0,
// retain=0, after pending ACK/telemetry/RFID work. Age >=2000 ms is rejected
// immediately before SDK submission. No retry or resend of a consumed sample.
// Failed acquisitions MUST NOT call this API: a previous success can remain
// eligible until consumed/replaced/expired, with its original timestamp intact.
// seq counts consumed publication attempts, including offline/stale/SDK drops;
// it starts at 1 per boot and wraps as uint32_t. Replacements, invalid updates
// and busy deferrals do not advance seq. uptime_ms = acquired_us / 1000.
// ESP_OK means snapshot acceptance only. MQTT outcomes never alter sensor health.
// Disabled handoffs return ESP_ERR_INVALID_STATE. Gate transitions cancel any
// consumed attempt not yet admitted to the SDK (counted as cancelled).
esp_err_t mqtt_manager_update_hcsr04p_snapshot(const hcsr04p_reading_t *reading,
                                              int64_t acquired_us);

// MH-Z1911A reader only: call once per validated success, with read-completion
// uptime. One latest sample, protected by the existing stats/snapshot spinlock;
// no allocation, queue wait or SDK call. Rejects invalid ppm/timestamps and
// repeated/out-of-order timestamps without replacing pending data.
// The existing publisher consumes pending data after other telemetry/event work.
// Publication follows the configured acquisition cycle (default 5000 ms).
// QoS 0, retain=0; age >=2000 ms at SDK admission is stale. Offline/stale/failed
// attempts are discarded, never retried. Pending samples can be replaced while
// the publisher is busy/offline; reconnect replays no consumed history.
// seq counts consumed attempts, including offline/stale/cancelled/SDK failures, starting
// at 1 per boot and wrapping as uint32_t. Updates/replacements/deferrals and
// invalid or duplicate handoffs do not advance it. uptime_ms = acquired_us/1000.
// ESP_OK means snapshot acceptance only, not broker or subscriber delivery.
// Disabled handoffs return ESP_ERR_INVALID_STATE.
esp_err_t mqtt_manager_update_mhz1911a_snapshot(uint16_t co2_ppm, int64_t acquired_us);

#define MHZ1911A_MIN_INTERVAL_MS 5000U
#define MHZ1911A_MAX_INTERVAL_MS 60000U

// Reader owner only, between reads: commit after every interval command and
// enabled-state edge, before the applied ACK. Infallible bounded update clears
// pending data and invalidates copied attempts not yet admitted to the SDK.
// The reader resets its schedule; enable requires a fresh handoff. Repeated
// enable values should not call this API. Already admitted sends may complete.
void mqtt_manager_commit_mhz1911a_config(bool enabled);

// HC-SR04P reader owner only, between complete acquisitions and after committing
// Sensor Status. Infallible bounded gate update; clears the pending snapshot and
// invalidates copied-but-not-admitted data on transitions. Repeated values are
// idempotent. No SDK call or publication-period change. Already admitted sends
// may finish after disable; enable requires a fresh reader handoff.
void mqtt_manager_commit_hcsr04p_enabled(bool enabled);

// LD2450 reader owner only, after generic availability/disabled-policy checks.
// Atomically commit the publication gate/interval; never call the MQTT SDK.
// Interval changes restart the deadline from commit time. A gate transition
// clears pending data; enable waits one retained interval and requires a fresh
// handoff. Repeated enabled=true leaves the existing deadline/data untouched.
// The owner must discard pre-enable UART bytes before enabling, and commit
// Sensor Status before issuing the applied ACK. Already admitted sends may complete.
esp_err_t mqtt_manager_commit_ld2450_config(const mqtt_config_update_t *update);

// Publish only after the acquisition owner commits the requested operation.
esp_err_t mqtt_manager_publish_config_ack(const mqtt_config_update_t *update);

// Reject a validated command, echoing its sensor and requested operation/value.
esp_err_t mqtt_manager_reject_config(const mqtt_config_update_t *update,
                                   const char *reason);


// Compatibility API: submission timestamp, jitter=0 (not measured).
esp_err_t mqtt_manager_publish_gy511(
    float accel_x_g,
    float accel_y_g,
    float accel_z_g,
    float mag_x_ut,
    float mag_y_ut,
    float mag_z_ut
);

// Acquisition owner: same zero-wait QoS0 queue and JSON axes/units. uptime_ms
// is acquired_us/1000 from the acceleration burst; jitter_us is signed read
// start minus scheduled deadline. Queue expiry uses acquisition time too.
// seq counts handoff attempts, NOT failed I2C acquisitions (detect those via
// timestamp gaps and Sensor Status). Never retry a cached reading.
esp_err_t mqtt_manager_publish_gy511_at(
    float accel_x_g, float accel_y_g, float accel_z_g,
    float mag_x_ut, float mag_y_ut, float mag_z_ut,
    int64_t acquired_us, int64_t jitter_us);

// One shared four-entry FIFO. Only app_main may peek/remove commands. Peeking
// allows a busy reader's command to remain queued until its handoff succeeds.
// Both calls are nonblocking; the MQTT callback only appends to this queue.
// Exactly one operation per command. Owners reject interval changes while
// disabled with sensor_disabled; the previous interval is retained on resume.
// Disable commits between reads and pauses acquisition, not hardware power;
// LD2450 instead keeps draining UART, pausing telemetry and health updates.
// Previously queued telemetry may still arrive afterward. Re-enable resets the
// deadline by one retained period; an already enabled sensor keeps its deadline.
// All eight sensors start enabled with default intervals after each boot.
bool mqtt_manager_peek_config_update(mqtt_config_update_t *out);
bool mqtt_manager_take_config_update(mqtt_config_update_t *out);

// Call only from app_main. DIAGNOSTIC_PERIOD_MS in mqtt_manager.c defaults to
// 60000; use 10000 for detailed testing. Connection/errors log immediately.
void mqtt_manager_log_diagnostics(void);
