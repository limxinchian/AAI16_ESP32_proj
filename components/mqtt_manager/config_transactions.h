#pragma once
#include "mqtt_manager.h"
#include "aai16_protocol.h"

// Pure bounded state; caller serializes access. No RTOS, heap, I/O or encoding
// while the manager's control spinlock is held.
#define CONFIG_HISTORY_SIZE 16
#define CONFIG_FRAGMENT_TIMEOUT_US INT64_C(2000000)
typedef struct {
    mqtt_config_update_t update;
    aai16_ResultCode result;
    aai16_Phase phase;
    uint32_t generation;
} config_receipt_t;
typedef struct {
    config_receipt_t receipt;
    bool used, started, complete;
} config_history_t;
typedef struct {
    config_history_t history[CONFIG_HISTORY_SIZE];
    mqtt_config_update_t sidecar[SENSOR_STATUS_CAPACITY];
    bool occupied[SENSOR_STATUS_CAPACITY];
    uint32_t generation[SENSOR_STATUS_CAPACITY];
    uint64_t high_water_request;
    unsigned replacement;
} config_transactions_t;
typedef struct {
    uint8_t data[AAI16_CONFIG_FRAME_MAX];
    int total, received, message_id;
    int64_t started_us;
    bool active;
    uint32_t rejected, expired;
} config_fragments_t;

bool config_sensor_from_wire(aai16_SensorId wire, sensor_id_t *sensor);
aai16_SensorId config_sensor_to_wire(sensor_id_t sensor);
void config_requested(const mqtt_config_update_t *u, aai16_SensorConfig *out);
// 1: enqueue, 0: duplicate pending (no success ACK), -1: send receipt.
int config_admit(config_transactions_t *s, const aai16_Envelope *e,
                 uint32_t boot_id, mqtt_config_update_t *u, config_receipt_t *reply);
bool config_begin(config_transactions_t *s, mqtt_config_update_t *u, config_receipt_t *failure);
bool config_finish(config_transactions_t *s, const mqtt_config_update_t *u,
                   aai16_ResultCode result, config_receipt_t *reply);
bool config_stage(config_transactions_t *s, const mqtt_config_update_t *u);
bool config_take(config_transactions_t *s, sensor_id_t sensor, mqtt_config_update_t *u);
void config_fragment_expire(config_fragments_t *s, int64_t now);
// Returns full length only on exact completion; otherwise zero. Caller copies
// completed data before unlocking. A new first fragment abandons partial input.
size_t config_fragment_feed(config_fragments_t *s, int64_t now, bool topic_matches,
                            bool topic_present, bool retained, int message_id,
                            int offset, int total, const void *data, int length);
