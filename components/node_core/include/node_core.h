#pragma once
/* Node core: wires config_manager, sensor_runtime, rule engine, bridge, ML selector, model store, REPL
 * and heartbeat into one node, with every hardware/network/storage dependency behind node_platform_t.
 * Platform-agnostic C: the host tests drive it with simulated devices; main/ will supply the ESP-IDF
 * implementation (MQTT, NVS, partitions, drivers, TFLM).
 *
 * Threading: one task owns the node. Feed MQTT payloads with node_on_message() and call node_tick()
 * regularly; never touch the node from another task.
 * Memory: a node_t is ~150 KB - allocate it in PSRAM (EXT_RAM_BSS_ATTR / heap_caps_calloc), not on a stack.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "aai16.pb.h"
#include "bridge.h"
#include "config_manager.h"
#include "ml_package.h"
#include "ml_selector.h"
#include "model_store.h"
#include "repl.h"
#include "rule_engine.h"
#include "sensor_runtime.h"

typedef enum {
    OUT_APPLY_ACK,     /* ApplyAck for ConfigApply (config topic) */
    OUT_READING,       /* Reading of a PHYSICAL sensor only (never virtual: prevents forwarding loops) */
    OUT_RULE_ACK,      /* ApplyAck for RuleSetApply */
    OUT_BRIDGE_ACK,    /* ApplyAck for BridgeApply */
    OUT_CHUNK_ACK,     /* ChunkAck */
    OUT_MODEL_STATUS,  /* ModelStatus */
    OUT_INFERENCE,     /* InferenceResult */
    OUT_HEARTBEAT,     /* Heartbeat */
    OUT_NOTIFICATION,  /* Notification */
    OUT_NODE_COMMAND,  /* NodeCommand for another node (target != NULL) */
} node_out_t;

typedef enum {
    IN_CONFIG, IN_RULES, IN_BRIDGE, IN_MODEL_CHUNK, IN_MODEL_CONTROL, IN_COMMAND,
    IN_REMOTE_READING, /* another node's Reading; `from_node` required */
} node_in_t;

typedef struct {
    const char *node_id; /* <= 15 characters */
    bus_backend_t bus;
    model_store_ops_t store;

    /* network: `target` is NULL for this node's own topics, else the destination node id */
    bool (*publish)(void *c, node_out_t kind, const char *target, const uint8_t *buf, size_t len);

    /* committed configuration (CommittedConfig bytes) */
    bool (*persist_config)(void *c, const uint8_t *blob, size_t len);
    bool (*load_config)(void *c, uint8_t *buf, size_t cap, size_t *len);
    /* NodeState (rules, bridge, ML mapping, model generation) - optional */
    bool (*persist_state)(void *c, const uint8_t *blob, size_t len);
    bool (*load_state)(void *c, uint8_t *buf, size_t cap, size_t *len);

    /* rule outputs */
    void (*gpio_set)(void *c, uint32_t pin, bool level);
    void (*gpio_toggle)(void *c, uint32_t pin);
    void (*gpio_pulse)(void *c, uint32_t pin, bool level, uint32_t ms);
    void (*pwm)(void *c, uint32_t pin, uint32_t permille);

    /* inference runtime (TFLM on target). ml_load must fully validate the model (allocate arena,
     * resolve ops, check tensor shapes against the contract) and, on success, make it the loaded model. */
    aai16_ModelReason (*ml_load)(void *c, const ml_package_t *pkg);
    bool (*ml_invoke)(void *c, const int8_t *in, uint32_t n_in, int8_t *out, uint32_t n_out, uint32_t *micros);

    void (*reboot)(void *c); /* optional: restart the chip (REPL REBOOT) */
    uint64_t (*now_us)(void *c);
    uint32_t (*free_heap)(void *c);
    void (*log)(void *c, const char *line);
    void *ctx;
} node_platform_t;

#define NODE_BLOB_MAX        12288
#define NODE_HEARTBEAT_US    10000000u
#define NODE_OUT_BUF         512

typedef struct {
    node_platform_t pf;
    config_manager_t cm;
    sensor_runtime_t rt;
    rule_engine_t re;
    bridge_t br;
    ml_selector_t sel;
    model_store_t ms;

    bool paused, booted;
    struct { bool on; uint64_t stop_us; } wake[CM_BUS_COUNT]; /* dormant sensors woken by a rule (RAM only, never persisted) */
    aai16_SensorProfile wake_scratch;                           /* the dormant profile turned into a running one */
    int watch;                   /* REPL WATCH: 0 off, 1..255 one sensor, 256 all */
    uint32_t watch_last_ms[256]; /* rate limit: one line per sensor per second */
    uint32_t model_gen, rule_acks;
    bool model_active;           /* a model is loaded in the runtime */
    aai16_ModelKind model_kind;
    uint32_t model_version;
    aai16_ModelState rep_state;  /* last reported model state/reason (publish on change) */
    aai16_ModelReason rep_reason;
    uint32_t inferences;
    uint64_t boot_us, next_heartbeat_us;

    /* decode / encode scratch (single owner task) */
    aai16_RuleSetApply rules_rx;
    aai16_BridgeApply bridge_rx;
    aai16_ModelChunk chunk_rx;
    aai16_ModelControl ctrl_rx;
    aai16_NodeState state;       /* scratch for save/restore of the persisted node state */
    uint8_t blob[NODE_BLOB_MAX];
    uint8_t out[NODE_OUT_BUF];
} node_t;

/* Wire everything up (no I/O). */
void node_init(node_t *n, const node_platform_t *pf);
/* Restore the last committed configuration and the active model (if any); publishes a ModelStatus. */
void node_boot(node_t *n);
void node_on_message(node_t *n, node_in_t kind, const char *from_node, const uint8_t *buf, size_t len);
/* Run due sensor cycles, bridge freshness and the heartbeat timer. */
void node_tick(node_t *n);

/* REPL glue: lines go through repl_exec with this node's handlers. */
int node_repl_line(node_t *n, const char *line, char *out, size_t cap);
/* Feed a reading that did not come from the local runtime (bridge). Public for tests. */
void node_feed_reading(node_t *n, const aai16_Reading *r);
