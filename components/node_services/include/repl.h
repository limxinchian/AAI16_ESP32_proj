#pragma once
/* Serial REPL command layer (handoff 6.2). Human-readable commands; configuration travels as
 * hex-encoded Protobuf and is handed to the SAME handlers the MQTT path uses, so there is one
 * validation/apply code path and no duplicated business logic.
 *
 *   HELP | RUN | STOP | CLEAR | SHOW | REBOOT | WATCH <id|ALL|OFF> | CFG <hex> | RULES <hex> | BRIDGE <hex>
 *
 * Hex payloads are capped (REPL_MAX_PAYLOAD); larger configurations must use MQTT.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define REPL_MAX_PAYLOAD 4096

typedef struct {
    bool (*run)(void *c);
    bool (*stop)(void *c);
    bool (*clear)(void *c);                       /* stop all buses, drop rules; goes through the config path */
    void (*apply_config)(void *c, const uint8_t *buf, size_t len); /* = MQTT config handler */
    void (*apply_rules)(void *c, const uint8_t *buf, size_t len);
    void (*apply_bridge)(void *c, const uint8_t *buf, size_t len);
    size_t (*show)(void *c, char *out, size_t cap);
    void *ctx;
    void (*reboot)(void *c); /* optional (NULL = unsupported): restart the node */
    void (*watch)(void *c, int which); /* optional: print readings (0 = off, 1..255 = that sensor, 256 = all) */
} repl_ops_t;

/* Executes one line (no trailing newline). Writes a reply into out. Returns 0 on success,
 * negative on an error that is also described in the reply ("ERR ..."). */
int repl_exec(const repl_ops_t *ops, const char *line, char *out, size_t cap);
