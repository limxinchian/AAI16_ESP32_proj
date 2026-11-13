#pragma once
/* MQTT topic scheme in one place (pure C, host-tested against tools/node_client.py):
 *   in  (to this node):  aai16/<id>/in/{config,rules,bridge,model/chunk,model/control,command}
 *   out (from this node): aai16/<id>/out/{config_ack,rules_ack,bridge_ack,model_chunk_ack,model_status,inference,
 *                                          heartbeat,notification}  and  aai16/<id>/out/reading/<sensor_id>
 *   rule -> other node:   aai16/<target>/in/command
 *   bridge input:         aai16/<other>/out/reading/<sensor_id>
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "node_core.h"

#define NODE_ID_MAX 15

/* Classifies a received topic. For IN_REMOTE_READING, `from` is the other node's id. */
bool node_topic_classify(const char *own_id, const char *topic, node_in_t *kind, char from[NODE_ID_MAX + 1]);

/* Builds the topic for an outgoing message. Returns false if the message cannot be published (bad Reading, no
 * target). *qos = 0 for readings (best effort), 1 for everything else. */
bool node_topic_build(node_out_t kind, const char *own_id, const char *target, const uint8_t *payload, size_t len,
                      char *topic, size_t cap, int *qos);
