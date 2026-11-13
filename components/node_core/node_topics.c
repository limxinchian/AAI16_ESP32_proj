/* MQTT topic names for every message kind; see node_topics.h (checked against the laptop client in tests/topics_host). */
#include "node_topics.h"

#include <stdio.h>
#include <string.h>

bool node_topic_classify(const char *own_id, const char *topic, node_in_t *kind, char from[NODE_ID_MAX + 1])
{
    from[0] = '\0';
    if (strncmp(topic, "aai16/", 6) != 0) return false;
    const char *p = topic + 6;
    const char *slash = strchr(p, '/');
    if (!slash || slash == p || (size_t)(slash - p) > NODE_ID_MAX) return false;
    char node[NODE_ID_MAX + 1];
    memcpy(node, p, (size_t)(slash - p));
    node[slash - p] = '\0';
    const char *rest = slash + 1;
    const bool mine = strcmp(node, own_id) == 0;

    if (mine && strncmp(rest, "in/", 3) == 0) {
        rest += 3;
        if (!strcmp(rest, "config")) *kind = IN_CONFIG;
        else if (!strcmp(rest, "rules")) *kind = IN_RULES;
        else if (!strcmp(rest, "bridge")) *kind = IN_BRIDGE;
        else if (!strcmp(rest, "model/chunk")) *kind = IN_MODEL_CHUNK;
        else if (!strcmp(rest, "model/control")) *kind = IN_MODEL_CONTROL;
        else if (!strcmp(rest, "command")) *kind = IN_COMMAND;
        else return false;
        return true;
    }
    if (!mine && strncmp(rest, "out/reading/", 12) == 0 && rest[12] != '\0') { /* another node's reading */
        *kind = IN_REMOTE_READING;
        strcpy(from, node);
        return true;
    }
    return false;
}

bool node_topic_build(node_out_t kind, const char *own_id, const char *target, const uint8_t *payload, size_t len,
                      char *topic, size_t cap, int *qos)
{
    const char *suffix = NULL;
    *qos = 1;
    switch (kind) {
    case OUT_APPLY_ACK:    suffix = "config_ack"; break;
    case OUT_RULE_ACK:     suffix = "rules_ack"; break;
    case OUT_BRIDGE_ACK:   suffix = "bridge_ack"; break;
    case OUT_CHUNK_ACK:    suffix = "model_chunk_ack"; break;
    case OUT_MODEL_STATUS: suffix = "model_status"; break;
    case OUT_INFERENCE:    suffix = "inference"; break;
    case OUT_HEARTBEAT:    suffix = "heartbeat"; break;
    case OUT_NOTIFICATION: suffix = "notification"; break;
    case OUT_NODE_COMMAND:
        if (!target || !*target || strlen(target) > NODE_ID_MAX) return false;
        return (size_t)snprintf(topic, cap, "aai16/%s/in/command", target) < cap;
    case OUT_READING:
        /* A Reading starts with field 1 (sensor_id varint): bytes 0x08 <id>. Physical ids are 1..127 (one byte). */
        if (len < 2 || payload[0] != 0x08 || payload[1] == 0 || payload[1] > 127) return false;
        *qos = 0;
        return (size_t)snprintf(topic, cap, "aai16/%s/out/reading/%u", own_id, (unsigned)payload[1]) < cap;
    default:
        return false;
    }
    return (size_t)snprintf(topic, cap, "aai16/%s/out/%s", own_id, suffix) < cap;
}
