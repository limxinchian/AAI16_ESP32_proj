/* Reads requests from stdin, prints answers on stdout (the judging is done in run.py):
 *   C <own_id> <topic>                       -> "R <INKIND|-> <from>"
 *   B <own_id> <OUTKIND> <target|-> <hex>    -> "T <topic|-> <qos>"            */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "node_topics.h"

static const char *IN_NAMES[] = {"IN_CONFIG", "IN_RULES", "IN_BRIDGE", "IN_MODEL_CHUNK", "IN_MODEL_CONTROL", "IN_COMMAND", "IN_REMOTE_READING"};
static const char *OUT_NAMES[] = {"OUT_APPLY_ACK", "OUT_READING", "OUT_RULE_ACK", "OUT_BRIDGE_ACK", "OUT_CHUNK_ACK", "OUT_MODEL_STATUS",
                                  "OUT_INFERENCE", "OUT_HEARTBEAT", "OUT_NOTIFICATION", "OUT_NODE_COMMAND"};

int main(void)
{
    char op[4], own[64], a[256], b[256], c[512];
    while (scanf("%3s", op) == 1) {
        if (op[0] == 'C') {
            if (scanf("%63s %255s", own, a) != 2) return 2;
            node_in_t kind = IN_CONFIG;
            char from[NODE_ID_MAX + 1];
            if (node_topic_classify(own, a, &kind, from)) printf("R %s %s\n", IN_NAMES[kind], from[0] ? from : "-");
            else printf("R - -\n");
        } else {
            if (scanf("%63s %255s %255s %511s", own, a, b, c) != 4) return 2;
            int k = -1;
            for (int i = 0; i < 10; i++) if (!strcmp(a, OUT_NAMES[i])) k = i;
            if (k < 0) return 3;
            uint8_t payload[256];
            size_t n = 0;
            if (strcmp(c, "-")) for (size_t i = 0; c[i] && c[i + 1] && n < sizeof(payload); i += 2) { unsigned v; sscanf(c + i, "%2x", &v); payload[n++] = (uint8_t)v; }
            char topic[80];
            int qos = -1;
            if (node_topic_build((node_out_t)k, own, strcmp(b, "-") ? b : NULL, payload, n, topic, sizeof(topic), &qos)) printf("T %s %d\n", topic, qos);
            else printf("T - -\n");
        }
    }
    return 0;
}
