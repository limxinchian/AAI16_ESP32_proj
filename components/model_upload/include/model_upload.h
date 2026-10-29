#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#define MODEL_CONTROL_TOPIC "aai16/model/control"
#define MODEL_CHUNK_TOPIC "aai16/model/upload"
#define MODEL_ACK_TOPIC "aai16/model/upload/ack"
#define MODEL_STATUS_TOPIC "aai16/model/status"
#ifdef __cplusplus
extern "C" {
#endif
// Copies the response to the existing MQTT publisher; never calls the MQTT SDK.
typedef bool (*model_upload_reply_fn)(bool status, const uint8_t *, size_t);
bool model_upload_start(uint32_t boot_id, model_upload_reply_fn);
// MQTT callback: bounded fragment assembly + zero-wait queue send only.
void model_upload_receive(const char *topic, size_t topic_length, const void *data,
                          int length, int offset, int total, int message_id, bool retained);
void model_upload_disconnected(void);
#ifdef __cplusplus
}
#endif
