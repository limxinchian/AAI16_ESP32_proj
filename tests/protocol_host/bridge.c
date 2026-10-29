// Binary stdin/stdout bridge. Test I/O is outside the allocation-free codec.
#include "aai16_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif
#define OK aai16_ResultCode_RESULT_OK
static void zero(const void *object, size_t size)
{
    const uint8_t *bytes = object;
    for (size_t i = 0; i < size; ++i) assert(bytes[i] == 0);
}
static aai16_Envelope disabled(void)
{
    aai16_Envelope e = aai16_Envelope_init_zero;
    e.has_protocol_version = e.has_request_id = e.has_phase = true;
    e.protocol_version = 1; e.request_id = UINT64_C(0xFEDCBA9876543210);
    e.phase = aai16_Phase_PHASE_VALIDATE;
    e.which_body = aai16_Envelope_sensor_config_tag;
    e.body.sensor_config.has_sensor = e.body.sensor_config.has_enabled = true;
    e.body.sensor_config.sensor = aai16_SensorId_SENSOR_GY511;
    e.body.sensor_config.enabled = false;
    return e;
}
static void self_test(void)
{
    aai16_Envelope e = disabled(), decoded;
    uint8_t bytes[AAI16_FRAME_MAX]; size_t length;
    assert(aai16_protocol_encode(&e, bytes, sizeof(bytes), &length) == OK);
    size_t ignored = 42;
    assert(aai16_protocol_encode(&e, bytes, length - 1, &ignored) == aai16_ResultCode_RESULT_TOO_LARGE);
    assert(ignored == 0);
    assert(aai16_protocol_decode(bytes, length, &decoded) == OK);
    assert(decoded.body.sensor_config.has_enabled && !decoded.body.sensor_config.enabled);
    e.body.sensor_config.has_interval_ms = true; e.body.sensor_config.interval_ms = 100;
    assert(aai16_protocol_validate(&e) == aai16_ResultCode_RESULT_AMBIGUOUS_OPERATION);
    e = disabled();
    aai16_owner_state_t state = {true, true, true, aai16_SensorHealth_HEALTH_READY, false, 7};
    assert(aai16_config_validate_owner(&e.body.sensor_config, &state) == OK);
    state.task_running = false;
    assert(aai16_config_validate_owner(&e.body.sensor_config, &state) == aai16_ResultCode_RESULT_SENSOR_UNAVAILABLE);
    state.task_running = true; state.health = aai16_SensorHealth_HEALTH_UNINITIALIZED;
    assert(aai16_config_validate_owner(&e.body.sensor_config, &state) == aai16_ResultCode_RESULT_SENSOR_UNAVAILABLE);
    e.body.sensor_config.sensor = aai16_SensorId_SENSOR_MHZ1911A;
    assert(aai16_config_validate_owner(&e.body.sensor_config, &state) == OK);
    state.health = aai16_SensorHealth_HEALTH_UNAVAILABLE;
    assert(aai16_config_validate_owner(&e.body.sensor_config, &state) == aai16_ResultCode_RESULT_SENSOR_UNAVAILABLE);
    state.probe_read_failure = true;
    assert(aai16_config_validate_owner(&e.body.sensor_config, &state) == OK);
    e.body.sensor_config.has_enabled = false; e.body.sensor_config.has_interval_ms = true;
    e.body.sensor_config.interval_ms = 5000; state.enabled = false;
    assert(aai16_config_validate_owner(&e.body.sensor_config, &state) == aai16_ResultCode_RESULT_SENSOR_DISABLED);
    e.body.sensor_config.has_expected_generation = true; e.body.sensor_config.expected_generation = 6;
    assert(aai16_config_validate_owner(&e.body.sensor_config, &state) == aai16_ResultCode_RESULT_STALE_GENERATION);
    // Stable bounded objects under malformed traffic, with canaries and no
    // reliance on success for random byte strings. Run in one process.
    uint32_t random = 0xA16;
    for (unsigned n = 0; n < 20000; ++n) {
        struct { uint64_t before; aai16_Envelope value; uint64_t after; } guard;
        guard.before = guard.after = UINT64_C(0x123456789ABCDEF0);
        size_t count = n % sizeof(bytes);
        for (size_t i = 0; i < count; ++i) {
            random = random * 1664525U + 1013904223U; bytes[i] = (uint8_t)(random >> 24);
        }
        aai16_ResultCode result = aai16_protocol_decode(bytes, count, &guard.value);
        assert(guard.before == UINT64_C(0x123456789ABCDEF0) && guard.after == guard.before);
        if (result != OK) zero(&guard.value, sizeof(guard.value));
    }
}
int main(int argc, char **argv)
{
    assert(argc >= 2);
#ifdef _WIN32
    _setmode(_fileno(stdin), _O_BINARY); _setmode(_fileno(stdout), _O_BINARY);
#endif
    uint8_t input[2048], output[AAI16_FRAME_MAX]; size_t length = 0;
    aai16_ResultCode result;
    if (!strcmp(argv[1], "self")) { self_test(); return 0; }
    if (!strcmp(argv[1], "emit")) {
        aai16_Envelope message = disabled();
        result = aai16_protocol_encode(&message, output, sizeof(output), &length);
    } else {
        size_t count = fread(input, 1, sizeof(input), stdin);
        assert(!ferror(stdin));
        if (!strcmp(argv[1], "snapshot")) {
            aai16_ConfigSnapshot snapshot;
            result = aai16_snapshot_decode(input, count, &snapshot);
            if (result != OK) zero(&snapshot, sizeof(snapshot));
            if (result == OK) result = aai16_snapshot_encode(&snapshot, output, sizeof(output), &length);
        } else if (!strcmp(argv[1], "prepare")) {
            aai16_ConfigSnapshot snapshot; aai16_PersistentConfigRecord record;
            result = aai16_snapshot_decode(input, count, &snapshot);
            if (result == OK) result = aai16_record_prepare(&snapshot, UINT64_C(0x123456789), &record);
            if (result == OK) result = aai16_record_encode(&record, output, sizeof(output), &length);
        } else if (!strcmp(argv[1], "record")) {
            aai16_ConfigSnapshot snapshot; aai16_PersistentConfigRecord record;
            result = aai16_record_decode(input, count, &record, &snapshot);
            if (result != OK) { zero(&snapshot, sizeof(snapshot)); zero(&record, sizeof(record)); }
            if (result == OK) result = aai16_record_encode(&record, output, sizeof(output), &length);
        } else {
            aai16_Envelope message;
            result = aai16_protocol_decode(input, count, &message);
            if (result != OK) zero(&message, sizeof(message));
            if (result == OK) result = aai16_protocol_encode(&message, output, sizeof(output), &length);
        }
    }
    if (result != OK) { fprintf(stderr, "protocol_result=%d\n", (int)result); return (int)result; }
    assert(fwrite(output, 1, length, stdout) == length);
    return 0;
}
