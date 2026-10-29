# Stage 2: live Protobuf configuration and APPLY/ACK

The [automatic configuration workflow](protobuf_config_workflow.md) now adds an
MQTTX Custom Function and persistent client IDs/saved-request retries. It
supersedes this report's earlier client-ID and MQTTX workflow guidance without
changing the Stage 2 firmware or schema.

The two configuration topics now carry binary `aai16.Envelope` messages using
the unchanged Stage 1 v1 schema and nanopb 0.4.9.2. JSON configuration is rejected
without fallback. This document supersedes Stage 1's description of live JSON
configuration. Model upload, switching and persistent configuration remain future work.

## Topics and supported operations

| Topic | Envelope body | Metadata | Delivery |
| --- | --- | --- | --- |
| `aai16/config/set` | `sensor_config` | version 1, nonzero request ID, `PHASE_VALIDATE` | CLI QoS 1, retain false |
| `aai16/config/ack` | `config_ack` | version, original request ID, current boot ID, sensor generation, phase, result | QoS 1, retain false |

Specify a sensor and exactly one of `enabled` or `interval_ms`. Explicit
`enabled=false` is preserved. Optional preconditions are `Envelope.boot_id` and
`sensor_config.expected_generation`. Request-level `Envelope.generation` and
`sequence` are unsupported on this topic. The current implementation supports
enable/disable and intervals only; unknown calibration/register settings and
other envelope bodies are rejected. Local sensor IDs are explicitly mapped to
wire enums rather than cast.

| Sensor | Interval range, ms | Meaning |
| --- | ---: | --- |
| DHT22 | 2000–10000 | Acquisition period |
| GY-511 | 100–10000 | Acquisition; default remains **100 ms** |
| RC522 | 200–2000 | Poll period |
| LD2450 | 500–10000 | Publication; disabled UART still drains |
| HC-SR04P | 100–10000 | Acquisition; MQTT cadence remains 1000 ms |
| MH-Z1911A | 5000–60000 | Acquisition; original warm-up remains |
| BME280 | 500–10000 | Acquisition period |
| MAX6675 | 250–10000 | Acquisition period |

GY-511 intervals other than 100 ms retain the ML worker's existing non-100-ms
status. Restoring 100 ms starts fresh windows. Model A, tensor arena, preprocessing
and inference backend are unchanged.

## Validation, handoff and ACK semantics

1. MQTT ingress assembles at most 128 bytes, checking topic, Retain, total length,
   message ID, contiguous offsets and fragment lengths/pointers. Continuations
   may omit the topic, including after an initial topic-only event. A partial
   frame expires two seconds after its first fragment. Ingress and app_main's
   normal approximately 100-ms polling service expiry. A disconnect or new first
   fragment abandons the previous partial frame.
2. The strict Stage 1 decoder rejects malformed varints/types, unknown/duplicate
   fields, invalid Booleans, missing/ambiguous operations, unsupported versions
   and out-of-range fields before FIFO admission. The config-specific wrapper
   additionally rejects bodies/metadata not supported on this topic.
3. Valid requests enter the existing four-entry FIFO with zero wait. app_main
   reserves one sidecar for the sensor before its no-overwrite notification. If
   notification fails, it cancels the sidecar and leaves the FIFO head pending.
   The owner takes the full request from the sidecar; the notification word is
   only a wakeup/value hint. No successful ACK is generated on admission.
4. The owner rechecks bounds and generation before any mutation, then applies its
   existing device-specific policy. Disabled sensors reject intervals; MH-Z1911A
   retains its warm-up/recoverable-read-failure exception. No task/driver or
   physical bus architecture is changed.
5. After committing status, schedule and publication gates, the owner completes
   the transaction. Success increments that sensor's generation once and sends
   `PHASE_APPLY / RESULT_OK`. Owner failures send APPLY with a failure result and
   unchanged generation. GY-511 passes the same assigned generation to ML.

Early failures, including an absent owner, use `PHASE_VALIDATE`. Successful ACKs
echo `requested`, preserving optional presence. The optional `effective` complete
profile is not populated in this stage. No COMMIT/persistence success is claimed.
Rejection leaves prior settings intact; configuration rollback is not a new
operation in this stage.

Structurally valid but semantically invalid messages with a nonzero request ID
receive a correlated VALIDATE failure. The new decoder returns only that ID on
failure and zeros the decoded message. These ACKs omit sensor/requested and use
generation 0 because there is no validated sensor context. Malformed, oversized,
incomplete or otherwise uncorrelatable messages are dropped without inventing an
ACK identity; clients time out. Retained commands are ignored. Routine diagnostics
report decode failures, fragment rejections/expiry and the request-ID high-water mark.

## Retry policy and implementation choices

The following are conservative Stage 2 choices, **not additional approved
Tribunal decisions**:

- Generation is per sensor, starts at zero per boot and increments on each
  successful new request, including a repeated enable value. Repeated enable
  does not postpone acquisition. Generation wrap is rejected.
- Sixteen request records are retained. Pending records cannot be evicted. An
  identical pending duplicate waits for the original outcome; an identical
  completed duplicate replays its original ACK without applying again. Its old
  generation may have been superseded by subsequent commands.
- Reusing an ID with changed sensor, operation, value or preconditions returns
  `RESULT_BUSY` without replacing the original transaction.
- **New request IDs must increase across all sensors/publishers within a boot.**
  This lets an evicted ID fail closed instead of being applied twice: uncached
  IDs at or below the high-water mark return `RESULT_STALE_GENERATION`. The CLI
  generates `time.time_ns()` IDs. Coordinate concurrent publishers and clock
  corrections; arbitrary random IDs from older Stage 1 tooling are unsuitable.
- IDs are consumed on admission, including a subsequent FIFO-full rejection.
  Retry a lost ACK using the same ID, value and optional preconditions. After an
  actual rejection, use a new increasing ID to attempt the operation again.
- Once a receipt is evicted its outcome cannot be recovered. It is rejected,
  never reapplied. History is volatile. Boot-bound requests reject after reboot;
  requests without a boot precondition intentionally target the receiving boot.

Receipts are cached before TX admission. An offline/full TX queue or failed SDK
enqueue does not undo an applied setting. Retrying an identical cached request
recovers the ACK. Broker PUBACK is not application success. Shared topics retain
the existing single-device/ACL-isolation assumption; multiple devices should be
targeted with a known boot ID until device-specific routing is introduced.

## Python commands

From the repository root in PowerShell, use the isolated Stage 1 environment:

```powershell
$py = '.\build\protocol_python\Scripts\python.exe'
# If it has not already been prepared:
# py -3.12 -m venv build/protocol_python
# & $py -m pip install -r tools/protocol/requirements.txt
$broker = 'YOUR_BROKER_HOSTNAME'
$env:AAI16_MQTT_USERNAME = 'YOUR_MQTT_USERNAME'
$env:AAI16_MQTT_PASSWORD = 'YOUR_MQTT_PASSWORD'

& $py tools/protocol/config_cli.py gy511 --enabled true --host $broker --tls --port 8883
& $py tools/protocol/config_cli.py gy511 --enabled false --host $broker --tls --port 8883
# Re-enable before updating the interval:
& $py tools/protocol/config_cli.py gy511 --enabled true --host $broker --tls --port 8883
& $py tools/protocol/config_cli.py gy511 --interval-ms 100 --host $broker --tls --port 8883
& $py tools/protocol/config_cli.py dht22 --interval-ms 5000 --host $broker --tls --port 8883

# Offline encoding, no broker/device access:
& $py tools/protocol/config_cli.py gy511 --enabled false --output build/disable.pb
```

For a local plaintext broker omit `--tls` and use port 1883. Private CAs can use
`--ca-file path/to/ca.pem` with TLS. Default topics are the production topics;
`--topic` and `--ack-topic` still allow isolated tests. The CLI subscribes before
publishing, never retains commands, waits for APPLY/rejection, and prints the
request ID, boot ID, phase, generation and result.

For conditional updates add `--boot-id 0x... --expected-generation N`, using
observed values for that boot and sensor; success must return `N+1`. Retry a
timed-out command with `--request-id THE_ORIGINAL_PRINTED_ID` and the same values
and preconditions. There is no automatic retry. Stale-boot rejection displays
the current boot ID instead of silently becoming a client timeout.

Python is the tested workflow. MQTTX was not hardware-validated here. If the
installed version supports proto2 schemas, import `protocol/aai16.proto`, choose
`aai16.Envelope` for both topics, preserve optional false and turn Retain off.
An editor may show JSON while encoding binary; plain JSON publication is invalid.
Check fixed64 handling: JavaScript numbers cannot exactly represent nanosecond
IDs. Prefer Python for conditional writes and retries.

## Memory and compatibility

Measured with the ESP32-S3 ABI and installed ESP-IDF v5.5.5:

| Item | Bytes / limit |
| --- | ---: |
| Config receive cap | 128 |
| Schema SensorConfig encoded maximum / decoded size | 15 / 24 |
| Schema ConfigAck body encoded maximum / decoded size | 98 / 128 |
| Conservative ACK-envelope allocation bound | 134 (36 overhead + 98 body) |
| Full shared Envelope decoded workspace | 776 |
| `mqtt_config_update_t` | 40, previously 12 |
| Four-entry FIFO item storage | 160, previously 48 (+112) |
| History, generation counters and owner sidecars | 1400 static |
| Fragment state, including 128-byte payload | 168 static |
| Additional counter and spinlock | 4 + 8 static |
| Total new static control state | 1580 |
| Shared TX queue | Unchanged: 8 slots, 320-byte payload each |
| RFID queue / SDK outbox total | Unchanged: 8 slots / 5120 bytes |
| Installed SDK default buffer size | Unchanged: 1024 bytes |

The schema maxima allow combinations the application validator rejects. With
the live v1 configuration rules, a request is at most 31 bytes and an emitted
ACK at most 42 bytes (fully populated optional preconditions and MH-Z1911A's
maximum interval). The larger checked allocation bounds remain for schema
compatibility. ACK lengths are passed explicitly; binary data never uses strlen.

Target `-fstack-usage` measured individual frames of 1152 bytes for ACK
serialization, 976 for complete-message handling, 192 for the event callback,
and 160–304 for reader loops. These exclude the complete nested call chain.
Stacks remain app_main 3584 bytes, readers 4096 and MQTT SDK 6144; hardware
high-water measurements under configuration bursts are still required.

Codec and transaction objects have no malloc/calloc/realloc/free references.
Application decoding, validation, assembly, history, handoff and ACK encoding use
bounded storage. No SDK call, queue wait, logging or encoding occurs under the
control spinlock. Existing RTOS queue allocation at initialization and SDK
transport/outbox allocation remain outside this processing guarantee.

The installed SDK's actual fragment delivery code was inspected, including
topic-less continuations; see also [Espressif's MQTT API](https://github.com/espressif/esp-mqtt/blob/master/include/mqtt_client.h).
No dependency pin, schema field number, sdkconfig or partition changed.

## Files and tests

- `components/mqtt_manager/config_transactions.{c,h}`: bounded transactions,
  sidecars and fragment assembly. Manager source/header/CMake: binary adapters,
  diagnostics and protocol dependency.
- `components/aai16_protocol/aai16_protocol.c` and public header: config-specific
  decoding with safe failure identity. Generic decoding and generated bindings
  remain compatible with Stage 1.
- `main/main.c`, `main/sensor_config.h`, six reader files: metadata handoff,
  owner validation and GY/ML generation synchronization.
- `tools/protocol/config_cli.py`: production defaults, timestamp IDs and ACK
  correlation. Pinned Python dependencies are unchanged.
- Host fixtures/runners: replace obsolete JSON configuration assertions with
  binary checks, retain telemetry assertions, and adapt reader notification
  fixtures to sidecars. `test_config_protocol.inc` covers live MQTT paths;
  `config_stub.c` supports actual owner scheduling tests.

Validation performed locally, without the project broker or ESP32:

- Generation verification, C/C++ compilation, 14 schema test groups with 123
  round trips, boundary/presence checks and 20,000 malformed inputs.
- 32 additional Python → actual C MQTT callback/FIFO/sidecar/ACK → Python cases
  across all eight sensors (enable, disable, minimum and maximum intervals).
- Live C tests for owner ACKs, no queue-success ACK, FIFO/cache saturation,
  stale/duplicate/conflicting/evicted requests, generation wrap, boot mismatch,
  dropped ACK recovery, invalid/unsupported fields and JSON rejection.
- Every fragment split boundary, topic-only first event, bad lengths/offsets/
  IDs/topics, retained/oversized input, timeout, replacement and disconnection.
- Local Paho MQTT peer: binary QoS1 publication, retain false, subscription
  before publish, ACK correlation and APPLY versus PUBACK/VALIDATE behavior.
- Sensor status/scheduling and eight-sensor/ML regressions; RC522, LD2450,
  HC-SR04P and MH-Z1911A driver/reader suites.
- Model A: four golden vectors, 1172-input class agreement, 10,000 stable invokes
  per host backend and 21 worker scenarios. Host results do not measure ESP32
  ESP-NN assembly latency.
- ESP-IDF v5.5.5 firmware build. Logs are `build/stage2_*`.

Representative rerun commands:

```powershell
& $py tests/protocol_host/run.py --cc C:/msys64/ucrt64/bin/gcc.exe --cxx C:/msys64/ucrt64/bin/g++.exe
& $py tests/mqtt_host/run.py --cc C:/msys64/ucrt64/bin/gcc.exe --idf-path D:/ESP/.espressif/v5.5.5/esp-idf
& $py tests/sensor_host/run.py --cc C:/msys64/ucrt64/bin/gcc.exe
& tools/ml_training/.venv/Scripts/python.exe tests/motion_host/run.py
# In an ESP-IDF v5.5.5 terminal, compile only:
idf.py -DCCACHE_ENABLE=0 build
```

The protocol runner accepts `--xtensa-cc` for target sizes. Individual driver
runners use gcc on PATH. ML tests require the existing compiled host binaries
and separate ML environment.

## Remaining migration and hardware verification

Sensor telemetry and ML predictions remain JSON temporarily. No live model
control/upload or persistent configuration backend was added. Wi-Fi's existing
NVS behavior is unchanged. Settings and transaction history remain volatile.
Later stages must migrate the remaining specified interfaces for the final
Tribunal requirement.

After the user chooses to install the built firmware:

1. Record boot ID, free/minimum heap and task stack minima. Verify all eight
   sensors and GY-511's 100-ms acquisition/Model A predictions before commands.
2. Send enable/disable and valid intervals using Python. Verify binary owner
   ACKs, exact request ID, APPLY phase, per-sensor generation and actual cadence.
   Check disabled-interval rejection, retained periods and MH-Z1911A warm-up.
3. For GY-511, change away from 100 ms, observe the existing ML status, restore
   100 ms and wait for fresh predictions. Check generation boundaries/windows.
4. Exercise malformed/oversized/fragment fixtures in a controlled harness; check
   rejection counters and that working settings do not change. Uncorrelatable
   input must time out without a fabricated success ACK.
5. Retry pending/completed requests, conflicting IDs and stale generations.
   Disconnect around owner completion and recover the cached ACK. Reboot and
   verify boot-bound stale requests reject while default settings return.
6. Run configuration bursts with all sensors and ML active; measure GY jitter,
   missed samples, inference latency, free heap and owner/MQTT stack minima.

No flashing, commits, pushes, merges or branch changes were performed.
