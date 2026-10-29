# Protobuf migration Stage 1

Historical Stage 1 report. [Stage 2](protobuf_stage2.md) now implements the live
Protobuf configuration/ACK path and supersedes the JSON configuration and
publisher-workflow statements below. The shared schema/tooling remain compatible.

Stage 1 adds a shared, bounded C protocol component and matching Python tools.
The live firmware still uses its existing JSON paths. No sensor driver, task,
MQTT subscription/topic, ML model, partition or persistent setting was changed.
`main/CMakeLists.txt` adds a build dependency only; no runtime caller uses the
new component yet. This stage is not final Tribunal compliance or a model
upload implementation.

## Requirements and baseline

The authoritative Week 8 decisions were supplied by the team during this task:
keep task/driver ownership; preserve generation and phase-tagged transaction
semantics; validate first in the Config Manager and again in the device owner;
reject invalid input without changing working settings; measure bounded wire
and decoded sizes; preserve GY-511 100 ms sampling; use versioned committed
configuration with integrity and recovery; migrate incrementally and test each
step. The AAI16 section of the course's `Change Requirement 2.docx` was also
inspected and agrees with these requirements. That course document is outside
this repository and is not a build dependency.

The inspected executable baseline is narrower than the complete Tribunal
design: its MQTT command is one enable/disable or interval operation, its JSON
ACK reports applied/rejected, and its internal generations protect acquisition
and mailboxes. It does not yet transport request IDs or transaction phases.
The foundation represents the required identities/phases without claiming the
existing JSON adapter already implements them.

The following are conservative **Stage 1 proposals**, not additional approved
Tribunal decisions: wire enum numbers, phase numbers, strict unknown-field
rejection, 512-byte upload chunks, 64 KiB model/72 KiB package admission bounds,
two possible bank indices per family, and the persistent record CRC format.
They can be reviewed before Stage 2 consumes the schema. Runtime model upload,
bank layout, activation policy and recovery selection remain later work.

## Files and dependency pinning

| Path | Purpose |
| --- | --- |
| `protocol/aai16.proto` | One versioned schema for all required interfaces |
| `protocol/aai16.options` | Static nanopb storage and every variable-field bound |
| `components/aai16_protocol/include/aai16_protocol.h` | Bounded public C API, C++ compatible |
| `components/aai16_protocol/aai16_protocol.c` | Wire checks, encode/decode, common/owner validation, in-memory record integrity |
| `components/aai16_protocol/generated/` | Checked-in nanopb C bindings |
| `components/aai16_protocol/vendor/nanopb/` | Unmodified nanopb 0.4.9.2 runtime, generator subset, license and hashes |
| `components/aai16_protocol/CMakeLists.txt` | ESP-IDF integration, UTF-8 validation and bounded nesting |
| `components/aai16_protocol/.gitattributes` | Preserve exact upstream bytes across Windows checkouts |
| `main/CMakeLists.txt` | Build dependency only |
| `tools/protocol/requirements.txt` | Pinned isolated Python tooling |
| `tools/protocol/generate.py` | Deterministic generation / stale-binding verification |
| `tools/protocol/generated/aai16_pb2.py` | Matching checked-in Python bindings |
| `tools/protocol/config_cli.py` | Offline binary/hex output or explicit MQTT publication |
| `tests/protocol_host/bridge.c`, `run.py` | Real C/Python interoperability and validation tests |

Nanopb is pinned by upstream tag `nanopb-0.4.9.2`, archive SHA-256 and individual
file hashes in `vendor/nanopb/UPSTREAM.json`. The runtime has no local patches.
Generation checks these hashes first. Firmware builds use the checked-in C
bindings and need neither protoc nor Python Protobuf packages. Existing
`dependencies.lock` and managed components are unchanged.

Python tooling pins grpcio-tools/grpcio 1.71.2, protobuf 5.29.5, paho-mqtt 2.1.0
and setuptools 80.9.0. Use a separate environment, not the ML or ESP-IDF Python
environment. The tested host was Python 3.12.10 and the firmware target is
ESP-IDF 5.5.5 / ESP32-S3.

## Wire contract and compatibility

The schema uses proto2 optional presence. A field's value and its presence are
independent: `enabled=false` encodes an explicitly present false value;
omitting `enabled` is not a disable operation. Nanopb exposes `has_enabled`.
No new required Protobuf fields are used; application validators enforce the
mandatory fields for each message kind.

One complete `Envelope` is one MQTT payload, with no extra length prefix.
The MQTT layer already frames messages; fragment reassembly belongs to Stage 2.
Persistent records are standalone `PersistentConfigRecord` messages, not
network envelopes. `ConfigSnapshot` is embedded as its exact encoded bytes so
integrity verification does not depend on reserialization order.

`protocol_version` and snapshot/record versions are currently exactly 1.
Never reuse field numbers or enum values; reserve removed identifiers.
Standard Protobuf readers may ignore unknown fields and merge duplicate
singular fields. The application wrapper intentionally rejects both, including
multiple oneof bodies, so an unsupported setting cannot silently disappear.
Consequently v1 is a closed contract: additions require explicit version or
capability negotiation and coordinated validators/bounds. It is wire-compatible
with Python Protobuf, but not permissively forward-compatible with arbitrary
future fields. Call the public wrapper, not raw `pb_decode`, at trust boundaries.

Wire sensor IDs are 1..8 in order DHT22, GY511, RC522, LD2450, HCSR04P,
MHZ1911A, BME280, MAX6675. Zero means unspecified and is rejected for a device
operation. The existing C `sensor_id_t` starts at zero: Stage 2 must translate
explicitly, never cast. Likewise ML wire classes UNKNOWN/REST/MOVING are 0/1/2,
whereas the current worker uses -1/0/1.

`request_id` is a nonzero fixed64 correlation ID, not a broker packet ID.
`boot_id` and `generation` may legitimately be zero when explicitly present.
`sequence` is uint32 with existing per-sensor attempt/event semantics; model
chunks use it for chunk sequence. Generation is the sensor configuration
generation for configuration/ML messages, and the session generation for model
uploads. `model_epoch` is separate from the sensor generation.

Config requests use phase VALIDATE=1. Owner APPLY=2 and ROLLBACK=3 messages
carry a complete device profile plus request ID, boot ID and generation.
COMMIT=4 identifies a future persistence acknowledgement. A successful VALIDATE
ACK is not proof of APPLY; a broker PUBACK is neither. Preserve failure codes,
phase, request identity and generation across all owner handoffs. No phase
dispatcher or transaction state machine is introduced in Stage 1.

## Message inventory and measured sizes

Encoded values below are nanopb's generated **conservative maximum encoded
sizes**, not average traffic or MQTT packet sizes. Some maxima include field
combinations that semantic validation rejects. Decoded values are exact
`sizeof` measurements compiled for the ESP32-S3 Xtensa ABI; native host values
were independently measured and matched in this run. These figures exclude
buffer/queue metadata, MQTT headers, stacks and call temporaries.

| Message | Purpose | Max encoded bytes | ESP32-S3 decoded bytes |
| --- | --- | ---: | ---: |
| `Envelope` | Version, correlation, boot/generation/phase/sequence, one body | 634 | 776 |
| `SensorConfig` | Exactly one runtime operation | 15 | 24 |
| `SensorProfile` | Complete owner/saved configuration | 10 | 16 |
| `ApplyConfig` | Phase-tagged owner profile handoff | 12 | 20 |
| `ConfigAck` | Result, attempted operation, optional effective profile | 98 | 128 |
| `Vector3` | XYZ vector with explicit units in its parent | 15 | 24 |
| `Dht22Reading` | Temperature and humidity | 10 | 16 |
| `Gy511Reading` | Acceleration, magnetic field, acquisition jitter | 45 | 72 |
| `Rc522Event` | Card-presented UID | 12 | 14 |
| `RadarTarget` | Original slot, presence, coordinates, speed, resolution | 32 | 40 |
| `Ld2450Reading` | Three radar slots | 102 | 124 |
| `Hcsr04pReading` | Distance and echo duration | 10 | 16 |
| `Mhz1911aReading` | CO2 ppm | 6 | 8 |
| `Bme280Reading` | Temperature, pressure, optional humidity | 15 | 24 |
| `Max6675Reading` | Thermocouple temperature | 5 | 8 |
| `SensorTelemetry` | Sensor reading plus timestamps/health/interval | 136 | 192 |
| `ModelIdentity` | Family, version, SHA-256, source and optional bank | 85 | 100 |
| `MlPrediction` | Identity, class/status, window timing, logits and metrics | 240 | 320 |
| `UploadManifest` | Model/package lengths, hashes and compatibility profile | 129 | 144 |
| `ModelControl` | Begin/finish/abort, activate/rollback, status/list | 244 | 288 |
| `ModelUploadChunk` | Transfer ID, offset and bounded binary payload | 539 | 544 |
| `ModelUploadAck` | Transfer progress, next offset, result and state | 99 | 120 |
| `ModelStatus` | Active/previous identities and four stored identities | 598 | 712 |
| `ConfigSnapshot` | Eight complete, unique device profiles | 107 | 148 |
| `PersistentConfigRecord` | Versioned snapshot bytes, sequence, CRC, marker | 540 | 560 |

The maximum full envelope includes its header and nested body. Standalone body
sizes must not be used as full MQTT buffer sizes.

Admission caps are 128 bytes for a config-request envelope, 768 bytes for any
network envelope, 512 bytes for a snapshot, and 640 bytes for a storage record.
Compile-time assertions ensure generated maxima fit those caps. A simple
future single-message RX/decode/TX workspace needs 768 + 776 + 768 = **2312
bytes**, before stacks and queues. The existing 320-byte MQTT TX entry cannot
carry every new message. Do not silently enlarge all existing queue entries;
budget separate typed/config/telemetry/upload paths during integration.

There are no mutable globals, heap calls, pointers/callback fields in generated
messages, or dynamically created RTOS objects in this component. Caller-owned
buffers and fixed local objects are used. `PB_ENABLE_MALLOC` causes an error at
the public API; tests inspect allocator references in all codec/runtime objects.
The wire check and nanopb limit nesting to eight. Actual message nesting is
shallower. Full stack high-water marks and worst-case processing time still
need target measurements when the component enters a task path.

## Bounds and validation rules

| Variable field | Hard bound / permitted content |
| --- | --- |
| `ConfigAck.detail`, `ModelUploadAck.detail`, `ModelStatus.detail` | 63 UTF-8 bytes plus C terminator; embedded NUL rejected |
| Model/manifest version | 1..39 ASCII bytes from letters, digits, `-_.` |
| Every SHA-256 field | Exactly 32 raw bytes |
| Every transfer ID | Exactly 16 raw bytes |
| RC522 UID | Exactly 4, 7 or 10 bytes |
| Radar targets | Exactly three slots; unique slot IDs 1..3 |
| Prediction INT8 output | Exactly two raw bytes for a prediction |
| Prediction logits | Exactly two finite floats for a prediction; absent for status-only output |
| Upload chunk data | 1..512 bytes |
| Stored model identities | At most four; unique family/bank pairs |
| Snapshot device profiles | Exactly eight, one per sensor |
| Encoded snapshot in a record | At most 512 bytes; also decoded and semantically checked |

All enums use only the declared supported values. Configuration `sensor` is
mandatory. Exactly one of enabled/interval is mandatory; specifying both is
rejected even when enabled is false. An optional expected generation spans the
full uint32 range and is compared by the owner validator. Complete profiles
require enabled AND interval; their valid retained interval is preserved when
disabled. No gain/range/calibration/threshold settings are exposed by the
current remote configuration implementation, so none are invented here.
Future settings require typed fields, explicit bounds and an owner handler.

| Sensor | Permitted interval ms | Meaning |
| --- | ---: | --- |
| DHT22 | 2000..10000 | Acquisition period |
| GY511 | 100..10000 | Acquisition period; ML only admits 100 ms |
| RC522 | 200..2000 | Poll period, retaining removal-filter behavior |
| LD2450 | 500..10000 | Publication period; UART acquisition continues |
| HC-SR04P | 100..10000 | Acquisition; publication remains independently paced |
| MH-Z1911A | 5000..60000 | Acquisition; warm-up/probe semantics retained |
| BME280 | 500..10000 | Acquisition period |
| MAX6675 | 250..10000 | Acquisition period |

The owner validation API is pure: it reads an owner-supplied state immediately
before APPLY and does not mutate anything. Generic unavailable/uninitialized
owners reject changes. Disabled owners reject interval changes, retaining the
previous interval. MH-Z1911A's live initialized probe owner can accept changes
during warm-up and READ-stage unavailability, preserving its existing exception.
Dependency/init/task failures remain unavailable. Expected-generation mismatch
returns STALE_GENERATION. Retained-command rejection, queue capacity, current
boot identity and transaction deduplication remain transport/manager duties.

Telemetry requires matching sensor/body, finite float readings and acquisition
time <= uptime <= INT64_MAX. Presence is retained for BMP280's absent humidity.
Radar coordinates/speed are -32767..32767 and resolution 0..65535; unused slots
contain zeros. HC-SR04P distance is 2..400 cm with positive finite echo duration.
MH-Z1911A is 400..5000 ppm. Additional physical plausibility remains with each
driver; serializers do not reinterpret health or invent successful readings.
Health is optional because older producers may not have a snapshot available.

Predictions require a complete identity, status, model epoch, readiness, class,
period and uptime. A prediction requires ready=true, a 100 ms interval, a
20-sample sequence span (including uint32 wrap), ordered window times, inference
duration and two outputs/logits. Status-only output has UNKNOWN class and no
old window/output fields. Clipped count is 0..60; arena used cannot exceed
reserved when both are supplied. Other counters retain their declared uint32
range and uptime/duration fields retain the current signed-64-bit timer ceiling.
Model quantization remains model-specific in the existing engine; Stage 1 does
not move or hardcode any quantization parameters.

Model mutations require request, boot and generation metadata; upload chunks
and ACKs additionally require sequence. GET_STATUS/LIST can discover state
without a known boot. BEGIN requires family/version, 32-byte package/model
hashes, model length 1..65536, model length <= package length <=73728 and
compatibility profile 1. FINISH/ABORT carry a transfer ID, ACTIVATE carries an
exact target identity and expected model epoch, and ROLLBACK carries the
expected epoch. Irrelevant operation fields are rejected. Stored identity bank
is 0 or 1; bundled identity permits Model A only. Chunk bounds use subtraction
to prevent offset overflow. Session-size, ordering, hash verification,
authentication and model compatibility enforcement belong to later upload
implementation; schema acceptance alone does not perform them.

## Committed configuration record foundation

`aai16_record_prepare` creates a record in caller-owned RAM only. It encodes the
full validated snapshot, includes a nonzero uint64 storage sequence and applies
CRC-32/ISO-HDLC over the exact bytes:

```text
LE32(format_version) || LE64(record_sequence) || LE32(snapshot_length) || snapshot
```

CRC protects the version, record sequence and embedded configuration generation
as well as the payload. `commit_marker` must equal `0x41414916`. Decode checks
presence, lengths, format, marker and CRC, then decodes and validates all eight
profiles. Both returned objects are zeroed on failure. CRC is not authentication.

A future storage layer should write an alternate record, verify it, then make
it committed, retaining the last valid record. Select the newest fully valid
record at recovery; fall back to safe defaults if neither validates. A protobuf
marker is not an atomic flash operation. Sector/NVS layout, commit ordering,
sequence exhaustion and actual power-cut testing are explicitly outside Stage 1.
The current tests simulate truncated/corrupt record bytes, not physical power loss.

## Reproduce generation and tests

From the repository in PowerShell (Python 3.12 recommended):

```powershell
python -m venv build/protocol_python
& build/protocol_python/Scripts/python.exe -m pip install -r tools/protocol/requirements.txt
& build/protocol_python/Scripts/python.exe -B tools/protocol/generate.py
& build/protocol_python/Scripts/python.exe -B tools/protocol/generate.py --check
& build/protocol_python/Scripts/python.exe -B tests/protocol_host/run.py `
  --cc C:/msys64/ucrt64/bin/gcc.exe --cxx C:/msys64/ucrt64/bin/g++.exe `
  --xtensa-cc C:/Espressif/tools/xtensa-esp-elf/esp-14.2.0_20260121/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc.exe
. 'C:/Espressif/tools/Microsoft.v5.5.5.PowerShell_profile.ps1'
idf.py build
```

Compiler paths are examples from the tested installation. Omitting `--xtensa-cc`
runs native tests only and does not produce target ABI size evidence.
`build/protocol_host/results.json` records sizes, comparisons and test results.
The generator verifies the vendor pin, compiles the same schema for Python/C,
and compares checked-in generated bytes in `--check` mode. It does not write
to vendor sources or install packages in ESP-IDF's environment.

The tested suite has 14 test groups and 123 Python/C round trips, plus 20,000
bounded malformed-decode stress iterations. It covers all sensors and model
operations, enabled true/false/absence, exact interval boundaries and invalid
values, complete profiles, phase ACKs, maximum strings/arrays/chunks, sequence
wrap, missing boot/generation/sequence, duplicate fields/oneofs, invalid bools,
unknown fields/enums, malformed varints, uint32 overflow, truncation, invalid
UTF-8/NUL, oversized payloads, record corruption and missing/duplicate profiles.
It also checks C++ linkage, all variable-field bounds, no allocator symbols in
codec objects and equivalence to the existing firmware's interval table.

A local synthetic MQTT peer tests the Python publisher with binary QoS1,
retain=false, a real C decode, broker PUBACK, retained/stale application ACKs
and a matching APPLY ACK. It never contacts the device or configured broker.
Existing sensor and MQTT/installed-SDK host regression suites also passed.
The complete ESP-IDF 5.5.5 firmware build passed with the new component compiled.
Hardware latency, sensor interference, stack high-water marks and final runtime
memory budgets remain Stage 2/later measurements.

For example, GY-511 disable is 19 Protobuf bytes including version/request/phase,
versus 34 bytes for the current compact JSON command (44.12% smaller). Equivalent
JSON including the new metadata is 89 bytes (78.65% smaller). These are payload
measurements, not MQTT/TCP/TLS packet savings, and do not imply smaller decoded
C structures. The report records both comparison baselines for every sensor.

## Configuration workflow now and in Stage 2

Offline, generate and inspect a disable or enable command:

```powershell
& build/protocol_python/Scripts/python.exe tools/protocol/config_cli.py gy511 --enabled false --request-id 1
& build/protocol_python/Scripts/python.exe tools/protocol/config_cli.py gy511 --enabled true --request-id 2
& build/protocol_python/Scripts/python.exe tools/protocol/config_cli.py gy511 --interval-ms 100 --request-id 3
```

The tool prints payload bytes as hex and performs no network operation by
default. `--output build/protocol_host/disable.pb` saves the actual binary.
Enable and interval are separate commands. When moving to a live Protobuf
adapter, wait for the enable APPLY ACK before sending an interval change to a
previously disabled sensor. `--expected-generation` is optional; zero is valid.

For a separate test subscriber on a local broker:

```powershell
& build/protocol_python/Scripts/python.exe tools/protocol/config_cli.py gy511 `
  --enabled false --request-id 1 --host localhost --topic aai16/protocol-test/config
```

No firmware topic or subscription is changed by this example. Credentials, if
needed, come from `AAI16_MQTT_USERNAME` and `AAI16_MQTT_PASSWORD` environment
variables; the tool never reads project credential headers. TLS is available
through `--tls --port 8883` and optional `--ca-file` with normal verification.

The current live firmware still expects JSON. Do not expect a Protobuf command
on `aai16/config/set` to work until Stage 2. After its adapter is implemented:

```powershell
& build/protocol_python/Scripts/python.exe tools/protocol/config_cli.py gy511 `
  --enabled false --host YOUR_BROKER --topic aai16/config/set `
  --ack-topic aai16/config/ack
```

This future command preserves existing topic names and requests QoS1,
retain=false. The CLI distinguishes broker delivery from the matching
request-ID/boot/sensor APPLY or rejection ACK. It does not automatically retry
configuration transactions; idempotent retries belong to Stage 2's manager.
Python is the tested binary workflow. MQTTX can be used as a subscriber to
inspect traffic, or to send the emitted bytes where its installed version
supports raw hex payloads; typing the hex characters as UTF-8 text is incorrect.
No claim is made that a particular MQTTX version supports this proto2 schema UI.

Stage 2 should replace only the MQTT configuration/ACK adapters first:

1. Reassemble complete bounded MQTT payloads and retain the current retained
   command rejection before decoding.
2. Call `aai16_protocol_decode`; map the wire sensor ID explicitly to the current
   C ID. Queue only validated commands and preserve FIFO/backpressure semantics.
3. Preserve request ID, boot, generation and phase through owner handoffs. The
   current notification word carries only an operation/value: use a bounded
   sidecar/typed handoff within existing task ownership rather than losing this
   metadata. Resolve its exact representation during Stage 2.
4. Have the acquisition owner validate current availability/disabled policy and
   expected generation immediately before APPLY, after older commands. Keep
   device-specific constraints and the MH-Z1911A probe exception.
5. Commit registry/schedule/mailbox/ML generation changes in the same order as
   today, then encode the applied ACK from the owner result. Repeated operations,
   preserved intervals and enable scheduling keep their existing semantics.
6. Add stale-generation, failed-APPLY and rollback tests using actual transaction
   paths, and measure RAM/stack/timing with all eight sensors and 100 ms GY-511.

Telemetry, committed storage and model protocols can then adopt the same
schema through separate adapters. Their future operation must never be inferred
from the mere existence of Stage 1 message definitions.
