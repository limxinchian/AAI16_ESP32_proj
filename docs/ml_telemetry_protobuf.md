# ML telemetry: binary Protobuf on the existing topic

Implemented 2026-10-30. `aai16/ml/motion/data` now carries binary
`aai16.Envelope.ml_prediction` encoded by the C/nanopb adapter. ML JSON publication
and the duplicate `aai16/ml/motion/protobuf` publication have been removed.
There are no JSON bytes inside the Protobuf message. Existing JSON-only ML
subscribers must use the schema decoder described below.

The subsequent [FR5 backend reporting fix](fr5_backend_reporting.md) makes the
backend field follow the compiled kernel configuration in both ML telemetry and
ModelStatus. That guide contains the latest optimized/ANSI build results and
manual ANSI hardware procedure; the verification table below records the earlier
ML telemetry migration.

**Hardware status:** host tests and the ESP-IDF build passed. No ESP32-S3 flash,
storage erase, device/broker connection or MQTTX setting change was performed.
Physical inference, timing, activation and power-bank validation remain manual.

## Wire contract and field preservation

The existing `protocol/aai16.proto` is sufficient: **Envelope body tag 21,
MlPrediction**, protocol version **1**. No fields, tags or generated bindings
changed. The schema edit only documents the topic and metadata semantics.

| Previous JSON information | Protobuf field |
| --- | --- |
| `model` version | `ml_prediction.model.version` |
| Model identity | `ml_prediction.model.family`, `.version`, `.sha256`, `.source`, optional `.bank` |
| `state`: REST / MOVING / UNKNOWN | `ml_prediction.classification`: CLASS_REST / CLASS_MOVING / CLASS_UNKNOWN |
| `status` | `ml_prediction.status`, e.g. MOTION_PREDICTION, MOTION_REQUIRES_100MS, MOTION_INIT_FAILED |
| `ready` | `ml_prediction.model_ready`, including explicit false |
| Hex-text `boot_id` | `Envelope.boot_id`, the same uint32 value |
| `first`, `last` | `ml_prediction.first_sequence`, `.last_sequence`; uint32 wrap remains valid |
| `generation` | `Envelope.generation`: sensor configuration generation captured with the result |
| `period_ms` | `ml_prediction.interval_ms` |
| `uptime_ms` | `ml_prediction.uptime_us`; divide by 1,000 for milliseconds |
| `inference_us` | `ml_prediction.inference_us` |
| `clipped` | `ml_prediction.clipped` |
| `model_epoch` | `ml_prediction.model_epoch` |

The message also preserves window start/end uptime in microseconds, two raw INT8
outputs, two dequantized logits, cumulative clipping, queue drops, window resets,
invoke failures, arena reservation/use, backend and trial flag from the existing
binary adapter. Logits are not probabilities. Heap metrics are optional in the
schema and are absent because the current ML result does not supply them.

Nonprediction statuses carry identity, epoch, readiness, interval, uptime and
cumulative diagnostics, with classification UNKNOWN. Sample range, window times,
outputs, logits, per-invoke latency and clipping are absent: they have no current
prediction to describe. The former JSON zero placeholders are not stale predictions.
The message has no request ID or config transaction phase; configuration ACKs and
model activation completions keep their existing messages/topics.

## Bounds, validation and transport

- `aai16.options` already uses static nanopb storage for all messages. Model
  version is at most 39 characters plus NUL, SHA exactly 32 bytes, raw outputs
  exactly two bytes, and logits exactly two floats for a prediction. There are
  no strings or byte arrays containing JSON.
- The generated MlPrediction maximum is 240 bytes. This publisher's envelope
  adds at most 16 bytes, fitting the unchanged 320-byte TX payload slot. A compile
  assertion guards that bound. Encoding uses local fixed buffers; nanopb malloc
  is prohibited at compile time. No application heap allocation is introduced
  during ML message processing.
- The adapter validates status, class/readiness, signed timestamps and narrowing
  before encoding. The shared codec checks identity, version syntax/termination,
  interval, finite logits, output counts, clipping <=60, arena use, chronological
  window timestamps and the 20-sample sequence span. Inference latency must not
  exceed uptime. Invalid results return `ESP_ERR_INVALID_ARG` without publication.
- Identity and epoch come from the same worker result; serialization never reads
  a later active-model snapshot. Inference/quantization, switching, window resets,
  uploads, rollback, model selection commands and configuration ACKs are unchanged.
- The existing eight-slot telemetry queue copies the encoded bytes with zero wait.
  Only the MQTT publisher task calls the SDK, using the **encoded byte length**,
  QoS **0**, Retain **false**. Embedded NUL bytes are preserved; `strlen()` is not
  used for the payload. The separate ML mailbox/second transmission is gone.
- Existing offline, queue-full, >2-second freshness and SDK failure accounting
  apply. ESP_OK means local queue acceptance, not delivery. QoS 0 is best effort;
  an offline/full-queue result is dropped and is not replayed on a side topic.

## MQTTX: decode into Pretty JSON

These are instructions for you to apply after manually installing the firmware;
the implementation did not change saved MQTTX settings. The installed MQTTX
1.13.1 receive pipeline was inspected locally and tested with captured C payloads.

1. In **Scripts → Schema**, add or replace the saved schema using the complete
   repository **`protocol/aai16.proto`**. Select **Protobuf**. The `.options` file is
   only for nanopb generation and is not imported into MQTTX.
2. Connect using your existing broker settings. Subscribe to
   **`aai16/ml/motion/data`**. The old `/protobuf` topic is no longer published.
   `aai16/model/status` and `aai16/model/upload/ack` can also be decoded with this schema.
3. Open **Use Script** and, for a telemetry-only monitor, select:

   | Setting | Value |
   | --- | --- |
   | Apply to | **Received** |
   | Function | **None** |
   | Schema Type | **Protobuf** |
   | Schema | Your saved `aai16.proto` |
   | Proto/message name | **`aai16.Envelope`** (not `aai16.MlPrediction`) |

4. In the **received-message format** selector, choose **JSON** and the formatted/
   pretty display. MQTTX decodes the binary envelope first, then formats the
   resulting JSON. Selecting JSON without the schema decoder cannot parse this
   payload. Plaintext without a decoder will show binary characters.
5. To also use `{"activate":"a"}`, `{"activate":"b"}`, `{"rollback":true}` and
   `{"status":true}` on the same demo connection, select **All**, Function
   **AAI16 Prepare Model** from `tools/protocol/mqttx_model.js`, and the same schema.
   Keep the **publish editor Plaintext** and **received display JSON**. Publish
   commands to `aai16/model/control`, QoS 1, Retain off. The Function prepares
   outgoing commands and passes decoded received telemetry through unchanged.
   Do not use the sensor-only Function for model commands.

MQTTX shares script selection through application state. While this decoder is
enabled, use subscriptions to binary topics only and disconnect other JSON-monitor
connections or temporarily unsubscribe their JSON topics. Avoid `aai16/#` in
this view: the eight sensor streams below still contain raw JSON. This is a
desktop decoding limitation, not a request to disable sensors.

MQTTX's Pretty JSON uses enum **names**, numeric uint32/fixed32 fields, decimal
**strings for uint64** (preserving timestamps above JavaScript's exact-number
range), and **base64 for bytes** such as SHA-256 and raw INT8 outputs. To compare
SHA with a hex manifest, decode the base64 first. Byte 128 represents signed -128.
An illustrative decoded fragment, not hardware evidence:

```json
{
  "protocol_version": 1,
  "boot_id": 305419896,
  "generation": 2,
  "ml_prediction": {
    "model_epoch": 1,
    "status": "MOTION_PREDICTION",
    "model_ready": true,
    "classification": "CLASS_REST",
    "first_sequence": 11,
    "last_sequence": 30,
    "interval_ms": 100,
    "uptime_us": "5000123",
    "inference_us": "123",
    "clipped": 3
  }
}
```

The complete actual message also includes model identity and the diagnostics
above. Host-generated complete examples are in
`build/protocol_client/mqttx_ml_previews.json`; their SHA/data are synthetic fixtures.

## Verification and build

| Check | Result |
| --- | --- |
| Production C MQTT adapter / SDK doubles | Passed: single binary publication on the data topic, all A/B fields, all runtime statuses, false readiness, NUL-safe length, exact-fit and undersized buffers, invalid inputs, sequence wrap, offline/full/stale/failure handling |
| C payload → Python / MQTTX-compatible decoder | Three real publisher payloads passed: A, B and unavailable status; includes uint64 timestamps above 2^53, raw bytes, enums, and receive Function compatibility |
| C/Python codec | 16 tests, 129 round trips, 20,000 malformed-wire stress iterations; required metadata, limits, invalid values, truncation, duplicate body, JSON rejection |
| MQTTX/client suite | 9 tests; existing 259 sensor and 52 model Function checks plus the new ML decode test; installed MQTTX 1.13.1 vm2 |
| Model store/runtime and clients | 35 store scenarios, 20 client tests; upload, A/B changes, short selections, retry protection, failures and reboot recovery |
| ML worker and kernels | 31 worker scenarios per model (62 runs); 1,284 windows per model/backend, exact golden checks, 10,000 repeated Invokes per backend. Existing desktop differences remain <=1 INT8 LSB with identical classes |
| Sensor/configuration/SDK regressions | MQTT/config ACK/outbox tests and sensor_host, rc522_host, ld2450_host, hcsr04p_host, mhz1911a_host passed |
| ESP-IDF 5.5.5, ESP32-S3, bundled-A boot | Build passed; application 1,136,144 bytes (`0x115610`), 73% of the 4 MiB app partition free |
| Schema generation | `tools/protocol/generate.py --check` passed; bindings unchanged |

Logs: `build/ml_telemetry_*.log`. Captured wire fixtures:
`build/mqtt_host/ml_a.pb`, `ml_b.pb`, `ml_status.pb`. No hardware success is implied.

Reproduce from the repository root with host GCC/CMake/Ninja on PATH and the
existing isolated Python environments:

```powershell
$protocolPython = '.\build\protocol_python\Scripts\python.exe'
& $protocolPython tools/protocol/generate.py --check
& $protocolPython tests/protocol_host/run.py
& $protocolPython tests/mqtt_host/run.py --idf-path D:/ESP/.espressif/v5.5.5/esp-idf
& $protocolPython tests/protocol_client/run.py --mqttx-asar D:/ESP_MQTTX/resources/app.asar
cmake --build build/motion_firmware_host -j4
& build/motion_firmware_host/test_upload.exe build/model_b_package/model.package
& $protocolPython tests/model_upload_host/test_client.py
& tools/ml_training/.venv/Scripts/python.exe tests/motion_host/run.py --build-dir build/motion_firmware_host --model a
& tools/ml_training/.venv/Scripts/python.exe tests/motion_host/run.py --build-dir build/motion_firmware_host --model b
```

Build only, from the configured **ESP-IDF 5.5.5 terminal**:

```powershell
idf.py --no-ccache -B build -D MOTION_BOOT_MODEL=A build
```

Manual hardware acceptance: observe A REST/MOVING and disabled/unavailable status;
activate stored B, return to A, and rollback using short commands. Check identities,
epoch, sample ranges, config generation, timing and explicit readiness in decoded
telemetry alongside ModelStatus. Confirm that `/data` contains binary wire bytes,
there is no `/protobuf` duplicate, and configuration ACKs/sensors remain functional.
Measure physical sensor timing and inference continuity on the ESP32-S3. This run
performed none of those physical checks.

## Remaining JSON sensor telemetry (separate work)

These existing publishers still format JSON and were deliberately not migrated:

| Sensor | Topic |
| --- | --- |
| DHT22 | `aai16/sensors/dht22/data` |
| GY-511 | `aai16/sensors/gy511/data` |
| RC522 card event | `aai16/sensors/rc522/event` |
| LD2450 | `aai16/sensors/ld2450/data` |
| HC-SR04P | `aai16/sensors/hcsr04p/data` |
| MH-Z1911A | `aai16/sensors/mhz1911a/data` |
| BME280/BMP280 | `aai16/sensors/bme280/data` |
| MAX6675 | `aai16/sensors/max6675/data` |

The ML migration is complete; the project as a whole still needs those sensor
telemetry migrations to meet the Tribunal's all-Protobuf wire requirement.
Existing JSON examples in the older Phase 3/4B ML documents describe historical
firmware and are superseded by this contract.

## Files changed in this step

- `components/mqtt_manager/mqtt_manager.c`: encode ML into the existing queue,
  remove JSON and the duplicate mailbox/topic, propagate invalid-result errors.
- `components/mqtt_manager/include/mqtt_manager.h`: document the binary API.
- `components/aai16_protocol/aai16_protocol.c`: reject latency exceeding uptime.
- `protocol/aai16.proto`: message/topic metadata comments only.
- `tests/mqtt_host/test_mqtt.c`, `test_motion_protocol.inc`, `run.py`: capture and
  verify production publications, field preservation, bounds, invalid values and
  transport behavior; Python decoding fixtures.
- `tests/protocol_host/run.py`: ML codec limits, metadata and malformed messages.
- `tests/protocol_client/run.py`, `mqttx_ml_test.cjs`: Pretty JSON and Model Function
  receive compatibility against captured C payloads.
- `docs/ml_telemetry_protobuf.md`, `docs/model_activation_phase5b.md`: current
  contract, decoding instructions, results and corrected demo topic.
