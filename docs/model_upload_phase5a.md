# Phase 5A — MQTT model upload, validation and persistent staging

Implemented and host-tested on 2026-10-29. **Phase 5A has not been tested on the ESP32-S3 board or production broker.** The earlier independent A/B hardware inference tests are user-confirmed; they do not establish upload-time timing or power-loss behavior on hardware.

Model A remains the default bundled startup model. The laptop sends the actual Model B TFLite bytes plus metadata and golden vectors. A successful transfer creates a stored candidate. No running interpreter is replaced, and reboot still selects the bundled startup model. ACTIVATE and ROLLBACK return `RESULT_NOT_SUPPORTED`.

## Files in this stage

Existing, previously modified working-tree files were preserved. This list describes only Phase 5A work, not the entire uncommitted repository diff.

| File | Phase 5A change / principal entry points |
| --- | --- |
| `components/model_upload/CMakeLists.txt` | New component registration |
| `components/model_upload/include/model_upload.h` | Topic names and bounded MQTT ingress/reply API |
| `components/model_upload/upload_ingress.h` | `Ingress::feed/reset`: bounded fragment assembly, retained/offset/topic checks |
| `components/model_upload/upload_store.h` | Portable storage interface, stored-candidate metadata, transfer owner |
| `components/model_upload/upload_store.cc` | `Store::handle/tick/verify/commit/recover`: sequencing, persistence, recovery |
| `components/model_upload/model_upload.cc` | `PartitionFlash`, upload worker, `model_upload_start/receive/disconnected` |
| `components/motion_ml/include/motion_candidate.h` | External-package validation and metadata API |
| `components/motion_ml/motion_candidate.cc` | `graph_contract`, `motion_candidate_validate/active_info`; real production engine + self-tests |
| `components/motion_ml/motion_runtime.cc` | Shared TFLM execution mutex |
| `components/motion_ml/motion_ml.cc` | Guard live init/Invoke; expose model readiness |
| `components/motion_ml/include/motion_ml.h` | `motion_ml_model_ready` declaration |
| `components/motion_ml/CMakeLists.txt` | Compile candidate validation/runtime guard |
| `components/aai16_protocol/aai16_protocol.c` | `identity_validate` now accepts a bundled B identity for the existing development B build |
| `components/mqtt_manager/mqtt_manager.c` | Model subscriptions, callback handoff, separate bounded response mailbox serviced by existing publisher |
| `components/mqtt_manager/CMakeLists.txt` | Depend on staging component |
| `tools/protocol/config_cli.py` | Dispatch `model` subcommands; existing sensor syntax remains intact |
| `tools/protocol/model_package.py` | Deterministic packaging and laptop integrity checks |
| `tools/protocol/model_cli.py` | Paho transfer, IDs, retry/reconnect, status confirmation |
| `tests/model_upload_host/test_store.cc` | Real store/validator/engine with NOR flash fault injection; Python bridge |
| `tests/model_upload_host/test_client.py` | CLI/C++ round trips, real local MQTT/Paho peer, disconnect/reboot tests |
| `tests/motion_host/CMakeLists.txt` | Compile shared validator/guard and upload test executable |
| `tests/mqtt_host/run.py`, `tests/mqtt_host/test_mqtt.c` | Model routing/mailbox/ACK-reserve checks alongside existing regressions |
| `docs/model_upload_phase5a.md` | This workflow, format and verification record |

The `.proto`, generated bindings, `model_provider.h`, `model_source.cc`, production `motion_engine.cc`, preprocessing, trained artifacts, partitions, sensor readers, credentials, and telemetry formats were not changed in this stage.

## Architecture and transfer

1. MQTT callback copies/reassembles at most 768 bytes, rejects retained messages, and submits to a four-entry queue with zero wait. No flash, hashing, model validation or inference runs in this callback.
2. A priority-2 worker on core 1 decodes with `aai16_protocol_decode`. It owns all transfer state and flash operations. The existing inference worker remains priority 3.
3. GET_STATUS supplies `boot_id` and upload `generation`. BEGIN supplies a fresh transfer UUID and manifest, using that boot/generation. Acceptance advances upload generation once and prepares an inactive bank.
4. Stop-and-wait chunks are exactly 512 bytes except the final remainder. Sequence is `offset / 512`; offsets start at zero. Each successful write is read back before its ACK.
5. FINISH requires all bytes and sequence `ceil(package_size / 512)`. The worker hashes the package from flash, maps it read-only, verifies metadata/model SHA/INT8 graph, and executes all four golden vectors using the production engine and preprocessing.
6. Only after validation does it write and read back the persistent record, then write the commit marker last. `UPLOAD_STORED` confirms this commit. The client then requests status and checks stored family/version/model SHA/source/bank and unchanged active identity.
7. Responses pass through a separate four-entry bounded mailbox. Only the existing MQTT publisher calls the SDK. Model responses share the 1 KiB event allowance; the existing 4 KiB configuration ACK allowance remains reserved.

The candidate uses its own 16 KiB internal arena. It never calls init on the live engine. The pinned TFLM FC wrapper contains a global timing accumulator, so live and candidate init/Invoke are serialized briefly with a priority-inheriting mutex. **Flash erase/write operations never hold this mutex.** `inference_us` continues to measure Invoke, excluding time waiting for the mutex. The worker stack is 12 KiB; finish logs its stack low-water mark and internal heap minimum.

## MQTT topics and correlation

All four model topics use binary `aai16.Envelope`, protocol version 1. Requests and responses use QoS 1, retain false. These are the existing project's shared topic namespace; use one intended device per namespace/broker ACL.

| Topic | Direction | Body |
| --- | --- | --- |
| `aai16/model/control` | Laptop → ESP32 | `ModelControl`: BEGIN_UPLOAD, FINISH_UPLOAD, ABORT_UPLOAD, GET_STATUS, LIST |
| `aai16/model/upload` | Laptop → ESP32 | `ModelUploadChunk` |
| `aai16/model/upload/ack` | ESP32 → laptop | `ModelUploadAck` |
| `aai16/model/status` | ESP32 → laptop | `ModelStatus`; also NOT_SUPPORTED replies to activation/rollback |

`request_id` uses the existing durable automatic allocator. Each new operation/chunk gets an ID; retries preserve its exact bytes and ID. `transfer_id` is a fresh 16-byte UUID. ACKs echo request ID, transfer ID and sequence, and report current boot/upload generation. Upload generation is independent of sensor configuration generation and model epoch. GET_STATUS/LIST need a request ID; state-changing requests also need boot and generation. BEGIN/ABORT use sequence 0 in this client. FINISH uses the next chunk sequence.

The client waits for both response-topic SUBACKs after every connect. Broker PUBACK alone never means a chunk was written or a model was stored. It checks the application ACK and final status.

### Retry, error and recovery behavior

- An identical BEGIN retry returns current progress without another erase. A reused transfer UUID with a different BEGIN request/manifest is rejected. One transfer may be receiving at a time; a different BEGIN gets BUSY.
- Duplicate chunks are compared with flash. Identical bytes return the current next offset without writing again; conflicting bytes terminate the upload with CHECKSUM_MISMATCH.
- Missing/out-of-order/wrong-sized/wrong-sequence chunks and premature FINISH get OUT_OF_RANGE with next offset. The correctly identified receiving session can continue. A normal CLI transfer never pipelines chunks.
- A silently corrupted new chunk is detected by the complete package SHA at FINISH; chunk ACKs acknowledge flash read-back, not final package integrity.
- Wrong boot/generation/transfer gets STALE_GENERATION. Malformed frames with a safely decoded request/transfer correlation get a rejection ACK; uncorrelatable, oversized-frame, retained and queue-dropped messages get no invented ACK. Client timeout/retry handles absence of a response.
- ABORT and 60 seconds of session inactivity invalidate the uncommitted candidate. Timeout is reported on the next correlated request; it does not erase the currently running model. Use a new BEGIN/UUID after abort/timeout.
- ABORT is serialized with other requests. If FINISH has already committed, the response remains STORED; abort does not delete a committed candidate.
- Disconnect discards fragment assembly but preserves the receiving session until timeout. Same-byte retries resume within that window. If the process gives up, `model abort --transfer-id ...` cancels it, or wait for timeout before starting a new transfer.
- Reboot discards partial sessions. The client detects a changed boot ID, queries committed candidates, and restarts an interrupted transfer once if needed. Partial-byte resume across reboot is intentionally absent.
- FINISH retries return the stored result while that session remains the most recent session. After a reboot or ambiguous connection failure, GET_STATUS is the confirmation source.
- Storage erase/write/read/map/commit errors produce INTERNAL_ERROR. Hash, graph, metadata and golden-vector failure produce CHECKSUM_MISMATCH with a specific detail. Invalid request sizes/profile are rejected by the protocol validator before flash admission.

## Persistent layout and validity

The actual compiled partition table was inspected. Both model partitions are data partitions in the existing 16 MiB layout:

| Partition | Subtype | Address | Capacity | Bank size |
| --- | --- | --- | --- | --- |
| `model_a` | `0x40` | `0x410000` | `0x300000` (3 MiB) | `0x180000` (1.5 MiB) |
| `model_b` | `0x41` | `0x710000` | `0x300000` (3 MiB) | `0x180000` (1.5 MiB) |

The adapter checks label/subtype/address/capacity and rejects encrypted or unexpected layouts. The portable store also checks capacity/sector alignment and bounds every transfer. The app partition ends at `0x410000`; it is never an upload destination.

Each bank begins with a 4 KiB metadata sector. Its package starts at bank offset 4096. New uploads use an invalid bank, otherwise the older committed bank of the chosen family; the newest valid candidate is retained. Only the metadata sector and package-length rounded-up sectors are erased, one sector per call. Writes are at most 512 bytes. Model B's 3920-byte package needs two sector erases total.

The 256-byte commit record is little-endian:

| Offset | Content |
| --- | --- |
| 0 | Eight bytes `AAI16ST1` |
| 8 | Monotonic uint64 record serial |
| 16 | uint32 encoded manifest length, at most 132 |
| 20 | Bounded encoded `UploadManifest` |
| 152 | FINISH request ID, uint64 |
| 160 | Transfer ID, 16 bytes |
| 176 | Upload generation, uint32 |
| 180–219 | Reserved zero bytes |
| 220 | SHA-256 of bytes 0–219 |
| 252 | uint32 `0x41414916`, written last |

Recovery requires the marker and record SHA, validates the decoded manifest, rehashes the entire package, checks the model and repeats all golden tests. Incomplete/invalid banks are excluded from status. The previous valid candidate and bundled fallback remain available after a failed replacement. A power failure after a complete marker write can leave a valid candidate even if its ACK was lost; status/recovery resolves this ambiguity.

## Package format and admission contract

`model_package.py` reads the existing training manifest and checks sizes/hashes of the TFLite, quantization, preprocessing contract and golden-vector files. It performs no training or conversion. Output files are `model.package` and `upload_manifest.json`.

The deterministic Model B package is **3920 bytes**, containing the original **2384-byte** `model_int8.tflite`.

- Model SHA-256: `6ec06da5b5fe3d12fdd99328be1e8adf10bc3d78f3c7191bcb06dcd2f5279aff`
- Package SHA-256: `c9a0d97ea0f4e2e982fb7339cc513ae64a4ee42c7a461ee9191f9ed002ba5ebe`
- Version: `aai16-motion-model-b-0.1.0`
- Input scale/zero point: `0.0031422548927366734`, `12`
- Output scale/zero point: **`0.0462011881172657`, `-29`**

The format is little-endian with IEEE-754 float32:

| Header offset | Field |
| --- | --- |
| 0 | Eight bytes `AAI16MP1` |
| 8, 12 | uint32 format version 1, header length 256 |
| 16, 20, 24, 28 | uint32 family, compatibility profile 1, model length, golden count 4 |
| 32, 36, 40 | uint32 model offset 256, 16-byte-aligned golden offset, total package length |
| 44, 48, 52, 56 | Input float scale, int32 zero point, output float scale, int32 zero point |
| 60 | Four reserved zeros |
| 64 | ASCII version, NUL-terminated and zero-padded to 40 bytes |
| 104, 136 | Model SHA-256 and fixed preprocessing-contract SHA-256 (32 bytes each) |
| 168–255 | Reserved zeros |

The TFLite bytes start at 256, followed by zero alignment padding. Four 320-byte golden records follow: 60 raw float32 values at 0, 60 INT8 features at 240, two INT8 outputs at 300, two zero bytes at 302, uint32 class at 304, twelve zero bytes at 308.

Firmware limits remain 64 KiB model, 72 KiB package, 768-byte envelope and 512-byte chunk. Version must be nonempty bounded ASCII using the protocol's permitted characters and must agree with the manifest. Family, version, sizes, model digest and profile must match between package and manifest. No version ordering/anti-downgrade policy is implied.

Admission checks structural FlatBuffer validity **and** bounded semantic safety before interpreter allocation: schema 3; one graph; two FULLY_CONNECTED v4 operators; seven connected tensors; INT8 `[1,60]` input and `[1,2]` output; hidden width 16 for A or 8 for B; RELU then linear; bounded inline INT8 weights/INT32 biases; finite compatible per-channel quantization; safe shapes, indices, constant lengths and accumulator scaling. Sparse/dynamic/external-buffer/custom graph variants are unsupported. Golden tests replay the existing preprocessing, quantization, real Invoke, exact expected INT8 outputs and decoded classes. Uploaded metadata never borrows A's output quantization.

SHA-256 provides integrity, not publisher authentication. This stage relies on the configured broker's TLS, credentials and ACLs. Package signing and version security policy are not implemented.

## Build, flash once, and demonstrate

Run from the repository root. Use the existing ESP-IDF PowerShell environment, or load the installed profile:

```powershell
. 'C:\Espressif\tools\Microsoft.v5.5.5.PowerShell_profile.ps1'
idf.py --no-ccache -B build -D MOTION_BOOT_MODEL=A build
idf.py --no-ccache -B build -D MOTION_BOOT_MODEL=A -p COM5 flash monitor
```

Replace COM5 with the board's port. This installs Phase 5A firmware once. Subsequent model upload commands write the data partition over MQTT. The monitor exit shortcut is Ctrl+]. No `erase-flash` is needed; it would remove stored candidates.

In a second terminal, use the existing protocol Python environment and the same credential environment variables as configuration commands:

```powershell
$broker = 'YOUR_BROKER_HOST'
$env:AAI16_MQTT_USERNAME = 'YOUR_MQTT_USERNAME'
$mqttCredential = Get-Credential -UserName $env:AAI16_MQTT_USERNAME -Message 'MQTT broker password'
$env:AAI16_MQTT_PASSWORD = $mqttCredential.GetNetworkCredential().Password

# Prepare inspectable files from the real trained Model B artifact.
.\build\protocol_python\Scripts\python.exe tools/protocol/config_cli.py model package --model-dir models/motion_model_b --output-dir build/model_b_package

# Upload the selected package/manifest, with automatic request/transfer IDs.
.\build\protocol_python\Scripts\python.exe tools/protocol/config_cli.py model upload --package build/model_b_package/model.package --manifest build/model_b_package/upload_manifest.json --host $broker --port 8883 --tls --timeout 15 --retries 4

# Inspect stored identities and the unchanged active bundled model.
.\build\protocol_python\Scripts\python.exe tools/protocol/config_cli.py model status --host $broker --port 8883 --tls
```

Use the broker's actual port/security settings. For a private CA, append `--ca-file path/to/ca.pem`. An existing plaintext development broker uses its plaintext port without `--tls`. The pinned dependencies remain in `tools/protocol/requirements.txt`; the environment above already exists in this workspace.

For a one-command demonstration that builds the package first:

```powershell
.\build\protocol_python\Scripts\python.exe tools/protocol/config_cli.py model upload --model-dir models/motion_model_b --output-dir build/model_b_package --host $broker --port 8883 --tls
```

Abort a pending transfer using its printed UUID, or repeat status after power cycling:

```powershell
.\build\protocol_python\Scripts\python.exe tools/protocol/config_cli.py model abort --transfer-id REPLACE_WITH_32_HEX_DIGITS --host $broker --port 8883 --tls
```

GY-511 currently defaults to enabled/100 ms in the source. If changed during testing, the existing commands still work:

```powershell
.\build\protocol_python\Scripts\python.exe tools/protocol/config_cli.py gy511 --enabled true --host $broker --port 8883 --tls
.\build\protocol_python\Scripts\python.exe tools/protocol/config_cli.py gy511 --interval-ms 100 --host $broker --port 8883 --tls
```

### Expected output, not captured board results

Serial messages are interleaved with existing sensor/network logs; IDs/timestamps/bank and timings vary:

```text
ML: initializing model=A version=aai16-motion-model-a-0.1.0 bytes=3104 profile=1
ML: Model A startup golden vectors: 4/4 PASS
MODEL_UPLOAD: partition=model_a address=0x410000 bytes=3145728 bank_bytes=1572864
MODEL_UPLOAD: partition=model_b address=0x710000 bytes=3145728 bank_bytes=1572864
MODEL_UPLOAD: staging ready recovered=0 max_package=73728 chunk=512 timeout_ms=60000; active model unchanged
ML: model=A state=REST inference_us=<measured> ...
MODEL_UPLOAD: request=<id> generation=1 state=1 result=0 next=0 detail=receiving
MODEL_UPLOAD: request=<id> generation=1 state=2 result=0 next=512 detail=chunk_stored
...
MODEL_UPLOAD: request=<id> generation=1 state=2 result=0 next=3920 detail=chunk_stored
MODEL_UPLOAD: request=<id> generation=1 state=4 result=0 next=3920 detail=validated_4_of_4_stored_active_unchanged
MODEL_UPLOAD: stored model=B version=aai16-motion-model-b-0.1.0 bank=0 model_bytes=2384 golden=4/4; candidate only
MODEL_UPLOAD: candidate STORED; active unchanged; flash_max_erase_us=<measured> flash_max_write_us=<measured> stack_min_bytes=<measured> heap_min_bytes=<measured>
ML: model=A state=MOVING inference_us=<measured> ...
```

On the next reboot, expect `recovered model=B ... golden=4/4; candidate only` and a recovered count of 1, while A initializes and runs. No live Model B predictions are expected merely from uploading it.

Decoded CLI/MQTT output:

```text
transfer_id=<32 hex digits> boot_id=0x<boot> package_bytes=3920
request_id=<id> generation=1 UPLOAD_ACCEPTED RESULT_OK next_offset=0 receiving
request_id=<id> generation=1 UPLOAD_RECEIVING RESULT_OK next_offset=512 chunk_stored
Upload 512/3920 bytes (13%)
...
Upload 3920/3920 bytes (100%)
request_id=<id> generation=1 UPLOAD_STORED RESULT_OK next_offset=3920 validated_4_of_4_stored_active_unchanged
STORED confirmed: aai16-motion-model-b-0.1.0 bank=0 sha256=6ec06da5b5fe3d12fdd99328be1e8adf10bc3d78f3c7191bcb06dcd2f5279aff; active unchanged; NOT activated
```

Status reports `active.family=MODEL_A`, `active.source=SOURCE_BUNDLED`, epoch 0, plus a stored B identity/source/bank. State is RUNNING only when the live engine reports ready; an initializing/failed engine reports UNAVAILABLE. Candidate presence itself never establishes live readiness.

## Automated verification completed

| Check | Actual result |
| --- | --- |
| Native upload store/real INT8 validator | 23 scenarios passed, including ten torn-write positions, completed-marker/failed-completion recovery, corrupted records/data, invalid graph/quantization, missing/duplicate/out-of-order chunks, abort/timeout, storage failures and concurrent A inference |
| Python upload client | 12 tests passed: actual artifact/wire/store, all four banks/status, retries, reboot recovery and wrong A output quantization for B |
| Real Paho/local MQTT peer | Included in those 12: complete CLI upload and TCP loss after chunk storage before ACK; exact-byte retry succeeded against real C++ storage code with simulated NOR flash |
| MQTT manager + installed MQTT SDK host harness | Passed existing sensor/ACK/outbox regressions and new model mailbox/routing/reconnect checks; 32 Python/C configuration/APPLY round trips |
| Sensor hosts | `sensor_host`, `rc522_host`, `ld2450_host`, `hcsr04p_host`, `mhz1911a_host` passed |
| Protocol + original configuration CLI | 14 protocol tests and 7 CLI tests passed; generated bindings check passed |
| ML Python regressions | 35 training/artifact tests and 5 bundle tests passed |
| Real A/B host inference | 1284 windows per model through reference and ESP-NN ANSI; classes all agree with desktop; four goldens exact; 10000 repeated invokes per model/backend; 42 worker fault scenarios passed |
| Desktop/TFLM numerical limit | A has 4 differing output elements, B 5, each at most 1 INT8 LSB; host reference and ANSI outputs agree exactly |
| ESP-IDF 5.5.5 builds | Complete A-default and separate B-development builds passed for ESP32-S3; each image is 1,129,040 bytes (`0x113a50`), with 73% of the 4 MiB factory partition free |

Host tests use real firmware algorithms with simulated flash and scheduling; local MQTT tests do not exercise the ESP-MQTT callback/flash adapter on a board. ESP32-S3 optimized instructions are built, not executed by host tests. No hardware upload, endurance, power-cut, flash stall or timing result is claimed.

Reproduce the new tests after configuring the existing host build (GCC, CMake and Ninja on PATH):

```powershell
cmake -S tests/motion_host -B build/motion_firmware_host -G Ninja -DCMAKE_C_COMPILER=C:/msys64/ucrt64/bin/gcc.exe -DCMAKE_CXX_COMPILER=C:/msys64/ucrt64/bin/g++.exe -DIDF_PATH=D:/ESP/.espressif/v5.5.5/esp-idf -DPYTHON=C:/Espressif/tools/python/v5.5.5/venv/Scripts/python.exe -DMOTION_BOOT_MODEL=A
cmake --build build/motion_firmware_host -j4
.\build\protocol_python\Scripts\python.exe tools/protocol/config_cli.py model package --model-dir models/motion_model_b --output-dir build/model_b_package
.\build\motion_firmware_host\test_upload.exe build/model_b_package/model.package
.\build\protocol_python\Scripts\python.exe tests/model_upload_host/test_client.py
```

The existing regression runners and logs are under `tests/*_host/run.py`, `tests/protocol_client/run.py`, `build/phase5a_*.log`, and `build/motion_firmware_host/results_{a,b}.json`. The development B build check is `idf.py --no-ccache -B build/model_b_firmware -D MOTION_BOOT_MODEL=B build`; this remains a build-time test option.

## Hardware acceptance and Phase 5B

First capture a baseline at 100 ms with A predicting REST/MOVING. Then upload B several times during stationary and moving intervals, and issue normal configuration commands during a transfer. Keep GY at 100 ms for the continuity comparison; changing its configuration intentionally resets the ML window. Confirm the stored ACK/status and unchanged active identity. Capture sensor acquisition timestamps/sequence/jitter, GY health overruns/max jitter, ML continuity/drop/reset counters, upload flash timing, stack/heap minima, and configuration APPLY ACKs.

**Cache suspension remains a hardware acceptance risk.** The current sdkconfig has flash auto-suspend disabled. IDF flash operations may suspend both CPUs' caches. Low task priority alone cannot prevent this. The adapter uses one 4 KiB erase per call, writes no more than 512 bytes, yields a tick after each operation, and logs operations over 20 ms. Existing reader jitter/overrun and ML discontinuity detection remain intact. These measures bound requested work; they cannot guarantee uninterrupted sampling on an unmeasured flash chip. Verify no upload-caused gap over the ML 150 ms contract, no new inference discontinuity/overflow, and acceptable acquisition jitter. If these fail, investigate supported flash auto-suspend or acquisition-aware flash scheduling before accepting Phase 5A. Auto-suspend was not enabled blindly because support depends on the actual flash chip.

Next disconnect/reconnect Wi-Fi during chunks, abort, timeout, and power-cycle during BEGIN, a chunk and FINISH. Incomplete candidates must remain absent; committed candidates must recover; A must keep working or restart normally. Check a failed replacement leaves the newest prior valid candidate available. Also test bad package/model SHA, incompatible metadata and corrupted goldens using a deliberate test sender (the standard CLI refuses obvious invalid packages before sending).

Phase 5B must add stored-model mapping lifetime/ownership, protection of the active bank during staging, switching at a safe inference/window boundary, trial health criteria, atomic active/previous selection records, reboot recovery and rollback. The provider descriptor, separate candidate validator, bank records and MQTT control definitions are reusable. Stored candidates currently never become active, including after reboot. Partial uploads restart after reboot; signed packages, anti-downgrade policy, encrypted-partition commits and multi-device topic routing remain outside this implementation.
