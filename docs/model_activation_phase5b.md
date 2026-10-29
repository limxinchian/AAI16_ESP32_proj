# Phase 5B — safe runtime activation and rollback

This firmware can start bundled Model A, activate the already uploaded Model B,
and return to A over MQTT while powered from a power bank. Flash this firmware
once to install Phase 5B. Subsequent model changes require neither USB nor a
reboot. The Phase 5A package/partition formats are unchanged. A backward-compatible
`ModelSelection` message now supports short MQTTX commands; existing full
`ModelControl` messages retain their fields and concurrency checks.

**MQTTX quick start:** install the updated schema and Model Function using the
[MQTTX setup below](#mqttx-live-demonstration-with-json--protobuf), then send
`{"status":true}`, `{"activate":"b"}`, `{"activate":"a"}`, or `{"rollback":true}`.
No dynamic status fields, CLI or separate GUI are needed for these commands.

**ML telemetry update (2026-10-30):** `aai16/ml/motion/data` now carries binary
Protobuf; the JSON publisher and `/protobuf` duplicate were removed. See
[ML telemetry and MQTTX Pretty JSON setup](ml_telemetry_protobuf.md) for the current
contract, latest test/build results and the remaining JSON sensor topics.

**Hardware status:** the user confirmed Phase 5A upload, four B goldens, concurrent
A inference, and recovery of B bank 0 after reset. Phase 5B has host and build
verification only. No Phase 5B firmware flash, ESP32-S3 runtime, physical timing,
power-cut, or power-bank test is claimed here.

## Runtime and reboot policy

- Every firmware boot starts **bundled A**, epoch **0**. Activation and previous
  selection exist only in RAM. This deliberately supersedes the old development
  `MOTION_BOOT_MODEL=B` worker startup selection. Provider/backend development
  tests can still exercise B; use `MOTION_BOOT_MODEL=A` for this firmware.
- Valid committed uploads survive reboot and are revalidated by the upload
  component. A stored B is never automatically activated after reboot. No active
  selection or model epoch is written to NVS/flash. There is no selection record
  to tear if power disappears during a change. Boot ID changes; old full
  ModelControl mutations fail their boot precondition. Short selections are live
  intents without a boot precondition; do not replay queued selections across a
  reboot. Upload generation also restarts; obtain fresh status for advanced commands.
- Only the ML worker owns interpreters. A one-command mailbox is consumed between
  live inference invocations. MQTT callbacks only assemble and enqueue frames;
  sensor submission remains a zero-wait queue send.
- A new candidate uses a separate 16 KiB arena. Stored packages are copied into
  aligned, owned internal RAM before constructing the provider/interpreter. The
  temporary flash map is released only after worker completion. All descriptor,
  quantization, SHA, version and golden-vector references outlive the interpreter.
- Admission rechecks package SHA, manifest identity, preprocessing profile, bounded
  INT8 graph/tensors, quantization and all four golden vectors. The worker repeats
  validation against its own RAM copy. New interpreters must pass all four goldens
  and an extra trial Invoke before becoming active. Returning to a retained,
  healthy interpreter reruns its goldens.
- The current working interpreter remains intact until success. A failed
  candidate is destroyed; active identity and epoch stay unchanged. A later live
  Invoke failure restores the healthy previous interpreter, or initializes
  bundled A. If even A fails, the worker reports unavailable and continues
  servicing commands; other sensors continue running.
- Successful changes and automatic live-failure rollback increment `model_epoch`.
  The sample-ingress epoch advances, partial/stride windows are reset, queued
  samples from the old epoch are ignored, and samples acquired at or before the commit
  timestamp are rejected even if submitted afterward. At 100 ms, expect roughly
  **two seconds of fresh samples before the next prediction**, then stride-10
  predictions about once per second. There is an intentional prediction gap.
- Active and healthy previous stored banks are protected from erase. If both
  banks of a family are protected, BEGIN returns BUSY. Activate bundled A to
  release one bank before uploading another B. Commands during an unfinished
  upload return BUSY; finish, abort, or wait for the existing 60-second timeout.

The two arenas are allocated as needed and retained; repeated A/B switching
reuses the interpreters and package. Typical A/B runtime storage is two 16 KiB
arenas plus B's 3,920-byte package, descriptors and interpreter state. Phase 5A's
separate 16 KiB validation arena remains. Larger packages can fail allocation;
that failure retains the working active model.

## MQTT contract

| Topic | Payload / behavior |
| --- | --- |
| `aai16/model/control` | Protobuf `aai16.Envelope.model_control` or `.model_selection`, publish QoS 1, Retain off |
| `aai16/model/status` | Protobuf `ModelStatus`; completion ACK for activation/rollback and GET_STATUS replies; QoS 1, not retained |
| `aai16/model/upload`, `aai16/model/upload/ack` | Existing upload protocol, unchanged |
| `aai16/ml/motion/data` | Binary Protobuf `Envelope.ml_prediction`, QoS 0, existing bounded telemetry queue, not retained |
| Existing sensor/config topics | Existing formats and behavior retained |

ACTIVATE is operation **4**, ROLLBACK **5**, GET_STATUS **6**, LIST **7**.
Full ModelControl ACTIVATE requires the complete target identity (family, version, binary 32-byte
SHA, source, and bank for stored models). Bundled remote activation accepts A.
ROLLBACK selects the previous healthy model; if none exists it tries bundled A.
**Use `activate a` when the intended destination is definitely A.** Rollback from
A can select B if B is the previous model.

Both full ModelControl mutations require `Envelope.request_id`, `boot_id`, `generation` (the upload
generation from status), and `ModelControl.expected_model_epoch`. No transfer ID
or configuration `phase` is attached. ACTIVATE to the already active exact
identity succeeds with `already_active`, leaving the epoch unchanged.

Short commands use the new `Envelope.model_selection` body (tag **34**, protocol
version still **1**). It accepts ACTIVATE with family A/B, or ROLLBACK without a
family. Request ID is required; boot/generation/sequence/phase are rejected if
present rather than silently ignored. `{"status":true}` uses existing ModelControl
GET_STATUS. New firmware accepts both interfaces. Old firmware rejects the new
selection body; update firmware and reimport the schema before using it.

The MQTTX Function runs in a fresh sandbox for each message and has no live
device-state API. It only validates shorthand, allocates a request ID and prepares
JSON for Protobuf encoding. Firmware resolves **A to the compiled bundled A
provider** and **B to the newest valid committed B bank by storage commit serial**,
including version/SHA/source/bank. This is upload order, not version-string order;
bank 0 is not assumed. Equal valid serials are rejected as ambiguous. Use full
ModelControl to choose a specific stored bank or identity.

Resolution occurs in the serialized upload worker, after duplicate detection and
the upload BUSY check. The resolved target follows the existing package integrity,
compatibility, quantization and golden-vector checks, then initialization/trial
inference and commit at the ML worker checkpoint. Its captured current epoch must
still match at that checkpoint. A concurrent automatic rollback produces a stale
result instead of silently resolving again. A failed selected candidate does not
cause a second attempt on an older B bank; the working active model is retained.

The final `ModelStatus` echoes the request ID and includes the actual active and
previous identities, epoch, result, state and detail. Autonomous boot/rollback
status updates omit request ID. The legacy upload ACK topic is **not** the
activation ACK. A broker PUBACK alone does not confirm activation.

Both interfaces share eight cached mutation receipts and the same request-ID
high-water mark. The original selection is cached before identity/epoch resolution,
so an identical rollback retry cannot toggle twice, and an activation retry cannot
select a later upload. Identical retries replay the original receipt. Changed content
under the same ID are rejected. Uncached older IDs are rejected by a per-boot
high-water mark. New mutation IDs must increase. Cached receipts describe their
original transaction; GET_STATUS gives current state after later commands or an
automatic rollback. The CLI checks fresh status after a successful receipt.
Use one serially coordinated controller for the demonstration; don't race CLI and
MQTTX mutations. The MQTTX time-based allocator assumes a forward-moving clock;
the Python allocator uses durable locked request-ID state.

Selection means "apply this intent to the state at admission", not "apply only if
the state I previously read is unchanged". Full ModelControl keeps the latter
boot/generation/epoch protection. Receipts are RAM-only and reset at boot; short
selections do **not** provide cross-reboot exactly-once execution. For the live
demo use a clean MQTT session, send only while connected, leave Retain off, and
do not replay offline commands after reconnect/reboot. Retained control frames
are still rejected. Use full ModelControl for commands that must be bound to a
particular boot and observed state.

## Build and flash once

From the repository root in your configured **ESP-IDF 5.5.5 terminal**:

```powershell
idf.py --no-ccache -B build -D MOTION_BOOT_MODEL=A build
idf.py --no-ccache -B build -D MOTION_BOOT_MODEL=A -p COM5 flash monitor
```

COM5 is the current IDE port; replace it if the board enumerates elsewhere.
Exit Monitor with Ctrl+]. Normal `flash` writes the bootloader, existing partition
table and application. **Do not run `erase-flash`**: the already uploaded package
is in the model data partitions and must be preserved. No partition change is
needed for Phase 5B. Confirm `SOURCE_STORED`, B bank 0 after flashing.

If using an ordinary PowerShell instead of the IDE terminal, this matches the
installed tool locations used for the build check:

```powershell
$env:IDF_PATH = 'D:\ESP\.espressif\v5.5.5\esp-idf'
$env:IDF_TOOLS_PATH = 'C:\Espressif\tools'
$env:IDF_PYTHON_ENV_PATH = 'C:\Espressif\tools\python\v5.5.5\venv'
$env:ESP_ROM_ELF_DIR = 'C:\Espressif\tools\esp-rom-elfs\20241011'
$env:PATH = 'C:/Espressif/tools/python/v5.5.5/venv/Scripts;C:/Espressif/tools/cmake/3.30.2/bin;C:/Espressif/tools/ninja/1.12.1;C:/Espressif/tools/xtensa-esp-elf/esp-14.2.0_20260121/xtensa-esp-elf/bin;' + $env:PATH
& "$env:IDF_PYTHON_ENV_PATH/Scripts/python.exe" "$env:IDF_PATH/tools/idf.py" --no-ccache -B build -D MOTION_BOOT_MODEL=A build
& "$env:IDF_PYTHON_ENV_PATH/Scripts/python.exe" "$env:IDF_PATH/tools/idf.py" --no-ccache -B build -D MOTION_BOOT_MODEL=A -p COM5 flash monitor
```

This repository uses the already configured 16 MiB ESP32-S3 partition layout.
Credentials in the existing headers were not changed. Keep GY-511 enabled at
100 ms for motion inference; other intervals legitimately report requires_100ms.

## Python CLI (optional advanced client)

Use the existing isolated protocol environment. Set the same broker host and,
if required, the same credentials as MQTTX; do not include `mqtt://` in the host.

```powershell
$protocolPython = '.\build\protocol_python\Scripts\python.exe'
$broker = '<your MQTT broker hostname>'
# If the broker requires authentication, set these locally:
# $env:AAI16_MQTT_USERNAME = '<username>'
# $env:AAI16_MQTT_PASSWORD = '<password>'

& $protocolPython tools/protocol/config_cli.py model status --host $broker
& $protocolPython tools/protocol/config_cli.py model activate b --bank 0 --host $broker
& $protocolPython tools/protocol/config_cli.py model activate a --host $broker
& $protocolPython tools/protocol/config_cli.py model rollback --host $broker
```

`activate b` can omit `--bank` if exactly one B is stored. If two banks are listed,
choose explicitly. `activate a` without `--bank` selects bundled A; specifying a
bank explicitly selects a stored A. The CLI obtains current boot/generation/epoch
and exact identity, allocates a request ID, retries identical bytes on a lost
receipt, and checks the completion and subsequent active state. It refuses an
ambiguous target. Add `--tls --port 8883` and optionally `--ca-file` only when those
match the broker. Existing package/upload/status/abort commands still work.

Example success (IDs and timings vary):

```text
request_id=... boot_id=0x... RESULT_OK MODEL_STATE_RUNNING epoch=1 active=aai16-motion-model-b-0.1.0 source=SOURCE_STORED detail=activated_golden_4_of_4_trial_ok_volatile
request_id=... boot_id=0x... RESULT_OK MODEL_STATE_RUNNING epoch=2 active=aai16-motion-model-a-0.1.0 source=SOURCE_BUNDLED detail=activated_golden_4_of_4_trial_ok_volatile
```

## MQTTX live demonstration with JSON → Protobuf

The sensor-only `mqttx_config.js` does not accept model commands. Install the
updated **`tools/protocol/mqttx_model.js`** as **AAI16 Prepare Model** under
**Scripts → Function**. Keep `execute(prepareModel);` at the end. Keep/reimport the
updated **`protocol/aai16.proto`** under **Scripts → Schema**, replacing the old
saved schema. It must contain `ModelSelection` and `model_selection = 34`.
Choose a clean session on this connection (MQTT 3.1.1 **Clean Session: true**;
MQTT 5 **Clean Start: true**, **Session Expiry Interval: 0**). Connect to the
same broker using your existing credentials; no broker or firmware credentials
need to change.

For a dedicated model-demo connection, subscribe to:

```text
aai16/model/status
aai16/model/upload/ack
aai16/ml/motion/data
```

Under **Use Script**, select **All**, Function **AAI16 Prepare Model**, Schema Type
**Protobuf**, the saved schema, Proto name **aai16.Envelope**. Choose **Plaintext**
for both the publish editor and receive display. The Function prepares outgoing
JSON and passes through incoming JSON after MQTTX's schema decoder. Publish to
`aai16/model/control`, QoS **1**, Retain **off**.

MQTTX 1.13.1 shares script selection through application state. While **All** is
selected, use binary-only subscriptions and disconnect other JSON-monitor
connections, or temporarily unsubscribe their JSON topics. This changes only
desktop display, not device sensors. For simultaneous raw JSON monitoring, use
**Published** instead; ACKs then remain binary. Restore the sensor Function when
sending sensor configurations. The installed app's sandbox and pipeline were
checked; no saved MQTTX connections/settings were modified.

### Recommended short commands

Send **one JSON object per publish** while connected. The editor is Plaintext,
but the selected Function and Protobuf Schema produce binary wire messages;
publishing raw JSON directly to firmware is not supported.

| Publish editor | Meaning |
| --- | --- |
| `{"status":true}` | Read current active/previous/stored identities and epoch; no switch |
| `{"activate":"b"}` | Activate newest valid committed stored B; with the verified single package, bank 0 |
| `{"activate":"a"}` | Activate bundled A, even if uploaded A packages also exist |
| `{"rollback":true}` | Return to previous healthy model, or bundled A if none exists |

For the live demonstration, send status, activate B, then activate A. Wait for
the completion on `aai16/model/status` after each change and check REST/MOVING
predictions before proceeding. **Do not copy** boot ID, generation, epoch, version,
SHA or source into these short objects. Lowercase `a`/`b` and literal boolean
`true` are required; mixtures and extra fields are rejected.

Starting at bundled A / epoch 0, expect:

| Command | ModelStatus fields (display may use enum names or numeric values) |
| --- | --- |
| status | `result: RESULT_OK (0)`, active `MODEL_A (1)` / `SOURCE_BUNDLED (1)`, `model_epoch: 0` |
| activate B | `result: RESULT_OK (0)`, `state: MODEL_STATE_RUNNING`, active `MODEL_B (2)` / `SOURCE_STORED (2)` / `bank: 0`, epoch 1, `detail: activated_golden_4_of_4_trial_ok_volatile` |
| activate A | active `MODEL_A (1)` / `SOURCE_BUNDLED (1)`, epoch 2, same success detail |
| rollback after that A→B→A sequence | active B / stored, epoch 3, `state: MODEL_STATE_ROLLED_BACK`, `detail: rollback_ok_volatile` |

Every completion echoes the generated request ID and reports actual active
version/SHA/source/bank plus boot ID and epoch. Compare that request ID with the
transformed sent envelope. A PUBACK alone is not activation success. A request to
the already active identity returns `already_active` without incrementing epoch.
After a switch, allow roughly two seconds of fresh 100 ms samples for predictions.
Serial Monitor output is the same as [below](#expected-serial-monitor-output);
Serial/USB is optional for the power-bank demonstration.

If B is absent, expect `RESULT_OUT_OF_RANGE` with
`no_committed_model_b_upload_first`; A keeps running. An upload in progress returns
BUSY. A corrupt selected bank, incompatible package, or failed candidate returns
a non-OK ModelStatus with failure detail and the retained active identity. If an
epoch race returns stale, inspect fresh status and send a new command.

Each Send of a short command creates a **new request**. Sending rollback twice
intentionally can switch back again. To retry a lost receipt within the same
boot, wrap the original transformed sent JSON as `{"retry":{...original envelope...}}`;
this preserves its request ID and bytes. Use `{"status":true}` to confirm current
state when a completion is uncertain. Do not manually repeat rollback as a retry.

### Full ModelControl commands (optional)

The original exact-target interface remains available for advanced clients.
Only this interface requires manually supplied preconditions:

1. Request status:

   ```json
   {"model_control":{"operation":6}}
   ```

   Read `boot_id`, `generation`, `model_status.model_epoch`, and `stored` from the
   decoded reply. Verify active A and stored B bank 0. The boot ID is an unsigned
   **decimal JSON number**, not the hexadecimal string printed by Serial/CLI.

2. Activate B using the following. **Replace example `boot_id:12345678`,
   `generation:1`, and `expected_model_epoch:0` with the values just received.**
   Use the bank/version/SHA actually listed for the intended package. For the
   verified repository B package, the identity is:

   ```json
   {
     "boot_id":12345678,
     "generation":1,
     "model_control":{
       "operation":4,
       "expected_model_epoch":0,
       "target":{
         "family":2,
         "version":"aai16-motion-model-b-0.1.0",
         "sha256":"6ec06da5b5fe3d12fdd99328be1e8adf10bc3d78f3c7191bcb06dcd2f5279aff",
         "source":2,
         "bank":0
       }
     }
   }
   ```

   The Function converts the hex SHA to the required 32 binary bytes and creates
   an exact fixed64 request ID using Long-like low/high bits. Do not paste a
   rounded large JavaScript Number as request ID. It returns JSON text to the
   schema encoder, which publishes binary Protobuf.

3. Wait for **ModelStatus on `aai16/model/status`**: `RESULT_OK`,
   `MODEL_STATE_RUNNING`, active `MODEL_B` / `SOURCE_STORED` / bank 0, epoch **1**
   if the starting epoch was 0. Observe fresh B REST/MOVING messages on the
   Protobuf motion topic after the new window fills. Status queries do not change
   the epoch or reset input windows.

4. Request status again, then activate bundled A with fresh preconditions:

   ```json
   {
     "boot_id":12345678,
     "generation":1,
     "model_control":{
       "operation":4,
       "expected_model_epoch":1,
       "target":{
         "family":1,
         "version":"aai16-motion-model-a-0.1.0",
         "sha256":"1885f051bb81daa9f4f8ff12ee39a0706ac4d45999b823ef91ade991e5a4b7aa",
         "source":1
       }
     }
   }
   ```

   **No bank field for bundled A.** Expect active A / SOURCE_BUNDLED, epoch **2**,
   RESULT_OK, then A predictions after a fresh window. Repeat with current epochs.

5. To select the previous healthy model instead, send operation 5 with fresh
   boot/generation/epoch and **no target**:

   ```json
   {"boot_id":12345678,"generation":1,"model_control":{"operation":5,"expected_model_epoch":2}}
   ```

   In this A→B→A example that rolls back to B, epoch 3, state ROLLED_BACK and
   `detail: "rollback_ok_volatile"`. For an intentional retry, copy the original
   transformed sent JSON, including its request ID and byte-array SHA, and wrap
   it as `{"retry":{...original envelope...}}`. Never edit an operation under an
   existing request ID. Get fresh status after an ambiguous/lost completion.

The Function's validation and binary encoding are tested against the installed
MQTTX vm2 runtime, Python Protobuf, and the firmware C decoder. Its receive path
preserves decoded ModelStatus. Invalid input produces a schema-encoding error
instead of publishing an accidentally valid original request.

### MQTTX Hex alternative (no Function required)

Prepare a fully bound activation without sending it:

```powershell
& $protocolPython tools/protocol/config_cli.py model activate b --bank 0 --host $broker --mqttx-dir build/mqttx_model
& $protocolPython tools/protocol/config_cli.py model activate a --host $broker --mqttx-dir build/mqttx_model
# Prepare only the command you want next; prepare the next one after completion.
```

The CLI queries status, prints the decimal request ID and hex bytes, and saves
`<request_id>.pb` plus `.hex`. In MQTTX choose **Function: none**, **Schema applied
to Received only**, `aai16.Envelope`, publish editor **Hex**, topic
`aai16/model/control`, QoS 1, Retain off. Paste the `.hex` contents and Send.
Incoming binary status is decoded by the schema. Do not apply send-side Protobuf
encoding to these already encoded bytes. Preparation does not activate a model.
Commands prepared before an intervening switch/reboot/upload can become stale;
prepare again with a new ID. The saved exact hex can be resent for a lost ACK.

## Expected Serial Monitor output

Illustrative successful output; exact timestamps, request IDs, inference times,
and sequence numbers depend on the board:

```text
ML: boot policy=bundled_A activation=volatile golden+trial=PASS detail=boot_bundled_a_volatile_activation
MODEL_UPLOAD: recovered model=B version=aai16-motion-model-b-0.1.0 bank=0 golden=4/4; candidate only
ML: model=A state=REST epoch=0 source=1 inference_us=... samples=... clipped=... total=...
ML: activation result=0 active=aai16-motion-model-b-0.1.0 source=2 epoch=1 checkpoint_us=... detail=activated_golden_4_of_4_trial_ok_volatile
MODEL_UPLOAD: request=... result=0 active=aai16-motion-model-b-0.1.0 source=2 epoch=1 detail=activated_golden_4_of_4_trial_ok_volatile
ML: model=B state=REST epoch=1 source=2 inference_us=... samples=... clipped=... total=...
ML: model=B state=MOVING epoch=1 source=2 inference_us=... samples=... clipped=... total=...
ML: activation result=0 active=aai16-motion-model-a-0.1.0 source=1 epoch=2 checkpoint_us=... detail=activated_golden_4_of_4_trial_ok_volatile
ML: model=A state=REST epoch=2 source=1 inference_us=... samples=... clipped=... total=...
```

`ML: activation result` uses the internal motion-change enum (0 means success).
`MODEL_UPLOAD: result` uses Protobuf ResultCode (0 means RESULT_OK). On a live
failure expect `Invoke failed; active=... epoch=... ready=1
detail=invoke_failed_automatic_rollback`, followed by fresh fallback predictions.
Health logs retain queue drops, window resets, failures, heap and stack minima.

| Failure | Expected completion / behavior |
| --- | --- |
| Short B selection with no valid committed B | RESULT_OUT_OF_RANGE, `no_committed_model_b_upload_first`; active unchanged |
| Short B selection with equal valid bank commit serials | RESULT_OUT_OF_RANGE, `ambiguous_stored_b_use_model_control`; choose explicitly with full ModelControl |
| Old boot, upload generation or model epoch | RESULT_STALE_GENERATION; inspect current status and use new ID |
| Wrong version/SHA/bank or missing committed target | RESULT_OUT_OF_RANGE, `stored_identity_not_found`; active unchanged |
| Corrupted/incompatible stored package | Non-OK, checksum/contract/golden detail; active unchanged |
| Interpreter allocation/init/golden/trial failure | RESULT_INTERNAL_ERROR; working active retained, epoch unchanged; often state ROLLED_BACK |
| Upload in progress or both staging banks protected | RESULT_BUSY with explicit detail; no erase/switch |
| Lost activation ACK | Identical retry replays receipt; GET_STATUS confirms current state |
| Automatic live Invoke rollback | Unsolicited ModelStatus, state ROLLED_BACK, new epoch and actual fallback identity |
| Both prior model and bundled A unavailable | State UNAVAILABLE and explicit detail; sensor/config/upload services remain available |

## Automated verification

Host tests exercise production C/C++ algorithms with simulated flash/RTOS and the
pinned real TFLM kernels. Worker fault tests replace only the interpreter/provider
with deterministic doubles, while using the production worker and switching
state machine. They do not emulate ESP32 SMP timing or optimized S3 instructions.

Passed checks after adding short model selections (historical baseline; see
[ML telemetry verification](ml_telemetry_protobuf.md#verification-and-build) for
the later telemetry change and current build):

| Check | Result |
| --- | --- |
| Production store + real runtime | 35 scenarios; short selections, commit-serial ordering and ambiguity, cached intent, corrupt candidate, allocation failure, shared request-ID namespace, worker epoch race, topic routing; also 100 A→B→A cycles (200 switches), REST/MOVING goldens after every switch, stable retained allocations, no activation flash writes |
| Worker + production switch state machine | 31 scenarios in each of two worker builds (62 runs), including partial windows, request inside Invoke, samples acquired before commit, initialization/golden/trial/allocation faults, automatic rollback and acquisition during a simulated 500 ms initialization |
| Real TFLM reference and ESP-NN ANSI | 1,284 windows per model/backend; all classes agree with desktop; four goldens exact; 10,000 repeated Invokes per model/backend |
| Desktop numerical limit | A: 4 differing output elements; B: 5; each at most one INT8 LSB. Host reference and ANSI outputs agree exactly |
| Python model client | 20 tests including real CLI/Paho → local MQTT → C++ store/runtime, TCP loss after activation, actual MQTTX short-command bytes through the real store/runtime, duplicate rollback, and bundled A selection despite stored A |
| C/Python Protobuf | 15 tests, 127 round trips, 20,000 malformed-wire stress iterations; new selection validation and unchanged full-command required preconditions |
| MQTTX/client workflow | 8 tests; 259 sensor Function checks and 52 model-command Function checks using installed MQTTX 1.13.1 vm2, with firmware C/Python binary compatibility |
| MQTT/config regression | Passed owner transactions, configuration ACKs, SDK/outbox/reconnect checks, upload routing, and new binary prediction identity/epoch/cleanup assertions |
| Sensor regression | sensor_host, rc522_host, ld2450_host, hcsr04p_host and mhz1911a_host all passed |
| ESP-IDF 5.5.5 / ESP32-S3 | A-default firmware build passed, 1,136,864 bytes (`0x1158e0`), leaving 73% of the 4 MiB application partition free |

Selection-extension logs are `build/model_selection_*.log`; real model reports are
`build/motion_firmware_host/results_a.json` and `results_b.json`. All model reports
explicitly record `hardware_tested: false`. No firmware was flashed in this run.
`build/phase5b_*.log` and `build/phase5b_results.json` retain the earlier Phase 5B
baseline, including its separate B-provider build, 5 bundle integrity tests and
20 collector tests; they do not describe the new selection binary.

Reproduce the main tests (host GCC, CMake and Ninja on PATH):

```powershell
cmake -S tests/motion_host -B build/motion_firmware_host -G Ninja -DCMAKE_C_COMPILER=C:/msys64/ucrt64/bin/gcc.exe -DCMAKE_CXX_COMPILER=C:/msys64/ucrt64/bin/g++.exe -DIDF_PATH=D:/ESP/.espressif/v5.5.5/esp-idf -DPYTHON=C:/Espressif/tools/python/v5.5.5/venv/Scripts/python.exe -DMOTION_BOOT_MODEL=A
cmake --build build/motion_firmware_host -j4
& $protocolPython tools/protocol/config_cli.py model package --model-dir models/motion_model_b --output-dir build/model_b_package
& build/motion_firmware_host/test_upload.exe build/model_b_package/model.package
& tools/ml_training/.venv/Scripts/python.exe tests/motion_host/run.py --build-dir build/motion_firmware_host --model a
& tools/ml_training/.venv/Scripts/python.exe tests/motion_host/run.py --build-dir build/motion_firmware_host --model b
& $protocolPython tests/model_upload_host/test_client.py
& $protocolPython tests/protocol_host/run.py
& $protocolPython tests/protocol_client/run.py --mqttx-asar D:/ESP_MQTTX/resources/app.asar
& $protocolPython tests/mqtt_host/run.py --idf-path D:/ESP/.espressif/v5.5.5/esp-idf
& $protocolPython tests/sensor_host/run.py
& $protocolPython tests/rc522_host/run.py
& $protocolPython tests/ld2450_host/run.py
& $protocolPython tests/hcsr04p_host/run.py
& $protocolPython tests/mhz1911a_host/run.py
```

The earlier Phase 5B baseline also checked the provider configuration with:

```powershell
idf.py --no-ccache -B build/model_b_firmware -D MOTION_BOOT_MODEL=B build
```

Use the A-default `build` image for the hardware procedure above.

## Required hardware acceptance

1. Flash once, verify normal eight-sensor behavior, Wi-Fi/MQTT configuration ACKs,
   A startup REST/MOVING, and recovered B bank 0. Record boot ID, epoch 0, GY jitter,
   queue drops, inference timing, heap/stack minima, and configuration generation.
2. Send A→B while partially filling a window, and during steady predictions.
   Require a successful ModelStatus and exact B identity, same boot ID, epoch +1,
   a fresh complete window, and both REST and MOVING predictions. Compare output
   quantization against the model-specific metadata.
3. Send B→bundled A; require the equivalent A checks. Repeat at least 20 cycles,
   checking heap minima stabilize after first use, no resets/reboots, no sample
   windows crossing epochs, and no unexpected GY sequence/jitter/queue losses.
4. Replay identical requests (one epoch change), reuse an ID with changed content,
   send stale boot/epoch/generation, wrong SHA/version/bank, retained requests,
   and commands during upload. Require rejection or intentional drop as documented,
   with working inference and normal configuration ACKs preserved. Exercise both
   short selections and full ModelControl; stale identity/precondition tests use
   full ModelControl. Confirm short B selection chooses the latest committed bank
   if two packages are present, and short A always selects bundled A.
5. Exercise explicit rollback. Golden/init/trial faults are host-injected; do not
   claim their physical behavior verified from a wrong-identity rejection. A
   dedicated lab fault build or controlled corrupt candidate is needed to test
   those failure paths on-device; no remotely exposed fault injection is shipped.
6. Power-cycle while running B and during activation. Every reboot must start A,
   epoch 0, new boot ID, and recover committed B as stored. Test an interrupted
   upload as a regression: the prior committed candidate must survive. Reactivate
   B explicitly. No active-selection persistence is expected.
7. Disconnect USB and use the power bank. Keep the board connected to reachable
   Wi-Fi/MQTT. Observe A, activate B, exercise REST/MOVING, then activate A, entirely
   through MQTTX using only the short JSON commands. Without USB, use ModelStatus/MlPrediction to verify identities,
   boot ID, epoch, sample ranges, resets, drops and inference times. Sensor jitter
   remains in existing GY telemetry; capture it in the JSON-monitor mode separately.

## Limits

Switching is a brief controlled pause in inference; no uninterrupted prediction
cadence or measured switch latency is claimed. Acquisition runs independently,
but physical GY timing, network delay, watchdog behavior and heap/stack margin
must be measured on the ESP32-S3. Flash upload cache stalls remain the Phase 5A
hardware concern; activation itself performs no flash erase/write. Trial health
is deterministic golden-vector inference, not a timed live-data trial.

The synchronous upload-worker bridge retains a borrowed map until the ML worker
finishes; it deliberately does not time out and free memory under a running
interpreter. An unresponsive ML task requires watchdog/reset recovery, which
starts A. Runtime-only selection does not automatically resume B after a reboot.
SHA checks provide integrity, not package signatures or publisher authentication.
Existing broker authentication/ACLs and the single-device topic scheme remain.

## Files changed for Phase 5B

- `components/motion_ml/candidate_provider.h`, `model_runtime.h`, `model_runtime.cc`,
  `include/motion_model_control.h`: owned candidate provider, dual interpreter
  runtime, checkpoint command/reply and identity types.
- `components/motion_ml/motion_candidate.cc`, `include/motion_candidate.h`,
  `motion_engine.h`, `motion_ml.cc`, `include/motion_core.h`, `CMakeLists.txt`:
  shared parser, explicit interpreter cleanup, worker integration, epoch and
  identity propagation, automatic rollback and build registration.
- `components/model_upload/upload_store.h`, `upload_store.cc`, `model_upload.cc`:
  runtime dispatch, exact-target validation, receipt cache, protected banks,
  completion/status publication and temporary-map ownership.
- `components/mqtt_manager/mqtt_manager.c`: actual per-result Protobuf identity,
  epoch and diagnostics on the additional motion topic; JSON epoch retained.
- `protocol/aai16.proto`: existing ModelControl/ModelStatus behavior, followed by
  the backward-compatible ModelSelection extension described below.
- `tools/protocol/model_cli.py`, `mqttx_model.js`: activation, rollback, exact
  preconditions, final-status confirmation and native MQTTX/Hex workflows.
- `tests/motion_host/CMakeLists.txt`, `test_worker.cc`, `run.py`,
  `worker_platform/fake_engine.h`, `worker_platform/worker_platform.h`:
  production-runtime integration and checkpoint/failure/timing doubles.
- `tests/model_upload_host/test_store.cc`, `test_client.py`: real runtime/storage,
  repeated A/B inference, recovery, protected banks, CLI and MQTT loopback tests.
- `tests/mqtt_host/test_mqtt.c`, `tests/protocol_client/run.py`,
  `tests/protocol_client/mqttx_model_test.cjs`: prediction serialization and MQTTX
  schema/sandbox compatibility.
- `docs/model_activation_phase5b.md`: this build, demo, verification and acceptance
  guide. Existing uncommitted Phase 5A work was preserved.

### Files changed for the MQTTX short-command extension

- `protocol/aai16.proto`, `components/aai16_protocol/generated/aai16.pb.c`,
  `components/aai16_protocol/generated/aai16.pb.h`,
  `tools/protocol/generated/aai16_pb2.py`: ModelSelection body and regenerated bindings.
- `components/aai16_protocol/aai16_protocol.c`: strict selection fields and
  operations; existing full ModelControl validation retained.
- `components/motion_ml/include/motion_model_control.h`,
  `components/motion_ml/model_runtime.cc`: expose the actual bundled-A identity
  from the provider. Worker checkpoint/rollback algorithms are unchanged.
- `components/model_upload/upload_store.h`, `upload_store.cc`: resolve selection
  at admission, reuse exact-target validation and the existing receipt cache.
- `components/model_upload/upload_ingress.h`, `model_upload.cc`: accept selections
  only on the control topic and send correlated rejection/completion statuses.
- `tools/protocol/mqttx_model.js`: shorthand preparation and validation; full
  commands and exact retries retained. No live-state cache or hardcoded model SHA.
- `tests/model_upload_host/test_store.cc`, `test_client.py`,
  `tests/protocol_host/run.py`, `tests/protocol_client/mqttx_model_test.cjs`,
  `tests/protocol_client/run.py`: selection behavior, safety, binary interoperability
  and advanced-interface regression tests.
- `docs/model_activation_phase5b.md`: exact short-command setup, semantics,
  verification results and pending hardware acceptance.
