# Phase 3: bundled Model A inference

Implemented on branch `feature/ml-gy511-data-collection`, 2026-10-28 (Singapore).
Firmware builds with ESP-IDF 5.5.5 for ESP32-S3. No flash, monitor connection,
commit, branch change, training, credential change, or partition change was performed.
The pre-existing untracked `models/motion_model_a_evaluation_01/` was preserved.
The owner subsequently hardware-tested Phase 3. The 100 ms startup-default
follow-up below still requires a fresh-boot hardware check; host tests are not
hardware results.

Final `idf.py build`: **PASS**. `build/AAI16_ESP32_proj.bin` is **1,117,632 bytes**
(`0x110dc0`); the existing 4 MiB factory partition has 73% free. The bootloader
also passes its size check. These are build artifacts, not measured runtime RAM.

## Files and integration

Added:

- `components/motion_ml/`: component manifest/build integration, portable window,
  preprocessing and output code, TFLM engine, bundled model provider, FreeRTOS worker,
  and public C result/sample APIs.
- `tools/ml_inference/generate_bundle.py`: verifies the existing artifacts and emits
  an aligned model/golden header under the build directory. It does not train or
  convert a model, and never writes into `models/`.
- `tests/motion_host/`: real installed TFLM host execution, C/Python equivalence,
  continuity tests, and a separate fault-injected worker harness.
- `tests/sensor_host/motion_stub.c`: verifies acquisition-owner metadata while the
  existing eight-sensor scheduler regressions run.
- This document.

Modified:

- `main/main.c`, `main/CMakeLists.txt`: worker startup, one nonblocking handoff from
  the existing GY-511 reader, and configuration-generation notifications.
- `components/mqtt_manager/{CMakeLists.txt,mqtt_manager.c,include/mqtt_manager.h}`:
  boot-identity access and a temporary bounded result adapter using the existing
  publication worker, queue, expiry, topic dispatch and drop counters.
- `dependencies.lock`, `sdkconfig.defaults`: pinned ML components and explicit
  optimized, bit-exact ESP-NN settings.
- `tests/mqtt_host/{run.py,test_mqtt.c}` and
  `tests/sensor_host/{run.py,test_app.c}`: dependency wiring and integration coverage.

All eight readers remain: DHT22, GY-511, RC522, LD2450, HC-SR04P, MH-Z1911A,
BME280/BMP280 and MAX6675. I2C/SPI/UART ownership and acquisition priorities remain
unchanged. The ML worker never accesses I2C or the MQTT SDK.

## Runtime, integrity and contract

| Item | Verified value |
| --- | --- |
| ESP-IDF / target | 5.5.5 / esp32s3, dual core, 100 Hz RTOS tick |
| Flash / PSRAM | Existing 16 MB DIO/80 MHz configuration; PSRAM remains disabled |
| Runtime | `espressif/esp-tflite-micro ==1.4.1` |
| Kernel dependency | `espressif/esp-nn ==1.3.0` |
| Existing dependencies | MQTT 1.1.0 and cJSON 1.7.19~2, unchanged including hashes |
| Model version | `aai16-motion-model-a-0.1.0` |
| Model size / schema | 3,104 bytes / TFLite schema 3 |
| Input | int8 `[1,60]`, scale `0.0031422548927366734`, zero point `12` |
| Output | int8 `[1,2]`, scale `0.046926043927669525`, zero point `-12` |
| Operations | FULLY_CONNECTED v4 twice: 60→16 with RELU, 16→2 with NONE |
| Weight quantization | Per-channel INT8, 16 and 2 scales respectively |
| Classes | REST=0, MOVING=1; ties select REST |

Confirmed SHA-256:

```text
1885f051bb81daa9f4f8ff12ee39a0706ac4d45999b823ef91ade991e5a4b7aa
```

The build verifies the model and the authoritative contract, quantization and
golden artifacts against `manifest.json`, plus the explicitly pinned model hash.
Startup independently verifies SHA-256 and the FlatBuffer, schema, operator
versions/activations, tensor shapes/types/quantization and allocation results.
Only FULLY_CONNECTED is registered. Four raw golden windows then pass through
the production preprocessing, input encoding, actual interpreter and decoding;
any exact golden mismatch disables live inference.

The installed package's `tensorflow/lite/micro/kernels/esp_nn/fully_connected.cc`
supports per-channel INT8 and fused activations. Real host execution verifies
that this particular export loads and runs. The package compiles wrappers with
`-DESP_NN`; firmware uses `CONFIG_NN_OPTIMIZED=y` and
`CONFIG_NN_SKIP_NUDGE` disabled. On ESP32-S3 the per-channel FC route selects
Espressif's assembly implementation. Host tests separately compile its reference
and ESP-NN ANSI routes. Assembly execution awaits the board's startup tests.

The component manager added only the two ML dependencies and their hashes,
updated the manifest hash/direct dependency list, and kept existing locks.
No managed source was patched. Do not edit `managed_components/` for benchmarks;
record runtime version, kernel selection, nudge setting, core and arena placement.
This phase does not add runtime model switching or a benchmark switch.

Official references: [runtime package](https://components.espressif.com/components/espressif/esp-tflite-micro/versions/1.4.1/readme),
[pinned fully connected wrapper](https://github.com/espressif/esp-tflite-micro/blob/v1.4.1/tensorflow/lite/micro/kernels/esp_nn/fully_connected.cc),
[ESP-NN package](https://components.espressif.com/components/espressif/esp-nn/versions/1.3.0/readme).
The installed package and `dependencies.lock` determine the build, rather than
documentation for a newer branch.

## Sampling and task design

The reader and ML sampling gate now start at **100 ms after every boot** (previously
500 ms). After model initialization/golden checks and 20 continuous valid samples,
predictions begin automatically without an MQTT configuration command. Generation
starts at zero; the 20-sample window and 10-sample stride are unchanged.

Runtime configuration remains available on `aai16/config/set`. Selecting any
interval other than 100 ms discards incomplete ML windows and reports
`requires_100ms`. Restore 10 Hz with this existing, non-retained command:

```json
{"sensor":"gy511","interval_ms":100}
```

Wait for its applied ACK on `aai16/config/ack`. An unavailable reader cannot apply
the command; ML reports `sensor_unavailable`. A disabled reader reports
`sensor_disabled`; enable it with the existing `{"sensor":"gy511","enabled":true}`
command before setting its interval. Read failures produce invalid-sample status.
Silence at an otherwise valid configuration produces `no_data`.

Each read attempt carries boot identity, sequence, applied configuration generation,
configured interval and the driver's acceleration-burst acquisition timestamp.
ML sequences count failed reads too; they are **not** the raw MQTT telemetry
sequence, which counts successful-reading publication attempts.

The first window requires 20 valid chronological samples; the next retains the
last ten and collects ten more. At 10 Hz this is about two seconds to first output
and one second per subsequent output. Every committed configuration command
increments generation, including repeated values. Boot/generation changes,
missing/duplicate/reordered sequences, non-increasing timestamps, gaps **greater
than** 150 ms, invalid readings and queue overflow discard partial windows.
Sequence wrap from UINT32_MAX to zero is allowed within a continuous boot.
After a discontinuity, collection restarts with a fresh valid segment, rather
than retaining the offline dataset's original row grid.

| Resource | Provisional allocation / policy |
| --- | --- |
| Existing GY-511 reader | Priority 4, unpinned, 4,096-byte stack, unchanged |
| ML worker | Priority 3, core 1, 6,144-byte stack |
| MQTT publisher | Existing priority 3, unpinned, 4,096-byte stack |
| Input FIFO | 32 fixed-size records, static allocation, zero-wait producer |
| Window | 20 samples, fixed storage, stride 10 |
| Tensor arena | 16,384 bytes, aligned to 16 bytes, internal 8-bit RAM |
| Model/golden data | Constant aligned build bundle in flash |

Queue overflow drops the newest submission, increments a loss epoch and cancels
older queued/in-flight windows. Stale queued samples over 150 ms old are rejected
even when the FIFO did not overflow. Configuration changes also cancel an Invoke
result before callback admission when detected. A result already admitted to
the transport can finish publication after a configuration change.

The queue, interpreter placement storage and window are bounded. The worker arena
and RTOS task allocation occur once and are checked. Interpreter initialization,
operator registration, tensor allocation and every Invoke are checked. An Invoke
failure disables inference until restart; the worker continues bounded status
reporting. Even failed initialization retains at most its one arena allocation
for the worker lifetime, with no retries or accumulating allocations.

There are no per-window heap allocations in the ML path. The existing MQTT SDK
retains its own allocation behavior. The worker measures `Engine::invoke` using
`esp_timer_get_time()` (including the 60-byte input and two-byte output copies,
excluding preprocessing). It logs arena-used bytes, internal free/minimum heap,
stack high-water mark, queue losses, resets, invocation failures and input clipping.
Hardware sizes, latency and long-run memory stability still require measurement.

`motion::ModelView` separates the immutable model pointer, length, digest, version
and expected tensor quantization from the interpreter. A future flash-mapped
provider must retain aligned model storage for the engine lifetime. The existing
model partitions are not read, written or redesigned in this phase.

## Preprocessing and output

Float32 sums accumulate chronologically per axis, divide by 20, then each sample
has its axis mean subtracted, is clamped to ±2g, divided by 2, and flattened XYZ
sample-major. This component disables floating-point reassociation/contraction.
Quantization explicitly rounds the float32 quotient to nearest, ties to even,
adds the integer zero point, counts out-of-range elements and saturates to int8.
Clipping is counted per window and cumulatively, including overlapping elements.

The two outputs are logits: `(q - output_zero_point) * output_scale`. Argmax uses
the common monotonic scale; a tie chooses REST. No probability or confidence is
invented. No threshold classifier is used in the firmware.

## Test evidence and remaining numerical difference

Host tests passed:

- Model mutation rejected by both the build generator and runtime hash check;
  malformed FlatBuffer, missing/misaligned/insufficient arena, wrong tensor
  quantization, Invoke-before-initialization and reinitialization checks.
- Four authoritative golden vectors have byte-exact inputs and outputs in both
  real TFLM host backends. All 168 stored training outputs reproduce in Python.
- C preprocessing and input quantization exactly match NumPy/TensorFlow for
  1,172 windows: 168 stored windows, four golden windows and 1,000 deterministic
  synthetic windows. Ties-to-even, negative ties, clamping, clipping and logits
  interpretation also have focused tests.
- Both TFLM backends agree byte-for-byte on all 1,172 outputs. Against desktop
  TensorFlow 2.20.0, **four of 2,344 output elements differ by one INT8 unit**
  (vector indices 123, 137, 181, 295). All 1,172 argmax decisions agree. This
  includes two stored-training-vector output differences. The difference also
  occurs with desktop BUILTIN_REF, so it is not a default-delegate artifact.
  The exact internal numerical cause was not isolated. Tests require exact golden
  outputs, at most one LSB across the wider corpus, and complete class agreement.
  One output LSB corresponds to approximately 0.046926 logit units.
- Each backend completes 10,000 repeated actual Invoke calls with unchanged
  outputs and arena-used count. Host arena usage was 1,168 bytes; this is **not**
  ESP32 RAM usage or its required arena size, and excludes transient startup needs.
- Twenty-one worker scenarios: live cadence, automatic startup without a command,
  500 ms gate, interval change/restoration with fresh windows, invalid data, gap,
  sequence loss, reboot identity, configuration rebuild, disabled, stale, silence,
  unavailable, overflow, configuration during Invoke, queue/task/arena allocation
  failures, interpreter initialization failure, golden failure and Invoke failure.
  These use explicit RTOS/engine fault doubles, separately from real TFLM tests.
- Existing sensor, MQTT/installed MQTT SDK, RC522, LD2450, HC-SR04P and MH-Z1911A
  host suites pass. MQTT tests cover prediction/status JSON, no SDK call from the
  producer, QoS 0, offline rejection, full queue and invalid result rejection.

The synthetic stress corpus intentionally clips heavily (47,133 input elements).
That is neither the independent evaluation's 157 clips nor a measured live count.
Synthetic tests and training replays are not independent hardware accuracy tests.
Logs and machine-readable ML results are in ignored `build/ml_*.log` files and
`build/motion_host/results.json`.

To repeat in PowerShell from the repository, with host GCC/G++ available:

```powershell
. 'C:\Espressif\tools\Microsoft.v5.5.5.PowerShell_profile.ps1'
idf.py build
cmake -S tests/motion_host -B build/motion_host -G Ninja `
  -DCMAKE_C_COMPILER=C:/msys64/ucrt64/bin/gcc.exe `
  -DCMAKE_CXX_COMPILER=C:/msys64/ucrt64/bin/g++.exe `
  -DIDF_PATH=D:/ESP/.espressif/v5.5.5/esp-idf `
  -DPYTHON=C:/Espressif/tools/python/v5.5.5/venv/Scripts/python.exe
cmake --build build/motion_host -j 4
& tools/ml_training/.venv/Scripts/python.exe tests/motion_host/run.py
python tests/sensor_host/run.py
python tests/mqtt_host/run.py --idf-path $env:IDF_PATH
python tests/rc522_host/run.py
python tests/ld2450_host/run.py
python tests/hcsr04p_host/run.py
python tests/mhz1911a_host/run.py
```

TensorFlow/NumPy remain in the existing ML virtual environment. Do not install
them into ESP-IDF's environment. The build environment needed registry network
access for the initial component download; subsequent builds used the lock/cache.

## MQTT and device handoff

Topic: **`aai16/ml/motion/data`**, QoS 0, non-retained, existing eight-entry TX queue
with 320-byte payload capacity and two-second telemetry expiry. Submission is
zero-wait and may drop while disconnected or congested. Existing MQTT diagnostics
include a new `motion_ml` entry. Status changes are throttled to at most once per
second, with a 60-second heartbeat; predictions emit once per completed window.

Illustrative payload (the duration here is an example, not a measurement):

```json
{"model":"aai16-motion-model-a-0.1.0","state":"REST","status":"prediction","ready":true,"boot_id":"5120e965","first":1,"last":20,"generation":0,"period_ms":100,"uptime_ms":45800,"inference_us":123,"clipped":0}
```

Unavailable output uses `state:"UNKNOWN"` and an explanatory `status`, such as
`requires_100ms`, `sensor_unavailable`, `invalid_sample` or `init_failed`.
`ready` describes the interpreter/golden checks, not the availability of a valid
window. A ready interpreter can therefore report `requires_100ms`.
`motion_result_t` carries typed logits, window timestamps and diagnostics
independently of this temporary JSON subset, ready for a later Protobuf adapter.

After explicit approval to flash, use the IDE's configured **COM5** (verify that
the board still uses that port):

```powershell
. 'C:\Espressif\tools\Microsoft.v5.5.5.PowerShell_profile.ps1'
idf.py -p COM5 flash
idf.py -p COM5 monitor
```

Exit the monitor with Ctrl+]. No such command was run during this implementation.
Check on hardware:

1. Startup reports Model A golden vectors `4/4 PASS`; capture arena, heap and
   stack statistics and confirm ESP-NN configuration.
2. Reboot with no MQTT configuration publisher running. Observe 100 ms acquisition
   deltas on `aai16/sensors/gy511/data` and HEALTH `period=100 ms`.
3. Without sending a command, after twenty valid samples observe
   `ML: model=A state=REST inference_us=...`
   on a stationary desk, then MOVING during sustained hand motion; record results.
4. Confirm approximately one output per second, all eight sensor streams and
   Wi-Fi/MQTT remain responsive, and actual acquisition gaps stay within bounds.
5. Publish `{"sensor":"gy511","interval_ms":500}` to `aai16/config/set` with
   retain off; require its applied ACK and `requires_100ms`, with predictions
   stopping after any already queued output drains. Restore 100 ms, require its
   ACK, and confirm predictions resume only after twenty fresh valid samples.
   Set 500 ms again, reboot without further commands, and confirm startup returns
   to 100 ms with automatic predictions. Exercise disable/re-enable, sensor disconnection, broker
   disconnection and reconnect. Confirm windows restart and stale predictions
   are not replayed. Capture clipping and loss/reset counters.
6. Measure inference distribution, stack minimum, arena-used bytes, free/minimum
   heap and long-duration stability with all sensors active. No hardware accuracy,
   latency, available RAM or SMP scheduling claim has yet been established.

Known limits: the model's trained scope is stationary desk REST versus continuous
hand motion; stronger motion can clip calibration range. Core/stack/arena choices
are provisional. Numeric desktop/runtime differences above remain documented.
Fault doubles do not emulate Xtensa assembly, physical buses, or dual-core races.
Initialization/Invoke failures require restart, and QoS 0 is best-effort. Model B,
upload, switching, Protobuf migration and flash partition work remain future phases.
