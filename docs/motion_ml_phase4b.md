# Model B firmware integration and hardware handoff

Implementation date: 2026-10-29 (Singapore). Model A was verified by the owner
on ESP32-S3 before this stage: REST/MOVING from live 100 ms GY-511 samples,
with approximately 230–380 us observed inference latency. That timing belongs
to Model A. **Model B has not been flashed or hardware-tested during this stage.**

## Model provider and integration

`components/motion_ml/model_provider.h` defines `ModelView`, `GoldenVector` and
`ModelDescriptor`. A descriptor carries family, compatibility profile, immutable
model bytes/length/SHA-256, version, input/output quantization and golden windows.
The provider owns all referenced storage for the interpreter/descriptor lifetime.
Model bytes must be 16-byte aligned. A future flash-mapped provider can supply
the same descriptor; storage ownership, upload, activation and rollback policy
remain separate from `Engine` and preprocessing.

`model_source.cc` provides `bundled_model(ModelFamily)`, `startup_model()` and
generic `model_self_test()`. Both frozen bundles are generated during the build.
The development-time CMake cache option `MOTION_BOOT_MODEL` selects startup A
or B; a new build directory defaults to A, and invalid selections are rejected.
An existing directory remembers its selection, so commands below specify it.
Only one interpreter and one arena are active. This is **not runtime switching**.

`motion_engine.cc` and `motion_core.c` are unchanged. The existing engine verifies
SHA-256, alignment, FlatBuffer/schema, operators, tensor shape/type/quantization
and allocation. Self-tests then run the selected model's raw windows through
the production preprocessing, INT8 encoding, interpreter and decoding, requiring
exact inputs, outputs and predictions. Empty/missing golden data, an unsupported
profile or mismatched quantization fails closed. Model initialization/self-test
failure prevents live predictions; an Invoke failure still requires restart.

`motion_ml.cc` obtains the startup descriptor, reports its identity and quantization,
and uses its version in the unchanged `motion_result_t`. Logs identify the selected
family instead of hardcoded A. Runtime input/output scales are read from the
validated interpreter. Model A's output parameters are never substituted for B's.

The existing 100 ms acquisition, 20-sample window, stride 10, mean/clamp/scale
pipeline, continuity gates, queue policy, 16 KiB internal arena, 6,144-byte worker
stack and core/priority remain unchanged. No MQTT manager, Protobuf codec/schema,
sensor owner, dependency version or partition implementation changed.
The full build exposed an unrelated standalone `X` at line 8 of the ignored
`mqtt_credentials.h`; only that stray line was removed. Every remaining header
byte, including all credential values, was preserved.
The MQTT adapter continues to send the existing JSON shape on
`aai16/ml/motion/data`, QoS 0; `model` becomes B's version in a B build.

## Frozen Model B

| Field | Value |
| --- | --- |
| Version | `aai16-motion-model-b-0.1.0` |
| Artifact | `models/motion_model_b/model_int8.tflite` |
| Bytes | 2,384 |
| SHA-256 | `6ec06da5b5fe3d12fdd99328be1e8adf10bc3d78f3c7191bcb06dcd2f5279aff` |
| Architecture | 60 → Dense(8, ReLU) → Dense(2, logits), 506 parameters |
| Input | INT8 `[1,60]`, scale `0.0031422548927366734`, zero point `12` |
| Output | INT8 `[1,2]`, scale `0.0462011881172657`, zero point `-29` |
| Operators | Two FULLY_CONNECTED v4, per-channel INT8 weights, INT32 biases |
| Compatibility profile | 1: GY-511 acceleration in g, 10 Hz, REST=0/MOVING=1 |
| Golden vectors | Four frozen raw windows, encoded inputs and expected outputs |

`generate_bundle.py` verifies the selected family/version, pinned model digest and
size, the shared frozen preprocessing contract, and manifest hashes for metadata
and goldens. It exports descriptor headers only into the build directory. No
training, conversion, recalibration or mutation of model/evaluation artifacts occurs.

## Validation evidence

Current host results are in `build/motion_firmware_host/results_a.json` and
`results_b.json`, with command logs in `build/model_b_*.log`.

ESP-IDF 5.5.5 builds **PASS** for both startup choices, including bootloader and
application partition-size checks. `build/` selects A and
`build/model_b_firmware/` selects B; compiler commands independently confirm
`MOTION_BOOT_MODEL_B=0` and `=1`. Both images are 1,114,816 bytes (`0x1102c0`),
leaving 73% of the existing 4 MiB factory partition free. Both link the S3
per-channel fully-connected assembly kernel and contain both bundled descriptors.
This is compile/link evidence, not hardware execution. Existing managed TFLM
shadow warnings remain; no managed dependency was patched.

Build logs: `build/model_a_firmware_build.log` and
`build/model_b_firmware_build.log`. SHA-256 of the resulting application `.bin`
files (not the bootloader or ELF hashes):

```text
A: eee10b9993707d96f0156aa190ab4edb1c46972e670d4a4df4a2604f278646bc
B: 6d210fc4da10abde330398a7cdef9dec46db91d197a578279d2013acf1f9837d
```

- Five build-generator tests pass, including wrong-family, model/metadata corruption,
  changed preprocessing contract, invalid quantization and missing golden checks.
- Both production host backends (TFLM reference and ESP-NN ANSI) pass four exact
  golden vectors and 10,000 stable Invoke calls per model/backend.
- The real provider was also rebuilt with startup B selected; both engine
  executables passed their startup-selection assertion and B self-tests
  (`build/model_b_host_selection.log`).
- C preprocessing/quantization matches Python exactly on 1,284 windows per model:
  168 training, four goldens, 1,000 deterministic synthetic and 112 held-out windows.
- Reference and ANSI outputs agree byte-for-byte. Against desktop TensorFlow,
  A differs on four output elements by one LSB; B differs on five synthetic output
  elements by one LSB (indices 262, 511, 626, 1094, 1118). All 1,284 classes agree
  for each model. B's training/held-out/golden outputs remain byte-exact.
- 21 worker scenarios pass for each family (42 total), including startup, interval
  restoration, stale/lost samples, configuration during Invoke and injected allocation,
  initialization, golden and Invoke failures. These use explicit RTOS/engine doubles;
  the production engine tests above run real TFLM.
- Cross-model output quantization, wrong/absent golden data, incorrect expected
  outputs/classes and unsupported profiles are rejected. Worker results carry the
  selected version and decoded logits using that model's scale/zero point.
- All 35 training/artifact regression tests pass. The frozen Phase 4A comparison
  remains untouched. Its byte-exact historical CMake recipe is preserved in
  `tests/motion_host/provenance/phase4a_CMakeLists.txt`; the provenance test checks
  that snapshot for the old report while current provider tests use the new recipe.
- Sensor, MQTT/installed SDK, RC522, LD2450, HC-SR04P, MH-Z1911A and Protobuf host
  suites pass, plus seven protocol-client tests. The MQTT test now also checks B's
  version on the unchanged topic, JSON shape and QoS.

Host arena-used counts are 1,168 bytes for A and 1,104 bytes for B. They are not
ESP32-S3 RAM measurements and do not justify reducing the 16 KiB target arena.
Insufficient-arena and golden-mismatch messages in host logs are deliberate
negative tests; successful process exit and the final PASS results are authoritative.

## Files changed in this stage

| Files | Purpose |
| --- | --- |
| `components/motion_ml/model_provider.h` (new) | Storage-independent descriptor, family and golden-vector contract |
| `components/motion_ml/model_source.cc` | A/B providers, startup choice and generic self-tests |
| `components/motion_ml/motion_engine.h` | Import shared view and expose allocation readiness |
| `components/motion_ml/motion_ml.cc` | Selected-model initialization, identity and logs |
| `components/motion_ml/CMakeLists.txt` | Generate both bundles and select startup model |
| `tools/ml_inference/generate_bundle.py` | Validate frozen A/B artifacts and generate typed descriptors |
| `tools/ml_inference/test_bundle.py` (new) | Generator integrity/admission tests |
| `tests/motion_host/CMakeLists.txt` | Build both descriptors and both worker-family harnesses |
| `tests/motion_host/test_engine.cc` | Actual A/B runtime, self-test and quantization rejection tests |
| `tests/motion_host/run.py` | Per-model numerical comparisons, held-out replay and reports |
| `tests/motion_host/test_worker.cc`, `worker_platform/fake_engine.h` | Per-family worker identity and logits checks |
| `tests/motion_host/provenance/phase4a_CMakeLists.txt` (new) | Byte-exact historical recipe for frozen comparison evidence |
| `tools/ml_training/test_model_b.py` | Check historical recipe at its preserved location |
| `tests/mqtt_host/test_mqtt.c` | Model B identity through unchanged MQTT adapter |
| `.gitattributes` | Preserve historical recipe bytes across checkout |
| `tools/ml_training/README.md`, this document | Current milestone and hardware handoff |
| `components/mqtt_manager/mqtt_credentials.h` (ignored local file) | Remove only a stray standalone `X` blocking compilation |

Other dirty/untracked files predate this stage. Frozen model bundles, evaluations,
comparison results, `motion_engine.cc` and `motion_core.c` were not modified.

## Build, flash and monitor

Run from the repository root in PowerShell using the installed ESP-IDF 5.5.5
environment. COM5 is the IDE's configured port; change it if the board enumerates
elsewhere. Build selection does not alter sensor configuration or flash partitions.

```powershell
. 'C:\Espressif\tools\Microsoft.v5.5.5.PowerShell_profile.ps1'

# Separate Model B test image; Model A's build directory remains available.
idf.py --no-ccache -B build/model_b_firmware -D MOTION_BOOT_MODEL=B build
idf.py --no-ccache -B build/model_b_firmware -D MOTION_BOOT_MODEL=B -p COM5 flash monitor
```

Exit the monitor with Ctrl+]. To build/restore Model A:

```powershell
idf.py --no-ccache -B build -D MOTION_BOOT_MODEL=A build
idf.py --no-ccache -B build -D MOTION_BOOT_MODEL=A -p COM5 flash monitor
```

`--no-ccache` avoids the local cache-wrapper stall encountered during validation;
it does not change model/kernel settings. The selected dependency versions remain
ESP-TFLM 1.4.1 and ESP-NN 1.3.0; optimized S3 kernels and the existing nudge setting
are unchanged. Reflashing here is solely the temporary development test workflow.
The next stage must implement runtime replacement without reflashing firmware.

Expected log forms, **not captured hardware results** (prefix timestamps omitted):

```text
ML: initializing model=B version=aai16-motion-model-b-0.1.0 bytes=2384 profile=1
ML: model=B input_scale=0.003142254893 input_zero_point=12 output_scale=0.04620118812 output_zero_point=-29
ML: Model B startup golden vectors: 4/4 PASS
ML: arena_reserved=16384 arena_used=<measured> internal_free=<measured> internal_min=<measured> ESP_NN=1 core=1 stack=6144
ML: model=B ready=1 status=collecting interval_ms=100
ML: model=B state=REST inference_us=<measured> samples=<first>..<last> clipped=<count> total=<count>
ML: model=B state=MOVING inference_us=<measured> samples=<first>..<last> clipped=<count> total=<count>
```

Transient `sensor_unavailable`, `no_data` or `collecting` statuses depend on startup
and scheduling. `ready=1` means initialization/goldens succeeded, not that a valid
window exists. After 20 continuous valid samples, expect a prediction; subsequent
predictions use ten new samples, approximately one per second at 100 ms.

For hardware sign-off, capture initialization/goldens, desk REST and sustained
hand-motion MOVING, model identity in MQTT, actual latency distribution, arena/heap
and stack minima. Exercise existing Protobuf disable/re-enable and interval-away-
from-100/restoration with successful ACKs and fresh windows. Keep all sensor streams
active and verify acquisition continuity and broker reconnect behavior. Preserve
the logs and firmware identity before declaring Model B hardware-tested.

## Repeating host validation

```powershell
$env:PATH = 'C:\msys64\ucrt64\bin;' + $env:PATH
cmake -S tests/motion_host -B build/motion_firmware_host -G Ninja `
  -DCMAKE_C_COMPILER=C:/msys64/ucrt64/bin/gcc.exe `
  -DCMAKE_CXX_COMPILER=C:/msys64/ucrt64/bin/g++.exe `
  -DIDF_PATH=D:/ESP/.espressif/v5.5.5/esp-idf `
  -DPYTHON=C:/Espressif/tools/python/v5.5.5/venv/Scripts/python.exe `
  -DMOTION_BOOT_MODEL=A
cmake --build build/motion_firmware_host -j 4
& tools/ml_training/.venv/Scripts/python.exe -m unittest discover -s tools/ml_inference -p 'test_*.py' -v
& tools/ml_training/.venv/Scripts/python.exe tests/motion_host/run.py --build-dir build/motion_firmware_host --model a
& tools/ml_training/.venv/Scripts/python.exe tests/motion_host/run.py --build-dir build/motion_firmware_host --model b
& tools/ml_training/.venv/Scripts/python.exe -m unittest discover -s tools/ml_training -p 'test_*.py' -v

# Also exercise the real provider's B startup choice, not only explicit lookup.
cmake -S tests/motion_host -B build/motion_firmware_host -DMOTION_BOOT_MODEL=B
cmake --build build/motion_firmware_host -j 4
& .\build\motion_firmware_host\test_reference.exe b
& .\build\motion_firmware_host\test_esp_nn_ansi.exe b
```

## Remaining limitations

No Model B S3 assembly parity, live accuracy, latency or memory measurement is
claimed. The unchanged held-out INT8 score is 105/112 (93.75%), compared with
Model A's 111/112 (99.11%); B misses seven MOVING windows. This is a limited
desk/hand-motion demo, not general motion recognition.

No MQTT upload, flash-backed provider, runtime model/backend switch, activation
transaction, persistent model selection or rollback is implemented. A future
manager must preserve a known-good model, coordinate worker ownership and window
invalidation, and handle power-loss-safe storage/activation. `Engine::init()` alone
is not a transactional switch because it releases the previous interpreter first.
