# FR5 preparation: accurate backend reporting

Implemented 2026-10-30. ML prediction telemetry, ModelStatus and the ML startup
log now use one shared backend identifier. This step changes reporting only;
it does not add runtime backend switching or performance benchmarking.

No firmware was flashed, model storage erased, MQTTX settings changed, or model
files modified. Both firmware configurations were built. ANSI execution on the
physical ESP32-S3 remains to be validated by the user.

## How the backend is identified

`motion_inference_backend()` and `motion_inference_backend_name()` are implemented
in `components/motion_ml/motion_backend.c`. The functions use the existing
`aai16_Backend` enum from the generated Protobuf header. No schema or binding
changes are needed.

The reporter is compiled **on the pinned TFLite Micro component target**, alongside
the fully-connected wrapper. This matters because `ESP_NN` is a private flag of
that target; its absence in the MQTT or motion component says nothing about the
kernel implementation. Project CMake adds the reporter source to the target;
no managed dependency source is patched.

The reporter mirrors the wrapper and ESP-NN header selection:

| Kernel build configuration | Reported enum | Numeric value |
| --- | --- | --- |
| `ESP_NN` absent or zero | `BACKEND_TFLM_REFERENCE` | 1 |
| `ESP_NN` enabled; `CONFIG_NN_OPTIMIZED` absent | `BACKEND_ESP_NN_ANSI` | 2 |
| `ESP_NN` enabled; `CONFIG_NN_OPTIMIZED` defined | `BACKEND_ESP_NN_OPTIMIZED` | 3 |

For firmware, the generated `sdkconfig.h` comes from the selected build's
configuration. ESP-IDF's ANSI choice defines `CONFIG_NN_ANSI_C` and leaves
`CONFIG_NN_OPTIMIZED` undefined. `ESP_NN=1` alone therefore never means optimized.
The optimized label describes the selected ESP-NN implementation; individual
optimized kernels may still contain scalar tails or fallback paths.

The pinned firmware dependency currently enables `ESP_NN` unconditionally. Its
existing Kconfig choice selects ANSI or optimized. Existing host targets also
exercise the reference path. This fix does not introduce a firmware option to
disable ESP-NN completely.

Both publications use the same function:

- `aai16/ml/motion/data`: `Envelope.ml_prediction.backend`.
- `aai16/model/status`: `Envelope.model_status.backend`, including command
  completions and spontaneous status updates.

Illustrative startup lines (memory figures vary):

```text
ML: arena_reserved=16384 arena_used=... internal_free=... internal_min=... backend=BACKEND_ESP_NN_OPTIMIZED backend_id=3 core=1 stack=6144
ML: arena_reserved=16384 arena_used=... internal_free=... internal_min=... backend=BACKEND_ESP_NN_ANSI backend_id=2 core=1 stack=6144
```

Model A and B in a given firmware use the same compiled backend. Activation and
rollback change model identity/epoch, not the backend.

## Verification results

| Check | Result |
| --- | --- |
| Backend selection | 5/5 host cases: reference, explicit ESP_NN=0 with optimized Kconfig present, ANSI Kconfig, ANSI host defaults, S3 optimized configuration; values and names checked |
| ML MQTT publication | Full MQTT host suite passed separately for reference, ANSI and optimized reporting configurations; C-captured A/B/status frames decoded by Python and backend checked |
| ModelStatus | 35 store/runtime scenarios passed on reference and another 35 on real ESP-NN ANSI kernels; every decoded ModelStatus reply checks the expected backend |
| ML worker/inference | 31 scenarios per model (62 runs), including startup backend log assertions; existing golden-vector and repeated-invocation correctness tests passed on reference and ANSI |
| Model clients | 20 tests passed, including upload, activation, rollback, lost receipts and short MQTTX commands |
| Protocol/MQTTX | 16 codec tests; 9 client/MQTTX tests, including the existing 259 sensor and 52 model Function checks |
| ESP-IDF 5.5.5 optimized | Passed: `build/AAI16_ESP32_proj.bin`, 1,136,208 bytes (`0x115650`) |
| ESP-IDF 5.5.5 ANSI | Passed: `build/fr5_backend_ansi/AAI16_ESP32_proj.bin`, 1,134,192 bytes (`0x114e70`) |
| Build selection evidence | Reporter and FC wrapper share the same TFLM target and `-DESP_NN`; generated ANSI/optimized configurations differ only in the three NN choice settings. Linked reporter returns 2 in ANSI ELF and 3 in optimized ELF |
| Preservation | SHA-256 comparisons confirmed all 36 checked input files (Model A/B directory contents, root sdkconfig/defaults, partition CSV and dependency lock) unchanged; both generated partition tables are identical |
| Schema | Generation check passed; schema and generated bindings unchanged |

Host optimized tests exercise reporting/serialization with SDK doubles; they do
not execute Xtensa optimized kernels on a PC. Repeated Invoke tests check
correctness and allocation stability; no performance measurements or speedup
claims were collected. Board timing and hardware backend execution are pending.

Logs are `build/fr5_backend_*.log`. The isolated ANSI configuration is
`build/fr5_backend_ansi.sdkconfig`; the project's root `sdkconfig` remains optimized.
Captured ML frames for each configuration are in `build/mqtt_host`,
`build/mqtt_host_ansi` and `build/mqtt_host_reference`.

Reproduce the reporting tests with host GCC/CMake/Ninja on PATH:

```powershell
$protocolPython = '.\build\protocol_python\Scripts\python.exe'
cmake --build build/motion_firmware_host -j4
ctest --test-dir build/motion_firmware_host -R backend_ --output-on-failure
& build/motion_firmware_host/test_upload.exe build/model_b_package/model.package
& build/motion_firmware_host/test_upload_ansi.exe build/model_b_package/model.package
& $protocolPython tests/mqtt_host/run.py --idf-path D:/ESP/.espressif/v5.5.5/esp-idf --backend optimized
& $protocolPython tests/mqtt_host/run.py --idf-path D:/ESP/.espressif/v5.5.5/esp-idf --backend ansi
& $protocolPython tests/mqtt_host/run.py --idf-path D:/ESP/.espressif/v5.5.5/esp-idf --backend reference
& $protocolPython tests/model_upload_host/test_client.py
& $protocolPython tests/protocol_host/run.py
& $protocolPython tests/protocol_client/run.py --mqttx-asar D:/ESP_MQTTX/resources/app.asar
```

## Safest next manual ANSI check

Use the isolated ANSI image, preserving the existing optimized configuration and
stored Model B. Do not re-upload B merely to change backend. The code, model
package format and partition layout are the same.

1. In an ESP-IDF 5.5.5 terminal at the repository root, inspect the prepared
   `build/fr5_backend_ansi.sdkconfig`: ANSI is enabled, optimized is unset, and
   `CONFIG_NN_OPTIMIZATIONS=0`. `CONFIG_NN_SKIP_NUDGE` remains disabled. CPU
   frequency remains 160 MHz. The root configuration still selects optimized.
2. Rebuild the isolated image if needed; this command does not flash:

   ```powershell
   $ansiConfig = (Resolve-Path .\build\fr5_backend_ansi.sdkconfig).Path
   idf.py --no-ccache -B build/fr5_backend_ansi -D "SDKCONFIG=$ansiConfig" -D MOTION_BOOT_MODEL=A build
   ```

3. **When you choose to perform hardware validation**, flash the application only:

   ```powershell
   idf.py --no-ccache -B build/fr5_backend_ansi -D "SDKCONFIG=$ansiConfig" -D MOTION_BOOT_MODEL=A -p COM5 app-flash monitor
   ```

   Replace COM5 with the board's port. `app-flash` writes the application, leaving
   the bootloader, partition table, NVS and model data partitions alone. Do not
   use `erase-flash` or change the partition layout. The IDE's ordinary default
   Flash action may select the optimized `build` directory; use the explicit
   ANSI directory above. These flash commands were **not executed** in this step.
4. On boot, require `backend=BACKEND_ESP_NN_ANSI backend_id=2`, bundled A, and
   successful golden/trial checks. Request `{"status":true}` through the existing
   MQTTX Model Function; require backend ANSI, active A, and the previously
   uploaded B listed as `SOURCE_STORED`. If B recovery/validation fails, investigate
   that result before changing or overwriting stored data.
5. Check A REST/MOVING Protobuf predictions and backend ANSI. Send
   `{"activate":"b"}`, require successful ModelStatus, B's original identity and
   backend ANSI, then check B predictions. Send `{"activate":"a"}` and exercise
   `{"rollback":true}` as needed. Backend stays ANSI; model identity/epoch change.
6. To restore optimized firmware later, use application-only flashing from the
   normal build with the original root configuration:

   ```powershell
   $optimizedConfig = (Resolve-Path .\sdkconfig).Path
   idf.py --no-ccache -B build -D "SDKCONFIG=$optimizedConfig" -D MOTION_BOOT_MODEL=A -p COM5 app-flash monitor
   ```

No MQTTX schema or Function changes are required. The existing enum decodes to
`BACKEND_ESP_NN_ANSI` or `BACKEND_ESP_NN_OPTIMIZED`. Reboots still select bundled A;
stored packages survive and can be selected explicitly.

## Changed files

- `components/motion_ml/include/motion_backend.h` and `motion_backend.c`: shared
  enum/name API and build-selection logic.
- `components/motion_ml/CMakeLists.txt`: compile the reporter with the kernel
  target and expose the existing Protobuf enum dependency.
- `components/motion_ml/motion_ml.cc`: accurate Serial startup reporting.
- `components/mqtt_manager/mqtt_manager.c`: actual backend in ML telemetry.
- `components/model_upload/upload_store.cc`: same backend in ModelStatus.
- `tests/motion_host/CMakeLists.txt`, `test_backend.c`, `test_engine.cc`,
  `test_worker.cc`: configuration matrix, actual host-kernel identity, startup
  logs, and reference/ANSI store test builds.
- `tests/model_upload_host/test_store.cc`: verify backend on every decoded status.
- `tests/mqtt_host/run.py`, `test_motion_protocol.inc`: three reporting builds
  and assertions at the C MQTT/Protobuf boundary.
- `docs/fr5_backend_reporting.md`, `docs/ml_telemetry_protobuf.md`: results,
  detection semantics and manual hardware procedure.
