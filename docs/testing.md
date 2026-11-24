# Testing

All firmware logic is tested **on the laptop, without a board**: the C components are compiled for the host
(with `zig cc`, installed by `pip` from `tools/requirements.txt`) and driven by Python, which encodes fixtures with the
independent Google Protobuf library and computes expected results with independent reference code. Only the thin
ESP-IDF platform layer (`components/platform_esp/`) needs the real hardware, and it is covered by the board checks below.

## Running the tests

```powershell
# every host suite plus the laptop client's offline self-test (about 3 minutes)
.\.venv\Scripts\python.exe tools\run_all_tests.py

# one suite
.\.venv\Scripts\python.exe tests\config_host\run.py
```

`build_and_flash.ps1` (and `tools\build_all.ps1`) run all suites before building, unless `-SkipTests` is given.
Generated files go to `tests/<suite>/_gen/` (git-ignored).

Result of the last full run (commit of this README): **all 13 suites pass**, about 6,500 checks.

## Test suites

| Suite | What it tests | Checks | Key test cases |
|---|---|---|---|
| `config_host` | `config_manager`: validating and applying sensor configurations | 122 | Config A encoded by Python decodes and applies; swapping one bus leaves the others running; Config B replaces all; generation rules (stale, duplicate, zero); a table of invalid single-profile mutations that must be rejected (bad pins, op counts, decode fields, periods); cross-profile pin conflicts; disabling a bus; apply failure and persist failure both roll back; malformed and oversize messages; persist and restore across a simulated reboot; restoring an empty store |
| `decode_host` | `sensor_decode`: configuration-driven decoding | 49 | Every sensor profile decodes recorded or hand-computed bytes to the value computed by independent Python (BME280 with Bosch's reference formulas: 25.08 C, 100653.27 Pa); signed/unsigned and big/little-endian fields; bit fields and shifts; checksums (SUM8, SUM8_NEG, XOR8) accept good and reject bad frames; RPN calculations |
| `runtime_host` | `sensor_runtime`: executor and per-bus scheduler on simulated devices | 44 | Config A runs and a single bus is swapped; device faults give invalid readings with the right error code; Config B and a UART timeout; overrun counting and stopping a sensor; failed reads publish only the bytes actually received (never stack contents) |
| `services_host` | `node_services`: rule engine, bridge, heartbeat, serial REPL | 152 | Rules from the Python encoder; statistics (latest, min, max, mean, delta, stddev), edge triggering, cooldown and actions; rule validation; bridge mapping, stale data and shape checks; the ML selector validating virtual sensors through the bridge; pins already in use; heartbeat encoded in C and decoded in Python; REPL commands (RUN, STOP, CFG, WATCH, REBOOT) |
| `ml_host` | `ml_core`: SHA-256, features, int8 glue, packages, input selector, model store | 4,315 | SHA-256 test vectors; feature extraction and int8 quantisation identical to the trained models' golden vectors; package parsing and contract checks; selector compatibility (a temperature sensor mapped to the motion model is refused and the previous mapping stays); streaming windows; a swapped-away bus suspends the model without stale samples; model store upload, digest check, activation and rollback |
| `node_host` | the whole node core on simulated devices, with the real trained models | 487 | Config, rules, bridge and model flows end to end; 8 inferences match the recorded int8 tensors of the real TFLite models; Motion and Environment results on recorded windows; the REPL; recovery of rules, bridge, ML mapping and model generation after a simulated reboot, including a damaged state record |
| `anomaly_host` | the anomaly model (Model C) | 103 | Package parsing; slot statistics match the Python reference; mapping rules (any subset, kinds checked); a full scenario; sensors disappearing and rule-requested inference |
| `dormant_host` | dormant sensors and the SENSOR_START / SENSOR_STOP rule actions | 1,058 | A stored (dormant) sensor stays off; a rule starts it, optionally for a set time; it stops on a timer or by rule; persistence of dormant profiles |
| `topics_host` | MQTT topic names | 28 | The firmware's topic code (`node_topics.c`) and the laptop client agree in both directions for every message kind |
| `client_mqtt` | the laptop client (`tools/node_client.py`) against a real MQTT broker and a scripted mock node | 9 groups | Config, rules, bridge, commands, model upload with a lost ACK and out-of-order recovery, exact bytes on the wire |
| `real_data_host` | the real-data path | 1 path | `collect_readings.py` CSV, through `ml/real_data.py`, to `ml/train_*.py --data` and a valid model package |
| `shorthand_host` | the short JSON config form (`tools/config_shorthand.py`) | 37 | Every bare sensor name expands to exactly the built-in profile; overrides (period, address, pins, UART settings); dormant and disable forms; unknown sensors, settings and presets are refused |
| `demo_pack_host` | the FlexiCargo copy-paste pack and the `configs/` examples | 134 (48 of them by the firmware validators) | Every pack message converts to Protobuf exactly as the JSON helper does; every config, rule set and bridge (pack and `configs/`) passes the firmware's own validators; model generations rise in demo order; every ACTIVATE uses the transfer id of its chunk files; chunks reassemble to the package with a matching SHA-256; every accepted ML mapping matches the model contract's value kinds and every deliberately bad mapping breaks them |
| `node_client selftest` | offline checks of the client | 9 | Chunking at 256 / 512 / 1024 bytes reassembles with a matching SHA-256; chunks above 1024 bytes are refused; SET_INPUTS and command encoding |

`tests/qemu_nvs_probe` is a separate, optional ESP-IDF project for running the real image in the QEMU emulator; it
needs a local QEMU build (not part of the repository) and is not in the automatic run.

## Firmware build checks

`build_and_flash.ps1` / `tools\build_all.ps1` build each node and then verify what actually went into the image:
node id, MQTT broker, Wi-Fi SSID set, target `esp32s3`, octal PSRAM, 1 kHz FreeRTOS tick, and that the image contains
the ESP-NN kernels (or, for the `-Reference` build, that it does not).

## Board checks (manual, on the FlexiCargo rig)

| Check | How | Expected |
|---|---|---|
| Both nodes online | `.\demo\run.ps1 status`, or the heartbeats in MQTTX | uptime, generations, running sensors, free heap for each node |
| Every sensor answers | `.\demo\run.ps1 check-wiring` (probes each sensor alone, then switches all off) | PASS per sensor, with its last value |
| Config, rules and bridge | demo Scenes 1 to 3 ([demo_guide.md](demo_guide.md)) | `APPLIED` acks and the expected notifications |
| ML inference | Scenes 4, 6, 7, 11 | `inference` messages with `inference_us` |
| Invalid mapping refused | Scene 8 | `REJECTED`, `VALUE_KIND_MISMATCH`, model stays `ACTIVE` |
| Model upload | Scene 11 | every chunk `OK`, then `ACTIVE` and live inferences |
| Persistence | reboot node1 after a full setup | it comes back with its config, rules, bridge and active model |
| Sampling timing | `tools\record_slot_data.py` then `tools\analyze_sampling_timing.py` | no skipped intervals; worst lateness reported |

What has and has not been verified on the boards is listed in [project_status.md](project_status.md).
