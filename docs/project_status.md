# Project status and evidence

Evidence levels used below:
**Implemented / Host-tested / Target-built / Emulated / Board-tested / Blocked**.

**Bottom line (2026-11-13):** the complete firmware builds for the ESP32-S3, runs in QEMU and **now runs on two real
ESP32-S3 boards** (see "Board results" right below). The sections after it were written on 2026-11-04, before any board
was available; where a later "Board results" line contradicts a "Not verified" item, the board result is the newer fact.

## Board results (2026-11-13: two ESP32-S3 nodes, phone hotspot, Mosquitto on the laptop)

Board-tested: Wi-Fi/MQTT with binary Protobuf for config, rules, bridge, model chunks and model control, with ACKs;
reading all eight sensors (GY-511, DHT22, BME280, HC-SR04, LD2450, MH-Z19B, RC522, MAX6675) through configuration only;
on-chip inference with TensorFlow Lite Micro of the motion, environment (including a faster variant) and anomaly
models (about 200-340 us per inference); the node-to-node bridge (node2 LD2450 and RC522 as virtual sensors 130 and 131
on node1); rules including the new dormant-sensor actions (a card tap on node2 starts stored sensors on both nodes,
which then run until stopped, or for a set time); persistence across a reboot (node1 rebooted and came back with its
config, dormant profiles, rules, bridge and model).

Findings from the boards (all handled in code or docs):
- RC522 clone chip (VersionReg 0xB2) never saw a card at the default receiver gain; the profile now sets RFCfgReg = 0x70.
- MH-Z19B needs 5 V and TX/RX crossed; wrong wiring only shows up as READ_TIMEOUT.
- DHT22 bit-banging times out more often when several sensors start together on one node (about half the reads in that case).
- A sensor that is configured as running never stops by itself; only dormant sensors woken by a rule can have a timer.

Still **not** done or not claimed: ESP-NN vs reference kernel benchmark (AC-16/AC-17); accuracy of the motion and
environment models on real data (synthetic training only); the anomaly model's real-data version covers temperature,
humidity, pressure and equipment temperature only (radar, motion, CO2 and distance slots are synthetic) and its
threshold comes from about 12 minutes of recording; long-run (hours) stability; many-node scaling.

## How to reproduce

| What | Command (repo root, PowerShell) |
|---|---|
| Host tests (13 suites, about 6,500 checks, plus the laptop client self-test; see docs/testing.md) | `.venv\Scripts\python.exe tools\run_all_tests.py` |
| Target build | `. .\tools\idf_env.ps1 ; idf.py set-target esp32s3 ; idf.py build` (ESP-IDF 6.1 from `C:\esp\v6.1`, tools from `C:\Espressif`; the script sets session variables only) |
| Run the real image in QEMU | merge image + `.venv\Scripts\python.exe tools\qemu_session.py` (steps in its docstring; QEMU unpacked to `tools/qemu` from `C:\Espressif\dist`) |
| Retrain the two models | `.venv\Scripts\python.exe ml\train_motion.py` and `ml\train_environment.py` |
| Laptop client (config, rules, bridge, commands, model upload/activate, input mapping, listen) | `python tools\node_client.py --broker <ip> --node node1 <command>`; offline check: `... node_client.py selftest` (tested against a local broker + mock node in `tests/client_mqtt`: retries after a lost ACK, out-of-order recovery, exact bytes; not yet against the real firmware over Wi-Fi) |
| Documentation | `docs/requirements_traceability.md` (AC-01..AC-17 status), `architecture.md`, `mqtt_protobuf_protocol.md`, `sensor_profiles.md`, `ml_model_contracts.md`, `demo_and_benchmarks.md` |
| Reference (non-accelerated) build for AC-16 | `idf.py -B build_ref "-DSDKCONFIG=build_ref/sdkconfig" "-DSDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.ref.defaults" build` - builds, 385,712 B vs 393,856 B with ESP-NN; timing/accuracy comparison needs a board |
| Step-by-step hardware guide (setup, MQTT broker, wiring, bring-up, demo, data collection, troubleshooting) | `docs/STEP_BY_STEP_GUIDE.md` (commands that can run without a board were executed; wiring/Mosquitto steps are unverified) |
| Record real sensor data and retrain | `tools/collect_readings.py` -> CSV -> `ml/train_motion.py --data DIR --out DIR --version N` (tested with simulated recordings: format, gap/rate rejection, by-session split; no real data yet) |
| Console bring-up aids | `WATCH <id\|ALL\|OFF>` prints readings; `python tools\sensor_profiles.py <sensor\|A\|B> <gen> --hex` makes a `CFG` line for one sensor |
| Wi-Fi / broker settings | `idf.py menuconfig` -> "AAI16 node" (empty SSID = serial console only) |

## Evidence by layer

| Layer | Files | Host-tested | Target-built | Emulated (QEMU) | Board |
|---|---|---|---|---|---|
| Bounded Protobuf schema | `protocol/` | yes (nanopb vs Python protobuf) | yes | - | - |
| Config manager (validate, generations, ACKs, rollback, persist/restore) | `components/config_manager/` | 122 checks | yes | accepts good / refuses bad config | - |
| Configurable decoding (bit fields, signs, checksums, UART sync, calibration, RPN) | `components/sensor_decode/`, `tools/profile_lib.py`, `tools/sensor_profiles.py` | 44 cases; BME280 reproduces Bosch's 25.08 °C / 100653.27 Pa from config | yes | - | - |
| Executor + per-bus scheduler | `components/sensor_runtime/` | 44 checks (simulated devices) | yes | runs 4 sensorless buses 4+ s without hang/crash | - |
| ML: 2 models (motion, environment), packages, contracts | `ml/` | features match Python; quantization identical to TFLite 225/225 | - | - | - |
| ML logic: selector, model store, SHA-256 | `components/ml_core/` | 4315 checks | yes | - | - |
| Rule engine, bridge, heartbeat, REPL | `components/node_services/` | 152 checks | yes | REPL commands verified | - |
| Node core (wiring, fan-out, model flow, recovery incl. rules / bridge / ML mapping / model generation) | `components/node_core/` | 487 checks; **8 inferences matched recorded int8 tensors of the real trained models** (only the matrix multiply is a double) | yes | boots, commands work | - |
| ESP-IDF platform layer: I2C/UART/SPI/GPIO drivers, NVS + flash banks, rule outputs, Wi-Fi + MQTT, TFLM wrapper | `components/platform_esp/`, `main/` | n/a (hardware code) | **yes (393,728 B image; clean rebuild from scratch, no warnings from project code)** | boots; NVS write verified | **no** |

QEMU session (`tools/qemu_session.py`, 17 checks): boot, 8 MB PSRAM, NVS ready, HELP/SHOW, forbidden-pin config
refused, good config accepted and written to NVS (329 bytes), STOP/RUN, bad input rejected, scheduler stays
responsive and does not panic, CLEAR, REBOOT command restarts the firmware.

MQTT topic names are defined once (`components/node_core/node_topics.c`, used by the firmware) and checked in both directions against the laptop client (`tests/topics_host`, 28 cases).

Bugs that only target/emulated runs revealed (all fixed): 8 KB console buffer on a 6 KB stack (memory
corruption/crash); `spi_device_polling_transmit` could hang the node forever (now queued with a timeout and
static buffers); empty committed-config restore (found by the host node test); the model-bank erase interface
needed a size so a 3 KB model does not erase a 3 MB bank.

## Not verified / not archived (do not claim)

Hardware evidence in this project is **live session output** from two boards; apart from the CSV recordings in `data/anomaly/`
it was not saved as log files. Treat it as reported, not archived, until logs are added.

- **Reboot persistence.** Observed on a real board (node1 restored its configuration, dormant profiles, rules, bridge settings and
  model after a reboot). **QEMU still does not keep flash across a restart** (`tests/qemu_nvs_probe` reads `boots=0`), so this
  cannot be shown in the emulator.
- **Sensors.** Seven of the eight sensor profiles were observed working on boards (GY-511, DHT22, BME280, MAX6675, MH-Z19B, LD2450,
  RC522, the last after adding the receiver-gain write for its clone chip). **The HC-SR04 is team-reported only.** A line-by-line
  datasheet check of every register, scale and polarity has not been done. The DHT22 (polling bit capture with interrupts off for
  about 5 ms, `plat_bus.c`) times out on some reads when several sensors start together on one node; the cause is not isolated.
- **Wi-Fi/MQTT.** Works on two boards (configs, rules, bridge, model chunks, telemetry). A ConfigApply near the 28 KB maximum was
  never sent to a board, and an interrupted model upload was never tried on hardware.
- **TFLM on the chip.** The motion, environment and anomaly models load and run (about 200-340 us per inference with ESP-NN). Not done:
  the ESP-NN versus reference timing comparison (the reference image builds and contains no ESP-NN kernels, but was never timed),
  logit-level equality between board and PC, and memory/latency benchmarks as archived evidence (AC-16, AC-17).
- **Timing.** A 12-minute recording (`data/anomaly/normal_01.csv`, analysed with `tools/analyze_sampling_timing.py`) shows no skipped
  sampling interval on four streams (worst lateness 10.8 ms). That does not cover model uploads, the node task's execution time or
  interrupt-off time, and zero detected skipped intervals does not by itself prove zero deadline violations.
- **Model quality.** The motion and environment models were trained on **synthetic** data; the anomaly model has a version trained
  partly on about 12 minutes of real recordings (temperature, humidity, pressure, equipment temperature only). No field accuracy claim.
- Persistence of rules, bridge settings, ML input mapping and the model generation counter is host-tested (including a damaged-state
  case) and its reboot restore was observed on node1.

## Findings that need a decision

1. **Memory:** `node_t` ≈ 150 KB (allocated in PSRAM); worst-case `ConfigApply` 28 KB (MQTT reassembly buffer;
   `NODE_BLOB_MAX` is 12 KB, so a larger committed config would fail to persist).
2. **Duplicate generation** is rejected; if an APPLIED ACK is lost the laptop sees a rejection although the
   config is active. Alternative: re-send APPLIED for a byte-identical retry.
3. **Pin allow-list** (`cm_pin_allowed`) is a conservative choice for the N16R8; verify on the board.
4. **Doubles** on the ESP32-S3 are software-emulated (used in decode/rules/features); cost unmeasured.
5. **Sample-rate coupling:** the ML selector rejects a sensor whose period deviates from the model's sample
   period (motion 20 ms ±25 %, environment 2000 ms ±30 %).
6. **Serial bus servicing:** buses run one after another in one task; a blocking op (DHT22 ~6 ms) delays others' timestamps.
7. **DHT22 bit timing** uses polling with interrupts disabled (~5 ms). RMT capture would be more robust; decide after board tests.
8. Open supervisor questions (legacy parity, acceleration scope, protocol scope, grading) remain open.

## Next steps

1. **Real board:** flash (`idf.py -p COMx flash`), check boot, persistence across reboot, then each sensor (verify
   profiles), Wi-Fi/MQTT with a broker, model upload and on-chip inference; record latency/arena/heap numbers.
2. Collect labelled GY-511 / DHT22 / BME280 windows and retrain; re-export; verify golden outputs on the board.
3. Persist rules, bridge and model generation; benchmark accelerated vs reference inference (ESP-NN build flag).
4. Update the `docs/` items (traceability, protocol, demo/benchmarks) with archived measurements.
