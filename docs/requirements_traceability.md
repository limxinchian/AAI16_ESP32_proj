# Requirements traceability (acceptance matrix)

Evidence levels: **H** host-tested (PC, simulated devices) - **T** target-built (ESP-IDF 6.1, esp32s3) - **Q** ran in QEMU -
**B** board-tested on two ESP32-S3 boards. Status as of 2026-11-18.

How to read the **B** column: *observed* = seen on a board in live sessions (not archived as log files, apart from the CSV
recordings in `data/anomaly/`); *reported* = the team observed it in an earlier session and it was not re-observed for this
record; *no* = never tried on hardware. Host tests are not hardware evidence, and mocked inference is not on-chip inference.

| ID | Requirement | Where | H | T | B (board) | Remaining gap |
|---|---|---|---|---|---|---|
| AC-01 | Four generic buses (GPIO/I2C/UART/SPI) | `sensor_runtime`, `plat_bus.c` | yes (simulated devices) | yes | observed on I2C, UART, SPI and GPIO bit-frame (DHT22); GPIO pulse-width (HC-SR04) reported | archived logs; bus-analyser capture |
| AC-02 | Eight sensors, four active, swap A<->B without reflash | `tools/sensor_profiles.py`, config manager, node core | yes: config A -> swap one bus -> config B | yes | seven sensors observed; swaps on a bus observed (including dormant -> running); HC-SR04 reported | one recorded pass/fail swap run (GY-511 <-> BME280); HC-SR04 re-check |
| AC-03 | Generic command/read: change bytes/length/address -> different transaction | profiles are data (`Op` list) | yes: mutated profiles change executed ops | yes | observed: RC522 profiles changed over MQTT (version-register read, register diagnostic, receiver-gain init write) changed the real SPI transactions with no reflash | a bus-analyser capture |
| AC-04 | Independent periods; requested vs actual | `sensor_runtime` scheduler | yes: 20 ms/2 s/100 ms/200 ms give 500/5/100/50 readings per 10 s | yes | measured from `data/anomaly/normal_01.csv` (`tools/analyze_sampling_timing.py`): 100/250/500/2000 ms streams, 0 skipped intervals, worst lateness 10.8 ms; GY-511 about 50 readings/s observed | timing during a model upload; zero skipped intervals is not proof of zero deadline violations |
| AC-05 | Protobuf change requirement | `protocol/aai16.proto`, `.options` | yes: Python-encoded fixtures decode in nanopb and re-encode byte-identically; oversize/malformed rejected | yes | observed: all node traffic is binary Protobuf; JSON exists only in the laptop helper | none |
| AC-06 | Reliable config application (stale/invalid rejected with reason, prior config kept, generation/ACK) | `config_manager` | yes (122 checks, rollback on engine and persist failure) | yes | observed: ADMITTED/APPLIED ACKs and a REJECTED (malformed) config; rollback not observed | lost-ACK retry policy undecided (STATUS finding 2) |
| AC-07 | Real Motion Model A: GY-511 window -> on-target inference -> MQTT | `ml/`, `ml_core`, `plat_ml.cc` | partial: node pipeline matches recorded TFLite tensors (matrix multiply is a test double) | yes | reported (earlier session): motion model inference on the board | archived run; logit equality board vs PC |
| AC-08 | Real Environment Model B: DHT22, then BME280, same model | same | partial: same as AC-07, incl. DHT22 -> BME280 switch | yes | observed: environment model running on node1 (about 340 us per inference); DHT22 -> BME280 switch reported | recorded switch run |
| AC-09 | Wrong sensor/model rejected before inference; previous mapping kept | `ml_selector` | yes (each incompatibility has its own reason; mapping preserved) | yes | no | a board run of a refused mapping |
| AC-10 | Actual model replacement over the network, no reflash | `model_store`, `plat_storage.c`, `tools/node_client.py` | yes: chunked upload -> verify -> stage -> activate in the other bank | yes | observed: anomaly model v1 then v2 uploaded and activated over MQTT | an interrupted upload on hardware |
| AC-11 | Upload robustness (out of order, duplicate, lost, invalid, corrupt); active model protected | `model_store`, node core | yes (all of these, plus persistence failure after the runtime accepted the model) | yes | no | interrupted Wi-Fi on hardware |
| AC-12 | Rule engine: runtime update, bounded action / inference trigger | `rule_engine` | yes (services suite, 152 checks, incl. pin policy re-checked at fire time) | yes | observed: threshold rule on a bridged sensor, edge-triggered tap rules, sensor start/stop; GPIO/PWM outputs not tried | GPIO/PWM outputs on a board |
| AC-13 | Serial REPL RUN/STOP/SHOW through the same managers | `repl`, `node_core` | yes | yes | Q: all commands on the real image; B: serial console used for boot logs, REPL commands not exercised on a board | REPL on a board |
| AC-14 | Node-to-node bridge, time-bounded virtual input | `bridge` | yes: virtual sensor 128..255, freshness, loop guards | yes | observed: node1 uses node2's LD2450 (130) and RC522 (131); rules fire on virtual sensors | latency measurement |
| AC-15 | Heartbeat / diagnostics / persistence and restore | `diag`, `config_manager`, `plat_storage.c` | yes: heartbeat fields; restore after reboot on simulated storage | yes | observed: heartbeat fields; node1 reboot restored configuration, dormant profiles, rules, bridge and model | persistence failure paths on hardware |
| AC-16 | Hardware optimisation (if confirmed): reference vs optimised kernels genuinely used and measured | `sdkconfig`: `CONFIG_NN_OPTIMIZED=y`, `CONFIG_ESP_TFLITE_MICRO_USE_ESP_NN=y`; `sdkconfig.ref.defaults` | no | optimised and reference images build; the reference ELF has 0 `esp_nn_` symbols, the optimised ELF has 14 (nm check) | no timing comparison | time the same model on both images; the supervisor must confirm scope |
| AC-17 | Benchmarks with raw data and method | `docs/project_status.md`, `tools/analyze_sampling_timing.py` | partial | - | partial: sampling accuracy (saved CSV); inference times observed but not archived | archived inference timings, memory and soak runs |
| extra | Dormant sensors started/stopped by rules | `config_manager`, `rule_engine`, `node_core` | yes (`tests/dormant_host`, 1058 checks) | yes | observed: a card tap starts stored sensors on both nodes; they run until stopped (or for a set time) | none |
| extra | Anomaly model (Model C), optional, outside the two-model scope | `ml/anomaly.py`, `ml_core` | yes (`tests/anomaly_host`, 103 checks) | yes | observed: runs on node1 with partial sensor mappings | synthetic/limited real data |

## Supervisor's recorded expectations (3 November 2026 transcript)

| Expectation | Covered by |
|---|---|
| Replace the entire ML model over the network | AC-10 |
| Choose the sensor data that feeds inference independently of the model | `ModelControl SET_INPUTS`, AC-08/AC-09 |
| Configured command bytes and response lengths instead of a sensor-name driver | AC-03 (`Op` list, no sensor-name code) |
| A believable use case | FlexiCargo warehouse monitoring (`README.md`, `docs/demo_guide.md`) |
| Explain the exact components and how they connect | `docs/architecture.md` |
| Not confirmed by the supervisor: accelerated vs non-accelerated inference, powered hot-plugging | AC-16 (both images build); hot-plugging is not claimed |

## Integrity questions

- Existing files reused: none (the project started from the written specification only); everything is new, FlexiEdgePico informed the rule/REPL/bridge design only.
- Toolchain actually used: ESP-IDF v6.1 (`C:\esp\v6.1`), Xtensa GCC 15.2.0, target esp32s3; TFLM = `espressif/esp-tflite-micro` + ESP-NN via the component manager (versions pinned in `dependencies.lock`); nanopb 0.4.9.1.
- Sensor-name logic in application code: none; acquisition is the generic `Op` interpreter. (`plat_bus.c` implements generic bus primitives only.)
- Real inference on hardware: observed (see AC-07/08/10); not archived as logs.
- Open supervisor questions from the specification: still open.
