# AAI16 Configurable ESP32-S3 Edge Node

INF2004 Embedded Systems Programming, group AAI16.

A **configuration-driven IoT edge node** for the **ESP32-S3-N16R8**. One firmware image is turned into different
devices at runtime: which sensors it reads and how to decode them, what rules it runs, which other node it listens to,
and which machine learning model it uses are all sent to it over **MQTT as bounded binary Protobuf**.
Nothing about a sensor, a rule or a model needs a rebuild or a reflash.

We demonstrate it with **FlexiCargo**, a two-board warehouse that switches from *receiving* to *storage* monitoring
without changing its firmware.

---

## Contents

1. [Features](#features)
2. [The FlexiCargo demo](#the-flexicargo-demo)
3. [Architecture](#architecture)
4. [Hardware and wiring](#hardware-and-wiring)
5. [Repository layout](#repository-layout)
6. [Quick start](#quick-start)
7. [Configuring a node over MQTT](#configuring-a-node-over-mqtt)
8. [Machine learning](#machine-learning)
9. [Testing](#testing)
10. [Security](#security)
11. [Status and limitations](#status-and-limitations)
12. [Documentation](#documentation)

---

## Features

| Capability | How | Code |
|---|---|---|
| **Sensors as data** | A sensor is a *profile*: bus (GPIO, I2C, UART, SPI), pins, a list of bus operations, and decode rules (bit fields, sign, scale, checksums, calibration formulas). There is no sensor-specific driver in the firmware. | `components/sensor_runtime`, `components/sensor_decode`, `tools/sensor_profiles.py` |
| **Runtime configuration** | Whole-message validation, generation numbers, `ADMITTED` / `APPLIED` / `REJECTED` acknowledgements, rollback, persistence in flash | `components/config_manager` |
| **Rule engine** | Up to 8 bounded rules: statistics over windows of readings, stack-based conditions, cooldowns; actions notify, set GPIO/PWM, trigger inference, command another node, start or stop a sensor | `components/node_services/rule_engine.c` |
| **Dormant sensors** | A sensor can be stored but not running, and started (optionally for a set time) by a rule | `config_manager`, `rule_engine.c` |
| **Node-to-node bridge** | Another node's readings become local *virtual sensors* (128 to 255) with a maximum age | `components/node_services/bridge.c` ([docs/bridging.md](docs/bridging.md)) |
| **On-device ML** | TensorFlow Lite Micro, int8 models: Motion, Environment and Anomaly | `components/ml_core`, `components/platform_esp/plat_ml.cc`, `ml/` |
| **Dynamic ML inputs** | Any sensor value of the right kind can feed a model input; wrong kinds are refused | `components/ml_core/ml_selector.c` |
| **Model replacement over MQTT** | Chunked upload into a spare flash bank, SHA-256 and contract checks, activation only after a successful load | `components/ml_core/model_store.c` |
| **Laptop tools** | JSON helper for MQTTX, client, demo driver, copy-paste pack, recorders | `tools/`, `demo/` |

---

## The FlexiCargo demo

| Node | Role | Mode A: receiving and handling | Mode B: storage and equipment |
|---|---|---|---|
| **node1** | sensing and ML node | GY-511 accelerometer, DHT22 temperature/humidity; Motion and Environment models | HC-SR04 distance, MAX6675 thermocouple; Anomaly model |
| **node2** | dock station | RC522 RFID reader, LD2450 radar (dormant until a tag is seen) | MH-Z19B CO2 (the RC522 keeps running) |

| Scene | Story moment | What it proves |
|---|---|---|
| 1 | A shipment arrives (RFID check-in) | sensor profiles as data; one trigger, two rule actions; a dormant sensor started by a rule |
| 2 | Activity near the receiving area | a monitoring zone defined as a rule |
| 3 | Two nodes working together | node-to-node bridge (virtual sensors) |
| 4 | Package movement | on-device inference with the Motion model |
| 5 | Receiving-area climate | windowed rules (minimum of the last 3 valid readings) |
| 6 | Switch the model | model change without reflashing (Environment) |
| 7 | Same model, different sensor | dynamic ML input selection |
| 8 | Reject an invalid mapping | value-kind checking keeps the last valid mapping |
| 9 | Storage mode on node1 | three buses reconfigured in one message |
| 10 | CO2 on node2 | a UART profile replaces another with a different protocol |
| 11 | New model over MQTT | chunked upload, SHA-256, activation, live anomaly inference |
| 12 | Evidence | inference time, health counters, honest limits |

Every message for every scene is ready in **`demo/copypaste/`**, one JSON file per message in demo order
(`ALL.md` lists them all with their topics), plus the model packages as hex chunks.
**Step by step: [docs/demo_guide.md](docs/demo_guide.md).**

---

## Architecture

```
 Laptop                                   MQTT broker (Mosquitto)                ESP32-S3 node (node1 / node2)
 MQTTX -- JSON --> json_bridge_all.py --> aai16/<node>/in/...   --Protobuf-->  plat_net.c (MQTT task, core 0)
                   (JSON <-> Protobuf)    aai16/<node>/out/...  <--Protobuf--        |  FreeRTOS queue (8)
                                                                                     v
                                                                   node_task (core 1, priority 5): owns all state
                                                                   +- config_manager  validate, generations, ACK, persist
                                                                   +- sensor_runtime  fixed-rhythm scheduler, 4 bus slots
                                                                   |    +- sensor_decode  bit fields, checksums, RPN maths
                                                                   +- rule_engine     windows, conditions, actions
                                                                   +- bridge          other nodes' readings -> virtual sensors
                                                                   +- ml_selector     input mapping, windows, features
                                                                   |    +- plat_ml.cc  TensorFlow Lite Micro (int8)
                                                                   +- model_store     chunks -> spare flash bank, SHA-256
                                                                   +- heartbeat / REPL
```

- **One owner, no locks.** `node_task` is the only task that touches sensors, rules, bridge, models and settings.
  MQTT and the serial console only hand it messages through a FreeRTOS queue, so there are no shared-data races.
- **Fixed-rhythm scheduling.** Each bus slot runs at its own period; the next due time is *previous due + period*,
  and missed slots are counted (`overruns` in the heartbeat) instead of bursting to catch up.
- **Interrupts and DMA where waiting matters.** UART sensors fill a ring buffer from the UART interrupt; SPI uses DMA.
- **Memory.** The node state lives in the 8 MB PSRAM; internal RAM stays free for DMA, Wi-Fi and the ML runtime.
- **Flash.** NVS for configuration, rules, bridge and ML mapping; two 3 MB model banks (`partitions.csv`).

Details: [docs/architecture.md](docs/architecture.md) and [docs/mqtt_protobuf_protocol.md](docs/mqtt_protobuf_protocol.md).

---

## Hardware and wiring

Two **ESP32-S3-N16R8** boards (16 MB flash, 8 MB octal PSRAM). All ESP32-S3 pins are **3.3 V only**.

| Sensor | Id | Node | Bus | ESP32-S3 pins | Power |
|---|---|---|---|---|---|
| GY-511 (LSM303DLHC accelerometer) | 1 | node1 | I2C, address 0x19 | SDA GPIO8, SCL GPIO9 | 3.3 V |
| DHT22 | 2 | node1 | GPIO (single wire) | DATA GPIO6 (pull-up if bare part) | 3.3 V |
| LD2450 radar | 3 | node2 | UART1, 256000 baud | ESP TX GPIO17 to module RX, ESP RX GPIO18 from module TX | 5 V |
| RC522 RFID | 4 | node2 | SPI | SCLK 12, MOSI 11, MISO 13, CS 14 | **3.3 V only** |
| BME280 | 5 | node1 | I2C, address 0x76 (0x77 if SDO high) | SDA GPIO8, SCL GPIO9 | 3.3 V |
| HC-SR04 | 6 | node1 | GPIO | TRIG GPIO4, ECHO GPIO5 **through a 1k/2k divider** | 5 V |
| MH-Z19B CO2 | 7 | node2 | UART2, 9600 baud | ESP TX GPIO15 to module RX, ESP RX GPIO16 from module TX | 5 V |
| MAX6675 thermocouple | 8 | node1 | SPI | SCLK 12, MISO 13, CS 10 | 3.3 V |

**One active profile per bus per node.** A node has four bus slots (GPIO, I2C, UART, SPI). Sending a profile for a
bus replaces what ran there; buses the message does not mention keep running.
Full wiring diagram and bring-up: [docs/STEP_BY_STEP_GUIDE.md](docs/STEP_BY_STEP_GUIDE.md) section 4.
Pins and demo thresholds used by the tools: `demo/rig.json`.

---

## Repository layout

```
AAI16_ESP32_proj/
|-- build_and_flash.ps1      one-time setup: venv, host tests, build node1 + node2, flash
|-- CMakeLists.txt           ESP-IDF project
|-- partitions.csv           NVS + factory app + two 3 MB model banks
|-- sdkconfig.defaults       ESP32-S3-N16R8, octal PSRAM, 1 kHz tick (no credentials)
|-- main/                    app_main, node task, console task, Kconfig menu (node id, Wi-Fi, broker)
|-- components/
|   |-- aai16_protocol/      generated nanopb code + nanopb runtime
|   |-- config_manager/      validation, generations, ACKs, rollback, persistence
|   |-- sensor_runtime/      bus-operation executor and per-bus scheduler
|   |-- sensor_decode/       data-driven decoding, checksums, calibration maths
|   |-- node_services/       rule engine, bridge, heartbeat, serial REPL
|   |-- ml_core/             SHA-256, features, input selector, model store
|   |-- node_core/           wires everything together; MQTT topic names
|   `-- platform_esp/        ESP-IDF drivers, NVS/flash banks, Wi-Fi/MQTT, TFLite Micro wrapper
|-- protocol/                aai16.proto + aai16.options (the wire contract)
|-- tools/                   JSON helper, client, sensor profiles, shorthand, recorders, test runner, build script
|-- demo/                    FlexiCargo demo driver (run.ps1, demo.py, rig.json) and the copy-paste pack
|-- configs/                 ready-to-send JSON examples (sensors, presets, short form, rules, bridge)
|-- ml/                      training, int8 export, model packages and contracts (ml/artifacts/)
|-- data/anomaly/            sensor recordings used to train and test the anomaly model
|-- tests/                   host test suites (C compiled with zig, Python references)
`-- docs/                    guides and reference documents
```

---

## Quick start

### Prerequisites (Windows)

| Tool | Version | Used for |
|---|---|---|
| Python | 3.10 or newer | laptop tools, host tests, ML (installed into `.venv` automatically) |
| ESP-IDF | 6.1 | firmware build and flash (open an ESP-IDF PowerShell, or adjust `tools/idf_env.ps1`) |
| Mosquitto | 2.x | MQTT broker on the laptop, port 1883 |
| MQTTX | any recent | sending and watching messages ([docs/mqttx_setup.md](docs/mqttx_setup.md)) |

The laptop and both boards must be on the same **2.4 GHz** Wi-Fi network (a phone hotspot works).

### 1. Build and flash both boards (one command)

```powershell
git clone https://github.com/limxinchian/AAI16_ESP32_proj.git
cd AAI16_ESP32_proj
.\build_and_flash.ps1
```

It asks for the two COM ports, the Wi-Fi name and password, and the broker address (the laptop's IP on that Wi-Fi),
then creates `.venv`, runs every host test, builds `node1` and `node2` into their own folders, checks the build settings
and flashes both boards. Later rebuilds remember the network settings:

```powershell
.\build_and_flash.ps1 -Node1Port COM12 -Node2Port COM11 -SkipTests
```

Advanced options (reference build without ESP-NN, one node only, tests only): `Get-Help .\tools\build_all.ps1 -Full`.

### 2. Start the broker and the JSON helper

Mosquitto needs a configuration with `listener 1883 0.0.0.0` and `allow_anonymous true` (local hotspot only; see
[Security](#security)). Then, in the project folder:

```powershell
.\.venv\Scripts\python.exe tools\json_bridge_all.py --broker 127.0.0.1
```

### 3. Open MQTTX and run the demo

Follow [docs/mqttx_setup.md](docs/mqttx_setup.md) for the connection and subscriptions, then
[docs/demo_guide.md](docs/demo_guide.md). The demo driver offers shortcuts for preparation and checks:

```powershell
.\demo\run.ps1 status         # both nodes: uptime, running sensors, model, heap, counters
.\demo\run.ps1 check-wiring   # probe every sensor on its own (before the demo)
.\demo\run.ps1 models         # upload + activate Motion, stage Environment
.\demo\run.ps1 watch          # live view of both nodes
```

---

## Configuring a node over MQTT

The boards only accept Protobuf. You write JSON in MQTTX; the helper converts it, the node validates and applies it,
and the reply comes back as JSON:

```
MQTTX  aai16/node2/json_in/config   ->  helper (JSON -> Protobuf)  ->  aai16/node2/in/config       ->  node2
MQTTX  aai16/node2/json/config_ack  <-  helper (Protobuf -> JSON)  <-  aai16/node2/out/config_ack  <-  node2
```

| Topic (`aai16/<node>/...`) | Message | Purpose |
|---|---|---|
| `in/config` | `ConfigApply` | sensor profiles, one per bus |
| `in/rules` | `RuleSetApply` | the node's whole rule set (replaces the old one) |
| `in/bridge` | `BridgeApply` | other nodes' readings to use as virtual sensors |
| `in/model/chunk` | `ModelChunk` | one piece (up to 1 KB) of a model package |
| `in/model/control` | `ModelControl` | `ACTIVATE`, `SET_INPUTS`, `QUERY`, `ABORT_TRANSFER` |
| `in/command` | `NodeCommand` | `RUN`, `STOP`, `CLEAR` |
| `out/config_ack`, `out/rules_ack`, `out/bridge_ack` | `ApplyAck` | `ADMITTED`, `APPLIED` or `REJECTED` with a reason |
| `out/model_chunk_ack`, `out/model_status`, `out/inference` | `ChunkAck`, `ModelStatus`, `InferenceResult` | model upload, state and results |
| `out/reading/<id>`, `out/notification`, `out/heartbeat` | `Reading`, `Notification`, `Heartbeat` | telemetry (readings at QoS 0, everything else QoS 1) |

**Example: sensors in short form.** The helper expands each name into the full profile (`tools/config_shorthand.py`):

```json
{"generation": "auto", "sensors": [{"sensor": "rc522", "period_ms": 200}, {"sensor": "ld2450", "dormant": true}]}
```

**Rules of the protocol**
- Every message is validated as a whole before anything changes; a rejection always carries a reason and leaves the
  previous state running.
- Each kind of change has a **generation number**. Older or repeated generations are refused. For config, rules and
  bridge write `"generation": "auto"`; for model control use the last `model_status` generation + 1.
- `APPLIED` means the configuration is running; it does not prove a sensor is wired. That shows as valid readings.

Reference: [docs/mqtt_protobuf_protocol.md](docs/mqtt_protobuf_protocol.md), [docs/sensor_profiles.md](docs/sensor_profiles.md),
[configs/README.md](configs/README.md).

---

## Machine learning

| Model | Type | Network | Input | Output | Package |
|---|---|---|---|---|---|
| **Motion** | classifier | 11 -> 16 -> 8 -> 3 (ReLU, softmax) | 11 features of 64 accelerometer samples, 20 ms apart | STATIONARY / MOVING / VIBRATING | 3.7 KB |
| **Environment** | classifier | 6 -> 12 -> 8 -> 2 | 6 trend features of 20 temperature/humidity samples, 2 s apart | STABLE / CHANGING | 3.4 KB |
| **Anomaly** | autoencoder | 32 -> 24 -> 12 -> 24 -> 32 | 4 features for each of 8 sensor roles; absent roles masked | NORMAL / ANOMALY (reconstruction error vs. threshold) | 7.6 KB |

- Trained with TensorFlow/Keras, exported as **full-integer int8** TensorFlow Lite, run with **TensorFlow Lite Micro**
  (ESP-NN accelerated). Measured on the board: about 340 us (Motion) and 360 us (Environment) per inference.
- Each package carries a **contract**: input shape and scaling, sample period, maximum input age and the **value kind**
  of every input. The node maps sensors to inputs only when kinds and timing match.
- Models live in **two flash banks**: an upload always goes to the spare bank, is checked with SHA-256 and against its
  contract, and is activated only after the runtime has loaded it. A failed upload never touches the running model.
- Motion and Environment were trained mostly on **synthetic** data, the Anomaly model partly on real recordings
  (`data/anomaly/`). No field accuracy is claimed.

Training and data: [ml/README.md](ml/README.md), [docs/ml_model_contracts.md](docs/ml_model_contracts.md), [ml/ANOMALY_SPEC.md](ml/ANOMALY_SPEC.md).

---

## Testing

All firmware logic is tested on the laptop, without a board: the C components are compiled for the host and driven by
Python with independent reference code.

```powershell
.\.venv\Scripts\python.exe tools\run_all_tests.py
```

| Suite | Covers | Checks |
|---|---|---|
| `config_host` | configuration validation, generations, rollback, persistence | 122 |
| `decode_host` | data-driven decoding against independent references (Bosch BME280 formulas) | 49 |
| `runtime_host` | bus executor and scheduler on simulated devices, faults, overruns | 44 |
| `services_host` | rule engine, bridge, heartbeat, REPL | 152 |
| `ml_host` | SHA-256, features vs. golden vectors, packages, input selector, model store | 4,315 |
| `node_host` | the whole node with the real trained models (8 inferences match recorded TFLite tensors) | 487 |
| `anomaly_host` | anomaly model statistics, mappings, scenarios | 103 |
| `dormant_host` | dormant sensors started and stopped by rules | 1,058 |
| `topics_host` | firmware and laptop agree on every MQTT topic | 28 |
| `client_mqtt` | laptop client against a real broker and a mock node | 9 groups |
| `real_data_host` | recording -> CSV -> training -> package path | 1 path |
| `shorthand_host` | short JSON config form | 37 |
| `demo_pack_host` | every demo message and config example passes the firmware's own validators; generations, transfer ids, chunks, ML kinds | 134 |

All 13 suites pass. The build script also checks each firmware image (node id, broker, PSRAM, tick rate, ESP-NN).
Test cases and the manual board checks: [docs/testing.md](docs/testing.md).

---

## Security

- **No credentials in the repository.** The Wi-Fi SSID and password are entered when building and stored only in the
  git-ignored build folders (`build_*/`) or a local `sdkconfig`; the Kconfig defaults are empty and the build script
  never prints the password.
- **Generated and local files are ignored:** `.venv/`, build output, `sdkconfig`, `managed_components/`, editor
  settings, `demo/.state.json`, recordings other than `data/anomaly/`.
- **Broker:** the demo uses a local Mosquitto without authentication, which is acceptable on a private hotspot only.
  On a shared network, enable a password or TLS in Mosquitto and use the matching broker URI when building.
- **The node validates everything it receives:** bounded message sizes (nanopb maxima), whole-message validation,
  a pin allow-list, generation checks, SHA-256 on every model package.
- **Third-party code:** nanopb is included under its own licence (`components/aai16_protocol/nanopb/LICENSE.txt`);
  TensorFlow Lite Micro and ESP-NN come from the ESP-IDF component manager (versions pinned in `dependencies.lock`).
  The previous group's FlexiEdgePico code informed the design but is not part of this repository.

---

## Status and limitations

Verified on two boards: Wi-Fi/MQTT with Protobuf for every message type, all eight sensors through configuration only
(the HC-SR04 team-reported), on-chip inference of all three models, the bridge, dormant sensors, and persistence across
a reboot. A 12-minute recording showed no skipped sampling interval on four streams (worst lateness 10.8 ms).

Not done or not claimed: ESP-NN versus reference timing, sampling behaviour during a model upload, model accuracy on
real data, long-run stability. Known weaknesses from our fail-safe review: SPI devices can fail silently (an unplugged
RC522 reads as "no card"), one slow sensor can delay others on the shared task, and the task watchdog is configured to
warn rather than reset. Full evidence table: [docs/project_status.md](docs/project_status.md).

---

## Documentation

| Document | Contents |
|---|---|
| [docs/demo_guide.md](docs/demo_guide.md) | the FlexiCargo demo, scene by scene |
| [docs/mqttx_setup.md](docs/mqttx_setup.md) | MQTTX connection, subscriptions, QoS, publishing, common mistakes |
| [docs/STEP_BY_STEP_GUIDE.md](docs/STEP_BY_STEP_GUIDE.md) | hardware setup from scratch: broker, wiring, bring-up, data collection |
| [docs/testing.md](docs/testing.md) | test suites, test cases, board checks |
| [docs/architecture.md](docs/architecture.md) | components, flows, concurrency and memory |
| [docs/mqtt_protobuf_protocol.md](docs/mqtt_protobuf_protocol.md) | topics, messages, generations, acknowledgements |
| [docs/bridging.md](docs/bridging.md) | how node-to-node bridging works |
| [docs/sensor_profiles.md](docs/sensor_profiles.md) | the eight sensors as data |
| [docs/ml_model_contracts.md](docs/ml_model_contracts.md) | model contracts and input matching |
| [docs/requirements_traceability.md](docs/requirements_traceability.md) | requirements against evidence |
| [docs/project_status.md](docs/project_status.md) | what has been verified, and how |
| [docs/demo_and_benchmarks.md](docs/demo_and_benchmarks.md) | measurements and benchmark plan |
