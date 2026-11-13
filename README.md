# AAI16 configurable ESP32-S3 edge node

INF2004 group AAI16. A **configuration-driven distributed IoT edge node** on an **ESP32-S3-N16R8**: sensors are described
by *configuration* (generic bus operations + decoding), every network message is bounded binary Protobuf over MQTT,
small int8 models run on the chip with TensorFlow Lite Micro, rules react to readings, and nodes can share sensors with
each other. Nothing about a sensor, a rule or a model needs a firmware rebuild.

| Capability | Where |
|---|---|
| Sensors as data: I2C / UART / SPI / GPIO engines, decoding, calibration maths, frame checks | `components/sensor_runtime`, `sensor_decode`, `tools/sensor_profiles.py` |
| Runtime configuration with generations, ACKs, rollback, persistence | `components/config_manager` |
| **Dormant sensors**: stored but not running, started (optionally for a set time) by a rule | `config_manager`, `node_services/rule_engine.c`, guide section 10c |
| Rule engine (RPN conditions, GPIO/PWM/notify/ML trigger/remote command/sensor start+stop) | `components/node_services/rule_engine.c` |
| Node-to-node bridge (a remote sensor becomes a virtual sensor 128..255) | `components/node_services/bridge.c` |
| On-chip ML: **motion**, **environment** and **anomaly** models, replaceable over MQTT | `ml/`, `components/ml_core`, `components/platform_esp/plat_ml.cc` |
| Anomaly model (Model C): 8 role slots, presence mask, per-slot error attribution | `ml/ANOMALY_SPEC.md`, `ml/anomaly.py` |
| Laptop tools: client, JSON helper for MQTTX, recorders, config shorthand | `tools/` |

**To build the hardware and run the demo, follow [docs/STEP_BY_STEP_GUIDE.md](docs/STEP_BY_STEP_GUIDE.md).**
[STATUS.md](STATUS.md) says exactly what has been verified (host tests, ESP-IDF build, real boards) and what has not.
The original specification is `ESP32-S3_Codex_Implementation_Handoff.md`.

## Repository layout

| Path | Contents |
|---|---|
| `protocol/` | `aai16.proto` + nanopb options: the wire contract (regenerate with `tools/generate_proto.py`) |
| `components/` | firmware components (`docs/architecture.md`); `platform_esp` is the only ESP-IDF-specific one |
| `main/` | `app_main` and the Kconfig menu (node id, Wi-Fi, broker) |
| `ml/` | training, int8 export, model packages and contracts (`ml/artifacts/`), real-data pipeline for the anomaly model |
| `tools/` | laptop client (`node_client.py`), JSON<->Protobuf helper (`json_bridge*.py`), sensor profiles, recorders, test runner, QEMU driver, MQTTX view scripts (`tools/mqttx/`) |
| `configs/` | ready-to-send JSON: one file per sensor, presets A/B, short form, rules, node bridge (see `configs/README.md`) |
| `tests/` | host test suites (C compiled with zig, Python references), client/broker test, QEMU NVS experiment |
| `data/anomaly/` | the sensor recordings used for / to test the real-data anomaly model |
| `docs/` | step-by-step guide, architecture, protocol, sensor profiles, model contracts, traceability, benchmark plan |

## Quick start (Windows, PowerShell)

```powershell
# one-time: host tooling (project-local venv)
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r tools\requirements.txt -r ml\requirements.txt

# everything that can be checked without a board (12 suites + client self-test)
.venv\Scripts\python.exe tools\run_all_tests.py

# firmware (ESP-IDF 6.1; tools\idf_env.ps1 sets session variables only - edit the paths inside it for your install)
. .\tools\idf_env.ps1
idf.py set-target esp32s3
idf.py menuconfig          # "AAI16 node": node id, Wi-Fi SSID/password, broker URI (kept in your local sdkconfig, never committed)
idf.py -p COMx flash monitor

# talk to a node over MQTT
.venv\Scripts\python.exe tools\node_client.py --broker <ip> --node node1 config A --gen 1
.venv\Scripts\python.exe tools\json_bridge_all.py --broker <ip>     # JSON mirror for MQTTX: aai16/<node>/json/#
```

A second node uses its own build directory and node id, for example
`idf.py -B build_node2 "-DSDKCONFIG=build_node2/sdkconfig" build` after setting `AAI16 node > Node id` to `node2`.

## Notes

- **Credentials:** the Wi-Fi SSID and password are entered with `idf.py menuconfig` and stored only in the git-ignored
  `sdkconfig`. The defaults in `main/Kconfig.projbuild` are empty on purpose.
- **ML data:** the motion and environment models are trained on synthetic data (`ml/README.md`). The anomaly model has a
  synthetic version and a version trained partly on real recordings; read `ml/README.md` before quoting any accuracy.
- **Reference project:** the design was informed by the previous group's FlexiEdgePico; that code is not part of this repository.
