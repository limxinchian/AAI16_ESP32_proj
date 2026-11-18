# Laptop tools

All Python tools run with the project venv: `.venv\Scripts\python.exe tools\<name>.py` (PowerShell).

| Tool | Purpose |
|---|---|
| `build_all.ps1` | one command: check tools, run the host tests, build (and optionally flash) each node, `-Reference` for the non-accelerated image |
| `run_all_tests.py` | every host test suite plus the client self-test |
| `node_client.py` | send config, rules, bridge, commands; upload/activate models and set ML inputs; `listen` |
| `json_bridge_all.py`, `json_bridge.py`, `json_monitor.py` | JSON mirror of the node's Protobuf topics for MQTTX (`aai16/<node>/json/#`, `json_in/<topic>`) |
| `send_json_config.py`, `config_shorthand.py` | send a JSON config file; short form (`{"sensor": "bme280"}`) expands to the full profile |
| `sensor_profiles.py`, `profile_lib.py` | the eight sensors as configuration data and the helpers that build profiles |
| `make_example_rules.py` | build a simple rule or bridge file |
| `record_slot_data.py`, `collect_readings.py` | record sensor readings to CSV (anomaly model / ML data) |
| `analyze_sampling_timing.py` | sampling accuracy from a recording: invalid readings, skipped intervals, lateness, deadline violations |
| `generate_proto.py` | regenerate the nanopb C code and the Python module from `protocol/aai16.proto` |
| `idf_env.ps1`, `qemu_session.py` | ESP-IDF shell helper; QEMU session driver |
| `mqttx/` | optional MQTTX "Run Script" viewers (the JSON helper is usually easier) |
