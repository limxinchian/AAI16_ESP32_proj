# Ready-to-send JSON examples

Small examples of each message type, in the form the laptop JSON helper accepts. With the helper running
(`.\.venv\Scripts\python.exe tools\json_bridge_all.py --broker 127.0.0.1`), publish a file's content in MQTTX to
`aai16/<node>/json_in/<topic>`. `"generation": "auto"` means "the next one". The full FlexiCargo demo has its own
ready-made messages in `demo/copypaste/`.

| File | Topic | What it shows |
|---|---|---|
| `short_form/config_A.json` | `config` | preset A: DHT22, GY-511, LD2450, RC522 in one message |
| `short_form/config_B.json` | `config` | preset B: HC-SR04, BME280, MH-Z19B, MAX6675 in one message |
| `short_form/example_change_interval.json` | `config` | change one sensor's sampling interval |
| `short_form/example_custom_pins.json` | `config` | override pins and an I2C address |
| `short_form/example_disable_sensor.json` | `config` | switch one sensor off (its bus slot stops) |
| `short_form/gy511_dormant.json` | `config` | store a sensor as **dormant**: kept, not running, a rule can start it |
| `rules/rules_tap_wakes_gy511.json` | `rules` | when the RC522 sees a tag, start the dormant GY-511 for 60 s |
| `bridge/bridge_node1_listens_to_node2_ld2450_rc522.json` | `bridge` | node1 uses node2's LD2450 (virtual 130) and RC522 (virtual 131) |

Short form: a sensor name expands to its built-in profile (`tools/sensor_profiles.py`), with optional overrides
(`period_ms`, `enabled`, `dormant`, pins, address). Sensor ids: 1 GY-511, 2 DHT22, 3 LD2450, 4 RC522, 5 BME280,
6 HC-SR04, 7 MH-Z19B, 8 MAX6675. A rules message replaces the node's whole rule set.
Every file here is checked against the firmware's validators by `tests/demo_pack_host`.
