# Ready-to-send JSON configurations

Plain JSON for the laptop helper. Run `python tools/json_bridge_all.py --broker <ip>` and publish a file's content to the
matching `aai16/<node>/json_in/<topic>` topic in MQTTX, or send a config file directly with
`python tools/send_json_config.py --broker <ip> --node node1 <file>`. `"generation": "auto"` means "the next one".

| Folder | Topic | What it is |
|---|---|---|
| `sensors/` | `config` | one full-form profile per sensor (`gy511`, `dht22`, `ld2450`, `rc522`, `bme280`, `hcsr04`, `mhz19b`, `max6675`) |
| `presets/` | `config` | `config_A` (DHT22, GY-511, LD2450, RC522) and `config_B` (HC-SR04, BME280, MH-Z19B, MAX6675) |
| `short_form/` | `config` | the same sensors in the short form (`{"sensor": "gy511"}`), plus examples: change interval, custom pins, disable, **dormant** |
| `rules/` | `rules` | rules: a card tap (RC522) starts dormant sensors on node1 / node2 (`duration_ms` 0 = run until stopped) |
| `bridge/` | `bridge` | node1 listens to node2's LD2450 (virtual sensor 130) and RC522 (virtual sensor 131) |

Sensor ids: 1 GY-511, 2 DHT22, 3 LD2450, 4 RC522, 5 BME280, 6 HC-SR04, 7 MH-Z19B, 8 MAX6675.
Rules replace the whole rule set of a node, so a rules file must contain every rule you want to keep.

Tap-wakes-everything demo (RC522 on node2, everything else stored dormant):
1. `short_form/node2_rc522_running_rest_dormant.json` to node2, `short_form/node1_all_dormant.json` to node1.
2. `bridge/bridge_node1_listens_to_node2_ld2450_rc522.json` to node1.
3. `rules/rules_node2_tap_wakes_gy511_ld2450.json` to node2 and `rules/rules_node1_tap_wakes_all.json` to node1.
