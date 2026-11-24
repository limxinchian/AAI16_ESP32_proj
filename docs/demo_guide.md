# FlexiCargo demo guide

FlexiCargo is a small warehouse that changes its monitoring task between two modes **with the same firmware**.
Everything is changed over MQTT: sensor profiles, rules, the bridge between nodes, ML input mappings and the ML model
itself. Nothing is reflashed.

| Node | Role | Mode A: receiving and handling | Mode B: storage and equipment |
|---|---|---|---|
| **node1** | sensing and ML node | GY-511 accelerometer, DHT22 temperature/humidity, Motion and Environment models | HC-SR04 distance, MAX6675 thermocouple, Anomaly model |
| **node2** | dock station | RC522 RFID reader, LD2450 radar (dormant until a tag is seen) | MH-Z19B CO2 (RC522 keeps running) |

Every message below is a file in `demo/copypaste/` (also collected, with its topic, in `demo/copypaste/ALL.md`).
Publish each one in MQTTX to the topic shown, payload format **JSON**, QoS 1, unless stated. MQTTX setup and
subscriptions: [mqttx_setup.md](mqttx_setup.md). Wiring: [STEP_BY_STEP_GUIDE.md](STEP_BY_STEP_GUIDE.md) section 4.

---

## 0. Before the demo

1. **Flash both boards** (once): `.\build_and_flash.ps1` (see the README, Quick start).
2. **Start Mosquitto**, then the JSON helper: `.\.venv\Scripts\python.exe tools\json_bridge_all.py --broker 127.0.0.1`.
3. **Open MQTTX** with the subscriptions from [mqttx_setup.md](mqttx_setup.md).
4. **Clear both nodes**: `01_s00_node1_command_clear.json` to `aai16/node1/json_in/command` and
   `02_s00_node2_command_clear.json` to `aai16/node2/json_in/command`. Within 10 s each heartbeat shows
   `active_sensors: []`. CLEAR stops every sensor and removes rules and bridge; it does not touch ML models.
5. **Put the models on node1**: Motion **active**, Environment **staged**. Either:
   - one command: `.\demo\run.ps1 models` (uploads both, activates Motion). It saves the Environment transfer id in
     `demo\.state.json` (`env_transfer`); use that number in Scene 6 instead of 7002; or
   - paste the hex chunks in MQTTX (`demo/copypaste/chunks/`, topic `aai16/node1/in/model/chunk`, format **Hex**):
     Motion chunks 1..4, then `08_s04_..._motion_activate_only_if_chunks_pasted.json`, then Environment chunks 1..4.
   **Do not reboot node1 after staging Environment**: only the active model survives a reboot.

**Model generation numbers.** The pack's model-control messages use 2000000, 2000001, ... in demo order. Each node
refuses any model message whose generation is not higher than the last accepted one (it remembers it across reboots).
If a message is refused with `STALE_GENERATION`, send `25_s99_node1_model-control_query_model_generation.json` and use
the `generation` in the reply + 1. Config, rules and bridge use `"generation": "auto"` and never need this.

---

## Scene overview

| Scene | Story moment | Shows | Time |
|---|---|---|---|
| 1 | A shipment arrives (RFID check-in) | configurable profiles, rule with two actions, dormant sensor started by a rule | 2.5 min |
| 2 | Activity near the receiving area | a monitoring zone defined as a rule | 1.5 min |
| 3 | Two nodes working together | node-to-node bridge (virtual sensors 130/131) | 2 min |
| 4 | Package movement | on-device ML inference (Motion) | 2 min |
| 5 | Receiving-area climate | windowed rule (minimum of 3 valid readings) | 1 min |
| 6 | Switch the model | model change without reflashing (Environment) | 2 min |
| 7 | Same model, different sensor | dynamic ML input selection | 2 min |
| 8 | Reject an invalid mapping | value-kind checking | 1 min |
| 9 | Storage mode (node1) | three buses reconfigured in one message | 2.5 min |
| 10 | CO2 on node2 | a UART profile replaces another | 1.5 min |
| 11 | New model over MQTT | chunked upload, SHA-256, activation, live inference | 3 min |
| 12 | Evidence and closing | timing, health counters, honest limits | 2 min |

---

## Scene 1: A shipment arrives (RFID check-in)

**Send**
1. `03_s01_node2_config_modeA.json` to `aai16/node2/json_in/config`: RC522 on SPI every 200 ms; LD2450 stored as **dormant**.
2. `04_s01_node2_rules_modeA.json` to `aai16/node2/json_in/rules`: three rules.

| Rule | Watches | Fires when | Action |
|---|---|---|---|
| 1 | RC522 `card_present` (sensor 4) | rises above 0.5, 10 s cooldown | notify `PACKAGE_CHECK_IN_DETECTED` |
| 2 | the same value | the same condition | **start the dormant radar** (sensor 3) |
| 3 | LD2450 x and y (sensor 3) | \|x\| < 500 mm and 0 < y < 1500 mm, 5 s cooldown | notify `RECEIVING_ZONE_ACTIVITY` |

**Do:** hold the tag flat on the RC522 for about a second.

**Expect:** `card_present: 1` on `reading/rc522`, the notification on `node2_notification`, and `reading/ld2450` starting.

**If it fails:** check both acks say `APPLIED`. Rules sent to the `config` topic do nothing. A tap within 10 s of the
last one is ignored (cooldown). If taps are never detected although readings arrive, resend the config (the reader may
have reset and lost its gain setting; SPI cannot report that) and use `28_s99_node2_config_probe_rc522_version.json`
to check the chip answers (178 = our clone chip, 0 or 255 = no chip).

## Scene 2: Activity near the receiving area

Nothing to send: rule 3 was loaded in Scene 1. Walk in front of the radar about 0.5 to 1.5 m away.
**Expect** `RECEIVING_ZONE_ACTIVITY`. Change the zone by editing the 500 / 1500 constants in the rules and resending.

## Scene 3: Two nodes working together (the bridge)

**Send**
1. `05_s03_node1_config_modeA.json` to `aai16/node1/json_in/config`: GY-511 (I2C, 20 ms) and DHT22 (GPIO6, 2 s).
2. `06_s03_node1_rules_modeA.json` to `aai16/node1/json_in/rules`: handling advisory (> 1.5 g), temperature and
   humidity advisories (minimum of 3 readings), **rule 4** on virtual sensor 131 (`REMOTE_CHECK_IN_SEEN`) and
   **rule 5** on virtual sensor 130 (`REMOTE_RADAR_TARGET`).
3. `07_s03_node1_bridge_listen_node2.json` to `aai16/node1/json_in/bridge`: node2's radar becomes virtual sensor 130
   and its RC522 virtual sensor 131 on node1, each with a 1 s maximum age.

**Do:** wait at least 10 s after the last tap, then tap again.

**Expect:** node2 `PACKAGE_CHECK_IN_DETECTED` and node1 `REMOTE_CHECK_IN_SEEN`. The bridge shares data only;
virtual readings are not republished (type `WATCH 131` on node1's serial console to see them).
How it works: [bridging.md](bridging.md).

## Scene 4: Package movement (Motion model)

**Send** `09_s04_node1_model-control_motion_inputs.json` to `aai16/node1/json_in/model/control` (GY-511 x, y, z to
the model's three channels). If you pasted the chunks yourself, send `08_..._motion_activate_only_if_chunks_pasted.json` first.

**Expect** `model_status` `ACTIVE`, then about 1.3 s later `inference` results: `STATIONARY`, `MOVING` or `VIBRATING`
with `inference_us` (about 340 us). Keep the box still, move it, then vibrate it gently (the sensor's range is +-2 g).

**Note:** the model was trained mostly on synthetic data; demonstrate the pipeline, not accuracy.

## Scene 5: Receiving-area climate

Nothing to send. The DHT22 has run since Scene 3. Rules 2 and 3 alert only if the **minimum of the last 3 valid
readings** exceeds 32 C / 70 %RH; an invalid reading restarts the count. Breathing on the sensor may raise the humidity.

## Scene 6: Switch the model (Motion to Environment)

**Send**
1. `10_s06_node1_model-control_env_activate.json`. Use `transfer_id` **7002** if you pasted the chunks, or the
   `env_transfer` value from `demo\.state.json` if you used `.\demo\run.ps1 models`.
   **Expect** `MODEL_ENVIRONMENT`, `SUSPENDED`, `NO_INPUTS` (loaded, waiting for inputs).
2. `11_s06_node1_model-control_env_inputs_dht22.json` (DHT22 temperature and humidity).

**Expect** `ACTIVE`, then after about 40 s (20 readings, 2 s apart) `STABLE` or `CHANGING` (about 360 us per inference).

## Scene 7: Same model, different sensor

**Path A (BME280 working):** `12_s07_..._pathA_swap_to_bme280.json` to `.../config` (the I2C slot changes from GY-511
to BME280; the BME280 calibration is entirely in its profile), wait for valid `reading/bme280`, then
`13_s07_..._pathA_env_inputs_bme280.json`. The same model, same version, now takes its inputs from the BME280.

**Path B (BME280 not answering):** `14_..._pathB_i2c_off.json`, `15_..._pathB_max6675_2s.json` (both to `.../config`),
wait for valid `reading/max6675`, then `16_..._pathB_env_inputs_max6675.json`. Say clearly that this is not the I2C
swap. To diagnose the BME280, send `26_...probe_bme280_0x76.json` or `27_..._0x77.json`: the first value is the chip
id (96 = BME280, 88 = BMP280, `READ_BUS_ERROR` = nothing answers).

## Scene 8: Reject an invalid mapping

**Send** `17_s08_node1_model-control_badmap_humidity_as_temperature.json` (DHT22 humidity on the temperature channel)
while the Environment model is active with the DHT22 running.

**Expect** `REJECTED`, `VALUE_KIND_MISMATCH`, followed by `ACTIVE`: the previous mapping stays in force.

## Scene 9: Storage mode (node1)

**Send**
1. `18_s09_node1_config_modeB.json`: GPIO HC-SR04 (replaces the DHT22), SPI MAX6675 every 250 ms, I2C off.
   **Expect** the Environment model `SUSPENDED`, `SENSOR_NOT_ACTIVE` (its DHT22 is gone).
2. `19_s09_node1_rules_modeB.json`: rule 1 `LOADING_LANE_OBSTRUCTION` when the **maximum of the last 3 distances**
   is below 15 cm; rule 2 `EQUIPMENT_TEMPERATURE_ADVISORY` when the mean of 4 thermocouple readings is above 40 C.

**Do:** cardboard at about 25 cm, then about 10 cm, in front of the HC-SR04; touch the thermocouple to something warm.

## Scene 10: CO2 on node2

**Send** `20_s10_node2_config_modeB.json` (MH-Z19B replaces the radar on the UART slot: 9600 baud, command and reply
with checksum, instead of 256000 baud streaming) and `21_s10_node2_rules_modeB.json` (mean of 15 readings > 1200 ppm).

**Expect** `reading/mhz19b` with `co2_ppm` (about 400 to 1000 in a room). The RC522 keeps running.
The sensor needs about 3 minutes to warm up. `READ_TIMEOUT` means wiring (TX/RX crossed, 5 V supply).

## Scene 11: New model over MQTT (anomaly detector)

**Do Scene 6 first:** this upload overwrites the spare flash bank, where the staged Environment model is kept.

1. **Upload** (8 chunks):
   ```powershell
   .\.venv\Scripts\python.exe tools\node_client.py --broker 127.0.0.1 --node node1 upload ml\artifacts\anomaly_real\anomaly.amdl --kind anomaly --transfer 7003 --chunk 1024
   ```
   or paste `chunks/anomaly_chunk_1_of_8.hex` ... `8_of_8.hex` (format **Hex**) to `aai16/node1/in/model/chunk`.
2. **Activate:** `22_s11_node1_model-control_anomaly_activate.json` (`transfer_id` 7003, the `--transfer` value).
3. **Inputs:** `23_s11_node1_model-control_anomaly_inputs.json` (HC-SR04 distance and MAX6675 equipment temperature).
4. **Optional refusal:** `24_..._anomaly_badmap_distance_as_temperature.json` gives `VALUE_KIND_MISMATCH`.

**Expect** `inference` about once a second with `verdict`, `score`, `threshold`, `slots_present` and `inference_us`.
For a normal baseline keep the thermocouple near 30 to 33 C and aim the HC-SR04 at a surface about 1 m away; the
training data is outside other conditions, so an `ANOMALY` at the start is expected there and should be explained.

## Scene 12: Evidence and closing

| Show | Where |
|---|---|
| inference time | `inference_us` (Motion about 340 us, Environment about 360 us) |
| sensor health | heartbeat `readings_ok`, `read_errors`, `overruns` |
| memory | heartbeat `free_heap` |
| failures are visible | readings with `valid: false` and an error code |
| sampling accuracy | earlier 12-minute recording: 11,891 readings, no skipped interval, worst lateness 10.8 ms |

Not measured: sampling during a model upload, ESP-NN versus the reference build. See [project_status.md](project_status.md).

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| A sensor shows `READ_BUS_ERROR` (I2C) | wrong address or wiring; try the chip-id probe |
| A sensor shows `READ_TIMEOUT` (UART, pulse) | TX/RX not crossed, missing 5 V, wrong pins |
| Model `SUSPENDED`, `SENSOR_NOT_ACTIVE` | a mapped sensor was replaced; expected after Scene 9 |
| Inference never appears | the window is not full yet (40 s for Environment) or keeps restarting because of invalid or late readings |
| `NO_STAGED_MODEL` on ACTIVATE | nothing staged (or node1 rebooted since): upload again |
| More MQTTX problems | [mqttx_setup.md](mqttx_setup.md), section 5 |
