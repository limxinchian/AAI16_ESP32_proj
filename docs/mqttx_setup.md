# MQTTX setup

[MQTTX](https://mqttx.app) is the desktop MQTT client used to drive and watch the nodes during the demo.
The nodes only speak **binary Protobuf**. MQTTX cannot type or read that, so a small helper on the laptop
(`tools/json_bridge_all.py`) translates in both directions:

```
MQTTX (JSON)  ->  aai16/<node>/json_in/<topic>  ->  [helper: JSON -> Protobuf]  ->  aai16/<node>/in/<topic>   ->  ESP32
MQTTX (JSON)  <-  aai16/<node>/json/<topic>     <-  [helper: Protobuf -> JSON]  <-  aai16/<node>/out/<topic>  <-  ESP32
```

JSON exists only on the laptop. The boards never parse JSON.

---

## 1. Before opening MQTTX

1. **Start the Mosquitto broker** on the laptop, port 1883, listening on all interfaces (see `docs/STEP_BY_STEP_GUIDE.md`, section 3).
2. **Start the JSON helper** and leave its window open for the whole session:

   ```powershell
   .\.venv\Scripts\python.exe tools\json_bridge_all.py --broker 127.0.0.1
   ```

   It prints `JSON bridge running for 'node1'` and `... 'node2'`. Every message it forwards is printed as
   `sent <topic>: N JSON bytes -> M protobuf bytes`, and every message it refuses is printed as `ERROR: ...`.

## 2. Connection

| Setting | Value |
|---|---|
| Name | anything, for example `AAI16` |
| Host | `mqtt://127.0.0.1` (MQTTX runs on the same laptop as Mosquitto) |
| Port | `1883` |
| Client ID | leave the generated one |
| Username / password | empty (local broker) |
| MQTT version | 3.1.1 or 5.0 |

## 3. Subscriptions

Create each one with **New Subscription**. Give each a short name and, if you like, its own colour.

### Always on

| Name | Topic | QoS | Shows |
|---|---|---|---|
| node1_config_ack | `aai16/node1/json/config_ack` | 1 | `ADMITTED`, `APPLIED` or `REJECTED` (with the reason) for sensor configs |
| node1_rules_ack | `aai16/node1/json/rules_ack` | 1 | the same for rule sets |
| node1_bridge_ack | `aai16/node1/json/bridge_ack` | 1 | the same for the bridge |
| node1_model_chunk_ack | `aai16/node1/json/model_chunk_ack` | 1 | model upload progress, one reply per chunk |
| node1_model_status | `aai16/node1/json/model_status` | 1 | active model, `ACTIVE` / `SUSPENDED` / `REJECTED` and the reason |
| node1_inference | `aai16/node1/json/inference` | 1 | ML results |
| node1_notification | `aai16/node1/json/notification` | 1 | rule events on node1 |
| node1_heartbeat | `aai16/node1/json/heartbeat` | 1 | every 10 s: running sensors, generations, model, free heap, counters |
| node2_config_ack | `aai16/node2/json/config_ack` | 1 | |
| node2_rules_ack | `aai16/node2/json/rules_ack` | 1 | |
| node2_bridge_ack | `aai16/node2/json/bridge_ack` | 1 | |
| node2_notification | `aai16/node2/json/notification` | 1 | rule events on node2 |
| node2_heartbeat | `aai16/node2/json/heartbeat` | 1 | |
| error | `aai16/+/json/error` | 1 | a paste the helper could not convert (broken JSON, unknown field) |

### Sensor readings (add when the scene needs them)

Readings are published at QoS 0 (a lost reading is replaced by the next one), so subscribe at QoS 0.

| Name | Topic | Values | Rate |
|---|---|---|---|
| node2_rc522 | `aai16/node2/json/reading/rc522` | `card_present`, `atqa` | every 200 ms |
| node2_ld2450 | `aai16/node2/json/reading/ld2450` | `x_mm`, `y_mm`, `speed_cm_s` | about 10 per second |
| node2_mhz19b | `aai16/node2/json/reading/mhz19b` | `co2_ppm` | every 2 s |
| node1_gy511 | `aai16/node1/json/reading/gy511` | `ax_g`, `ay_g`, `az_g` | **50 per second** (mute it when not needed) |
| node1_dht22 | `aai16/node1/json/reading/dht22` | `humidity_pct`, `temp_c` | every 2 s |
| node1_bme280 | `aai16/node1/json/reading/bme280` | `temp_c`, `pressure_pa`, `humidity_pct` | every 2 s |
| node1_max6675 | `aai16/node1/json/reading/max6675` | `temp_c` | every 250 ms |
| node1_hcsr04 | `aai16/node1/json/reading/hcsr04` | `distance_cm` | every 100 ms |

Every reading also carries `valid` and `error`. A failed read is published as `"valid": false` with an error code
(`READ_TIMEOUT`, `READ_BUS_ERROR`, `READ_BAD_FRAME`, ...), never as a made-up value.

**Do not subscribe to `aai16/node1/json/#`.** The GY-511 publishes 50 readings a second and floods the list.

### Showing the raw Protobuf (optional)

These are the bytes the boards really send and receive. Set the payload view at the top of the message list to **Hex**.

| Name | Topic | QoS |
|---|---|---|
| RAW node1 heartbeat | `aai16/node1/out/heartbeat` | 1 |
| RAW node2 RC522 | `aai16/node2/out/reading/4` | 0 |
| RAW to node1 | `aai16/node1/in/#` | 1 |
| RAW to node2 | `aai16/node2/in/#` | 1 |

A reading starts with `08 <sensor id>` (field 1, `sensor_id`). The schema is `protocol/aai16.proto`.

## 4. Publishing

| Publish to | Payload format | QoS | Purpose |
|---|---|---|---|
| `aai16/<node>/json_in/config` | JSON | 1 | sensor profiles (one per bus) |
| `aai16/<node>/json_in/rules` | JSON | 1 | the node's whole rule set (it replaces the old set) |
| `aai16/<node>/json_in/bridge` | JSON | 1 | which other-node readings to use as virtual sensors |
| `aai16/<node>/json_in/model/control` | JSON | 1 | `ACTION_ACTIVATE`, `ACTION_SET_INPUTS`, `ACTION_QUERY` |
| `aai16/<node>/json_in/command` | JSON | 1 | `RCMD_RUN`, `RCMD_STOP`, `RCMD_CLEAR` |
| `aai16/node1/in/model/chunk` | **Hex** | 1 | one model chunk (`in/`, not `json_in/`) |

Ready-made messages for every demo step are in `demo/copypaste/` (one file per message, and all of them in `ALL.md`).

### Generation numbers
- **Config, rules, bridge:** write `"generation": "auto"`. The helper reads the node's current number from its
  heartbeat and uses the next one (the first message can wait up to 10 s for a heartbeat).
- **Model control:** write the number yourself. It must be higher than the last accepted one. To find it, send
  `{"generation": 1, "action": "ACTION_QUERY"}` to `aai16/node1/json_in/model/control` and read `generation` in the
  `model_status` reply. A refused message does not use up its number.

## 5. Common mistakes

| Symptom | Cause | Fix |
|---|---|---|
| Nothing happens, and the helper window prints nothing | the topic does not start with `aai16/<node>/json_in/` (for example `Topic aai16/...` pasted from a document) | retype the topic by hand |
| Helper prints `cannot send JSON to 'command '` | a trailing space in the topic | delete it; MQTTX history can bring it back, so type the topic fresh |
| `REJECTED`, `STALE_GENERATION` on a model message | the number is not higher than the last accepted one | `ACTION_QUERY`, then use `generation + 1` |
| `REJECTED`, `TRANSFER_MISMATCH` on ACTIVATE | `transfer_id` differs from the staged upload | 7001/7002/7003 for the pack's chunks; for `demo\run.ps1 models` use `env_transfer` in `demo\.state.json` |
| Rules sent but nothing fires | the rules were published to `.../config` | rules go to `.../json_in/rules` |
| No chunk acks visible | the helper was not running during the upload, or the subscription is QoS 0 | keep the helper running; subscribe at QoS 1 |
| Message list empty for a topic | a subscription is selected (filter chip at the top) | click the chip's **x**, then **All** |
