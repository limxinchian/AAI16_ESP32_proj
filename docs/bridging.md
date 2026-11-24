# Node-to-node bridging

How one node uses another node's sensors as its own (virtual sensors).

| Item | Detail |
|---|---|
| System | AAI16 configurable edge node, two ESP32-S3 boards (node1, node2), MQTT broker on the laptop |
| Source code | `components/node_services/bridge.c`, `components/node_core/node_topics.c`, `node_core.c`, `components/platform_esp/plat_net.c` |
| Demo scene | Scene 3 in [demo_guide.md](demo_guide.md) |

---

## 1. The idea

A node can use **another node's sensor readings as if they were its own sensors**. In the FlexiCargo demo, node1 has no RFID reader and no radar, but it can react to the ones on node2.

The other node's readings appear as **virtual sensors** with numbers from **128 to 255**. Real sensors use 1 to 127, so the two can never clash. To node1's rule engine and ML model, a virtual sensor looks exactly like a local one.

| Real sensor on node2 | Becomes on node1 | Values |
|---|---|---|
| LD2450 radar (sensor 3), every 100 ms | virtual sensor **130** | x position, distance ahead, speed |
| RC522 RFID (sensor 4), every 200 ms | virtual sensor **131** | card_present, atqa |

---

## 2. How the data flows

```
node2: RC522 read (sensor 4)
   |  publish  aai16/node2/out/reading/4   (QoS 0)
   v
MQTT broker
   |  delivered to every node subscribed to  aai16/+/out/reading/+
   v
node1: does a bridge source match node2 / sensor 4?
   |  yes -> virtual reading, sensor 131 (node1's receive time)
   v
node1 rule engine (rule 4 -> REMOTE_CHECK_IN_SEEN)  and  ML selector (if mapped)
```

### Step 1: node2 publishes its readings as normal
Every reading node2 takes is published on its own topic, `aai16/node2/out/reading/<sensor id>` (QoS 0). node2 does nothing special for bridging and doesn't know who is listening.

### Step 2: every node listens to every other node's readings
When a node connects to the broker, it subscribes to `aai16/+/out/reading/+` (`plat_net.c:87`). A reading from another node enters `node_task` as a remote reading (`node_topics.c:31`). It's **ignored unless a bridge source matches** that node and sensor.

### Step 3: the bridge configuration says which remote readings to use
The operator sends a bridge message to the listening node. In the demo it goes to `aai16/node1/json_in/bridge`:

```json
{
  "generation": "auto",
  "sources": [
    {"local_sensor_id": 130, "remote_node": "node2", "remote_sensor_id": 3,
     "max_age_ms": 1000, "period_ms": 100,
     "kinds": ["VALUE_OTHER", "VALUE_RADAR_DISTANCE_MM", "VALUE_OTHER"]},
    {"local_sensor_id": 131, "remote_node": "node2", "remote_sensor_id": 4,
     "max_age_ms": 1000, "period_ms": 200,
     "kinds": ["VALUE_OTHER", "VALUE_OTHER"]}
  ]
}
```

| Field | Meaning |
|---|---|
| `local_sensor_id` | The virtual sensor number on this node (128 to 255) |
| `remote_node`, `remote_sensor_id` | Which node and which of its real sensors to take (1 to 127) |
| `max_age_ms` | How old the latest reading may get before it counts as stale (1 second in the demo) |
| `period_ms` | How often readings are expected. Used by the ML selector to check the timing fits a model |
| `kinds` | What each value is (for example radar distance). Lets the ML model check the input type, exactly as with a real sensor |

### Step 4: a matching reading arrives (bridge_on_remote in bridge.c)
1. node1 decodes node2's Protobuf reading.
2. It looks for a source whose `remote_node` and `remote_sensor_id` match.
3. It builds a **virtual reading** with the local sensor number (130 or 131). The timestamp is **node1's own receive time**, so freshness is judged on the listening node.
4. **Invalid stays invalid.** If node2's reading was invalid, the virtual reading is invalid too, carrying the same error code (`bridge.c:95`).
5. **Shape check.** If the number of values doesn't match the number of declared `kinds`, the reading is rejected.
6. The virtual reading is passed to `node_feed_reading` (`node_core.c:541-545`), the **same path a real sensor uses**: the rule engine, then the ML selector.

### Step 5: stale remote data is never used as current (bridge_poll in bridge.c)
On every loop, node1 checks each source. If no reading has arrived for longer than `max_age_ms`, it produces **one invalid reading** for that virtual sensor, with error `READ_TIMEOUT`. Rules clear their windows and the ML model resets, so node1 never acts on old data from a node that has gone quiet. When fresh readings arrive again, the source recovers automatically.

---

## 3. Validation: a bad bridge config changes nothing

Before applying a bridge message, the node checks the whole message (`bridge_validate`, `bridge.c:22-56`). If any check fails, it replies `REJECTED` on `bridge_ack` with a reason, and the previous bridge stays in force.

| Check | Why |
|---|---|
| The generation must be newer than the current one | Old or repeated messages are refused (`STALE_GENERATION` / `DUPLICATE_GENERATION`) |
| At most 4 sources (`BR_MAX_SOURCES`) | Bounded memory |
| `local_sensor_id` must be a virtual number (128 to 255) | It can't hide or replace a real sensor |
| `remote_node` must not be this node itself | That would loop back to ourselves |
| `remote_sensor_id` must be 1 to 127 | Only real sensors can be bridged, so there are no chains of virtual sensors |
| `max_age_ms` 100 to 600,000; `period_ms` 10 to 3,600,000 | Sensible timing limits |
| 1 to 8 `kinds`, each a known value kind | The values can be type-checked later |
| No duplicate local numbers or remote sources | Each mapping is unambiguous |

When a new bridge is applied, the old source slots are **cleared**, so values from the previous mapping can't leak into the new one. The bridge is also saved with the node's state, so it survives a reboot.

---

## 4. How it's used in the FlexiCargo demo

node1's Mode A rules include two rules that use the virtual sensors:

| Rule | Watches | Fires when | Notification |
|---|---|---|---|
| 4 | virtual 131, value 0 (node2's RFID `card_present`) | it rises above 0.5 (edge-triggered, 5 s cooldown) | `REMOTE_CHECK_IN_SEEN` |
| 5 | virtual 130, value 1 (node2's radar distance ahead) | it is above 0 (5 s cooldown) | `REMOTE_RADAR_TARGET` |

**What you see:**
1. Tap the tag on node2's reader.
2. node2 publishes `PACKAGE_CHECK_IN_DETECTED` on `aai16/node2/json/notification`.
3. node1 publishes `REMOTE_CHECK_IN_SEEN` on `aai16/node1/json/notification`.
4. Walking in front of the radar while it runs gives `REMOTE_RADAR_TARGET` on node1.

**Order matters.** Send node1's rules and the bridge message, wait for both `APPLIED` acknowledgements, and wait at least 10 s after the previous tap (because of the cooldown) before tapping again.

---

## 5. Things to be exact about

- **The bridge shares data only.** node1 can't control node2's hardware.
- **Virtual readings aren't republished** to MQTT, so MQTTX can't show sensor 130 or 131 directly. You see the effect through node1's notifications. For a direct view, type `WATCH 131` (or `WATCH 130`) on node1's serial console, and `WATCH OFF` to stop.
- **Everything goes through the broker.** There's no direct board-to-board link. Adding a third node would only need another bridge message, not new firmware.
- **Without a rule or an ML mapping that uses the virtual sensor, nothing visible happens.** node1 receives the data and quietly does nothing.

---

## 6. Why we designed it this way

| Decision | Reason |
|---|---|
| Use MQTT publish/subscribe instead of a direct link | Nodes already publish their readings, so bridging is just subscribing. No extra protocol, and any node can listen to any other |
| Turn remote data into virtual sensors | Rules and ML work unchanged: they don't need to know whether a sensor is local or remote |
| Put the timestamp at the receiving node, with a maximum age | Freshness is judged where the data is used, and stale data becomes invalid instead of silently old |
| Copy validity and error codes through | A failure on node2 is never presented as valid data on node1 |
| Declare value kinds | A remote value can safely feed an ML model, with the same type check as a local sensor |
| Validate the whole message, with generations | A bad or outdated bridge config can never leave the node half-configured |

---

## 7. One line to say in the demo

"node2 publishes its readings like always. node1 subscribes to the ones named in its bridge config and turns them into virtual sensors 130 and 131, so its rules and ML can use them like local sensors. Each source has a maximum age, so stale remote data is marked invalid instead of being used."
