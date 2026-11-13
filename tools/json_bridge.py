"""JSON <-> Protobuf helper so MQTTX (or any MQTT tool) can use plain JSON with NO scripts.

The node keeps speaking binary Protobuf on aai16/<node>/in/... and aai16/<node>/out/... (unchanged).
This laptop-side helper adds a JSON mirror:

  board -> you :  aai16/<node>/out/<topic>      (protobuf)  ==>  aai16/<node>/json/<topic>     (JSON)
  you -> board :  aai16/<node>/json_in/<topic>  (JSON)      ==>  aai16/<node>/in/<topic>       (protobuf)

  topics you can SEND as JSON:  config  rules  bridge  command  model/control
  readings show the sensor NAME and named values (no numbers to look up); add --with-raw for the raw bytes.
  topics you SEE as JSON:       config_ack rules_ack bridge_ack model_chunk_ack model_status inference
                                heartbeat notification (+ json/error when your JSON is rejected)
                                reading/<sensor name>: gy511 dht22 ld2450 rc522 bme280 hcsr04 mhz19b max6675

  python tools/json_bridge.py --broker 172.20.10.2 --node node1

In MQTTX: subscribe to  aai16/node1/json/#  and publish JSON to  aai16/node1/json_in/config . No Run Script needed.
Model upload (chunks) stays on tools/node_client.py. Leave this window open while you use MQTTX.
"""
import argparse
import json
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from google.protobuf import json_format  # noqa: E402

import node_client as nc  # noqa: E402
import config_shorthand as short  # noqa: E402

pb = nc.pb
# sensor id -> (name, names of the Reading.values in order). Same ids/order as tools/sensor_profiles.py.
SENSORS = {
    1: ("GY-511", ["ax_g", "ay_g", "az_g"]),
    2: ("DHT22", ["humidity_pct", "temp_c"]),
    3: ("LD2450", ["x_mm", "y_mm", "speed_cm_s"]),
    4: ("RC522", ["card_present", "atqa"]),
    5: ("BME280", ["temp_c", "pressure_pa", "humidity_pct"]),
    6: ("HC-SR04", ["distance_cm"]),
    7: ("MH-Z19B", ["co2_ppm"]),
    8: ("MAX6675", ["temp_c"]),
}


# topic name for each sensor's readings: aai16/<node>/json/reading/<slug>
SLUG = {1: "gy511", 2: "dht22", 3: "ld2450", 4: "rc522", 5: "bme280", 6: "hcsr04", 7: "mhz19b", 8: "max6675"}


def reading_topic_suffix(suffix):
    """'reading/5' -> 'reading/bme280' (unknown ids keep a readable fallback)."""
    try:
        sid = int(suffix.split("/", 1)[1])
    except (IndexError, ValueError):
        return suffix
    return "reading/" + SLUG.get(sid, f"sensor{sid}")


# class names per model kind, in the order of the model's scores (see ml/artifacts/*/contract.json)
MODEL_LABELS = {"MODEL_MOTION": ["STATIONARY", "MOVING", "VIBRATING"], "MODEL_ENVIRONMENT": ["STABLE", "CHANGING"]}
# role slots of the anomaly model (ml/ANOMALY_SPEC.md), indexed by slot number = bit in slot_mask
ANOMALY_SLOTS = ["motion", "temperature", "humidity", "pressure", "co2", "distance", "radar_distance", "equipment_temp"]


def sensor_name(sid):
    return SENSORS.get(sid, (f"sensor-{sid}", []))[0] if sid else None


def humanize(suffix, d, with_raw):
    """Replace sensor numbers by names and name the values; unknown shapes pass through unchanged."""
    if suffix.startswith("reading"):
        sid = d.get("sensor_id", 0)
        name, labels = SENSORS.get(sid, (f"sensor-{sid}", []))
        vals = d.get("values", [])
        out = {"sensor": name, "sensor_id": sid, "valid": d.get("valid", False), "error": d.get("error", "READ_OK")}
        out["values"] = {(labels[i] if i < len(labels) else f"value{i}"): v for i, v in enumerate(vals)}
        out["generation"] = d.get("generation", 0)
        out["timestamp_us"] = d.get("timestamp_us", 0)
        if with_raw:
            out["raw"] = d.get("raw", "")
        return out
    if suffix == "inference" and d.get("kind") == "MODEL_ANOMALY":
        mask = d.get("slot_mask", 0)
        errs = d.get("slot_errors", [])
        present = [ANOMALY_SLOTS[i] for i in range(8) if mask >> i & 1]
        per_slot = {ANOMALY_SLOTS[i]: errs[i] for i in range(min(8, len(errs))) if mask >> i & 1}
        worst = max(per_slot, key=per_slot.get) if per_slot else None
        score, thr = d.get("anomaly_score", 0.0), d.get("anomaly_threshold", 0.0)
        out = {"model": "MODEL_ANOMALY", "verdict": "ANOMALY" if d.get("anomaly") else "NORMAL",
               "score": score, "threshold": thr, "score_over_threshold": round(score / thr, 3) if thr else None,
               "slots_present": present, "slots_absent": [s for s in ANOMALY_SLOTS if s not in present],
               "slot_errors": per_slot, "most_unusual_slot": worst if d.get("anomaly") else None,
               "inference_us": d.get("inference_us"), "model_version": d.get("model_version"),
               "generation": d.get("generation"), "timestamp_us": d.get("timestamp_us")}
        return out
    if suffix == "inference":
        labels = MODEL_LABELS.get(d.get("kind"), [])
        scores = d.get("scores", [])
        if scores:
            k = max(range(len(scores)), key=lambda i: scores[i])
            name = labels[k] if k < len(labels) else f"class {k}"
            named = {(labels[i] if i < len(labels) else f"class {i}"): v for i, v in enumerate(scores)}
            return {"model": d.get("kind"), "prediction": name, "confidence": scores[k], "scores": named,
                    "inference_us": d.get("inference_us"), "model_version": d.get("model_version"),
                    "generation": d.get("generation"), "timestamp_us": d.get("timestamp_us")}
    if suffix == "heartbeat":
        d["active_sensors"] = [sensor_name(i) for i in d.get("active_sensor_ids", [])]
    if suffix.endswith("_ack") and d.get("sensor_id"):
        d["sensor"] = sensor_name(d["sensor_id"])
    return d


BUS_OF = {1: 2, 2: 1, 3: 3, 4: 4, 5: 2, 6: 1, 7: 3, 8: 4}           # sensor id -> bus number
BUS_NAME = {1: "GPIO", 2: "I2C", 3: "UART", 4: "SPI"}


def describe_profile(prof, counts=False):
    """One line of facts about a requested profile (a pb.SensorProfile): which sensor, on what, what to do."""
    sid = prof.sensor_id
    d = {"sensor": sensor_name(sid), "sensor_id": sid, "bus": BUS_NAME.get(int(prof.bus), str(int(prof.bus)))}
    if prof.dormant and not prof.enabled:
        d["action"] = "store as DORMANT (stop this bus; a rule can start this sensor later, it never starts by itself)"
    elif not prof.enabled:
        d["action"] = "disable (stop this sensor)"
        return d
    else:
        d["action"] = "enable (start, replacing whatever was on this bus)"
    d["period_ms"] = prof.period_ms
    which = prof.WhichOneof("params")
    if which == "i2c":
        q = prof.i2c
        d["settings"] = {"address": f"0x{q.addr:02x}", "sda": q.sda, "scl": q.scl, "clock_hz": q.clock_hz}
    elif which == "uart":
        q = prof.uart
        d["settings"] = {"port": q.port, "tx": q.tx, "rx": q.rx, "baud": q.baud}
    elif which == "spi":
        q = prof.spi
        d["settings"] = {"cs": q.cs, "sclk": q.sclk, "mosi": q.mosi, "miso": q.miso, "clock_hz": q.clock_hz, "mode": q.mode}
    elif which == "gpio":
        q = prof.gpio
        d["settings"] = {"data_pin": q.data_pin, "aux_pin": q.aux_pin}
    if counts:  # only with --details: how big the profile is
        d["operations_per_cycle"] = len(prof.ops)
        d["decode_entries"] = len(prof.decode)
    return d


def reason_text(reason, detail):
    n = detail
    t = {
        "STALE_GENERATION": "the generation is lower than the one the node already runs (it must be higher)",
        "DUPLICATE_GENERATION": "the generation equals the one the node already runs (it must be higher)",
        "EMPTY_REQUEST": "the config contains no profiles",
        "TOO_MANY_PROFILES": f"{n} profiles were sent; the maximum is 4 (one per bus)",
        "BAD_SENSOR_ID": f"sensor_id {n} is not allowed (use 1 to 127)",
        "DUPLICATE_SENSOR_ID": "the same sensor_id appears twice in this message",
        "DUPLICATE_BUS": "two sensors for the same bus in one message (only one sensor can run on a bus)",
        "UNKNOWN_BUS": f"bus {n} does not exist (1=GPIO, 2=I2C, 3=UART, 4=SPI)",
        "PARAMS_MISMATCH": "the bus settings block (i2c / uart / spi / gpio) is missing or does not match 'bus'",
        "FORBIDDEN_PIN": f"GPIO {n} is not allowed on this board",
        "PIN_CONFLICT": f"GPIO {n} is used twice (by two sensors, or twice in one sensor)",
        "BAD_ADDRESS": f"I2C address {n} (0x{n:02x}) is outside the allowed range 0x08 to 0x77",
        "BAD_BAUD": f"baud rate {n} is outside the allowed range 1200 to 2000000",
        "BAD_CLOCK": f"clock {n} Hz is out of range (I2C 10 kHz to 1 MHz, SPI 100 kHz to 20 MHz)",
        "BAD_SPI_MODE": f"SPI mode {n} is invalid (use 0 to 3)",
        "BAD_PORT": f"UART port {n} is invalid (use 1 or 2)",
        "BAD_PERIOD": f"period_ms {n} is outside the allowed range 10 to 3600000",
        "NO_OPS": "the sensor has no operations (the ops list is empty)",
        "TOO_MANY_OPS": f"{n} operations were sent; the maximum is 12",
        "UNKNOWN_OP": f"operation #{n} (counting from 0 in ops) has an unknown type",
        "OP_NOT_ALLOWED_ON_BUS": f"operation #{n} (counting from 0 in ops) is not allowed on this bus",
        "BAD_LENGTH": f"operation #{n} (counting from 0): a length is out of range (read_len must be 1 to 64, total reply at most 64 bytes)",
        "BAD_DELAY": f"operation #{n} (counting from 0): delay_us is out of range (1 to 1000000)",
        "BAD_TIMEOUT": f"operation #{n} (counting from 0): a timeout is out of range",
        "PIN_NOT_CONFIGURED": f"operation #{n} (counting from 0) uses a pin that is not named in the gpio block",
        "BAD_DECODE": "a decode entry points outside the bytes that are read, or has an invalid width, shift or sign",
        "BAD_INIT": "init_ops are invalid, or an init decode entry points outside the init bytes",
        "BAD_CHECK": "a frame check points outside the bytes that are read",
        "BAD_CALC": f"calc step #{n} is invalid (stack, register, or use before define)",
        "BAD_OUTPUT": "output_count / output_kinds are invalid, or an output register is never written",
        "APPLY_FAILED": f"the config was valid but the node could not start profile #{n}",
        "PERSIST_FAILED": "the node could not save the config to flash, so it was rolled back",
        "MALFORMED": "the node could not decode the message (not valid protobuf, or larger than the fixed maximum)",
    }
    return t.get(reason, reason)


CHUNK_MEANING = {
    "OK": "chunk accepted and stored",
    "DUPLICATE": "the node already had this chunk, so it was ignored (harmless)",
    "OUT_OF_ORDER": "a chunk was missing; the node wants the chunk named in next_expected_seq next",
    "BAD_LENGTH": "the chunk size is wrong or larger than the maximum (1024 bytes)",
    "BAD_TRANSFER": "this chunk does not belong to the transfer in progress",
    "STORAGE_ERROR": "the node could not write the chunk to flash",
    "DIGEST_MISMATCH": "all chunks arrived but the file's SHA-256 did not match (corrupted); the running model is untouched",
}


IN_TYPES = {"config": pb.ConfigApply, "rules": pb.RuleSetApply, "bridge": pb.BridgeApply,
            "command": pb.NodeCommand, "model/control": pb.ModelControl}


def to_json_text(m, suffix="", with_raw=False):
    d = json_format.MessageToDict(m, preserving_proto_field_name=True, always_print_fields_with_no_presence=True)

    def fix(x, key=""):
        if isinstance(x, dict):
            return {k: fix(v, k) for k, v in x.items()}
        if isinstance(x, list):
            return [fix(v, key) for v in x]
        if isinstance(x, str) and x.isdigit() and (key.endswith("_us") or key.endswith("_ms") or key == "uptime_ms"):
            return int(x)
        if isinstance(x, float):
            return round(x, 4)
        return x
    return json.dumps(humanize(suffix, fix(d), with_raw), indent=2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", required=True)
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--node", default="node1")
    ap.add_argument("--seconds", type=int, default=0, help="stop after N seconds (default: until Ctrl+C)")
    ap.add_argument("--details", action="store_true", help="also show operations_per_cycle and decode_entries in config ACKs")
    ap.add_argument("--with-raw", action="store_true", help="include the raw sensor bytes (base64) in reading messages")
    a = ap.parse_args()
    import paho.mqtt.client as mqtt

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"aai16-json-bridge-{a.node}-{int(time.time())}")
    out_prefix, json_prefix, jin_prefix, in_prefix = (f"aai16/{a.node}/out/", f"aai16/{a.node}/json/",
                                                      f"aai16/{a.node}/json_in/", f"aai16/{a.node}/in/")

    def err(text, detail=""):
        c.publish(json_prefix + "error", json.dumps({"error": text, "detail": detail}, indent=2), qos=1)
        print("ERROR:", text, detail, flush=True)

    def info(text):
        c.publish(json_prefix + "info", json.dumps({"info": text}, indent=2), qos=1)
        print(time.strftime("%H:%M:%S"), text, flush=True)

    # Generation numbers: config / rules / bridge each have their own strictly increasing counter on the node.
    # The heartbeat reports the committed ones; "generation": "auto" (or 0 / missing) means "use the next one".
    AUTO = {"config": "config_generation", "rules": "rule_generation", "bridge": "bridge_generation"}
    gens = {k: None for k in AUTO}
    pending = []   # (suffix, doc, since) waiting for the first heartbeat

    sent = {}      # generation -> list of requested pb.SensorProfile (recorded from aai16/<node>/in/config)
    active = set()  # sensor ids currently running, from heartbeats and applied configs
    transfers = {}  # transfer id -> what the helper saw being uploaded (from aai16/<node>/in/model/chunk)

    def enrich_chunk_ack(m):
        status = pb.ChunkStatus.Name(m.status).replace("CHUNK_", "")
        tr = transfers.get(int(m.transfer_id))
        out = {"result": status, "meaning": CHUNK_MEANING.get(status, status), "transfer": int(m.transfer_id)}
        if tr:
            total, seq = tr["total"], int(m.seq)
            out["chunk"] = f"{seq + 1} of {total}"
            got = sum(n for k, n in tr["lens"].items() if k <= seq)
            if status == "OK":
                out["progress"] = f"{got} of {tr['size']} bytes ({100 * got // max(tr['size'], 1)} %)"
            out["model"] = tr["kind"]
            out["model_version"] = tr["version"]
            out["chunk_bytes"] = tr["lens"].get(seq)
            if status == "OK" and seq == total - 1:
                out["done"] = "last chunk accepted, the file passed its SHA-256 check and is staged; send ACTIVATE to run it"
        else:
            out["chunk"] = f"seq {int(m.seq)} (the helper did not see this upload start)"
        out["seq"] = int(m.seq)
        out["next_expected_seq"] = int(m.next_expected)
        out["generation"] = int(m.generation)
        return json.dumps(out, indent=2)

    def names_of(ids):
        return sorted((sensor_name(i) for i in ids), key=str)

    def enrich_ack(m, body):
        """Add what was requested, what is running, and (if rejected) exactly what was refused and why."""
        phase = pb.ApplyPhase.Name(m.phase).replace("PHASE_", "")
        reason = pb.Reason.Name(m.reason).replace("REASON_", "") if m.reason else "NONE"
        reqs = sent.get(int(m.generation))
        out = {"result": phase, "generation": int(m.generation)}
        if phase == "ADMITTED":
            out["meaning"] = "the node checked every part of this config and accepted it; it is being applied now"
        elif phase == "APPLIED":
            out["meaning"] = "the config is now running"
        else:
            out["meaning"] = "NOTHING was changed; the previous config keeps running"
            out["reason"] = reason
            out["why"] = reason_text(reason, int(m.detail))
            if m.sensor_id:
                out["problem_sensor"] = sensor_name(int(m.sensor_id))
            if reqs and (m.sensor_id or m.profile_index) and 0 <= int(m.profile_index) < len(reqs):
                out["problem_profile"] = (f"profile #{int(m.profile_index)} in your list (counting from 0): "
                                          f"{sensor_name(reqs[int(m.profile_index)].sensor_id)}")
            if m.detail:
                out["detail_value"] = int(m.detail)
        key = "applied" if phase == "APPLIED" else "requested"
        if reqs is not None:
            out[key] = [describe_profile(p, a.details) for p in reqs]
        else:
            out[key] = "(this config was not seen by the helper, so its contents are unknown)"
        if phase == "APPLIED":
            if reqs is not None:  # predict what is running now
                for pr in reqs:
                    for i in [i for i in active if BUS_OF.get(i) == int(pr.bus)]:
                        active.discard(i)
                    if pr.enabled:
                        active.add(pr.sensor_id)
            out["now_running"] = names_of(active)
        elif phase == "REJECTED":
            out["still_running"] = names_of(active)
        return json.dumps(out, indent=2)

    def is_auto(g):
        return g is None or g == 0 or (isinstance(g, str) and g.strip().lower() in ("", "0", "auto"))

    def send(suffix, doc):
        cls = IN_TYPES[suffix]
        try:
            m = json_format.Parse(json.dumps(doc), cls())
        except Exception as e:
            return err(f"your JSON for '{suffix}' is not valid", str(e))
        data = m.SerializeToString()
        c.publish(in_prefix + suffix, data, qos=1)
        print(f"{time.strftime('%H:%M:%S')} sent {suffix}: {len(json.dumps(doc))} JSON bytes -> {len(data)} protobuf bytes", flush=True)

    def submit(suffix, doc, since):
        if suffix in AUTO:
            g = doc.get("generation")
            if is_auto(g):
                if gens[suffix] is None:
                    pending.append((suffix, doc, since))
                    return info(f"waiting for the node's first heartbeat (up to 10 s) to pick the next {suffix} generation")
                gens[suffix] += 1
                doc["generation"] = gens[suffix]
                info(f"{suffix} sent as generation {gens[suffix]} (auto)")
            else:
                try:
                    gens[suffix] = max(gens[suffix] or 0, int(g))
                except (TypeError, ValueError):
                    return err(f"'generation' must be a number or \"auto\" (got {g!r})")
        send(suffix, doc)

    def learn_generations(suffix, m):
        if suffix == "heartbeat":
            for kind, field in AUTO.items():
                v = int(getattr(m, field))
                gens[kind] = v if gens[kind] is None else max(gens[kind], v)
        elif suffix == "config_ack" and m.phase in (pb.PHASE_ADMITTED, pb.PHASE_APPLIED):
            gens["config"] = max(gens["config"] or 0, int(m.generation))
        if suffix == "heartbeat":
            active.clear()
            active.update(int(i) for i in m.active_sensor_ids)
        if suffix == "heartbeat" and pending:
            todo = pending[:]
            del pending[:]
            for sfx, doc, since in todo:
                submit(sfx, doc, since)

    def on_message(_c, _u, msg):
        if msg.topic.startswith(out_prefix):
            suffix = msg.topic[len(out_prefix):]
            try:
                m = nc.decode(suffix, bytes(msg.payload))
                body = to_json_text(m, suffix, a.with_raw) if m is not None else json.dumps({"raw_hex": msg.payload.hex()})
                if m is not None:
                    learn_generations(suffix, m)
                    if suffix == "config_ack":
                        body = enrich_ack(m, body)
                    elif suffix == "model_chunk_ack":
                        body = enrich_chunk_ack(m)
            except Exception as e:  # never die on one bad message
                body = json.dumps({"decode_error": str(e), "raw_hex": msg.payload.hex()})
            topic_suffix = reading_topic_suffix(suffix) if suffix.startswith("reading/") else suffix
            c.publish(json_prefix + topic_suffix, body, qos=0 if suffix.startswith("reading") else 1)
        elif msg.topic == in_prefix + "model/chunk":
            try:
                ch = pb.ModelChunk.FromString(bytes(msg.payload))
                t = transfers.setdefault(int(ch.transfer_id), {"total": int(ch.total_chunks), "size": int(ch.total_size),
                                                              "kind": pb.ModelKind.Name(ch.kind), "version": int(ch.model_version), "lens": {}})
                t["lens"][int(ch.seq)] = len(ch.data)
            except Exception:
                pass
        elif msg.topic == in_prefix + "config":
            try:
                req = pb.ConfigApply.FromString(bytes(msg.payload))
                sent[int(req.generation)] = list(req.profiles)
            except Exception:
                pass
        elif msg.topic.startswith(jin_prefix):
            suffix = msg.topic[len(jin_prefix):]
            if suffix not in IN_TYPES:
                return err(f"cannot send JSON to '{suffix}'", "use one of: " + ", ".join(IN_TYPES))
            try:
                doc = json.loads(msg.payload.decode("utf-8"))
                if not isinstance(doc, dict):
                    raise ValueError("the JSON must be an object { ... }")
            except Exception as e:
                return err(f"your JSON for '{suffix}' is not valid", str(e))
            if suffix == "config":
                try:
                    doc = short.expand(doc)
                except Exception as e:
                    return err("your short config is not valid", str(e))
            submit(suffix, doc, time.time())

    c.on_message = on_message
    c.connect(a.broker, a.port, 30)
    c.subscribe([(f"aai16/{a.node}/out/#", 1), (f"aai16/{a.node}/json_in/#", 1), (f"aai16/{a.node}/in/config", 1), (f"aai16/{a.node}/in/model/chunk", 1)])
    print(f"JSON bridge running for '{a.node}'. In MQTTX subscribe to  aai16/{a.node}/json/#  "
          f"and publish JSON to  aai16/{a.node}/json_in/config", flush=True)
    c.loop_start()
    try:
        end = time.time() + a.seconds if a.seconds else None
        while end is None or time.time() < end:
            time.sleep(0.2)
            for item in pending[:]:
                if time.time() - item[2] > 15:
                    pending.remove(item)
                    err(f"no heartbeat from node '{a.node}' in 15 s, so the next generation is unknown",
                        "is the board on and connected? or put a number in \"generation\"")
    except KeyboardInterrupt:
        pass
    c.loop_stop()


if __name__ == "__main__":
    main()
