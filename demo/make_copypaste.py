"""Writes the MQTTX copy-paste pack for the FlexiCargo demo (plain JSON, published to aai16/<node>/json_in/<topic>).

    .\\.venv\\Scripts\\python.exe demo\\make_copypaste.py

Output (in demo\\copypaste\\):
    <NN>_<scene>_<node>_<topic>_<what>.json   one pretty-printed message per file, in demo order
    ALL.md                                    every message, grouped by scene, with its topic
    chunks\\<model>_chunk_<i>_of_<n>.hex       model packages split into 1 KB chunks (MQTTX payload format: Hex);
                                              generated on demand, not stored in git

Everything comes from the same builders the demo tool uses (demo\\demo.py), so pins, intervals and thresholds follow
demo\\rig.json. The JSON helper must be running for json_in topics:
    .\\.venv\\Scripts\\python.exe tools\\json_bridge_all.py --broker <broker ip>
"""
import hashlib
import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import demo  # noqa: E402

pb = demo.pb
OUT = HERE / "copypaste"
ML, ST = demo.ML, demo.ST

# sampling interval of every sensor in the FlexiCargo plan (period_ms)
PERIOD = {"rc522": 200, "ld2450": 100, "gy511": 20, "dht22": 2000, "hcsr04": 100, "bme280": 2000, "max6675": 250,
          "mhz19b": 2000}

# Fixed transfer ids, so an ACTIVATE can be pasted for chunks uploaded from this pack (or with --transfer <id>).
TRANSFER = {"motion": 7001, "environment": 7002, "anomaly": 7003}

# Model-control generations. Each node remembers the last accepted model generation (also across reboots) and refuses
# anything not higher. These start far above anything a fresh node has seen; if one is refused as STALE_GENERATION,
# send {"generation": 1, "action": "ACTION_QUERY"} and use the generation in the model_status reply + 1.
G = 2000000

# model packages sent chunk by chunk as Hex payloads to aai16/<ml node>/in/model/chunk
PACKAGES = {"motion": (pb.MODEL_MOTION, "ml/artifacts/motion/motion.amdl"),
            "environment": (pb.MODEL_ENVIRONMENT, "ml/artifacts/environment/environment.amdl"),
            "anomaly": (pb.MODEL_ANOMALY, "ml/artifacts/anomaly_real/anomaly.amdl")}
CHUNK = 1024


def cfg(items):
    """Short-form config with the rig wiring and the plan's sampling interval filled in."""
    out = []
    for it in items:
        it = dict(it)
        if it.get("enabled", True):
            it.setdefault("period_ms", PERIOD[it["sensor"]])
        out.append({**demo.RIG["wiring"].get(it["sensor"], {}), **it})
    return {"generation": "auto", "sensors": out}


def token(t):
    d = {"op": pb.CalcOp.Name(t.op)}
    if t.op == pb.CALC_PUSH_REG:
        d["reg"] = t.reg
    if t.op == pb.CALC_PUSH_CONST:
        d["value"] = round(t.value, 6)
    return d


def action(a):
    d = {"type": pb.RuleActionType.Name(a.type)}
    if a.type == pb.RACT_NOTIFY:
        d["text"] = a.text
    if a.type in (pb.RACT_SENSOR_START, pb.RACT_SENSOR_STOP):
        d["sensor_id"] = a.sensor_id
        d["duration_ms"] = a.duration_ms
    return d


def rules_json(rs):
    rules = []
    for r in rs.rules:
        rules.append({
            "rule_id": r.rule_id, "enabled": True,
            "vars": [{"sensor_id": v.sensor_id, "value_index": v.value_index, "stat": pb.RuleStat.Name(v.stat),
                      "window": v.window} for v in r.vars],
            "when": [token(t) for t in r.when], "action": action(r.action),
            "cooldown_ms": r.cooldown_ms, "edge_triggered": r.edge_triggered})
    return {"generation": "auto", "rules": rules}


def control(gen, action_name, kind, inputs=(), transfer=None):
    d = {"generation": gen, "action": action_name, "kind": kind}
    if transfer is not None:
        d["transfer_id"] = transfer
    if inputs:
        d["inputs"] = [{"sensor_id": s, "value_index": v, "feature_slot": f} for s, v, f in inputs]
    return d


def bme280_probe(addr):
    """Troubleshooting: reads only the BME280 chip-id register (0xD0). First value: 96 = BME280, 88 = BMP280."""
    w = demo.RIG["wiring"]["bme280"]
    return {"generation": "auto", "profiles": [{
        "sensor_id": 5, "bus": "BUS_I2C", "enabled": True, "period_ms": 1000,
        "i2c": {"sda": w["sda"], "scl": w["scl"], "addr": addr, "clock_hz": 100000},
        "ops": [{"type": "OP_WRITE_READ", "tx": "0A==", "read_len": 1}],
        "decode": [{"dest_reg": 0, "width": 1, "scale": 1.0}],
        "output_count": 1, "output_kinds": ["VALUE_OTHER"]}]}


def rc522_probe():
    """Troubleshooting: reads only the RC522 VersionReg (0x37). First value: 145/146 genuine, 178 clone, 0/255 = no chip."""
    w = demo.RIG["wiring"]["rc522"]
    return {"generation": "auto", "profiles": [{
        "sensor_id": 4, "bus": "BUS_SPI", "enabled": True, "period_ms": 1000,
        "spi": {**{k: w[k] for k in ("sclk", "mosi", "miso", "cs")}, "clock_hz": 1000000, "mode": 0},
        "ops": [{"type": "OP_WRITE_READ", "tx": "7gA=", "read_len": 2}],
        "decode": [{"dest_reg": 0, "src_offset": 1, "width": 1, "scale": 1.0}],
        "output_count": 1, "output_kinds": ["VALUE_OTHER"]}]}


BRIDGE = json.loads((demo.ROOT / "configs/bridge/bridge_node1_listens_to_node2_ld2450_rc522.json").read_text(encoding="utf-8"))
BRIDGE["generation"] = "auto"
for _s in BRIDGE["sources"]:
    _s["remote_node"] = ST

ITEMS = [
    # scene, node, topic, what, document
    ("00 Reset", ML, "command", "clear", {"cmd": "RCMD_CLEAR"}),
    ("00 Reset", ST, "command", "clear", {"cmd": "RCMD_CLEAR"}),
    ("01 Shipment arrives (RFID check-in)", ST, "config", "modeA",
     cfg([{"sensor": "rc522"}, {"sensor": "ld2450", "dormant": True}])),
    ("01 Shipment arrives (RFID check-in)", ST, "rules", "modeA", rules_json(demo.rules_for("A", "station"))),
    ("03 Two nodes working together (bridge)", ML, "config", "modeA", cfg([{"sensor": "gy511"}, {"sensor": "dht22"}])),
    ("03 Two nodes working together (bridge)", ML, "rules", "modeA", rules_json(demo.rules_for("A", "ml"))),
    ("03 Two nodes working together (bridge)", ML, "bridge", "listen_node2", BRIDGE),
    ("04 Package movement (Motion model)", ML, "model/control", "motion_activate_only_if_chunks_pasted",
     control(G, "ACTION_ACTIVATE", "MODEL_MOTION", transfer=TRANSFER["motion"])),
    ("04 Package movement (Motion model)", ML, "model/control", "motion_inputs",
     control(G + 1, "ACTION_SET_INPUTS", "MODEL_MOTION", [(1, 0, 0), (1, 1, 1), (1, 2, 2)])),
    ("06 Switch the model (Motion to Environment)", ML, "model/control", "env_activate",
     control(G + 2, "ACTION_ACTIVATE", "MODEL_ENVIRONMENT", transfer=TRANSFER["environment"])),
    ("06 Switch the model (Motion to Environment)", ML, "model/control", "env_inputs_dht22",
     control(G + 3, "ACTION_SET_INPUTS", "MODEL_ENVIRONMENT", [(2, 1, 0), (2, 0, 1)])),
    ("07 Same model, different sensor", ML, "config", "pathA_swap_to_bme280", cfg([{"sensor": "bme280"}])),
    ("07 Same model, different sensor", ML, "model/control", "pathA_env_inputs_bme280",
     control(G + 4, "ACTION_SET_INPUTS", "MODEL_ENVIRONMENT", [(5, 0, 0), (5, 2, 1)])),
    ("07 Same model, different sensor", ML, "config", "pathB_i2c_off", cfg([{"sensor": "bme280", "enabled": False}])),
    ("07 Same model, different sensor", ML, "config", "pathB_max6675_2s", cfg([{"sensor": "max6675", "period_ms": 2000}])),
    ("07 Same model, different sensor", ML, "model/control", "pathB_env_inputs_max6675",
     control(G + 4, "ACTION_SET_INPUTS", "MODEL_ENVIRONMENT", [(8, 0, 0), (2, 0, 1)])),
    ("08 Reject an invalid mapping", ML, "model/control", "badmap_humidity_as_temperature",
     control(G + 5, "ACTION_SET_INPUTS", "MODEL_ENVIRONMENT", [(2, 0, 0), (2, 0, 1)])),
    ("09 Storage mode (node1)", ML, "config", "modeB",
     cfg([{"sensor": "hcsr04"}, {"sensor": "max6675"}, {"sensor": "gy511", "enabled": False}])),
    ("09 Storage mode (node1)", ML, "rules", "modeB", rules_json(demo.rules_for("B", "ml"))),
    ("10 CO2 on node2", ST, "config", "modeB", cfg([{"sensor": "mhz19b"}])),
    ("10 CO2 on node2", ST, "rules", "modeB", rules_json(demo.rules_for("B", "station"))),
    ("11 New model over MQTT (anomaly)", ML, "model/control", "anomaly_activate",
     control(G + 6, "ACTION_ACTIVATE", "MODEL_ANOMALY", transfer=TRANSFER["anomaly"])),
    ("11 New model over MQTT (anomaly)", ML, "model/control", "anomaly_inputs",
     control(G + 7, "ACTION_SET_INPUTS", "MODEL_ANOMALY", [(6, 0, 7), (8, 0, 9)])),
    ("11 New model over MQTT (anomaly)", ML, "model/control", "anomaly_badmap_distance_as_temperature",
     control(G + 8, "ACTION_SET_INPUTS", "MODEL_ANOMALY", [(6, 0, 9)])),
    ("99 Troubleshooting", ML, "model/control", "query_model_generation", {"generation": 1, "action": "ACTION_QUERY"}),
    ("99 Troubleshooting", ML, "config", "probe_bme280_0x76", bme280_probe(0x76)),
    ("99 Troubleshooting", ML, "config", "probe_bme280_0x77", bme280_probe(0x77)),
    ("99 Troubleshooting", ST, "config", "probe_rc522_version", rc522_probe()),
]


def write_chunks(out=OUT):
    cdir = out / "chunks"
    cdir.mkdir(exist_ok=True)
    for old in cdir.glob("*.hex"):
        old.unlink()
    rows = []
    for name, (kind, rel) in PACKAGES.items():
        pkg = (demo.ROOT / rel).read_bytes()
        chunks = demo.nc.make_chunks(pkg, kind, TRANSFER[name], CHUNK, demo.nc.package_version(pkg))
        back = b"".join(pb.ModelChunk.FromString(c).data for c in chunks)
        digest = hashlib.sha256(pkg).digest()
        assert back == pkg and all(pb.ModelChunk.FromString(c).sha256 == digest for c in chunks)
        for i, c in enumerate(chunks, 1):
            (cdir / f"{name}_chunk_{i}_of_{len(chunks)}.hex").write_text(c.hex(), encoding="utf-8")
        rows.append(f"| {name} | {TRANSFER[name]} | {len(pkg)} bytes | {len(chunks)} |")
    return rows


def main(out=OUT):
    out.mkdir(parents=True, exist_ok=True)
    for old in out.glob("*.json"):
        old.unlink()
    md = ["# FlexiCargo copy-paste pack", "",
          "Generated by `demo/make_copypaste.py`; do not edit by hand. Publish each JSON in MQTTX to the topic shown,",
          "payload format **JSON**, QoS 1. `json_in` topics need the JSON helper running:",
          "`.\\.venv\\Scripts\\python.exe tools\\json_bridge_all.py --broker 127.0.0.1`", "",
          f"Model-control generations start at {G}. If one is refused as `STALE_GENERATION`, send the",
          "`query_model_generation` message (section 99) and use the reply's `generation` + 1.", ""]
    scene = None
    for n, (sc, node, topic, what, doc) in enumerate(ITEMS, 1):
        name = f"{n:02d}_s{sc[:2]}_{node}_{topic.replace('/', '-')}_{what}.json"
        (out / name).write_bytes((json.dumps(doc, indent=1) + "\n").encode("utf-8"))
        if sc != scene:
            md += [f"## {sc}", ""]
            scene = sc
        md += [f"**{node}**, topic `aai16/{node}/json_in/{topic}` (file `{name}`)", "", "```json",
               json.dumps(doc), "```", ""]
    rows = write_chunks(out)
    md += [f"## Model chunks (topic `aai16/{ML}/in/model/chunk`, payload format **Hex**, QoS 1)", "",
           "Send the files of one model in order, waiting for `OK` on `model_chunk_ack` after each one.", "",
           "| Model | Transfer id | Size | Chunks |", "|---|---|---|---|", *rows, ""]
    (out / "ALL.md").write_bytes("\n".join(md).encode("utf-8"))
    print(f"wrote {len(ITEMS)} messages and {len(rows)} chunked models to {out}")
    print("\n".join(rows))


if __name__ == "__main__":
    main()
