"""Host test: the FlexiCargo copy-paste pack (demo/copypaste) is valid end to end.

Checks, without any board:
  1. every JSON message converts to Protobuf exactly as the laptop JSON helper does it (short form expanded,
     "generation": "auto" filled in), for the message type of its topic;
  2. every sensor config, rule set and bridge passes the FIRMWARE's own validators (C, compiled for the host);
  3. model-control generations rise in demo order, and every ACTIVATE uses the transfer id of its chunk files;
  4. every model chunk file decodes, the chunks reassemble to the package byte for byte, and the SHA-256 matches;
  5. every SET_INPUTS mapping that should be accepted gives each model channel a sensor value of the kind the model
     contract demands, and every deliberately bad mapping really breaks that rule (so the node must refuse it).
  6. every ready-to-send example in configs/ converts and passes the same firmware validators;
  7. the committed pack is up to date: regenerating it with demo/make_copypaste.py gives the same files.
The model chunks are not stored in git; they are regenerated here (into _gen/pack/chunks) and checked.

Run:  .venv\\Scripts\\python.exe tests\\demo_pack_host\\run.py
"""
import hashlib
import json
import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "_gen"
PACK = ROOT / "demo" / "copypaste"
sys.path.insert(0, str(ROOT / "tools"))
from google.protobuf import json_format  # noqa: E402

import config_shorthand as short  # noqa: E402
import node_client as nc  # noqa: E402

pb = nc.pb
sys.stdout.reconfigure(line_buffering=True)  # keep our lines in order with the C program's output
TYPES = {"config": pb.ConfigApply, "rules": pb.RuleSetApply, "bridge": pb.BridgeApply, "command": pb.NodeCommand,
         "model-control": pb.ModelControl}
CONTRACTS = {"MODEL_MOTION": "motion", "MODEL_ENVIRONMENT": "environment", "MODEL_ANOMALY": "anomaly_real"}
fails = 0


def check(name, cond, extra=""):
    global fails
    fails += 0 if cond else 1
    print(f"  {'ok  ' if cond else 'FAIL'} {name} {extra}".rstrip())


def topic_of(name):
    m = re.match(r"^\d+_s\d+_(node\d+)_(config|rules|bridge|command|model-control)_", name)
    assert m, f"unexpected pack file name {name}"
    return m.group(1), m.group(2)


def main():
    subprocess.run([sys.executable, str(ROOT / "tools" / "generate_proto.py")], stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL, check=True)
    OUT.mkdir(exist_ok=True)
    for old in OUT.glob("*.bin"):
        old.unlink()

    files = sorted(PACK.glob("*.json"))
    check("copy-paste pack exists", len(files) > 0, f"({len(files)} messages)")

    print("0) the committed pack matches a fresh regeneration")
    sys.path.insert(0, str(ROOT / "demo"))
    import make_copypaste  # noqa: E402  (imports demo/demo.py and demo/rig.json)
    fresh = OUT / "pack"
    make_copypaste.main(fresh)
    norm = lambda p: p.read_bytes().replace(b"\r\n", b"\n")  # noqa: E731 - Windows checkouts may use CRLF
    committed = sorted(p.name for p in PACK.glob("*.json")) + ["ALL.md"]
    generated = sorted(p.name for p in fresh.glob("*.json")) + ["ALL.md"]
    check("same message files", committed == generated)
    stale = [n for n in generated if (PACK / n).exists() and norm(PACK / n) != norm(fresh / n)]
    check("every committed message is identical to the regenerated one", not stale, str(stale) if stale else "")

    # sensor id -> output kinds of the profile the pack configures (for the ML mapping checks)
    kinds_of = {}
    controls = []
    print("1) JSON -> Protobuf, as the JSON helper converts it")
    for f in files:
        node, topic = topic_of(f.name)
        doc = json.loads(f.read_text(encoding="utf-8"))
        body = dict(doc)
        if topic == "config":
            body = short.expand(body)
        if body.get("generation") == "auto":
            body["generation"] = 1
        try:
            msg = json_format.ParseDict(body, TYPES[topic]())
            ok = True
        except Exception as e:  # noqa: BLE001 - any parse error is a test failure
            ok, msg = False, None
            print(f"       {e}")
        check(f"{f.name}", ok)
        if not ok:
            continue
        ext = {"config": "config", "rules": "rules", "bridge": "bridge"}.get(topic)
        if ext:
            (OUT / f"{f.stem}.{ext}.bin").write_bytes(msg.SerializeToString())
        if topic == "config":
            for p in msg.profiles:
                if p.enabled or p.dormant:
                    kinds_of[p.sensor_id] = [pb.ValueKind.Name(k) for k in p.output_kinds]
        if topic == "model-control":
            # snapshot: a mapping is judged against the sensors configured at that point of the demo
            controls.append((f.name, doc, {k: list(v) for k, v in kinds_of.items()}))

    print("1b) configs/ examples -> Protobuf")
    for f in sorted((ROOT / "configs").rglob("*.json")):
        folder = f.parent.name
        topic = {"sensors": "config", "presets": "config", "short_form": "config", "rules": "rules", "bridge": "bridge"}[folder]
        body = json.loads(f.read_text(encoding="utf-8"))
        if topic == "config":
            body = short.expand(body)
        if body.get("generation") == "auto":
            body["generation"] = 1
        try:
            msg = json_format.ParseDict(body, TYPES[topic]())
            ok = True
        except Exception as e:  # noqa: BLE001 - any parse error is a test failure
            ok = False
            print(f"       {e}")
        check(f"configs/{folder}/{f.name}", ok)
        if ok:
            (OUT / f"example_{folder}_{f.stem}.{topic}.bin").write_bytes(msg.SerializeToString())

    print("2) firmware validators (cm_validate_apply, re_validate, bridge_validate)")
    exe = OUT / "test_demo_pack.exe"
    comps = ROOT / "components"
    srcs = [HERE / "test_demo_pack.c", comps / "node_services/rule_engine.c", comps / "node_services/bridge.c",
            comps / "config_manager/config_manager.c", comps / "ml_core/ml_selector.c", comps / "ml_core/ml_features.c",
            comps / "aai16_protocol/generated/aai16.pb.c", comps / "aai16_protocol/nanopb/pb_common.c",
            comps / "aai16_protocol/nanopb/pb_decode.c", comps / "aai16_protocol/nanopb/pb_encode.c"]
    cmd = [sys.executable, "-m", "ziglang", "cc", "-std=c11", "-Wall", "-Wextra", "-g",
           f"-I{comps / 'node_services/include'}", f"-I{comps / 'config_manager/include'}",
           f"-I{comps / 'ml_core/include'}", f"-I{comps / 'aai16_protocol/generated'}",
           f"-I{comps / 'aai16_protocol/nanopb'}", "-o", str(exe), *map(str, srcs)]
    subprocess.run(cmd, check=True)
    rc = subprocess.run([str(exe), str(OUT)]).returncode
    check("every config, rule set and bridge passes the firmware validators", rc == 0)

    print("3) generations and transfer ids")
    gens = [(n, d["generation"]) for n, d, _ in controls if d["action"] != "ACTION_QUERY"]
    rising = all(b >= a for (_, a), (_, b) in zip(gens, gens[1:]))
    check("model-control generations never go down in demo order", rising, str([g for _, g in gens]))
    chunk_dir = fresh / "chunks"  # regenerated above; chunks are not stored in git
    transfers = {}
    for cf in chunk_dir.glob("*.hex"):
        m = pb.ModelChunk.FromString(bytes.fromhex(cf.read_text(encoding="utf-8").strip()))
        transfers.setdefault(pb.ModelKind.Name(m.kind), set()).add(m.transfer_id)
    for n, d, _ in controls:
        if d["action"] == "ACTION_ACTIVATE":
            check(f"{n}: transfer_id {d['transfer_id']} matches the {d['kind']} chunk files",
                  transfers.get(d["kind"]) == {d["transfer_id"]})

    print("4) model chunk files")
    for model, rel in (("motion", "ml/artifacts/motion/motion.amdl"),
                       ("environment", "ml/artifacts/environment/environment.amdl"),
                       ("anomaly", "ml/artifacts/anomaly_real/anomaly.amdl")):
        pkg = (ROOT / rel).read_bytes()
        parts = sorted(chunk_dir.glob(f"{model}_chunk_*_of_*.hex"), key=lambda p: int(p.stem.split("_")[2]))
        msgs = [pb.ModelChunk.FromString(bytes.fromhex(p.read_text(encoding="utf-8").strip())) for p in parts]
        check(f"{model}: {len(msgs)} chunks, seq 0..n-1 in order", [m.seq for m in msgs] == list(range(len(msgs))))
        check(f"{model}: every chunk <= 1024 data bytes", all(1 <= len(m.data) <= 1024 for m in msgs))
        check(f"{model}: chunks reassemble to the package", b"".join(m.data for m in msgs) == pkg)
        check(f"{model}: SHA-256 in every chunk matches", all(m.sha256 == hashlib.sha256(pkg).digest() for m in msgs))

    print("5) ML input mappings against the model contracts")
    for n, d, kinds_at in controls:
        if d["action"] != "ACTION_SET_INPUTS":
            continue
        channels = json.loads((ROOT / "ml/artifacts" / CONTRACTS[d["kind"]] / "contract.json").read_text())["channels"]
        kinds_ok = all(i["sensor_id"] in kinds_at and i["value_index"] < len(kinds_at[i["sensor_id"]])
                       and kinds_at[i["sensor_id"]][i["value_index"]] == channels[i["feature_slot"]] for i in d["inputs"])
        if "badmap" in n:
            check(f"{n}: deliberately wrong (a kind does not match, so the node must refuse it)", not kinds_ok)
        else:
            check(f"{n}: every channel gets a value of the contract's kind", kinds_ok)

    print("\nDEMO PACK OK" if fails == 0 else f"\n{fails} demo pack check(s) FAILED")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
