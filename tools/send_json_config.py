"""Send a JSON config file to the node as a real Protobuf ConfigApply and print the ACK in words.

  python tools/send_json_config.py --broker 172.20.10.2 --node node1 FILE.json [--gen N] [--period MS [--sensor ID]]

The JSON uses the .proto field names (enums as numbers, bytes as base64), the same text you would paste into MQTTX.
Only binary Protobuf goes on the wire; JSON is just the input format here.
Without --gen, "generation": "auto" (or 0 / missing) uses the node's current generation + 1.
--gen overrides "generation"; --period overrides period_ms (of one --sensor, or of every profile).
"""
import argparse
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from google.protobuf import json_format  # noqa: E402

import node_client as nc  # noqa: E402
import config_shorthand as short  # noqa: E402

pb = nc.pb


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", required=True)
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--node", default="node1")
    ap.add_argument("file")
    ap.add_argument("--gen", type=int)
    ap.add_argument("--period", type=int, help="period_ms override")
    ap.add_argument("--sensor", type=int, help="apply --period only to this sensor id")
    a = ap.parse_args()

    doc = json.loads(pathlib.Path(a.file).read_text(encoding="utf-8"))
    doc = short.expand(doc)            # short form ({"sensors": [...]} / {"preset": "A"}) -> full; full passes through
    g = doc.get("generation")
    auto = a.gen is None and (g is None or g == 0 or (isinstance(g, str) and g.strip().lower() in ("", "0", "auto")))
    if auto or isinstance(g, str):
        doc["generation"] = 0          # filled in below from the node's heartbeat
    msg = json_format.ParseDict(doc, pb.ConfigApply())
    if a.gen is not None:
        msg.generation = a.gen
    if a.period is not None:
        for p in msg.profiles:
            if a.sensor is None or p.sensor_id == a.sensor:
                p.period_ms = a.period
    n = None
    if auto:
        n = nc.Node(a.broker, a.node, a.port)
        hb = n.wait("heartbeat", lambda m: True, 15)
        if not hb:
            print("no heartbeat from the node in 15 s, so the next generation is unknown (use --gen N)")
            return 1
        msg.generation = hb.config_generation + 1
        print(f"node is at generation {hb.config_generation}, using {msg.generation} (auto)")
    payload = msg.SerializeToString()
    print(f"sending generation {msg.generation}, {len(msg.profiles)} profile(s), {len(payload)} protobuf bytes")

    if n is None:
        n = nc.Node(a.broker, a.node, a.port)
    n.send("config", payload)
    # wait() returns the first reply after the request that satisfies the predicate, so ask for the final one
    ack = n.wait("config_ack", lambda m: m.phase in (pb.PHASE_APPLIED, pb.PHASE_REJECTED), 15)
    if not ack:
        print("no ACK (node offline, or ACK lost)")
        return 1
    phase = pb.ApplyPhase.Name(ack.phase).replace("PHASE_", "")
    reason = pb.Reason.Name(ack.reason).replace("REASON_", "") if ack.reason else ""
    print(f"ACK generation={ack.generation} phase={phase}" + (f" reason={reason}" if reason else ""))
    return 0 if ack.phase == pb.PHASE_APPLIED else 2


if __name__ == "__main__":
    sys.exit(main())
