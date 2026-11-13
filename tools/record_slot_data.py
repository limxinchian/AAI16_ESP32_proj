"""Record raw sensor readings from one or two nodes into a CSV for training / testing the anomaly model.

  python tools/record_slot_data.py --broker 172.20.10.2 --seconds 720 --out data/anomaly/normal_01.csv \
         --source node1:5 --source node1:8 --source node2:3

Each --source is NODE:SENSOR_ID. Columns: node,sensor_id,timestamp_us,valid,v0,v1,v2 (timestamp = the node's own clock;
only differences within one sensor are used). Keep conditions NORMAL while recording the training file (nothing near
the radar, no breathing/heating on the sensors); record a second file with deliberate disturbances to TEST the model.
"""
import argparse
import csv
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import node_client as nc  # noqa: E402

pb = nc.pb


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", required=True)
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--seconds", type=float, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--source", action="append", required=True, help="NODE:SENSOR_ID (repeatable)")
    ap.add_argument("--note", action="append", default=[], help="SECONDS:TEXT marker printed while recording")
    a = ap.parse_args()
    want = {}
    for s in a.source:
        node, sid = s.split(":")
        want[(node, int(sid))] = 0
    import paho.mqtt.client as mqtt
    msgs = []
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"aai16-rec-{int(time.time())}")
    c.on_message = lambda _c, _u, m: msgs.append((m.topic, bytes(m.payload)))
    c.connect(a.broker, a.port)
    for node, sid in want:
        c.subscribe(f"aai16/{node}/out/reading/{sid}", 0)
    c.loop_start()
    out = pathlib.Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    notes = sorted((float(x.split(":", 1)[0]), x.split(":", 1)[1]) for x in a.note)
    done = 0
    with out.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["node", "sensor_id", "timestamp_us", "valid", "v0", "v1", "v2"])
        seen = 0
        while time.time() - t0 < a.seconds:
            time.sleep(0.25)
            batch, seen = msgs[seen:], len(msgs)
            for topic, payload in batch:
                parts = topic.split("/")
                node, sid = parts[1], int(parts[4])
                m = pb.Reading()
                m.ParseFromString(payload)
                v = list(m.values) + [0.0] * 3
                w.writerow([node, sid, m.timestamp_us, int(m.valid), v[0], v[1], v[2]])
                want[(node, sid)] += 1
            f.flush()
            el = time.time() - t0
            while done < len(notes) and el >= notes[done][0]:
                print(f">>> {notes[done][0]:.0f}s: {notes[done][1]}", flush=True)
                done += 1
    c.loop_stop()
    print("rows per source:", {f"{n}:{s}": k for (n, s), k in want.items()}, "->", out)


if __name__ == "__main__":
    main()
