"""Record a node's sensor readings (from MQTT) into a CSV file for ML training.

  python tools/collect_readings.py --broker 192.168.1.10 --node node1 --sensor 1 --label MOVING \\
         --seconds 120 --out data/motion/moving_01.csv                      # GY-511 (sensor id 1)
  python tools/collect_readings.py ... --sensor 2 --label CHANGING --columns temperature=1,humidity=0 \\
         --seconds 600 --out data/environment/changing_01.csv               # DHT22 (value 0 = humidity, 1 = temperature)
  python tools/collect_readings.py ... --sensor 5 --columns temperature=0,humidity=2 ...   # BME280

CSV: a comment line "# node=... sensor=... columns=name=index,..." then "t_us,label,v0,v1,..." (valid readings only).
Rules for good data (see docs/ml_model_contracts.md): one label per file; several separate sessions per class;
record BOTH temperature sensors for the environment model; keep the node's sensor period equal to the model's
(motion 20 ms, environment 2000 ms), otherwise the loader will reject the windows.
"""
import argparse
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from profile_lib import load_pb  # noqa: E402

pb = load_pb()
DEFAULT_COLUMNS = {"motion": "ax=0,ay=1,az=2", "environment": "temperature=1,humidity=0"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", required=True)
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--node", required=True)
    ap.add_argument("--sensor", type=int, required=True)
    ap.add_argument("--label", required=True, help="STATIONARY|MOVING|VIBRATING|STABLE|CHANGING")
    ap.add_argument("--columns", help="name=value_index,... (default: ax=0,ay=1,az=2 for labels of the motion model, "
                                       "temperature=1,humidity=0 for the environment model)")
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    label = a.label.upper()
    if label in ("STATIONARY", "MOVING", "VIBRATING"):
        columns = a.columns or DEFAULT_COLUMNS["motion"]
    elif label in ("STABLE", "CHANGING"):
        columns = a.columns or DEFAULT_COLUMNS["environment"]
    else:
        sys.exit("label must be STATIONARY, MOVING, VIBRATING, STABLE or CHANGING")

    import paho.mqtt.client as mqtt
    rows, invalid = [], 0

    def on_message(_c, _u, msg):
        nonlocal invalid
        r = pb.Reading.FromString(msg.payload)
        if r.sensor_id != a.sensor:
            return
        if r.valid:
            rows.append((r.timestamp_us, list(r.values)))
        else:
            invalid += 1

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"aai16-collect-{int(time.time())}")
    c.on_message = on_message
    c.connect(a.broker, a.port, 30)
    c.subscribe(f"aai16/{a.node}/out/reading/{a.sensor}", qos=0)
    c.loop_start()
    print(f"recording sensor {a.sensor} as {label} for {a.seconds:.0f} s ...", flush=True)
    time.sleep(a.seconds)
    c.loop_stop()
    c.disconnect()

    out = pathlib.Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    n = max((len(v) for _, v in rows), default=0)
    with open(out, "w", newline="") as f:
        f.write(f"# node={a.node} sensor={a.sensor} columns={columns}\n")
        f.write("t_us,label," + ",".join(f"v{i}" for i in range(n)) + "\n")
        for t, v in rows:
            f.write(f"{t},{label}," + ",".join(f"{x:.6g}" for x in v) + "\n")
    span = (rows[-1][0] - rows[0][0]) / 1e6 if len(rows) > 1 else 0
    print(f"wrote {len(rows)} valid readings ({invalid} invalid skipped, {span:.1f} s of data) to {out}")
    if not rows:
        sys.exit("no readings received: is the sensor configured and the node publishing?")


if __name__ == "__main__":
    main()
