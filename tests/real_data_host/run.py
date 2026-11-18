"""Real-data path: tools/collect_readings.py -> CSV -> ml/real_data.py -> ml/train_*.py --data -> model package.

The "sensor" is simulated (windows from the synthetic generators streamed at the model's real sample period), so this
proves the PLUMBING (recorder format, gap/jitter rejection, session-wise split, retrain, package, version), not
accuracy on real sensors. Run:  .venv\\Scripts\\python.exe tests\\real_data_host\\run.py
"""
import importlib.util
import json
import os
import pathlib
import shutil
import subprocess
import sys
import time

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "3")
os.environ.setdefault("TF_ENABLE_ONEDNN_OPTS", "0")
ROOT = pathlib.Path(__file__).resolve().parents[2]
HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "_gen"
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "ml"))
import numpy as np  # noqa: E402
import common as C  # noqa: E402

pb = C.pb
spec = importlib.util.spec_from_file_location("client_mqtt_run", ROOT / "tests" / "client_mqtt" / "run.py")
cm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cm)

ok = True


def check(name, cond, detail=""):
    global ok
    ok &= bool(cond)
    print(f"  {'ok  ' if cond else 'FAIL'} {name}" + (f"   [{detail}]" if detail else ""))


def write_csv(path, label, columns, t0, period_us, windows, jitter_us=0, gap_after=None):
    """A recording: consecutive windows streamed back to back at the sample period."""
    path.parent.mkdir(parents=True, exist_ok=True)
    rows, t = [], t0
    for wi, w in enumerate(windows):
        for r in w:
            rows.append((t, r))
            t += period_us + (jitter_us if jitter_us and (len(rows) % 7 == 0) else 0)
        if gap_after is not None and wi == gap_after:
            t += 10 * period_us  # a dropout in the recording
    with open(path, "w", newline="") as f:
        f.write(f"# node=test sensor=0 columns={columns}\n")
        f.write("t_us,label," + ",".join(f"v{i}" for i in range(len(rows[0][1]))) + "\n")
        for t, r in rows:
            f.write(f"{t},{label}," + ",".join(f"{x:.6g}" for x in r) + "\n")


def train(task, data, out, version):
    r = subprocess.run([sys.executable, str(ROOT / "ml" / f"train_{task}.py"), "--data", str(data), "--out", str(out), "--version", str(version)],
                       capture_output=True, text=True)
    return r.returncode, r.stdout + r.stderr


def main():
    rng = np.random.default_rng(11)
    if OUT.exists():
        shutil.rmtree(OUT)
    # ---- 1. the recorder against a real broker: format is what the loader expects
    stop_broker = cm.start_broker()
    try:
        import paho.mqtt.client as mqtt
        pub = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="sim-node")
        pub.connect("127.0.0.1", cm.PORT, 30)
        pub.loop_start()
        csv_path = OUT / "recorded" / "moving.csv"
        proc = subprocess.Popen([sys.executable, str(ROOT / "tools" / "collect_readings.py"), "--broker", "127.0.0.1", "--port", str(cm.PORT),
                                 "--node", "n1", "--sensor", "1", "--label", "moving", "--seconds", "3", "--out", str(csv_path)],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        time.sleep(1.2)
        t = 5_000_000
        for i in range(60):
            m = pb.Reading(sensor_id=1, valid=(i != 30), timestamp_us=t)
            m.values.extend([0.01 * i, 0.5, 1.0])
            pub.publish("aai16/n1/out/reading/1", m.SerializeToString(), qos=0)
            pub.publish("aai16/n1/out/reading/2", pb.Reading(sensor_id=2, valid=True, timestamp_us=t).SerializeToString(), qos=0)  # other sensor: ignored
            t += 20_000
            time.sleep(0.01)
        out, _ = proc.communicate(timeout=20)
        pub.loop_stop()
        lines = csv_path.read_text().splitlines()
        check("recorder wrote header + only valid readings of the chosen sensor", lines[0].startswith("# node=n1 sensor=1 columns=ax=0,ay=1,az=2") and lines[1] == "t_us,label,v0,v1,v2" and len(lines) == 2 + 59, f"{len(lines) - 2} rows; {out.strip().splitlines()[-1]}")
        check("rows carry the device timestamp and the label", lines[2].split(",")[:2] == ["5000000", "MOVING"])
    finally:
        stop_broker()

    # ---- 2. recordings -> windows -> retrained models (motion)
    data = OUT / "data_motion"
    for c, name in enumerate(C.MOTION_LABELS):
        for s in range(4):  # 4 separate sessions per class
            wins = [C.generate_motion(c, rng, n=64 * 12)]   # ONE continuous stream, like a real recording
            write_csv(data / name.lower() / f"{name.lower()}_{s}.csv", name, "ax=0,ay=1,az=2", 1_000_000 * (s + 1), 20_000, wins)
    code, log = train("motion", data, OUT / "model_motion", 7)
    rep = json.loads((OUT / "model_motion" / "report.json").read_text()) if code == 0 else {}
    check("motion: retrain from recordings succeeds", code == 0, log.strip().splitlines()[-4] if code else "")
    check("report says REAL data and a session-wise split", "REAL recordings" in rep.get("data_provenance", "") and rep.get("split_mode") == "by_session")
    check("every class has windows from 4 files", all(v == 4 for v in rep["real_data_stats"]["per_class_files"].values()) if rep else False)
    check("int8 test accuracy is high on (simulated) recordings", rep.get("int8_test_accuracy", 0) > 0.9, f"{rep.get('int8_test_accuracy')}")
    if code == 0:
        c_, _ = C.parse_package((OUT / "model_motion" / "motion.amdl").read_bytes())
        check("package parses, carries the requested model_version 7 and the motion contract", c_.model_version == 7 and c_.kind == pb.MODEL_MOTION and c_.window_samples == 64)

    # ---- 3. environment: DHT22 order (value0 = humidity) and BME280 order (value0 = temperature) map to the same windows
    sys.path.insert(0, str(ROOT / "ml"))
    import pipeline
    import real_data
    wins = [C.generate_env(1, rng) for _ in range(4)]            # columns: temperature, humidity
    d1, d2 = OUT / "env_dht", OUT / "env_bme"
    write_csv(d1 / "changing_0.csv", "CHANGING", "temperature=1,humidity=0", 1_000_000, 2_000_000, [w[:, ::-1] for w in wins])
    write_csv(d2 / "changing_0.csv", "CHANGING", "temperature=0,humidity=1", 1_000_000, 2_000_000, wins)
    w1, y1, _, _ = real_data.load_task(pipeline.TASKS["environment"], d1, 5)
    w2, y2, _, _ = real_data.load_task(pipeline.TASKS["environment"], d2, 5)
    check("DHT22-ordered and BME280-ordered recordings produce identical (temperature, humidity) windows", w1.shape == w2.shape and np.allclose(w1, w2) and len(w1) > 0, f"{len(w1)} windows")
    data = OUT / "data_env"
    for c, name in enumerate(C.ENV_LABELS):
        for s in range(3):
            write_csv(data / name.lower() / f"{name.lower()}_{s}.csv", name, "temperature=0,humidity=1", 1_000_000, 2_000_000, [C.generate_env(c, rng) for _ in range(10)])
    code, log = train("environment", data, OUT / "model_env", 3)
    rep = json.loads((OUT / "model_env" / "report.json").read_text()) if code == 0 else {}
    check("environment: retrain from recordings completes and writes a package (accuracy of glued fake data is not meaningful)", code == 0 and (OUT / "model_env" / "environment.amdl").exists())
    # ---- 4. things the loader must refuse or flag
    data = OUT / "data_bad_period"
    for c, name in enumerate(C.MOTION_LABELS):
        for s in range(3):
            write_csv(data / f"{name}_{s}.csv", name, "ax=0,ay=1,az=2", 0, 100_000, [C.generate_motion(c, rng) for _ in range(5)])  # 10 Hz, model needs 50 Hz
    code, log = train("motion", data, OUT / "model_bad", 1)
    check("recordings at the wrong sample rate are rejected (no windows)", code != 0 and "no usable windows" in log)

    data = OUT / "data_missing_class"
    for c, name in enumerate(C.MOTION_LABELS[:2]):
        for s in range(3):
            write_csv(data / f"{name}_{s}.csv", name, "ax=0,ay=1,az=2", 0, 20_000, [C.generate_motion(c, rng) for _ in range(5)])
    code, log = train("motion", data, OUT / "model_missing", 1)
    check("a missing class is refused", code != 0 and "VIBRATING" in log)

    data = OUT / "data_gap"
    for c, name in enumerate(C.MOTION_LABELS):
        for s in range(3):
            write_csv(data / f"{name}_{s}.csv", name, "ax=0,ay=1,az=2", 0, 20_000, [C.generate_motion(c, rng) for _ in range(8)], gap_after=3)
    code, log = train("motion", data, OUT / "model_gap", 1)
    rep = json.loads((OUT / "model_gap" / "report.json").read_text()) if code == 0 else {}
    check("a dropout inside a recording splits it into runs instead of making a bad window", code == 0 and rep["real_data_stats"]["runs"] >= 18, f"runs={rep.get('real_data_stats', {}).get('runs')}")

    data = OUT / "data_two_sessions"
    for c, name in enumerate(C.MOTION_LABELS):
        for s in range(2):
            write_csv(data / f"{name}_{s}.csv", name, "ax=0,ay=1,az=2", 0, 20_000, [C.generate_motion(c, rng) for _ in range(10)])
    code, log = train("motion", data, OUT / "model_two", 1)
    rep = json.loads((OUT / "model_two" / "report.json").read_text()) if code == 0 else {}
    check("fewer than 3 sessions per class: falls back to blocks and says it is optimistic", code == 0 and "optimistic" in rep.get("split_mode", ""))

    print("\nREAL-DATA PATH OK" if ok else "\nREAL-DATA PATH FAILED", flush=True)
    os._exit(0 if ok else 1)


if __name__ == "__main__":
    main()
