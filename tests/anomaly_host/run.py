"""Host tests for Model C (anomaly detector): slot statistics, scoring, package, selector with partial mappings.

The C code (components/ml_core) is compared with the independent Python reference in ml/anomaly.py and with the
trained artifact in ml/artifacts/anomaly (run ml/train_anomaly.py first).

Run:  .venv\\Scripts\\python.exe tests\\anomaly_host\\run.py
"""
import json
import math
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "_gen"
ART = ROOT / "ml" / "artifacts" / "anomaly"
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "ml"))
import anomaly as A  # noqa: E402


def f9(x):
    return f"{float(x):.9g}"


def reference_scenario(events, mapping, max_age_ms, interval_ms):
    """Independent re-implementation of the selector's anomaly rules (see ml/ANOMALY_SPEC.md)."""
    trackers = [A.SlotTracker(s) for s in range(A.N_SLOTS)]
    last_ts = [0] * A.N_SLOTS
    have, latest = {}, {}
    last_infer = 0
    out = []
    for sid, valid, ts, vals in events:
        touched = set()
        for ch, (msid, vi) in mapping.items():
            if msid != sid:
                continue
            slot = A.SLOT_OF_CHANNEL[ch]
            if not valid:
                have[ch] = False
                trackers[slot] = A.SlotTracker(slot)
                last_ts[slot] = 0
                continue
            latest[ch], have[ch] = vals[vi], True
            touched.add(slot)
        for slot in sorted(touched):
            if slot == 0:
                if not all(have.get(c) for c in (0, 1, 2)):
                    continue
                x = math.sqrt(latest[0] ** 2 + latest[1] ** 2 + latest[2] ** 2)
            else:
                x = latest[slot + 2]
            trackers[slot].update(x, ts / 1e6)
            last_ts[slot] = ts
        if touched and (last_infer == 0 or (ts >= last_infer and ts - last_infer >= interval_ms * 1000)):
            feats, mask = [0.0] * A.N_FEATURES, 0
            for slot in range(A.N_SLOTS):
                if trackers[slot].n >= A.MIN_READINGS and ts >= last_ts[slot] and ts - last_ts[slot] <= max_age_ms * 1000:
                    feats[slot * 4:(slot + 1) * 4] = trackers[slot].features()
                    mask |= 1 << slot
            if mask:
                last_infer = ts
                out.append((ts, mask, feats))
    return out


def make_scenario():
    """~40 s of readings: GY-511 (id 1, 100 ms), BME280 (id 5, 500 ms), MH-Z19B (id 7, 2 s); id 7 goes silent from
    12 s to 30 s (stale slot); id 5 delivers one invalid reading at 34 s (slots reset and re-warm)."""
    ev = []
    for k in range(0, 400):
        t = k * 100_000
        ev.append((1, True, t, [0.01 * math.sin(k / 7.0), 0.02 * math.cos(k / 5.0), 1.0 + 0.01 * math.sin(k / 3.0)]))
    for k in range(0, 80):
        t = k * 500_000
        ev.append((5, k != 68, t + 1, [25.0 + 0.001 * k, 101300.0 + 8.0 * math.sin(k), 60.0 + 0.5 * math.sin(k / 5.0)]))
    for k in range(0, 20):
        t = k * 2_000_000
        if 12_000_000 <= t < 30_000_000:
            continue
        ev.append((7, True, t + 2, [600.0 + 3.0 * math.sin(k)]))
    # readings travel as float32 on the node (pressure ~101300 Pa has 0.008 Pa resolution): feed the reference the same values
    import numpy as np
    ev = [(s, v, t, [float(np.float32(x)) for x in vals]) for s, v, t, vals in ev]
    ev.sort(key=lambda e: (e[2], e[0]))
    # channel -> (sensor id, value index): motion from sensor 1, temperature / humidity / pressure from 5, CO2 from 7
    mapping = {0: (1, 0), 1: (1, 1), 2: (1, 2), 3: (5, 0), 4: (5, 2), 5: (5, 1), 6: (7, 0)}
    return ev, mapping


def write_cases():
    OUT.mkdir(exist_ok=True)
    g = json.loads((ART / "golden.json").read_text())
    c = json.loads((ART / "contract.json").read_text())
    L = []
    q = g["quant"]
    L.append(f"Q {q['s_in']!r} {q['z_in']} {q['s_out']!r} {q['z_out']} {g['threshold']!r} {c['maxAgeMs']} {c['samplePeriodMs']}")
    for s in g["slot_series"]:
        L.append(f"S {s['slot']} {len(s['t'])}")
        L.append(" ".join(f9(x) for x in s["t"]))
        L.append(" ".join(f9(x) for x in s["x"]))
        L.append(" ".join(f9(x) for x in s["features"]))
    for v in g["vectors"]:
        mask = sum(1 << i for i, p in enumerate(v["slot_present"]) if p)
        L.append(f"V {mask} {int(v['anomaly'])} {f9(v['score'])}")
        L.append(" ".join(str(x) for x in v["int8_input"]))
        L.append(" ".join(str(x) for x in v["int8_output"]))
        L.append(" ".join(f9(x) for x in v["slot_errors"]))
    ev, mapping = make_scenario()
    for sid, valid, ts, vals in ev:
        L.append(f"E {sid} {int(valid)} {ts} {len(vals)} " + " ".join(f9(x) for x in vals))
    exp = reference_scenario(ev, mapping, c["maxAgeMs"], c["samplePeriodMs"])
    for ts, mask, feats in exp:
        L.append(f"X {ts} {mask}")
        L.append(" ".join(f9(x) for x in feats))
    (OUT / "cases.txt").write_text("\n".join(L) + "\n")
    (OUT / "anomaly.amdl").write_bytes((ART / "anomaly.amdl").read_bytes())
    return len(exp)


def main():
    if not (ART / "anomaly.amdl").exists():
        sys.exit("missing ml/artifacts/anomaly: run ml/train_anomaly.py first")
    subprocess.run([sys.executable, str(ROOT / "tools" / "generate_proto.py")], stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL, check=True)
    n = write_cases()
    print(f"python reference: {n} expected inferences in the scenario")
    exe = OUT / "test_anomaly.exe"
    srcs = [HERE / "test_anomaly.c"] + [ROOT / "components/ml_core" / x for x in ("sha256.c", "ml_features.c", "ml_package.c", "ml_selector.c")] + [
        ROOT / "components/aai16_protocol/generated/aai16.pb.c",
        ROOT / "components/aai16_protocol/nanopb/pb_common.c",
        ROOT / "components/aai16_protocol/nanopb/pb_decode.c",
        ROOT / "components/aai16_protocol/nanopb/pb_encode.c"]
    cmd = [sys.executable, "-m", "ziglang", "cc", "-std=c11", "-Wall", "-Wextra", "-g",
           f"-I{ROOT / 'components/ml_core/include'}", f"-I{ROOT / 'components/aai16_protocol/generated'}",
           f"-I{ROOT / 'components/aai16_protocol/nanopb'}", "-o", str(exe), *map(str, srcs), "-lm"]
    subprocess.run(cmd, check=True)
    r = subprocess.run([str(exe), str(OUT)])
    sys.exit(r.returncode)


if __name__ == "__main__":
    main()
