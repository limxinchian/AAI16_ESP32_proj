"""Node-level integration test (host, simulated devices).

Python prepares: ConfigApply fixtures, simulated sensor data, and the recorded int8 input->output pairs
of the REAL trained TFLite models for exactly the data the simulated sensors will deliver. The C test
drives a full node (config -> readings -> model upload/activation -> input selection -> inference ->
rules/bridge/REPL/heartbeat -> reboot). The only test double is the inference matrix multiply: it
answers ONLY when handed the exact recorded int8 tensor, so everything up to the model is verified.

Run:  .venv\\Scripts\\python.exe tests\\node_host\\run.py   (needs ml/artifacts from the training scripts)
"""
import importlib.util
import json
import os
import pathlib
import struct
import subprocess
import sys

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "3")
os.environ.setdefault("TF_ENABLE_ONEDNN_OPTS", "0")

ROOT = pathlib.Path(__file__).resolve().parents[2]
HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "_gen"
ART = ROOT / "ml" / "artifacts"
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "ml"))
import numpy as np  # noqa: E402
import sensor_profiles as sp  # noqa: E402
import common as C  # noqa: E402  (ml/common.py: reference features + tflite runner)

spec = importlib.util.spec_from_file_location("decode_run", ROOT / "tests" / "decode_host" / "run.py")
dh = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dh)
pb = C.pb


def f32(x):
    return np.float32(x)


def main():
    import tensorflow as tf
    OUT.mkdir(exist_ok=True)

    (OUT / "cfg1.bin").write_bytes(_cfg(pb, 1, [sp.gy511(pb), sp.dht22(pb)]))
    (OUT / "cfg2.bin").write_bytes(_cfg(pb, 2, [sp.hcsr04(pb), sp.bme280(pb)]))
    for task in ("motion", "environment"):
        (OUT / f"{task}.amdl").write_bytes((ART / task / f"{task}.amdl").read_bytes())

    runners = {}
    for task in ("motion", "environment"):
        runners[task] = C.tflite_runner((ART / task / "model_int8.tflite").read_bytes(), tf)[0]

    pairs = []

    def record(task, features):
        q_in, q_out, probs = runners[task](features)
        pairs.append((0 if task == "motion" else 1, q_in, q_out, int(probs.argmax())))
        return int(probs.argmax())

    # ---- motion: a MOVING golden window, rounded to what the GY-511 can deliver (1 mg, 12-bit)
    gm = json.loads((ART / "motion" / "golden.json").read_text())["vectors"]
    win = next(np.array(v["window"]) for v in gm if v["label"] == 1)
    mg = np.clip(np.round(win * 1000), -2048, 2047).astype(int)               # (64, 3)
    sc = float(np.float32(0.001))
    rows = np.float32(mg * sc)                                                 # C: (double)raw*(double)scale -> float
    regs = bytearray()
    for r in mg:
        for v in r:
            regs += struct.pack("<H", ((int(v) & 0xFFF) << 4) & 0xFFFF)
    (OUT / "motion_rows.bin").write_bytes(bytes(regs))
    exp_a = record("motion", C.motion_features(rows))
    exp_b = record("motion", C.motion_features(np.concatenate([rows[32:], rows[:32]])))  # next stride (32)

    # ---- environment from a DHT22: a CHANGING golden window rounded to 0.1
    ge = json.loads((ART / "environment" / "golden.json").read_text())["vectors"]
    ew = next(np.array(v["window"]) for v in ge if v["label"] == 1)             # (20, 2) = T, H
    t10, h10 = np.round(ew[:, 0] * 10).astype(int), np.round(ew[:, 1] * 10).astype(int)
    frames = bytearray()
    for t, h in zip(t10, h10):
        tr = (0x8000 | -t) if t < 0 else t
        b = [h >> 8, h & 255, tr >> 8, tr & 255]
        frames += bytes(b + [sum(b) & 255])
    (OUT / "dht_frames.bin").write_bytes(bytes(frames))
    sc1 = float(np.float32(0.1))
    erows = np.stack([f32(t10 * sc1), f32(h10 * sc1)], axis=1)                  # columns: temperature, humidity
    exp_c = record("environment", C.env_features(erows))

    # ---- environment from a BME280: constant (stable) readings from the Bosch example registers
    c, adc_t, adc_p, adc_h = dh.BOSCH, 519888, 415148, 30000
    init = dh.bme_init_bytes(c)
    regs = bytearray(256)
    regs[0x88:0x88 + 24] = init[:24]
    regs[0xA1] = init[24]
    regs[0xE1:0xE1 + 7] = init[25:32]
    regs[0xF7:0xF7 + 8] = dh.bme_cycle_bytes(adc_t, adc_p, adc_h)
    (OUT / "bme_regs.bin").write_bytes(bytes(regs))
    T, _, H = dh.bme_ref(c, adc_t, adc_p, adc_h)
    brows = np.tile(np.array([f32(T), f32(H)], dtype=np.float32), (20, 1))
    feats = C.env_features(brows)
    assert np.allclose(feats, 0, atol=1e-6), feats
    exp_d = record("environment", feats)

    with open(OUT / "pairs.txt", "w") as f:
        for task, qi, qo, pred in pairs:
            f.write(f"P {task} {len(qi)} {len(qo)} {pred}\n{' '.join(map(str, qi.tolist()))}\n{' '.join(map(str, qo.tolist()))}\n")
    print(f"recorded TFLite pairs: motion A->{exp_a} B->{exp_b} (0=STATIONARY 1=MOVING 2=VIBRATING), "
          f"env DHT22 window->{exp_c}, env BME280 stable->{exp_d} (0=STABLE 1=CHANGING)")
    (OUT / "expect.txt").write_text(f"{exp_a} {exp_b} {exp_c} {exp_d}\n")

    # chunks produced by the LAPTOP CLIENT (tools/node_client.py), fed to the firmware logic by the C test
    import node_client as nc
    for task, kind, size in (("motion", pb.MODEL_MOTION, 256), ("environment", pb.MODEL_ENVIRONMENT, 1024)):
        pkg = (ART / task / f"{task}.amdl").read_bytes()
        blob = b"".join(len(c).to_bytes(4, "little") + c for c in nc.make_chunks(pkg, kind, 9, size, nc.package_version(pkg)))
        (OUT / f"client_chunks_{task}.bin").write_bytes(blob)

    exe = OUT / "test_node.exe"
    comps = ROOT / "components"
    srcs = [HERE / "test_node.c", comps / "node_core/node_core.c", comps / "sensor_runtime/sensor_runtime.c",
            comps / "sensor_decode/sensor_decode.c", comps / "config_manager/config_manager.c",
            *[comps / "node_services" / n for n in ("rule_engine.c", "bridge.c", "diag.c", "repl.c")],
            *[comps / "ml_core" / n for n in ("sha256.c", "ml_features.c", "ml_package.c", "ml_selector.c", "model_store.c")],
            comps / "aai16_protocol/generated/aai16.pb.c", *[comps / "aai16_protocol/nanopb" / n for n in ("pb_common.c", "pb_decode.c", "pb_encode.c")]]
    incs = [comps / n / "include" for n in ("node_core", "node_services", "sensor_runtime", "sensor_decode", "ml_core", "config_manager")]
    cmd = [sys.executable, "-m", "ziglang", "cc", "-std=c11", "-Wall", "-Wextra", "-g", *[f"-I{i}" for i in incs],
           f"-I{comps / 'aai16_protocol/generated'}", f"-I{comps / 'aai16_protocol/nanopb'}", "-o", str(exe), *map(str, srcs)]
    subprocess.run(cmd, check=True)
    sys.exit(subprocess.run([str(exe), str(OUT)]).returncode)


def _cfg(pb, gen, profiles):
    m = pb.ConfigApply(generation=gen)
    m.profiles.extend(profiles)
    return m.SerializeToString()


if __name__ == "__main__":
    main()
