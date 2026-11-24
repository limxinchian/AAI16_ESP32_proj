"""Shared code for the two ML tasks: Motion Model A and Environment Model B.

!! DATA PROVENANCE: the training data here is SYNTHETIC (generated below), used to bootstrap and
!! integrate the pipeline. Models trained on it have NOT been validated on real GY-511 / DHT22 /
!! BME280 recordings and must not be presented as field-accurate. Replace `generate_*` with real
!! labelled captures (same window shapes) and retrain before any accuracy claim.

The preprocessing functions below are the REFERENCE for the firmware (components/ml_features); the
golden vectors exported by the training scripts pin the two implementations together.
"""
import hashlib
import json
import pathlib
import struct
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from profile_lib import load_pb  # noqa: E402

pb = load_pb()

MAGIC = b"AMDL"

# ----------------------------------------------------------------------------- motion
MOTION_FS_HZ = 50
MOTION_WINDOW = 64
MOTION_LABELS = ["STATIONARY", "MOVING", "VIBRATING"]
MOTION_FEATURES = 11
ZCR_DEADBAND_G = 0.002


def motion_features(win):
    """PREPROC_MOTION_STATS_V1. win: (64, 3) acceleration in g (columns x, y, z).

    Per axis: std/0.5, mean|first difference|/0.25, zero-crossing rate of the mean-removed signal
    (dead-band 0.002 g); then total std/0.5 and a roughness ratio. Each clipped to [0, 4].
    Output order: [std_x, mad_x, zcr_x, std_y, mad_y, zcr_y, std_z, mad_z, zcr_z, total_std, roughness]
    """
    win = np.asarray(win, dtype=np.float64)
    n = win.shape[0]
    assert win.shape == (MOTION_WINDOW, 3)
    c = win - win.mean(axis=0)
    std = np.sqrt((c ** 2).mean(axis=0))
    mad = np.abs(np.diff(win, axis=0)).mean(axis=0)
    s = np.where(c > ZCR_DEADBAND_G, 1, np.where(c < -ZCR_DEADBAND_G, -1, 0))
    zcr = ((s[:-1] * s[1:]) == -1).sum(axis=0) / (n - 1)
    f = []
    for a in range(3):
        f += [std[a] / 0.5, mad[a] / 0.25, zcr[a]]
    f.append(np.sqrt((std ** 2).sum()) / 0.5)
    f.append(mad.sum() / (std.sum() + 1e-3) / 4.0)
    return np.clip(np.array(f, dtype=np.float64), 0.0, 4.0).astype(np.float32)


def generate_motion(cls, rng, n=MOTION_WINDOW):
    """One synthetic n x 3 window (g), n = 64 by default; larger n gives one CONTINUOUS stream (used to simulate a
    recording in tests). cls: 0 STATIONARY, 1 MOVING, 2 VIBRATING."""
    t = np.arange(n) / MOTION_FS_HZ
    d = rng.normal(size=3)
    d /= np.linalg.norm(d)
    gvec = d * rng.uniform(0.97, 1.03)
    sig = np.tile(gvec, (n, 1))
    if cls == 0:
        sigma = rng.uniform(0.002, 0.012)
        sig += rng.normal(0, sigma, sig.shape)
        sig += np.outer(np.linspace(0, 1, n), rng.normal(0, 0.003, 3))  # slow thermal drift
    else:
        lo, hi, flo, fhi = (0.10, 0.9, 0.4, 3.0) if cls == 1 else (0.06, 0.6, 7.0, 22.0)
        amp = rng.uniform(lo, hi) * rng.dirichlet(np.ones(3) * 2) * 1.7
        freq = rng.uniform(flo, fhi)
        phase = rng.uniform(0, 2 * np.pi, 3)
        wave = np.sin(2 * np.pi * freq * t[:, None] + phase[None, :])
        wave += 0.3 * np.sin(2 * np.pi * 2 * freq * t[:, None] + rng.uniform(0, 2 * np.pi, 3))
        sig += amp[None, :] * wave
        sig += rng.normal(0, rng.uniform(0.004, 0.02), sig.shape)
    return sig.astype(np.float32)


# ------------------------------------------------------------------------ environment
ENV_PERIOD_MS = 2000
ENV_WINDOW = 20
ENV_LABELS = ["STABLE", "CHANGING"]
ENV_FEATURES = 6
# labels are defined OUTSIDE a gap: stable = total drift < 0.15 C and < 0.6 %RH over the window,
# changing = at least 0.8 C or 4 %RH. Windows inside the gap are not sampled (ambiguous by design).


def env_features(win):
    """PREPROC_ENV_TREND_V1. win: (20, 2) = temperature C, humidity %RH.

    Per channel: least-squares trend over the window (slope*(n-1)), residual std, range - scaled by
    (2.0 C | 8 %), (0.3 | 1.5), (2.0 | 8). Trend clipped to [-4, 4], others to [0, 4].
    Output order: [trend_T, resid_std_T, range_T, trend_H, resid_std_H, range_H]
    """
    win = np.asarray(win, dtype=np.float64)
    n = win.shape[0]
    assert win.shape == (ENV_WINDOW, 2)
    tt = np.arange(n) - (n - 1) / 2.0
    scales = [(2.0, 0.3, 2.0), (8.0, 1.5, 8.0)]
    f = []
    for ch in range(2):
        y = win[:, ch]
        slope = (tt * y).sum() / (tt * tt).sum()
        resid = y - y.mean() - slope * tt
        f += [np.clip(slope * (n - 1) / scales[ch][0], -4, 4),
              np.clip(np.sqrt((resid ** 2).mean()) / scales[ch][1], 0, 4),
              np.clip((y.max() - y.min()) / scales[ch][2], 0, 4)]
    return np.array(f, dtype=np.float32)


def _quantize_sensor(win, sensor):
    t, h = win[:, 0], win[:, 1]
    if sensor == "dht22":
        return np.stack([np.round(t * 10) / 10, np.round(h * 10) / 10], axis=1)
    return np.stack([np.round(t * 100) / 100, np.round(h / 0.008) * 0.008], axis=1)


def generate_env(cls, rng):
    """One synthetic 20x2 window (temperature C, humidity %RH) from a random DHT22- or BME280-like sensor."""
    sensor = "dht22" if rng.random() < 0.5 else "bme280"
    sT, sH = (0.15, 0.6) if sensor == "dht22" else (0.05, 0.25)
    x = np.linspace(0, 1, ENV_WINDOW)
    t0, h0 = rng.uniform(10, 35), rng.uniform(25, 85)

    def shape():
        kind = rng.integers(3)
        if kind == 0:
            return x
        if kind == 1:
            return (x > rng.uniform(0.3, 0.7)).astype(float)
        return np.sin(0.5 * np.pi * x)

    if cls == 0:
        dT, dH = rng.uniform(-0.15, 0.15), rng.uniform(-0.6, 0.6)
        T = t0 + dT * x
        H = h0 + dH * x
    else:
        mode = rng.integers(3)  # which channel(s) change
        big = lambda lo, hi: rng.choice([-1, 1]) * rng.uniform(lo, hi)
        dT = big(0.8, 4.0) if mode in (0, 2) else rng.uniform(-0.15, 0.15)
        dH = big(4.0, 15.0) if mode in (1, 2) else rng.uniform(-0.6, 0.6)
        T = t0 + dT * shape()
        H = h0 + dH * shape()
    win = np.stack([T + rng.normal(0, sT, ENV_WINDOW), H + rng.normal(0, sH, ENV_WINDOW)], axis=1)
    return _quantize_sensor(win, sensor).astype(np.float32)


# ----------------------------------------------------------------------------- export
def sha256(b):
    return hashlib.sha256(b).digest()


def export_int8(model, rep_features, tf):
    conv = tf.lite.TFLiteConverter.from_keras_model(model)
    conv.optimizations = [tf.lite.Optimize.DEFAULT]
    conv.representative_dataset = lambda: ([f[None, :].astype(np.float32)] for f in rep_features[:500])
    conv.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    conv.inference_input_type = tf.int8
    conv.inference_output_type = tf.int8
    return conv.convert()


def tflite_runner(tflite_bytes, tf):
    it = tf.lite.Interpreter(
        model_content=tflite_bytes,  # no XNNPACK: keeps the op list equal to what TFLM must implement
        experimental_op_resolver_type=tf.lite.experimental.OpResolverType.BUILTIN_WITHOUT_DEFAULT_DELEGATES)
    it.allocate_tensors()
    inp, out = it.get_input_details()[0], it.get_output_details()[0]
    (s_in, z_in), (s_out, z_out) = inp["quantization"], out["quantization"]

    def run(features):
        q = np.clip(np.round(features / s_in) + z_in, -128, 127).astype(np.int8)
        it.set_tensor(inp["index"], q[None, :])
        it.invoke()
        o = it.get_tensor(out["index"])[0]
        return q, o, (o.astype(np.float32) - z_out) * s_out

    return run, (float(s_in), int(z_in), float(s_out), int(z_out)), it


def op_names(it):
    return sorted({d["op_name"] for d in it._get_ops_details()})


def build_package(contract, tflite_bytes):
    """"AMDL" | u32 LE contract_len | ModelContract | .tflite  (contract.tflite_sha256 is filled in)."""
    contract.tflite_sha256 = sha256(tflite_bytes)
    c = contract.SerializeToString()
    return MAGIC + struct.pack("<I", len(c)) + c + tflite_bytes


def parse_package(data):
    assert data[:4] == MAGIC
    (n,) = struct.unpack("<I", data[4:8])
    c = pb.ModelContract()
    c.ParseFromString(data[8:8 + n])
    tfl = data[8 + n:]
    assert sha256(tfl) == c.tflite_sha256, "package digest mismatch"
    return c, tfl


def write_json(path, obj):
    path.write_text(json.dumps(obj, indent=2, sort_keys=True) + "\n")
