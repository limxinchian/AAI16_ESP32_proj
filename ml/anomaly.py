"""Model C - cargo anomaly detector: reference feature maths, SYNTHETIC data and training helpers.

Spec: ml/ANOMALY_SPEC.md. The functions here are the REFERENCE for components/ml_core/ml_features.c (slot statistics)
and components/node_core (scoring); golden vectors written by train_anomaly.py pin them together.

!! All data is synthetic (generated below). The model proves the mechanism, not field accuracy.
"""
import math

import numpy as np

SLOTS = ["motion", "temperature", "humidity", "pressure", "co2", "distance", "radar", "equipment"]
N_SLOTS = 8
N_FEAT_SLOT = 4
N_FEATURES = N_SLOTS * N_FEAT_SLOT
CHANNEL_KINDS = ["VALUE_ACCEL_X_G", "VALUE_ACCEL_Y_G", "VALUE_ACCEL_Z_G", "VALUE_TEMPERATURE_C", "VALUE_HUMIDITY_PCT",
                 "VALUE_PRESSURE_PA", "VALUE_CO2_PPM", "VALUE_DISTANCE_CM", "VALUE_RADAR_DISTANCE_MM", "VALUE_TEMPERATURE_C"]
SLOT_OF_CHANNEL = [0, 0, 0, 1, 2, 3, 4, 5, 6, 7]
TAU_FAST, TAU_MID, TAU_SLOW, TAU_JIT = 3.0, 15.0, 90.0, 8.0
MIN_READINGS = 5
# center, s0 (level), s1 (fast - slow), s2 (jitter), s3 (fast - mid)
CONST = [
    (1.0, 0.5, 0.5, 0.2, 0.5),
    (25.0, 15.0, 2.0, 0.5, 1.0),
    (60.0, 40.0, 10.0, 2.0, 4.0),
    (101300.0, 3000.0, 100.0, 30.0, 60.0),
    (700.0, 1000.0, 300.0, 60.0, 150.0),
    (100.0, 100.0, 20.0, 5.0, 20.0),
    (1500.0, 1500.0, 400.0, 150.0, 400.0),
    (40.0, 40.0, 3.0, 0.5, 1.5),
]


def _clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class SlotTracker:
    """Scalar reference (mirrors the C code step for step, double precision)."""

    def __init__(self, slot):
        self.slot = slot
        self.n = 0
        self.fast = self.mid = self.slow = self.jit = 0.0
        self.t_prev = 0.0

    def update(self, x, t):
        if self.n == 0:
            self.fast = self.mid = self.slow = float(x)
            self.jit = 0.0
        else:
            dt = t - self.t_prev
            if dt <= 0.0:
                return
            self.fast += (1 - math.exp(-dt / TAU_FAST)) * (x - self.fast)
            self.mid += (1 - math.exp(-dt / TAU_MID)) * (x - self.mid)
            self.slow += (1 - math.exp(-dt / TAU_SLOW)) * (x - self.slow)
            self.jit += (1 - math.exp(-dt / TAU_JIT)) * (abs(x - self.fast) - self.jit)
        self.n += 1
        self.t_prev = t

    def features(self):
        c, s0, s1, s2, s3 = CONST[self.slot]
        return [_clip((self.fast - c) / s0, -4, 4), _clip((self.fast - self.slow) / s1, -4, 4),
                _clip(self.jit / s2, 0, 4), _clip((self.fast - self.mid) / s3, -4, 4)]


def slot_features_batch(slot, V, dt):
    """Vectorised version of SlotTracker over a batch. V: (B, T) values, dt: (B,) constant sample spacing."""
    V = np.asarray(V, dtype=np.float64)
    dt = np.asarray(dt, dtype=np.float64)
    B, T = V.shape
    af, am, asl, aj = (1 - np.exp(-dt / tau) for tau in (TAU_FAST, TAU_MID, TAU_SLOW, TAU_JIT))
    fast = V[:, 0].copy()
    mid, slow = fast.copy(), fast.copy()
    jit = np.zeros(B)
    for k in range(1, T):
        x = V[:, k]
        fast += af * (x - fast)
        mid += am * (x - mid)
        slow += asl * (x - slow)
        jit += aj * (np.abs(x - fast) - jit)
    c, s0, s1, s2, s3 = CONST[slot]
    return np.stack([np.clip((fast - c) / s0, -4, 4), np.clip((fast - slow) / s1, -4, 4),
                     np.clip(jit / s2, 0, 4), np.clip((fast - mid) / s3, -4, 4)], axis=1).astype(np.float32)


# ------------------------------------------------------------------ synthetic signals
T_STEPS = 300
EVENT_S = 15.0  # an anomaly acts during the last 15 s of the series


def _times(dt):
    return np.arange(T_STEPS)[None, :] * dt[:, None]


def _quant(v, step):
    return np.round(v / step) * step


def gen_normal(slot, B, rng):
    """(values (B, T), dt (B,)) of NORMAL operation for one slot."""
    u = lambda a, b: rng.uniform(a, b, B)
    if slot == 0:  # |acceleration| of a (nearly) still or gently carried sensor
        dt = rng.choice([0.02, 0.05, 0.1], B)
        t = _times(dt)
        sway = (rng.random(B) < 0.3) * u(0.005, 0.06)
        v = u(0.97, 1.03)[:, None] + sway[:, None] * np.sin(2 * np.pi * u(0.2, 2.0)[:, None] * t + u(0, 6.28)[:, None])
        v += rng.normal(0, 1, (B, T_STEPS)) * u(0.003, 0.03)[:, None]
    elif slot == 1:
        dt = rng.choice([0.5, 1.0, 2.0], B)
        t = _times(dt)
        v = u(15, 35)[:, None] + u(-0.003, 0.003)[:, None] * t + rng.normal(0, 1, (B, T_STEPS)) * u(0.03, 0.25)[:, None]
        v = np.where((rng.random(B) < 0.5)[:, None], _quant(v, 0.1), _quant(v, 0.01))
    elif slot == 2:
        dt = rng.choice([0.5, 1.0, 2.0], B)
        t = _times(dt)
        v = u(25, 85)[:, None] + u(-0.01, 0.01)[:, None] * t + rng.normal(0, 1, (B, T_STEPS)) * u(0.1, 0.9)[:, None]
        v = np.where((rng.random(B) < 0.5)[:, None], _quant(v, 0.1), _quant(v, 0.008))
    elif slot == 3:
        dt = rng.choice([0.5, 1.0, 2.0], B)
        t = _times(dt)
        v = u(100500, 102000)[:, None] + u(-0.3, 0.3)[:, None] * t + rng.normal(0, 1, (B, T_STEPS)) * u(3, 25)[:, None]
    elif slot == 4:
        dt = rng.choice([1.0, 2.0], B)
        t = _times(dt)
        v = _quant(u(400, 1200)[:, None] + u(-1.0, 1.0)[:, None] * t + rng.normal(0, 1, (B, T_STEPS)) * u(3, 30)[:, None], 1.0)
    elif slot == 5:
        dt = rng.choice([0.1, 0.5], B)
        v = u(15, 300)[:, None] + rng.normal(0, 1, (B, T_STEPS)) * u(0.2, 1.5)[:, None]
    elif slot == 6:  # radar: mostly an empty bay (0), sometimes static clutter far away
        dt = np.full(B, 0.1)
        clutter = (rng.random(B) < 0.15)
        v = np.where(clutter[:, None], u(2500, 3500)[:, None] + rng.normal(0, 20, (B, T_STEPS)), 0.0)
    else:
        dt = rng.choice([0.25, 0.5], B)
        t = _times(dt)
        v = _quant(u(25, 60)[:, None] + u(-0.01, 0.01)[:, None] * t + rng.normal(0, 1, (B, T_STEPS)) * u(0.1, 0.4)[:, None], 0.25)
    return v.astype(np.float64), dt


def gen_anomaly(slot, B, rng):
    """Like gen_normal, but an event acts during the last 15 s (what the node would see right after it)."""
    v, dt = gen_normal(slot, B, rng)
    t = _times(dt)
    t_end = (T_STEPS - 1) * dt
    ts = t_end - EVENT_S                              # event start (s)
    ramp = np.clip((t - ts[:, None]) / EVENT_S, 0, 1)  # 0 before the event, 1 at its end
    active = (t >= ts[:, None]).astype(np.float64)
    sign = rng.choice([-1, 1], B)[:, None]
    u = lambda a, b: rng.uniform(a, b, B)[:, None]
    if slot == 0:    # shake / shock
        amp, f = u(0.1, 0.9), u(3.0, 12.0)
        v = v + active * (amp * np.sin(2 * np.pi * f * t) + rng.normal(0, 1, v.shape) * u(0.03, 0.2))
    elif slot == 1:
        v = v + ramp * sign * u(1.5, 6.0)
    elif slot == 2:
        v = v + ramp * sign * u(8, 30)
    elif slot == 3:
        v = v + ramp * sign * u(150, 600)
    elif slot == 4:
        v = v + ramp * u(300, 3000)
    elif slot == 5:  # door opened / load shifted: step in distance
        v = np.maximum(v + active * sign * u(40, 200), 2.0)
    elif slot == 6:  # a target appears and moves
        pos = u(300, 3000) + active * ramp * sign * u(0, 800)
        v = np.where(active > 0, pos + rng.normal(0, 1, v.shape) * u(5, 60), v)
        v = np.maximum(v, 0)
    else:
        v = v + ramp * u(8, 30)
    return v, dt


# ------------------------------------------------------------------ datasets
def slot_pool(kind, n, rng):
    """Per slot feature pools: list of (n, 4) arrays (kind = 'normal' | 'anomaly')."""
    pool = []
    for s in range(N_SLOTS):
        v, dt = (gen_normal if kind == "normal" else gen_anomaly)(s, n, rng)
        pool.append(slot_features_batch(s, v, dt))
    return pool


def assemble(pool, n, rng, p_present=None, force=None):
    """n examples of 32 features + a 32-element mask. Slots are drawn independently from the pool; each slot is
    present with probability p (random per example) except `force` (a slot index that is always present)."""
    X = np.zeros((n, N_FEATURES), dtype=np.float32)
    M = np.zeros((n, N_FEATURES), dtype=np.float32)
    p = rng.uniform(0.3, 0.95, n) if p_present is None else np.full(n, p_present)
    pres = rng.random((n, N_SLOTS)) < p[:, None]
    none = ~pres.any(axis=1)
    pres[none, rng.integers(0, N_SLOTS, none.sum())] = True  # at least one slot
    if force is not None:
        pres[:, force] = True
    for s in range(N_SLOTS):
        idx = rng.integers(0, len(pool[s]), n)
        feats = pool[s][idx]
        X[:, s * 4:(s + 1) * 4] = np.where(pres[:, s:s + 1], feats, 0.0)
        M[:, s * 4:(s + 1) * 4] = pres[:, s:s + 1]
    return X, M


def score_from_int8(q_in, q_out, mask, s_in, z_in, s_out, z_out):
    """Exactly what the node computes (components/node_core): only int8 tensors and the contract go in."""
    xq = (q_in.astype(np.float64) - z_in) * s_in
    yq = (q_out.astype(np.float64) - z_out) * s_out
    err = (xq - yq) ** 2
    m = mask.astype(np.float64)
    present = max(m.sum() / N_FEAT_SLOT, 1)
    score = float((err * m).sum() / (N_FEAT_SLOT * present))
    slot_err = [float((err[s * 4:(s + 1) * 4] * m[s * 4:(s + 1) * 4]).sum() / 4.0) if m[s * 4] else 0.0 for s in range(N_SLOTS)]
    return score, slot_err
