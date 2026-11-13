"""Model C on REAL recordings: replay raw readings (tools/record_slot_data.py CSV) through the node's slot rules.

The replay mirrors what the node does: a slot is present when it has >= MIN_READINGS readings and its newest one is
not older than MAX_AGE; an invalid reading resets the slot; the model is evaluated every TICK seconds.
Rows are JOINT (all slots at the same moment), so correlations between sensors stay realistic.
"""
import csv

import numpy as np

import anomaly as A

# slot -> (node, sensor_id, value_index) for the two recorded setups on node1/node2
# The radar (slot 6, node2:3 value 1) is NOT used: during the first recording somebody was always moving in its view
# (it never read "empty"), so it stays synthetic ("empty bay = normal").
SETUPS = {
    "bme280": {1: ("node1", 5, 0), 2: ("node1", 5, 2), 3: ("node1", 5, 1), 7: ("node1", 8, 0)},
    "dht22": {1: ("node1", 2, 1), 2: ("node1", 2, 0), 7: ("node1", 8, 0)},
}
MAX_AGE = 6.0
TICK = 0.5
WARMUP_S = 20.0


def load(path):
    """-> {(node, sensor): [(t_s, valid, (v0, v1, v2))]} with t measured from each node's first row (see note)."""
    raw = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            raw.setdefault(r["node"], []).append((int(r["sensor_id"]), int(r["timestamp_us"]), int(r["valid"]),
                                                  (float(r["v0"]), float(r["v1"]), float(r["v2"]))))
    out = {}
    for node, rows in raw.items():
        t0 = min(t for _, t, _, _ in rows)  # all streams of a node start within one sensor period of each other
        for sid, t, valid, v in rows:
            out.setdefault((node, sid), []).append(((t - t0) / 1e6, valid, v))
    for k in out:
        out[k].sort(key=lambda r: r[0])
    return out


def replay(streams, mapping, tick=TICK, warmup=WARMUP_S):
    """-> (times (N,), X (N,32), M (N,32)) for one setup (mapping slot -> (node, sensor, value_index))."""
    need = {k: streams.get((v[0], v[1]), []) for k, v in mapping.items()}
    if any(len(r) == 0 for r in need.values()):
        return np.zeros(0), np.zeros((0, 32), np.float32), np.zeros((0, 32), np.float32)
    t_end = min(r[-1][0] for r in need.values())
    trackers = {s: A.SlotTracker(s) for s in mapping}
    last = {s: -1e9 for s in mapping}
    ptr = {s: 0 for s in mapping}
    T, X, M = [], [], []
    t = 0.0
    while t <= t_end:
        for s, (node, sid, vi) in mapping.items():
            rows = need[s]
            while ptr[s] < len(rows) and rows[ptr[s]][0] <= t:
                rt, valid, v = rows[ptr[s]]
                ptr[s] += 1
                if not valid:
                    trackers[s] = A.SlotTracker(s)
                    last[s] = -1e9
                    continue
                trackers[s].update(v[vi], rt)
                last[s] = rt
        if t >= warmup:
            x = np.zeros(32, np.float32)
            m = np.zeros(32, np.float32)
            for s in mapping:
                if trackers[s].n >= A.MIN_READINGS and t - last[s] <= MAX_AGE:
                    x[s * 4:(s + 1) * 4] = trackers[s].features()
                    m[s * 4:(s + 1) * 4] = 1.0
            if m.any():
                T.append(t)
                X.append(x)
                M.append(m)
        t += tick
    return np.array(T), np.array(X, np.float32), np.array(M, np.float32)


def replay_all(path):
    streams = load(path)
    parts = [replay(streams, mp) for mp in SETUPS.values()]
    T = [p[0] for p in parts]
    X = np.concatenate([p[1] for p in parts]) if parts else np.zeros((0, 32), np.float32)
    M = np.concatenate([p[2] for p in parts]) if parts else np.zeros((0, 32), np.float32)
    return T, X, M, [len(p[0]) for p in parts]


def split_time(n, frac=(0.7, 0.15, 0.15)):
    a = int(n * frac[0])
    b = int(n * (frac[0] + frac[1]))
    return slice(0, a), slice(a, b), slice(b, n)


# ---------------------------------------------------------------------------------------------- augmentation
LEVEL_SLOTS = (1, 2, 3, 7)  # level (feature 0) of these slots varies from day to day (room temperature, ...)


def augment(Xr, Mr, n, pool, rng, p_drop=0.25, p_fill=0.3):
    """n training rows built from real joint rows: random level shifts / noise scaling, random missing slots, and
    synthetic values for slots that were not recorded (motion, co2, distance)."""
    idx = rng.integers(0, len(Xr), n)
    X, M = Xr[idx].copy(), Mr[idx].copy()
    for s in range(A.N_SLOTS):
        sl = slice(s * 4, (s + 1) * 4)
        pres = M[:, s * 4] > 0
        if s in LEVEL_SLOTS:
            X[:, s * 4] += rng.uniform(-0.5, 0.5, n) * pres
        X[:, s * 4 + 2] *= rng.uniform(0.7, 1.6, n)
        X[:, s * 4 + 1] *= rng.uniform(0.8, 1.4, n)
        X[:, s * 4 + 3] *= rng.uniform(0.8, 1.4, n)
        drop = pres & (rng.random(n) < p_drop)
        fill = (~pres) & (rng.random(n) < p_fill) & (s in (0, 4, 5, 6))
        if fill.any():
            fi = rng.integers(0, len(pool[s]), n)
            X[fill, sl] = pool[s][fi][fill]
            M[fill, sl] = 1.0
        X[drop, sl] = 0.0
        M[drop, sl] = 0.0
    X[:, 0::4] = np.clip(X[:, 0::4], -4, 4)
    X[:, 1::4] = np.clip(X[:, 1::4], -4, 4)
    X[:, 2::4] = np.clip(X[:, 2::4], 0, 4)
    X[:, 3::4] = np.clip(X[:, 3::4], -4, 4)
    none = ~(M.reshape(n, 8, 4)[:, :, 0] > 0).any(axis=1)  # never an empty row: restore one originally present slot
    for i in np.where(none)[0]:
        j = idx[i]
        pr = np.where(Mr[j, ::4] > 0)[0]
        s = int(pr[0]) if len(pr) else 1
        X[i, s * 4:(s + 1) * 4] = Xr[j, s * 4:(s + 1) * 4]
        M[i, s * 4:(s + 1) * 4] = 1.0
    return X.astype(np.float32), M.astype(np.float32)


def mixture(Xr, Mr, pool, n, rng, real_frac=0.6):
    nr = int(n * real_frac)
    Xa, Ma = augment(Xr, Mr, nr, pool, rng)
    Xs, Ms = A.assemble(pool, n - nr, rng)
    X, M = np.concatenate([Xa, Xs]), np.concatenate([Ma, Ms])
    p = rng.permutation(len(X))
    return X[p], M[p]
