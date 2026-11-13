"""Load REAL recordings (tools/collect_readings.py CSV files) as labelled windows for ml/pipeline.py.

A window is accepted only if its samples are consecutive in time (spacing within the model's tolerance of the model
period), share one label, and come from one file. Splitting is by whole recording session (file) so that overlapping
windows of one session never appear in both the training and the test set.
"""
import csv
import pathlib

import numpy as np


def _read_file(path):
    cols, rows = {}, []
    with open(path, newline="") as f:
        header = f.readline()
        if not header.startswith("#") or "columns=" not in header:
            raise ValueError(f"{path}: first line must be the '# ... columns=name=index,...' comment")
        spec = header.split("columns=", 1)[1].strip()
        for part in spec.split(","):
            name, idx = part.split("=")
            cols[name.strip()] = int(idx)
        rd = csv.reader(f)
        next(rd)  # column header
        for r in rd:
            if len(r) >= 3:
                rows.append((int(r[0]), r[1].strip().upper(), [float(x) for x in r[2:]]))
    rows.sort(key=lambda x: x[0])
    return cols, rows


def load_task(cfg, data_dir, train_stride):
    """Returns windows (N, window, channels) float32, labels (N,), groups (N,) = file index, stats dict."""
    files = sorted(pathlib.Path(data_dir).rglob("*.csv"))
    if not files:
        raise SystemExit(f"no .csv recordings under {data_dir}")
    window, period_us = cfg["window"], cfg["period_ms"] * 1000
    tol_us = period_us * cfg["tolerance_pct"] / 100.0
    want = cfg["columns"]
    labels = cfg["labels"]
    wins, ys, groups = [], [], []
    stats = dict(files=len(files), rows=0, runs=0, rejected_runs_too_short=0, per_class_windows={l: 0 for l in labels},
                 per_class_files={l: 0 for l in labels}, skipped_files=[])
    for gi, path in enumerate(files):
        cols, rows = _read_file(path)
        if any(w not in cols for w in want):
            stats["skipped_files"].append(f"{path.name}: needs columns {want}, has {list(cols)}")
            continue
        order = [cols[w] for w in want]
        stats["rows"] += len(rows)
        seen_labels = set()
        run = []

        def flush():
            nonlocal run
            if len(run) >= window:
                lab = labels.index(run[0][1])
                arr = np.array([[r[2][i] for i in order] for r in run], dtype=np.float32)
                stats["runs"] += 1
                for s in range(0, len(arr) - window + 1, train_stride):
                    wins.append(arr[s:s + window])
                    ys.append(lab)
                    groups.append(gi)
                    stats["per_class_windows"][run[0][1]] += 1
                    seen_labels.add(run[0][1])
            elif run:
                stats["rejected_runs_too_short"] += 1
            run = []

        for r in rows:
            if r[1] not in labels:
                stats["skipped_files"].append(f"{path.name}: unknown label {r[1]!r}")
                run = []
                break
            if run and (r[1] != run[-1][1] or abs((r[0] - run[-1][0]) - period_us) > tol_us):
                flush()  # a gap, jitter beyond tolerance or a label change ends the run
            run.append(r)
        flush()
        for l in seen_labels:
            stats["per_class_files"][l] += 1
    if not wins:
        raise SystemExit("no usable windows: check sensor periods (motion 20 ms, environment 2000 ms) and columns; "
                         f"details: {stats}")
    return np.array(wins, dtype=np.float32), np.array(ys), np.array(groups), stats


def split_indices(ys, groups, rng, window, train_stride, n_classes):
    """Session-wise split. Falls back to a chronological block split (with a gap) when a class has < 3 sessions."""
    tr, va, te = [], [], []
    per_class_sessions = [sorted(set(groups[ys == c])) for c in range(n_classes)]
    if all(len(s) >= 3 for s in per_class_sessions):
        for c in range(n_classes):
            sess = list(per_class_sessions[c])
            rng.shuffle(sess)
            n_te = max(1, round(0.15 * len(sess)))
            n_va = max(1, round(0.15 * len(sess)))
            te_s, va_s, tr_s = set(sess[:n_te]), set(sess[n_te:n_te + n_va]), set(sess[n_te + n_va:])
            for i in np.where(ys == c)[0]:
                (te if groups[i] in te_s else va if groups[i] in va_s else tr).append(i)
        return np.array(tr), np.array(va), np.array(te), "by_session"
    gap = max(1, window // train_stride)  # windows to skip between blocks: they would overlap the neighbouring block
    for g in sorted(set(groups)):
        idx = np.where(groups == g)[0]
        n = len(idx)
        a, b = int(0.7 * n), int(0.85 * n)
        tr += list(idx[:max(a - gap, 0)])
        va += list(idx[a:max(b - gap, a)])
        te += list(idx[b:])
    return np.array(tr), np.array(va), np.array(te), "chronological_blocks_per_file (optimistic: fewer than 3 sessions per class)"
