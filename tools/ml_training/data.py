"""Raw CSV validation and session-anchored windows; no TensorFlow dependency."""
from __future__ import annotations

from collections import Counter
import csv
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import re

import numpy as np

LABELS = {"REST": 0, "MOVING": 1}
WINDOW = 20
STRIDE = 10
GAP_MS = 150
TRAIN_FILES = ("rest_orientation_01.csv", "moving_directions_01.csv", "moving_directions_02.csv")
EVALUATION_FILES = ("rest_orientation_02.csv", "moving_directions_03.csv")
REQUIRED = {"timestamp_ms", "uptime_ms", "boot_id", "seq", "ax_g", "ay_g", "az_g", "label", "session_id"}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def integer(text: str, maximum: int) -> int:
    if not re.fullmatch(r"0|[1-9][0-9]*", text or ""):
        raise ValueError("invalid integer")
    value = int(text)
    if value > maximum:
        raise ValueError("integer outside range")
    return value


@dataclass(frozen=True)
class Row:
    timestamp_ms: int
    boot_id: str
    seq: int
    axes: tuple[float, float, float]
    label: str
    session_id: str


def parse_row(record: dict) -> Row:
    if None in record or any(record.get(key) is None for key in REQUIRED):
        raise ValueError("malformed/missing CSV fields")
    timestamp = integer(record["timestamp_ms"], (2**63 - 1) // 1000)
    if integer(record["uptime_ms"], (2**63 - 1) // 1000) != timestamp:
        raise ValueError("timestamp_ms differs from uptime_ms")
    seq = integer(record["seq"], 2**32 - 1)
    boot = record["boot_id"]
    if not re.fullmatch(r"[0-9a-fA-F]{8}", boot):
        raise ValueError("invalid boot_id")
    label, session = record["label"], record["session_id"]
    if label not in LABELS or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,79}", session):
        raise ValueError("invalid label/session_id")
    axes = tuple(float(record[key]) for key in ("ax_g", "ay_g", "az_g"))
    if any(not math.isfinite(value) or abs(value) > 2.048 for value in axes):
        raise ValueError("invalid/non-finite/out-of-register-range acceleration")
    if record.get("jitter_us", "") and not re.fullmatch(r"-?[0-9]+", record["jitter_us"]):
        raise ValueError("invalid jitter_us")
    return Row(timestamp, boot.lower(), seq, axes, label, session)


def preprocess(raw: np.ndarray) -> np.ndarray:
    """20x3 g -> 60 float32, mean removal -> clamp +/-2 -> /2, sample-major.

    Accumulate each axis in chronological order using float32, divide by 20,
    subtract in float32, clamp, divide by 2. This specifies a reproducible order
    for the later C implementation instead of relying on a parallel reduction.
    """
    raw = np.asarray(raw, dtype=np.float32)
    if raw.shape != (WINDOW, 3) or not np.isfinite(raw).all():
        raise ValueError("preprocessing requires exactly 20 finite XYZ samples")
    total = np.zeros(3, dtype=np.float32)
    for sample in raw:
        total += sample
    mean = total / np.float32(WINDOW)
    centered = raw - mean
    return (np.clip(centered, -2.0, 2.0) / np.float32(2.0)).reshape(60).astype(np.float32)


def movement_rms(raw: np.ndarray) -> float:
    """Baseline: RMS magnitude of per-axis-centered acceleration, in g.

    sqrt(mean_over_time(dx**2+dy**2+dz**2)); no model or threshold labels here.
    Float64 is used for this laptop-only diagnostic/baseline statistic.
    """
    values = np.asarray(raw, dtype=np.float64)
    centered = values - values.mean(axis=0)
    return float(np.sqrt(np.mean(np.sum(centered * centered, axis=1))))


def load_session(path: Path, expected_label: str | None = None):
    """Preserve original row positions; invalid rows break continuity.

    Candidate starts are original session rows 0,10,20,... . Reject a candidate
    touching an invalid row, sequence discontinuity, reboot or >150 ms gap.
    Never sort, compress invalid rows out, interpolate, or reset the stride grid.
    """
    rows: list[Row | None] = []
    boundaries: dict[int, list[str]] = {}
    invalid = []
    gaps = []
    deltas = []
    previous = None
    active_boot = None
    history = {}
    seen_identity = None
    reboot_count = sequence_gaps = duplicate_count = out_of_order = 0
    with path.open(newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        if (not reader.fieldnames or len(set(reader.fieldnames)) != len(reader.fieldnames) or
                not REQUIRED.issubset(reader.fieldnames)):
            raise ValueError(f"{path.name}: missing/duplicate CSV columns")
        for index, record in enumerate(reader):
            try:
                row = parse_row(record)
                identity = (row.session_id, row.label)
                if seen_identity is None:
                    seen_identity = identity
                if identity != seen_identity or (expected_label and row.label != expected_label):
                    raise RuntimeError(f"{path.name}: mixed or unexpected session/label")
                prior = history.get(row.boot_id)
                if prior is not None:
                    step = (row.seq - prior.seq) & 0xFFFFFFFF
                    if step == 0:
                        duplicate_count += 1
                        raise ValueError("duplicate sequence")
                    if step >= 0x80000000:
                        out_of_order += 1
                        raise ValueError("out-of-order sequence")
                    if row.timestamp_ms <= prior.timestamp_ms:
                        raise ValueError("non-increasing timestamp within boot")
                    if row.boot_id != active_boot:
                        raise ValueError("reappearing old boot_id")
                reasons = []
                if active_boot is not None and row.boot_id != active_boot:
                    reboot_count += 1
                    reasons.append("reboot")
                if previous is not None:
                    if row.boot_id == previous.boot_id:
                        delta = row.timestamp_ms - previous.timestamp_ms
                        deltas.append(delta)
                        if delta > GAP_MS:
                            reasons.append("sampling_gap")
                            gaps.append({"row_index": index, "csv_line": reader.line_num, "delta_ms": delta})
                        step = (row.seq - previous.seq) & 0xFFFFFFFF
                        if step != 1:
                            reasons.append("sequence_gap")
                            sequence_gaps += step - 1
                if reasons:
                    boundaries[index] = reasons
                rows.append(row)
                history[row.boot_id] = row
                active_boot = row.boot_id
                previous = row
            except (ValueError, TypeError, OverflowError) as exc:
                rows.append(None)
                invalid.append({"row_index": index, "csv_line": reader.line_num, "reason": str(exc)})
                previous = None
    valid = [row for row in rows if row is not None]
    if not valid:
        raise ValueError(f"{path.name}: no valid samples")
    raw_windows, metadata = [], []
    rejected = Counter()
    constant_windows = 0
    for start in range(0, len(rows) - WINDOW + 1, STRIDE):
        selected = rows[start:start + WINDOW]
        if any(row is None for row in selected):
            rejected["invalid_row"] += 1
            continue
        reasons = {reason for index in range(start + 1, start + WINDOW)
                   for reason in boundaries.get(index, [])}
        if reasons:
            rejected["+".join(sorted(reasons))] += 1
            continue
        raw = np.array([row.axes for row in selected], dtype=np.float32)
        constant_windows += int(np.all(raw == raw[0]))
        raw_windows.append(raw)
        metadata.append({"file": path.name, "session_id": selected[0].session_id,
                         "label": selected[0].label, "class_id": LABELS[selected[0].label],
                         "boot_id": selected[0].boot_id, "start_row_index": start,
                         "end_row_index": start + WINDOW - 1,
                         "start_timestamp_ms": selected[0].timestamp_ms,
                         "end_timestamp_ms": selected[-1].timestamp_ms,
                         "start_seq": selected[0].seq, "end_seq": selected[-1].seq})
    axes = np.array([row.axes for row in valid], dtype=np.float64)
    ranges = {axis: {"min_g": float(axes[:, i].min()), "max_g": float(axes[:, i].max()),
                     "std_g": float(axes[:, i].std())} for i, axis in enumerate(("x", "y", "z"))}
    rail_rows = [i for i, row in enumerate(rows) if row and any(abs(a) >= 2.047 for a in row.axes)]
    boot_ranges = [{"boot_id": boot, "start_ms": min(r.timestamp_ms for r in valid if r.boot_id == boot),
                    "end_ms": max(r.timestamp_ms for r in valid if r.boot_id == boot)}
                   for boot in sorted({r.boot_id for r in valid})]
    report = {"file": path.name, "sha256": sha256(path), "session_id": valid[0].session_id,
              "label": valid[0].label, "rows": len(rows), "valid_rows": len(valid),
              "invalid_rows": invalid, "boot_ranges": boot_ranges, "reboots": reboot_count,
              "duplicates": duplicate_count, "out_of_order": out_of_order,
              "sequence_gaps": sequence_gaps, "sampling_gaps": gaps,
              "sampling_gap_count": len(gaps),
              "delta_ms": {"min": min(deltas) if deltas else None,
                           "median": float(np.median(deltas)) if deltas else None,
                           "max": max(deltas) if deltas else None},
              "axis_statistics": ranges, "possible_saturation_row_indices": rail_rows,
              "constant_axes": [axis for i, axis in enumerate(("x", "y", "z"))
                                if np.all(axes[:, i] == axes[0, i])],
              "constant_windows": constant_windows,
              "candidate_windows": max(0, (len(rows) - WINDOW) // STRIDE + 1),
              "usable_windows": len(raw_windows), "rejected_windows": dict(rejected),
              "warnings": []}
    if rail_rows:
        report["warnings"].append("Possible raw full-scale saturation; valid values retained, review recording")
    if report["constant_axes"] or constant_windows:
        report["warnings"].append("Constant data detected; may be REST or a stuck sensor, review provenance")
    if deltas and min(deltas) < 50:
        report["warnings"].append("Intervals below 50 ms detected; check the 10 Hz acquisition configuration")
    return raw_windows, metadata, report


@dataclass
class Dataset:
    raw: np.ndarray
    x: np.ndarray
    y: np.ndarray
    rms: np.ndarray
    windows: list[dict]
    reports: list[dict]

    def report(self):
        return {"window_samples": WINDOW, "stride_samples": STRIDE, "gap_limit_ms": GAP_MS,
                "stride_policy": "original session row grid; reject crossing windows; no reanchoring",
                "total_windows": len(self.y),
                "class_windows": {name: int(np.sum(self.y == value)) for name, value in LABELS.items()},
                "sessions": self.reports}


def load_dataset(paths) -> Dataset:
    raw, metadata, reports = [], [], []
    identities = set()
    for path in paths:
        path = Path(path)
        expected = "REST" if path.name.startswith("rest_") else "MOVING" if path.name.startswith("moving_") else None
        windows, origins, report = load_session(path, expected)
        if report["session_id"] in identities:
            raise ValueError("Duplicate session_id across input files")
        identities.add(report["session_id"])
        raw.extend(windows)
        metadata.extend(origins)
        reports.append(report)
    if not raw:
        raise ValueError("No usable 20-sample windows")
    array = np.array(raw, dtype=np.float32)
    return Dataset(array, np.stack([preprocess(window) for window in array]),
                   np.array([origin["class_id"] for origin in metadata], dtype=np.int32),
                   np.array([movement_rms(window) for window in array]), metadata, reports)


def require_independent(dataset: Dataset, training_reports):
    """Reject reused bytes, session IDs or overlapping device/boot time spans."""
    for test in dataset.reports:
        for train in training_reports:
            if test["sha256"] == train["sha256"] or test["session_id"] == train["session_id"]:
                raise ValueError("Evaluation reuses training data/session")
            for a in test["boot_ranges"]:
                for b in train["boot_ranges"]:
                    if a["boot_id"] == b["boot_id"] and max(a["start_ms"], b["start_ms"]) <= min(a["end_ms"], b["end_ms"]):
                        raise ValueError("Evaluation overlaps a training boot/time range")
    if set(dataset.y.tolist()) != {0, 1}:
        raise ValueError("Independent evaluation needs usable REST and MOVING windows")
