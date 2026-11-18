"""Sampling-timing analysis of a recording made with tools/record_slot_data.py (stdlib only, deterministic).

  python tools/analyze_sampling_timing.py data/anomaly/normal_01.csv \
         --period node1:5=500 --period node1:8=250 --period node1:2=2000 --period node2:3=100
  python tools/analyze_sampling_timing.py FILE.csv --infer-period          # nominal period = median interval (rounded)
  python tools/analyze_sampling_timing.py FILE.csv ... --deadline-ms 20 --json out.json
  python tools/analyze_sampling_timing.py --selftest

Input columns: node, sensor_id, timestamp_us, valid (the timestamp is the node's own clock at the START of a cycle).
Each --period is NODE:SENSOR_ID=MILLISECONDS, i.e. the period the sensor profile requested.

The metrics are kept apart on purpose:
  readings / valid / invalid   an invalid reading is a cycle that ran but returned an error (timeout, bad frame, ...).
                               It says nothing about timing.
  skipped intervals            consecutive readings further apart than SKIP_FACTOR (1.5) x period. Each such gap hides
                               round(gap / period) - 1 missed cycles. "Zero detected skipped intervals" is only the absence
                               of gaps larger than that criterion.
  interval excess              longest gap minus the period (jitter between consecutive cycles).
  lateness vs ideal grid       signed distance of each timestamp from the nearest slot of the ideal grid (first timestamp +
                               k x period). The firmware keeps its phase after a missed cycle (it skips slots instead of
                               bursting), so one skipped cycle does not make every later reading "late"; skips are counted
                               separately. The first reading is the reference, so this is relative lateness, not absolute
                               latency, and a clock-rate error above half a period over the whole recording would wrap.
  deadline violations          only evaluated when --deadline-ms is given (readings whose lateness exceeds it). Without a
                               stated deadline nothing can be called a violation.
What this does NOT show: node-task execution time, interrupt-off time, behaviour during model uploads or flash writes, or
latency between the sensor and MQTT delivery. Timestamps taken at cycle start cannot reveal a cycle that started on time
but finished late.
"""
import argparse
import csv
import json
import statistics
import sys

SKIP_FACTOR = 1.5


def load(path):
    """-> {(node, sensor_id): [(timestamp_us, valid), ...]} sorted by time."""
    streams = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            key = (r["node"], int(r["sensor_id"]))
            streams.setdefault(key, []).append((int(r["timestamp_us"]), int(r["valid"])))
    for rows in streams.values():
        rows.sort()
    return streams


def percentile(sorted_values, q):
    return sorted_values[min(len(sorted_values) - 1, int(len(sorted_values) * q))]


def analyze_stream(rows, period_us, deadline_us=None):
    ts = [t for t, _ in rows]
    valid = sum(1 for _, v in rows if v)
    n = len(rows)
    out = {"readings": n, "valid": valid, "invalid": n - valid, "period_ms": period_us / 1000.0}
    if n < 2:
        out.update(expected_cycles=n, shortfall=0, skipped_intervals=0, missing_cycles_in_skipped=0, max_interval_excess_ms=0.0,
                   p99_interval_excess_ms=0.0, max_lateness_ms=0.0, min_lateness_ms=0.0, span_s=0.0, deadline_violations=None)
        return out
    intervals = [b - a for a, b in zip(ts, ts[1:])]
    span = ts[-1] - ts[0]
    expected = round(span / period_us) + 1
    skipped = [iv for iv in intervals if iv > SKIP_FACTOR * period_us]
    half = period_us // 2
    lateness = [((t - ts[0] + half) % period_us) - half for t in ts]  # signed distance to the nearest grid slot
    excess = sorted(iv - period_us for iv in intervals)
    out.update(
        span_s=span / 1e6,
        expected_cycles=expected,
        shortfall=expected - n,
        skipped_intervals=len(skipped),
        missing_cycles_in_skipped=sum(round(iv / period_us) - 1 for iv in skipped),
        max_interval_excess_ms=max(excess) / 1000.0,
        p99_interval_excess_ms=percentile(excess, 0.99) / 1000.0,
        max_lateness_ms=max(lateness) / 1000.0,
        min_lateness_ms=min(lateness) / 1000.0,
        deadline_violations=(sum(1 for x in lateness if x > deadline_us) if deadline_us is not None else None),
    )
    return out


def analyze(streams, periods_us, deadline_us=None):
    """streams: load() result; periods_us: {(node, sensor): period in microseconds} -> {(node, sensor): metrics}"""
    return {key: analyze_stream(rows, periods_us[key], deadline_us) for key, rows in sorted(streams.items()) if key in periods_us}


def infer_periods(streams):
    periods = {}
    for key, rows in streams.items():
        if len(rows) > 2:
            ivs = [b[0] - a[0] for a, b in zip(rows, rows[1:])]
            periods[key] = round(statistics.median(ivs) / 1000.0) * 1000  # whole milliseconds
    return periods


def print_report(results, deadline_us):
    hdr = (f"{'stream':12}{'period':>9}{'readings':>9}{'valid':>7}{'invalid':>8}{'expected':>9}{'skipped':>8}{'missing':>8}"
           f"{'max excess':>12}{'p99 excess':>12}{'lateness max/min':>22}{'deadline viol.':>16}")
    print(hdr)
    tot = {"readings": 0, "invalid": 0}
    per_node = {}
    for (node, sid), m in results.items():
        dv = "not evaluated" if m["deadline_violations"] is None else str(m["deadline_violations"])
        print(f"{node + ':' + str(sid):12}{m['period_ms']:7.0f}ms{m['readings']:9d}{m['valid']:7d}{m['invalid']:8d}{m['expected_cycles']:9d}"
              f"{m['skipped_intervals']:8d}{m['missing_cycles_in_skipped']:8d}{m['max_interval_excess_ms']:10.1f}ms"
              f"{m['p99_interval_excess_ms']:10.1f}ms{m['max_lateness_ms']:10.1f}/{m['min_lateness_ms']:.1f}ms{dv:>16}")
        tot["readings"] += m["readings"]
        tot["invalid"] += m["invalid"]
        pn = per_node.setdefault(node, {"readings": 0, "invalid": 0})
        pn["readings"] += m["readings"]
        pn["invalid"] += m["invalid"]
    print("\nper node: " + ", ".join(f"{n} {v['readings']} readings ({v['invalid']} invalid)" for n, v in sorted(per_node.items()))
          + f"; all streams {tot['readings']} readings ({tot['invalid']} invalid)")
    skipped = sum(m["skipped_intervals"] for m in results.values())
    print(f"skipped intervals under the {SKIP_FACTOR}x-period criterion: {skipped}")
    print("Note: zero detected skipped intervals under this criterion does not by itself prove zero deadline violations; "
          "see the module docstring for what is and is not measured.")


def selftest():
    P = 100_000  # 100 ms
    # 40 on-schedule readings, then a 300 ms gap (2 missed cycles hidden, phase kept), one reading 30 ms late, two invalid ones
    ts = [i * P for i in range(40)]
    ts += [ts[-1] + 3 * P + k * P for k in range(10)]            # gap of 3 periods -> 2 missing cycles
    ts[45] += 30_000                                             # one late reading
    rows = [(t, 0 if i in (7, 8) else 1) for i, t in enumerate(ts)]
    m = analyze_stream(rows, P, deadline_us=20_000)
    checks = {
        "readings": (m["readings"], 50),
        "invalid": (m["invalid"], 2),
        "valid": (m["valid"], 48),
        "skipped_intervals": (m["skipped_intervals"], 1),
        "missing_cycles_in_skipped": (m["missing_cycles_in_skipped"], 2),
        "max_interval_excess_ms": (round(m["max_interval_excess_ms"], 1), 200.0),
        "max_lateness_ms (late reading)": (round(m["max_lateness_ms"], 1), 30.0),
        "deadline_violations (>20 ms)": (m["deadline_violations"], 1),
    }
    ok = True
    for name, (got, want) in checks.items():
        status = "ok  " if got == want else "FAIL"
        ok &= got == want
        print(f"  {status} {name}: {got} (expected {want})")
    # a clean stream has no skipped intervals, no invalid readings and no excess
    clean = analyze_stream([(i * P, 1) for i in range(100)], P)
    for name, got, want in (("clean skipped", clean["skipped_intervals"], 0), ("clean invalid", clean["invalid"], 0),
                            ("clean max lateness", clean["max_lateness_ms"], 0.0)):
        status = "ok  " if got == want else "FAIL"
        ok &= got == want
        print(f"  {status} {name}: {got} (expected {want})")
    # two nodes and two sensors are kept separate
    rows2 = {("a", 1): [(i * P, 1) for i in range(10)], ("b", 1): [(i * P, 1) for i in range(5)]}
    res = analyze(rows2, {("a", 1): P, ("b", 1): P})
    sep = res[("a", 1)]["readings"] == 10 and res[("b", 1)]["readings"] == 5
    print(f"  {'ok  ' if sep else 'FAIL'} streams of different nodes are not mixed")
    ok &= sep
    print("SELFTEST " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description="Sampling-timing analysis of a record_slot_data.py CSV")
    ap.add_argument("csv", nargs="?")
    ap.add_argument("--period", action="append", default=[], help="NODE:SENSOR_ID=MS (repeatable)")
    ap.add_argument("--infer-period", action="store_true", help="use the median interval as the nominal period")
    ap.add_argument("--deadline-ms", type=float, help="lateness above this counts as a deadline violation (otherwise not evaluated)")
    ap.add_argument("--json", help="also write the metrics to this file")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        sys.exit(selftest())
    if not a.csv:
        ap.error("give a CSV file (or --selftest)")
    streams = load(a.csv)
    periods = infer_periods(streams) if a.infer_period else {}
    for item in a.period:
        key, ms = item.split("=")
        node, sid = key.split(":")
        periods[(node, int(sid))] = round(float(ms) * 1000)
    missing = [k for k in streams if k not in periods]
    if missing:
        ap.error("no period for stream(s): " + ", ".join(f"{n}:{s}" for n, s in missing) + " (use --period or --infer-period)")
    deadline_us = None if a.deadline_ms is None else round(a.deadline_ms * 1000)
    results = analyze(streams, periods, deadline_us)
    print_report(results, deadline_us)
    if a.json:
        with open(a.json, "w") as f:
            json.dump({f"{n}:{s}": m for (n, s), m in results.items()}, f, indent=1)


if __name__ == "__main__":
    main()
