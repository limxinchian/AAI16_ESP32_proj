"""Score a REAL recording with a trained anomaly artifact (int8 model, exactly like the node) and print a timeline.

  python ml/eval_real_anomaly.py ml/artifacts/anomaly_real data/anomaly/test_01.csv [--setup bme280] [--bin 10]
Prints per time bin: worst score / threshold, the most unusual slot, and which slots were present.
With a NORMAL recording the number of bins above 1.0 is the false-alarm figure.
"""
import argparse
import json
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import anomaly as A  # noqa: E402
import common as C  # noqa: E402
import real_anomaly as R  # noqa: E402


def score_rows(art, X, M):
    import tensorflow as tf
    tfl = (art / "model_int8.tflite").read_bytes()
    run, (s_in, z_in, s_out, z_out), _ = C.tflite_runner(tfl, tf)
    thr = json.loads((art / "golden.json").read_text())["threshold"]
    sc, worst = [], []
    for x, m in zip(X, M):
        q_in, q_out, _ = run(x)
        s, e = A.score_from_int8(q_in, q_out, m, s_in, z_in, s_out, z_out)
        sc.append(s)
        worst.append(int(np.argmax(e)))
    return np.array(sc), np.array(worst), thr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("artifact")
    ap.add_argument("csv")
    ap.add_argument("--setup", default="bme280")
    ap.add_argument("--bin", type=float, default=10.0)
    a = ap.parse_args()
    streams = R.load(a.csv)
    T, X, M = R.replay(streams, R.SETUPS[a.setup], warmup=0.0)
    sc, worst, thr = score_rows(pathlib.Path(a.artifact), X, M)
    print(f"{len(T)} rows, threshold {thr:.5f}; above threshold: {100 * (sc > thr).mean():.1f} % of rows")
    t = 0.0
    while t < T[-1]:
        sel = (T >= t) & (T < t + a.bin)
        if sel.any():
            i = np.where(sel)[0][np.argmax(sc[sel])]
            pres = "".join("x" if M[i, s * 4] else "." for s in range(8))
            print(f"{t:5.0f}-{t + a.bin:4.0f}s  max {sc[i] / thr:6.2f}x  {'ANOMALY' if sc[i] > thr else 'normal '}  worst={A.SLOTS[worst[i]]:11s} present={pres}")
        t += a.bin


if __name__ == "__main__":
    main()
