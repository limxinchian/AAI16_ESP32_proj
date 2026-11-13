"""Model C - cargo anomaly detector (MODEL_ANOMALY). Synthetic normal data, optionally mixed with real recordings (--real).

Run:  .venv\\Scripts\\python.exe ml\\train_anomaly.py [--out ml/artifacts/anomaly] [--version 1] [--real data/anomaly/normal_01.csv]
Writes the model package (anomaly.amdl), contract.json, golden.json (for the C tests) and report.json.
"""
import argparse
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import anomaly as A  # noqa: E402
import common as C  # noqa: E402

SEED = 20261113


def build_model(tf):
    inp = tf.keras.Input(shape=(A.N_FEATURES,))
    x = tf.keras.layers.Dense(24, activation="relu")(inp)
    x = tf.keras.layers.Dense(12, activation="relu")(x)
    x = tf.keras.layers.Dense(24, activation="relu")(x)
    out = tf.keras.layers.Dense(A.N_FEATURES)(x)
    return tf.keras.Model(inp, out)


def masked_mse(tf):
    def loss(y_true, y_pred):  # y_true = [features | mask]
        x, m = y_true[:, :A.N_FEATURES], y_true[:, A.N_FEATURES:]
        return tf.reduce_sum(m * tf.square(x - y_pred), axis=1) / (tf.reduce_sum(m, axis=1) + 1e-6)
    return loss


def check_reference(rng):
    """The vectorised features must equal the scalar SlotTracker (which the C code mirrors)."""
    worst = 0.0
    for s in range(A.N_SLOTS):
        v, dt = A.gen_normal(s, 6, rng)
        vec = A.slot_features_batch(s, v, dt)
        for i in range(6):
            tr = A.SlotTracker(s)
            for k in range(v.shape[1]):
                tr.update(v[i, k], k * dt[i])
            worst = max(worst, float(np.abs(np.array(tr.features()) - vec[i]).max()))
    assert worst < 1e-5, f"vectorised and scalar features differ by {worst}"
    return worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(HERE / "artifacts" / "anomaly"))
    ap.add_argument("--version", type=int, default=1)
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--real", action="append", default=[], help="CSV from tools/record_slot_data.py with NORMAL conditions (repeatable)")
    ap.add_argument("--margin", type=float, default=1.5, help="threshold = p99.7 of real validation scores x margin (with --real)")
    a = ap.parse_args()
    import tensorflow as tf

    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    tf.keras.utils.set_random_seed(SEED)
    rng = np.random.default_rng(SEED)
    print("scalar vs vectorised feature check, worst difference:", check_reference(np.random.default_rng(1)))

    print("generating synthetic pools ...")
    train_pool = A.slot_pool("normal", 12000, np.random.default_rng(SEED + 1))
    val_pool = A.slot_pool("normal", 3000, np.random.default_rng(SEED + 2))
    test_pool = A.slot_pool("normal", 3000, np.random.default_rng(SEED + 3))
    anom_pool = A.slot_pool("anomaly", 1500, np.random.default_rng(SEED + 4))

    real = None
    if a.real:
        import real_anomaly as R
        tr, va, te = [], [], []
        for path in a.real:
            _, X_, M_, counts = R.replay_all(path)
            # chronological split inside each setup (BME280 rows first, then DHT22 rows)
            start = 0
            for c_ in counts:
                s_tr, s_va, s_te = R.split_time(c_)
                tr.append((X_[start:start + c_][s_tr], M_[start:start + c_][s_tr]))
                va.append((X_[start:start + c_][s_va], M_[start:start + c_][s_va]))
                te.append((X_[start:start + c_][s_te], M_[start:start + c_][s_te]))
                start += c_
        cat = lambda parts, k: np.concatenate([p[k] for p in parts if len(p[0])])
        real = dict(train=(cat(tr, 0), cat(tr, 1)), val=(cat(va, 0), cat(va, 1)), test=(cat(te, 0), cat(te, 1)))
        print("real rows: train %d  val %d  test %d  (joint rows, 0.5 s ticks; %d file(s))" % (
            len(real["train"][0]), len(real["val"][0]), len(real["test"][0]), len(a.real)))
        X, M = R.mixture(real["train"][0], real["train"][1], train_pool, 80000, rng)
    else:
        X, M = A.assemble(train_pool, 80000, rng)
    model = build_model(tf)
    model.compile(optimizer=tf.keras.optimizers.Adam(2e-3), loss=masked_mse(tf))
    hist = model.fit(X, np.concatenate([X, M], axis=1), epochs=a.epochs, batch_size=256, validation_split=0.1, verbose=0)
    print(f"final masked loss: train {hist.history['loss'][-1]:.5f}  val {hist.history['val_loss'][-1]:.5f}")

    Xv, Mv = A.assemble(val_pool, 6000, rng)
    tfl = C.export_int8(model, Xv[:500], tf)
    run, (s_in, z_in, s_out, z_out), it = C.tflite_runner(tfl, tf)
    ops = C.op_names(it)
    print("TFLite ops:", ops, "| input scale %.5f zp %d | output scale %.5f zp %d" % (s_in, z_in, s_out, z_out))

    def scores(X_, M_):
        sc, se = [], []
        for x, m in zip(X_, M_):
            q_in, q_out, _ = run(x)
            s, e = A.score_from_int8(q_in, q_out, m, s_in, z_in, s_out, z_out)
            sc.append(s)
            se.append(e)
        return np.array(sc), np.array(se)

    def subsets(X_, M_, rng_):
        """the real rows as they are, plus copies where each recorded slot is missing with probability 0.25"""
        Xs_, Ms_ = [X_], [M_]
        for _ in range(3):
            keep = rng_.random((len(X_), 8)) > 0.25
            keep[~keep.any(axis=1), 0] = True
            k32 = np.repeat(keep, 4, axis=1).astype(np.float32)
            m2 = M_ * k32
            ok = m2.any(axis=1)
            Xs_.append((X_ * k32)[ok])
            Ms_.append(m2[ok])
        return np.concatenate(Xs_), np.concatenate(Ms_)

    real_report = None
    if real is not None:
        Xrv, Mrv = subsets(real["val"][0], real["val"][1], rng)
        val_scores, _ = scores(Xrv, Mrv)
        p997 = float(np.percentile(val_scores, 99.7))
        threshold = float(p997 * a.margin)
        print(f"REAL validation score: median {np.median(val_scores):.5f}  p99.7 {p997:.5f}  max {val_scores.max():.5f}  -> threshold {threshold:.5f} (x{a.margin})")
        Xrt, Mrt = subsets(real["test"][0], real["test"][1], rng)
        rt_scores, _ = scores(Xrt, Mrt)
        real_far = float((rt_scores > threshold).mean())
        print(f"false alarm rate on held-out REAL normal data (last 15 % of each recording): {100 * real_far:.2f} %  (max score {rt_scores.max():.5f})")
        real_report = dict(files=[pathlib.Path(p).name for p in a.real], rows=dict(train=len(real["train"][0]), val=len(real["val"][0]), test=len(real["test"][0])),
                           p99_7_val=p997, margin=a.margin, false_alarm_rate_real_test=real_far, max_score_real_test=float(rt_scores.max()))
        Xv, Mv = A.assemble(val_pool, 6000, rng)
        val_scores_syn, _ = scores(Xv, Mv)
        print(f"(synthetic normal for reference: median {np.median(val_scores_syn):.5f}  p99.7 {np.percentile(val_scores_syn, 99.7):.5f})")
    else:
        val_scores, _ = scores(Xv, Mv)
        p997 = float(np.percentile(val_scores, 99.7))
        threshold = float(p997 * 1.25)
        print(f"validation normal score: median {np.median(val_scores):.5f}  p99.7 {p997:.5f}  -> threshold {threshold:.5f}")

    Xt, Mt = A.assemble(test_pool, 6000, rng)
    test_scores, _ = scores(Xt, Mt)
    far = float((test_scores > threshold).mean())
    print(f"false alarm rate on held-out SYNTHETIC normal data: {100 * far:.2f} %")

    detect = {}
    anom_examples = []
    for slot, name in enumerate(A.SLOTS):
        # the injected slot comes from the anomaly pool, every other slot stays normal
        mixed = [anom_pool[t] if t == slot else test_pool[t] for t in range(A.N_SLOTS)]
        Xs, Ms = A.assemble(mixed, 1500, rng, force=slot)
        sc, se = scores(Xs, Ms)
        hit = sc > threshold
        top = np.array([int(np.argmax(e)) for e in se])  # is the injected slot the one with the largest error?
        detect[name] = dict(detection_rate=float(hit.mean()),
                            attribution_rate_among_hits=float((top[hit] == slot).mean()) if hit.any() else 0.0)
        anom_examples.append((Xs[0], Ms[0]))
        print(f"  {name:12s} anomaly: detected {100 * detect[name]['detection_rate']:5.1f} %   named correctly {100 * detect[name]['attribution_rate_among_hits']:5.1f} %")

    # contract + package
    c = C.pb.ModelContract(contract_version=1, kind=C.pb.MODEL_ANOMALY, model_version=a.version, format=C.pb.FORMAT_TFLITE,
                           window_samples=1, sample_period_ms=1000, period_tolerance_pct=50, max_age_ms=6000,
                           preprocessing=C.pb.PREPROC_SLOT_STATS_V1, feature_count=A.N_FEATURES, stride_samples=1,
                           tensor_arena_bytes=16384, input_scale=s_in, input_zero_point=z_in, output_scale=s_out,
                           output_zero_point=z_out, output_count=A.N_FEATURES, anomaly_threshold=threshold)
    c.channels.extend(getattr(C.pb, k) for k in A.CHANNEL_KINDS)
    c.labels.extend(["NORMAL", "ANOMALY"])
    package = C.build_package(c, tfl)
    C.parse_package(package)
    (out / "model_int8.tflite").write_bytes(tfl)
    (out / "anomaly.amdl").write_bytes(package)
    from google.protobuf import json_format
    C.write_json(out / "contract.json", json_format.MessageToDict(c))

    # golden vectors for the C tests: 8 normal + one anomaly per slot
    vecs = []
    examples = [(Xt[i], Mt[i], "normal") for i in range(8)] + [(x, m, "anomaly") for x, m in anom_examples]
    for x, mf, lab in examples:
        q_in, q_out, _ = run(x)
        sc, se = A.score_from_int8(q_in, q_out, mf, s_in, z_in, s_out, z_out)
        vecs.append(dict(label=lab, features=x.tolist(), slot_present=[int(v) for v in mf[::A.N_FEAT_SLOT]],
                         int8_input=q_in.tolist(), int8_output=q_out.tolist(), score=sc, slot_errors=se,
                         anomaly=bool(sc > threshold)))
    series = []
    gr = np.random.default_rng(SEED + 9)
    for s in range(A.N_SLOTS):
        for _ in range(2):
            v, dt = A.gen_normal(s, 1, gr)
            n = 200
            ts = (np.arange(n) * dt[0]).tolist()
            tr = A.SlotTracker(s)
            for k in range(n):
                tr.update(v[0, k], ts[k])
            series.append(dict(slot=s, t=ts, x=v[0, :n].tolist(), features=tr.features()))
    C.write_json(out / "golden.json", dict(task="anomaly", preprocessing="PREPROC_SLOT_STATS_V1", threshold=threshold,
                                           quant=dict(s_in=s_in, z_in=z_in, s_out=s_out, z_out=z_out), vectors=vecs, slot_series=series))
    C.write_json(out / "report.json", dict(
        task="anomaly",
        data_provenance=("SYNTHETIC - not validated on real sensors; normal cargo operation only for training; anomalies injected for evaluation"
                         if real_report is None else
                         "MIXED - 60 % real normal recordings (BME280/DHT22/MAX6675/LD2450, augmented) + 40 % synthetic; the unrecorded slots (motion, co2, distance) are synthetic only"),
        real=real_report, seed=SEED, tflite_bytes=len(tfl), ops=ops, threshold=threshold, p99_7_normal=p997, false_alarm_rate=far,
        per_anomaly=detect, final_loss=dict(train=float(hist.history["loss"][-1]), val=float(hist.history["val_loss"][-1]))))
    print(f"wrote {out}  (package {len(package)} B, tflite {len(tfl)} B)")


if __name__ == "__main__":
    main()
