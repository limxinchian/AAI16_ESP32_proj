"""Train, quantize, verify and package one model. Used by train_motion.py / train_environment.py.

Artifacts -> ml/artifacts/<task>/ :
  model_int8.tflite   full-integer quantized TFLite (what TFLM runs)
  <task>.amdl         model package = ModelContract + .tflite (what is uploaded over MQTT)
  golden.json         raw windows + expected features + int8 in/out, to pin firmware to Python
  contract.json       human-readable copy of the embedded ModelContract
  report.json         accuracy, confusion matrix, ops, seeds, versions, data provenance
  SHA256SUMS
"""
import hashlib
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import common as C  # noqa: E402

SEED = 20261104
PER_CLASS = 3000
GOLDEN_PER_CLASS = 5

TASKS = {
    "motion": dict(
        generate=C.generate_motion, features=C.motion_features, labels=C.MOTION_LABELS,
        n_features=C.MOTION_FEATURES, hidden=(16, 8), model_version=1,
        channels=["VALUE_ACCEL_X_G", "VALUE_ACCEL_Y_G", "VALUE_ACCEL_Z_G"],
        window=C.MOTION_WINDOW, stride=32, period_ms=1000 // C.MOTION_FS_HZ, tolerance_pct=25, max_age_ms=500,
        preproc="PREPROC_MOTION_STATS_V1", kind="MODEL_MOTION", columns=["ax", "ay", "az"],
        data_note="synthetic: gravity + noise (STATIONARY), 0.4-3 Hz sway (MOVING), 7-22 Hz shake (VIBRATING)"),
    "environment": dict(
        generate=C.generate_env, features=C.env_features, labels=C.ENV_LABELS,
        n_features=C.ENV_FEATURES, hidden=(12, 8), model_version=1,
        channels=["VALUE_TEMPERATURE_C", "VALUE_HUMIDITY_PCT"],
        window=C.ENV_WINDOW, stride=5, period_ms=C.ENV_PERIOD_MS, tolerance_pct=30, max_age_ms=3 * C.ENV_PERIOD_MS,
        preproc="PREPROC_ENV_TREND_V1", kind="MODEL_ENVIRONMENT", columns=["temperature", "humidity"],
        data_note="synthetic: DHT22-like and BME280-like noise/quantization; STABLE <0.15C&<0.6%RH drift, "
                  "CHANGING >=0.8C or >=4%RH over the window; the gap between is never sampled"),
}


def run(task, data_dir=None, out_dir=None, model_version=None, period_ms=None, stride=None, max_age_ms=None):
    """data_dir=None -> SYNTHETIC data (default). data_dir=<folder of collect_readings CSVs> -> REAL recordings."""
    import tensorflow as tf

    cfg = dict(TASKS[task])
    # Timing of the contract (NOT of the network): the window is always 20 samples, so a shorter sample period gives a
    # shorter window (20 x 500 ms = 10 s instead of 20 x 2000 ms = 40 s) and a smaller stride predicts more often.
    if period_ms:
        cfg["period_ms"] = int(period_ms)
        cfg["max_age_ms"] = int(max_age_ms) if max_age_ms else 3 * int(period_ms)
    if stride:
        cfg["stride"] = int(stride)
    if max_age_ms and not period_ms:
        cfg["max_age_ms"] = int(max_age_ms)
    if period_ms or stride:
        cfg["data_note"] = cfg["data_note"] + f"; contract timing overridden: sample period {cfg['period_ms']} ms, stride {cfg['stride']}"
    out = pathlib.Path(out_dir) if out_dir else pathlib.Path(__file__).resolve().parent / "artifacts" / task
    out.mkdir(parents=True, exist_ok=True)
    tf.keras.utils.set_random_seed(SEED)
    rng = np.random.default_rng(SEED)
    n_cls = len(cfg["labels"])

    real_stats, split_mode = None, "random (synthetic)"
    if data_dir:
        import real_data
        train_stride = max(1, cfg["window"] // 8)
        windows, ys, groups, real_stats = real_data.load_task(cfg, data_dir, train_stride)
        missing = [l for l in cfg["labels"] if real_stats["per_class_windows"][l] == 0]
        if missing:
            raise SystemExit(f"no usable windows for class(es) {missing}; need recordings of every class. stats: {real_stats}")
        X = np.array([cfg["features"](w) for w in windows], dtype=np.float32)
        tr, va, te, split_mode = real_data.split_indices(ys, groups, rng, cfg["window"], train_stride, n_cls)
        if min(len(tr), len(va), len(te)) == 0:
            raise SystemExit(f"split produced an empty set (train {len(tr)}, val {len(va)}, test {len(te)}): record more data")
    else:
        windows, ys = [], []
        for cls in range(n_cls):
            for _ in range(PER_CLASS):
                windows.append(cfg["generate"](cls, rng))
                ys.append(cls)
        windows, ys = np.array(windows), np.array(ys)
        X = np.array([cfg["features"](w) for w in windows], dtype=np.float32)
        perm = rng.permutation(len(ys))
        n_tr, n_va = int(0.7 * len(ys)), int(0.15 * len(ys))
        tr, va, te = perm[:n_tr], perm[n_tr:n_tr + n_va], perm[n_tr + n_va:]

    layers = [tf.keras.layers.Input(shape=(cfg["n_features"],))]
    layers += [tf.keras.layers.Dense(h, activation="relu") for h in cfg["hidden"]]
    layers += [tf.keras.layers.Dense(n_cls, activation="softmax")]
    model = tf.keras.Sequential(layers)
    model.compile(optimizer=tf.keras.optimizers.Adam(1e-2), loss="sparse_categorical_crossentropy", metrics=["accuracy"])
    hist = model.fit(X[tr], ys[tr], validation_data=(X[va], ys[va]), epochs=80, batch_size=64, verbose=0,
                     callbacks=[tf.keras.callbacks.EarlyStopping(patience=10, restore_best_weights=True)])
    float_pred = model.predict(X[te], verbose=0).argmax(1)
    float_acc = float((float_pred == ys[te]).mean())

    tfl = C.export_int8(model, X[tr], tf)
    run_q, (s_in, z_in, s_out, z_out), interp = C.tflite_runner(tfl, tf)
    q_pred = np.array([run_q(f)[2].argmax() for f in X[te]])
    q_acc = float((q_pred == ys[te]).mean())
    agree = float((q_pred == float_pred).mean())
    conf = np.zeros((n_cls, n_cls), int)
    for t, p in zip(ys[te], q_pred):
        conf[t, p] += 1

    # firmware golden vectors: raw window -> features -> int8 in/out
    gold = []
    for cls in range(n_cls):
        idx = [i for i in te if ys[i] == cls][:GOLDEN_PER_CLASS]
        for i in idx:
            q_in, q_out, probs = run_q(X[i])
            gold.append(dict(label=int(ys[i]), window=windows[i].tolist(), features=X[i].tolist(),
                             int8_input=q_in.tolist(), int8_output=q_out.tolist(),
                             probs=probs.tolist(), predicted=int(probs.argmax())))

    c = C.pb.ModelContract(contract_version=1, kind=getattr(C.pb, cfg["kind"]), model_version=model_version or cfg["model_version"],
                           format=C.pb.FORMAT_TFLITE, window_samples=cfg["window"], sample_period_ms=cfg["period_ms"],
                           period_tolerance_pct=cfg["tolerance_pct"], max_age_ms=cfg["max_age_ms"],
                           preprocessing=getattr(C.pb, cfg["preproc"]), feature_count=cfg["n_features"],
                           stride_samples=cfg["stride"],
                           tensor_arena_bytes=16384, input_scale=s_in, input_zero_point=z_in,
                           output_scale=s_out, output_zero_point=z_out)
    c.channels.extend(getattr(C.pb, n) for n in cfg["channels"])
    c.labels.extend(cfg["labels"])
    package = C.build_package(c, tfl)
    C.parse_package(package)  # round-trip self-check

    (out / "model_int8.tflite").write_bytes(tfl)
    (out / f"{task}.amdl").write_bytes(package)
    C.write_json(out / "golden.json", dict(task=task, preprocessing=cfg["preproc"], vectors=gold))
    from google.protobuf import json_format
    C.write_json(out / "contract.json", json_format.MessageToDict(c))
    report = dict(
        task=task,
        data_provenance=("REAL recordings from " + str(data_dir) + f" ({real_stats['files']} files); split: {split_mode}") if data_dir
        else "SYNTHETIC - not validated on real sensor data; " + cfg["data_note"],
        real_data_stats=real_stats, split_mode=split_mode,
        seed=SEED, windows_per_class=PER_CLASS if not data_dir else None, split=dict(train=len(tr), val=len(va), test=len(te)),
        epochs_run=len(hist.history["loss"]), float_test_accuracy=float_acc, int8_test_accuracy=q_acc,
        float_vs_int8_agreement=agree, confusion_matrix_rows_true_cols_pred=conf.tolist(), labels=cfg["labels"],
        tflite_bytes=len(tfl), package_bytes=len(package), ops=C.op_names(interp),
        tensor_arena_bytes_declared=16384, tensor_arena_note="declared guess; NOT measured on target (TFLM arena probe pending)",
        versions=dict(tensorflow=tf.__version__, numpy=np.__version__, python=sys.version.split()[0]))
    C.write_json(out / "report.json", report)
    sums = "".join(f"{hashlib.sha256((out / n).read_bytes()).hexdigest()}  {n}\n"
                   for n in ("model_int8.tflite", f"{task}.amdl", "golden.json", "contract.json"))
    (out / "SHA256SUMS").write_text(sums)

    print(f"[{task}] float acc {float_acc:.4f} | int8 acc {q_acc:.4f} | agreement {agree:.4f} | "
          f"tflite {len(tfl)} B | ops {report['ops']}")
    print(f"[{task}] confusion (rows=true):\n{conf}")
    if not data_dir:
        assert q_acc > 0.95, "int8 accuracy unexpectedly low"
    elif q_acc < 0.8:
        print(f"[{task}] WARNING: int8 test accuracy {q_acc:.2f} is low: more/cleaner recordings are needed before this model is useful")
    return report
