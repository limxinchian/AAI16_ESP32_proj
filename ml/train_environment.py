"""Model B - Environmental Classification (DHT22 or BME280 temperature + humidity): STABLE / CHANGING.

Run:  .venv\\Scripts\\python.exe ml\\train_environment.py
Trains on SYNTHETIC windows (see ml/common.py). Replace the generator with real captures to retrain.
"""
import argparse

from pipeline import run

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--data", help="folder of tools/collect_readings.py CSVs (REAL data). Omit for the synthetic bootstrap.")
    ap.add_argument("--out", help="output folder (default ml/artifacts/environment); use a new folder to keep the synthetic model")
    ap.add_argument("--version", type=int, help="model_version in the contract (use a higher number than the one on the node)")
    ap.add_argument("--period-ms", type=int, help="sample period in the contract (default 2000). 500 = fast variant for the BME280")
    ap.add_argument("--stride", type=int, help="predict every N new samples (default 5)")
    ap.add_argument("--max-age-ms", type=int, help="newest sample may be at most this old (default 3 x period)")
    a = ap.parse_args()
    run("environment", data_dir=a.data, out_dir=a.out, model_version=a.version,
        period_ms=a.period_ms, stride=a.stride, max_age_ms=a.max_age_ms)
