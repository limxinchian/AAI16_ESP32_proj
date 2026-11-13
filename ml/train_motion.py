"""Model A - Motion Classification (GY-511 accelerometer): STATIONARY / MOVING / VIBRATING.

Run:  .venv\\Scripts\\python.exe ml\\train_motion.py
Trains on SYNTHETIC windows (see ml/common.py). Replace the generator with real captures to retrain.
"""
import argparse

from pipeline import run

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--data", help="folder of tools/collect_readings.py CSVs (REAL data). Omit for the synthetic bootstrap.")
    ap.add_argument("--out", help="output folder (default ml/artifacts/motion); use a new folder to keep the synthetic model")
    ap.add_argument("--version", type=int, help="model_version in the contract (use a higher number than the one on the node)")
    a = ap.parse_args()
    run("motion", data_dir=a.data, out_dir=a.out, model_version=a.version)
