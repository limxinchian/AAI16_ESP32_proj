"""Host tests for ml_core (SHA-256, feature extraction, int8 glue, package parsing, input selector,
model store). Uses the REAL trained artifacts in ml/artifacts/ and independent Python references.

Run:  .venv\\Scripts\\python.exe tests\\ml_host\\run.py      (train first: ml/train_motion.py, ml/train_environment.py)
"""
import hashlib
import json
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "_gen"
ART = ROOT / "ml" / "artifacts"
sys.path.insert(0, str(ROOT / "tools"))
from profile_lib import load_pb  # noqa: E402

PRE = {"PREPROC_MOTION_STATS_V1": 1, "PREPROC_ENV_TREND_V1": 2}


def f9(x):
    return f"{float(x):.9g}"


def write_cases():
    OUT.mkdir(exist_ok=True)
    lines = []
    for task in ("motion", "environment"):
        g = json.loads((ART / task / "golden.json").read_text())
        for v in g["vectors"]:
            w = v["window"]
            lines.append(f"V {PRE[g['preprocessing']]} {len(w)} {len(w[0])} {len(v['features'])} {len(v['probs'])} {v['predicted']}")
            lines.append(" ".join(f9(x) for row in w for x in row))
            lines.append(" ".join(f9(x) for x in v["features"]))
            lines.append(" ".join(str(x) for x in v["int8_input"]))
            lines.append(" ".join(str(x) for x in v["int8_output"]))
            lines.append(" ".join(f9(x) for x in v["probs"]))
        c = json.loads((ART / task / "contract.json").read_text())
        lines.append(f"Q {c['inputScale']!r} {c.get('inputZeroPoint', 0)} {c['outputScale']!r} {c.get('outputZeroPoint', 0)}")
    (OUT / "cases.txt").write_text("\n".join(lines) + "\n")

    # independent SHA-256 vectors
    import random
    rnd = random.Random(7)
    sha = []
    for n in [0, 1, 3, 55, 56, 57, 63, 64, 65, 119, 120, 128, 1000, 5000]:
        data = bytes(rnd.randrange(256) for _ in range(n))
        sha.append(f"{n} {data.hex() or '-'} {hashlib.sha256(data).hexdigest()}")
    (OUT / "sha_cases.txt").write_text("\n".join(sha) + "\n")

    dig = []
    for task in ("motion", "environment"):
        data = (ART / task / f"{task}.amdl").read_bytes()
        (OUT / f"{task}.amdl").write_bytes(data)
        dig.append(f"{task}.amdl {hashlib.sha256(data).hexdigest()}")  # digest computed by hashlib, not by C
    (OUT / "packages.txt").write_text("\n".join(dig) + "\n")


def main():
    for task in ("motion", "environment"):
        if not (ART / task / f"{task}.amdl").exists():
            sys.exit(f"missing artifacts for {task}: run ml/train_{task}.py first")
    subprocess.run([sys.executable, str(ROOT / "tools" / "generate_proto.py")], stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL, check=True)
    load_pb()
    write_cases()

    exe = OUT / "test_ml.exe"
    srcs = [HERE / "test_ml.c"] + [ROOT / "components/ml_core" / n for n in
                                   ("sha256.c", "ml_features.c", "ml_package.c", "ml_selector.c", "model_store.c")] + [
        ROOT / "components/aai16_protocol/generated/aai16.pb.c",
        ROOT / "components/aai16_protocol/nanopb/pb_common.c",
        ROOT / "components/aai16_protocol/nanopb/pb_decode.c",
        ROOT / "components/aai16_protocol/nanopb/pb_encode.c"]
    cmd = [sys.executable, "-m", "ziglang", "cc", "-std=c11", "-Wall", "-Wextra", "-g",
           f"-I{ROOT / 'components/ml_core/include'}", f"-I{ROOT / 'components/aai16_protocol/generated'}",
           f"-I{ROOT / 'components/aai16_protocol/nanopb'}", "-o", str(exe), *map(str, srcs)]
    subprocess.run(cmd, check=True)
    r = subprocess.run([str(exe), str(OUT)])
    sys.exit(r.returncode)


if __name__ == "__main__":
    main()
