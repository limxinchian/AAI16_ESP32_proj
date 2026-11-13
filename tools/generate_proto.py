"""Regenerate nanopb C sources from protocol/aai16.proto.

Run from the repo root with the project venv:
    .venv\\Scripts\\python.exe tools\\generate_proto.py
Requires: pip install nanopb grpcio-tools protobuf  (see tools/requirements.txt)
"""
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "components" / "aai16_protocol" / "generated"


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable, "-m", "nanopb.generator.nanopb_generator",
        "-I", str(ROOT / "protocol"),
        "-f", str(ROOT / "protocol" / "aai16.options"),
        "-D", str(OUT),
        str(ROOT / "protocol" / "aai16.proto"),
    ]
    return subprocess.call(cmd)


if __name__ == "__main__":
    sys.exit(main())
