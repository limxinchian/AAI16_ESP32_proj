"""Create/install the isolated ML venv, including a Windows long-path workaround.

Run with a standalone Python 3.12: py -3.12 tools/ml_training/setup_env.py
Only this project's tools/ml_training/.venv receives installed packages.
"""
from pathlib import Path
import argparse
import os
import subprocess
import sys
import venv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--requirements", type=Path,
                        default=Path(__file__).resolve().parent / "requirements.txt",
                        help="Use the saved environment.lock.txt for exact package reproduction")
    args = parser.parse_args()
    if sys.version_info[:2] != (3, 12):
        raise SystemExit("Use standalone Python 3.12: py -3.12 tools/ml_training/setup_env.py")
    here = Path(__file__).resolve().parent
    target = here / ".venv"
    executable = target / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    if not executable.exists():
        venv.EnvBuilder(with_pip=True).create(target)
    python = str(executable)
    # TensorFlow ships deeply nested C++ headers. Prefix only the isolated venv
    # interpreter, so pip's sys.prefix uses extended paths without OS changes.
    if os.name == "nt" and not python.startswith("\\\\?\\"):
        python = "\\\\?\\" + python
    subprocess.run([python, "-m", "pip", "install", "--disable-pip-version-check",
                    "--no-cache-dir", "-r", str(args.requirements.resolve())], check=True)
    print(f"ML environment ready: {executable}")


if __name__ == "__main__":
    main()
