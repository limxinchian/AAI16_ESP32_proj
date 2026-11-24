"""Run every host test suite plus the laptop client's offline self-test.
Usage:  .venv\\Scripts\\python.exe tools\\run_all_tests.py"""
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
SUITES = ["config_host", "decode_host", "ml_host", "runtime_host", "services_host", "node_host", "client_mqtt", "topics_host", "real_data_host", "shorthand_host", "anomaly_host", "dormant_host", "demo_pack_host"]


def run(label, cmd, keep):
    print(f"=== {label}", flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True)
    tail = [ln for ln in r.stdout.splitlines() if any(k in ln for k in keep)]
    print("\n".join(tail) or (r.stdout[-400:] + r.stderr[-400:]))
    return r.returncode == 0


def main():
    failed = []
    for s in SUITES:
        if not run(s, [sys.executable, str(ROOT / "tests" / s / "run.py")], ("checks,", "cases,", "FAIL", "recorded TFLite", "ALL CLIENT", "TOPICS AGREE", "REAL-DATA PATH", "DEMO PACK", "ALL SHORTHAND")):
            failed.append(s)
    if not run("node_client selftest", [sys.executable, str(ROOT / "tools" / "node_client.py"), "selftest"], ("ok ", "FAIL", "skip")):
        failed.append("node_client selftest")
    print("\nALL HOST SUITES PASS" if not failed else f"\nFAILED: {', '.join(failed)}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
