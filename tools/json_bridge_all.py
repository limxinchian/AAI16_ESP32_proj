"""Run the JSON helper for several nodes with ONE command (one helper per node, started for you).

  python tools/json_bridge_all.py --broker 172.20.10.2                    # node1 and node2
  python tools/json_bridge_all.py --broker 172.20.10.2 --nodes node1 node2 node3
  python tools/json_bridge_all.py --broker 172.20.10.2 --details --with-raw   # extra flags go to every helper

Each node gets its own JSON topics:  aai16/<node>/json/...  and takes JSON on  aai16/<node>/json_in/...
Output lines are prefixed with the node name. Press Ctrl+C once to stop all of them.
Do not run another json_bridge.py for the same node at the same time (every message would be answered twice).
"""
import argparse
import pathlib
import subprocess
import sys
import threading
import time

HERE = pathlib.Path(__file__).resolve().parent


def pump(name, proc):
    for line in proc.stdout:
        print(f"[{name}] {line.rstrip()}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--broker", required=True)
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--nodes", nargs="+", default=["node1", "node2"])
    a, extra = ap.parse_known_args()

    procs = []
    for node in a.nodes:
        cmd = [sys.executable, "-u", str(HERE / "json_bridge.py"), "--broker", a.broker, "--port", str(a.port), "--node", node, *extra]
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        procs.append((node, p))
        threading.Thread(target=pump, args=(node, p), daemon=True).start()
    print(f"started the JSON helper for: {', '.join(a.nodes)}  (Ctrl+C stops all)", flush=True)

    try:
        while True:
            time.sleep(0.5)
            for node, p in procs:
                if p.poll() is not None:
                    print(f"[{node}] helper stopped with code {p.returncode}; stopping the others", flush=True)
                    raise KeyboardInterrupt
    except KeyboardInterrupt:
        pass
    finally:
        for _, p in procs:
            if p.poll() is None:
                p.terminate()
        for _, p in procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()


if __name__ == "__main__":
    main()
