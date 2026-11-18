"""The firmware's MQTT topic logic (components/node_core/node_topics.c, also used by plat_net.c) must agree with the
laptop client (tools/node_client.py) in BOTH directions; a single typo on either side would silently break the demo.

Run:  .venv\\Scripts\\python.exe tests\\topics_host\\run.py
"""
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "_gen"
sys.path.insert(0, str(ROOT / "tools"))
import node_client as nc  # noqa: E402

pb = nc.pb
NODE = "node1"
IN_KIND = {"config": "IN_CONFIG", "rules": "IN_RULES", "bridge": "IN_BRIDGE", "model/chunk": "IN_MODEL_CHUNK",
           "model/control": "IN_MODEL_CONTROL", "command": "IN_COMMAND"}
OUT_KIND = {"config_ack": "OUT_APPLY_ACK", "rules_ack": "OUT_RULE_ACK", "bridge_ack": "OUT_BRIDGE_ACK",
            "model_chunk_ack": "OUT_CHUNK_ACK", "model_status": "OUT_MODEL_STATUS", "inference": "OUT_INFERENCE",
            "heartbeat": "OUT_HEARTBEAT", "notification": "OUT_NOTIFICATION"}


def main():
    OUT.mkdir(exist_ok=True)
    exe = OUT / "test_topics.exe"
    comps = ROOT / "components"
    incs = [comps / n / "include" for n in ("node_core", "node_services", "sensor_runtime", "sensor_decode", "ml_core", "config_manager")]
    subprocess.run([sys.executable, "-m", "ziglang", "cc", "-std=c11", "-Wall", "-Wextra", *[f"-I{i}" for i in incs],
                    f"-I{comps / 'aai16_protocol/generated'}", f"-I{comps / 'aai16_protocol/nanopb'}",
                    "-o", str(exe), str(HERE / "test_topics.c"), str(comps / "node_core/node_topics.c")], check=True)

    requests, expect = [], []

    def classify(topic, want):
        requests.append(f"C {NODE} {topic}")
        expect.append(("R", want))

    def build(kind, target, payload, want_topic, want_qos):
        requests.append(f"B {NODE} {kind} {target or '-'} {payload.hex() if payload else '-'}")
        expect.append(("T", (want_topic, want_qos)))

    # laptop -> node: every topic the client publishes to must be understood by the firmware
    for t in nc.IN_TOPICS:
        classify(f"aai16/{NODE}/in/{t}", (IN_KIND[t], "-"))
    classify(f"aai16/{NODE}/in/bogus", ("-", "-"))
    classify("aai16/other/in/config", ("-", "-"))                       # someone else's command topic
    classify(f"aai16/{NODE}/out/reading/3", ("-", "-"))                  # our own telemetry is not an input
    classify("aai16/nodeA/out/reading/2", ("IN_REMOTE_READING", "nodeA"))
    classify("aai16/nodeA/out/reading/", ("-", "-"))
    classify("aai16/nodeA/out/heartbeat", ("-", "-"))
    classify("aai16//in/config", ("-", "-"))
    classify("other/node1/in/config", ("-", "-"))
    classify("aai16/" + "x" * 16 + "/in/config", ("-", "-"))               # id longer than 15 characters

    # node -> laptop: every topic the firmware publishes must be one the client subscribes to and can decode
    for suffix, kind in OUT_KIND.items():
        build(kind, None, b"", f"aai16/{NODE}/out/{suffix}", 1)
    reading = pb.Reading(sensor_id=5, valid=True).SerializeToString()
    build("OUT_READING", None, reading, f"aai16/{NODE}/out/reading/5", 0)
    build("OUT_READING", None, pb.Reading(sensor_id=200).SerializeToString(), None, None)      # virtual ids never leave the node
    build("OUT_READING", None, b"\x08", None, None)
    build("OUT_NODE_COMMAND", "nodeB", b"", "aai16/nodeB/in/command", 1)
    build("OUT_NODE_COMMAND", None, b"", None, None)

    r = subprocess.run([str(exe)], input="\n".join(requests) + "\n", capture_output=True, text=True, check=True)
    got = r.stdout.strip().splitlines()
    ok = len(got) == len(expect)
    for req, g, (kind, want) in zip(requests, got, expect):
        parts = g.split()
        if kind == "R":
            good = (parts[1], parts[2]) == want
        else:
            wt, wq = want
            good = (parts[1] == (wt or "-")) and (parts[2] == (str(wq) if wq is not None else "-"))
        ok &= good
        if not good:
            print(f"  FAIL {req!r} -> {g!r}  (expected {want})")
    print(f"  checked {len(requests)} topic cases against the client's topic list")

    # the client must be able to decode every message type the firmware emits
    for suffix in OUT_KIND:
        d = nc.decode(suffix, b"")
        ok &= d is not None
        if d is None:
            print(f"  FAIL client cannot decode out/{suffix}")
    ok &= nc.decode("reading/5", reading) is not None
    print("TOPICS AGREE (firmware <-> client)" if ok else "TOPIC MISMATCH")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
