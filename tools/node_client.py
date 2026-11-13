"""Laptop-side client for an AAI16 edge node (talks MQTT + binary Protobuf; no JSON anywhere).

  python tools/node_client.py --broker 192.168.1.10 --node node1 config A --gen 1
  python tools/node_client.py ... config FILE.bin          # a ConfigApply you built yourself
  python tools/node_client.py ... rules FILE.bin | bridge FILE.bin
  python tools/node_client.py ... command run|stop|clear
  python tools/node_client.py ... upload ml/artifacts/motion/motion.amdl --kind motion --transfer 1 [--chunk 512]
  python tools/node_client.py ... activate --kind motion --transfer 1 --gen 1
  python tools/node_client.py ... inputs --kind motion --gen 2  1:0:0 1:1:1 1:2:2     # sensor:value_index:slot
  python tools/node_client.py ... listen [--seconds 60]
  python tools/node_client.py selftest                     # offline checks (no broker needed)

Topics: aai16/<node>/in/{config,rules,bridge,model/chunk,model/control,command}; replies on aai16/<node>/out/...
Tested against a real local broker with a scripted MOCK node (tests/client_mqtt); not yet against the real firmware over Wi-Fi.
"""
import argparse
import hashlib
import pathlib
import sys
import threading
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from profile_lib import load_pb  # noqa: E402

pb = load_pb()
KINDS = {"motion": pb.MODEL_MOTION, "environment": pb.MODEL_ENVIRONMENT, "anomaly": pb.MODEL_ANOMALY}

# Topic scheme - must match components/node_core/node_topics.c (tests/topics_host checks both ways)
IN_TOPICS = ("config", "rules", "bridge", "model/chunk", "model/control", "command")
OUT_TOPICS = ("config_ack", "rules_ack", "bridge_ack", "model_chunk_ack", "model_status", "inference", "heartbeat", "notification")
MAX_CHUNK = 1024


def make_chunks(package: bytes, kind: int, transfer: int, chunk: int = 512, version: int = 1, generation=None):
    """Split a model package into serialized ModelChunk messages (bounded <= 1024 payload bytes)."""
    if not 1 <= chunk <= MAX_CHUNK:
        raise ValueError(f"chunk size must be 1..{MAX_CHUNK}")
    digest = hashlib.sha256(package).digest()
    total = (len(package) + chunk - 1) // chunk
    out = []
    for seq in range(total):
        m = pb.ModelChunk(transfer_id=transfer, generation=generation or transfer, kind=kind, format=pb.FORMAT_TFLITE,
                          model_version=version, seq=seq, total_chunks=total, total_size=len(package), sha256=digest,
                          data=package[seq * chunk:(seq + 1) * chunk])
        out.append(m.SerializeToString())
    return out


def package_version(package: bytes) -> int:
    n = int.from_bytes(package[4:8], "little")
    c = pb.ModelContract()
    c.ParseFromString(package[8:8 + n])
    return c.model_version


def control(generation, action, kind=0, transfer=0, inputs=()):
    m = pb.ModelControl(generation=generation, action=action, kind=kind, transfer_id=transfer)
    for sid, vi, slot in inputs:
        m.inputs.add(sensor_id=sid, value_index=vi, feature_slot=slot)
    return m.SerializeToString()


def command(name):
    return pb.NodeCommand(cmd={"run": pb.RCMD_RUN, "stop": pb.RCMD_STOP, "clear": pb.RCMD_CLEAR}[name]).SerializeToString()


class Node:
    def __init__(self, broker, node, port=1883):
        import paho.mqtt.client as mqtt
        self.node = node
        self.msgs = []
        self.mark = 0
        self.cv = threading.Condition()
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"aai16-client-{int(time.time())}")
        self.c.on_message = self._on_message
        self.c.connect(broker, port, 30)
        self.c.subscribe(f"aai16/{node}/out/#", qos=1)
        self.c.loop_start()

    def _on_message(self, _c, _u, msg):
        with self.cv:
            self.msgs.append((msg.topic, bytes(msg.payload)))
            self.cv.notify_all()

    def send(self, suffix, payload):
        with self.cv:
            self.mark = len(self.msgs)  # only replies that arrive AFTER this request may answer it
        self.c.publish(f"aai16/{self.node}/in/{suffix}", payload, qos=1).wait_for_publish()

    def wait(self, suffix, pred, timeout=10.0):
        """Wait for a message on aai16/<node>/out/<suffix> whose decoded form satisfies pred."""
        end = time.time() + timeout
        with self.cv:
            seen = self.mark
            while time.time() < end:
                while seen < len(self.msgs):
                    topic, data = self.msgs[seen]
                    seen += 1
                    if topic.endswith("/out/" + suffix):
                        obj = decode(suffix, data)
                        if pred(obj):
                            return obj
                self.cv.wait(0.2)
        return None

    def upload(self, package, kind, transfer, chunk=512, retries=3):
        chunks = make_chunks(package, kind, transfer, chunk, package_version(package))
        i = 0
        while i < len(chunks):
            for _ in range(retries):
                self.send("model/chunk", chunks[i])
                ack = self.wait("model_chunk_ack", lambda a: a.transfer_id == transfer, 5.0)
                if ack is not None:
                    break
            else:
                raise RuntimeError(f"no ChunkAck for chunk {i}")
            name = pb.ChunkStatus.Name(ack.status)
            if ack.status in (pb.CHUNK_OK, pb.CHUNK_DUPLICATE):
                i = max(i + 1, ack.next_expected) if ack.status == pb.CHUNK_DUPLICATE else i + 1
            elif ack.status == pb.CHUNK_OUT_OF_ORDER:
                i = ack.next_expected
            else:
                raise RuntimeError(f"chunk {i} refused: {name}")
        return len(chunks)


def decode(suffix, data):
    cls = {"config_ack": pb.ApplyAck, "rules_ack": pb.ApplyAck, "bridge_ack": pb.ApplyAck, "model_chunk_ack": pb.ChunkAck,
           "model_status": pb.ModelStatus, "inference": pb.InferenceResult, "heartbeat": pb.Heartbeat,
           "notification": pb.Notification}.get(suffix.split("/")[0])
    if suffix.startswith("reading"):
        cls = pb.Reading
    if cls is None:
        return None
    m = cls()
    m.ParseFromString(data)
    return m


def selftest():
    """Offline: chunking round-trips, digests match, bounds enforced, messages parse."""
    ok = True

    def check(name, cond):
        nonlocal ok
        ok &= bool(cond)
        print(f"  {'ok  ' if cond else 'FAIL'} {name}")

    for task in ("motion", "environment"):
        path = ROOT / "ml" / "artifacts" / task / f"{task}.amdl"
        if not path.exists():
            print(f"  skip {task}: train the models first")
            continue
        pkg = path.read_bytes()
        for size in (256, 512, 1024):
            chunks = make_chunks(pkg, KINDS[task], 7, size, package_version(pkg))
            msgs = [pb.ModelChunk.FromString(c) for c in chunks]
            joined = b"".join(m.data for m in msgs)
            check(f"{task} @ {size}: {len(chunks)} chunks reassemble to the package, sha256 matches",
                  joined == pkg and all(m.sha256 == hashlib.sha256(pkg).digest() for m in msgs)
                  and [m.seq for m in msgs] == list(range(len(msgs))) and max(len(m.data) for m in msgs) <= size)
    try:
        make_chunks(b"x" * 10, pb.MODEL_MOTION, 1, 2048)
        check("chunk size above 1024 is refused", False)
    except ValueError:
        check("chunk size above 1024 is refused", True)
    c = pb.ModelControl.FromString(control(5, pb.ACTION_SET_INPUTS, pb.MODEL_MOTION, 0, [(1, 0, 0), (1, 1, 1), (1, 2, 2)]))
    check("SET_INPUTS message carries 3 inputs with the right slots", [(i.sensor_id, i.value_index, i.feature_slot) for i in c.inputs] == [(1, 0, 0), (1, 1, 1), (1, 2, 2)] and c.generation == 5)
    check("command messages", pb.NodeCommand.FromString(command("stop")).cmd == pb.RCMD_STOP)
    return 0 if ok else 1


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        sys.exit(selftest())
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", required=True)
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--node", required=True)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("config"); p.add_argument("which"); p.add_argument("--gen", type=int, default=1)
    for n in ("rules", "bridge"):
        sub.add_parser(n).add_argument("file")
    sub.add_parser("command").add_argument("name", choices=["run", "stop", "clear"])
    p = sub.add_parser("upload"); p.add_argument("file"); p.add_argument("--kind", required=True, choices=KINDS)
    p.add_argument("--transfer", type=int, required=True); p.add_argument("--chunk", type=int, default=512)
    p = sub.add_parser("activate"); p.add_argument("--kind", required=True, choices=KINDS)
    p.add_argument("--transfer", type=int, required=True); p.add_argument("--gen", type=int, required=True)
    p = sub.add_parser("inputs"); p.add_argument("--kind", required=True, choices=KINDS)
    p.add_argument("--gen", type=int, required=True); p.add_argument("map", nargs="+")
    sub.add_parser("listen").add_argument("--seconds", type=float, default=60)
    a = ap.parse_args()

    n = Node(a.broker, a.node, a.port)
    if a.cmd == "config":
        import sensor_profiles as sp
        if a.which.upper() in ("A", "B"):
            payload = (sp.config_a if a.which.upper() == "A" else sp.config_b)(pb, a.gen).SerializeToString()
        else:
            payload = pathlib.Path(a.which).read_bytes()
        n.send("config", payload)
        ack = n.wait("config_ack", lambda m: m.phase in (pb.PHASE_APPLIED, pb.PHASE_REJECTED), 15)
        print(ack if ack else "no ACK (node offline, or ACK lost)")
    elif a.cmd in ("rules", "bridge"):
        n.send(a.cmd, pathlib.Path(a.file).read_bytes())
        ack = n.wait(a.cmd + "_ack", lambda m: True, 10)
        print(ack if ack else "no ACK")
    elif a.cmd == "command":
        n.send("command", command(a.name))
        print("sent")
    elif a.cmd == "upload":
        pkg = pathlib.Path(a.file).read_bytes()
        t0 = time.time()
        k = n.upload(pkg, KINDS[a.kind], a.transfer, a.chunk)
        print(f"uploaded {len(pkg)} bytes in {k} chunks, {time.time() - t0:.2f} s (staged on the node)")
    elif a.cmd == "activate":
        n.send("model/control", control(a.gen, pb.ACTION_ACTIVATE, KINDS[a.kind], a.transfer))
        print(n.wait("model_status", lambda m: m.generation >= 0, 15) or "no status")
    elif a.cmd == "inputs":
        ins = [tuple(int(x) for x in s.split(":")) for s in a.map]
        n.send("model/control", control(a.gen, pb.ACTION_SET_INPUTS, KINDS[a.kind], 0, ins))
        print(n.wait("model_status", lambda m: True, 15) or "no status")
    elif a.cmd == "listen":
        end = time.time() + a.seconds
        seen = 0
        while time.time() < end:
            with n.cv:
                n.cv.wait(0.5)
                batch, seen = n.msgs[seen:], len(n.msgs)
            for topic, data in batch:
                suffix = topic.split("/out/", 1)[-1]
                print(topic, "->", str(decode(suffix, data)).replace("\n", " "))


if __name__ == "__main__":
    main()
