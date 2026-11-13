"""tools/node_client.py against a REAL MQTT broker (amqtt, in-process) and a scripted MOCK NODE.

The mock node is a few lines of Python that answer on the node's topics; it is NOT the firmware. What this proves is
the client side: paho usage, topic names, Protobuf encoding, ACK waiting, retry after a lost ACK, recovery from an
out-of-order reply, and that the exact package bytes arrive. Real-node behaviour is covered by tests/node_host.

Run:  .venv\\Scripts\\python.exe tests\\client_mqtt\\run.py
"""
import asyncio
import hashlib
import os
import pathlib
import sys
import threading
import time

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import node_client as nc  # noqa: E402

pb = nc.pb
PORT = 18830
NODE = "mocknode"


def start_broker():
    from amqtt.broker import Broker
    config = {"listeners": {"default": {"type": "tcp", "bind": f"127.0.0.1:{PORT}"}}, "sys_interval": 0,
              "auth": {"allow-anonymous": True, "plugins": ["auth_anonymous"]},
              "topic-check": {"enabled": False}}
    loop = asyncio.new_event_loop()
    started = threading.Event()
    holder = {}

    async def boot():
        holder["broker"] = Broker(config)   # must be created inside the running loop
        await holder["broker"].start()
        started.set()

    def run():
        asyncio.set_event_loop(loop)
        loop.create_task(boot())
        loop.run_forever()

    th = threading.Thread(target=run)
    th.start()
    if not started.wait(15):
        raise RuntimeError("broker did not start")

    def stop():
        try:
            asyncio.run_coroutine_threadsafe(holder["broker"].shutdown(), loop).result(5)
        except Exception:
            pass
        loop.call_soon_threadsafe(loop.stop)
        th.join(5)

    return stop


class MockNode:
    """Answers like the node would, with two scripted faults on the chunk path."""

    def __init__(self):
        import paho.mqtt.client as mqtt
        self.got = {"config": [], "chunks": [], "control": [], "command": []}
        self.expected = 0
        self.package = b""
        self.dropped_ack = False
        self.forced_ooo = False
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="mock-node")
        self.c.on_message = self.on_message
        self.c.connect("127.0.0.1", PORT, 30)
        self.c.subscribe(f"aai16/{NODE}/in/#", qos=1)
        self.c.loop_start()

    def out(self, suffix, msg):
        self.c.publish(f"aai16/{NODE}/out/{suffix}", msg.SerializeToString(), qos=1)

    def on_message(self, _c, _u, m):
        kind = m.topic.split("/in/", 1)[1]
        if kind == "config":
            req = pb.ConfigApply.FromString(m.payload)
            self.got["config"].append(req)
            self.out("config_ack", pb.ApplyAck(generation=req.generation, phase=pb.PHASE_ADMITTED))
            self.out("config_ack", pb.ApplyAck(generation=req.generation, phase=pb.PHASE_APPLIED))
        elif kind == "command":
            self.got["command"].append(pb.NodeCommand.FromString(m.payload).cmd)
        elif kind == "model/control":
            c = pb.ModelControl.FromString(m.payload)
            self.got["control"].append(c)
            self.out("model_status", pb.ModelStatus(generation=c.generation, kind=c.kind, state=pb.MODEL_STATE_SUSPENDED,
                                                    reason=pb.MODEL_REASON_NO_INPUTS))
        elif kind == "model/chunk":
            ch = pb.ModelChunk.FromString(m.payload)
            self.got["chunks"].append(ch.seq)
            if ch.seq == 2 and not self.forced_ooo:       # fault 1: pretend chunk 1 never arrived
                self.forced_ooo = True
                self.out("model_chunk_ack", pb.ChunkAck(transfer_id=ch.transfer_id, seq=ch.seq, status=pb.CHUNK_OUT_OF_ORDER, next_expected=1))
                return
            if ch.seq < self.expected:
                self.out("model_chunk_ack", pb.ChunkAck(transfer_id=ch.transfer_id, seq=ch.seq, status=pb.CHUNK_DUPLICATE, next_expected=self.expected))
                return
            if ch.seq > self.expected:
                self.out("model_chunk_ack", pb.ChunkAck(transfer_id=ch.transfer_id, seq=ch.seq, status=pb.CHUNK_OUT_OF_ORDER, next_expected=self.expected))
                return
            self.package += ch.data
            self.expected += 1
            if ch.seq == 3 and not self.dropped_ack:       # fault 2: the ACK for chunk 3 is lost
                self.dropped_ack = True
                return
            self.out("model_chunk_ack", pb.ChunkAck(transfer_id=ch.transfer_id, seq=ch.seq, status=pb.CHUNK_OK, next_expected=self.expected))


def main():
    pkg_path = ROOT / "ml" / "artifacts" / "motion" / "motion.amdl"
    if not pkg_path.exists():
        sys.exit("train the models first (ml/train_motion.py)")
    pkg = pkg_path.read_bytes()
    stop_broker = start_broker()
    ok = True
    mock = client = None

    def check(name, cond, detail=""):
        nonlocal ok
        ok &= bool(cond)
        print(f"  {'ok  ' if cond else 'FAIL'} {name}" + (f"   [{detail}]" if detail else ""))

    try:
        mock = MockNode()
        time.sleep(0.5)
        client = nc.Node("127.0.0.1", NODE, PORT)
        time.sleep(0.5)

        import sensor_profiles as sp
        client.send("config", sp.config_a(pb, 1).SerializeToString())
        ack = client.wait("config_ack", lambda m: m.phase in (pb.PHASE_APPLIED, pb.PHASE_REJECTED), 10)
        check("config A: client publishes, receives APPLIED", ack is not None and ack.phase == pb.PHASE_APPLIED and ack.generation == 1)
        check("mock node saw a decodable ConfigApply with 4 profiles", len(mock.got["config"]) == 1 and len(mock.got["config"][0].profiles) == 4)

        client.send("command", nc.command("stop"))
        time.sleep(0.5)
        check("command STOP delivered", mock.got["command"] == [pb.RCMD_STOP])

        n = client.upload(pkg, pb.MODEL_MOTION, 5, 256)
        check(f"upload of {len(pkg)} B in {n} chunks completes despite an out-of-order reply and a lost ACK", n > 5)
        check("the node received exactly the package bytes", mock.package == pkg and hashlib.sha256(mock.package).digest() == hashlib.sha256(pkg).digest())
        check("client resent chunks (more messages than chunks)", len(mock.got["chunks"]) > n, f"{len(mock.got['chunks'])} sent for {n} chunks")

        client.send("model/control", nc.control(1, pb.ACTION_ACTIVATE, pb.MODEL_MOTION, 5))
        st = client.wait("model_status", lambda m: True, 10)
        check("ACTIVATE reaches the node and its status comes back", st is not None and st.reason == pb.MODEL_REASON_NO_INPUTS and mock.got["control"][0].transfer_id == 5)
        client.send("model/control", nc.control(2, pb.ACTION_SET_INPUTS, pb.MODEL_MOTION, 0, [(1, 0, 0), (1, 1, 1), (1, 2, 2)]))
        time.sleep(0.8)
        ins = mock.got["control"][-1].inputs
        check("SET_INPUTS carries the mapping", [(i.sensor_id, i.value_index, i.feature_slot) for i in ins] == [(1, 0, 0), (1, 1, 1), (1, 2, 2)])
    finally:
        for node in (client, mock):
            if node is not None:
                c = node.c
                c.disconnect()
                c.loop_stop()
        stop_broker()
    print("\nALL CLIENT/BROKER CHECKS PASSED" if ok else "\nCLIENT/BROKER CHECKS FAILED", flush=True)
    sys.stdout.flush()
    os._exit(0 if ok else 1)   # hard exit: skips interpreter teardown, which crashed with leftover library threads on Windows


if __name__ == "__main__":
    main()
