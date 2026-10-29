"""Real Python CLI -> real C++ store/INT8 validator; no broker or board required."""
import itertools
import base64
import json
import os
from pathlib import Path
import subprocess
import socket
import sys
import threading
import types
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/protocol"))
from model_cli import Uploader, MqttTransport, CONTROL, pb
from model_package import package_model, validate_package

BINARY = ROOT / "build/motion_firmware_host/test_upload.exe"
if os.name != "nt":
    BINARY = BINARY.with_suffix("")
PACKAGE_PATH = ROOT / "build/model_b_package/model.package"


class Bridge:
    def __init__(self):
        self.proc = subprocess.Popen([str(BINARY), str(PACKAGE_PATH), "--bridge"], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, text=True)
        self.trace = []
        self.duplicate = False
        self.reboot_chunk = False
        self.reboot_finish = False

    def close(self):
        self.proc.stdin.close()
        self.proc.wait(timeout=10)
        self.proc.stdout.close()
        assert self.proc.returncode == 0

    def line(self, line):
        self.proc.stdin.write(line + "\n")
        self.proc.stdin.flush()
        answer = self.proc.stdout.readline().strip()
        if not answer:
            raise RuntimeError("C++ storage bridge terminated")
        return answer

    def exchange(self, request, topic, body):
        if self.reboot_chunk and request.WhichOneof("body") == "model_upload_chunk":
            self.reboot_chunk = False
            assert self.line("REBOOT") == "OK"
        wire = request.SerializeToString(deterministic=True)
        self.trace.append(wire)
        answer = self.line(wire.hex())
        if self.duplicate:
            # MQTT QoS1 redelivery / application ACK lost: the same request is
            # processed twice by the real C++ store before Python sees its ACK.
            self.trace.append(wire)
            answer = self.line(wire.hex())
        if self.reboot_finish and request.WhichOneof("body") == "model_control" and request.model_control.operation == pb.MODEL_OP_FINISH_UPLOAD:
            self.reboot_finish = False
            assert self.line("REBOOT") == "OK"
        return pb.Envelope.FromString(bytes.fromhex(answer))


class UploadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package, cls.manifest = package_model(ROOT / "models/motion_model_b")
        PACKAGE_PATH.parent.mkdir(parents=True, exist_ok=True)
        PACKAGE_PATH.write_bytes(cls.package)

    def bridge(self):
        bridge = Bridge()
        self.addCleanup(bridge.close)
        return bridge

    def uploader(self, bridge):
        return Uploader(bridge, itertools.count(1000).__next__, progress=lambda unused: None)

    def test_actual_b_package_metadata(self):
        validate_package(self.package, self.manifest)
        self.assertEqual(len(self.package), 3920)
        self.assertEqual(self.manifest["model_sha256"], "6ec06da5b5fe3d12fdd99328be1e8adf10bc3d78f3c7191bcb06dcd2f5279aff")
        self.assertEqual(self.package[256:2640], (ROOT / "models/motion_model_b/model_int8.tflite").read_bytes())

    def test_bad_package_sha_size_identity(self):
        for key, value in (("package_sha256", "00" * 32), ("model_size", 65537), ("package_size", 73729),
                           ("version", "different"), ("family", "A"), ("compatibility_profile", 2)):
            m = dict(self.manifest, **{key: value})
            with self.assertRaises(ValueError):
                validate_package(self.package, m)
        with self.assertRaises(ValueError):
            validate_package(self.package[:-1], self.manifest)

    def test_upload_real_store(self):
        b = self.bridge()
        final = self.uploader(b).upload(self.package, self.manifest)
        self.assertEqual(final.model_status.active.family, pb.MODEL_A)
        self.assertEqual(final.model_status.stored[0].family, pb.MODEL_B)
        frames = [pb.Envelope.FromString(w) for w in b.trace]
        chunks = [e.model_upload_chunk for e in frames if e.WhichOneof("body") == "model_upload_chunk"]
        self.assertEqual(b"".join(c.data for c in chunks), self.package)
        self.assertEqual(len({e.request_id for e in frames}), len(frames))

    def test_mqttx_short_commands_to_real_runtime(self):
        b = self.bridge()
        uploader = self.uploader(b)
        uploader.upload(self.package, self.manifest)
        # Real MQTTX Function -> protobufjs -> real C codec/store/TFLM runtime.
        command = ['node', str(ROOT / 'tests/protocol_client/mqttx_model_test.cjs')]
        installed = ROOT / 'build/protocol_client/mqttx_runtime/node_modules/vm2'
        if installed.exists():
            command.append(str(installed))
        fixtures = json.loads(subprocess.check_output(command, text=True))['simple']
        for fixture, family, epoch in zip(fixtures, (pb.MODEL_A, pb.MODEL_A, pb.MODEL_B, pb.MODEL_A, pb.MODEL_B), (0, 0, 1, 2, 3)):
            request = pb.Envelope.FromString(base64.b64decode(fixture['wire']))
            reply = b.exchange(request, CONTROL, 'model_status')
            self.assertEqual(reply.request_id, request.request_id)
            self.assertEqual(reply.model_status.result, pb.RESULT_OK)
            self.assertEqual(reply.model_status.active.family, family)
            self.assertEqual(reply.model_status.model_epoch, epoch)
            duplicate = b.exchange(request, CONTROL, 'model_status')
            self.assertEqual(duplicate, reply)
        self.assertEqual(uploader.status().model_status.model_epoch, 3)

    def test_selection_a_uses_bundled_provider_even_with_stored_a(self):
        b = self.bridge(); uploader = self.uploader(b)
        a_package, a_manifest = package_model(ROOT / 'models/motion_model_a', 'a')
        uploader.upload(a_package, a_manifest)
        uploader.upload(self.package, self.manifest)
        uploader.change('b')
        request = uploader.request()
        request.model_selection.operation, request.model_selection.family = pb.MODEL_OP_ACTIVATE, pb.MODEL_A
        result = b.exchange(request, CONTROL, 'model_status').model_status
        self.assertEqual(result.result, pb.RESULT_OK)
        self.assertEqual(result.active.source, pb.SOURCE_BUNDLED)
        self.assertEqual(result.active.sha256.hex(), a_manifest['model_sha256'])

    def test_activation_rollback_and_exact_retry(self):
        b = self.bridge()
        uploader = self.uploader(b)
        uploader.upload(self.package, self.manifest)
        b.duplicate = True
        for i in range(5):
            active = uploader.change("b").model_status
            self.assertEqual(active.active.source, pb.SOURCE_STORED)
            self.assertEqual(active.model_epoch, 2*i+1)
            back = uploader.change("a").model_status
            self.assertEqual(back.active.source, pb.SOURCE_BUNDLED)
            self.assertEqual(back.model_epoch, 2*i+2)
        self.assertEqual(uploader.change().model_status.active.family, pb.MODEL_B)
        self.assertEqual(b.line("REBOOT"), "OK")
        recovered = uploader.status().model_status
        self.assertEqual(recovered.active.family, pb.MODEL_A)
        self.assertEqual(recovered.model_epoch, 0)
        self.assertEqual(len(recovered.stored), 1)

    def test_mqttx_prepare_does_not_send_activation(self):
        b = self.bridge()
        uploader = self.uploader(b)
        uploader.upload(self.package, self.manifest)
        before = len(b.trace)
        with tempfile.TemporaryDirectory(dir=ROOT / "build") as temp:
            request = uploader.change("b", mqttx_dir=Path(temp))
            self.assertEqual(len(b.trace), before+1)  # GET_STATUS only
            saved = Path(temp) / f"{request.request_id}.pb"
            self.assertEqual(saved.read_bytes(), request.SerializeToString(deterministic=True))
            self.assertEqual(bytes.fromhex(saved.with_suffix('.hex').read_text()), saved.read_bytes())
            self.assertEqual(uploader.status().model_status.active.family, pb.MODEL_A)
            reply = b.exchange(request, CONTROL, "model_status")
            self.assertEqual(reply.model_status.active.family, pb.MODEL_B)

    def test_stale_activation_reboot_and_multiple_banks(self):
        b = self.bridge()
        uploader = self.uploader(b)
        uploader.upload(self.package, self.manifest)
        request = uploader.prepare_change("b")
        self.assertEqual(b.line("REBOOT"), "OK")
        reply = b.exchange(request, CONTROL, "model_status")
        self.assertEqual(reply.model_status.result, pb.RESULT_STALE_GENERATION)
        uploader.upload(self.package, self.manifest)
        with self.assertRaises(ValueError):
            uploader.prepare_change("b")
        self.assertEqual(uploader.change("b", bank=0).model_status.active.bank, 0)

    def test_duplicate_requests_after_ack_loss(self):
        b = self.bridge()
        b.duplicate = True
        final = self.uploader(b).upload(self.package, self.manifest)
        self.assertEqual(len(final.model_status.stored), 1)
        for first, second in zip(b.trace[::2], b.trace[1::2]):
            self.assertEqual(first, second)

    def test_both_families_two_banks_and_full_status(self):
        b = self.bridge()
        uploader = self.uploader(b)
        a, manifest_a = package_model(ROOT / "models/motion_model_a", "a")
        for package, manifest in ((a, manifest_a), (self.package, self.manifest)) * 2:
            final = uploader.upload(package, manifest)
        self.assertEqual(len(final.model_status.stored), 4)
        self.assertGreater(len(final.SerializeToString()), 320)
        self.assertLessEqual(len(final.SerializeToString()), 768)
        # Third B upload replaces its older bank, never the active bundled A.
        final = uploader.upload(self.package, self.manifest)
        self.assertEqual(len(final.model_status.stored), 4)
        self.assertEqual(final.model_status.active.family, pb.MODEL_A)

    def test_reboot_mid_upload_restarts(self):
        b = self.bridge()
        b.reboot_chunk = True
        final = self.uploader(b).upload(self.package, self.manifest)
        self.assertEqual(len(final.model_status.stored), 1)
        begins = [pb.Envelope.FromString(w).model_control for w in b.trace
                  if pb.Envelope.FromString(w).model_control.operation == pb.MODEL_OP_BEGIN_UPLOAD]
        self.assertEqual(len(begins), 2)
        self.assertNotEqual(begins[0].transfer_id, begins[1].transfer_id)

    def test_reboot_after_commit_confirms_storage(self):
        b = self.bridge()
        b.reboot_finish = True
        final = self.uploader(b).upload(self.package, self.manifest)
        self.assertEqual(len(final.model_status.stored), 1)

    def test_firmware_rejects_model_a_output_quantization_for_b(self):
        import hashlib
        import struct
        data = bytearray(self.package)
        struct.pack_into("<fi", data, 52, 0.046926043927669525, -12)
        m = dict(self.manifest, package_sha256=hashlib.sha256(data).hexdigest())
        with self.assertRaisesRegex(RuntimeError, "tensor_contract"):
            self.uploader(self.bridge()).upload(bytes(data), m)

    def test_transport_reconnect_retries_identical_bytes(self):
        args = types.SimpleNamespace(timeout=0.01, retries=2, tls=False, ca_file=None)
        transport = MqttTransport(args)
        bridge = self.bridge()
        request = self.uploader(bridge).request()
        request.model_control.operation = pb.MODEL_OP_GET_STATUS
        wires = []

        def publish(topic, wire, qos, retain):
            self.assertEqual((qos, retain), (1, False))
            wires.append(wire)
            answer = bridge.exchange(pb.Envelope.FromString(wire), topic, "model_status")
            if len(wires) == 1:
                transport.client.on_disconnect(None, None, None, None, None)
                self.assertFalse(transport.ready.is_set())
                # Reconnect/resubscribe completes before next retry. First
                # application ACK was lost across the disconnect.
                transport.ready.set()
                return types.SimpleNamespace(rc=transport.mqtt.MQTT_ERR_NO_CONN)
            transport.responses.put(answer)
            return types.SimpleNamespace(rc=transport.mqtt.MQTT_ERR_SUCCESS)

        transport.client.publish = publish
        transport.ready.set()
        answer = transport.exchange(request, CONTROL, "model_status")
        self.assertEqual(answer.request_id, request.request_id)
        self.assertEqual(len(wires), 2)
        self.assertEqual(wires[0], wires[1])

    def test_transport_timeout_is_bounded(self):
        args = types.SimpleNamespace(timeout=0.001, retries=1, tls=False, ca_file=None)
        transport = MqttTransport(args)
        request = pb.Envelope(protocol_version=1, request_id=1)
        with self.assertRaisesRegex(RuntimeError, "No matching application reply"):
            transport.exchange(request, CONTROL, "model_status")

    def mqtt_loopback(self, disconnect, operation="upload"):
        # A local MQTT 3.1.1 peer exercises the real CLI/Paho callbacks and
        # binary wire transport. Its device endpoint is the actual C++ store.
        # No production broker, credentials, ESP SDK flash, or board involved.
        bridge = self.bridge()
        errors, packets = [], []
        dropped = []
        if operation != "upload":
            uploader = self.uploader(bridge)
            uploader.upload(self.package, self.manifest)
            if operation in ("activate_a", "rollback"):
                uploader.change("b")

        def packet(header, body):
            n, encoded = len(body), bytearray([header])
            while True:
                digit = n % 128
                n //= 128
                encoded.append(digit | (128 if n else 0))
                if not n:
                    return bytes(encoded) + body

        def receive(connection):
            def exact(n):
                value = b""
                while len(value) < n:
                    chunk = connection.recv(n - len(value))
                    if not chunk:
                        raise EOFError("MQTT EOF")
                    value += chunk
                return value
            header = exact(1)[0]
            length, multiplier = 0, 1
            for _ in range(4):
                byte = exact(1)[0]
                length += (byte & 127) * multiplier
                if byte < 128:
                    break
                multiplier *= 128
            if length > 2048:
                raise ValueError("Unexpected large local test packet")
            return header, exact(length)

        with socket.socket() as listener:
            listener.bind(("127.0.0.1", 0))
            listener.listen()
            listener.settimeout(15)
            port = listener.getsockname()[1]

            def server():
                try:
                    finished = False
                    while not finished:
                        with listener.accept()[0] as connection:
                            connection.settimeout(15)
                            header, _ = receive(connection)
                            self.assertEqual(header >> 4, 1)
                            connection.sendall(b"\x20\x02\x00\x00")
                            while True:
                                header, body = receive(connection)
                                kind = header >> 4
                                if kind == 8:
                                    connection.sendall(b"\x90\x04" + body[:2] + b"\x01\x01")
                                elif kind == 4:
                                    continue
                                elif kind == 12:
                                    connection.sendall(b"\xd0\x00")
                                elif kind == 14:
                                    finished = True
                                    break
                                elif kind == 3:
                                    self.assertEqual(header & 7, 2) # QoS1, retain=0
                                    length = int.from_bytes(body[:2], "big")
                                    topic = body[2:2 + length].decode()
                                    mid = body[2 + length:4 + length]
                                    wire = body[4 + length:]
                                    packets.append(wire)
                                    request = pb.Envelope.FromString(wire)
                                    response = bridge.exchange(request, topic, "unused")
                                    mutation = request.WhichOneof("body") == "model_upload_chunk" if operation == "upload" else (
                                        request.WhichOneof("body") == "model_control" and request.model_control.operation in (pb.MODEL_OP_ACTIVATE, pb.MODEL_OP_ROLLBACK))
                                    if disconnect and not dropped and mutation:
                                        dropped.append(wire)
                                        break # Drop TCP after flash write, before any ACK.
                                    connection.sendall(b"\x40\x02" + mid)
                                    reply_topic = b"aai16/model/status" if response.WhichOneof("body") == "model_status" else b"aai16/model/upload/ack"
                                    body = len(reply_topic).to_bytes(2, "big") + reply_topic + b"\x00\x01" + response.SerializeToString()
                                    connection.sendall(packet(0x32, body))
                                else:
                                    raise AssertionError(f"Unexpected MQTT kind {kind}")
                except Exception as error:
                    errors.append(error)

            thread = threading.Thread(target=server, daemon=True)
            thread.start()
            env = dict(os.environ)
            env.pop("AAI16_MQTT_USERNAME", None)
            env.pop("AAI16_MQTT_PASSWORD", None)
            command = ["upload", "--model-dir", str(ROOT / "models/motion_model_b"), "--output-dir", str(ROOT / "build/model_upload_client")] if operation == "upload" else (
                ["rollback"] if operation == "rollback" else ["activate", "a" if operation == "activate_a" else "b"])
            run = subprocess.run([sys.executable, str(ROOT / "tools/protocol/config_cli.py"), "model"] + command + [
                "--id-state", str(ROOT / "build/model_upload_client/request_ids.json"),
                "--host", "127.0.0.1", "--port", str(port), "--timeout", "3", "--retries", "3"],
                capture_output=True, text=True, timeout=30, env=env)
            thread.join(timeout=16)
            self.assertFalse(thread.is_alive())
            if errors:
                raise errors[0]
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            if operation == "upload":
                self.assertIn("STORED confirmed:", run.stdout)
                self.assertIn("NOT activated", run.stdout)
            else:
                self.assertIn("RESULT_OK", run.stdout)
                self.assertIn("active=aai16-motion-model-" + ("b" if operation == "activate_b" else "a"), run.stdout)
            if disconnect:
                self.assertEqual(len(dropped), 1)
                self.assertGreaterEqual(packets.count(dropped[0]), 2)

    def test_real_mqtt_cli_loopback(self):
        self.mqtt_loopback(False)

    def test_real_mqtt_disconnect_after_flash_write(self):
        self.mqtt_loopback(True)

    def test_real_mqtt_activation_after_lost_receipt(self):
        self.mqtt_loopback(True, "activate_b")

    def test_real_mqtt_bundled_a(self):
        self.mqtt_loopback(False, "activate_a")

    def test_real_mqtt_rollback(self):
        self.mqtt_loopback(False, "rollback")


if __name__ == "__main__":
    unittest.main(verbosity=2)
