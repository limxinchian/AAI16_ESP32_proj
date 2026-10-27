"""Optional real Paho/socket checks against a tiny loopback MQTT 3.1.1 broker.

No external broker, credentials, ESP32 or measured dataset. Run with the pinned
requirements installed; otherwise these tests skip explicitly.
"""
import contextlib
import csv
import importlib.util
import io
import json
from pathlib import Path
import socket
import tempfile
import threading
import unittest
from unittest.mock import patch

import collector
from test_collector import payload


def frame(kind, body):
    remaining = len(body)
    encoded = bytearray([kind])
    while True:
        digit = remaining % 128
        remaining //= 128
        encoded.append(digit | (128 if remaining else 0))
        if not remaining:
            break
    return bytes(encoded) + body


def receive(sock):
    def exact(count):
        data = b""
        while len(data) < count:
            chunk = sock.recv(count - len(data))
            if not chunk:
                raise EOFError
            data += chunk
        return data
    kind = exact(1)[0]
    size, multiplier = 0, 1
    for _ in range(4):
        byte = exact(1)[0]
        size += (byte & 127) * multiplier
        if not byte & 128:
            return kind, exact(size)
        multiplier *= 128
    raise ValueError("invalid MQTT frame")


@unittest.skipUnless(importlib.util.find_spec("paho"), "install paho-mqtt for socket integration tests")
class LoopbackTests(unittest.TestCase):
    def record(self, disconnect=False):
        errors = []
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        listener.settimeout(5)
        port = listener.getsockname()[1]

        def broker():
            try:
                with listener.accept()[0] as client:
                    client.settimeout(5)
                    kind, _ = receive(client)
                    assert kind == 0x10  # CONNECT
                    client.sendall(frame(0x20, b"\x00\x00"))
                    kind, body = receive(client)
                    assert kind == 0x82 and collector.TOPIC.encode() in body
                    client.sendall(frame(0x90, body[:2] + b"\x01"))
                    topic = collector.TOPIC.encode()
                    topic_prefix = len(topic).to_bytes(2, "big") + topic
                    messages = [(payload(0, 900), True), (payload(), False),
                                (payload(), False), (payload(3, 1300), False),
                                (payload(2, 1100), False), (b"invalid JSON", False),
                                (payload(4, 1400), False)]
                    for data, retained in messages:
                        client.sendall(frame(0x31 if retained else 0x30, topic_prefix + data))
                    if disconnect:
                        return
                    kind, _ = receive(client)
                    assert kind == 0xE0  # DISCONNECT
            except Exception as exc:
                errors.append(exc)
            finally:
                listener.close()

        thread = threading.Thread(target=broker, daemon=True)
        thread.start()
        with tempfile.TemporaryDirectory() as directory:
            with patch.dict(collector.os.environ, {}, clear=True), contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                result = collector.main(["--host", "127.0.0.1", "--port", str(port),
                                         "--label", "REST", "--duration", "0.4",
                                         "--session-id", "loopback", "--output-dir", directory])
            thread.join(timeout=6)
            self.assertFalse(thread.is_alive())
            self.assertEqual(errors, [])
            summary = json.loads((Path(directory) / "loopback.summary.json").read_text())
            with (Path(directory) / "loopback.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            return result, summary, rows

    def test_real_paho_qos0_receive_and_csv(self):
        result, summary, rows = self.record()
        self.assertEqual(result, 0)
        self.assertEqual([row["seq"] for row in rows], ["1", "3", "4"])
        self.assertEqual([row["timestamp_ms"] for row in rows], ["1000", "1300", "1400"])
        self.assertEqual((summary["received"], summary["accepted"], summary["dropped"]), (7, 3, 4))
        for key in ("retained", "duplicates", "out_of_order", "invalid", "sequence_gaps", "excessive_gaps"):
            self.assertEqual(summary[key], 1, key)

    def test_real_paho_disconnect_preserves_partial_session(self):
        result, summary, rows = self.record(disconnect=True)
        self.assertEqual(result, 1)
        self.assertIn("disconnected", summary["stop_reason"])
        self.assertEqual(summary["received"], summary["accepted"] + summary["dropped"])


if __name__ == "__main__":
    unittest.main()
