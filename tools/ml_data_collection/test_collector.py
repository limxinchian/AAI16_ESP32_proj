"""Offline tests; generated fixture values are NOT measured training data."""
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import collector as c


def payload(seq=1, uptime=1000, **updates):
    data = dict(sensor="gy511", boot_id="abcdef01", seq=seq, uptime_ms=uptime,
                acceleration_g=dict(x=0.123, y=-0.456, z=1.001), jitter_us=-123)
    data.update(updates)
    return json.dumps(data).encode()


class DecoderTests(unittest.TestCase):
    def test_current_schema_and_legacy(self):
        sample = c.decode_json(payload(magnetic_field_ut=dict(x=1, y=2, z=3)))
        self.assertEqual((sample.ax_g, sample.ay_g, sample.az_g), (0.123, -0.456, 1.001))
        self.assertEqual(sample.jitter_us, -123)
        data = json.loads(payload())
        del data["jitter_us"]
        self.assertIsNone(c.decode_json(json.dumps(data).encode()).jitter_us)

    def test_invalid_envelopes(self):
        for raw in (b"", b"null", b"[]", b"1", b"{", b"\xff", b"x" * 4097,
                    b'{"sensor":"gy511","sensor":"other"}',
                    b"[" * 1500 + b"]" * 1500):
            with self.subTest(raw=raw[:20]), self.assertRaises(ValueError):
                c.decode_json(raw)

    def test_invalid_identity_and_timestamps(self):
        for updates in (dict(sensor="dht22"), dict(boot_id=""), dict(boot_id=42),
                        dict(seq=True), dict(seq=-1), dict(seq=2**32), dict(seq=1.0),
                        dict(uptime_ms="100"), dict(uptime_ms=-1), dict(uptime_ms=True),
                        dict(jitter_us=None), dict(jitter_us=float("nan"))):
            with self.subTest(updates=updates), self.assertRaises(ValueError):
                c.decode_json(payload(**updates))

    def test_invalid_axes(self):
        for value in (True, None, "1.0", float("nan"), float("inf"), -float("inf"), 3, 10**400):
            with self.subTest(value=str(value)[:20]), self.assertRaises(ValueError):
                c.decode_json(payload(acceleration_g=dict(x=value, y=0, z=1)))
        for axes in (None, [], {}, dict(x=0, y=0)):
            with self.assertRaises(ValueError):
                c.decode_json(payload(acceleration_g=axes))

    def test_no_clipping_scaling_or_axis_mapping(self):
        sample = c.decode_json(payload(acceleration_g=dict(x=2.048, y=-2.048, z=0.999)))
        self.assertEqual((sample.ax_g, sample.ay_g, sample.az_g), (2.048, -2.048, 0.999))


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.stream = io.StringIO(newline="")
        self.recording = c.Recording(self.stream, "REST", "rest_orientation_1")

    def rows(self):
        return list(csv.DictReader(io.StringIO(self.stream.getvalue())))

    def test_schema_real_timestamps_session_and_label(self):
        self.recording.receive(payload())
        self.recording.receive(payload(2, 1107))
        rows = self.rows()
        self.assertEqual(tuple(rows[0]), c.CSV_FIELDS)
        self.assertEqual([r["timestamp_ms"] for r in rows], ["1000", "1107"])
        self.assertTrue(all(r["uptime_ms"] == r["timestamp_ms"] for r in rows))
        self.assertTrue(all(r["label"] == "REST" and r["session_id"] == "rest_orientation_1" for r in rows))
        self.assertEqual(rows[1]["ay_g"], "-0.456")
        self.assertEqual(self.recording.stats.min_delta_ms, 107)

    def test_loss_duplicate_late_and_gap_no_interpolation(self):
        for seq, uptime in ((1, 1000), (3, 1301), (3, 1301), (2, 1100), (4, 1402)):
            self.recording.receive(payload(seq, uptime))
        stats = self.recording.stats
        self.assertEqual((stats.received, stats.accepted, stats.dropped), (5, 3, 2))
        self.assertEqual((stats.sequence_gaps, stats.duplicates, stats.out_of_order), (1, 1, 1))
        self.assertEqual(stats.excessive_gaps, 1)
        self.assertEqual([r["seq"] for r in self.rows()], ["1", "3", "4"])

    def test_uint32_wrap(self):
        for seq, uptime in ((0xFFFFFFFF, 1000), (0, 1100), (1, 1200)):
            self.assertTrue(self.recording.receive(payload(seq, uptime)))
        self.assertEqual(self.recording.stats.sequence_gaps, 0)

    def test_bad_timestamp_does_not_advance_watermark(self):
        self.recording.receive(payload(1, 1000))
        self.assertFalse(self.recording.receive(payload(2, 1000)))
        self.assertFalse(self.recording.receive(payload(3, 900)))
        self.assertTrue(self.recording.receive(payload(2, 1100)))
        self.assertEqual(self.recording.stats.timestamp_errors, 2)

    def test_boot_change_stops_session(self):
        self.recording.receive(payload())
        with self.assertRaisesRegex(RuntimeError, "boot changed"):
            self.recording.receive(payload(1, 1, boot_id="12345678"))
        self.assertEqual(len(self.rows()), 1)
        self.assertEqual(self.recording.stats.boot_changes, 1)

    def test_retained_and_invalid_do_not_start_history(self):
        self.assertFalse(self.recording.receive(payload(), retained=True))
        self.assertFalse(self.recording.receive(b"invalid"))
        self.assertTrue(self.recording.receive(payload(99, 9999)))
        self.assertEqual(self.recording.stats.sequence_gaps, 0)

    def test_identical_values_are_accepted(self):
        self.assertTrue(self.recording.receive(payload(1, 1000)))
        self.assertTrue(self.recording.receive(payload(2, 1100)))
        self.assertEqual(self.recording.stats.duplicates, 0)

    def test_gap_boundary_and_legacy_metadata(self):
        data = json.loads(payload())
        del data["jitter_us"]
        self.recording.receive(json.dumps(data).encode())
        self.recording.receive(payload(2, 1150))
        self.assertEqual(self.recording.stats.excessive_gaps, 0)
        self.assertEqual(self.recording.stats.missing_jitter, 1)
        self.assertEqual(self.rows()[0]["jitter_us"], "")

    def test_explicit_label_and_safe_session(self):
        for label, session in (("AUTO", "s"), ("rest", "s"), ("REST", "../s"), ("MOVING", "=SUM(1)")):
            with self.assertRaises(ValueError):
                c.Recording(io.StringIO(), label, session)
        moving = c.Recording(io.StringIO(), "MOVING", "moving_1")
        self.assertEqual(moving.label, "MOVING")

    def test_decoder_can_be_replaced_without_csv_changes(self):
        sample = c.Sample("12345678", 42, 12345, 0.2, 0.3, 1.2, 0)
        output = io.StringIO()
        recording = c.Recording(output, "MOVING", "proto", decoder=lambda raw: sample)
        recording.receive(b"future protobuf bytes")
        self.assertEqual(tuple(next(csv.DictReader(io.StringIO(output.getvalue())))), c.CSV_FIELDS)

    def test_write_error_stops_without_accepting(self):
        class BrokenStream(io.StringIO):
            broken = False

            def flush(self):
                if self.broken:
                    raise OSError("disk full")

        output = BrokenStream()
        recording = c.Recording(output, "REST", "disk")
        output.broken = True
        with self.assertRaises(OSError):
            recording.receive(payload())
        self.assertEqual(recording.stats.accepted, 0)
        self.assertEqual(recording.stats.write_errors, 1)


class FakeClient:
    mode = "normal"

    def __init__(self, *args, **kwargs):
        self.stopped = False

    def username_pw_set(self, *args):
        pass

    def tls_set(self, **kwargs):
        pass

    def connect_async(self, *args, **kwargs):
        pass

    def subscribe(self, topic, qos):
        assert topic == c.TOPIC and qos == 1
        self.on_subscribe(self, None, 1, [SimpleNamespace(is_failure=False)], None)
        return 0, 1

    def loop_start(self):
        if self.mode == "interrupt":
            raise KeyboardInterrupt
        if self.mode == "connect_failure":
            self.on_connect_fail(self, None)
            return
        self.on_connect(self, None, None, SimpleNamespace(is_failure=False), None)
        if self.mode == "no_data":
            return
        for seq in (1, 2):
            self.on_message(self, None, SimpleNamespace(topic=c.TOPIC, payload=payload(seq, 9000 + seq * 103), retain=False))
        if self.mode == "disconnect":
            self.on_disconnect(self, None, None, None, None)

    def disconnect(self):
        self.stopped = True

    def loop_stop(self):
        pass


class CLITests(unittest.TestCase):
    def run_session(self, directory, mode="normal", session="cli"):
        mqtt = SimpleNamespace(Client=FakeClient, CallbackAPIVersion=SimpleNamespace(VERSION2=2), MQTT_ERR_SUCCESS=0)
        modules = {"paho": SimpleNamespace(mqtt=SimpleNamespace(client=mqtt)),
                   "paho.mqtt": SimpleNamespace(client=mqtt), "paho.mqtt.client": mqtt}
        FakeClient.mode = mode
        with patch.dict(sys.modules, modules), patch.dict(c.os.environ, {}, clear=True), patch("sys.stdout", io.StringIO()), patch("sys.stderr", io.StringIO()):
            return c.main(["--host", "test.invalid", "--label", "MOVING", "--duration", "0.01",
                           "--session-id", session, "--output-dir", directory])

    def test_recording_and_summary_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertEqual(self.run_session(directory), 0)
            path = Path(directory) / "cli.csv"
            before = path.read_bytes()
            rows = list(csv.DictReader(io.StringIO(before.decode())))
            self.assertEqual([r["timestamp_ms"] for r in rows], ["9103", "9206"])
            summary = json.loads((Path(directory) / "cli.summary.json").read_text())
            self.assertEqual((summary["received"], summary["accepted"], summary["dropped"]), (2, 2, 0))
            self.assertEqual(summary["label"], "MOVING")
            self.assertEqual(self.run_session(directory), 1)
            self.assertEqual(path.read_bytes(), before)

    def test_disconnect_empty_interrupt_and_connect_failure(self):
        for mode, expected in (("disconnect", 1), ("no_data", 1), ("interrupt", 0), ("connect_failure", 1)):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                self.assertEqual(self.run_session(directory, mode), expected)
                summary = json.loads((Path(directory) / "cli.summary.json").read_text())
                self.assertEqual(summary["received"], summary["accepted"] + summary["dropped"])
                self.assertNotEqual(summary["stop_reason"], "duration_complete")

    def test_callback_queue_is_bounded(self):
        mqtt = SimpleNamespace(Client=FakeClient, CallbackAPIVersion=SimpleNamespace(VERSION2=2))
        args = SimpleNamespace(username=None, tls=False)
        connection = c.MQTTInput(mqtt, args, None)
        connection.deadline = c.time.monotonic() + 60
        message = SimpleNamespace(topic=c.TOPIC, payload=payload(), retain=False)
        for _ in range(1030):
            connection.client.on_message(None, None, message)
        self.assertEqual(connection.messages.qsize(), 1024)
        self.assertEqual(connection.overflow, 6)

    def test_header_and_close_errors_are_reported_without_traceback(self):
        class BrokenHeader:
            def write(self, value):
                raise OSError("disk full writing header")

            def close(self):
                raise OSError("disk full closing")

        with tempfile.TemporaryDirectory() as directory, patch.object(Path, "open", return_value=BrokenHeader()):
            self.assertEqual(self.run_session(directory), 1)


if __name__ == "__main__":
    unittest.main()
