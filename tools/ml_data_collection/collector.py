"""Raw GY-511 recording. JSON is a temporary adapter; CSV is transport independent."""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import getpass
import json
import math
import os
from pathlib import Path
import queue
import re
import sys
import threading
import time
import uuid

TOPIC = "aai16/sensors/gy511/data"
CSV_FIELDS = (
    "timestamp_ms", "boot_id", "seq", "uptime_ms", "ax_g", "ay_g", "az_g",
    "label", "session_id", "jitter_us",
)


@dataclass(frozen=True)
class Sample:
    boot_id: str
    seq: int
    uptime_ms: int
    ax_g: float
    ay_g: float
    az_g: float
    jitter_us: int | None = None


def integer(value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError("invalid integer")
    return value


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON field")
        result[key] = value
    return result


def decode_json(payload: bytes) -> Sample:
    """Replace this function with a Protobuf decoder later, returning Sample.

    Values are decoded unchanged, including legitimate repeated axes at rest.
    The 2.048 g bound is the representable +/-2 g register range rounded to the
    firmware's three decimal places, not model clipping or normalization.
    """
    if len(payload) > 4096:
        raise ValueError("oversize payload")
    try:
        data = json.loads(payload, object_pairs_hook=unique_object)
        if not isinstance(data, dict) or data.get("sensor") != "gy511":
            raise ValueError("wrong sensor")
        boot = data["boot_id"]
        if not isinstance(boot, str) or not re.fullmatch(r"[0-9a-fA-F]{8}", boot):
            raise ValueError("invalid boot_id")
        seq = integer(data["seq"], 0, 0xFFFFFFFF)
        uptime = integer(data["uptime_ms"], 0, (2**63 - 1) // 1000)
        axes = data["acceleration_g"]
        if not isinstance(axes, dict):
            raise ValueError("missing axes")
        values = [axes[axis] for axis in ("x", "y", "z")]
        if any(type(v) not in (int, float) or not math.isfinite(v) or
               abs(v) > 2.048 for v in values):
            raise ValueError("invalid acceleration")
        jitter = data.get("jitter_us")
        if "jitter_us" in data:
            jitter = integer(jitter, -(2**63), 2**63 - 1)
        return Sample(boot.lower(), seq, uptime, *values, jitter)
    except (KeyError, TypeError, OverflowError, UnicodeError, RecursionError) as exc:
        raise ValueError("invalid telemetry") from exc


@dataclass
class Statistics:
    received: int = 0
    accepted: int = 0
    dropped: int = 0
    invalid: int = 0
    retained: int = 0
    duplicates: int = 0
    out_of_order: int = 0
    timestamp_errors: int = 0
    boot_changes: int = 0
    sequence_gaps: int = 0
    excessive_gaps: int = 0
    missing_jitter: int = 0
    queue_overflow: int = 0
    shutdown_drops: int = 0
    write_errors: int = 0
    min_delta_ms: int | None = None
    max_delta_ms: int | None = None
    max_abs_jitter_us: int = 0


class Recording:
    """Single-threaded validation and CSV writer, independent of MQTT/Paho."""

    def __init__(self, stream, label, session_id, max_gap_ms=150, decoder=decode_json):
        if label not in ("REST", "MOVING"):
            raise ValueError("explicit REST or MOVING label required")
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,79}", session_id):
            raise ValueError("invalid session_id")
        if not math.isfinite(max_gap_ms) or max_gap_ms <= 0:
            raise ValueError("max_gap_ms must be positive")
        self.label, self.session_id = label, session_id
        self.max_gap_ms, self.decoder = max_gap_ms, decoder
        self.stream = stream
        self.writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        stream.flush()
        self.stats = Statistics()
        self.last = None
        # Bounded duplicate history. Older repeats are still rejected as late.
        self.recent = {}

    def drop(self, reason):
        self.stats.dropped += 1
        setattr(self.stats, reason, getattr(self.stats, reason) + 1)
        return False

    def receive(self, payload, retained=False):
        self.stats.received += 1
        if retained:
            return self.drop("retained")
        try:
            sample = self.decoder(payload)
        except ValueError:
            return self.drop("invalid")
        delta = None
        distance = 1
        if self.last is not None:
            if sample.boot_id != self.last.boot_id:
                self.drop("boot_changes")
                raise RuntimeError("Device boot changed or another publisher appeared; start a new session")
            if sample.seq in self.recent:
                return self.drop("duplicates")
            distance = (sample.seq - self.last.seq) & 0xFFFFFFFF
            if distance == 0 or distance >= 0x80000000:
                return self.drop("out_of_order")
            delta = sample.uptime_ms - self.last.uptime_ms
            if delta <= 0:
                return self.drop("timestamp_errors")
        row = asdict(sample)
        row.update(timestamp_ms=sample.uptime_ms, label=self.label, session_id=self.session_id)
        try:
            self.writer.writerow(row)
            self.stream.flush()
        except (OSError, ValueError):
            self.drop("write_errors")
            raise
        self.stats.accepted += 1
        self.stats.sequence_gaps += distance - 1
        if delta is not None:
            self.stats.min_delta_ms = delta if self.stats.min_delta_ms is None else min(self.stats.min_delta_ms, delta)
            self.stats.max_delta_ms = delta if self.stats.max_delta_ms is None else max(self.stats.max_delta_ms, delta)
            self.stats.excessive_gaps += int(delta > self.max_gap_ms)
        if sample.jitter_us is None:
            self.stats.missing_jitter += 1
        else:
            self.stats.max_abs_jitter_us = max(self.stats.max_abs_jitter_us, abs(sample.jitter_us))
        self.last = sample
        self.recent[sample.seq] = None
        if len(self.recent) > 4096:
            del self.recent[next(iter(self.recent))]
        return True


class MQTTInput:
    """Callbacks only queue messages. Disk writes happen in the main thread."""

    def __init__(self, mqtt, args, password):
        self.messages = queue.Queue(maxsize=1024)
        self.ready = threading.Event()
        self.failed = threading.Event()
        self.failure = ""
        self.started = None
        self.deadline = None
        self.overflow = 0
        self.stopping = False
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id="gy511-" + uuid.uuid4().hex[:16],
                                  clean_session=True, reconnect_on_failure=False)
        if args.username:
            self.client.username_pw_set(args.username, password)
        if args.tls:
            self.client.tls_set(ca_certs=args.ca_file)

        def fail(reason):
            self.failure = reason
            self.failed.set()

        def on_connect(client, userdata, flags, reason_code, properties):
            if reason_code.is_failure:
                fail("Broker rejected connection: " + str(reason_code))
                return
            result, _ = client.subscribe(TOPIC, qos=1)
            if result != mqtt.MQTT_ERR_SUCCESS:
                fail("Cannot subscribe")

        def on_subscribe(client, userdata, mid, reason_codes, properties):
            if not reason_codes or any(code.is_failure for code in reason_codes):
                fail("Broker rejected subscription")
                return
            self.started = time.monotonic()
            self.deadline = self.started + args.duration
            self.ready.set()

        def on_message(client, userdata, message):
            arrived = time.monotonic()
            if (self.stopping or self.deadline is None or arrived >= self.deadline or
                    message.topic != TOPIC):
                return
            try:
                self.messages.put_nowait((message.payload, message.retain))
            except queue.Full:
                self.overflow += 1

        def on_disconnect(client, userdata, flags, reason_code, properties):
            if not self.stopping:
                fail("MQTT disconnected; partial session saved (start a new session after reconnecting)")

        self.client.on_connect = on_connect
        self.client.on_subscribe = on_subscribe
        self.client.on_message = on_message
        self.client.on_disconnect = on_disconnect
        self.client.on_connect_fail = lambda client, userdata: fail("Cannot connect to MQTT broker")

    def stop(self):
        self.stopping = True
        self.client.disconnect()
        self.client.loop_stop()


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.environ.get("MQTT_HOST"))
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--username", default=os.environ.get("MQTT_USERNAME"))
    parser.add_argument("--password-prompt", action="store_true",
                        help="Prompt without echo; otherwise use MQTT_PASSWORD environment variable")
    parser.add_argument("--tls", action="store_true")
    parser.add_argument("--ca-file", help="Optional private broker CA; TLS verification stays enabled")
    parser.add_argument("--label", required=True, choices=("REST", "MOVING"))
    parser.add_argument("--duration", type=positive, default=60)
    parser.add_argument("--connect-timeout", type=positive, default=15)
    parser.add_argument("--max-gap-ms", type=positive, default=150)
    parser.add_argument("--session-id", default=None)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).parent / "recordings")
    args = parser.parse_args(argv)
    if not args.host or "://" in args.host:
        parser.error("set MQTT_HOST or --host to the broker hostname (not a URI)")
    args.port = args.port if args.port is not None else (8883 if args.tls else 1883)
    if not 1 <= args.port <= 65535:
        parser.error("invalid port")
    if args.ca_file and not args.tls:
        parser.error("--ca-file requires --tls")
    if args.session_id is None:
        args.session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:12]
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,79}", args.session_id):
        parser.error("session ID must be 1..80 letters/digits/dots/underscores/hyphens, starting with a letter/digit")
    return args


def main(argv=None):
    args = arguments(argv)
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("Install dependencies: python -m pip install -r tools/ml_data_collection/requirements.txt", file=sys.stderr)
        return 2
    connection = recording = stream = None
    result, reason = 0, "duration_complete"
    created_utc = datetime.now(timezone.utc).isoformat()
    csv_path = args.output_dir / (args.session_id + ".csv")
    summary_path = args.output_dir / (args.session_id + ".summary.json")
    try:
        password = getpass.getpass("MQTT password: ") if args.password_prompt else os.environ.get("MQTT_PASSWORD")
        if password and not args.username:
            raise ValueError("A password requires --username or MQTT_USERNAME")
        args.output_dir.mkdir(parents=True, exist_ok=True)
        if summary_path.exists():
            raise FileExistsError("Session summary already exists; choose a new session ID")
        stream = csv_path.open("x", newline="", encoding="utf-8")
        recording = Recording(stream, args.label, args.session_id, args.max_gap_ms)
        connection = MQTTInput(mqtt, args, password)
        connection.client.connect_async(args.host, args.port, keepalive=30)
        connection.client.loop_start()
        timeout = time.monotonic() + args.connect_timeout
        while not connection.ready.wait(0.1):
            if connection.failed.is_set():
                raise RuntimeError(connection.failure)
            if time.monotonic() >= timeout:
                raise RuntimeError("Connection/subscription timed out")
        print(f"Recording {args.label} for {args.duration:g}s: {csv_path}", flush=True)
        while time.monotonic() < connection.deadline:
            if connection.failed.is_set():
                raise RuntimeError(connection.failure)
            try:
                payload, retained = connection.messages.get(timeout=0.1)
            except queue.Empty:
                continue
            recording.receive(payload, retained)
        connection.stop()
        while not connection.messages.empty():
            recording.receive(*connection.messages.get_nowait())
        if not recording.stats.accepted:
            raise RuntimeError("No valid samples received; check topic, device and 100 ms configuration")
    except KeyboardInterrupt:
        reason = "interrupted"
    except (OSError, ValueError, RuntimeError) as exc:
        result, reason = 1, str(exc)
        print("Recording stopped: " + reason, file=sys.stderr)
    finally:
        if connection is not None:
            try:
                connection.stop()
            except (OSError, RuntimeError) as exc:
                result, reason = 1, "MQTT shutdown failed: " + str(exc)
        if stream is not None:
            try:
                stream.close()
            except OSError as exc:
                result, reason = 1, "CSV close failed: " + str(exc)
                print(reason, file=sys.stderr)
        if recording is not None:
            if connection is not None:
                pending = connection.messages.qsize()
                recording.stats.queue_overflow = connection.overflow
                recording.stats.shutdown_drops = pending
                recording.stats.received += pending + connection.overflow
                recording.stats.dropped += pending + connection.overflow
            summary = dict(schema_version=1, session_id=args.session_id, label=args.label,
                           created_utc=created_utc, finished_utc=datetime.now(timezone.utc).isoformat(),
                           requested_duration_s=args.duration, expected_interval_ms=100,
                           max_gap_ms=args.max_gap_ms, decoder="gy511-json-v1",
                           csv_fields=CSV_FIELDS, preprocessing="none; raw measured g",
                           stop_reason=reason, **asdict(recording.stats))
            print(json.dumps(summary, indent=2))
            if recording.stats.excessive_gaps:
                print("Sampling gaps detected; inspect timestamps and confirm the 100 ms configuration before using this session.", file=sys.stderr)
            if recording.stats.missing_jitter:
                print("Samples lack jitter metadata; verify the updated firmware before treating uptime_ms as acquisition time.", file=sys.stderr)
            try:
                with summary_path.open("x", encoding="utf-8") as output:
                    json.dump(summary, output, indent=2)
                    output.write("\n")
            except OSError as exc:
                result = 1
                print("Cannot save summary: " + str(exc), file=sys.stderr)
    return result


if __name__ == "__main__":
    sys.exit(main())
