"""Compile the real allocation-free C codec and test it against Python Protobuf.

No broker, device, credentials or flash access. Outputs stay in build/protocol_host.
Use the isolated environment from tools/protocol/requirements.txt.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import socket
import struct
import subprocess
import sys
import threading
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/protocol"))
sys.path.insert(0, str(ROOT / "tools/protocol/generated"))
import aai16_pb2 as pb
from config_cli import SENSORS, make_config

OUT = ROOT / "build/protocol_host"
COMPONENT = ROOT / "components/aai16_protocol"
BRIDGE = OUT / ("bridge.exe" if os.name == "nt" else "bridge")
ROUNDTRIPS = []


def raw(message):
    return message.SerializeToString(deterministic=True)


def bridge(payload=b"", mode="roundtrip", success=True):
    completed = subprocess.run([str(BRIDGE), mode], input=payload, capture_output=True, timeout=10)
    if success:
        if completed.returncode:
            raise AssertionError(completed.stderr.decode(errors="replace"))
    else:
        if completed.returncode not in range(1, 16) or completed.stdout:
            raise AssertionError(f"Expected clean rejection: {completed.returncode} {completed.stderr!r}")
    return completed


def roundtrip(message, mode="roundtrip"):
    encoded = raw(message)
    output = bridge(encoded, mode).stdout
    reconstructed = type(message).FromString(output)
    assert reconstructed == message, (message, reconstructed)
    assert bridge(raw(reconstructed), mode).stdout == output
    ROUNDTRIPS.append({"message": message.DESCRIPTOR.name, "bytes": len(encoded)})
    return reconstructed


def varint(number):
    result = bytearray()
    while number >= 128:
        result.append((number & 127) | 128)
        number >>= 7
    result.append(number)
    return bytes(result)


def field(number, value):
    return varint(number << 3) + varint(value)


def blob(number, value):
    return varint((number << 3) | 2) + varint(len(value)) + value


def envelope():
    return pb.Envelope(protocol_version=1, request_id=123, boot_id=0, generation=0, sequence=0)


def identity(family=pb.MODEL_A, source=pb.SOURCE_BUNDLED, bank=None):
    directory = ROOT / "models" / ("motion_model_a" if family == pb.MODEL_A else "motion_model_b")
    manifest = json.loads((directory / "manifest.json").read_text())
    message = pb.ModelIdentity(family=family, source=source, version=manifest["model_version"],
                               sha256=bytes.fromhex(manifest["artifacts"]["model_int8.tflite"]["sha256"]))
    if bank is not None:
        message.bank = bank
    return message


def snapshot():
    message = pb.ConfigSnapshot(schema_version=1, generation=0)
    for index, (_, (sensor, low, high)) in enumerate(SENSORS.items()):
        message.sensors.add(sensor=sensor, enabled=bool(index % 2), interval_ms=low)
    return message


def record(snapshot_bytes=None):
    payload = raw(snapshot()) if snapshot_bytes is None else snapshot_bytes
    value = pb.PersistentConfigRecord(format_version=1, record_sequence=1, snapshot=payload, commit_marker=0x41414916)
    value.crc32 = zlib.crc32(struct.pack("<IQI", value.format_version, value.record_sequence, len(payload)) + payload)
    return value


def telemetry(sensor):
    e = envelope()
    t = e.sensor_telemetry
    t.sensor, t.acquired_us, t.uptime_us, t.health = sensor, 1234567, 1234600, pb.HEALTH_READY
    if sensor == pb.SENSOR_DHT22:
        t.dht22.temperature_c, t.dht22.humidity_pct = 23.5, 51
    elif sensor == pb.SENSOR_GY511:
        t.gy511.acceleration_g.CopyFrom(pb.Vector3(x=-0.125, y=0.25, z=1))
        t.gy511.magnetic_field_ut.CopyFrom(pb.Vector3(x=10, y=-20, z=30))
        t.gy511.jitter_us = -19
    elif sensor == pb.SENSOR_RC522:
        t.rc522.uid = bytes(range(10))
    elif sensor == pb.SENSOR_LD2450:
        for slot in range(1, 4):
            t.ld2450.targets.add(slot=slot, present=True, x_mm=-slot, y_mm=slot * 100,
                                speed_cm_s=-32767, resolution_mm=65535)
    elif sensor == pb.SENSOR_HCSR04P:
        t.hcsr04p.distance_cm, t.hcsr04p.echo_us = 100, 5800
    elif sensor == pb.SENSOR_MHZ1911A:
        t.mhz1911a.co2_ppm = 400
    elif sensor == pb.SENSOR_BME280:
        t.bme280.temperature_c, t.bme280.pressure_hpa = 24, 1013.25
    else:
        t.max6675.temperature_c = 42.25
    return e


def prediction(family):
    e = envelope()
    p = e.ml_prediction
    p.model.CopyFrom(identity(family, pb.SOURCE_STORED, 0))
    p.model_epoch, p.status, p.model_ready = 1, pb.MOTION_PREDICTION, True
    p.classification, p.interval_ms, p.uptime_us = pb.CLASS_REST, 100, 2000000
    p.first_sequence, p.last_sequence = 0xFFFFFFF8, 11  # Sequence wrap is legal.
    p.window_start_us, p.window_end_us, p.inference_us = 0, 1900000, 10
    p.output_int8 = bytes((10, 222))
    p.logits.extend((1.0, -1.0))
    p.backend = pb.BACKEND_ESP_NN_OPTIMIZED
    p.arena_reserved_bytes, p.arena_used_bytes = 16384, 1200
    return e


class ProtocolTests(unittest.TestCase):
    def test_all_configurations_and_boundaries(self):
        for name, (_, low, high) in SENSORS.items():
            for enabled in (False, True):
                with self.subTest(name=name, enabled=enabled):
                    message = make_config(name, enabled=enabled, request_id=1,
                                          expected_generation=0, boot_id=0)
                    decoded = roundtrip(message)
                    self.assertTrue(decoded.sensor_config.HasField("enabled"))
                    self.assertEqual(decoded.sensor_config.enabled, enabled)
                    self.assertFalse(decoded.sensor_config.HasField("interval_ms"))
            for interval in (low, low + 1, high):
                roundtrip(make_config(name, interval_ms=interval, request_id=(1 << 64) - 1))
            for interval in (0, low - 1, high + 1, 0xFFFFFFFF):
                message = make_config(name, interval_ms=low, request_id=1)
                message.sensor_config.interval_ms = interval
                bridge(raw(message), success=False)

    def test_current_firmware_policy_equivalence(self):
        source = (ROOT / "components/mqtt_manager/mqtt_manager.c").read_text()
        for name, (_, low, high) in SENSORS.items():
            if name not in ("hcsr04p", "mhz1911a"):
                self.assertRegex(source, rf'"{name}",\s*{low},\s*{high}')
        for name in ("hcsr04p", "mhz1911a"):
            header = (ROOT / ("components/hcsr04p_sensor/include/hcsr04p_sensor.h" if name == "hcsr04p"
                             else "components/mqtt_manager/include/mqtt_manager.h")).read_text()
            low, high = SENSORS[name][1:]
            self.assertRegex(header, rf'{name.upper()}_MIN_INTERVAL_MS\s+{low}(?:U)?\b')
            self.assertRegex(header, rf'{name.upper()}_MAX_INTERVAL_MS\s+{high}(?:U)?\b')
        self.assertRegex((ROOT / "main/main.c").read_text(), r"#define GY511_SAMPLE_MS\s+100\b")
        defaults = snapshot()
        self.assertEqual(next(p.interval_ms for p in defaults.sensors if p.sensor == pb.SENSOR_GY511), 100)

    def test_presence_and_ambiguous_config(self):
        e = make_config("gy511", enabled=False, request_id=1)
        e.sensor_config.ClearField("enabled")
        bridge(raw(e), success=False)
        e.sensor_config.enabled = False
        e.sensor_config.interval_ms = 100
        self.assertEqual(bridge(raw(e), success=False).returncode, pb.RESULT_AMBIGUOUS_OPERATION)
        for missing in ("protocol_version", "request_id", "phase"):
            e = make_config("gy511", enabled=False, request_id=1)
            e.ClearField(missing)
            bridge(raw(e), success=False)
        for version in (0, 2, 0xFFFFFFFF):
            e = make_config("gy511", enabled=True, request_id=1); e.protocol_version = version
            self.assertEqual(bridge(raw(e), success=False).returncode, pb.RESULT_UNSUPPORTED_VERSION)
        e = make_config("gy511", enabled=True, request_id=1); e.request_id = 0
        bridge(raw(e), success=False)

    def test_phase_ack_and_owner_profiles(self):
        for sensor, low, _ in SENSORS.values():
            for phase in (pb.PHASE_APPLY, pb.PHASE_ROLLBACK):
                e = envelope(); e.phase = phase
                e.apply_config.profile.CopyFrom(pb.SensorProfile(sensor=sensor, enabled=False, interval_ms=low))
                roundtrip(e)
                ack = envelope(); ack.phase = phase
                ack.config_ack.sensor, ack.config_ack.result = sensor, pb.RESULT_OK
                ack.config_ack.effective.CopyFrom(e.apply_config.profile)
                roundtrip(ack)
            e = envelope(); e.phase = pb.PHASE_VALIDATE
            e.config_ack.sensor, e.config_ack.result, e.config_ack.detail = sensor, pb.RESULT_OUT_OF_RANGE, "x" * 63
            e.config_ack.requested.CopyFrom(pb.SensorConfig(sensor=sensor, interval_ms=0))
            roundtrip(e)
            e.config_ack.detail += "x"; bridge(raw(e), success=False)
        e = envelope(); e.phase = pb.PHASE_COMMIT
        e.config_ack.result, e.config_ack.sensor = pb.RESULT_OK, pb.SENSOR_GY511
        roundtrip(e)

    def test_eight_sensor_telemetry_and_optional_humidity(self):
        for sensor in range(1, 9):
            roundtrip(telemetry(sensor))
        e = telemetry(pb.SENSOR_BME280)
        self.assertFalse(roundtrip(e).sensor_telemetry.bme280.HasField("humidity_pct"))
        e.sensor_telemetry.bme280.humidity_pct = 0
        self.assertTrue(roundtrip(e).sensor_telemetry.bme280.HasField("humidity_pct"))
        for size in (4, 7, 10):
            e = telemetry(pb.SENSOR_RC522); e.sensor_telemetry.rc522.uid = bytes(size); roundtrip(e)
        for size in (0, 3, 5, 11):
            e = telemetry(pb.SENSOR_RC522); e.sensor_telemetry.rc522.uid = bytes(size); bridge(raw(e), success=False)
        e = telemetry(pb.SENSOR_LD2450)
        e.sensor_telemetry.ld2450.targets.add().CopyFrom(e.sensor_telemetry.ld2450.targets[0])
        bridge(raw(e), success=False)
        e = telemetry(pb.SENSOR_GY511); e.sensor_telemetry.gy511.acceleration_g.x = float("nan")
        bridge(raw(e), success=False)
        e = telemetry(pb.SENSOR_DHT22); e.sensor_telemetry.sensor = pb.SENSOR_GY511
        bridge(raw(e), success=False)
        for temperature, humidity in ((-40, 0), (80, 100)):
            e = telemetry(pb.SENSOR_DHT22)
            e.sensor_telemetry.dht22.temperature_c, e.sensor_telemetry.dht22.humidity_pct = temperature, humidity
            roundtrip(e)
        for temperature, humidity in ((-40.5, 50), (80.5, 50), (25, -0.5), (25, 100.5), (float("nan"), 50), (25, float("inf"))):
            e = telemetry(pb.SENSOR_DHT22)
            e.sensor_telemetry.dht22.temperature_c, e.sensor_telemetry.dht22.humidity_pct = temperature, humidity
            bridge(raw(e), success=False)
        for temperature in (0, 1023.75):
            e = telemetry(pb.SENSOR_MAX6675); e.sensor_telemetry.max6675.temperature_c = temperature
            roundtrip(e)
        for temperature in (-0.25, 1024, float("nan"), float("inf")):
            e = telemetry(pb.SENSOR_MAX6675); e.sensor_telemetry.max6675.temperature_c = temperature
            bridge(raw(e), success=False)
        for ppm in (399, 5001):
            e = telemetry(pb.SENSOR_MHZ1911A); e.sensor_telemetry.mhz1911a.co2_ppm = ppm
            bridge(raw(e), success=False)

    def test_ml_models_and_statuses(self):
        for family in (pb.MODEL_A, pb.MODEL_B):
            roundtrip(prediction(family))
        for status in range(pb.MOTION_COLLECTING, pb.MOTION_SWITCHING + 1):
            if status == pb.MOTION_PREDICTION:
                continue
            e = envelope()
            e.ml_prediction.CopyFrom(pb.MlPrediction(model=identity(), model_epoch=0, status=status,
                                    model_ready=True, classification=pb.CLASS_UNKNOWN, interval_ms=100, uptime_us=0))
            roundtrip(e)
        e = prediction(pb.MODEL_B); e.ml_prediction.logits.append(2.0)
        bridge(raw(e), success=False)
        e = prediction(pb.MODEL_B); e.ml_prediction.model.sha256 = bytes(33)
        bridge(raw(e), success=False)
        for version in ("v" * 39,):
            e = prediction(pb.MODEL_B); e.ml_prediction.model.version = version; roundtrip(e)
        for version in ("v" * 40, "bad\x00hidden", "bad/version", ""):
            e = prediction(pb.MODEL_B); e.ml_prediction.model.version = version
            bridge(raw(e), success=False)

    def test_ml_telemetry_bounds_and_invalid_values(self):
        e = prediction(pb.MODEL_B)
        e.boot_id = e.generation = e.ml_prediction.model_epoch = 0xffffffff
        p = e.ml_prediction
        p.model.version = 'v' * 39
        p.uptime_us = p.inference_us = (1 << 63) - 1
        p.output_int8 = b'\x80\x00'
        p.clipped = 60
        p.clipped_total = p.queue_drops = p.window_resets = p.invoke_failures = 0xffffffff
        roundtrip(e)
        self.assertLess(len(raw(e)), 320)
        for name in ('boot_id', 'generation'):
            bad = copy.deepcopy(e); bad.ClearField(name); bridge(raw(bad), success=False)
        for name, values in {
            'model_ready': [False], 'classification': [pb.CLASS_UNKNOWN],
            'interval_ms': [0, 99, 500, 10001], 'clipped': [61],
            'last_sequence': [10, 12], 'inference_us': [(1 << 63)],
            'window_start_us': [1900001], 'window_end_us': [1 << 63],
            'uptime_us': [1 << 63], 'arena_used_bytes': [16385],
            'output_int8': [b'', b'\x80', b'\x80\x00\x01'],
        }.items():
            for value in values:
                bad = copy.deepcopy(e); setattr(bad.ml_prediction, name, value)
                bridge(raw(bad), success=False)
        bad = prediction(pb.MODEL_A); bad.ml_prediction.inference_us = bad.ml_prediction.uptime_us + 1
        bridge(raw(bad), success=False)
        for values in ([], [1.0], [1.0, 2.0, 3.0], [float('nan'), 0], [0, float('inf')]):
            bad = copy.deepcopy(e); del bad.ml_prediction.logits[:]; bad.ml_prediction.logits.extend(values)
            bridge(raw(bad), success=False)
        for name in ('first_sequence', 'window_start_us', 'output_int8', 'inference_us'):
            bad = copy.deepcopy(e); bad.ml_prediction.ClearField(name); bridge(raw(bad), success=False)
        # An unavailable status preserves identity/epoch/metadata but is not a prediction.
        status = pb.Envelope(protocol_version=1, boot_id=0, generation=0)
        status.ml_prediction.CopyFrom(pb.MlPrediction(model=identity(), model_epoch=0,
            status=pb.MOTION_INIT_FAILED, model_ready=False, classification=pb.CLASS_UNKNOWN,
            interval_ms=100, uptime_us=0))
        roundtrip(status)
        for name, value in (('classification', pb.CLASS_MOVING), ('output_int8', b'\x00\x00'),
                            ('first_sequence', 0), ('window_end_us', 0), ('inference_us', 1)):
            bad = copy.deepcopy(status); setattr(bad.ml_prediction, name, value); bridge(raw(bad), success=False)
        for wire in (raw(e)[:-1], raw(e) + b'\x00', raw(e) + blob(21, raw(e.ml_prediction)), b'{"state":"REST"}'):
            bridge(wire, success=False)

    def test_all_model_control_operations(self):
        for operation in range(pb.MODEL_OP_BEGIN_UPLOAD, pb.MODEL_OP_LIST + 1):
            e = envelope(); c = e.model_control; c.operation = operation
            if operation <= pb.MODEL_OP_ABORT_UPLOAD:
                c.transfer_id = bytes(range(16))
            if operation == pb.MODEL_OP_BEGIN_UPLOAD:
                c.upload.CopyFrom(pb.UploadManifest(family=pb.MODEL_B, version="b", package_size=73728,
                                  package_sha256=bytes(32), model_size=65536, model_sha256=bytes(32), compatibility_profile=1))
            if operation == pb.MODEL_OP_ACTIVATE:
                c.target.CopyFrom(identity(pb.MODEL_B, pb.SOURCE_STORED, 1))
            if operation in (pb.MODEL_OP_ACTIVATE, pb.MODEL_OP_ROLLBACK):
                c.expected_model_epoch = 0
            roundtrip(e)
            if operation == pb.MODEL_OP_BEGIN_UPLOAD:
                c.upload.model_size += 1; bridge(raw(e), success=False)
        e = envelope(); e.model_control.operation = pb.MODEL_OP_GET_STATUS
        e.model_control.transfer_id = bytes(16); bridge(raw(e), success=False)

    def test_model_selection_and_advanced_preconditions(self):
        for family in (pb.MODEL_A, pb.MODEL_B):
            e = pb.Envelope(protocol_version=1, request_id=234)
            e.model_selection.operation, e.model_selection.family = pb.MODEL_OP_ACTIVATE, family
            roundtrip(e)
        for field in ('boot_id', 'generation', 'sequence', 'phase'):
            bad = copy.deepcopy(e); setattr(bad, field, 1); bridge(raw(bad), success=False)
        for field in ('protocol_version', 'request_id'):
            bad = copy.deepcopy(e); bad.ClearField(field); bridge(raw(bad), success=False)
        for field in ('operation', 'family'):
            bad = copy.deepcopy(e); bad.model_selection.ClearField(field); bridge(raw(bad), success=False)
        bad = copy.deepcopy(e); bad.model_selection.family = pb.MODEL_UNSPECIFIED; bridge(raw(bad), success=False)
        for op in (pb.MODEL_OP_UNSPECIFIED, pb.MODEL_OP_BEGIN_UPLOAD, pb.MODEL_OP_FINISH_UPLOAD,
                   pb.MODEL_OP_ABORT_UPLOAD, pb.MODEL_OP_GET_STATUS, pb.MODEL_OP_LIST, pb.MODEL_OP_ROLLBACK):
            bad = copy.deepcopy(e); bad.model_selection.operation = op; bridge(raw(bad), success=False)
        e.model_selection.ClearField('family'); e.model_selection.operation = pb.MODEL_OP_ROLLBACK; roundtrip(e)
        prefix = raw(pb.Envelope(protocol_version=1, request_id=234))
        for body in (b'\x08\x04\x08\x04\x10\x02', b'\x08\x04\x10\x02\x18\x00', b'\x08\x04\x10\x03'):
            bridge(prefix + b'\x92\x02' + bytes([len(body)]) + body, success=False)
        other = pb.Envelope(); other.model_control.operation = pb.MODEL_OP_GET_STATUS
        bridge(raw(e) + raw(other), success=False)  # conflicting oneof bodies
        full = pb.Envelope(protocol_version=1, request_id=235)
        full.model_control.operation = pb.MODEL_OP_ROLLBACK
        full.model_control.expected_model_epoch = 0
        bridge(raw(full), success=False)  # No relaxation of legacy required boot/generation.
        full.boot_id = 0; full.generation = 0; roundtrip(full)
        full.model_control.ClearField('expected_model_epoch'); bridge(raw(full), success=False)

    def test_upload_bounds_sequence_and_generation(self):
        e = envelope()
        e.model_upload_chunk.CopyFrom(pb.ModelUploadChunk(transfer_id=bytes(16), offset=0, data=bytes(512)))
        roundtrip(e)
        for missing in ("generation", "sequence", "boot_id", "request_id"):
            bad = copy.deepcopy(e); bad.ClearField(missing); bridge(raw(bad), success=False)
        for size in (0, 513, 1000):
            bad = copy.deepcopy(e); bad.model_upload_chunk.data = bytes(size); bridge(raw(bad), success=False)
        for size in (0, 15, 17):
            bad = copy.deepcopy(e); bad.model_upload_chunk.transfer_id = bytes(size); bridge(raw(bad), success=False)
        e.model_upload_chunk.offset = 73728 - 512; roundtrip(e)
        e.model_upload_chunk.offset += 1; bridge(raw(e), success=False)
        e = envelope(); e.model_upload_ack.CopyFrom(pb.ModelUploadAck(transfer_id=bytes(16), result=pb.RESULT_OK,
                         state=pb.UPLOAD_RECEIVING, next_offset=512, max_chunk_bytes=512, detail="stored"))
        roundtrip(e)
        e = envelope(); s = e.model_status
        s.result, s.state, s.model_epoch, s.backend = pb.RESULT_OK, pb.MODEL_STATE_RUNNING, 0, pb.BACKEND_ESP_NN_OPTIMIZED
        s.active.CopyFrom(identity()); s.previous.CopyFrom(identity(pb.MODEL_B, pb.SOURCE_STORED, 1))
        for family in (pb.MODEL_A, pb.MODEL_B):
            for bank in (0, 1): s.stored.add().CopyFrom(identity(family, pb.SOURCE_STORED, bank))
        s.detail = "s" * 63; roundtrip(e)
        s.stored.add().CopyFrom(identity()); bridge(raw(e), success=False)

    def test_persistent_records_integrity_and_interruption(self):
        roundtrip(snapshot(), "snapshot")
        r = record(); roundtrip(r, "record")
        generated = pb.PersistentConfigRecord.FromString(bridge(raw(snapshot()), "prepare").stdout)
        self.assertEqual(pb.ConfigSnapshot.FromString(generated.snapshot), snapshot())
        self.assertEqual(generated.crc32, zlib.crc32(struct.pack("<IQI", 1, generated.record_sequence,
                                                             len(generated.snapshot)) + generated.snapshot))
        for member in ("snapshot", "crc32", "record_sequence", "format_version", "commit_marker"):
            bad = copy.deepcopy(r)
            if member == "snapshot": bad.snapshot = bad.snapshot[:-1] + bytes([bad.snapshot[-1] ^ 1])
            else: setattr(bad, member, getattr(bad, member) + 1)
            bridge(raw(bad), "record", success=False)
        for cut in range(len(raw(r))):
            bridge(raw(r)[:cut], "record", success=False)
        for count in (7, 9):
            s = snapshot()
            if count == 7: del s.sensors[-1]
            else: s.sensors.add().CopyFrom(s.sensors[0])
            bridge(raw(s), "snapshot", success=False)
        s = snapshot(); s.sensors[7].sensor = s.sensors[0].sensor
        bridge(raw(record(raw(s))), "record", success=False)
        r.snapshot = bytes(513); bridge(raw(r), "record", success=False)

    def test_malformed_wire_and_duplicates(self):
        e = make_config("gy511", enabled=False, request_id=1)
        for cut in range(len(raw(e))): bridge(raw(e)[:cut], success=False)
        for invalid in (b"", b"\x00", b"\x08" + b"\xff" * 11, bytes(769), b'{"enabled":false}'):
            bridge(invalid, success=False)
        prefix = raw(pb.Envelope(protocol_version=1, request_id=1, phase=pb.PHASE_VALIDATE))
        # Duplicate scalar/operation, conflicting operations, invalid bool, enum,
        # wrong wire type, unknown setting, uint32 overflow, truncated submessage.
        for body in (b"\x08\x02\x10\x00\x10\x01", b"\x08\x02\x10\x00\x18\x64",
                     b"\x08\x02\x10\x02", field(1, 999) + field(2, 0),
                     b"\x08\x02\x12\x01\x00", b"\x08\x02\x10\x00\x28\x01",
                     field(1, 2) + field(3, 1 << 32), b"\x08\x02\x10"):
            bridge(prefix + blob(10, body), success=False)
        bridge(raw(e) + field(1, 1), success=False)
        bridge(raw(e) + blob(10, raw(e.sensor_config)), success=False)
        bridge(raw(e) + blob(30, field(1, pb.MODEL_OP_GET_STATUS)), success=False)
        bridge(raw(e) + field(100, 1), success=False)
        # Invalid UTF-8 in a bounded diagnostic string.
        ack = envelope(); ack.phase = pb.PHASE_VALIDATE
        ack.config_ack.sensor, ack.config_ack.result = pb.SENSOR_GY511, pb.RESULT_OUT_OF_RANGE
        prefix = copy.deepcopy(ack); prefix.ClearField("config_ack")
        bridge(raw(prefix) + blob(11, raw(ack.config_ack) + blob(3, b"\xff")), success=False)

    def test_c_origin_false_and_stress(self):
        e = pb.Envelope.FromString(bridge(mode="emit").stdout)
        self.assertEqual(e.request_id, 0xFEDCBA9876543210)
        self.assertTrue(e.sensor_config.HasField("enabled"))
        self.assertFalse(e.sensor_config.enabled)
        self.assertEqual(e.sensor_config.sensor, pb.SENSOR_GY511)
        subprocess.run([str(BRIDGE), "self"], check=True, timeout=30)

    def test_python_cli_offline(self):
        completed = subprocess.run([sys.executable, str(ROOT / "tools/protocol/config_cli.py"), "gy511",
                                   "--enabled", "false", "--request-id", "1"], capture_output=True, text=True, check=True)
        data = bytes.fromhex(re.search(r"hex=([0-9a-f]+)", completed.stdout)[1])
        roundtrip(pb.Envelope.FromString(data))
        self.assertIn("no MQTT message sent", completed.stdout)
        with self.assertRaises(ValueError): make_config("gy511", enabled=False, interval_ms=100)
        with self.assertRaises(ValueError): make_config("gy511", interval_ms=99)

    def test_all_variable_fields_are_statically_bounded(self):
        options = (ROOT / "protocol/aai16.options").read_text()
        for message in pb.DESCRIPTOR.message_types_by_name.values():
            for member in message.fields:
                if member.label == member.LABEL_REPEATED:
                    self.assertRegex(options, rf"{re.escape(member.full_name)} max_count:\d+")
                if member.type in (member.TYPE_BYTES, member.TYPE_STRING):
                    self.assertRegex(options, rf"{re.escape(member.full_name)} max_(?:size|length):\d+")

    def test_python_publisher_loopback_and_application_ack(self):
        # Minimal local MQTT 3.1.1 peer. Does not contact the project broker.
        # Exercises Paho binary publication, retain=0, C validation, request
        # correlation and the distinction between PUBACK/VALIDATE/APPLY.
        errors = []
        def receive(connection):
            def exact(count):
                data = b""
                while len(data) < count:
                    part = connection.recv(count - len(data))
                    if not part: raise RuntimeError("Unexpected MQTT EOF")
                    data += part
                return data
            header = exact(1)[0]
            length, multiplier = 0, 1
            for _ in range(4):
                value = exact(1)[0]; length += (value & 127) * multiplier
                if value < 128: break
                multiplier *= 128
            if length > 2048: raise RuntimeError("Oversized test packet")
            return header, exact(length)

        with socket.socket() as listener:
            listener.bind(("127.0.0.1", 0)); listener.listen(); listener.settimeout(10)
            port = listener.getsockname()[1]
            def server():
                try:
                    with listener.accept()[0] as connection:
                        connection.settimeout(10)
                        header, _ = receive(connection); self.assertEqual(header >> 4, 1)
                        connection.sendall(b"\x20\x02\x00\x00")
                        header, body = receive(connection); self.assertEqual(header >> 4, 8)
                        connection.sendall(b"\x90\x03" + body[:2] + b"\x01")
                        header, body = receive(connection)
                        self.assertEqual(header, 0x32) # QoS 1, not retained.
                        size = int.from_bytes(body[:2], "big")
                        self.assertEqual(body[2:2 + size], b"protocol-test/config")
                        packet_id = body[2 + size:4 + size]
                        request = pb.Envelope.FromString(body[4 + size:])
                        roundtrip(request)
                        self.assertFalse(request.sensor_config.enabled)
                        connection.sendall(b"\x40\x02" + packet_id)
                        ack = envelope(); ack.request_id = request.request_id
                        ack.phase = pb.PHASE_APPLY; ack.generation = 2
                        ack.config_ack.result, ack.config_ack.sensor = pb.RESULT_OK, pb.SENSOR_GY511
                        ack.config_ack.requested.CopyFrom(request.sensor_config)
                        topic = b"protocol-test/ack"
                        def send_ack(retained=False):
                            payload = len(topic).to_bytes(2, "big") + topic + raw(ack)
                            connection.sendall(bytes([0x31 if retained else 0x30]) + varint(len(payload)) + payload)
                        send_ack(retained=True)
                        ack.request_id += 1; send_ack()
                        ack.request_id = request.request_id; ack.phase = pb.PHASE_VALIDATE; send_ack()
                        ack.phase = pb.PHASE_APPLY; send_ack()
                        header, _ = receive(connection); self.assertEqual(header >> 4, 14)
                except Exception as error:
                    errors.append(error)
            thread = threading.Thread(target=server, daemon=True); thread.start()
            env = dict(os.environ)
            env.pop("AAI16_MQTT_USERNAME", None); env.pop("AAI16_MQTT_PASSWORD", None)
            completed = subprocess.run([sys.executable, str(ROOT / "tools/protocol/config_cli.py"), "gy511",
                        "--enabled", "false", "--request-id", "55", "--host", "127.0.0.1", "--port", str(port),
                        "--topic", "protocol-test/config", "--ack-topic", "protocol-test/ack", "--timeout", "5"],
                        capture_output=True, text=True, timeout=20, env=env)
            thread.join(timeout=10)
            self.assertFalse(thread.is_alive())
            if errors: raise errors[0]
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("phase=PHASE_APPLY generation=2 result=RESULT_OK", completed.stdout)


def compile_and_measure(args):
    OUT.mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable, str(ROOT / "tools/protocol/generate.py"), "--check"], check=True)
    cc = shutil.which(args.cc)
    if not cc: raise RuntimeError(f"Compiler unavailable: {args.cc}")
    include = sum((["-I", str(COMPONENT / path)] for path in ("include", "generated", "vendor/nanopb")), [])
    flags = ["-std=c11", "-Wall", "-Wextra", "-Werror", "-O2", "-DPB_VALIDATE_UTF8=1", "-DPB_MESSAGE_NESTING_MAX=8"] + include
    sources = [COMPONENT / "aai16_protocol.c", COMPONENT / "generated/aai16.pb.c"]
    sources += [COMPONENT / "vendor/nanopb" / name for name in ("pb_common.c", "pb_encode.c", "pb_decode.c")]
    objects = []
    nm = str(Path(cc).with_name("nm.exe" if os.name == "nt" else "nm"))
    for source in sources:
        obj = OUT / (source.stem + ".o")
        subprocess.run([cc] + flags + ["-c", str(source), "-o", str(obj)], check=True)
        symbols = subprocess.check_output([nm, "-u", str(obj)], text=True)
        if re.search(r"\b(malloc|calloc|realloc|free)\b", symbols):
            raise AssertionError(f"Heap reference in protocol runtime: {source}")
        objects.append(str(obj))
    subprocess.run([cc] + flags + [str(Path(__file__).with_name("bridge.c"))] + objects + ["-lm", "-o", str(BRIDGE)], check=True)
    cxx = shutil.which(args.cxx)
    if not cxx: raise RuntimeError(f"C++ compiler unavailable: {args.cxx}")
    smoke = OUT / "cpp_smoke.cc"
    smoke.write_text('#include "aai16_protocol.h"\nint main() { aai16_Envelope e = aai16_Envelope_init_zero; return aai16_protocol_validate(&e) == aai16_ResultCode_RESULT_MISSING_FIELD ? 0 : 1; }\n')
    smoke_binary = OUT / ("cpp_smoke.exe" if os.name == "nt" else "cpp_smoke")
    subprocess.run([cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror"] + include + [str(smoke)] + objects + ["-o", str(smoke_binary)], check=True)
    subprocess.run([str(smoke_binary)], check=True)
    header = (COMPONENT / "generated/aai16.pb.h").read_text()
    if "pb_callback_t" in header or re.search(r"\b(?:CALLBACK|POINTER)\b", header):
        raise AssertionError("Generated schema has dynamic/callback storage")
    sizes = {name: {"encoded_max": int(size)} for name, size in re.findall(r"#define aai16_(\w+)_size\s+(\d+)", header)}
    probe = OUT / "size_probe.c"
    probe.write_text('#include "aai16_protocol.h"\n' + '\n'.join(
        f'const unsigned char size_{name}[sizeof(aai16_{name})] = {{0}};' for name in sizes) + '\n')
    # PE/COFF nm does not reliably expose object sizes. Measure native sizeof by
    # running C, and use ELF symbol sizes only for the actual Xtensa ABI.
    native_probe = OUT / "native_sizes.c"
    native_probe.write_text('#include <stdio.h>\n#include "aai16_protocol.h"\nint main(void) {\n' + '\n'.join(
        f'printf("{name} %zu\\n", sizeof(aai16_{name}));' for name in sizes) + '\nreturn 0; }\n')
    native_binary = OUT / ("native_sizes.exe" if os.name == "nt" else "native_sizes")
    subprocess.run([cc, "-std=c11"] + include + [str(native_probe), "-o", str(native_binary)], check=True)
    for line in subprocess.check_output([str(native_binary)], text=True).splitlines():
        name, size = line.split()
        sizes[name]["host_decoded"] = int(size)
    compilers = []
    if args.xtensa_cc:
        xtensa = str(Path(args.xtensa_cc).resolve())
        compilers.append(("esp32s3_decoded", xtensa, xtensa.replace("gcc", "nm")))
    for label, compiler, symbol_tool in compilers:
        obj = OUT / (label + ".o")
        subprocess.run([compiler, "-std=c11"] + include + ["-c", str(probe), "-o", str(obj)], check=True)
        symbols = subprocess.check_output([symbol_tool, "-S", str(obj)], text=True)
        for size, name in re.findall(r"(?m)^[0-9a-fA-F]+[ \t]+([0-9a-fA-F]+)[ \t]+\w[ \t]+size_(\w+)$", symbols):
            sizes[name][label] = int(size, 16)
        if any(label not in values for values in sizes.values()):
            raise AssertionError(f"Missing size evidence for {label}")
    return sizes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="gcc")
    parser.add_argument("--cxx", default="g++")
    parser.add_argument("--xtensa-cc")
    args = parser.parse_args()
    sizes = compile_and_measure(args)
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ProtocolTests))
    comparisons = []
    for name in SENSORS:
        for enabled in (False, True):
            e = make_config(name, enabled=enabled, request_id=1)
            legacy = {"sensor": name, "enabled": enabled}
            equivalent = dict(legacy, protocol_version=1, request_id=1, phase="validate")
            legacy_size = len(json.dumps(legacy, separators=(",", ":")).encode())
            equivalent_size = len(json.dumps(equivalent, separators=(",", ":")).encode())
            size = len(raw(e))
            comparisons.append({"sensor": name, "enabled": enabled, "protobuf_bytes": size,
                                "legacy_json_bytes": legacy_size, "equivalent_json_bytes": equivalent_size,
                                "vs_legacy_reduction_percent": round(100 * (1 - size / legacy_size), 2),
                                "vs_equivalent_reduction_percent": round(100 * (1 - size / equivalent_size), 2)})
    report = {"nanopb": "0.4.9.2", "tests_run": result.testsRun, "success": result.wasSuccessful(),
              "roundtrips": len(ROUNDTRIPS), "malformed_stress_iterations": 20000,
              "allocator_symbols": [], "sizes": sizes, "configuration_size_comparison": comparisons,
              "hardware_tested": False}
    (OUT / "results.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: report[key] for key in ("tests_run", "success", "roundtrips", "malformed_stress_iterations")}))
    print(json.dumps(sizes, indent=2))
    raise SystemExit(0 if result.wasSuccessful() else 1)


if __name__ == "__main__":
    main()
