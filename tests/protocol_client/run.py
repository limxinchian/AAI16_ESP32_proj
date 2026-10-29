"""Client workflow tests; no project broker/device. npm ci in this directory first.

Optional --mqttx-asar reads the installed app's vm2/acorn libraries into build/;
the application and its connections/settings are never changed.
"""
import argparse
import base64
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/protocol'))
from config_cli import make_config, make_from_input, load_retry, pb
from request_ids import allocate_request_id, atomic_write, MAX_ID

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--mqttx-asar', type=Path)
args = parser.parse_args()
OUT = ROOT / 'build/protocol_client'
OUT.mkdir(parents=True, exist_ok=True)


def extract_runtime(archive):
    with archive.open('rb') as file:
        _, header_size, _, json_size = struct.unpack('<4I', file.read(16))
        tree = json.loads(file.read(json_size))
        offset = 8 + header_size
        def walk(node, base=''):
            for name, item in node['files'].items():
                key = base + name
                if 'files' in item:
                    yield from walk(item, key + '/')
                else:
                    yield key, item
        entries = dict(walk(tree))
        file.seek(offset + int(entries['package.json']['offset']))
        version = json.loads(file.read(entries['package.json']['size']))['version']
        runtime = OUT / 'mqttx_runtime'
        for name, entry in entries.items():
            if not any(name.startswith('node_modules/' + package + '/') for package in ('vm2', 'acorn', 'acorn-walk')):
                continue
            target = (runtime / name).resolve()
            if not target.is_relative_to(runtime.resolve()) or entry.get('unpacked'):
                raise ValueError('Unexpected archive entry')
            target.parent.mkdir(parents=True, exist_ok=True)
            file.seek(offset + int(entry['offset']))
            target.write_bytes(file.read(entry['size']))
    print(f'Testing installed MQTTX {version} sandbox runtime', flush=True)
    return runtime / 'node_modules/vm2'


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=OUT)
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.state = self.directory / 'ids.json'

    def test_persistent_ids_clock_changes_and_exhaustion(self):
        self.assertEqual(allocate_request_id(self.state, clock_ns=lambda: 100), 100)
        self.assertEqual(allocate_request_id(self.state, clock_ns=lambda: 100), 101)
        self.assertEqual(allocate_request_id(self.state, clock_ns=lambda: 1), 102)
        self.assertEqual(allocate_request_id(self.state, clock_ns=lambda: -1, floor=1000), 1001)
        self.assertEqual(allocate_request_id(self.state, clock_ns=lambda: MAX_ID), MAX_ID)
        with self.assertRaises(ValueError): allocate_request_id(self.state, clock_ns=lambda: 0)

    def test_parallel_processes_share_state_and_restart(self):
        code = ('import sys;sys.path.insert(0,sys.argv[1]);from request_ids import allocate_request_id;'
                'print(allocate_request_id(sys.argv[2],clock_ns=lambda:100))')
        def allocate(_):
            return int(subprocess.check_output([sys.executable, '-B', '-c', code,
                str(ROOT / 'tools/protocol'), str(self.state)], text=True))
        with ThreadPoolExecutor(max_workers=4) as pool:
            ids = list(pool.map(allocate, range(12)))
        self.assertEqual(sorted(ids), list(range(100, 112)))
        self.assertEqual(allocate(0), 112)

    def test_corrupt_state_fails_closed(self):
        for data in [b'', b'{', b'{}', b'{"format_version":true,"last_id":"1"}',
                     b'{"format_version":1,"last_id":"0"}',
                     b'{"format_version":1,"last_id":"1","last_id":"2"}']:
            self.state.write_bytes(data)
            with self.assertRaises(ValueError): allocate_request_id(self.state)
            self.assertEqual(self.state.read_bytes(), data)

    def test_client_fields_false_and_invalid_inputs(self):
        first = make_from_input('{"sensor_config":{"sensor":2,"enabled":false}}', self.state)
        second = make_from_input('{"sensor_config":{"sensor":2,"enabled":true}}', self.state)
        self.assertLess(first.request_id, second.request_id)
        self.assertTrue(first.sensor_config.HasField('enabled'))
        self.assertFalse(first.sensor_config.enabled)
        self.assertEqual(first.protocol_version, 1)
        self.assertEqual(first.phase, pb.PHASE_VALIDATE)
        saved = self.state.read_bytes()
        for text in ['{}', '[]', '{', '{"sensor_config":{"sensor":2,"enabled":false,"interval_ms":500}}',
                     '{"sensor_config":{"sensor":2,"enabled":false,"enabled":true}}',
                     '{"sensor_config":{"sensor":2,"calibrate":true}}',
                     '{"sensor_config":{"sensor":2,"interval_ms":99}}',
                     '{"sensor_config":{"sensor":2,"enabled":null}}',
                     '{"sensor_config":{"sensor":true,"enabled":true}}']:
            with self.assertRaises(ValueError): make_from_input(text, self.state)
            self.assertEqual(self.state.read_bytes(), saved)

    def test_exact_saved_retry_and_cli(self):
        base = [sys.executable, '-B', str(ROOT / 'tools/protocol/config_cli.py'), '--id-state', str(self.state)]
        result = subprocess.run(base + ['--input', '-'], input='{"sensor_config":{"sensor":2,"interval_ms":500}}',
                                capture_output=True, text=True, check=True)
        files = list((self.directory / 'requests').glob('*.pb'))
        self.assertEqual(len(files), 1)
        message, payload = load_retry(files[0])
        saved = self.state.read_bytes()
        retry = subprocess.run(base + ['--retry-file', str(files[0])], capture_output=True, text=True, check=True)
        self.assertIn(f'request_id={message.request_id}', retry.stdout)
        self.assertIn(f'hex={payload.hex()}', result.stdout)
        self.assertEqual(self.state.read_bytes(), saved)
        mixed = subprocess.run(base + ['--retry-file', str(files[0]), '--enabled', 'false'], capture_output=True)
        self.assertNotEqual(mixed.returncode, 0)
        for bad in [payload[:-1], payload + b'\x08\x01', payload + b'\xa0\x06\x01', b'{}', bytes(129)]:
            path = self.directory / 'invalid.pb'; path.write_bytes(bad)
            with self.assertRaises(ValueError): load_retry(path)

    def test_publisher_automatic_ids_and_exact_retry(self):
        # Local MQTT peer, not the project broker/device. Models cached owner
        # receipts; the separate mqtt_host suite tests actual C idempotency.
        received, cache, errors = [], {}, []
        def varint(value):
            data = bytearray()
            while value > 127:
                data.append((value & 127) | 128); value >>= 7
            data.append(value)
            return bytes(data)
        def packet(connection):
            def exact(count):
                result = b''
                while len(result) < count:
                    chunk = connection.recv(count - len(result))
                    if not chunk: raise ValueError('Unexpected EOF')
                    result += chunk
                return result
            header, length, shift = exact(1)[0], 0, 0
            for _ in range(4):
                value = exact(1)[0]; length |= (value & 127) << shift
                if value < 128: break
                shift += 7
            if length > 4096: raise ValueError('Oversized test packet')
            return header, exact(length)
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0)); listener.listen(); listener.settimeout(10)
            def server():
                try:
                    for _ in range(3):
                        with listener.accept()[0] as connection:
                            connection.settimeout(10)
                            self.assertEqual(packet(connection)[0] >> 4, 1)
                            connection.sendall(b'\x20\x02\x00\x00')
                            header, body = packet(connection)
                            self.assertEqual(header >> 4, 8)
                            self.assertIn(b'aai16/config/ack', body)
                            connection.sendall(b'\x90\x03' + body[:2] + b'\x01')
                            header, body = packet(connection)
                            self.assertEqual(header, 0x32)  # QoS 1, not retained.
                            topic_len = int.from_bytes(body[:2], 'big')
                            self.assertEqual(body[2:2+topic_len], b'aai16/config/set')
                            identity, raw = body[2+topic_len:4+topic_len], body[4+topic_len:]
                            request = pb.Envelope.FromString(raw); received.append(raw)
                            if request.request_id not in cache:
                                ack = pb.Envelope(protocol_version=1, request_id=request.request_id,
                                    boot_id=0x12345678, generation=len(cache)+1, phase=pb.PHASE_APPLY)
                                ack.config_ack.result, ack.config_ack.sensor = pb.RESULT_OK, request.sensor_config.sensor
                                ack.config_ack.requested.CopyFrom(request.sensor_config)
                                cache[request.request_id] = ack.SerializeToString()
                            connection.sendall(b'\x40\x02' + identity)
                            topic = b'aai16/config/ack'
                            reply = len(topic).to_bytes(2,'big') + topic + cache[request.request_id]
                            connection.sendall(b'\x30' + varint(len(reply)) + reply)
                            self.assertEqual(packet(connection)[0] >> 4, 14)
                except Exception as error:
                    errors.append(error)
            thread = threading.Thread(target=server, daemon=True); thread.start()
            env = dict(os.environ)
            env.pop('AAI16_MQTT_USERNAME', None); env.pop('AAI16_MQTT_PASSWORD', None)
            base = [sys.executable, '-B', str(ROOT / 'tools/protocol/config_cli.py'),
                    '--id-state', str(self.state), '--host', '127.0.0.1',
                    '--port', str(listener.getsockname()[1]), '--timeout', '5']
            for enabled in ['false', 'true']:
                result = subprocess.run(base + ['gy511', '--enabled', enabled], capture_output=True, text=True, env=env, timeout=15)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('PHASE_APPLY', result.stdout)
            first = min((self.directory / 'requests').glob('*.pb'), key=lambda path: int(path.stem))
            result = subprocess.run(base + ['--retry-file', str(first)], capture_output=True, text=True, env=env, timeout=15)
            self.assertEqual(result.returncode, 0, result.stderr)
            thread.join(10)
            self.assertFalse(thread.is_alive())
            if errors: raise errors[0]
        self.assertEqual(len(cache), 2)
        self.assertEqual(received[0], received[2])
        self.assertLess(pb.Envelope.FromString(received[0]).request_id, pb.Envelope.FromString(received[1]).request_id)

    def test_mqttx_function_schema_and_c_compatibility(self):
        command = ['node', str(Path(__file__).with_name('mqttx_test.cjs'))]
        if args.mqttx_asar:
            command.append(str(extract_runtime(args.mqttx_asar)))
        result = json.loads(subprocess.check_output(command, text=True))
        bridge = ROOT / 'build/protocol_host' / ('bridge.exe' if os.name == 'nt' else 'bridge')
        if not bridge.exists():
            self.fail('Run tests/protocol_host/run.py first to build the real C codec bridge')
        for index, fixture in enumerate(result['fixtures']):
            raw = base64.b64decode(fixture)
            message = pb.Envelope.FromString(raw)
            self.assertEqual(message.protocol_version, 1)
            self.assertEqual(message.phase, pb.PHASE_VALIDATE)
            self.assertGreater(message.request_id, 2**53)
            completed = subprocess.run([str(bridge), 'roundtrip'], input=raw, capture_output=True, check=True)
            self.assertEqual(pb.Envelope.FromString(completed.stdout), message)
            if index < 3:
                self.assertTrue(message.sensor_config.HasField('enabled'))
                self.assertFalse(message.sensor_config.enabled)
        (OUT / 'mqttx_results.json').write_text(json.dumps(result, indent=2))
        print(f"PASS: {result['checks']} MQTTX function checks using {result['runtime']}; C/Python binary compatibility", flush=True)

    def test_mqttx_model_function(self):
        command = ['node', str(Path(__file__).with_name('mqttx_model_test.cjs'))]
        if args.mqttx_asar:
            command.append(str(extract_runtime(args.mqttx_asar)))
        result = json.loads(subprocess.check_output(command, text=True))
        bridge = ROOT / 'build/protocol_host' / ('bridge.exe' if os.name == 'nt' else 'bridge')
        for fixture in result['fixtures']:
            raw = base64.b64decode(fixture)
            message = pb.Envelope.FromString(raw)
            self.assertIn(message.WhichOneof('body'), ('model_control', 'model_selection'))
            decoded = subprocess.check_output([str(bridge), 'roundtrip'], input=raw)
            self.assertEqual(pb.Envelope.FromString(decoded), message)
        print(f"PASS: {result['checks']} MQTTX ModelControl checks using {result['runtime']}; C/Python compatibility", flush=True)

    def test_mqttx_ml_telemetry_decode(self):
        for name in ('a', 'b', 'status'):
            if not (ROOT / f'build/mqtt_host/ml_{name}.pb').is_file():
                self.fail('Run tests/mqtt_host/run.py first to capture actual C ML publications')
        command = ['node', str(Path(__file__).with_name('mqttx_ml_test.cjs'))]
        if args.mqttx_asar:
            command.append(str(extract_runtime(args.mqttx_asar)))
        result = json.loads(subprocess.check_output(command, text=True))
        self.assertEqual(result['messages'], 3)
        (OUT / 'mqttx_ml_previews.json').write_text(json.dumps(result, indent=2) + '\n')
        print(f"PASS: 3 C ML publications decoded to Pretty JSON; exact uint64/bytes/enums/false and Model Function receive via {result['runtime']}", flush=True)


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]], verbosity=2)
