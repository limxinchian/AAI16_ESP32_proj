// Mirrors MQTTX 1.13.1 sandbox.ts and protobuf.ts. Optionally uses its installed
// vm2 runtime; otherwise Node's fresh VM exercises the same execute() contract.
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
const vm = require('node:vm');
const protobuf = require('protobufjs');
const root = path.resolve(__dirname, '../..');
const script = fs.readFileSync(path.join(root, 'tools/protocol/mqttx_config.js'), 'utf8');
const Envelope = protobuf.parse(fs.readFileSync(path.join(root, 'protocol/aai16.proto'), 'utf8'),
  { keepCase: true }).root.lookupType('aai16.Envelope');
const installed = process.argv[2] ? require(process.argv[2]) : null;
const fixtures = [];
let checks = 0;
function execute(input, clock = Date.now, type = 'publish') {
  const sandbox = { Date: { now: clock }, execute(callback) {
    const output = callback(input, type, undefined); // index is absent on manual sends.
    return output === undefined ? 'undefined' : output === null ? 'null' : output.toString();
  } };
  return installed ? new installed.VM({ timeout: 10000, sandbox, eval: false, wasm: false }).run(new installed.VMScript(script)) :
    vm.runInNewContext(script, sandbox, { timeout: 10000, contextCodeGeneration: { strings: false, wasm: false } });
}
function encode(text) {
  const object = JSON.parse(text);
  const error = Envelope.verify(object);
  if (error) throw new Error(error);
  return Buffer.from(Envelope.encode(Envelope.create(object)).finish());
}
function id(text) {
  const bits = JSON.parse(text).request_id;
  return (BigInt(bits.high >>> 0) << 32n) | BigInt(bits.low >>> 0);
}
let ticks = 0;
const advancing = () => 1793180000000 + Math.floor(ticks++ / 4);
const simple = enabled => JSON.stringify({ sensor_config: { sensor: 2, enabled } });
let previous = 0n;
for (let i = 0; i < 200; ++i) {
  const output = execute(simple(false), advancing);
  assert(id(output) > previous); previous = id(output);
  const binary = encode(output), decoded = Envelope.decode(binary);
  assert(Object.hasOwn(decoded.sensor_config, 'enabled'));
  assert.equal(decoded.sensor_config.enabled, false);
  assert.equal(decoded.protocol_version, 1); assert.equal(decoded.phase, 1);
  if (i < 2) fixtures.push(binary.toString('base64'));
  checks++;
}
for (const enabled of [false, true]) {
  const output = execute(simple(enabled), advancing);
  fixtures.push(encode(output).toString('base64')); checks++;
}
const interval = execute('{"sensor_config":{"sensor":2,"interval_ms":500}}', advancing);
fixtures.push(encode(interval).toString('base64')); checks++;
// An intentional retry never reads the clock or changes any original metadata.
const retry = execute(JSON.stringify({ retry: JSON.parse(interval) }), () => { throw new Error('Retry read clock'); });
assert.deepEqual(encode(retry), encode(interval)); checks++;
const high = JSON.parse(interval); high.request_id = '18446744073709551615';
const maximum = execute(JSON.stringify({ retry: high }));
assert.equal(id(maximum), 18446744073709551615n);
fixtures.push(encode(maximum).toString('base64')); checks++;
for (const input of ['{', 'null', '[]', '{}',
  '{"sensor_config":{"sensor":2,"enabled":null}}',
  '{"sensor_config":{"sensor":2,"enabled":0}}',
  '{"sensor_config":{"sensor":2,"enabled":false,"interval_ms":500}}',
  '{"sensor_config":{"sensor":2,"enabled":false,"enabled":true}}',
  '{"sensor_config":{"sensor":2,"enabled":false,"\\u0065nabled":true}}',
  '{"sensor_config":{"sensor":2,"calibrate":true}}',
  '{"sensor_config":{"sensor":9,"enabled":true}}',
  '{"sensor_config":{"sensor":2,"interval_ms":99}}',
  '{"sensor_config":{"sensor":2,"interval_ms":500.5}}',
  '{"sensor_config":{"sensor":2,"interval_ms":10001}}',
  '{"sensor_config":{"sensor":2,"enabled":true},"boot_id":-1}',
  JSON.stringify({ ...JSON.parse(interval), unsupported: true }),
  JSON.stringify({ retry: { ...JSON.parse(interval), request_id: 9007199254740992 } }),
  JSON.stringify({ retry: { ...JSON.parse(interval), request_id: '18446744073709551616' } }),
  ' '.repeat(4097)]) {
  // Function errors must not throw: MQTTX's caller would fall back to input.
  const output = execute(input, advancing);
  assert(output.startsWith('AAI16 CONFIG ERROR:'));
  assert.throws(() => encode(output)); checks++;
}
assert.throws(() => encode(execute(simple(true), () => 1000))); checks++;
let backwards = 1000;
assert.throws(() => encode(execute(simple(true), () => backwards--))); checks++;
assert.throws(() => encode(execute(simple(true), advancing, 'received'))); checks++;
assert.throws(() => encode(execute(JSON.parse(simple(true)), advancing))); checks++;
// Demonstrate the documented limitation: fresh VMs cannot detect rollback
// BETWEEN messages. This is why the persistent Python allocator is supplied.
let oldClock = 2000, newClock = 1000;
assert(id(execute(simple(true), () => oldClock++)) > id(execute(simple(true), () => newClock++))); checks++;
// Real-clock rapid calls under the same fresh-VM API, not just a mock counter.
previous = 0n;
for (let i = 0; i < 30; ++i) {
  const current = id(execute(simple(true)));
  assert(current > previous); previous = current; checks++;
}
process.stdout.write(JSON.stringify({ checks, runtime: installed ? 'installed MQTTX vm2' : 'node:vm contract', fixtures }));
