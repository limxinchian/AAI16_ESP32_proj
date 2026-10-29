// Match MQTTX 1.13.1's schema decode -> toJSON -> receive formatting pipeline.
// Inputs are actual payloads captured at the production C MQTT SDK boundary.
const fs = require('node:fs'), path = require('node:path'), vm = require('node:vm');
const assert = require('node:assert/strict'), protobuf = require('protobufjs');
const root = path.resolve(__dirname, '../..');
const Envelope = protobuf.parse(fs.readFileSync(path.join(root, 'protocol/aai16.proto'), 'utf8'),
  {keepCase: true}).root.lookupType('aai16.Envelope');
const script = fs.readFileSync(path.join(root, 'tools/protocol/mqttx_model.js'), 'utf8');
const installed = process.argv[2] ? require(process.argv[2]) : null;
const previews = [];
for (const name of ['a', 'b', 'status']) {
  const wire = fs.readFileSync(path.join(root, `build/mqtt_host/ml_${name}.pb`));
  assert.throws(() => JSON.parse(wire.toString('utf8')));
  const decoded = Envelope.decode(wire);
  assert.equal(Envelope.verify(decoded), null);
  // Repeated floats may be emitted packed or unpacked by different encoders;
  // Protobuf promises the same decoded values, not identical serialization.
  assert.deepEqual(Envelope.decode(Envelope.encode(decoded).finish()).toJSON(), decoded.toJSON());
  const pretty = JSON.stringify(decoded.toJSON(), null, 2), value = JSON.parse(pretty);
  const p = value.ml_prediction;
  assert.equal(value.boot_id, 0x12345678); assert.equal(value.generation, 2);
  assert.equal(p.uptime_us, ((1n << 54n) + 5000123n).toString());
  assert.equal(p.model.family, name === 'b' ? 'MODEL_B' : 'MODEL_A');
  assert.equal(p.model.source, name === 'b' ? 'SOURCE_STORED' : 'SOURCE_BUNDLED');
  assert.equal(p.model_epoch, name === 'b' ? 7 : 0);
  assert.equal(p.model.sha256, Buffer.from(Array.from({length: 32}, (_, i) => i)).toString('base64'));
  assert.equal(p.model_ready, name !== 'status');
  if (name === 'status') {
    assert.equal(p.status, 'MOTION_INIT_FAILED'); assert.equal(p.classification, 'CLASS_UNKNOWN');
    for (const key of ['output_int8', 'logits', 'first_sequence', 'window_start_us', 'inference_us'])
      assert(!Object.hasOwn(p, key));
  } else {
    assert.equal(p.status, 'MOTION_PREDICTION');
    assert.equal(p.classification, name === 'b' ? 'CLASS_REST' : 'CLASS_MOVING');
    assert.equal(p.output_int8, 'gAA='); assert.deepEqual(p.logits, [-1.25, 2.5]);
    assert.equal(p.first_sequence, 0xfffffff8); assert.equal(p.last_sequence, 11);
    assert.equal(p.inference_us, '123'); assert.equal(p.clipped, 3);
  }
  // Keep simplified model commands usable with Use Script: All; decoded
  // telemetry passes through the existing Model Function in either display mode.
  for (const input of [JSON.stringify(value), value]) {
    const sandbox = {execute: fn => fn(input, 'received').toString()};
    const result = installed ? new installed.VM({timeout: 10000, sandbox, eval: false, wasm: false}).run(new installed.VMScript(script)) :
      vm.runInNewContext(script, sandbox, {timeout: 10000});
    assert.deepEqual(JSON.parse(result), value);
  }
  previews.push(value);
}
console.log(JSON.stringify({messages: previews.length, runtime: installed ? 'installed MQTTX vm2' : 'Node VM', previews}));
