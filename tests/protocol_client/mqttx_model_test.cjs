const fs = require('node:fs'), path = require('node:path'), vm = require('node:vm');
const assert = require('node:assert/strict'), protobuf = require('protobufjs');
const root = path.resolve(__dirname, '../..');
const script = fs.readFileSync(path.join(root, 'tools/protocol/mqttx_model.js'), 'utf8');
const Envelope = protobuf.parse(fs.readFileSync(path.join(root, 'protocol/aai16.proto'), 'utf8'),
  {keepCase: true}).root.lookupType('aai16.Envelope');
const installed = process.argv[2] ? require(process.argv[2]) : null;
let ticks = 0, checks = 0;
function execute(input, clock = () => 1793180000000 + Math.floor(ticks++ / 4), type = 'publish') {
  const sandbox = {Date: {now: clock}, execute: fn => fn(input, type).toString()};
  return installed ? new installed.VM({timeout: 10000, sandbox, eval: false, wasm: false}).run(new installed.VMScript(script)) :
    vm.runInNewContext(script, sandbox, {timeout: 10000});
}
function encode(text) {
  const object = JSON.parse(text), error = Envelope.verify(object);
  assert.equal(error, null);
  return Buffer.from(Envelope.encode(Envelope.create(object)).finish());
}
const fixtures = [], a = JSON.parse(fs.readFileSync(path.join(root, 'models/motion_model_a/manifest.json')));
const b = JSON.parse(fs.readFileSync(path.join(root, 'models/motion_model_b/manifest.json')));
for (const operation of [6, 7, 4, 5]) {
  const r = {model_control: {operation}};
  if (operation < 6) Object.assign(r, {boot_id: 42, generation: 1}), r.model_control.expected_model_epoch = 0;
  if (operation === 4) r.model_control.target = {family: 2, version: b.model_version, source: 2, bank: 0,
    sha256: b.artifacts['model_int8.tflite'].sha256};
  const output = execute(JSON.stringify(r));
  fixtures.push(encode(output).toString('base64')); checks++;
  const retry = execute(JSON.stringify({retry: JSON.parse(output)}), () => {throw Error('Retry clock');});
  assert.deepEqual(encode(retry), encode(output)); checks++;
}
const back = {boot_id: 42, generation: 1, model_control: {operation: 4, expected_model_epoch: 1,
  target: {family: 1, source: 1, version: a.model_version, sha256: a.artifacts['model_int8.tflite'].sha256}}};
fixtures.push(encode(execute(JSON.stringify(back))).toString('base64')); checks++;
const good = JSON.stringify(back);
for (const input of ['{}', 'null', '[1]', '{', '{"model_control":{"operation":4}}',
  '{"model_control":{"operation":6,"operation":5}}', '{"model_control":{"operation":1}}',
  good.replace('"source":1', '"source":1,"bank":0'), good.replace('"generation":1,', ''),
  good.replace('"expected_model_epoch":1', '"expected_model_epoch":-1'),
  good.replace(a.artifacts['model_int8.tflite'].sha256, 'abcd'), good.replace('"family":1', '"family":3')]) {
  assert(execute(input).startsWith('AAI16 MODEL ERROR:')); checks++;
}
assert(execute(good, () => 123).startsWith('AAI16 MODEL ERROR:')); checks++;
const decoded = JSON.stringify({protocol_version: 1, boot_id: 42, model_status: {result: 'RESULT_OK', model_epoch: 1}});
assert.equal(execute(decoded, () => {throw Error('Receive clock');}, 'received'), decoded); checks++;
const simple = [];
for (const [command, input, family] of [
  ['status', {status:true}, null], ['activate_a_initial', {activate:'a'}, 1],
  ['activate_b', {activate:'b'}, 2], ['activate_a', {activate:'a'}, 1], ['rollback', {rollback:true}, null]]) {
  const output = execute(JSON.stringify(input)), message = JSON.parse(output), wire = encode(output);
  assert.equal(message.boot_id, undefined); assert.equal(message.generation, undefined);
  if (command === 'status') assert.equal(message.model_control.operation, 6);
  else {
    assert.equal(message.model_selection.operation, command === 'rollback' ? 5 : 4);
    if (family) assert.equal(message.model_selection.family, family);
    else assert.equal(message.model_selection.family, undefined);
  }
  fixtures.push(wire.toString('base64'));
  const retry = encode(execute(JSON.stringify({retry:message}), () => {throw Error('Retry clock');}));
  assert.deepEqual(retry, wire);
  simple.push({command, wire:wire.toString('base64')}); checks += 2;
}
for (const input of [
  '{"activate":"c"}', '{"activate":"A"}', '{"activate":true}', '{"activate":null}',
  '{"rollback":false}', '{"rollback":1}', '{"status":false}', '{"status":"true"}',
  '{"activate":"a","rollback":true}', '{"status":true,"boot_id":42}',
  '{"activate":"a","activate":"b"}', '{"rollback":true,"\\u0072ollback":true}',
  '{"activate":"b","model_control":{"operation":6}}',
  '{"model_selection":{"operation":4}}', '{"model_selection":{"operation":4,"family":3}}',
  '{"model_selection":{"operation":5,"family":1}}', '{"model_selection":{"operation":6}}',
  '{"boot_id":42,"model_selection":{"operation":5}}',
  '{"model_control":{"operation":6},"model_selection":{"operation":5}}']) {
  assert(execute(input).startsWith('AAI16 MODEL ERROR:')); checks++;
}
console.log(JSON.stringify({checks, fixtures, simple, runtime: installed ? 'installed MQTTX vm2' : 'Node VM'}));
