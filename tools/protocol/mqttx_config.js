// MQTTX Desktop 1.13.1: Custom Function -> Protobuf Schema, Published only.
// Return JSON TEXT: execute() calls .toString(), not JSON.stringify(), itself.
// Read docs/protobuf_config_workflow.md before using this lab convenience mode.
// Each invocation has a fresh VM: globals, counters and localStorage cannot
// provide durable IDs. Use config_cli.py for persisted monotonic allocation.
function prepareConfig(value, msgType) {
  const own = (obj, key) => Object.prototype.hasOwnProperty.call(obj, key);
  const fail = (message) => { throw new Error(message); };
  const object = (value, name) => {
    if (!value || typeof value !== 'object' || Array.isArray(value)) fail(name + ' must be an object');
  };
  const keys = (value, allowed, name) => {
    object(value, name);
    if (Object.keys(value).some(key => !allowed.includes(key))) fail('Unsupported ' + name + ' field');
  };
  const uint32 = (value, name) => {
    if (!Number.isInteger(value) || value < 0 || value > 0xffffffff) fail(name + ' must be uint32');
    return value;
  };
  const longBits = id => {
    if (id <= 0n || id > 18446744073709551615n) fail('request_id outside fixed64 range');
    // MQTTX uses protobufjs verify()+create(), which rejects decimal strings.
    // JSON-safe Long-like bits avoid passing a rounded JavaScript Number.
    return { low: Number(id & 0xffffffffn) | 0, high: Number(id >> 32n) | 0, unsigned: true };
  };
  const readId = id => {
    if (typeof id === 'string' && /^[1-9][0-9]{0,19}$/.test(id)) return BigInt(id);
    if (Number.isSafeInteger(id) && id > 0) return BigInt(id);
    keys(id, ['low', 'high', 'unsigned'], 'request_id');
    if (!Number.isInteger(id.low) || !Number.isInteger(id.high) ||
        id.low < -2147483648 || id.low > 0xffffffff ||
        id.high < -2147483648 || id.high > 0xffffffff || id.unsigned !== true) fail('Invalid unsigned Long bits');
    return (BigInt(id.high >>> 0) << 32n) | BigInt(id.low >>> 0);
  };
  const sensorConfig = config => {
    keys(config, ['sensor', 'enabled', 'interval_ms', 'expected_generation'], 'sensor_config');
    const limits = [[0, 0], [2000, 10000], [100, 10000], [200, 2000], [500, 10000],
      [100, 10000], [5000, 60000], [500, 10000], [250, 10000]];
    if (!Number.isInteger(config.sensor) || config.sensor < 1 || config.sensor > 8) fail('Unsupported sensor');
    const enabled = own(config, 'enabled');
    if (enabled === own(config, 'interval_ms')) fail('Specify exactly one of enabled or interval_ms');
    const out = { sensor: config.sensor };
    if (enabled) {
      if (typeof config.enabled !== 'boolean') fail('enabled must be a Boolean');
      out.enabled = config.enabled; // Do not test truthiness: false must be present.
    } else {
      const interval = uint32(config.interval_ms, 'interval_ms');
      if (interval < limits[config.sensor][0] || interval > limits[config.sensor][1]) fail('interval_ms outside sensor limits');
      out.interval_ms = interval;
    }
    if (own(config, 'expected_generation')) out.expected_generation = uint32(config.expected_generation, 'expected_generation');
    return out;
  };
  try {
    if (msgType !== 'publish') fail('Use Published only');
    // Plaintext preserves the source so duplicate JSON keys can be rejected.
    if (typeof value !== 'string' || value.length > 4096) fail('Select Plaintext; input limit is 4096 characters');
    const input = JSON.parse(value);
    const tokens = value.match(/"(?:\\.|[^"\\])*"|[{}\[\],:]/g) || [];
    const stack = [];
    for (let i = 0; i < tokens.length; ++i) {
      const token = tokens[i];
      if (token === '{') stack.push(new Set());
      else if (token === '[') stack.push(null);
      else if (token === '}' || token === ']') stack.pop();
      else if (tokens[i + 1] === ':') {
        const key = JSON.parse(token);
        const keys = stack[stack.length - 1];
        if (keys.has(key)) fail('Duplicate JSON field: ' + key);
        keys.add(key);
      }
    }
    object(input, 'input');
    const retry = own(input, 'retry');
    if (retry) keys(input, ['retry'], 'retry wrapper');
    const source = retry ? input.retry : input;
    keys(source, retry ? ['protocol_version', 'request_id', 'phase', 'boot_id', 'sensor_config'] :
      ['boot_id', 'sensor_config'], 'envelope');
    const config = sensorConfig(source.sensor_config);
    const result = { protocol_version: 1, phase: 1, sensor_config: config };
    if (own(source, 'boot_id')) result.boot_id = uint32(source.boot_id, 'boot_id');
    if (retry) {
      if (source.protocol_version !== 1 || source.phase !== 1) fail('Retry must contain the original v1 VALIDATE envelope');
      result.request_id = longBits(readId(source.request_id));
    } else {
      // One synchronous publisher and a forward-moving wall clock only.
      // Wait for a NEW millisecond, rather than Date.now() alone or randomness:
      // serial invocations cannot share a tick. Bounded loop fails on a stalled
      // clock; a backwards step inside this call also fails. Across calls or
      // restarts, clock rollback cannot be detected in MQTTX's fresh sandbox.
      const start = Date.now();
      if (!Number.isSafeInteger(start) || start <= 0) fail('Invalid system clock');
      let tick = start;
      for (let i = 0; i < 200000 && tick === start; ++i) tick = Date.now();
      if (!Number.isSafeInteger(tick) || tick <= start) fail('Clock did not advance; use the persistent Python client');
      result.request_id = longBits(BigInt(tick) * 1000000n);
    }
    return JSON.stringify(result);
  } catch (error) {
    // MQTTX catches thrown Function errors and falls back to the ORIGINAL input.
    // Return deliberately invalid JSON instead: the required Schema encoder
    // fails and aborts publication, even if the original input was a valid envelope.
    return 'AAI16 CONFIG ERROR: ' + error.message;
  }
}
execute(prepareConfig);
