// MQTTX Desktop: Function + Protobuf Schema, All, Plaintext input/display.
// Separate from mqttx_config.js. See docs/model_activation_phase5b.md.
function prepareModel(value, msgType) {
  // MQTTX decodes received Protobuf BEFORE applying the Function.
  if (msgType === 'received') return typeof value === 'string' ? value : JSON.stringify(value);
  const own = (o, k) => Object.prototype.hasOwnProperty.call(o, k);
  const fail = s => { throw new Error(s); };
  const keys = (o, allowed) => {
    if (!o || typeof o !== 'object' || Array.isArray(o) || Object.keys(o).some(k => !allowed.includes(k)))
      fail('Unexpected object or field');
  };
  const u32 = v => {
    if (!Number.isInteger(v) || v < 0 || v > 0xffffffff) fail('Expected uint32');
    return v;
  };
  const bits = id => {
    if (id <= 0n || id > 18446744073709551615n) fail('Invalid request_id');
    return { low: Number(id & 0xffffffffn) | 0, high: Number(id >> 32n) | 0, unsigned: true };
  };
  const readId = id => {
    if (typeof id === 'string' && /^[1-9][0-9]{0,19}$/.test(id)) return BigInt(id);
    if (Number.isSafeInteger(id) && id > 0) return BigInt(id);
    keys(id, ['low', 'high', 'unsigned']);
    for (const v of [id.low, id.high])
      if (!Number.isInteger(v) || v < -2147483648 || v > 0xffffffff) fail('Invalid Long bits');
    if (id.unsigned !== true) fail('Unsigned request_id required');
    return (BigInt(id.high >>> 0) << 32n) | BigInt(id.low >>> 0);
  };
  try {
    if (msgType !== 'publish' || typeof value !== 'string' || value.length > 4096) fail('Use Published / Plaintext');
    let input = JSON.parse(value);
    const tokens = value.match(/"(?:\\.|[^"\\])*"|[{}\[\],:]/g) || [], stack = [];
    for (let i = 0; i < tokens.length; ++i) {
      const token = tokens[i];
      if (token === '{') stack.push(new Set());
      else if (token === '[') stack.push(null);
      else if (token === '}' || token === ']') stack.pop();
      else if (tokens[i + 1] === ':') {
        const key = JSON.parse(token), used = stack[stack.length - 1];
        if (used.has(key)) fail('Duplicate JSON field');
        used.add(key);
      }
    }
    if (['activate', 'rollback', 'status'].some(k => own(input, k))) {
      keys(input, ['activate', 'rollback', 'status']);
      if (Object.keys(input).length !== 1) fail('Choose exactly one command');
      if (own(input, 'activate')) {
        if (!['a', 'b'].includes(input.activate)) fail('activate must be "a" or "b"');
        input = { model_selection: { operation: 4, family: input.activate === 'a' ? 1 : 2 } };
      } else if (own(input, 'rollback')) {
        if (input.rollback !== true) fail('rollback must be true');
        input = { model_selection: { operation: 5 } };
      } else {
        if (input.status !== true) fail('status must be true');
        input = { model_control: { operation: 6 } }; // Existing read-only API.
      }
    }
    keys(input, own(input, 'retry') ? ['retry'] : ['boot_id', 'generation', 'model_control', 'model_selection']);
    const retry = own(input, 'retry'), source = retry ? input.retry : input;
    if (retry) {
      keys(source, ['protocol_version', 'request_id', 'boot_id', 'generation', 'model_control', 'model_selection']);
      if (source.protocol_version !== 1) fail('Retry must be protocol v1');
    }
    const result = { protocol_version: 1 };
    if (own(source, 'model_selection')) {
      keys(source, retry ? ['protocol_version', 'request_id', 'model_selection'] : ['model_selection']);
      const s = source.model_selection;
      keys(s, ['operation', 'family']);
      if (![4, 5].includes(s.operation) || own(s, 'family') !== (s.operation === 4)) fail('Invalid model selection');
      if (s.operation === 4 && ![1, 2].includes(s.family)) fail('Select Model A or B');
      result.model_selection = { operation: s.operation };
      if (s.operation === 4) result.model_selection.family = s.family;
    } else {
      const c = source.model_control;
      keys(c, ['operation', 'target', 'expected_model_epoch']);
      if (![4, 5, 6, 7].includes(c.operation)) fail('Use activate=4, rollback=5, status=6, list=7');
      const change = c.operation === 4 || c.operation === 5;
      if (own(c, 'target') !== (c.operation === 4) || own(c, 'expected_model_epoch') !== change) fail('Operation fields mismatch');
      const command = { operation: c.operation };
      result.model_control = command;
      if (change) {
        result.boot_id = u32(source.boot_id); result.generation = u32(source.generation);
        command.expected_model_epoch = u32(c.expected_model_epoch);
      } else {
        if (own(source, 'boot_id')) result.boot_id = u32(source.boot_id);
        if (own(source, 'generation')) result.generation = u32(source.generation);
      }
      if (c.operation === 4) {
        const t = c.target;
        keys(t, ['family', 'version', 'sha256', 'source', 'bank']);
        if (![1, 2].includes(t.family) || ![1, 2].includes(t.source) ||
            typeof t.version !== 'string' || !/^[A-Za-z0-9_.-]{1,39}$/.test(t.version)) fail('Invalid model identity');
        if (own(t, 'bank') !== (t.source === 2) || (t.source === 2 && ![0, 1].includes(t.bank))) fail('Stored model needs bank 0/1');
        let sha = t.sha256;
        if (typeof sha === 'string' && /^[a-fA-F0-9]{64}$/.test(sha))
          sha = sha.match(/../g).map(v => parseInt(v, 16));
        if (!Array.isArray(sha) || sha.length !== 32 || sha.some(v => !Number.isInteger(v) || v < 0 || v > 255)) fail('sha256: 64 hex characters or 32 bytes');
        command.target = { family: t.family, version: t.version, sha256: sha, source: t.source };
        if (t.source === 2) command.target.bank = t.bank;
      }
    }
    if (retry) result.request_id = bits(readId(source.request_id));
    else {
      const start = Date.now(); let tick = start;
      for (let i = 0; i < 200000 && tick === start; ++i) tick = Date.now();
      if (!Number.isSafeInteger(start) || start <= 0 || !Number.isSafeInteger(tick) || tick <= start) fail('Clock did not advance; correct the system clock and retry');
      result.request_id = bits(BigInt(tick) * 1000000n);
    }
    return JSON.stringify(result);
  } catch (error) {
    // MQTTX may fall back to original input on thrown errors. Invalid JSON
    // deliberately makes the required Schema encoder abort publication.
    return 'AAI16 MODEL ERROR: ' + error.message;
  }
}
execute(prepareModel);
