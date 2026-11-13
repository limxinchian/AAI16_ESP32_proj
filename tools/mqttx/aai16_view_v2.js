// AAI16 protobuf viewer for MQTTX (Function, applied to Received, no Schema needed). v2
// Reads raw protobuf bytes, guesses the message type from its shape, returns readable JSON text.
function viewAai16(value, msgType) {
  try {
    if (msgType !== 'received') return value;

    // ---- 1) get the bytes (works for Buffers/typed arrays from any JS realm) ----
    let arr = null;
    if (typeof value === 'string') {
      const compact = value.replace(/\s+/g, '');
      if (compact.charAt(0) === '{') return value;                 // already JSON text
      if (compact.length >= 2 && compact.length % 2 === 0 && /^[0-9a-fA-F]+$/.test(compact)) {
        // MQTTX in Hex mode: lossless hex string
        arr = [];
        for (let k = 0; k < compact.length; k += 2) arr.push(parseInt(compact.substr(k, 2), 16));
      } else {
        if (value.indexOf('�') >= 0) return 'MQTTX passed text with damaged bytes. Set the message format dropdown to Hex. Got: ' + value;
        arr = [];
        for (let k = 0; k < value.length; k++) arr.push(value.charCodeAt(k) & 0xff);
      }
    } else if (value && typeof value === 'object') {
      if (value.type === 'Buffer' && value.data && typeof value.data.length === 'number') arr = Array.prototype.slice.call(value.data);
      else if (typeof value.length === 'number') arr = Array.prototype.slice.call(value);
      else if (typeof value.byteLength === 'number') arr = Array.prototype.slice.call(new Uint8Array(value));
      else {
        const j = JSON.parse(JSON.stringify(value));
        if (j && j.type === 'Buffer' && j.data) arr = j.data; else return JSON.stringify(value, null, 2);
      }
    } else {
      return 'DEBUG: unexpected value type ' + typeof value + ': ' + String(value);
    }
    const b = Uint8Array.from(arr);
    const hex = function (u) { return Array.prototype.map.call(u, function (x) { return ('0' + x.toString(16)).slice(-2); }).join(' '); };

    // ---- 2) parse the protobuf wire format ----
    const f = [];
    let i = 0;
    const varint = function () {
      let r = 0, s = 1;
      for (;;) {
        if (i >= b.length) throw new Error('eof');
        const x = b[i++];
        r += (x & 0x7f) * s;
        if (!(x & 0x80)) return r;
        s *= 128;
      }
    };
    try {
      while (i < b.length) {
        const t = varint(), n = Math.floor(t / 8), w = t % 8;
        if (n === 0) throw new Error('bad tag');
        if (w === 0) f.push({ n: n, w: w, v: varint() });
        else if (w === 2) {
          const l = varint();
          if (i + l > b.length) throw new Error('bad length');
          f.push({ n: n, w: w, v: b.slice(i, i + l) });
          i += l;
        } else if (w === 5) {
          if (i + 4 > b.length) throw new Error('eof');
          f.push({ n: n, w: w, v: new DataView(b.buffer, b.byteOffset + i, 4).getFloat32(0, true) });
          i += 4;
        } else throw new Error('unsupported wire type ' + w);
      }
    } catch (e) {
      return 'Not an AAI16 protobuf message (' + e.message + '): ' + hex(b);
    }

    const get = function (n) { for (let k = 0; k < f.length; k++) if (f[k].n === n) return f[k]; return undefined; };
    const num = function (n) { const x = get(n); return x && x.w === 0 ? x.v : undefined; };
    const text = function (u) { let s = ''; for (let k = 0; k < u.length; k++) s += String.fromCharCode(u[k]); return s; };
    const floats = function (u) {
      const d = new DataView(u.buffer, u.byteOffset, u.length), o = [];
      for (let k = 0; k + 4 <= u.length; k += 4) o.push(Number(d.getFloat32(k, true).toFixed(4)));
      return o;
    };
    const packed = function (u) {
      const o = []; let j = 0;
      while (j < u.length) { let r = 0, s = 1, x; do { x = u[j++]; r += (x & 0x7f) * s; s *= 128; } while (x & 0x80); o.push(r); }
      return o;
    };
    const E = {
      phase: ['unspecified', 'ADMITTED', 'APPLIED', 'REJECTED'],
      kind: ['unspecified', 'MOTION', 'ENVIRONMENT'],
      state: ['NONE', 'STAGED', 'ACTIVE', 'REJECTED', 'SUSPENDED']
    };
    const out = {};
    const put = function (name, n, mapper) { const v = get(n); if (v) out[name] = mapper ? mapper(v) : v.v; };

    let type;
    const first = get(1);
    if (first && first.w === 2 && get(2) && get(2).w === 0 && /^[\x20-\x7e]+$/.test(text(first.v))) {
      type = 'Heartbeat';
      out.node_id = text(first.v);
      put('uptime_ms', 2); put('config_generation', 3);
      const ids = get(4); out.active_sensor_ids = ids ? packed(ids.v) : [];
      put('model_state', 5, function (v) { return E.state[v.v] || v.v; });
      put('model_kind', 6, function (v) { return E.kind[v.v] || v.v; });
      put('model_version', 7); put('model_reason', 8);
      put('readings_ok', 9); put('read_errors', 10); put('overruns', 11);
      put('rules_fired', 12); put('rule_errors', 13); put('free_heap', 14);
      put('rule_generation', 15); put('bridge_generation', 16);
    } else if (get(6) && get(6).w === 2 && get(7) && get(7).w === 0) {
      type = 'InferenceResult';
      put('kind', 1, function (v) { return E.kind[v.v] || v.v; });
      put('model_version', 2); put('generation', 3); put('timestamp_us', 4);
      out.predicted_class = num(5) || 0;
      out.scores = floats(get(6).v);
      put('inference_us', 7);
    } else if (get(6) && get(6).w === 2) {
      type = 'Reading';
      out.sensor_id = num(1); out.generation = num(2) || 0; out.timestamp_us = num(3) || 0;
      out.valid = !!num(4); out.error = num(5) || 0;
      out.values = floats(get(6).v);
      if (get(7)) out.raw_hex = hex(get(7).v);
    } else if (f.every(function (x) { return x.w === 0; }) && num(2) >= 1 && num(2) <= 3 && f.length <= 6) {
      type = 'ApplyAck (if this is a model_status message, field names differ)';
      out.generation = num(1) || 0;
      out.phase = E.phase[num(2)];
      out.reason = num(3) || 0;
      if (num(4)) out.sensor_id = num(4);
      if (num(5)) out.profile_index = num(5);
      if (num(6)) out.detail = num(6);
    } else {
      type = 'unknown message';
      f.forEach(function (x) { out['field_' + x.n] = x.w === 2 ? hex(x.v) : x.v; });
    }
    return JSON.stringify({ message: type, data: out }, null, 2);
  } catch (err) {
    return 'viewAai16 error: ' + (err && err.message ? err.message : String(err)) +
           ' | value type: ' + typeof value +
           ' | constructor: ' + (value && value.constructor ? value.constructor.name : 'n/a');
  }
}

execute(viewAai16);

