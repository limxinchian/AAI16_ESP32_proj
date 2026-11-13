# MQTTX view scripts

JavaScript snippets for MQTTX's "Run Script" feature that decode the node's binary Protobuf payloads for display.
`aai16_view_v3.js` is the current one; `aai16_view.js` and `aai16_view_v2.js` are earlier versions kept for reference.
Most people can skip them: `tools/json_bridge_all.py` publishes a readable JSON mirror under `aai16/<node>/json/#`
and accepts JSON on `aai16/<node>/json_in/<topic>`, with no script needed.
