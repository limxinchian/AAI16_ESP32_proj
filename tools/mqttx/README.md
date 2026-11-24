# MQTTX view script (optional)

`aai16_view.js` is a script for MQTTX's "Run Script" feature that decodes the nodes' binary Protobuf payloads for display.

Most people do not need it: `tools/json_bridge_all.py` publishes a readable JSON mirror of every node topic under
`aai16/<node>/json/...` and accepts JSON on `aai16/<node>/json_in/<topic>`, with no script in MQTTX.
See [docs/mqttx_setup.md](../../docs/mqttx_setup.md).
