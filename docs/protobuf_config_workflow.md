# Automatic Protobuf configuration requests

This is a client-only improvement. The schema, C codec, MQTT manager, transaction
manager, sensor owners, telemetry and ML firmware are unchanged. New requests
still go to `aai16/config/set`; binary ACKs still use `aai16/config/ack`.

## What the firmware requires

`components/mqtt_manager/config_transactions.c` checks its 16-entry history
before the high-water mark. An identical pending request waits for its outcome;
an identical completed request replays its cached ACK. Changed content under an
existing ID returns `RESULT_BUSY`. An uncached ID at or below `high_water_request`
returns `RESULT_STALE_GENERATION`. **New IDs must increase, not merely be unique.**
Admission and queue-full failure can consume an ID; only an actual owner commit
produces successful APPLY. A cached replay can report an older generation: it is
the original transaction outcome, not a promise about the latest sensor state.

Two client choices are provided:

| Workflow | Appropriate use | Guarantee / limit |
| --- | --- | --- |
| MQTTX Custom Function | One synchronous lab publisher, forward-moving PC clock | Automatic metadata and distinct increasing IDs on rapid serial sends; no durable state or cross-session rollback detection |
| Existing Python CLI, enhanced | Persistent IDs, clock rollback, exact saved-request retry | IDs reserved under an OS lock and persisted before publication; still requires coordinated publication order |

## MQTTX Desktop 1.13.1 setup

The installed application at `D:/ESP_MQTTX/MQTTX.exe` reports version 1.13.1.
Its packaged code and corresponding tagged source were inspected; no application
files, connection settings or credentials were changed by this work.

1. Open **Scripts → Function**. Choose JavaScript. Import
   `tools/protocol/mqttx_config.js`, or paste its complete contents into a new
   function. Save it as **AAI16 Prepare Config**. Keep the final
   `execute(prepareConfig);` line. If using the Function Test panel, select
   **Plaintext** there too.
2. Keep the existing schema under **Scripts → Schema**, using the repository's
   unchanged `protocol/aai16.proto`.
3. Open and connect **AAI16 ACK Monitor**. Open the connection menu's **Use Script**
   dialog (the same dialog currently selecting the Protobuf schema).
4. Select these values **together**, then confirm:

   | Setting | Value |
   | --- | --- |
   | Applied to | **Published** |
   | Function | **AAI16 Prepare Config** |
   | Schema Type | **Protobuf** |
   | Schema | Your saved **aai16.proto** |
   | Proto name | **aai16.Envelope** |

5. In the publish editor choose **Plaintext**, topic `aai16/config/set`, QoS **1**,
   Retain **off**. Paste the simplified JSON shown below. Plaintext is intentional:
   the function receives the original text, allowing it to reject duplicate JSON
   keys before they are lost by parsing. The selected Schema subsequently encodes
   the returned JSON text into **binary Protobuf**.
6. Subscribe to `aai16/config/ack`. With **Published** selected, ACKs remain raw
   binary as in the existing setup. Leave **AAI16 Local MQTT** monitoring its
   JSON telemetry; do not select **Received** or **All** for this combined function.

In this MQTTX version the active script selection is shared through application
state, so do not assume a second connection isolates it. Published-only leaves
received JSON telemetry untouched. Publish configurations only from the ACK
Monitor while this selection is active. If you later publish unrelated messages,
remove the script selection first. Recheck these selections after restarting MQTTX.

The actual API is `execute(callback)`, with callback arguments
`(value, msgType, index)`. Manual sends do not provide a durable sequence index.
MQTTX converts the callback result with `.toString()`, so this function returns
`JSON.stringify(envelope)`, not an object. The caller runs Function before Schema.
These details were checked in the tagged
[sandbox](https://github.com/emqx/MQTTX/blob/v1.13.1/src/utils/sandbox.ts),
[publish pipeline](https://github.com/emqx/MQTTX/blob/v1.13.1/src/views/connections/ConnectionsDetail.vue),
[selection dialog](https://github.com/emqx/MQTTX/blob/v1.13.1/src/components/UseScript.vue) and
[Protobuf encoder](https://github.com/emqx/MQTTX/blob/v1.13.1/src/utils/protobuf.ts).

## Before and after

Previously the user maintained the metadata manually:

```json
{"protocol_version":1,"request_id":1004,"phase":1,"sensor_config":{"sensor":2,"enabled":false}}
```

Now use either of these as ordinary independent operations:

```json
{"sensor_config":{"sensor":2,"enabled":false}}
```

```json
{"sensor_config":{"sensor":2,"enabled":true}}
```

Send the same simplified payload again to create another independent request.
An interval example, while GY-511 is enabled:

```json
{"sensor_config":{"sensor":2,"interval_ms":500}}
```

For conditional writes you may additionally supply a numeric `boot_id` at the
root and `expected_generation` inside `sensor_config`. All previous sensor
interval limits apply. Exactly one operation is required; `false` is preserved
by property presence checks. Unknown fields, invalid values and duplicate JSON
keys produce an error instead of a valid configuration.

For illustration, the function might produce this complete encoder input:

```json
{
  "protocol_version": 1,
  "phase": 1,
  "sensor_config": {"sensor": 2, "enabled": false},
  "request_id": {"low": -1771765184, "high": 417507253, "unsigned": true}
}
```

Those bits represent the exact unsigned ID `1793180000001000000`. **Do not copy
this example ID for new operations.** Long-like bits are a protobufjs input
representation, not a schema change. MQTTX calls `verify()` then `create()`;
decimal strings are not accepted there as fixed64 values. Converting a large ID
to a JavaScript Number would lose precision. The implementation uses BigInt for
arithmetic and two exact 32-bit integers at the JSON/encoder boundary.

## Intentional MQTTX retry

Copy the **transformed, published payload** from the original sent-message entry,
including request ID, operation and any boot/generation preconditions. Wrap it:

```json
{"retry": {"protocol_version":1,"phase":1,"request_id":{"low":-1771765184,"high":417507253,"unsigned":true},"sensor_config":{"sensor":2,"enabled":false}}}
```

Use your original payload, not the illustrative values above. The wrapper is
client-only and is removed before encoding. This path does not read the clock or
allocate a new ID; tests verify identical Protobuf bytes. Do not change the
operation while retaining the ID. Return to simplified input for the next
independent operation. MQTTX publish-input history may contain the original
simplified text; replaying that creates a **new** operation, not a retry.

There is no automatic application retry. A broker PUBACK alone is not APPLY.
MQTTX does not correlate the raw ACKs for you; use a decoder or the Python CLI
to inspect request ID, phase, result and generation. The firmware's 16-entry
cache may evict old outcomes; an evicted retry is rejected, never reapplied.
Device reboot clears that history. Include a known `boot_id` precondition when
a retry must not apply to a different boot; an unbound saved request cannot
provide that protection.

## MQTTX ID limits

MQTTX creates a **fresh VM for each invocation**. No persistent globals, filesystem
or localStorage are exposed by this API. A global counter, UUID or random suffix
does not satisfy this firmware's monotonic requirement.

The lab function waits until the wall clock advances to a new millisecond, then
computes `BigInt(tick) * 1000000n`. Serial invocations with a forward-moving clock
cannot use the same tick. This intentionally adds about a millisecond to desktop
preparation; it does not alter device sensor timing. A bounded 200,000-iteration
loop fails if no new tick is observed; a backwards step inside that invocation
also fails. **Date.now() alone is not claimed to guarantee uniqueness.**

This is not a persistent or distributed allocator. Clock rollback between calls,
a restart with an earlier clock, two publisher windows/machines, or another client
raising the firmware high-water mark can cause a collision or stale rejection.
It cannot guarantee a fresh ID in those conditions. Do not use simultaneous
publishers, and use the Python fallback when clock/restart guarantees matter.
Both clients use the same nanosecond-scale ID range, so normal sequential use
with an advancing clock does not introduce a millisecond/nanosecond mismatch.

MQTTX catches thrown Function errors and can continue using the original input.
To prevent that fallback from publishing a misleading valid request, validation
errors return deliberately invalid JSON beginning `AAI16 CONFIG ERROR:`. With
the required Schema selected, encoding fails and publication is aborted. The
Function Test panel shows the detailed reason; the publish dialog may show only
a schema/JSON parse error. Keep Function **and** Schema selected.

## Persistent Python alternative

No new service or application is needed. `config_cli.py` already generates
protocol metadata and handles ACK correlation. It now uses a locked, atomic
local high-water file and saves exact retry bytes before connecting:

```powershell
$py = '.\build\protocol_python\Scripts\python.exe'
$broker = 'YOUR_BROKER_HOSTNAME'
# Existing credentials: AAI16_MQTT_USERNAME / AAI16_MQTT_PASSWORD environment variables.
& $py tools/protocol/config_cli.py gy511 --enabled false --host $broker --tls --port 8883
& $py tools/protocol/config_cli.py gy511 --enabled true --host $broker --tls --port 8883
& $py tools/protocol/config_cli.py gy511 --interval-ms 500 --host $broker --tls --port 8883

# Or the same simplified JSON as MQTTX:
'{"sensor_config":{"sensor":2,"enabled":false}}' | & $py tools/protocol/config_cli.py --input - --host $broker --tls --port 8883

# Exact retry: use the path printed by the original invocation.
& $py tools/protocol/config_cli.py --retry-file 'PATH_PRINTED_BY_CLIENT.pb' --host $broker --tls --port 8883
```

Omit `--host` for offline preparation. `--input path.json` accepts a file instead
of stdin. For a plaintext local broker omit TLS and use port 1883. Existing
`--topic`, `--ack-topic`, `--boot-id`, `--expected-generation` and `--output` remain.

The default store is `tools/protocol/.state/request_ids.json`; requests are saved
under `.state/requests/<id>.pb`. This directory is ignored by Git. IDs are
`max(persisted+1, time_ns, optional_floor+1)`, checked against uint64. OS locking
prevents same-store allocation collisions; contents are flushed and atomically
replaced. Corrupt state fails closed. Restarting the CLI or moving the clock
backwards preserves increasing IDs **while the existing state is retained**.
Gaps from unsent/failed requests are harmless. Retrying a saved file allocates
nothing and preserves all original bytes; operation/metadata overrides are refused.

`--id-state PATH` selects a shared store. Different stores/machines are not
coordinated. Unique allocated IDs do not guarantee MQTT arrival order: use one
publisher with one outstanding operation. A lost/deleted/restored store, another
sender's larger ID or storage loss needs reconciliation with the device. The
optional `--id-floor N` seeds new automatic IDs above a known firmware high-water
value; this is an exceptional recovery step, not ordinary manual ID management.
Expert `--request-id` remains for compatibility but bypasses automatic allocation;
avoid it for new operations. Stored request files accumulate and can be archived;
retain the ID state and any request still awaiting an outcome.

## Tests and manual acceptance

Run the test-only Node dependency installation, then the client tests:

```powershell
npm.cmd ci --prefix tests/protocol_client --ignore-scripts --no-audit --no-fund
& $py tests/protocol_client/run.py --mqttx-asar D:/ESP_MQTTX/resources/app.asar
```

Without `--mqttx-asar`, tests use Node's fresh VM with the same execute contract.
The optional archive mode copies only the installed vm2/acorn runtime to ignored
`build/` files. It does not launch/change MQTTX. Run `tests/protocol_host/run.py`
first if the C codec bridge has not been built. protobufjs 7.5.4 is pinned in the
test package and lockfile; no new firmware or Python dependency was added.

Passed locally: seven client test groups, 259 JavaScript checks using installed
MQTTX 1.13.1's sandbox, protobufjs → Python/C binary round trips including maximum
uint64, explicit false, rapid sends, retries, invalid/duplicate fields and clock
failure cases. Python tests cover restarts, rollback, 12 concurrent allocations,
corrupt state, exact saved retries, and an isolated local MQTT peer with two new
commands plus a cached retry. The 14-group schema suite and live-C MQTT transaction
regressions also pass, including unchanged duplicate/stale/owner-ACK behavior.
No project-broker, UI-click or hardware test is claimed.

Manual acceptance on the configured ACK Monitor:

| Test | Action | Expected |
| --- | --- | --- |
| A | Send the same simplified enabled payload twice | Different exact ID bit pairs; separate APPLY ACKs |
| B | Send false, then true | New IDs; presence retained; acquisition pauses/resumes after owner APPLY |
| C | While enabled, send interval 500 | APPLY follows existing rules; ML reports its existing non-100-ms state |
| D | Wrap the original transformed payload in retry | Same ID/bytes; cached outcome, no second generation increment |
| E | Send both operations, invalid values, unknown/duplicate fields or malformed JSON | Schema/function error; no valid configuration publication |
| F | Inspect captured final bytes with a Protobuf decoder | Envelope fields, fixed64 exact ID, enabled present and false; not JSON text |

For F, outgoing history shows the transformed editor input, so it alone does not
prove wire encoding. Export/capture the raw subscribed configuration payload in
an isolated monitor and decode it, or use the tested Python/C fixtures. Leave
the JSON telemetry connection's receive processing alone. Finally re-enable
GY-511 if needed and restore interval 100; verify fresh Model A predictions.

Files in this change: `tools/protocol/mqttx_config.js`, `request_ids.py`,
`config_cli.py`, `.gitignore`; `tests/protocol_client/` (runner, JavaScript tests,
package manifest/lock and ignore file); this guide and the Stage 2 workflow note.
No firmware/schema edits, flashing or Git history operations were performed.
