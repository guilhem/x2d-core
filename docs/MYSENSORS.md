# MySensors serial adapter

Wire format: `node;child;command;echo;type;payload\n`, 115200 baud, MySensors 2.3.
ASCII header fields are unsigned bytes; command 0–4, echo 0–1; payload at most
25 bytes. Input lines are bounded to 63 bytes before LF, CRLF accepted. Invalid
or oversized input is discarded through the next LF and reported. Output is a
4096-byte ring; radio service never waits for a reader. Overflow closes command
admission, cancels pending work and stops active RF at a frame boundary. The
transport must reopen its connection before more requests can be admitted.

| Node | Child | Presentation | Values |
| --- | --- | --- | --- |
| 1 | 1–16 (paired only) | S_COVER (5) | V_STATUS (2), V_UP (29), V_DOWN (30), V_STOP (31) |
| 1 | 17 / 18 | S_BINARY (3) | V_STATUS (2), momentary pair / confirm |
| 1 | 19 | S_CUSTOM (23) | V_VAR1 (24), read-only diagnostic |

Node 1 presents as S_ARDUINO_NODE, with sketch name/version. Cover descriptions
are `X2D <16-digit USB ID> <slot>`. These descriptions establish HA's initial
`cover.x2d_*` IDs; node/child identifiers stay fixed in the same journal.

The adapter handles gateway version, discovery, presentation and heartbeat
requests. These and reads never submit RF. It sends no REQ, state-restoration
request, SmartSleep notification or queued command on reconnect. Only explicit
SET payload `1` on cover UP/DOWN/STOP or switch STATUS can submit an operation.
SET `0` is inert. Broadcast actuator SET, percentages, tilt, stream and
unsupported values cannot reach the radio. Switches always return OFF.

Recognized SET messages requesting an echo receive their exact value with echo
1, before admission; the echo means receipt, even on refusal. Read requests
receive SET snapshots (with the requested echo flag). Radio results/refusals
are separate diagnostic messages. Presentation/internal replies carry no RF
semantics. Malformed and unsupported commands produce diagnostics, not echoes.

HA 2026.9.4 accepts S_COVER/V_STATUS as its binary-state fallback. No DIMMER or
PERCENTAGE value is published, so there is no position feature. STOP stays 1,
UP/DOWN 0 in snapshots; echoes cannot imply measured motion. V_STATUS becomes 0
only after a complete close burst; open/unknown use 1. STOP, USB loss, reboot
and uncertain TX invalidate estimates; `pos_unknown` marks that condition.
Use HA's global `assumed_state` customization documented by ha-x2d.

The common controller permits one pending identity, allocates only unused slots,
requires explicit build authorization, and persists reservations before RF.
Confirmation requires a reserved attempt, no pending/active radio work and human
observation. RP2040 presents immediately; ESPHome owns its own restart policy.
Corrupt flash blocks RF and is never reformatted. The MySensors serial adapter
still publishes its diagnostic without deleting previously known HA children.

The radio queue holds 16 requests with a 30-second waiting limit, enough for
a group command at nominal chip timing. Active bursts get their full encoded
duration plus a one-second watchdog margin; enrollment retains its six-second
two-phase watchdog. STOP still preempts at a frame boundary. Extremely slow
calibration or a stalled backend can exhaust the queue's waiting limit and
report `queue_expired`; expired commands never reserve a counter or transmit.

MySensors does not carry the journal generation. Never reuse a slot; before a
full flash reset, explicitly remove HA's corresponding device and persisted
MySensors node while the integration is stopped. Never restore an old sensor
inventory against a new journal. HA can retain stale displayed availability
when USB disappears; the protocol cannot make that a radio guarantee.
