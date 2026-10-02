# JSONL protocol v2 — operational X2D gateway

Common contract for firmware, independent Python client and HA. Unverified
radio profiles advertise `tx_enabled:false` and reject pairing/movement.
The passive RX debug sketch has a separate USB product and protocol.

## Transport

A byte stream (USB CDC or serial-over-TCP), UTF-8 JSONL; LF/CRLF, maximum 4096 bytes including LF in both directions.
Device identity and boot session are supplied by the firmware.
Firmware stays silent until a valid hello; disconnect clears handshake and queue. The adapter must notify the core of link loss
and continue ticking the radio runtime while offline. An output-buffer or
serialization failure is latched; the adapter closes the link and notifies
disconnect before reconnecting. No output operation waits on the transport.
No v1 fallback. Device/session/journal generation are uppercase hex16.

`v` is integer 2, `id` integer 1–2147483647 (booleans rejected). Hello has
exactly v/id/op. Every other request has v/id/op/session/args, with an empty
args object when no arguments. Reject unknown fields and invalid types.

```json
{"v":2,"id":1,"op":"hello"}
{"v":2,"id":1,"ok":true,"result":{"product":"ha-x2d","firmware":"0.3.0","device_id":"0123456789ABCDEF","session":"FEDCBA9876543210","max_line_bytes":4096,"max_shutters":16,"capabilities":["status","shutters"]}}
{"v":2,"id":2,"op":"status","session":"FEDCBA9876543210","args":{}}
```

Identifiers above are fictitious. Capabilities describe usable operations,
never hardware proof from compilation. IDs increase within a connection.
The unqualified build exposes only status/shutters: allocating a new slot must
wait for evidence of a valid radio identity format and initial counter. An
existing slot can still be read with provision. No guessed counter is persisted.
A duplicate with identical parsed operation/arguments returns its cached
response without a side effect (up to 16 responses retained);
conflicting/older evicted IDs fail. Requests from old boot sessions fail.
Responses precede completion events. Request timeout 3 s; TX result timeout 8 s.
Cancellation, timeout or invalid peer data closes the session; no automatic retry.
An outstanding command lost during disconnect has an unknown RF result.

The host must keep this USB device awake during transactions. The pinned USB
core reports suspension as a disconnected CDC port; the firmware then cancels
at a frame boundary and discards the session, without rolling counters back.
Use the targeted [Linux power rule](https://github.com/guilhem/ha-x2d/blob/main/firmware/99-ha-x2d-power.rules) and verify
`power/control=on`. Qualify HA OS power behavior separately. Polling status or
extending timeouts is not a substitute for this requirement.

## Operations

| Operation | Arguments | Result |
| --- | --- | --- |
| status | {} | uptime_ms uint32, tx_enabled bool, radio {detected bool, partnum/version/marcstate integer 0–255 when detected else null}, storage {state:empty/ready/corrupt/full, generation:hex16 or null only when empty/corrupt}. |
| shutters | {} | generation and shutters list; each record {shutter_id:integer 1–16, state:pending/paired, last_command:null/open/close/stop}. |
| provision | {shutter_id:1} | Persist a new controller in an unused slot; return existing slot unchanged. Result: shutter record plus generation. No RF/reset. |
| pair | {shutter_id:1} | Qualified enrollment or explicit supervised trial only. Ack {accepted:true,shutter_id:1}, then tx_result. State remains pending. |
| confirm | {shutter_id:1} | Persist paired state after user confirms motor response; return record plus generation. TX result alone never confirms association. |
| command | {shutter_id:1,action:stop} | Actions exactly open/close/stop; verified TX profile and paired slot required. Ack {accepted:true,shutter_id:1}, then final tx_result. |

The user opens the motor's enrollment window with an existing physical
controller and confirms readiness in Home Assistant. `pair` sends only the
new gateway controller's enrollment request. It does not reproduce the existing
controller's buttons or require detection of its opening transmission.
The observed enrollment gesture consists of two trains: a 12-byte press body,
then a 13-byte hold body beginning 2001 ms later. Both get a separate durable
counter before the first RF chip; copies within each train share that counter.
The active enrollment deadline is 6 s; the queue deadline remains 3 s. STOP,
disconnect or expiry before the second train prevents it from starting.

The distributed 0.3.0 build disables RF. Its private supervised trial permits
provision/pair only for slot 1, using seed 0 as an experimental hypothesis.
`HA_X2D_TRIAL_EXPECTED_NEXT_COUNTER` defaults to 0 and accepts only 0 or 2.
With 0, pair requires the next counter to be zero and consumes counters 0/1;
another pair is refused even after reboot. A private build with 2 permits one
manual resume of the same persisted C identity at counters 2/3, refuses a slot
still at next=0, and refuses another pair at next=4 even after reboot.
This bounded resume requires an established USB interruption, effective host
keep-awake, a quiet no-RF digital result of 48 copies, private readback matching
the original C identity and generation, and a new user-confirmed manual window.
The journal retains identity and consumed counters; there is no seed reset,
automatic retry or reprovisioning. USB fail-stop remains active. On this motor,
the resumed C was acknowledged and then physically opened/stopped and
closed/stopped, with A/B still working. General enrollment remains unqualified:
the identity suffix and a clean initial enrollment at counters 0/1 have not
been established for other motors.

The explicit `HA_X2D_COMMANDS_TX=1` build identifies as `0.3.0-commands` and
advertises status/shutters/command. It commands only existing paired journal
slots. It refuses pair, confirm and new provision with `profile_unverified`;
reading an existing slot through provision remains side-effect free. It embeds
no trial suffix and cannot create an identity or reset a counter. Home Assistant
can adopt an unclaimed paired slot directly from inventory, then test and ask
for the observed result. Host keep-awake must be qualified before HA OS use.

Generation belongs to journal, not boot. HA stores generation+slot references,
never rolling counters. Generation change invalidates old references. Slots
are never recycled by deleting HA entities. Corrupt storage rejects mutations,
without formatting. Fresh empty storage initializes only on explicit provision.

## Events and scheduling

One reader routes responses by ID and events by event. Events have v:2,
current session, event, and uint32 seq (with wrap), monotonically increasing.

```json
{"v":2,"session":"FEDCBA9876543210","seq":1,"event":"tx_result","request_id":3,"shutter_id":1,"result":"emitted","completed_copies":25}
```

TX results: emitted/cancelled/expired/unknown/rejected. Emitted means RF burst
executed, not motor reception or position. `completed_copies` is optional,
integer 0–64; it counts only frames whose final chip finished, across both
trains for enrollment. Optional `error`
is one of the application error codes below. A refused admission returns a
negative reply only; a failure after acceptance returns a terminal event.
Events from a disconnected connection are never routed to a new one,
even if the new client reuses request IDs. Validated `rx` events may contain
identity hex6, action open/close/stop, counter uint16 and optional shutter_id;
these private radio fields are excluded from exported diagnostics.

Bounded queue: 16 entries, 3000 ms queue deadline per command. STOP removes
pending movements for its slot, precedes queued movements, and interrupts
repeat bursts at the next complete-frame boundary. Reserve one counter durably
before each logical command; repetitions share it; abandoned reservations stay
consumed. Counter 65535 is reserved for STOP; movements fail before using it.
Keep 16 journal records for STOP (one per slot), coalesce pending STOPs, and run
maintenance at idle before accepting more movements when maintenance is due.
This reserve covers 16 STOPs between maintenances, not unlimited traffic.
Flash erases happen at idle, never on STOP's critical path. No command
replay after disconnect/reboot. Exhausted/corrupt state fails without RF.

## Errors and acceptance

Negative reply: {v:2,id:3,ok:false,error:profile_unverified}; malformed requests
may have id:null. Errors: invalid_request, unsupported_operation, line_too_long,
stale_session, duplicate_conflict, stale_request, storage_corrupt, storage_full,
unknown_shutter, not_paired, counter_exhausted, queue_full, profile_unverified.
Runtime errors additionally include tx_disabled, unqualified_profile,
storage_io_error, maintenance_pending, stop_in_progress, body_refused,
radio_fault, radio_start_failed, stop_preempted.
Queue/cancellation errors: session_disconnected, deadline_expired,
incomplete_burst, enrollment_overlap, queue_expired, queue_cancelled.
Application rejection differs from protocol corruption. Invalid framing/JSON,
unknown response ID or invalid event fields closes client.

Validate separately: RF profile, enrollment, journal power-cut recovery, STOP
latency, and real motor response. PTY/codec tests do not prove those observations.
