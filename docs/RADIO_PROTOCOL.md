# Observed X2D radio format

This is the format implemented by `src/radio_codec.h`, derived from public
vectors and captures. It does not qualify a new motor, radio board or enrollment
identity. The firmware supplies and authorizes each identity, action and counter.

| Bytes | Meaning |
| --- | --- |
| 0–2 | 24-bit identity, big-endian |
| 3–6 | `01 05 98 22`; meaning of the fixed bytes remains unestablished |
| 7 | Caller-supplied action |
| 8–9 | Encoded rolling word, little-endian |
| 10–11 | Negative sum of bytes 0–9 modulo 65536, big-endian |

The rolling transform uses the identity as key and is reversible for a 16-bit
clear counter. It never increments or persists counters. See `NOTICE` for its
Apache-2.0 attribution to Sven Fabricius.

A burst begins with eight zero bits. Each copy consists of six one bits and a
zero, body bytes least-significant bit first, a zero stuffed after every five
consecutive ones, then eight ones and a zero. Biphase mark represents each bit
by two chips: toggle at the bit boundary, and toggle mid-bit for a one. Chip 1
means carrier on. `frame_end()` identifies complete copy boundaries for STOP.
The waveform accepts up to 32 copies and bodies of 2–15 bytes. The strict
ordinary-body parser accepts only the 12-byte layout and a valid checksum.

The observed enrollment builder makes two bodies. The first is 12 bytes, with
byte 4 changed to `85` and action `02`. The second is 13 bytes, action `20`, with
`07` inserted before the rolling word. Their checksums cover the extended body.
The scheduler reserves both counters before any RF and starts the second burst
2001 ms after the first start, within a 6 s active deadline. Enrollment requires
an explicit firmware authorization and a motor window opened by the user.

The shared gateway maps open/close/STOP to `81`/`82`/`04` and builds the
canonical command and enrollment bodies. Firmware supplies authorization, copy
counts and calibrated chip duration in nanoseconds. The lower-level portable
scheduler also accepts a caller-supplied body builder.
The RP2040 adapter retains its 208500 ns calibration and existing PIO program.

The journal reserves counter 65535 and the last 16 records of its active bank
for STOP. A reservation is durable before RF, and is never rolled back after
cancellation or link loss. Repeated copies share one counter. Maintenance runs
while idle; disconnect cancels queued work and ends an active burst, including
STOP, at the next complete frame. Neither reconnect nor reboot replays commands.
