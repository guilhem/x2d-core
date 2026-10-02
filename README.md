# X2D Core

Portable C++17 X2D shutter control shared by the RP2040 USB dongle and the
ESPHome ESP32-S3 component. No Arduino, ESPHome, JSON or heap dependency.

`controller.h` owns command admission, association, human confirmation,
authorization and diagnostics. `radio_runtime.h` reserves counters durably,
schedules the codec's bursts and gives STOP priority. `journal.h` preserves the
existing on-flash format. Adapters own physical flash mapping, radio output and
transport lifecycle; only ESPHome restarts after confirmation.

`mysensors.h` implements the bounded MySensors USB adapter used by
[ha-x2d](https://github.com/guilhem/ha-x2d). Supply a RadioRuntime-compatible radio
with `available()` and a policy with `random_u32()` and `firmware()`. Initialize
with `begin(transmit, enrollment, chip_ns, authorization)`, notify `connected()` /
`disconnected()`, feed at most one line per loop, call `tick(now_ms)` continuously,
and drain only bytes the transport can currently write. Keep all calls on one
loop owner and all instances alive until the radio stops. RF gates default off
in the firmware adapters. There is no automatic provisioning or command replay.

See the [serial contract](docs/MYSENSORS.md) and [radio format](docs/RADIO_PROTOCOL.md).

## Checks

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Tests cover power cuts, corruption, exhausted counters, STOP reserves, the
shared controller, serial framing, echoes, pairing recovery and a journal
fixture predating extraction. `mysensors_server` exposes the real adapter over
stdin/stdout with simulated flash/radio for ha-x2d's native HA/pymysensors PTY
checks. These establish software behavior, not RF or motor qualification.

Apache-2.0. See [LICENSE](LICENSE) and [NOTICE] for transform attribution.
