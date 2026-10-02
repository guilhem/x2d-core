# X2D Core

Portable C++17 components for an X2D shutter gateway: radio codec, durable
counter journal, STOP queue, radio scheduler, CC1101 control and a JSONL v2
server. Firmware supplies flash, radio, identity, session, clock and authorization.
The core contains no Arduino, RP2040, Home Assistant or direct serial I/O.

The JSONL module alone uses **ArduinoJson 7.4.3**. Include `radio_codec.h`,
`journal.h`, `tx_queue.h` or `radio_runtime.h` without it. `gateway.h` adds the
command protocol and bounded output buffering. All instances have one loop
owner; keep the gateway and waveforms alive while radio output is active.

The [gateway contract](docs/GATEWAY_PROTOCOL.md) and
[radio format](docs/RADIO_PROTOCOL.md) are canonical here. RF transmission is
default-deny. Counters are reserved durably, STOP has priority, and reconnects
never replay commands. The journal format is unchanged from ha-x2d `a0f98d0`;
firmware retains ownership of its reserved flash mapping.

## Native checks

Obtain the pinned ArduinoJson amalgamated header, then run:

```sh
mkdir -p build/include
curl -fL https://github.com/bblanchon/ArduinoJson/releases/download/v7.4.3/ArduinoJson-v7.4.3.h -o build/include/ArduinoJson.h
echo 'ab5fbb8268b846b5f4bc5a5fee11bb2c96f7b8b846f5bef6540afb6a9cc76a5b  build/include/ArduinoJson.h' | sha256sum -c -
cmake -S . -B build -DARDUINOJSON_INCLUDE_DIR="$PWD/build/include"
cmake --build build
ctest --test-dir build --output-on-failure
```

Tests include power cuts, corruption, exhausted counters, STOP reserves,
scheduling, gateway transcripts and a journal fixture produced before extraction.
The native `gateway_server` is a simulated-radio byte-stream adapter used by
ha-x2d's real Python/serialx tests over PTY and local TCP. These checks establish
software behavior, not physical RF or motor qualification.

## Firmware use

Use this repository as a pinned dependency. Arduino metadata is in
`library.properties`; [ha-x2d](https://github.com/guilhem/ha-x2d) consumes it as
`lib/x2d-core` through a local `dir:` entry in its Arduino CLI profiles.
Implement the flash and radio interfaces and the gateway hooks, keep servicing
the runtime while disconnected, and drain only writable output bytes.

This extraction prepares interfaces for a future ESPHome adapter. It includes
no ESPHome component, ESP32 driver or embedded network server.

Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE), including the original
rolling transform attribution.
