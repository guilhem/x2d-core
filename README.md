# X2D Core

A header-only C++17 library for the observed X2D shutter radio format: codec,
durable rolling-counter journal, STOP-priority scheduler, supervised association
and a CC1101 register driver. It uses the standard library without heap
allocation or Arduino dependencies. Public headers are `<x2d/...>`; public
symbols live in `x2d`.

## Try the codec

Save this as `main.cpp`. It computes and checks a command offline; it does not
transmit anything or provision a radio identity.

```cpp
#include <x2d/radio_codec.h>

int main() {
  x2d::radio::Body body;
  x2d::radio::ParsedBody parsed;
  x2d::radio::Waveform wave;
  if (!x2d::radio::make_command_body(0x123456, x2d::Action::stop, 42, &body) ||
      !x2d::radio::parse_body(body.bytes, body.length, &parsed) ||
      parsed.identity != 0x123456 || parsed.counter != 42 || parsed.action != 0x04 ||
      !x2d::radio::encode_burst(body, 1, &wave) ||
      wave.copies() != 1 || wave.frame_end(0) != wave.chips()) return 1;
  return 0;
}
```

From the library checkout:

```sh
c++ -std=c++17 -Isrc main.cpp -o codec-example
./codec-example
```

These identity/counter values are demonstration inputs. An application must use
its authorized identity and a durable reservation before emitting RF.

## Use with CMake

CMake 3.16 or newer is required. The interface target carries the include path
and C++17 requirement, without imposing warnings, sanitizers or global flags.
With the library checked out at `vendor/x2d-core`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_device LANGUAGES CXX)
add_subdirectory(vendor/x2d-core)
add_executable(my_device main.cpp)
target_link_libraries(my_device PRIVATE x2d::core)
```

To install the headers and relocatable package:

```sh
cmake -S . -B build -DX2D_BUILD_TESTING=OFF
cmake --install build --prefix "$PWD/install"
```

Replace `add_subdirectory(...)` in the consumer with:

```cmake
find_package(x2d-core 0.1 CONFIG REQUIRED)
```

Configure that consumer with `-DCMAKE_PREFIX_PATH=/path/to/install` and link the
same `x2d::core` target. Arduino library metadata exposes `x2d/controller.h`;
the application still supplies its board adapters and a C++17 toolchain.

## API and ownership

| Header | Responsibility |
| --- | --- |
| `x2d/types.h` | `Action` and the 16-slot bound |
| `x2d/radio_codec.h` | Bodies, rolling transform, strict ordinary-body parsing and packed biphase-mark waveforms |
| `x2d/journal.h` | Durable identities, association state and counter reservations through `journal::Flash` |
| `x2d/tx_queue.h` | Bounded queue, expiry and STOP priority |
| `x2d/radio_runtime.h` | `radio::RadioRuntime<Radio, Hooks>`: reservation, burst scheduling and cancellation |
| `x2d/controller.h` | `Controller<Radio, Observer>`: admission, supervised association and authorization |
| `x2d/cc1101.h` | `cc1101::Driver<Bus, Mode>`: checked SPI/register/GPIO operations |

The application owns `Flash`, `Journal`, `Radio`, `Observer` (or runtime `Hooks`)
and their lifetime. References are borrowed; dependencies must outlive the
controller/runtime. Keep runtime instances static or otherwise off small stacks:
they contain two waveforms, roughly 1.1 KiB each. `MemoryFlash` is a 64 KiB native
test backend, not a production persistence adapter.

Call the controller/runtime from one loop owner, never from an ISR or concurrent
transport callbacks. Call `tick(now_ms)` regularly, including while a paused or
disconnected controller finishes cancellation. There are no internal locks or
threads, and callbacks must not reenter the controller/runtime.

The injected `Radio` provides nonblocking `start_burst`, `poll_burst`,
`request_stop` and `end_burst`; the controller also requires `available()`.
`start_burst` returning false guarantees that no chip was sent. The waveform
must remain borrowed until `end_burst`. Completion requires the last full chip
to have elapsed and the carrier to be safely parked; timing faults must report
`unknown`. STOP cancellation occurs at a complete-frame boundary. A backend
with peripheral prefetch must document its cancellation latency.

`Observer` provides `random_u32()`, `status(message, slot)` and
`tx_result(event)`. Supply appropriate entropy for identities/generations, and
record or enqueue events promptly without blocking on transport I/O. Direct
runtime users supply `profile`, `build` and `report` hooks; their contracts are
documented in `radio_runtime.h`.

`Flash` supplies synchronous `size`, `read`, `erase_sector` and `program_page`
operations. They must finish with reliable results before returning. Journal
calls may block for those operations: keep flash work bounded, and account for
its worst-case latency in the owning loop. Erasure is deferred to idle
maintenance; one maintenance step erases at most one sector or writes one
snapshot. The CC1101 driver likewise performs bounded synchronous SPI polls
(up to 2 ms readiness and 20 ms state waits); it is not the waveform backend.

Transports, serialization, session/restart policy, physical flash mapping,
SPI/GPIO implementations and waveform peripherals belong to the application.
Home Assistant, MySensors, RP2040 and ESPHome adapters are outside this library.

## Qualification and persistence

The codec is based on public vectors and receiver captures, not universal X2D
motor qualification. The ordinary parser accepts the observed 12-byte layout;
the burst encoder accepts bounded bodies up to 15 bytes and 32 copies. Enrollment
builds the observed two-stage genuine B gesture. The high-level controller uses
25 copies for commands and 24 for enrollment, with the second enrollment burst
starting 2001 ms after the first. These values do not establish compatibility
with other motors or boards. Calibrate chip timing through `chip_ns` (nominal
208500 ns); do not remove that adjustment for real hardware.

Runtime transmission/enrollment gates start disabled. Controller callers choose
those gates in `begin()`. Association additionally requires explicit
`PairingAuthorization` for one slot, identity suffix and expected next counter.
This is supervised trial authorization, not a qualified RF profile. After an
attempt, only the user's observation of motor response justifies `confirm()`;
`emitted` proves local waveform completion, never motor reception or position.

`Flash` must map exactly 64 KiB outside application images and filesystems,
with 256-byte pages and 4096-byte sectors. The journal's version-1 binary format
uses two 32 KiB banks, body/readback/commit records and CRCs. A reservation is
durable before RF; every repeated copy shares it. Cancellation, failed RF,
disconnection and reboot never roll counters back or replay work. Both enrollment
counters are reserved before either burst. Corruption refuses mutations without
automatic formatting; storage I/O faults require journal recovery and a fresh
runtime/controller. Simultaneous damage to both pages of a record is outside the
recovery guarantee.

Counter `0xFFFF` and the last 16 records of the active bank are reserved for STOP.
This is a bounded reserve, not unlimited STOP capacity. Run maintenance while
idle before further movements. STOP takes queue priority and preempts movement
at a full-frame boundary; an already committed peripheral frame can add latency.
There is no guarantee of motor response or absolute position feedback.

See [the radio format](docs/RADIO_PROTOCOL.md) and the contracts in the public
headers for exact limits.

## Native checks

```sh
cmake -S . -B build -DX2D_BUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
(cd build && ctest --output-on-failure)
```

`X2D_BUILD_TESTING` defaults on for the standalone library and off under
`add_subdirectory`, even if the parent's `BUILD_TESTING` is on. Dependency use
does not enable testing globally. The existing checks cover codec vectors,
power cuts/corruption, counter exhaustion, queue/STOP behavior, runtime faults,
supervised association and simulated CC1101 SPI/GPIO. A small external consumer
runs the same source against both `add_subdirectory` and a relocated installed
package, compiles every public header alone, checks inherited C++17, and checks
that parent tests remain untouched. Independent CI runs these checks with GCC
and Clang. Native checks establish software behavior; RF/motor/flash hardware
and embedded toolchains require separate validation.

Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE) for transform attribution.
