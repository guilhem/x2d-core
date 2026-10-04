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
find_package(x2d-core 0.2.0 CONFIG REQUIRED)
```

Configure that consumer with `-DCMAKE_PREFIX_PATH=/path/to/install` and link the
same `x2d::core` target. Arduino library metadata exposes `x2d/controller.h`;
the application still supplies its board adapters and a C++17 toolchain.

## API and ownership

| Header | Responsibility |
| --- | --- |
| `x2d/types.h` | `Action` and the 16-slot bound |
| `x2d/radio_codec.h` | Bodies, rolling transform, strict ordinary-body parsing and packed biphase-mark waveforms |
| `x2d/journal.h` | Durable logical IDs, reusable slots, RF incarnations, candidates and counter reservations through `journal::Flash` |
| `x2d/tx_queue.h` | Bounded queue, expiry and STOP priority |
| `x2d/radio_runtime.h` | `radio::RadioRuntime<Radio, Hooks>`: reservation, burst scheduling and cancellation |
| `x2d/controller.h` | `Controller<Radio, Observer>`: admission, supervised add/replace/retire lifecycle and user confirmation |
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

Runtime transmission/enrollment gates start disabled. Call
`begin(transmit, enrollment, chip_ns, EnrollmentProfile{identity_suffix})` to
choose them. The public default suffix `0x01` is a candidate profile awaiting
motor qualification. A private profile must match the initialized journal's
policy: `enrollment_profile_matches(suffix)` checks this without exporting RF
values. A mismatched profile refuses association before allocating identities;
existing ordinary commands retain their stored identities.

## Add, replace and retire

There are 16 reusable physical slots and one pending association candidate at a
time. `associate(now_ms)` allocates the first free slot, a never-reused logical
ID and a fresh private RF incarnation. No per-shutter recompilation is needed.
Logical IDs run from 2 through 254: 253 additions over a journal's lifetime,
including cancelled initial candidates. The RF allocator walks at most 65,536
prefix positions without wrapping, skips prefix zero and retained exclusions,
and combines each prefix with the initialized suffix. Replacement uses another
RF identity/incarnation without consuming another logical ID. Exhaustion refuses
new allocations; erasing storage is not a safe identity-renewal procedure.

Use `pending_slot()` and `Journal::shutter(slot)` for authoritative state. A
candidate's existence and claimed attempt count persist, but RF jobs and emission
permits do not. Boot never resumes association. The initial gesture reserves
counters 0/1 before either burst; `retry(slot, now_ms)` allows one explicit retry
with counters 2/3. A partial reservation ending at counter 1 cannot retry and
requires cancellation. A restored complete attempt may be confirmed, retried
once when eligible, or cancelled by explicit user action.

Only the user's observation of motor response justifies `confirm(slot)`.
Confirmation targets that candidate, swaps it into the primary binding and
marks it in service. Local `emitted` results prove waveform completion, never
motor reception or position. `service(slot, false)` disables ordinary movement;
STOP remains available on a disabled paired binding.

`replace(slot, now_ms)` requires a disabled paired shutter. It preserves the
logical ID, keeps the old primary identity/counter disabled, and persists a new
candidate separately. Confirmation replaces the primary atomically; cancellation
drops the candidate and leaves the old primary disabled. `cancel(slot)` on an
initial candidate frees its slot without recycling its logical or RF IDs.
`retire(slot)` requires a disabled paired shutter without a candidate; its public
view becomes empty (`logical_id == 0`). A later addition may reuse the physical
slot but always receives a fresh logical ID. Adapters can therefore preserve an
HA device on explicit replacement and prevent old automations from targeting a
newly added shutter after retirement.

Lifecycle changes require idle RF. Cancellation or disabling during queued or
active work requests draining and returns a refusal; tick until idle, then repeat
the user action. Idle maintenance can also defer a mutation. These refusals do
not authorize automatic RF retry.

## Persistence and upgrade

`Flash` must map exactly 64 KiB outside application images and filesystems,
with 256-byte pages and 4096-byte sectors. The mapping and OTA partition layout
remain application-owned and unchanged. Journal v2 uses two 32 KiB banks, each
with 42 records. A record contains a 512-byte CRC-protected body, programmed and
read back page by page, followed by a separately verified 256-byte commit. The
remaining 512 bytes hold a downgrade fence.

`open()` distinguishes `empty`, `legacy` (valid v1), `initializing`, `ready`,
`full` and `corrupt`. On erased storage the controller starts initialization.
A valid v1 image requires the explicit one-time `initialize()` controller action
(or `Journal::initialize(generation, identity_seed, suffix)` for direct users).
This discards all v1 associations/counters and retains their RF identities solely
as permanent allocator exclusions: existing motors must be paired again. It
commits the v2 initialization marker and exclusions into the other bank before
idle `maintain()` erases the v1 bank and finalizes readiness. Power loss resumes
initialization with RF gated off. No v1 bindings are migrated. A downgrade fence
makes v1 readers fail closed after completion; reintroduced stale v1 records
are refused by v2. Unknown versions and corrupt data are never autoformatted.

A reservation is durable before RF; repeated copies share that reservation.
Cancellation, failed RF, disconnection and reboot never roll counters back or
replay jobs. Intact uncommitted successor bodies conservatively burn allocator
claims and counters, without enabling a new binding. Storage I/O faults require
journal recovery and a fresh runtime/controller. Simultaneous damage to body and
commit, or externally restoring an older complete flash image, is outside the
recovery guarantee.

Every `TxJob` captures a nonzero `incarnation`; runtime admission, reservation
and emission recheck it so work for an old binding cannot reach a replacement
or a reused physical slot. Direct consumers must capture
`Journal::incarnation(slot, candidate)` when admitting work and pass it to
`reserve(slot, command, critical, out, incarnation, candidate)`. A successful
`Reservation` also carries that incarnation. The default zero incarnation is a
trusted low-level primitive escape hatch; it is never used by the runtime.
`provision()`/primitive `confirm()` remain available for backend consumers,
with a bounded 32-entry exclusion history for arbitrary supplied RF identities;
they do not migrate v1 storage or bypass a pending lifecycle candidate.

Counter `0xFFFF` and the last 16 records of the active bank are reserved for STOP.
Ordinary writes stop at that reserve; 16 further critical reservations can serve
one STOP per registered slot. Maintenance becomes due at 32 free records, giving
idle bank preparation room before the reserve is reached. Run it while idle;
each step erases at most one sector or writes one snapshot/fence. STOP bypasses
idle maintenance, takes queue priority and preempts movement at a full-frame
boundary. This is bounded capacity, not a guarantee of motor response or position;
an already committed peripheral frame can add latency.

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
initialization/upgrade power cuts, allocator history, slot reuse, replacement,
retirement, corruption, counter exhaustion, queue/STOP behavior, runtime faults,
supervised association and simulated CC1101 SPI/GPIO. A small external consumer
runs the same source against both `add_subdirectory` and a relocated installed
package, compiles every public header alone, checks inherited C++17, and checks
that parent tests remain untouched. GitHub Actions runs these checks on pull
requests and pushes to `main`, using GCC in Release mode and Clang in Debug mode
with AddressSanitizer and UndefinedBehaviorSanitizer. Assertions remain active in
Release mode. Native checks establish software behavior; RF/motor/flash hardware
and embedded toolchains require separate validation.

## Releases

`library.properties` is the single source of the library version; CMake reads it
and requires `X.Y.Z` without leading zeros. Update this version and merge with
passing CI before creating a matching `vX.Y.Z` tag and publishing its GitHub
release. Both a tag push and release publication rerun the checks on the tagged
commit. A release created directly is checked after publication.

To check a proposed tag locally, configure with `-DX2D_RELEASE_TAG=v0.2.0`,
using the intended version. A mismatched tag fails configuration. GitHub's
source ZIP and tar.gz archives contain the complete header-only library.

Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE) for transform attribution.
