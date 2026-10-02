// Host MySensors serial gateway for the Python/Home Assistant PTY tests: the
// real core adapter (ha_x2d::mysensors::Gateway) over simulated flash and radio,
// with stdin/stdout as the byte transport. It parses no MySensors line itself;
// the Python side only forwards bytes and observes RF through the report below.
//
//   mysensors_server [--control-fd=N] [--burst-ms=N] [--paired=N] [--journal=FILE]
//                    [--device-id=HEX16] [--generation=HEX16] [--identity-base=HEX6]
//                    [--seed=N] [--no-tx] [--enrollment] [--authorize=SLOT:SUFFIX:COUNTER]
//                    [--fault=none|unavailable|start|poll]
//
//   --paired=N     slots 1..N confirmed on first use of EMPTY storage (default 16,
//                  0 = leave the journal empty). A populated journal is never
//                  reprovisioned.
//   --journal=FILE flash image kept across server restarts; a missing file is a
//                  new (empty) journal. Without it the flash dies with the process.
//   --identity-base=HEX6  radio identity of slot N is BASE | N<<8 | N (default A00000);
//                  slot N's first counter is 100*N.
//   --burst-ms=N   duration of a full 25-copy burst (default 80).
//   --fault=...    radio faults: unavailable (available() is false), start
//                  (start_burst refuses), poll (the burst reports unknown).
//   --authorize    PairingAuthorization{slot, identity suffix (hex), next counter}.
//
// stdin/stdout carry serial bytes only. EOF on stdin exits after the output is
// flushed. --control-fd is an inherited bidirectional stream socket; without it
// the link is connected at start and can only end with EOF.
//   bridge -> server  'C'  host opened the port: gateway.connected(); echoed.
//                     'D'  link lost: gateway.disconnected(), buffered and unread
//                          input discarded, then echoed. Every stdout byte written
//                          earlier is already in the pipe, so the bridge drains
//                          up to the echo and may then reconnect.
//                     'H'/'R' hold/release the radio (burst progress freezes);
//                          echoed.
//   server -> bridge  'F'  gateway failed (output overflow); sent once per link.
//
// RF report on stderr, one line per event. A decoded burst names the journal
// identity, action byte and counter the radio would have sent:
//   x2d-sim: ready paired=16 generation=0123456789ABCDEF
//   x2d-sim: burst start identity=A00301 action=81 counter=100 copies=25
//   x2d-sim: burst stop requested
//   x2d-sim: burst end completed=25 stopped=0
// The simulated clock is monotonic real time shared by adapter and radio.
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <chrono>
#include <string>

#include "mysensors.h"

using namespace ha_x2d;

namespace {

void report(const char* format, ...) __attribute__((format(printf, 1, 2)));
void report(const char* format, ...) {
  char line[256];
  va_list args;
  va_start(args, format);
  vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  fprintf(stderr, "x2d-sim: %s\n", line);
  fflush(stderr);
}

// Flash that survives restarts: the image is loaded once and every successful
// erase/program is written through. NOR rules as MemoryFlash: programming only
// erased pages.
class FileFlash final : public journal::Flash {
 public:
  explicit FileFlash(const std::string& path) {
    memset(data_, 0xFF, sizeof(data_));
    if (path.empty()) return;
    fd_ = open(path.c_str(), O_RDWR | O_CREAT, 0644);
    if (fd_ < 0) {
      fprintf(stderr, "cannot open the journal file %s\n", path.c_str());
      exit(1);
    }
    const ssize_t got = pread(fd_, data_, sizeof(data_), 0);
    if (got != static_cast<ssize_t>(sizeof(data_))) {  // new or truncated: start erased
      memset(data_, 0xFF, sizeof(data_));
      if (pwrite(fd_, data_, sizeof(data_), 0) != static_cast<ssize_t>(sizeof(data_))) {
        fprintf(stderr, "cannot initialize the journal file %s\n", path.c_str());
        exit(1);
      }
    }
  }
  ~FileFlash() override { if (fd_ >= 0) close(fd_); }
  uint32_t size() const override { return journal::REGION_BYTES; }
  bool read(uint32_t offset, void* out, uint32_t length) override {
    if (offset > journal::REGION_BYTES || length > journal::REGION_BYTES - offset) return false;
    memcpy(out, data_ + offset, length);
    return true;
  }
  bool erase_sector(uint32_t offset) override {
    if (offset % journal::SECTOR_BYTES || offset >= journal::REGION_BYTES) return false;
    memset(data_ + offset, 0xFF, journal::SECTOR_BYTES);
    return write_through(offset, journal::SECTOR_BYTES);
  }
  bool program_page(uint32_t offset, const uint8_t* page) override {
    if (offset % journal::PAGE_BYTES || offset >= journal::REGION_BYTES) return false;
    for (uint32_t i = 0; i < journal::PAGE_BYTES; ++i)
      if (data_[offset + i] != 0xFF) return false;
    memcpy(data_ + offset, page, journal::PAGE_BYTES);
    return write_through(offset, journal::PAGE_BYTES);
  }

 private:
  bool write_through(uint32_t offset, uint32_t length) {
    return fd_ < 0 ||
           pwrite(fd_, data_ + offset, length, offset) == static_cast<ssize_t>(length);
  }
  uint8_t data_[journal::REGION_BYTES];
  int fd_ = -1;
};

// Decodes the first copy of a command burst back to its body (the inverse of
// radio::encode_burst) so the report names what would be on the air.
bool decode_first_copy(const radio::Waveform& wave, radio::ParsedBody& parsed) {
  size_t at = 0;
  bool bit = false;
  auto cell = [&] {
    if (at + 1 >= wave.chips()) return false;
    bit = wave.chip(at) != wave.chip(at + 1);  // biphase mark: a 1 flips mid-bit
    at += 2;
    return true;
  };
  for (int i = 0; i < 8; ++i) if (!cell() || bit) return false;    // preamble zeros
  for (int i = 0; i < 6; ++i) if (!cell() || !bit) return false;   // frame start
  if (!cell() || bit) return false;
  uint8_t body[radio::BODY_BYTES] = {};
  unsigned ones = 0;
  for (size_t i = 0; i < sizeof(body); ++i) {
    for (int j = 0; j < 8; ++j) {
      if (!cell()) return false;
      if (bit) body[i] |= static_cast<uint8_t>(1u << j);
      ones = bit ? ones + 1 : 0;
      if (ones == 5) {  // stuffed zero
        if (!cell() || bit) return false;
        ones = 0;
      }
    }
  }
  return radio::parse_body(body, sizeof(body), &parsed);
}

// Radio backend: a burst advances one whole copy per copy_ms of clock time, and
// not at all while held. request_stop() parks after the next full copy.
class SimRadio {
 public:
  explicit SimRadio(const uint32_t& clock) : now_(clock) {}

  uint32_t copy_ms = 3;
  bool held = false, unavailable = false, fail_start = false, fail_poll = false;

  bool available() const { return !unavailable; }
  bool start_burst(const radio::Waveform& wave, uint32_t chip_ns, uint32_t& started_ms) {
    if (running_ || !chip_ns || !wave.copies()) abort();
    if (fail_start) return false;
    wave_ = &wave;
    running_ = true;
    stop_ = false;
    elapsed_ = 0;
    last_ = now_;
    started_ms = now_;
    radio::ParsedBody parsed;
    if (decode_first_copy(wave, parsed))
      report("burst start identity=%06X action=%02X counter=%u copies=%u",
             static_cast<unsigned>(parsed.identity), parsed.action, parsed.counter, wave.copies());
    else report("burst start undecoded copies=%u", wave.copies());
    return true;
  }
  radio::FrameState poll_burst(uint8_t& completed) {
    if (!running_) abort();
    advance();
    completed_ = done();
    completed = completed_;
    if (fail_poll) { running_ = false; return radio::FrameState::unknown; }
    if (completed_ < (stop_ ? stop_at_ : wave_->copies())) return radio::FrameState::busy;
    running_ = false;
    return radio::FrameState::complete;
  }
  void request_stop() {
    if (!running_) abort();
    advance();
    if (!stop_) report("burst stop requested");
    stop_ = true;
    stop_at_ = done() < wave_->copies() ? done() + 1 : wave_->copies();
  }
  void end_burst() {
    if (running_) abort();
    report("burst end completed=%u stopped=%d", completed_, stop_ ? 1 : 0);
  }

 private:
  void advance() {
    if (!held) elapsed_ += now_ - last_;
    last_ = now_;
  }
  uint8_t done() const {
    const uint32_t copies = elapsed_ / copy_ms;
    return static_cast<uint8_t>(copies < wave_->copies() ? copies : wave_->copies());
  }
  const uint32_t& now_;
  const radio::Waveform* wave_ = nullptr;
  bool running_ = false, stop_ = false;
  uint32_t elapsed_ = 0, last_ = 0;
  uint8_t completed_ = 0, stop_at_ = 0;
};

// Firmware policy: deterministic entropy, so a run is reproducible.
struct Policy {
  uint32_t state = 1;
  uint32_t random_u32() {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
  }
  const char* firmware() { return "x2d-core-mysensors-sim"; }
};

// failed is a method in the JSONL gateway; accept a plain member as well.
template <class G>
auto failed_of(const G& gateway, int) -> decltype(static_cast<bool>(gateway.failed())) {
  return gateway.failed();
}
template <class G>
bool failed_of(const G& gateway, long) { return gateway.failed; }

struct Options {
  int control = -1;
  uint32_t burst_ms = 80, paired = journal::SLOTS, seed = 1;
  uint64_t generation = 0x0123456789ABCDEFull;
  uint32_t identity_base = 0xA00000;
  std::string device_id = "0123456789ABCDEF", journal_file;
  bool tx = true, enrollment = false, unavailable = false, fail_start = false, fail_poll = false;
  PairingAuthorization authorization{};
};

bool number(const std::string& text, uint64_t maximum, int base, uint64_t& out) {
  char* end = nullptr;
  errno = 0;
  const unsigned long long value = strtoull(text.c_str(), &end, base);
  if (text.empty() || *end || errno || value > maximum) return false;
  out = value;
  return true;
}

bool parse(int argc, char** argv, Options& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const size_t eq = arg.find('=');
    const std::string name = arg.substr(0, eq);
    const bool has_value = eq != std::string::npos;
    const std::string value = has_value ? arg.substr(eq + 1) : "";
    uint64_t n = 0;
    if (name == "--no-tx" && !has_value) options.tx = false;
    else if (name == "--enrollment" && !has_value) options.enrollment = true;
    else if (name == "--control-fd" && number(value, 1023, 10, n)) options.control = static_cast<int>(n);
    else if (name == "--burst-ms" && number(value, 600000, 10, n) && n) options.burst_ms = static_cast<uint32_t>(n);
    else if (name == "--paired" && number(value, journal::SLOTS, 10, n)) options.paired = static_cast<uint32_t>(n);
    else if (name == "--seed" && number(value, 0xFFFFFFFFu, 10, n) && n) options.seed = static_cast<uint32_t>(n);
    else if (name == "--generation" && hex16(value.c_str()) && number(value, ~0ull, 16, n) && n) options.generation = n;
    else if (name == "--identity-base" && number(value, 0xFF0000, 16, n) && !(n & 0xFFFF)) options.identity_base = static_cast<uint32_t>(n);
    else if (name == "--device-id") options.device_id = value;
    else if (name == "--journal" && has_value) options.journal_file = value;
    else if (name == "--fault" && value == "none") {}
    else if (name == "--fault" && value == "unavailable") options.unavailable = true;
    else if (name == "--fault" && value == "start") options.fail_start = true;
    else if (name == "--fault" && value == "poll") options.fail_poll = true;
    else if (name == "--authorize") {
      unsigned slot = 0, suffix = 0, counter = 0;
      char extra = 0;
      if (sscanf(value.c_str(), "%u:%x:%u%c", &slot, &suffix, &counter, &extra) != 3 ||
          !slot || slot > journal::SLOTS || suffix > 0xFF) return false;
      options.authorization = {static_cast<uint8_t>(slot), static_cast<uint8_t>(suffix), counter};
    } else return false;
  }
  return true;
}

void provision_paired(journal::Journal& journal, const Options& options) {
  if (journal.open() != journal::StorageState::empty) return;
  for (uint8_t slot = 1; slot <= options.paired; ++slot) {
    journal::NewController fresh;
    fresh.identity = options.identity_base | (uint32_t{slot} << 8) | slot;
    fresh.first_counter = static_cast<uint16_t>(100u * slot);
    fresh.generation = options.generation;
    if (journal.provision(slot, fresh) != journal::Status::ok ||
        journal.confirm(slot) != journal::Status::ok) {
      fputs("cannot provision the simulated journal\n", stderr);
      exit(1);
    }
  }
}

void reply(int control, char byte) {
  if (control >= 0 && write(control, &byte, 1) < 0) {}  // the bridge going away shows as EOF
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parse(argc, argv, options) || !hex16(options.device_id.c_str())) {
    fputs("usage: mysensors_server [--control-fd=N] [--burst-ms=N] [--paired=N] [--journal=FILE]\n"
          "       [--device-id=HEX16] [--generation=HEX16] [--identity-base=HEX6] [--seed=N]\n"
          "       [--no-tx] [--enrollment]\n"
          "       [--authorize=SLOT:SUFFIX:COUNTER] [--fault=none|unavailable|start|poll]\n",
          stderr);
    return 2;
  }
  signal(SIGPIPE, SIG_IGN);
  for (int fd : {1, options.control})
    if (fd >= 0) fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);

  const auto epoch = std::chrono::steady_clock::now();
  uint32_t clock = 0;
  auto update_clock = [&] {
    clock = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::steady_clock::now() - epoch).count());
  };

  FileFlash flash(options.journal_file);
  journal::Journal journal(flash);
  provision_paired(journal, options);
  SimRadio radio(clock);
  radio.copy_ms = options.burst_ms >= 25 ? options.burst_ms / 25 : 1;
  radio.unavailable = options.unavailable;
  radio.fail_start = options.fail_start;
  radio.fail_poll = options.fail_poll;
  Policy policy;
  policy.state = options.seed;
  mysensors::Gateway<SimRadio, Policy> gateway(journal, radio, policy, options.device_id.c_str());
  gateway.begin(options.tx, options.enrollment, 208500, options.authorization);
  char generation[17] = "none";
  journal.generation_hex(generation);
  unsigned paired = 0;
  for (uint8_t slot = 1; slot <= journal::SLOTS; ++slot)
    paired += journal.shutter(slot).state == journal::SlotState::paired;
  report("ready paired=%u generation=%s", paired, generation);

  char rx[256];  // like the firmware: a small window, one line per feed
  size_t rx_used = 0, rx_pos = 0;
  bool eof = false, failure_reported = false, link_up = options.control < 0;
  if (link_up) {
    update_clock();
    gateway.connected();
  }

  auto flush = [&] {
    while (gateway.output_size()) {
      const ssize_t put = write(1, gateway.output_data(), gateway.output_contiguous());
      if (put > 0) gateway.consume_output(static_cast<size_t>(put));
      else if (errno == EINTR) continue;
      else if (errno == EAGAIN) return true;
      else return false;  // the reader is gone
    }
    return true;
  };

  for (;;) {
    pollfd fds[2];
    nfds_t count = 0;
    int in_slot = -1, control_slot = -1;
    if (!eof && !failed_of(gateway, 0) && rx_pos == rx_used) {
      in_slot = static_cast<int>(count);
      fds[count++] = {0, POLLIN, 0};
    }
    if (options.control >= 0) {
      control_slot = static_cast<int>(count);
      fds[count++] = {options.control, POLLIN, 0};
    }
    poll(fds, count, rx_pos < rx_used ? 0 : 1);

    if (control_slot >= 0 && (fds[control_slot].revents & (POLLIN | POLLHUP))) {
      char byte;
      const ssize_t got = read(options.control, &byte, 1);
      if (got == 0) return 0;  // bridge closed the control channel
      if (got == 1 && byte == 'C') {
        update_clock();
        link_up = true;
        gateway.connected();
        reply(options.control, 'C');
      } else if (got == 1 && byte == 'D') {
        update_clock();
        link_up = false;
        gateway.disconnected();
        rx_used = rx_pos = 0;
        failure_reported = false;
        for (;;) {  // bytes of the lost connection still unread on stdin
          pollfd stale{0, POLLIN, 0};
          char junk[4096];
          if (poll(&stale, 1, 0) <= 0 || !(stale.revents & (POLLIN | POLLHUP))) break;
          const ssize_t dropped = read(0, junk, sizeof(junk));
          if (dropped <= 0) {
            eof = eof || dropped == 0;
            break;
          }
        }
        reply(options.control, 'D');
      } else if (got == 1 && (byte == 'H' || byte == 'R')) {
        radio.held = byte == 'H';
        reply(options.control, byte);
      } else if (got == 1) reply(options.control, '?');
    }
    if (in_slot >= 0 && (fds[in_slot].revents & (POLLIN | POLLHUP))) {
      const ssize_t got = read(0, rx, sizeof(rx));
      if (got > 0) {
        rx_used = static_cast<size_t>(got);
        rx_pos = 0;
      } else if (got == 0 || (errno != EINTR && errno != EAGAIN)) eof = true;
    }

    update_clock();
    // Like the firmware loop, nothing is offered to the adapter while the host
    // has the port closed: those bytes are lost, never queued.
    if (!link_up) rx_pos = rx_used;
    else if (rx_pos < rx_used) rx_pos += gateway.feed(rx + rx_pos, rx_used - rx_pos);
    gateway.tick(clock);  // after feed(): the reply is queued before any completion event
    if (!flush()) return 1;
    if (failed_of(gateway, 0) && !failure_reported) {
      failure_reported = true;
      reply(options.control, 'F');
    }
    if (eof && rx_pos == rx_used && !gateway.output_size()) return 0;
  }
}
