#pragma once

// Test support only: simulated flash, radio and firmware policy for
// check_gateway and gateway_server. The JSON server itself is gateway.h; nothing
// here parses or writes protocol lines. Drivers update \`clock\` (the one
// millisecond basis shared by the radio and the hooks) before calling the gateway.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include "gateway.h"

namespace sim {
using namespace ha_x2d;

#define SIM_REQUIRE(condition)                                                  \
  do {                                                                          \
    if (!(condition)) {                                                         \
      fprintf(stderr, "%s:%d: requirement failed: %s\n", __FILE__, __LINE__, #condition); \
      abort();                                                                  \
    }                                                                           \
  } while (0)

constexpr const char* DEVICE_ID = "0123456789ABCDEF";
constexpr const char* SESSION = "FEDCBA9876543210";
constexpr const char* FIRMWARE = "x2d-core-sim";
constexpr uint64_t GENERATION = 0x0123456789ABCDEFull;  // reads back as "0123456789ABCDEF"
constexpr uint32_t IDENTITY[] = {0, 0x1234AB, 0x5678CD};  // by shutter_id
constexpr uint16_t FIRST_COUNTER[] = {0, 100, 200};

// Opens the journal; fully erased storage gets slots 1 and 2 provisioned and
// confirmed, which is how a paired installation looks to the gateway.
inline void open_paired(journal::Journal& journal, bool pair = true) {
  const journal::StorageState state = journal.open();
  if (!pair || state != journal::StorageState::empty) return;
  for (uint8_t slot = 1; slot <= 2; ++slot) {
    journal::NewController fresh;
    fresh.identity = IDENTITY[slot];
    fresh.first_counter = FIRST_COUNTER[slot];
    fresh.generation = GENERATION;
    SIM_REQUIRE(journal.provision(slot, fresh) == journal::Status::ok);
    SIM_REQUIRE(journal.confirm(slot) == journal::Status::ok);
  }
}

// Radio backend: a burst advances one whole copy per copy_ms of clock time, and
// not at all while held. request_stop() parks after the next full copy.
class Radio {
 public:
  struct Burst {
    uint32_t started_ms;
    uint8_t copies, completed;
    bool stopped;
    radio::Waveform wave;
  };
  explicit Radio(const uint32_t& clock) : now_(clock) {}

  uint32_t copy_ms = 10;
  bool held = false, fail_start = false, fail_poll = false, record = false;
  unsigned started = 0, stops = 0;
  std::vector<Burst> bursts;  // only while record

  bool running() const { return running_; }

  bool start_burst(const radio::Waveform& wave, uint32_t chip_ns, uint32_t& started_ms) {
    SIM_REQUIRE(!running_ && chip_ns && wave.copies() && copy_ms);
    if (fail_start) return false;
    wave_ = &wave;
    running_ = true;
    stop_ = false;
    elapsed_ = 0;
    last_ = now_;
    completed_ = 0;
    ++started;
    started_ms = now_;
    if (record) bursts.push_back({now_, wave.copies(), 0, false, wave});
    return true;
  }
  radio::FrameState poll_burst(uint8_t& completed) {
    SIM_REQUIRE(running_);
    advance();
    completed_ = done();
    completed = completed_;
    if (fail_poll) { running_ = false; return radio::FrameState::unknown; }
    if (completed_ < (stop_ ? stop_at_ : wave_->copies())) return radio::FrameState::busy;
    running_ = false;  // final chip elapsed and output parked
    return radio::FrameState::complete;
  }
  void request_stop() {
    SIM_REQUIRE(running_);
    advance();
    if (!stop_) ++stops;
    stop_ = true;
    stop_at_ = done() < wave_->copies() ? done() + 1 : wave_->copies();
  }
  void end_burst() {
    SIM_REQUIRE(!running_);
    if (record && !bursts.empty()) {
      bursts.back().completed = completed_;
      bursts.back().stopped = stop_;
    }
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

// Firmware policy. Defaults describe the commands-only build: radio verified,
// command capability only. enroll=true reproduces the private supervised trial
// (slot 1, expected next counter, fixed suffix) with deterministic "entropy".
struct Hooks {
  explicit Hooks(const uint32_t& clock) : clock(clock) {}
  const uint32_t& clock;
  bool tx = true, enroll = false;
  uint8_t trial_slot = 1, suffix = 0x5A;
  uint32_t trial_next = 0;
  RadioStatus radio_status{true, 0, 0x14, 1};
  const char* firmware_string = FIRMWARE;

  uint32_t now_ms() { return clock; }
  bool tx_available() { return tx; }
  bool enrollment_allowed() { return enroll; }
  RadioStatus probe_radio() { return radio_status; }
  const char* firmware() { return firmware_string; }
  bool profile(const TxJob& job, radio::TxProfile& profile) {
    profile.copies = job.enrollment ? 24 : 25;
    return true;
  }
  bool authorize_provision(const journal::Journal&, uint8_t shutter_id) {
    return shutter_id == trial_slot;
  }
  bool authorize_pair(const journal::Journal& journal, uint8_t shutter_id) {
    uint32_t next;
    return shutter_id == trial_slot && journal.next_counter(shutter_id, &next) &&
           next == trial_next;
  }
  bool new_controller(const journal::Journal& journal, journal::NewController& fresh) {
    uint32_t identity = 0x00ABCD00u | suffix;
    while (journal.find_identity(identity)) identity += 0x100;
    fresh.identity = identity;
    fresh.first_counter = 0;
    fresh.generation = 0x1122334455667788ull;
    return true;
  }
};

using Gateway = ha_x2d::Gateway<Radio, Hooks>;
}  // namespace sim
