#include <cassert>
#include <string>
#include <vector>
#include "mysensors.h"

using namespace ha_x2d;

struct Radio {
  bool healthy = true, running = false, stop = false, finish = false, fault = false;
  uint32_t starts = 0;
  uint8_t copies = 0;
  bool available() const { return healthy; }
  bool start_burst(const radio::Waveform &wave, uint32_t, uint32_t &) {
    assert(!running);
    copies = wave.copies();
    running = true;
    stop = false;
    ++starts;
    return true;
  }
  radio::FrameState poll_burst(uint8_t &completed) {
    assert(running);
    completed = stop ? 1 : finish ? copies : 0;
    return fault ? radio::FrameState::unknown : stop || finish ? radio::FrameState::complete : radio::FrameState::busy;
  }
  void request_stop() { stop = true; }
  void end_burst() { running = false; }
};
struct Policy {
  uint32_t entropy = 42;
  uint32_t random_u32() { return ++entropy; }
  const char *firmware() { return "test"; }
};
struct Rig {
  journal::Journal journal;
  Radio radio;
  Policy policy;
  mysensors::Gateway<Radio, Policy> gateway;
  uint32_t now = 0;
  explicit Rig(journal::MemoryFlash &flash, bool enrollment = false)
      : journal(flash), gateway(journal, radio, policy, "0123456789ABCDEF") {
    gateway.begin(true, enrollment, 208500, enrollment ? PairingAuthorization{1, 0x5A, 0} : PairingAuthorization{});
    gateway.connected();
  }
  std::string drain() {
    std::string out;
    while (gateway.output_size()) {
      out.append(gateway.output_data(), gateway.output_contiguous());
      gateway.consume_output(gateway.output_contiguous());
    }
    return out;
  }
  void feed(const std::string &data) {
    size_t at = 0;
    while (at < data.size()) {
      const size_t used = gateway.feed(data.data() + at, data.size() - at);
      if (!used) break;
      at += used;
    }
  }
  void tick() { gateway.tick(now++); }
  uint32_t next(uint8_t slot = 1) {
    uint32_t value;
    assert(journal.next_counter(slot, &value));
    return value;
  }
};
static void paired(journal::MemoryFlash &flash, uint8_t count = 1) {
  journal::Journal journal(flash);
  assert(journal.open() == journal::StorageState::empty);
  for (uint8_t slot = 1; slot <= count; ++slot) {
    assert(journal.provision(slot, {static_cast<uint32_t>(0x100000 + slot), 10, 1}) == journal::Status::ok);
    assert(journal.confirm(slot) == journal::Status::ok);
  }
  while (journal.maintenance_due()) assert(journal.maintain() == journal::Status::ok);
}
static void has(const std::string &text, const std::string &part) {
  if (text.find(part) == std::string::npos) {
    fprintf(stderr, "Missing [%s] in [%s]\n", part.c_str(), text.c_str());
    abort();
  }
}

static void parser_and_discovery() {
  mysensors::Message message;
  for (const std::string bad : {"", "1;1;1;1;29", "-1;1;1;1;29;1", "256;1;1;1;29;1",
       "1;1;5;1;29;1", "1;1;1;2;29;1", "1;1;1;1;29;\t", "01a;1;1;1;29;1",
       "1;1;1;1;29;12345678901234567890123456789"})
    assert(!mysensors::decode(bad.data(), bad.size(), message));
  const std::string with_nul("1;1;1;1;29;1\0more", 17);
  assert(!mysensors::decode(with_nul.data(), with_nul.size(), message));
  const std::string good = "1;1;1;1;29;1";
  assert(mysensors::decode(good.data(), good.size(), message) && message.type == 29);
  journal::MemoryFlash flash;
  paired(flash, 16);
  Rig rig(flash);
  const auto first = rig.drain();
  assert(!rig.gateway.failed());
  for (unsigned slot = 1; slot <= 16; ++slot) {
    has(first, "1;" + std::to_string(slot) + ";0;0;5;X2D 0123456789ABCDEF " + std::to_string(slot) + "\n");
    has(first, "1;" + std::to_string(slot) + ";1;0;2;1\n");
  }
  const uint32_t writes = flash.programs();
  for (const std::string line : {"0;255;3;0;2;\n", "255;255;3;0;20;\n", "1;255;3;0;19;\n",
       "1;1;2;0;29;1\n", "1;1;2;1;31;\n", "1;19;2;0;24;\n", "1;255;3;0;18;\n",
       "1;1;0;0;5;fake\n", "1;1;4;0;0;anything\n", "255;1;1;1;29;1\n",
       "1;17;1;1;2;0\n", "1;18;1;1;2;0\n", "1;1;1;1;29;0\n"}) {
    for (char byte : line) rig.feed(std::string(1, byte));  // real fragmented USB
    rig.tick();
    rig.drain();
  }
  assert(rig.radio.starts == 0 && flash.programs() == writes);
  rig.feed(std::string(200, 'x') + "\n1;1;2;0;2;\r\n");
  has(rig.drain(), "invalid_message");
  rig.feed("1;1;1;1;29;1;injected\n1;1;1;1;3;100\n1;19;1;1;24;write\n");
  rig.tick();
  has(rig.drain(), "unsupported_command");
  assert(!rig.radio.starts);
}

static void diagnostics() {
  journal::MemoryFlash flash;
  Rig rig(flash);
  rig.drain();
  const struct { const char *message; uint8_t slot; const char *expected; } cases[] = {
    {"association_profile_unqualified", 1, "1:pair_unqualified"},
    {"association_counter_mismatch", 1, "1:pair_counter_mismatch"},
    {"multiple_pending_associations", 0, "multiple_pending"},
    {"identity_generation_failed", 1, "1:identity_failed"},
    {"transmission_disabled", 0, "tx_off,pos_unknown"},
    {"radio_unavailable", 0, "radio_off,pos_unknown"},
    {"ready", 0, "ready,pos_unknown"},
    {"association_pending", 0, "pair_pending,pos_unknown"},
    {"paired", 16, "16:paired,pos_unknown"},
    {"storage_io_error", 1, "1:storage_io_error"},
    {"storage_corrupt", 0, "storage_corrupt"},
    {"ready_extra", 0, "ready_extra"},
  };
  for (const auto &test : cases) {
    rig.gateway.status(test.message, test.slot);
    assert(rig.drain() == std::string("1;19;1;0;24;") + test.expected + "\n");
    assert(!rig.gateway.failed());
  }
}

static void commands_stop_and_reconnect() {
  journal::MemoryFlash flash;
  paired(flash);
  Rig rig(flash);
  rig.drain();
  rig.feed("1;1;1;1;30;1\n");
  auto out = rig.drain();
  has(out, "1;1;1;1;30;1\n");
  has(out, "1;1;1;0;31;1\n");
  assert(out.find("1;1;1;0;2;0\n") == std::string::npos);
  rig.tick();
  assert(rig.radio.starts == 1 && rig.next() == 11);
  rig.radio.finish = true;
  rig.tick();
  out = rig.drain();
  has(out, "1;1;1;0;2;0\n");
  has(out, "1:emitted");
  rig.radio.finish = false;
  rig.feed("1;1;1;0;29;1\n");
  rig.tick();
  rig.feed("1;1;1;0;30;1\n1;1;1;1;31;1\n");
  rig.tick();
  assert(rig.radio.starts == 3 && rig.next() == 13); // close in queue consumed no counter
  rig.radio.finish = true;
  rig.tick();
  has(rig.drain(), "1:pos_unknown");
  rig.radio.finish = false;
  rig.feed("1;1;1;0;29;1\n");
  rig.tick();
  rig.feed("1;1;1;0;30;1\n1;1;1;0;30;");
  rig.gateway.disconnected();
  assert(rig.gateway.output_size() == 0);
  rig.tick();
  const auto starts = rig.radio.starts;
  rig.gateway.connected();
  rig.feed("1\n"); // partial command from the previous connection was discarded
  rig.tick();
  has(rig.drain(), "invalid_message");
  assert(rig.radio.starts == starts && rig.next() == 14);
  // Uncertain TX consumes its reservation and invalidates the previous estimate.
  rig.feed("1;1;1;0;30;1\n");
  rig.tick();
  rig.radio.fault = true;
  rig.tick();
  has(rig.drain(), "rf_fault,pos_unknown");
  assert(rig.next() == 15);
}

static void pairing_power_loss_and_corruption() {
  journal::MemoryFlash flash;
  {
    Rig rig(flash, true);
    assert(rig.drain().find(";0;0;5;") == std::string::npos);
    rig.feed("1;18;1;1;2;1\n");
    has(rig.drain(), "no_pending_association");
    rig.feed("1;17;1;1;2;1\n1;17;1;1;2;1\n1;18;1;1;2;1\n");
    auto out = rig.drain();
    has(out, "radio_busy");
    has(out, "1;17;1;0;2;0\n");
    has(out, "1;18;1;0;2;0\n");
    rig.tick();
    assert(rig.next() == 2 && rig.radio.starts == 1);
  }
  {
    Rig rig(flash, true);
    rig.drain();
    rig.tick();
    assert(rig.radio.starts == 0 && rig.next() == 2);
    rig.feed("1;18;1;1;2;1\n");
    has(rig.drain(), "1;1;0;0;5;X2D 0123456789ABCDEF 1\n");
    rig.feed("1;18;1;1;2;1\n");
    has(rig.drain(), "no_pending_association");
    assert(rig.radio.starts == 0);
  }
  flash.raw()[0] ^= 0xFF;
  const auto writes = flash.programs();
  Rig broken(flash, true);
  has(broken.drain(), "storage_corrupt");
  broken.feed("1;1;1;1;29;1\n1;17;1;1;2;1\n1;18;1;1;2;1\n");
  broken.tick();
  has(broken.drain(), "storage_corrupt");
  assert(broken.radio.starts == 0 && flash.programs() == writes);
}

static void slow_host() {
  journal::MemoryFlash flash;
  paired(flash);
  Rig rig(flash);
  rig.drain();
  rig.feed("1;1;1;1;29;1\n");
  rig.tick();
  for (unsigned i = 0; i < 1000 && !rig.gateway.failed(); ++i)
    rig.feed("1;1;2;0;2;\n"); // host never reads; cannot block radio service
  assert(rig.gateway.failed());
  rig.tick();
  assert(!rig.radio.running && rig.next() == 11);
  has(rig.drain(), "serial_overflow");
  rig.gateway.disconnected();
  rig.gateway.connected();
  rig.drain();
  rig.tick();
  assert(!rig.gateway.failed() && rig.radio.starts == 1);
}

int main() {
  parser_and_discovery();
  diagnostics();
  commands_stop_and_reconnect();
  pairing_power_loss_and_corruption();
  slow_host();
  puts("mysensors: framing, discovery, echoes, STOP, reconnect, pairing and bounded buffers passed");
}
