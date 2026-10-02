#include "controller.h"
#include <cassert>
#include <cstdio>
#include <string>
#include <vector>

using namespace ha_x2d;

struct Radio {
  bool healthy = true, running = false, stop = false, fail = false;
  uint32_t now = 0, starts = 0;
  uint8_t copies = 0;
  bool available() const { return healthy; }
  bool start_burst(const radio::Waveform &wave, uint32_t, uint32_t &started) {
    assert(!running);
    running = true;
    stop = false;
    copies = wave.copies();
    started = now;
    ++starts;
    return true;
  }
  radio::FrameState poll_burst(uint8_t &completed) {
    assert(running);
    running = false;
    completed = fail ? 0 : stop ? 1 : copies;
    return fail ? radio::FrameState::unknown : radio::FrameState::complete;
  }
  void request_stop() { stop = true; }
  void end_burst() { assert(!running); }
};

struct Observer {
  std::string last;
  uint8_t slot = 0;
  uint32_t random = 100;
  std::vector<radio::TxEvent> events;
  uint32_t random_u32() { return ++random; }
  void status(const char *message, uint8_t id) { last = message; slot = id; }
  void tx_result(const radio::TxEvent &event) { events.push_back(event); }
};
using Node = Controller<Radio, Observer>;

static void tick(Node &node, Radio &radio, uint32_t now) { radio.now = now; node.tick(now); }
static uint32_t next_counter(journal::Journal &journal, uint8_t slot = 1) {
  uint32_t next = 0;
  assert(journal.next_counter(slot, &next));
  return next;
}

static void association_confirmation() {
  journal::MemoryFlash flash;
  {
    journal::Journal journal(flash);
    Radio radio;
    Observer observer;
    Node node(journal, radio, observer);
    assert(node.begin(true, true, 208500, {1, 0x5A, 0}));
    assert(!node.confirm() && observer.last == "no_pending_association");
    assert(node.associate(0));
    assert(!node.associate(0) && observer.last == "radio_busy");
    assert(!node.confirm() && !node.paired(1));
    tick(node, radio, 0);
    assert(radio.starts == 1 && next_counter(journal) == 2);
    // Power disappears after reservation/start; the motor response can only be
    // asserted by the human. Recovery must not automatically emit another pair.
  }
  {
    journal::Journal journal(flash);
    Radio radio;
    Observer observer;
    Node node(journal, radio, observer);
    assert(node.begin(true, true, 208500, {1, 0x5A, 0}));
    tick(node, radio, 100);
    assert(!node.paired(1) && radio.starts == 0 && next_counter(journal) == 2);
    assert(!node.associate(100) && observer.last == "association_counter_mismatch");
    assert(node.confirm() && observer.last == "paired" && observer.slot == 1 && node.paired(1));
    // The adapter owns whatever follows: the core neither pauses nor restarts.
    assert(!node.confirm() && observer.last == "no_pending_association");
    assert(node.command(1, Action::open, 100));
  }
  {
    journal::Journal journal(flash);
    Radio radio;
    Observer observer;
    Node node(journal, radio, observer);
    assert(node.begin(true, false, 208500));
    assert(node.paired(1) && !node.paired(2) && radio.starts == 0);
    assert(node.command(1, Action::open, 0));
    tick(node, radio, 0);
    tick(node, radio, 1);
    assert(radio.starts == 1 && next_counter(journal) == 3);
    assert(observer.events.size() == 1 && !strcmp(observer.events.back().outcome, "emitted"));
  }
}

static void bounded_authorization_and_radio_failure() {
  journal::MemoryFlash flash;
  journal::Journal journal(flash);
  Radio radio;
  Observer observer;
  {
    Node node(journal, radio, observer);
    assert(node.begin(true, true, 208500));
    assert(!node.associate(0) && observer.last == "association_profile_unqualified");
    assert(flash.programs() == 0);
  }
  {
    Node node(journal, radio, observer);
    assert(node.begin(true, true, 208500, {2, 0x5A, 0}));
    assert(!node.associate(0) && flash.programs() == 0);
  }
  {
    Node node(journal, radio, observer);
    assert(node.begin(true, true, 208500, {1, 0x5A, 2}));
    assert(!node.associate(0) && flash.programs() == 0);
  }
  {
    Node node(journal, radio, observer);
    assert(node.begin(true, true, 208500, {1, 0x5A, 0}));
    assert(node.associate(0));
    tick(node, radio, 0);
    radio.fail = true;
    tick(node, radio, 1);
    assert(next_counter(journal) == 2 && !node.paired(1));
    assert(!strcmp(observer.events.back().outcome, "unknown"));
  }
  {
    Radio resumed;
    Node node(journal, resumed, observer);
    assert(node.begin(true, true, 208500, {1, 0x5A, 2}));
    assert(resumed.starts == 0 && node.associate(0));
    tick(node, resumed, 0);
    tick(node, resumed, 1);
    tick(node, resumed, 2001);
    tick(node, resumed, 2002);
    assert(resumed.starts == 2 && next_counter(journal) == 4);
    assert(observer.last == "awaiting_confirmation");
    assert(!node.associate(2003) && observer.last == "association_counter_mismatch");
  }
  {
    Node node(journal, radio, observer);
    assert(node.begin(true, true, 208500, {2, 0x5A, 0}));
    assert(!node.confirm() && observer.last == "association_profile_unqualified");
  }
}

static void pending_slots() {
  journal::MemoryFlash flash;
  journal::Journal journal(flash);
  assert(journal.open() == journal::StorageState::empty);
  assert(journal.provision(1, {0xAA0001, 0, 7}) == journal::Status::ok);
  assert(journal.provision(3, {0xAA0003, 2, 7}) == journal::Status::ok);
  Radio radio;
  Observer observer;
  Node node(journal, radio, observer);
  assert(node.begin(true, true, 208500, {1, 0x5A, 0}));
  assert(node.pending_slot() == Node::MULTIPLE_PENDING && observer.last == "association_pending");
  const auto programs = flash.programs();
  assert(!node.associate(0) && observer.last == "multiple_pending_associations");
  assert(!node.confirm() && observer.last == "multiple_pending_associations");
  assert(flash.programs() == programs && !node.paired(1) && !node.paired(3));
}

static void pending_slot_authorization() {
  journal::MemoryFlash flash;
  journal::Journal journal(flash);
  assert(journal.open() == journal::StorageState::empty);
  assert(journal.provision(3, {0xAA0003, 0, 7}) == journal::Status::ok);
  Radio radio;
  Observer observer;
  {
    Node node(journal, radio, observer);
    assert(node.begin(true, true, 208500, {1, 0x5A, 0}));
    assert(node.pending_slot() == 3);
    assert(!node.associate(0) && observer.last == "association_profile_unqualified" && observer.slot == 3);
    assert(!node.confirm() && observer.last == "association_profile_unqualified" && observer.slot == 3);
  }
  {
    Node node(journal, radio, observer);
    assert(node.begin(true, true, 208500, {3, 0x5A, 0}));
    // The attempt was never prepared: its reservations are the only evidence.
    assert(!node.confirm() && observer.last == "no_association_attempt" && !node.paired(3));
  }
  {
    journal::MemoryFlash other;
    journal::Journal reserved(other);
    assert(reserved.open() == journal::StorageState::empty);
    assert(reserved.provision(3, {0xAA0003, 2, 7}) == journal::Status::ok);
    Node node(reserved, radio, observer);
    assert(node.begin(true, true, 208500, {3, 0x5A, 2}));
    assert(node.confirm() && observer.last == "paired" && observer.slot == 3 && node.paired(3));
    assert(node.pending_slot() == 0 && radio.starts == 0);
  }
}

static void inventory_and_stop() {
  journal::MemoryFlash flash;
  journal::Journal journal(flash);
  assert(journal.open() == journal::StorageState::empty);
  for (uint8_t slot = 1; slot <= MAX_SHUTTERS; ++slot) {
    assert(journal.provision(slot, {static_cast<uint32_t>(0xAA0000 + slot), 10, 1}) == journal::Status::ok);
    if (!(slot % 2)) assert(journal.confirm(slot) == journal::Status::ok);
  }
  Radio radio;
  Observer observer;
  Node node(journal, radio, observer);
  assert(node.begin(true, false, 208500));
  for (uint8_t slot = 1; slot <= MAX_SHUTTERS; ++slot) {
    assert(node.paired(slot) == !(slot % 2));
    if (slot % 2) assert(!node.command(slot, Action::open, 0));
  }
  assert(node.command(2, Action::open, 0));
  tick(node, radio, 0);
  assert(node.command(2, Action::close, 1));
  assert(node.command(2, Action::stop, 1));
  tick(node, radio, 1);
  tick(node, radio, 2);
  assert(radio.starts == 2 && next_counter(journal, 2) == 12);
  assert(next_counter(journal, 4) == 10);  // queued movement cancelled before reservation
  assert(observer.events.back().job.action == Action::stop);
  assert(!strcmp(observer.events.back().outcome, "emitted"));
  // Native API has no session hook: local progress continues while HA is away.
  assert(node.command(4, Action::open, 3));
  tick(node, radio, 3);
  tick(node, radio, 4);
  assert(next_counter(journal, 4) == 11);
  assert(node.command(2, Action::open, 5));
  tick(node, radio, 5);
  node.pause("update_in_progress");
  assert(!node.command(4, Action::close, 5) && observer.last == "update_in_progress");
  tick(node, radio, 6);
  // A paused controller still services the cancelled frame, then goes quiet.
  assert(!node.active() && !strcmp(observer.events.back().outcome, "cancelled") &&
         !strcmp(observer.events.back().error, "session_disconnected"));
  const auto starts = radio.starts;
  node.resume();
  tick(node, radio, 7);
  assert(radio.starts == starts);  // OTA error/resume never replays a command
  assert(node.command(2, Action::close, 8));  // a clean reconnect accepts new work
  tick(node, radio, 8);
  tick(node, radio, 9);
  assert(radio.starts == starts + 1 && !strcmp(observer.events.back().outcome, "emitted"));
}

static void pause_and_disconnect() {
  journal::MemoryFlash flash;
  journal::Journal journal(flash);
  assert(journal.open() == journal::StorageState::empty);
  assert(journal.provision(1, {0xAB0001, 10, 1}) == journal::Status::ok);
  assert(journal.confirm(1) == journal::Status::ok);
  Radio radio;
  Observer observer;
  Node node(journal, radio, observer);
  assert(node.begin(true, true, 208500, {2, 0x5A, 0}));
  node.pause();
  assert(!node.command(1, Action::open, 0) && observer.last == "paused");
  assert(!node.associate(0) && observer.last == "paused");
  assert(!node.confirm() && observer.last == "paused");
  node.resume();
  // A lost session drops queued work without leaving the controller paused.
  assert(node.command(1, Action::open, 0));
  node.disconnect();
  assert(!strcmp(observer.events.back().outcome, "cancelled"));
  tick(node, radio, 0);
  assert(!radio.starts && next_counter(journal) == 10);
  assert(node.command(1, Action::open, 1));
  tick(node, radio, 1);
  assert(radio.starts == 1 && node.active());
  node.disconnect();  // the active frame ends at its boundary, never replays
  tick(node, radio, 2);
  assert(!node.active() && !strcmp(observer.events.back().outcome, "cancelled") &&
         !strcmp(observer.events.back().error, "session_disconnected"));
  assert(next_counter(journal) == 11);
  assert(node.command(1, Action::stop, 3));
  tick(node, radio, 3);
  tick(node, radio, 4);
  assert(radio.starts == 2 && next_counter(journal) == 12);
}

static void storage_failure_and_last_stop() {
  journal::MemoryFlash flash;
  {
    journal::Journal journal(flash);
    assert(journal.open() == journal::StorageState::empty);
    assert(journal.provision(1, {0xAB1234, 65535, 1}) == journal::Status::ok);
    assert(journal.confirm(1) == journal::Status::ok);
    Radio radio;
    Observer observer;
    Node node(journal, radio, observer);
    assert(node.begin(true, false, 208500));
    assert(node.command(1, Action::open, 0));
    tick(node, radio, 0);
    assert(!radio.starts && observer.last == "counter_exhausted");
    assert(node.command(1, Action::stop, 1));
    tick(node, radio, 1);
    tick(node, radio, 2);
    assert(radio.starts == 1 && next_counter(journal) == 65536);
    assert(node.command(1, Action::stop, 3));
    tick(node, radio, 3);
    assert(radio.starts == 1 && observer.last == "counter_exhausted");
  }
  flash.raw()[0] ^= 0xFF;
  journal::Journal journal(flash);
  Radio radio;
  Observer observer;
  Node node(journal, radio, observer);
  assert(!node.begin(true, true, 208500, {1, 0x5A, 0}));
  assert(!node.command(1, Action::stop, 0) && !node.associate(0) && !node.confirm());
  tick(node, radio, 0);
  assert(!radio.starts && observer.last == "storage_corrupt");
}

int main() {
  association_confirmation();
  bounded_authorization_and_radio_failure();
  pending_slots();
  pending_slot_authorization();
  inventory_and_stop();
  pause_and_disconnect();
  storage_failure_and_last_stop();
  puts("controller: pairing recovery, authorization, pending slots, inventory, STOP, pause and storage checks passed");
}
