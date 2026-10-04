#include <x2d/controller.h>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace x2d;
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
static uint32_t next_counter(journal::Journal &journal, uint8_t slot = 1, bool candidate = false) {
  uint32_t next = 0;
  assert(journal.next_counter(slot, &next, candidate));
  return next;
}
static void ready(Node &node, Radio &radio, journal::Journal &journal) {
  for (uint32_t i = 0; journal.state() == journal::StorageState::initializing && i < 100; ++i)
    tick(node, radio, i);
  assert(journal.state() == journal::StorageState::ready);
}
static void settle(Node &node, Radio &radio, journal::Journal &journal, uint32_t now) {
  assert(!node.busy());
  for (uint32_t i = 0; journal.maintenance_due() && i < 100; ++i)
    tick(node, radio, now + i);
  assert(!journal.maintenance_due());
}
static void emission(Node &node, Radio &radio, uint32_t now = 0) {
  const auto starts = radio.starts;
  for (uint32_t i = 0; node.busy() && radio.starts == starts && i < 100; ++i)
    tick(node, radio, now + i);
  assert(radio.starts == starts + 1); // claimed attempt survives idle maintenance
  const auto started = radio.now;
  tick(node, radio, started + 1);
  tick(node, radio, started + 2001);
  tick(node, radio, started + 2002);
  assert(!node.busy());
}
static void add(Node &node, Radio &radio, uint8_t slot, uint32_t now = 0) {
  for (uint32_t i = 0; i < 100; ++i) {
    // tick() is the sole maintenance owner. A maintenance refusal is retried
    // before a candidate exists, never after an emission permit was claimed.
    if (node.associate(now)) break;
    assert(i < 99);
    tick(node, radio, now);
  }
  assert(node.pending_slot() == slot);
  emission(node, radio, now);
  bool confirmed = false;
  for (uint32_t i = 0; !confirmed && i < 100; ++i) {
    confirmed = node.confirm(slot);
    if (!confirmed) tick(node, radio, now + 2003 + i);
  }
  assert(confirmed && node.paired(slot));
}
static void first_use_and_runtime_sessions() {
  journal::MemoryFlash flash;
  journal::Journal journal(flash);
  Radio radio;
  Observer observer;
  Node node(journal, radio, observer);
  assert(node.begin(true, true, 208500));
  assert(!node.busy() && !node.pending_slot() && !radio.starts);
  ready(node, radio, journal);
  tick(node, radio, 100);
  assert(!node.busy() && !node.pending_slot() && !radio.starts);
  assert(journal.shutter(1).state == journal::SlotState::unused);
  assert(!node.initialize());
  assert(!node.confirm(1));
  assert(node.associate(0));
  assert(node.busy() && !node.active()); // queue-inclusive OTA quiescence
  const auto candidate = journal.shutter(1);
  assert(candidate.has_candidate && candidate.attempts == 1 && candidate.candidate_incarnation);
  uint32_t identity = 0;
  assert(journal.identity(1, &identity, true) && (identity & 255) == 1);
  assert(!node.associate(0) && observer.last == "radio_busy");
  assert(!node.confirm(1) && !node.paired(1));
  emission(node, radio);
  assert(next_counter(journal, 1, true) == 2 && observer.last == "awaiting_confirmation");
  const auto writes = flash.programs();
  assert(!node.associate(2003) && observer.last == "association_pending");
  assert(flash.programs() == writes);
  assert(node.confirm(1) && journal.shutter(1).in_service);
  assert(journal.shutter(1).incarnation == candidate.candidate_incarnation);
  assert(!node.confirm(1));
  assert(node.command(1, Action::open, 2100));
  tick(node, radio, 2100);
  tick(node, radio, 2101);
  assert(next_counter(journal) == 3);
  add(node, radio, 2, 2200);  // no compiled slot, suffix or counter authorization
  assert(journal.shutter(2).logical_id != journal.shutter(1).logical_id);
}
static void retry_and_power_recovery() {
  journal::MemoryFlash flash;
  uint32_t epoch = 0, identity = 0;
  {
    journal::Journal journal(flash);
    Radio radio;
    Observer observer;
    Node node(journal, radio, observer);
    assert(node.begin(true, true, 208500));
    ready(node, radio, journal);
    assert(node.associate(0));
    epoch = journal.incarnation(1, true);
    assert(journal.identity(1, &identity, true));
    tick(node, radio, 0);  // both reservations committed; interruption before completion
    assert(next_counter(journal, 1, true) == 2);
  }
  {
    journal::Journal journal(flash);
    Radio radio;
    Observer observer;
    Node node(journal, radio, observer);
    assert(node.begin(true, true, 208500));
    tick(node, radio, 100);
    assert(!radio.starts && node.pending_slot() == 1 && !node.paired(1));
    uint32_t preserved = 0;
    assert(journal.identity(1, &preserved, true) && preserved == identity);
    assert(journal.incarnation(1, true) == epoch);
    assert(!node.associate(100));
    assert(node.retry(1, 100));
    emission(node, radio, 100);
    assert(next_counter(journal, 1, true) == 4 && journal.shutter(1).attempts == 2);
    assert(!node.retry(1, 2200));
    assert(node.confirm(1));
  }
  journal::Journal rebooted(flash);
  Radio radio;
  Observer observer;
  Node node(rebooted, radio, observer);
  assert(node.begin(true, true, 208500));
  tick(node, radio, 0);
  assert(!radio.starts && node.paired(1) && !node.pending_slot());
}
static void disconnected_before_reservation() {
  journal::MemoryFlash flash;
  journal::Journal journal(flash);
  Radio radio;
  Observer observer;
  Node node(journal, radio, observer);
  assert(node.begin(true, true, 208500));
  ready(node, radio, journal);
  assert(node.associate(0));
  node.disconnect();
  tick(node, radio, 1);
  assert(!radio.starts && next_counter(journal, 1, true) == 0);
  assert(!node.retry(1, 1) && !node.confirm(1));
  const auto id = journal.shutter(1).logical_id;
  assert(node.cancel(1));
  add(node, radio, 1, 10);
  assert(journal.shutter(1).logical_id > id);
}
static void partial_reservation_requires_cancel() {
  journal::MemoryFlash flash;
  {
    journal::Journal journal(flash);
    Radio radio;
    Observer observer;
    Node node(journal, radio, observer);
    assert(node.begin(true, true, 208500));
    ready(node, radio, journal);
    assert(node.associate(0));
    flash.cut_after(journal::RECORD_BYTES / journal::PAGE_BYTES, 0);  // first reservation commits, second body fails
    tick(node, radio, 0);
    assert(!radio.starts && !node.valid());
  }
  flash.restore_power();
  journal::Journal journal(flash);
  Radio radio;
  Observer observer;
  Node node(journal, radio, observer);
  assert(node.begin(true, true, 208500));
  assert(next_counter(journal, 1, true) == 1);
  assert(!node.retry(1, 1) && !node.confirm(1));
  assert(node.cancel(1));
  add(node, radio, 1, 10);
}
static void replacement_cancel_and_reuse() {
  journal::MemoryFlash flash;
  journal::Journal journal(flash);
  Radio radio;
  Observer observer;
  Node node(journal, radio, observer);
  assert(node.begin(true, true, 208500));
  ready(node, radio, journal);
  add(node, radio, 1);
  const auto original = journal.shutter(1);
  const auto primary_next = next_counter(journal);
  uint32_t old_identity = 0;
  assert(journal.identity(1, &old_identity));
  assert(!node.replace(1, 3000) && !node.retire(1));
  assert(node.service(1, false));
  assert(!node.command(1, Action::open, 3000));
  assert(node.command(1, Action::stop, 3000));  // STOP remains available when disabled
  tick(node, radio, 3000);
  tick(node, radio, 3001);
  assert(node.replace(1, 3100));
  assert(journal.shutter(1).logical_id == original.logical_id);
  assert(journal.shutter(1).incarnation == original.incarnation);
  assert(!node.service(1, true));
  assert(!node.cancel(1) && observer.last == "radio_busy"); // queued candidate drained
  settle(node, radio, journal, 3101);
  assert(node.cancel(1));
  assert(!journal.shutter(1).in_service && !journal.shutter(1).has_candidate);
  uint32_t kept = 0;
  assert(journal.identity(1, &kept) && kept == old_identity);
  assert(next_counter(journal) == primary_next + 1);
  assert(node.service(1, true) && node.service(1, false));
  assert(node.replace(1, 4000));
  const auto replacement_epoch = journal.incarnation(1, true);
  emission(node, radio, 4000);
  assert(node.confirm(1));
  assert(journal.shutter(1).logical_id == original.logical_id);
  assert(journal.incarnation(1) == replacement_epoch && replacement_epoch != original.incarnation);
  assert(next_counter(journal) == 2);
  assert(node.service(1, false));
  settle(node, radio, journal, 6100);
  assert(node.retire(1));
  assert(!node.paired(1));
  add(node, radio, 1, 7000);
  assert(journal.shutter(1).logical_id > original.logical_id);
  assert(journal.incarnation(1) != replacement_epoch);
  const auto events = observer.events.size();
  const auto status = observer.last;
  node.report({{99,0,1,Action::open,false,original.incarnation}, "emitted"});
  assert(observer.events.size() == events && observer.last == status);
}
static void stop_pause_and_disable_drain() {
  journal::MemoryFlash flash;
  journal::Journal journal(flash);
  Radio radio;
  Observer observer;
  Node node(journal, radio, observer);
  assert(node.begin(true, true, 208500));
  ready(node, radio, journal);
  add(node, radio, 1);
  assert(node.command(1, Action::open, 3000));
  tick(node, radio, 3000);
  assert(node.command(1, Action::close, 3001));
  assert(node.command(1, Action::stop, 3001));
  tick(node, radio, 3001);
  tick(node, radio, 3002);
  assert(next_counter(journal) == 4); // queued close consumes nothing
  assert(node.command(1, Action::open, 3010));
  tick(node, radio, 3010);
  assert(!node.service(1, false));
  assert(journal.shutter(1).in_service); // persistent mutation waits for frame boundary
  tick(node, radio, 3011);
  settle(node, radio, journal, 3012);
  assert(!node.busy() && node.service(1, false));
  assert(node.service(1, true));
  assert(node.command(1, Action::open, 3020));
  tick(node, radio, 3020);
  node.pause("update_in_progress");
  assert(!node.command(1, Action::stop, 3020));
  tick(node, radio, 3021);
  const auto starts = radio.starts;
  node.resume();
  tick(node, radio, 3022);
  assert(radio.starts == starts && !node.busy());
}
static void paused_stop_drain_never_maintains() {
  journal::MemoryFlash flash;
  journal::Journal journal(flash);
  Radio radio;
  Observer observer;
  Node node(journal, radio, observer);
  assert(node.begin(true,true,208500));
  ready(node,radio,journal);
  add(node,radio,1);
  journal::Reservation reservation;
  while (!journal.maintenance_due())
    assert(journal.reserve(1,1,false,&reservation) == journal::Status::ok);
  assert(node.command(1,Action::stop,3000));
  tick(node,radio,3000);
  node.pause("ota_busy");
  const auto writes = flash.programs(), erases = flash.erases();
  tick(node,radio,3001);
  assert(!node.busy() && journal.maintenance_due());
  for (uint32_t now : {3002u,3003u,3004u}) tick(node,radio,now);
  assert(flash.programs() == writes && flash.erases() == erases);
  node.resume();
  tick(node,radio,3005);
  assert(flash.programs() > writes || flash.erases() > erases);
}
static void paused_initialization_never_maintains() {
  journal::MemoryFlash flash;
  journal::Journal journal(flash);
  Radio radio;
  Observer observer;
  Node node(journal, radio, observer);
  assert(node.begin(false, false, 208500));
  assert(journal.state() == journal::StorageState::initializing);
  node.pause("ota_busy");
  const auto writes = flash.programs(), erases = flash.erases();
  for (uint32_t now : {1u, 2u, 3u}) tick(node, radio, now);
  assert(flash.programs() == writes && flash.erases() == erases);
  assert(!node.busy() && !node.initialize());
  node.resume();
  ready(node, radio, journal);
  assert(!radio.starts && !node.associate(10));
}
static void mismatched_profile_refuses_enrollment_only() {
  journal::MemoryFlash flash;
  journal::Journal journal(flash);
  Radio radio;
  Observer observer;
  {
    Node node(journal, radio, observer);
    assert(node.begin(true, true, 208500, {1}));
    ready(node, radio, journal);
    add(node, radio, 1);
  }
  Node node(journal, radio, observer);
  assert(node.begin(true, true, 208500, {90}));
  const auto writes = flash.programs(), erases = flash.erases(), starts = radio.starts;
  const auto epoch = journal.incarnation(1), counter = next_counter(journal);
  assert(!node.associate(3000) && observer.last == "association_profile_unqualified");
  assert(!node.retry(1, 3000) && observer.last == "association_profile_unqualified");
  assert(!node.pending_slot() && !node.busy() && !journal.shutter(2).logical_id);
  assert(flash.programs() == writes && flash.erases() == erases && radio.starts == starts);
  assert(journal.incarnation(1) == epoch && next_counter(journal) == counter);
  assert(node.command(1, Action::open, 3001));
  tick(node, radio, 3001);
  tick(node, radio, 3002);
  assert(radio.starts == starts + 1 && next_counter(journal) == counter + 1);
}
static void disabled_enrollment_and_corruption() {
  journal::MemoryFlash flash;
  journal::Journal journal(flash);
  Radio radio;
  Observer observer;
  Node node(journal, radio, observer);
  assert(node.begin(false, false, 208500));
  ready(node, radio, journal);
  assert(!node.associate(0));
  assert(!node.command(1, Action::open, 0));
  assert(!radio.starts);
  flash.raw()[0] ^= 0xFF;
  journal::Journal corrupt(flash);
  Node broken(corrupt, radio, observer);
  assert(!broken.begin(true, true, 208500));
  const auto writes = flash.programs();
  assert(!broken.initialize() && !broken.associate(0));
  tick(broken, radio, 0);
  assert(flash.programs() == writes && !radio.starts);
}
int main() {
  first_use_and_runtime_sessions();
  retry_and_power_recovery();
  disconnected_before_reservation();
  partial_reservation_requires_cancel();
  replacement_cancel_and_reuse();
  stop_pause_and_disable_drain();
  paused_stop_drain_never_maintains();
  paused_initialization_never_maintains();
  mismatched_profile_refuses_enrollment_only();
  disabled_enrollment_and_corruption();
  puts("controller: first use, bounded sessions, recovery, replacement, retirement/reuse and STOP checks passed");
}
