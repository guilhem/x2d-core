// Integrated native checks: real journal/queue/codec, no USB or RF hardware.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <utility>
#include <vector>
#include <x2d/radio_runtime.h>

// Codec, journal, queue and runtime are plain C++17, with no JSON dependency.
#ifdef ARDUINOJSON_VERSION
#error "radio/journal/runtime headers must not include ArduinoJson"
#endif

using namespace x2d;
using namespace x2d::journal;

// Decode each scheduled segment independently, including stuffing and footer.
static radio::Body decode_frame(const radio::Waveform& wave, size_t begin, size_t end) {
  assert(begin % 2 == 0 && end % 2 == 0 && begin < end);
  size_t at = begin;
  auto cell = [&]() {
    assert(at + 1 < end);
    assert(!at || wave.chip(at) != wave.chip(at - 1));
    const bool value = wave.chip(at) != wave.chip(at + 1);
    at += 2;
    return value;
  };
  if (!begin) {
    assert(wave.chip(0));
    for (size_t i = 0; i < radio::PREAMBLE_ZERO_BITS; ++i) assert(!cell());
  }
  for (int i = 0; i < 6; ++i) assert(cell());
  assert(!cell());
  radio::Body body{};
  unsigned ones = 0;
  for (size_t i = 0; i < body.length * 8; ++i) {
    const bool value = cell();
    if (value) body.bytes[i / 8] |= static_cast<uint8_t>(1u << (i % 8));
    ones = value ? ones + 1 : 0;
    if (ones == 5) { assert(!cell()); ones = 0; }
    if (i == 63 && body.bytes[4] == 0x05 && body.bytes[7] == 0x20)
      body.length = 13;
  }
  for (int i = 0; i < 8; ++i) assert(cell());
  assert(!cell() && at == end);
  return body;
}

// Inspect COMMITTED flash, not the reservation passed to a fake builder.
static journal::detail::Image durable_image(MemoryFlash& flash) {
  journal::detail::Image newest{};
  for (uint32_t bank = 0; bank < 2; ++bank) {
    for (uint32_t record = 0; record < PAIRS_PER_BANK; ++record) {
      const uint32_t offset = bank * BANK_BYTES + record * RECORD_BYTES;
      journal::detail::Image image;
      if (journal::detail::classify(flash.raw() + offset, flash.raw() + offset + BODY_BYTES,
                                   &image) == journal::detail::PairKind::committed &&
          image.sequence > newest.sequence) newest = image;
    }
  }
  return newest;
}

struct FakeRadio {
  struct Frame {
    radio::Body body;
    radio::ParsedBody parsed;
    size_t begin, end;
    uint32_t programs, chip_ns;
  };
  struct Burst { uint32_t start_ms; uint8_t copies; uint32_t programs, erases; };
  MemoryFlash& flash;
  std::vector<Frame> frames;
  std::vector<Burst> bursts;
  const radio::Waveform* wave = nullptr;
  bool running = false, done = false, fail_start = false, fail_poll = false;
  bool stop_requested = false, parked = false;
  uint8_t completed = 0;
  uint32_t now = 0, chip_ns = 0, programs = 0, erases = 0;
  unsigned ends = 0, stops = 0;

  void next_frame() {
    assert(running && wave && completed < wave->copies());
    const size_t begin = completed ? wave->frame_end(completed - 1) : 0;
    const size_t end = wave->frame_end(completed);
    Frame frame{decode_frame(*wave, begin, end), {}, begin, end,
                flash.programs(), chip_ns};
    const auto& body = frame.body;
    const bool pair = body.bytes[4] == 0x85 || body.length == 13;
    if (!pair) assert(radio::parse_body(body.bytes, body.length, &frame.parsed));
    else {
      const uint8_t header0[] = {0x01, 0x85, 0x98, 0x22, 0x02};
      const uint8_t header1[] = {0x01, 0x05, 0x98, 0x22, 0x20, 0x07};
      const bool phase = body.length == 13;
      assert(!memcmp(body.bytes + 3, phase ? header1 : header0, phase ? 6 : 5));
      const size_t rolling_at = phase ? 9 : 8;
      frame.parsed.identity = (uint32_t{body.bytes[0]} << 16) |
                              (uint32_t{body.bytes[1]} << 8) | body.bytes[2];
      frame.parsed.action = body.bytes[7];
      frame.parsed.rolling_word = body.bytes[rolling_at] |
                                  (body.bytes[rolling_at + 1] << 8);
      frame.parsed.counter = radio::rolling_decode(frame.parsed.rolling_word,
                                                   frame.parsed.identity);
      assert(((body.bytes[body.length - 2] << 8) | body.bytes[body.length - 1]) ==
             radio::body_checksum(body.bytes, body.length));
    }
    Journal committed(flash);
    const auto state = committed.open();
    assert(state == StorageState::ready || state == StorageState::full);
    bool durable = false;
    for (uint8_t id = 1; id <= SLOTS; ++id) {
      uint32_t identity = 0, next = 0;
      if (committed.identity(id, &identity, pair) && identity == frame.parsed.identity) {
        assert(committed.next_counter(id, &next, pair));
        durable = next >= uint32_t{frame.parsed.counter} + 1;
        if (pair) assert(next == uint32_t{frame.parsed.counter} + (body.length == 12 ? 2 : 1));
      }
    }
    assert(durable);  // BOTH pair reservations survive reopening before RF
    assert(flash.programs() == programs && flash.erases() == erases);
    frames.push_back(frame);
  }
  bool start_burst(const radio::Waveform& burst, uint32_t ns, uint32_t& started_ms) {
    started_ms = now;
    assert(!running && !wave && ns);
    if (fail_start) return false;
    wave = &burst;
    chip_ns = ns;
    programs = flash.programs();
    erases = flash.erases();
    completed = 0;
    running = true;
    done = stop_requested = parked = false;
    bursts.push_back({now, burst.copies(), programs, erases});
    next_frame();
    return true;
  }
  radio::FrameState poll_burst(uint8_t& copies) {
    assert(running && wave);
    assert(flash.programs() == programs && flash.erases() == erases);
    copies = completed;
    if (fail_poll) { running = false; return radio::FrameState::unknown; }
    if (!done) return radio::FrameState::busy;
    done = false;  // one full copy completed by the continuous backend
    copies = ++completed;
    if (stop_requested || completed == wave->copies()) {
      running = false;
      parked = true;  // complete includes the final chip duration and safe park
      return radio::FrameState::complete;
    }
    next_frame();  // no runtime call needed between copies
    return radio::FrameState::busy;
  }
  void request_stop() { assert(running); stop_requested = true; ++stops; }
  void end_burst() {
    assert(wave && !running && (parked || fail_poll));
    wave = nullptr;
    ++ends;
  }
};

struct Hooks {
  MemoryFlash& flash;
  std::vector<radio::TxEvent> events;
  std::vector<Reservation> reservations;
  uint8_t copies = 3;
  uint32_t chip_ns = 208500, second_start_ms = 2001;
  bool qualified = true, refuse_body = false;
  int refuse_phase = -1;
  std::vector<uint8_t> phases;
  unsigned enrollment_calls = 0;
  bool profile(const TxJob&, radio::TxProfile& out) {
    out = {copies, chip_ns, second_start_ms};
    return qualified;
  }
  bool build(const TxJob& job, const Reservation& reservation, uint8_t phase,
             radio::Body& body) {
    Journal committed(flash);
    const auto state = committed.open();
    assert(state == StorageState::ready || state == StorageState::full);
    uint32_t next = 0;
    assert(committed.next_counter(job.shutter_id, &next, job.enrollment));
    assert(next == uint32_t{reservation.counter} + (job.enrollment && phase == 0 ? 2 : 1));
    assert(committed.incarnation(job.shutter_id, job.enrollment) == job.incarnation);
    assert(reservation.incarnation == job.incarnation);
    reservations.push_back(reservation);
    phases.push_back(phase);
    if (refuse_body || phase == refuse_phase) return false;
    if (job.enrollment) {
      ++enrollment_calls;
      return radio::make_enrollment_body(reservation.identity, reservation.counter, phase, &body);
    }
    assert(phase == 0);
    // Strict-capture action bytes are caller-owned, never runtime defaults.
    const uint8_t action = job.action == Action::open ? 0x81
                         : job.action == Action::close ? 0x82 : 0x04;
    return radio::make_body(reservation.identity, action, reservation.counter, &body);
  }
  void report(const radio::TxEvent& event) {
    assert(!strcmp(event.outcome, "emitted") || event.error);
    events.push_back(event);
  }
  const radio::TxEvent& event(uint32_t id) const {
    const radio::TxEvent* found = nullptr;
    for (const auto& item : events) if (item.job.request_id == id) {
      assert(!found);  // one terminal outcome per submitted request
      found = &item;
    }
    assert(found);
    return *found;
  }
};

struct Fixture {
  MemoryFlash flash;
  Journal journal{flash};
  FakeRadio radio{flash, {}, {}};
  Hooks hooks{flash, {}, {}, 3, 208500, 2001, true, false, -1, {}};
  radio::RadioRuntime<FakeRadio, Hooks> runtime{journal, radio, hooks};
  explicit Fixture(bool enabled = true, uint16_t first = 100) {
    assert(journal.open() == StorageState::empty);
    assert(journal.initialize(0x123456789ABCDEF0ull, 0xABCD, 1) == Status::ok);
    while (journal.state() == StorageState::initializing) assert(journal.maintain() == Status::ok);
    assert(journal.provision(1, {0x186054, first, 0x123456789ABCDEF0ull}) == Status::ok);
    assert(journal.provision(2, {0x186091, 700, 0x123456789ABCDEF0ull}) == Status::ok);
    assert(journal.confirm(1) == Status::ok && journal.confirm(2) == Status::ok);
    if (enabled) runtime.set_enabled(true);
  }
  bool submit(uint32_t id, uint8_t slot, Action action, uint32_t now = 10,
              bool enrollment = false) {
    if (enrollment) prepare_candidate(slot);
    return runtime.submit({id, 0, slot, action, enrollment, journal.incarnation(slot, enrollment)}, now);
  }
  void prepare_candidate(uint8_t slot = 1) {
    if (journal.shutter(slot).has_candidate) return;
    assert(journal.set_service(slot, false) == Status::ok);
    assert(journal.allocate_candidate(slot, true) == Status::ok);
    assert(journal.claim_attempt(slot, false) == Status::ok);
  }
  void tick(uint32_t now) { radio.now = now; runtime.tick(now); }
  void drain(uint32_t now = 10) {
    for (unsigned i = 0; i < 10000 && (runtime.pending() || runtime.active()); ++i) {
      if (radio.running) radio.done = true;
      tick(now++);
    }
    assert(!runtime.pending() && !runtime.active());
  }
  void settle(uint32_t now = 10) {
    assert(!runtime.active() && !runtime.pending());
    for (unsigned i = 0; journal.maintenance_due() && i < 100; ++i) tick(now);
    assert(!journal.maintenance_due());
  }
  void due() {
    Reservation reservation;
    while (!journal.maintenance_due())
      assert(journal.reserve(1, 1, false, &reservation) == Status::ok);
  }
};

static void check_independent_slots_and_shared_copies() {
  Fixture f;
  const auto programs = f.flash.programs(), erases = f.flash.erases();
  assert(f.submit(1, 1, Action::open));
  assert(f.submit(2, 2, Action::close));
  assert(f.submit(3, 1, Action::close));
  assert(f.hooks.reservations.empty());
  f.drain();
  assert(f.radio.frames.size() == 9 && f.hooks.reservations.size() == 3);
  for (size_t command = 0; command < 3; ++command) {
    const auto& first = f.radio.frames[3 * command];
    assert(first.parsed.counter == (command == 1 ? 700 : 100 + command / 2));
    assert(first.parsed.identity == (command == 1 ? 0x186091u : 0x186054u));
    assert(first.parsed.action == (command == 0 ? 0x81 : 0x82));
    for (size_t copy = 0; copy < 3; ++copy) {
      const auto& frame = f.radio.frames[3 * command + copy];
      assert(!memcmp(first.body.bytes, frame.body.bytes, radio::BODY_BYTES));
      assert(frame.programs == first.programs && frame.chip_ns == 208500);
    }
    assert(!strcmp(f.hooks.event(command + 1).outcome, "emitted"));
    assert(f.hooks.event(command + 1).completed_copies == 3);
  }
  assert(f.flash.programs() == programs + 3 * RECORD_BYTES / PAGE_BYTES &&
         f.flash.erases() == erases && !f.flash.violations());
}

static void check_stop_preemption() {
  {
    Fixture f;
    assert(f.submit(1, 1, Action::stop));
    f.tick(10);
    assert(f.submit(2, 2, Action::stop, 11));
    assert(f.radio.stops == 0);  // finish the first STOP before the other slot
    f.drain(11);
    assert(f.radio.frames.size() == 6);
    for (uint32_t id : {1u, 2u}) {
      assert(!strcmp(f.hooks.event(id).outcome, "emitted"));
      assert(f.hooks.event(id).completed_copies == 3);
    }
  }
  for (uint8_t target : {uint8_t{1}, uint8_t{2}}) {
    Fixture f;
    assert(f.submit(1, 1, Action::open));
    f.tick(10);  // first movement frame is physically in flight
    assert(f.submit(2, target, Action::close));
    assert(f.submit(3, target, Action::stop, 11));
    assert(!strcmp(f.hooks.event(2).outcome, "cancelled"));
    assert(f.hooks.reservations.size() == 1);
    for (unsigned i = 0; i < 10; ++i) f.tick(11);
    assert(f.radio.frames.size() == 1 && f.radio.ends == 0);  // no truncation
    f.radio.done = true;
    f.tick(12);
    assert(f.radio.frames.size() == 2 && f.radio.frames[1].parsed.action == 0x04);
    assert(f.radio.frames[1].parsed.counter == (target == 1 ? 101 : 700));
    assert(!strcmp(f.hooks.event(1).outcome, "cancelled"));
    assert(f.hooks.event(1).completed_copies == 1);
    assert(!strcmp(f.hooks.event(1).error, "stop_preempted"));
    // Active STOP is coalesced; it cannot consume another reserved page.
    assert(!f.submit(4, target, Action::stop, 12));
    assert(!strcmp(f.hooks.event(4).error, "stop_in_progress"));
    f.drain(12);
    assert(f.hooks.reservations.size() == 2 && f.radio.frames.size() == 4);
  }
}

static void check_stop_during_maintenance() {
  Fixture f;
  f.due();
  // First rotation also installs the downgrade fence in the blank bank.
  for (unsigned i = 0; i < 100 && f.journal.maintenance_due(); ++i) f.tick(10);
  assert(!f.journal.maintenance_due());
  f.due();  // bank 0 still has committed pages, so rotation needs sector erases
  assert(!f.submit(1, 2, Action::close));  // refuse admission before maintenance
  assert(!strcmp(f.hooks.event(1).error, "maintenance_pending"));
  f.tick(10);  // bounded idle step erases just one sector
  assert(f.flash.erases() == 1 && f.journal.maintenance_due());
  const uint32_t programs = f.flash.programs();
  assert(f.submit(2, 1, Action::stop, 11));
  f.tick(11);
  assert(f.radio.frames.size() == 1 && f.radio.frames[0].parsed.action == 0x04);
  assert(f.flash.programs() == programs + RECORD_BYTES / PAGE_BYTES && f.flash.erases() == 1);
  assert(!f.submit(3, 2, Action::open, 12));
  f.tick(12);
  assert(f.flash.erases() == 1);  // never erase while a radio frame is in flight
  f.drain(12);
  assert(!strcmp(f.hooks.event(2).outcome, "emitted"));
  assert(f.hooks.reservations.size() == 1);
  for (unsigned i = 0; i < 10 && f.journal.maintenance_due(); ++i) f.tick(12);
  assert(!f.journal.maintenance_due());
  assert(f.submit(4, 2, Action::open, 13));
  f.drain(13);
  assert(f.radio.frames.back().parsed.counter == 700);

  // A movement admitted earlier must also wait when the journal becomes due.
  Fixture queued;
  assert(queued.submit(1, 2, Action::open));
  queued.due();
  const uint32_t before = queued.flash.programs();
  queued.tick(10);
  assert(queued.radio.frames.empty() && queued.flash.programs() == before + 2);
  queued.drain();
  assert(queued.radio.frames[0].parsed.counter == 700);

  Fixture failure;
  assert(failure.submit(1, 2, Action::open));
  failure.due();
  failure.flash.cut_after(0, 0);
  failure.tick(10);
  assert(!strcmp(failure.hooks.event(0).error, "storage_io_error"));
  assert(!strcmp(failure.hooks.event(1).error, "storage_io_error"));
  assert(failure.radio.frames.empty() && !failure.runtime.pending());
}

static void check_expiry_disconnect_and_recovery() {
  Fixture f;
  const uint32_t near_wrap = 0xfffffff0u;
  assert(f.submit(1, 1, Action::open, near_wrap));
  assert(f.submit(2, 2, Action::close, near_wrap));
  f.tick(near_wrap);
  f.tick(near_wrap + TxQueue::TTL_MS - 1);
  assert(f.runtime.pending() == 1 && f.radio.frames.size() == 1);
  f.tick(near_wrap + TxQueue::TTL_MS);
  assert(!strcmp(f.hooks.event(2).outcome, "expired") && f.runtime.pending() == 0);
  assert(f.radio.ends == 0);  // in-flight frame is still allowed to finish
  f.radio.done = true;
  f.tick(near_wrap + TxQueue::TTL_MS + 1);
  assert(!strcmp(f.hooks.event(1).outcome, "expired"));
  assert(f.hooks.event(1).completed_copies == 1);
  assert(f.submit(3, 1, Action::close));
  f.tick(10);
  assert(f.radio.frames.back().parsed.counter == 101);
  assert(f.submit(4, 2, Action::open));
  f.runtime.disconnect();
  f.tick(11);
  assert(f.radio.running && !strcmp(f.hooks.event(4).outcome, "cancelled"));
  f.radio.done = true;
  f.tick(12);
  assert(!strcmp(f.hooks.event(3).outcome, "cancelled"));
  const size_t before = f.radio.frames.size();
  for (int i = 0; i < 10; ++i) f.tick(12);
  assert(before == f.radio.frames.size());  // no replay after disconnect
  Journal rebooted(f.flash);
  assert(rebooted.open() == StorageState::ready);
  radio::RadioRuntime<FakeRadio, Hooks> runtime(rebooted, f.radio, f.hooks);
  runtime.set_enabled(true);
  assert(runtime.submit({5, 0, 1, Action::open, false, rebooted.incarnation(1)}, 20));
  runtime.tick(20);
  assert(f.radio.frames.back().parsed.counter == 102);
  runtime.disconnect();
  f.radio.done = true;
  runtime.tick(21);
  assert(runtime.submit({6, 0, 2, Action::open, false, rebooted.incarnation(2)}, 22));
  runtime.tick(22);
  assert(f.radio.frames.back().parsed.counter == 700);  // never reserved pending job
  runtime.disconnect();
  f.radio.done = true;
  runtime.tick(23);
}

static void check_gates_builders_and_faults() {
  Fixture f(false);
  assert(!f.submit(1, 1, Action::open));
  assert(!strcmp(f.hooks.event(1).error, "tx_disabled"));
  f.runtime.set_enabled(true);
  assert(!f.submit(2, 1, Action::none, 10, true));
  assert(!f.submit(3, 1, Action::stop, 10, true));
  assert(!strcmp(f.hooks.event(3).error, "invalid_request"));
  assert(f.journal.cancel_candidate(1) == Status::ok);
  assert(f.journal.set_service(1, true) == Status::ok);
  while (f.journal.maintenance_due()) assert(f.journal.maintain() == Status::ok);
  const uint32_t programs = f.flash.programs();
  f.hooks.qualified = false;
  assert(f.submit(4, 1, Action::open));
  f.drain();
  assert(!strcmp(f.hooks.event(4).error, "unqualified_profile"));
  assert(f.flash.programs() == programs && f.hooks.reservations.empty());
  f.hooks.qualified = true;
  f.runtime.set_enabled(true, true);
  f.hooks.refuse_phase = 1;
  assert(f.submit(5, 1, Action::none, 10, true));
  f.drain();
  assert(f.hooks.enrollment_calls == 1 && f.radio.frames.empty());
  f.hooks.refuse_phase = -1;
  assert(!strcmp(f.hooks.event(5).error, "body_refused"));
  assert(f.journal.cancel_candidate(1) == Status::ok);
  assert(f.journal.set_service(1, true) == Status::ok);
  f.runtime.set_enabled(true);
  f.radio.fail_start = true;
  assert(f.submit(6, 1, Action::open));
  f.drain();
  assert(!strcmp(f.hooks.event(6).error, "radio_start_failed"));
  f.radio.fail_start = false;
  assert(f.submit(7, 1, Action::open));
  f.tick(10);
  assert(f.radio.frames.back().parsed.counter == 101);
  f.radio.fail_poll = true;
  f.tick(11);
  assert(!strcmp(f.hooks.event(7).outcome, "unknown"));
  f.radio.fail_poll = false;
  f.settle();
  assert(f.submit(8, 1, Action::open));
  f.drain();
  assert(f.radio.frames.back().parsed.counter == 102);

  Fixture io;
  assert(io.submit(1, 1, Action::open) && io.submit(2, 2, Action::close));
  io.flash.cut_after(BODY_BYTES / PAGE_BYTES, 0);  // body written; commit fails, counter is burnt
  io.tick(10);
  assert(io.radio.frames.empty() && io.runtime.pending() == 0);
  for (uint32_t id : {1u, 2u}) {
    assert(!strcmp(io.hooks.event(id).error, "storage_io_error"));
    assert(io.hooks.event(id).storage_status == Status::io_error);
  }
  assert(!io.submit(3, 1, Action::stop));
  io.flash.restore_power();
  Journal recovered(io.flash);
  assert(recovered.open() == StorageState::ready);
  radio::RadioRuntime<FakeRadio, Hooks> fresh(recovered, io.radio, io.hooks);
  fresh.set_enabled(true);
  assert(fresh.submit({4, 0, 1, Action::stop, false, recovered.incarnation(1)}, 20));
  fresh.tick(20);
  assert(io.radio.frames.back().parsed.counter == 101);
  fresh.disconnect();
  io.radio.done = true;
  fresh.tick(21);
}

static void check_overflow_and_critical_counter() {
  Fixture f;
  for (uint32_t id = 1; id <= TxQueue::CAPACITY; ++id)
    assert(f.submit(id, 2, Action::open));
  assert(!f.submit(17, 2, Action::open));
  assert(!strcmp(f.hooks.event(17).error, "queue_full"));
  assert(f.submit(18, 1, Action::stop));
  assert(!strcmp(f.hooks.event(16).outcome, "cancelled"));
  assert(f.submit(19, 1, Action::stop));
  assert(!strcmp(f.hooks.event(18).outcome, "cancelled"));
  f.tick(10);
  assert(f.radio.frames[0].parsed.counter == 100 && f.radio.frames[0].parsed.action == 0x04);
  f.runtime.disconnect();
  f.radio.done = true;
  f.tick(11);

  Fixture last(true, 0xffff);
  assert(last.submit(1, 1, Action::open));
  last.drain();
  assert(!strcmp(last.hooks.event(1).error, "counter_exhausted"));
  last.runtime.set_enabled(true, true);
  assert(last.submit(2, 1, Action::none, 10, true));
  last.drain();
  assert(!strcmp(last.hooks.event(2).outcome, "emitted"));
  assert(last.hooks.enrollment_calls == 2);  // candidate counters are independent
  assert(last.journal.cancel_candidate(1) == Status::ok);
  assert(last.submit(3, 1, Action::stop));
  last.drain();
  assert(last.radio.frames.size() == 9 && last.radio.frames.back().parsed.counter == 0xffff);
  assert(last.submit(4, 1, Action::stop));
  last.drain();
  assert(!strcmp(last.hooks.event(4).error, "counter_exhausted"));

  // The scheduler can drain a STOP for every slot with ONLY the 16 reserved
  // pages left. Pending duplicates coalesce before any reservation is made.
  Fixture batch;
  for (uint8_t slot = 3; slot <= SLOTS; ++slot) {
    while (batch.journal.maintenance_due()) assert(batch.journal.maintain() == Status::ok);
    assert(batch.journal.provision(slot, {uint32_t{0x180000} + slot, 900, 1}) == Status::ok);
    assert(batch.journal.confirm(slot) == Status::ok);
  }
  const auto erases = batch.flash.erases();
  Reservation reservation;
  while (batch.journal.reserve(1, 1, false, &reservation) == Status::ok) {}
  assert(batch.journal.maintenance_due());
  batch.hooks.copies = 1;
  for (uint8_t slot = 1; slot <= SLOTS; ++slot)
    assert(batch.submit(slot, slot, Action::stop));
  assert(batch.submit(17, 1, Action::stop));
  assert(!strcmp(batch.hooks.event(1).outcome, "cancelled"));
  batch.drain();
  assert(batch.hooks.reservations.size() == 16 && batch.radio.frames.size() == 16);
  assert(batch.flash.violations() == 0);
  for (const auto& burst : batch.radio.bursts) assert(burst.erases == erases);
  for (uint32_t id = 2; id <= 17; ++id)
    assert(!strcmp(batch.hooks.event(id).outcome, "emitted"));

  Fixture full;
  while (full.journal.reserve(1, 1, true, &reservation) == Status::ok) {}
  assert(full.journal.state() == StorageState::full);
  assert(!full.submit(1, 1, Action::stop));  // known full storage: admission refusal
  assert(!strcmp(full.hooks.event(1).error, "storage_full"));
  assert(full.hooks.event(1).storage_status == Status::no_space);
  assert(full.radio.frames.empty());
}

static uint32_t next_counter(const Journal& journal) {
  uint32_t next = 0;
  assert(journal.next_counter(1, &next, true));
  return next;
}

static void finish_stage(Fixture& f, uint32_t now) {
  const size_t bursts = f.radio.bursts.size();
  for (unsigned i = 0; i < radio::MAX_COPIES && f.radio.running; ++i) {
    f.radio.done = true;
    f.tick(now);
    assert(f.radio.bursts.size() == bursts);  // continuous copies, one start
  }
  assert(!f.radio.running);
}

static void check_pair_sequence_and_gap() {
  Fixture f(true, 0);
  f.runtime.set_enabled(true, true);
  // Arrange for the TWO reservations to make maintenance due, then prove
  // neither the copies nor the inter-stage gap run that maintenance.
  f.prepare_candidate();
  Reservation reserved;
  while (durable_image(f.flash).sequence < PAIRS_PER_BANK - MAINTAIN_BELOW - 1)
    assert(f.journal.reserve(2, 1, false, &reserved) == Status::ok);
  assert(!f.journal.maintenance_due());
  assert(f.submit(1, 1, Action::none, 10, true));
  f.tick(10);
  assert(f.journal.maintenance_due());
  assert(f.hooks.reservations.size() == 2 && f.hooks.phases == (std::vector<uint8_t>{0, 1}));
  assert(f.hooks.reservations[0].counter == 0 && f.hooks.reservations[1].counter == 1);
  assert(f.hooks.reservations[0].identity == f.hooks.reservations[1].identity);
  assert(f.hooks.reservations[0].identity != 0x186091);
  const uint32_t programs = f.flash.programs(), erases = f.flash.erases();
  finish_stage(f, 500);
  assert(f.runtime.active() && f.hooks.events.empty() && f.radio.ends == 1);
  for (uint32_t now : {501u, 1000u, 2010u}) {
    f.tick(now);
    assert(f.radio.bursts.size() == 1 && !f.radio.running);
    assert(f.flash.programs() == programs && f.flash.erases() == erases);
  }
  f.tick(2011);
  assert(f.radio.running && f.radio.bursts.size() == 2);
  assert(f.radio.bursts[1].start_ms - f.radio.bursts[0].start_ms == 2001);
  assert(f.radio.bursts[1].programs == f.radio.bursts[0].programs);
  assert(f.radio.bursts[1].erases == f.radio.bursts[0].erases);
  finish_stage(f, 2300);
  assert(!f.runtime.active() && f.radio.ends == 2);
  const auto& event = f.hooks.event(1);
  assert(!strcmp(event.outcome, "emitted") && !event.error && event.completed_copies == 6);
  assert(f.radio.frames.size() == 6);
  for (size_t i = 0; i < 6; ++i) {
    const auto& frame = f.radio.frames[i];
    const auto& first = f.radio.frames[i < 3 ? 0 : 3];
    assert(frame.body.length == (i < 3 ? 12 : 13));
    assert(frame.parsed.counter == (i < 3 ? 0 : 1));
    assert(frame.parsed.identity == f.hooks.reservations[0].identity);
    assert(frame.programs == programs && frame.chip_ns == 208500);
    assert(!memcmp(frame.body.bytes, first.body.bytes, frame.body.length));
  }
  assert(f.journal.confirm_candidate(1) == Status::ok);
  f.settle(2301);
  assert(f.submit(2, 1, Action::open, 2301));
  f.drain(2301);
  assert(f.radio.frames.back().parsed.counter == 2 && f.hooks.phases.back() == 0);
  assert(!strcmp(f.hooks.event(2).outcome, "emitted"));
}

static void check_pair_cancellation() {
  // STOP on either slot, disconnect, expiry: phase 0, idle gap, phase 1.
  for (uint8_t stage = 0; stage < 3; ++stage) {
    for (uint8_t cause = 0; cause < 4; ++cause) {
      Fixture f(true, 0);
      f.runtime.set_enabled(true, true);
      assert(f.submit(1, 1, Action::none, 10, true));
      f.tick(10);
      if (stage) finish_stage(f, 500);
      if (stage == 2) f.tick(2011);
      const size_t bursts = f.radio.bursts.size();
      const uint32_t programs = f.flash.programs(), erases = f.flash.erases();
      const uint32_t now = cause == 3 ? 6010 : stage == 2 ? 2100 : 600;
      if (cause < 2) assert(f.submit(2, cause + 1, Action::stop, now));
      else if (cause == 2) f.runtime.disconnect();
      else f.tick(now);
      if (stage == 1) {
        assert(!f.runtime.active());  // cancellation in the gap is immediate
        assert(f.radio.stops == 0 && f.radio.ends == 1);
      } else {
        assert(f.runtime.active() && f.radio.running && f.radio.stop_requested);
        assert(f.radio.stops == 1 && f.radio.ends == (stage == 2 ? 1u : 0u));
        for (unsigned i = 0; i < 4; ++i) f.tick(now);
        assert(f.radio.stops == 1 && f.radio.running);  // no partial frame counted
        assert(f.flash.programs() == programs && f.flash.erases() == erases);
        // The already queued STOP is removed without new flash; cancellation
        // remains sticky until the safe boundary even when the STOP disappears.
        if (cause < 2) f.runtime.disconnect();
        f.radio.done = true;
        f.tick(now + 1);
        assert(!f.runtime.active() && !f.radio.running);
      }
      const auto& event = f.hooks.event(1);
      assert(!strcmp(event.outcome, cause == 3 ? "expired" : "cancelled"));
      assert(!strcmp(event.error, cause < 2 ? "stop_preempted" :
                                  cause == 2 ? "session_disconnected" : "deadline_expired"));
      assert(event.completed_copies == (stage == 0 ? 1 : stage == 1 ? 3 : 4));
      assert(f.hooks.reservations.size() == 2);
      // Completion can perform idle bank maintenance on this same tick.
      // It must not consume another candidate counter or start/replay RF.
      assert(next_counter(f.journal) == 2);
      if (stage == 1 && cause < 2) f.runtime.disconnect();
      for (uint32_t later : {6011u, 7000u, 9000u}) f.tick(later);
      assert(f.radio.bursts.size() == bursts);  // no second stage or replay
      assert(next_counter(f.journal) == 2);
      Journal rebooted(f.flash);
      assert(rebooted.open() == StorageState::ready && next_counter(rebooted) == 2);
    }
  }
}

static void check_pair_stop_dispatch() {
  for (uint8_t stage = 0; stage < 3; ++stage) {
    for (uint8_t target : {uint8_t{1}, uint8_t{2}}) {
      Fixture f(true, 0);
      f.runtime.set_enabled(true, true);
      assert(f.submit(1, 1, Action::none, 10, true));
      f.tick(10);
      if (stage) finish_stage(f, 500);
      if (stage == 2) f.tick(2011);
      const uint32_t now = stage == 2 ? 2100 : 600;
      const uint32_t programs = f.flash.programs();
      assert(f.submit(2, target, Action::stop, now));
      assert(f.flash.programs() == programs);  // STOP admission cannot write flash
      if (stage != 1) f.radio.done = true;
      f.tick(now + 1);
      assert(!strcmp(f.hooks.event(1).outcome, "cancelled"));
      assert(f.hooks.event(1).completed_copies == (stage == 0 ? 1 : stage == 1 ? 3 : 4));
      assert(f.radio.frames.back().parsed.action == 0x04);
      assert(f.radio.frames.back().parsed.counter == (target == 1 ? 0 : 700));
      assert(f.hooks.reservations.size() == 3 && f.flash.programs() == programs + RECORD_BYTES / PAGE_BYTES);
      f.drain(now + 2);
      assert(!strcmp(f.hooks.event(2).outcome, "emitted"));
      const size_t bursts = f.radio.bursts.size();
      for (uint32_t later : {6011u, 7000u}) f.tick(later);
      assert(f.radio.bursts.size() == bursts);  // cancelled pair stays cancelled
    }
  }
}

static void check_pair_deadlines_and_profiles() {
  // A pair admitted just before its queue expiry gets a fresh active 6000 ms
  // deadline, including uint32_t clock rollover.
  const uint32_t start = 0xfffffff0u;
  Fixture f(true, 0);
  f.runtime.set_enabled(true, true);
  assert(f.submit(1, 1, Action::none, start - TxQueue::TTL_MS + 1, true));
  assert(f.submit(2, 2, Action::close, start - TxQueue::TTL_MS + 1));
  f.tick(start);
  f.tick(start + 1);
  assert(!strcmp(f.hooks.event(2).outcome, "expired"));
  finish_stage(f, start + 500);
  f.tick(start + 2000);
  assert(f.radio.bursts.size() == 1);
  f.tick(start + 2001);
  f.tick(start + 5999);
  assert(!f.radio.stop_requested && f.runtime.active());
  f.tick(start + 6000);
  assert(f.radio.stop_requested);
  f.radio.done = true;
  f.tick(start + 6001);
  assert(!strcmp(f.hooks.event(1).outcome, "expired") && f.hooks.event(1).completed_copies == 4);

  Fixture queued;
  queued.runtime.set_enabled(true, true);
  assert(queued.submit(1, 1, Action::none, 10, true));
  queued.tick(10 + TxQueue::TTL_MS);
  assert(!strcmp(queued.hooks.event(1).outcome, "expired"));
  assert(queued.hooks.reservations.empty() && queued.radio.bursts.empty());

  Fixture calibrated(true, 0);
  calibrated.runtime.set_enabled(true, true);
  calibrated.hooks.copies = radio::MAX_COPIES;
  calibrated.hooks.second_start_ms = 2500;
  assert(calibrated.submit(1, 1, Action::none, 10, true));
  calibrated.tick(10);
  finish_stage(calibrated, 1900);
  calibrated.tick(2509);
  assert(calibrated.radio.bursts.size() == 1);
  calibrated.tick(2510);
  finish_stage(calibrated, 4500);
  assert(!strcmp(calibrated.hooks.event(1).outcome, "emitted"));
  assert(calibrated.hooks.event(1).completed_copies == 64);
  assert(calibrated.journal.confirm_candidate(1) == Status::ok);
  calibrated.hooks.second_start_ms = 0;  // ignored for normal commands
  calibrated.settle(4501);
  assert(calibrated.submit(2, 1, Action::open, 4501));
  calibrated.drain(4501);
  assert(!strcmp(calibrated.hooks.event(2).outcome, "emitted"));

  for (uint8_t copies : {uint8_t{0}, uint8_t{33}}) {
    Fixture invalid;
    invalid.runtime.set_enabled(true, true);
    invalid.hooks.copies = copies;
    assert(invalid.submit(1, 1, Action::none, 10, true));
    invalid.drain();
    assert(!strcmp(invalid.hooks.event(1).error, "unqualified_profile"));
    assert(invalid.radio.bursts.empty() && invalid.hooks.reservations.empty());
  }
  for (int duration_delta : {-1, 0}) {
    Fixture overlap(true, 0);
    overlap.runtime.set_enabled(true, true);
    radio::Body body;
    radio::Waveform wave;
    overlap.prepare_candidate();
    uint32_t identity = 0;
    assert(overlap.journal.identity(1, &identity, true));
    assert(radio::make_enrollment_body(identity, 0, 0, &body));
    assert(radio::encode_burst(body, overlap.hooks.copies, &wave));
    overlap.hooks.chip_ns = 1000000;
    overlap.hooks.second_start_ms = wave.chips() + duration_delta;
    assert(overlap.submit(1, 1, Action::none, 10, true));
    overlap.drain();
    assert(!strcmp(overlap.hooks.event(1).error, "enrollment_overlap"));
    assert(overlap.radio.bursts.empty() && next_counter(overlap.journal) == 2);
  }
}

static void check_movement_gets_its_full_burst_after_queueing() {
  const uint32_t start = 0xfffffff0u;
  Fixture f;
  f.hooks.copies = 25;
  f.hooks.chip_ns = 1000000;  // the largest supported calibration
  assert(f.submit(1, 1, Action::close, start - TxQueue::TTL_MS + 1));
  f.tick(start);
  const uint32_t duration_ms = f.radio.wave->chips();
  for (uint32_t copy = 1; copy <= f.hooks.copies; ++copy) {
    f.radio.done = true;
    f.tick(start + duration_ms * copy / f.hooks.copies);
  }
  assert(!f.runtime.active() && !f.radio.stops);
  assert(!strcmp(f.hooks.event(1).outcome, "emitted"));
  assert(f.hooks.event(1).completed_copies == f.hooks.copies);

  // A stuck backend still receives a bounded, frame-safe cancellation.
  assert(f.submit(2, 1, Action::open, 10000));
  f.tick(10000);
  const uint32_t watchdog = 10000 + f.radio.wave->chips() + 1000;
  f.tick(watchdog - 1);
  assert(!f.radio.stop_requested);
  f.tick(watchdog);
  assert(f.radio.stop_requested);
  f.radio.done = true;
  f.tick(watchdog + 1);
  assert(!strcmp(f.hooks.event(2).outcome, "expired"));
}

static void check_pair_reservation_cuts_and_faults() {
  // Second reservation: before body, intact torn body, before/during commit.
  // The first counter is always burnt; the second burns iff its body is intact.
  for (const auto &cut : {
      std::pair<unsigned, unsigned>{RECORD_BYTES / PAGE_BYTES, 0},
      {RECORD_BYTES / PAGE_BYTES + BODY_BYTES / PAGE_BYTES - 1, PAGE_BYTES},
      {RECORD_BYTES / PAGE_BYTES + BODY_BYTES / PAGE_BYTES, 0},
      {RECORD_BYTES / PAGE_BYTES + BODY_BYTES / PAGE_BYTES, PAGE_BYTES / 2}}) {
    Fixture f(true, 0);
    f.runtime.set_enabled(true, true);
    assert(f.submit(1, 1, Action::none, 10, true));
    assert(f.submit(2, 2, Action::open));
    f.flash.cut_after(cut.first, cut.second);
    f.tick(10);
    assert(f.radio.bursts.empty() && f.hooks.reservations.empty());
    assert(!f.runtime.active() && !f.runtime.pending());
    assert(!strcmp(f.hooks.event(1).error, "storage_io_error"));
    assert(!strcmp(f.hooks.event(2).error, "storage_io_error"));
    f.flash.restore_power();
    Journal recovered(f.flash);
    assert(recovered.open() == StorageState::ready);
    const uint32_t next = cut.first == RECORD_BYTES / PAGE_BYTES && cut.second == 0 ? 1 : 2;
    assert(next_counter(recovered) == next);
    assert(!f.submit(3, 1, Action::stop));  // old runtime cannot resume after fault
    assert(f.radio.bursts.empty());
  }
  // A candidate can consume only the two reservations of its durable
  // attempt. Re-enqueueing cannot open another pair without a retry claim.
  Fixture exhausted(true, 0xfffe);
  exhausted.runtime.set_enabled(true, true);
  exhausted.prepare_candidate();
  Reservation consumed;
  for (unsigned i = 0; i < 2; ++i)
    assert(exhausted.journal.reserve(1, 0, false, &consumed,
        exhausted.journal.incarnation(1, true), true) == Status::ok);
  while (exhausted.journal.maintenance_due()) assert(exhausted.journal.maintain() == Status::ok);
  assert(exhausted.submit(1, 1, Action::none, 10, true));
  exhausted.drain();
  assert(exhausted.hooks.event(1).storage_status == Status::no_attempt);
  assert(next_counter(exhausted.journal) == 2);
  assert(exhausted.radio.bursts.empty() && exhausted.hooks.reservations.empty());

  for (uint8_t phase = 0; phase < 2; ++phase) {
    for (bool start_failure : {false, true}) {
      Fixture f(true, 0);
      f.runtime.set_enabled(true, true);
      assert(f.submit(1, 1, Action::none, 10, true));
      if (!phase && start_failure) f.radio.fail_start = true;
      f.tick(10);
      if (phase) {
        finish_stage(f, 500);
        if (start_failure) f.radio.fail_start = true;
        f.tick(2011);
      }
      if (!start_failure) {
        // One known copy, then a fault during the next copy. Partial output
        // must never increase completed_copies or trigger a replay.
        f.radio.done = true;
        f.tick(phase ? 2100 : 100);
        f.radio.fail_poll = true;
        f.tick(phase ? 2101 : 101);
      }
      const auto& event = f.hooks.event(1);
      assert(!strcmp(event.outcome, start_failure ? "rejected" : "unknown"));
      assert(!strcmp(event.error, start_failure ? "radio_start_failed" : "radio_fault"));
      assert(event.completed_copies == (phase ? 3 : 0) + (start_failure ? 0 : 1));
      assert(f.radio.ends == unsigned(start_failure ? phase : phase + 1));
      assert(next_counter(f.journal) == 2 && f.hooks.reservations.size() == 2);
      const size_t bursts = f.radio.bursts.size();
      for (uint32_t now : {3000u, 6000u, 9000u}) f.tick(now);
      assert(f.radio.bursts.size() == bursts && next_counter(f.journal) == 2);
    }
  }
}

static void check_incarnation_fences_and_slot_reuse() {
  Fixture f;
  const auto old = f.journal.shutter(1);
  assert(!f.runtime.submit({90,0,1,Action::open,false,0},10));
  assert(!strcmp(f.hooks.event(90).error, "invalid_request"));
  assert(f.submit(1,1,Action::open));
  // Exercise the lower-level runtime fence independently of the controller's
  // busy admission: trusted journal calls retire and reuse a queued slot.
  assert(f.journal.set_service(1,false) == Status::ok);
  assert(f.journal.retire(1) == Status::ok);
  assert(f.journal.allocate_candidate(1,false) == Status::ok);
  assert(f.journal.claim_attempt(1,false) == Status::ok);
  Reservation reservation;
  const auto epoch = f.journal.incarnation(1,true);
  for (unsigned i=0; i<2; ++i)
    assert(f.journal.reserve(1,0,false,&reservation,epoch,true) == Status::ok);
  assert(f.journal.confirm_candidate(1) == Status::ok);
  assert(f.journal.shutter(1).logical_id != old.logical_id);
  while (f.journal.maintenance_due()) assert(f.journal.maintain() == Status::ok);
  const auto writes = f.flash.programs();
  f.tick(10);
  assert(f.radio.bursts.empty() && f.flash.programs() == writes);
  assert(f.hooks.event(1).job.incarnation == old.incarnation);
  assert(!strcmp(f.hooks.event(1).error,"stale_incarnation"));
  assert(!f.runtime.submit({2,0,1,Action::open,false,old.incarnation},11));
  assert(!strcmp(f.hooks.event(2).error,"stale_incarnation"));
  assert(f.submit(3,1,Action::open,12));
  f.drain(12);
  assert(f.hooks.event(3).job.incarnation == epoch);
  assert(f.radio.frames.back().parsed.counter == 2);
}

static void check_candidate_replacement_during_gap() {
  Fixture f(true,0);
  f.runtime.set_enabled(true,true);
  assert(f.submit(1,1,Action::none,10,true));
  f.tick(10);
  finish_stage(f,500);
  const auto old = f.journal.incarnation(1,true);
  assert(f.journal.cancel_candidate(1) == Status::ok);
  assert(f.journal.allocate_candidate(1,true) == Status::ok);
  assert(f.journal.claim_attempt(1,false) == Status::ok);
  const auto fresh = f.journal.incarnation(1,true);
  assert(old != fresh);
  f.tick(2011);
  assert(!f.runtime.active() && f.radio.bursts.size() == 1);
  assert(!strcmp(f.hooks.event(1).error,"stale_incarnation"));
  assert(f.hooks.event(1).job.incarnation == old && next_counter(f.journal) == 0);
  while (f.journal.maintenance_due()) assert(f.journal.maintain() == Status::ok);
  assert(f.submit(2,1,Action::none,2200,true));
  f.drain(2200);
  assert(f.hooks.event(2).job.incarnation == fresh && next_counter(f.journal) == 2);
}

static void check_suspended_gap_never_starts_burst() {
  Fixture f(true, 0);
  f.runtime.set_enabled(true, true);
  assert(f.submit(1, 1, Action::none, 10, true));
  f.tick(10);
  finish_stage(f, 500);
  const auto programs = f.flash.programs(), erases = f.flash.erases();
  f.radio.now = 2011;
  f.runtime.tick(2011, false);
  assert(f.runtime.active() && !f.radio.running && f.radio.bursts.size() == 1);
  assert(f.flash.programs() == programs && f.flash.erases() == erases);
  f.tick(2012);
  assert(f.radio.bursts.size() == 2);
  f.drain(2013);
  assert(!strcmp(f.hooks.event(1).outcome, "emitted"));
}

static void check_pause_drain_never_maintains() {
  Fixture f;
  f.due();
  assert(f.submit(1,1,Action::stop));
  f.tick(10);
  f.runtime.disconnect();
  const auto programs = f.flash.programs(), erases = f.flash.erases();
  f.radio.done = true;
  f.radio.now = 11;
  f.runtime.tick(11,false);
  assert(!f.runtime.active());
  for (uint32_t now : {12u,13u,14u}) f.runtime.tick(now,false);
  assert(f.flash.programs() == programs && f.flash.erases() == erases);
  assert(f.journal.maintenance_due());
  f.tick(15);
  assert(f.flash.programs() > programs || f.flash.erases() > erases);
}

int main() {
  check_independent_slots_and_shared_copies();
  check_stop_preemption();
  check_stop_during_maintenance();
  check_expiry_disconnect_and_recovery();
  check_gates_builders_and_faults();
  check_overflow_and_critical_counter();
  check_incarnation_fences_and_slot_reuse();
  check_candidate_replacement_during_gap();
  check_pause_drain_never_maintains();
  check_suspended_gap_never_starts_burst();
  check_pair_sequence_and_gap();
  check_pair_cancellation();
  check_pair_stop_dispatch();
  check_pair_deadlines_and_profiles();
  check_movement_gets_its_full_burst_after_queueing();
  check_pair_reservation_cuts_and_faults();
  puts("OK radio_runtime: durable counters, complete-frame STOP, expiry, disconnect, gates, faults, two-stage B enrollment");
}
