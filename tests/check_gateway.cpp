// Native checks for the shared JSONL gateway: the real Gateway, journal, queue,
// runtime and codec over simulated flash, radio and clock. No hardware, no USB.
#include <memory>
#include <string>
#include <vector>

#include "sim_gateway.h"

using namespace ha_x2d;

#define CHECK(condition)                                                        \
  do {                                                                          \
    if (!(condition)) {                                                         \
      fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
      abort();                                                                  \
    }                                                                           \
  } while (0)
#define CHECK_EQ(actual, expected)                                              \
  do {                                                                          \
    const std::string a_ = (actual), e_ = (expected);                           \
    if (a_ != e_) {                                                             \
      fprintf(stderr, "%s:%d: %s\n  got:      %s\n  expected: %s\n", __FILE__,  \
              __LINE__, #actual, a_.c_str(), e_.c_str());                       \
      abort();                                                                  \
    }                                                                           \
  } while (0)

// One simulated device: journal over caller-owned flash (so a "reboot" keeps
// it), radio, firmware policy, and the gateway under test. The helpers behave
// like the firmware loop: offer one line, tick, flush the output.
struct Rig {
  uint32_t clock;
  journal::Journal journal;
  sim::Radio radio;
  sim::Hooks hooks;
  sim::Gateway gateway;
  std::string pending;

  explicit Rig(journal::MemoryFlash& flash, const char* session = sim::SESSION,
               bool pair = true, bool tx = true, bool enroll = false)
      : clock(1000), journal(flash), radio(clock), hooks(clock),
        gateway(journal, radio, hooks, sim::DEVICE_ID, session) {
    hooks.tx = tx;
    hooks.enroll = enroll;
    radio.record = true;
    sim::open_paired(journal, pair);
    gateway.begin();
  }

  void drain() {
    while (gateway.output_size()) {
      const size_t count = gateway.output_contiguous();
      pending.append(gateway.output_data(), count);
      gateway.consume_output(count);
    }
  }
  void feed(const std::string& bytes, bool flush = true) {
    for (size_t at = 0; at < bytes.size();) {
      const size_t used = gateway.feed(bytes.data() + at, bytes.size() - at);
      gateway.tick();  // the ACK is already queued
      if (flush) drain();
      if (!used) return;
      at += used;
    }
  }
  void send(const std::string& line) { feed(line + "\n"); }
  void run(uint32_t ms) {
    while (ms--) {
      ++clock;
      gateway.tick();
      drain();
    }
  }
  std::vector<std::string> take() {
    std::vector<std::string> lines;
    size_t at = 0, end;
    while ((end = pending.find('\n', at)) != std::string::npos) {
      lines.push_back(pending.substr(at, end - at));
      at = end + 1;
    }
    CHECK(at == pending.size());
    pending.clear();
    return lines;
  }
  std::string one() {
    const auto lines = take();
    CHECK(lines.size() == 1);
    return lines[0];
  }
  uint32_t next(uint8_t slot) {
    uint32_t value = 0;
    CHECK(journal.next_counter(slot, &value));
    return value;
  }
};

static std::unique_ptr<Rig> make(journal::MemoryFlash& flash, const char* session = sim::SESSION,
                                 bool pair = true, bool tx = true, bool enroll = false) {
  return std::make_unique<Rig>(flash, session, pair, tx, enroll);
}

static std::string num(unsigned value) { return std::to_string(value); }
static std::string hello(unsigned id) { return R"({"v":2,"id":)" + num(id) + R"(,"op":"hello"})"; }
static std::string op(unsigned id, const char* name, const std::string& args = "{}",
                      const char* session = sim::SESSION) {
  return R"({"v":2,"id":)" + num(id) + R"(,"op":")" + name + R"(","session":")" + session +
         R"(","args":)" + args + "}";
}
static std::string slot_op(unsigned id, const char* name, unsigned slot) {
  return op(id, name, R"({"shutter_id":)" + num(slot) + "}");
}
static std::string command(unsigned id, unsigned slot, const char* action,
                           const char* session = sim::SESSION) {
  return op(id, "command", R"({"shutter_id":)" + num(slot) + R"(,"action":")" + action + R"("})",
            session);
}
static std::string ok(unsigned id, const std::string& result) {
  return R"({"v":2,"id":)" + num(id) + R"(,"ok":true,"result":)" + result + "}";
}
static std::string ack(unsigned id, unsigned slot) {
  return ok(id, R"({"accepted":true,"shutter_id":)" + num(slot) + "}");
}
static std::string fail(unsigned id, const char* error) {
  return R"({"v":2,"id":)" + num(id) + R"(,"ok":false,"error":")" + error + R"("})";
}
static std::string event(unsigned seq, unsigned request, unsigned slot, const char* result,
                         unsigned copies, const char* error = nullptr) {
  return R"({"v":2,"session":")" + std::string(sim::SESSION) + R"(","seq":)" + num(seq) +
         R"(,"event":"tx_result","request_id":)" + num(request) + R"(,"shutter_id":)" + num(slot) +
         R"(,"result":")" + result + R"(","completed_copies":)" + num(copies) +
         (error ? std::string(R"(,"error":")") + error + "\"" : "") + "}";
}
static std::string hello_ok(const char* session = sim::SESSION,
                            const char* capabilities = R"(["status","shutters","command"])") {
  return ok(1, R"({"product":"ha-x2d","firmware":")" + std::string(sim::FIRMWARE) +
                   R"(","device_id":"0123456789ABCDEF","session":")" + session +
                   R"(","max_line_bytes":4096,"max_shutters":16,"capabilities":)" +
                   capabilities + "}");
}
static const char* GEN = "0123456789ABCDEF";

static bool same_wave(const radio::Waveform& a, const radio::Waveform& b) {
  if (a.chips() != b.chips() || a.copies() != b.copies()) return false;
  for (size_t i = 0; i < a.chips(); ++i)
    if (a.chip(i) != b.chip(i)) return false;
  return true;
}
static void expect_burst(const sim::Radio::Burst& burst, uint8_t slot, uint8_t action,
                         uint16_t counter, uint8_t copies = 25) {
  radio::Waveform expected;
  CHECK(radio::encode(sim::IDENTITY[slot], action, counter, copies, &expected));
  CHECK(same_wave(burst.wave, expected));
}

static void handshake_and_framing() {
  journal::MemoryFlash flash;
  auto r = make(flash);
  // Silent until a valid hello, whatever arrives.
  r->send(op(1, "status"));
  r->send("not json");
  r->feed(std::string(5000, 'x') + "\n");
  CHECK(r->take().empty() && !r->gateway.handshaken());

  // Fragmented CRLF line, one byte per call: reply only after the LF.
  const std::string line = hello(1) + "\r\n";
  for (size_t i = 0; i < line.size(); ++i) {
    CHECK(r->gateway.feed(&line[i], 1) == 1);
    CHECK(i + 1 == line.size() || r->gateway.output_size() == 0);
  }
  r->drain();
  CHECK_EQ(r->one(), hello_ok());
  CHECK(r->gateway.handshaken());

  // feed() stops after exactly one line so the caller can tick between lines.
  const std::string first = op(2, "status"), second = op(3, "shutters");
  const std::string both = first + "\n" + second + "\n";
  CHECK(r->gateway.feed(both.data(), both.size()) == first.size() + 1);
  r->drain();
  CHECK_EQ(r->one(), ok(2, R"({"uptime_ms":1000,"tx_enabled":true,"radio":{"detected":true,)"
                           R"("partnum":0,"version":20,"marcstate":1},"storage":{"state":"ready",)"
                           R"("generation":")" + std::string(GEN) + R"("}})"));
  CHECK(r->gateway.feed(both.data() + first.size() + 1, both.size() - first.size() - 1) ==
        second.size() + 1);
  r->drain();
  CHECK_EQ(r->one(), ok(3, R"({"generation":")" + std::string(GEN) + R"(","shutters":[)"
                           R"({"shutter_id":1,"state":"paired","last_command":null},)"
                           R"({"shutter_id":2,"state":"paired","last_command":null}]})"));

  // Line limit: 4095 bytes + LF is the longest accepted line.
  CHECK(r->gateway.feed(std::string(5000, 'x').c_str(), 5000) == 5000);  // no LF yet
  r->send("");  // terminates the 5000-byte overlong line
  CHECK_EQ(r->one(), R"({"v":2,"id":null,"ok":false,"error":"line_too_long"})");
  const std::string tail = std::string(4096, 'x') + "\n" + hello(9) + "\n";
  CHECK(r->gateway.feed(tail.data(), tail.size()) == 4097);
  r->drain();
  CHECK_EQ(r->one(), R"({"v":2,"id":null,"ok":false,"error":"line_too_long"})");
  r->send(std::string(4095, 'x'));
  CHECK_EQ(r->one(), R"({"v":2,"id":null,"ok":false,"error":"invalid_request"})");

  // Application-level rejections do not close or reset the session.
  r->send(R"({"v":2,"id":7,"op":"reboot","session":"FEDCBA9876543210","args":{}})");
  CHECK_EQ(r->one(), fail(7, "unsupported_operation"));
  r->send(R"({"v":2,"id":9,"op":"command","session":"FEDCBA9876543210","args":{"shutter_id":99,"action":"open"}})");
  CHECK_EQ(r->one(), fail(9, "invalid_request"));
  r->send("not json");
  CHECK_EQ(r->one(), R"({"v":2,"id":null,"ok":false,"error":"invalid_request"})");
}

static void command_ack_event_and_bodies() {
  journal::MemoryFlash flash;
  auto r = make(flash);
  r->send(hello(1));
  r->take();
  CHECK(r->next(1) == 100 && r->next(2) == 200);

  r->send(command(2, 1, "open"));
  CHECK_EQ(r->one(), ack(2, 1));
  CHECK(r->next(1) == 101 && r->radio.started == 1);  // counter reserved before RF
  r->run(249);
  CHECK(r->take().empty() && r->radio.running());
  r->run(1);
  CHECK_EQ(r->one(), event(1, 2, 1, "emitted", 25));
  expect_burst(r->radio.bursts[0], 1, 0x81, 100);

  r->send(command(3, 2, "close"));
  r->take();
  r->run(250);
  CHECK_EQ(r->one(), event(2, 3, 2, "emitted", 25));
  expect_burst(r->radio.bursts[1], 2, 0x82, 200);
  r->send(command(4, 2, "stop"));
  r->take();
  r->run(250);
  CHECK_EQ(r->one(), event(3, 4, 2, "emitted", 25));
  expect_burst(r->radio.bursts[2], 2, 0x04, 201);

  r->send(op(5, "shutters"));
  CHECK_EQ(r->one(), ok(5, R"({"generation":")" + std::string(GEN) + R"(","shutters":[)"
                           R"({"shutter_id":1,"state":"paired","last_command":"open"},)"
                           R"({"shutter_id":2,"state":"paired","last_command":"stop"}]})"));

  // Rejected admissions: negative reply only, no event, cached.
  r->send(command(6, 3, "open"));
  const std::string unknown = r->one();
  CHECK_EQ(unknown, fail(6, "unknown_shutter"));
  r->send(command(6, 3, "open"));
  CHECK_EQ(r->one(), unknown);
  r->run(50);
  CHECK(r->take().empty() && r->radio.started == 3);

  // Admission is acknowledged before a failure that happens at RF start.
  r->radio.fail_start = true;
  r->send(command(7, 1, "close"));
  const auto lines = r->take();
  CHECK(lines.size() == 2);
  CHECK_EQ(lines[0], ack(7, 1));
  CHECK_EQ(lines[1], event(4, 7, 1, "rejected", 0, "radio_start_failed"));
  CHECK(r->next(1) == 102);  // a failed start still consumed its counter
}

static void duplicates_staleness_and_cache() {
  journal::MemoryFlash flash;
  auto r = make(flash);
  r->send(hello(1));
  const std::string hello_reply = r->one();
  r->send(command(2, 1, "open"));
  const std::string first = r->one();
  const uint32_t counter = r->next(1);

  r->send(command(2, 1, "open"));  // identical duplicate: cached bytes, no side effect
  CHECK_EQ(r->one(), first);
  CHECK(r->next(1) == counter && r->radio.started == 1);
  r->send(command(2, 1, "close"));
  CHECK_EQ(r->one(), fail(2, "duplicate_conflict"));
  r->send(command(2, 2, "open"));
  CHECK_EQ(r->one(), fail(2, "duplicate_conflict"));
  r->run(250);
  CHECK_EQ(r->one(), event(1, 2, 1, "emitted", 25));  // only one burst, one event
  CHECK(r->radio.started == 1);

  // Wrong boot session is rejected without consuming the request id.
  r->send(op(20, "status", "{}", "0000000000000000"));
  CHECK_EQ(r->one(), fail(20, "stale_session"));
  r->send(op(20, "status"));
  const std::string status = r->one();
  CHECK(status.find(R"("ok":true)") != std::string::npos);
  r->send(op(5, "status"));  // older id that was never answered
  CHECK_EQ(r->one(), fail(5, "stale_request"));
  r->send(hello(1));  // duplicate hello replays the cached reply
  CHECK_EQ(r->one(), hello_reply);
  r->send(op(1, "status"));
  CHECK_EQ(r->one(), fail(1, "duplicate_conflict"));

  // The cache holds 16 replies; older ids are then merely stale.
  std::string last;
  for (unsigned id = 21; id <= 40; ++id) {
    r->send(op(id, "shutters"));
    last = r->one();
  }
  r->send(op(40, "shutters"));
  CHECK_EQ(r->one(), last);
  r->send(hello(1));
  CHECK_EQ(r->one(), fail(1, "stale_request"));
}

static void stop_preempts_in_event_order() {
  journal::MemoryFlash flash;
  auto r = make(flash);
  r->send(hello(1));
  r->take();
  r->send(command(2, 1, "open"));
  CHECK_EQ(r->one(), ack(2, 1));
  r->run(30);  // three copies on air
  r->send(command(3, 1, "close"));
  CHECK_EQ(r->one(), ack(3, 1));
  r->send(command(4, 2, "open"));
  CHECK_EQ(r->one(), ack(4, 2));

  // STOP drops the queued movement of its slot (that event precedes the STOP
  // reply, as before extraction) and interrupts the burst at a frame boundary.
  r->send(command(5, 1, "stop"));
  const auto lines = r->take();
  CHECK(lines.size() == 2);
  CHECK_EQ(lines[0], event(1, 3, 1, "cancelled", 0, "queue_cancelled"));
  CHECK_EQ(lines[1], ack(5, 1));
  CHECK(r->radio.stops == 1 && r->radio.running());
  r->run(9);
  CHECK(r->take().empty());  // nothing is cut mid-copy
  r->run(1);
  CHECK_EQ(r->one(), event(2, 2, 1, "cancelled", 4, "stop_preempted"));
  r->run(250);
  CHECK_EQ(r->one(), event(3, 5, 1, "emitted", 25));  // STOP precedes slot 2's movement
  r->run(250);
  CHECK_EQ(r->one(), event(4, 4, 2, "emitted", 25));

  CHECK(r->radio.bursts.size() == 3);
  CHECK(r->radio.bursts[0].stopped && r->radio.bursts[0].completed == 4);
  expect_burst(r->radio.bursts[0], 1, 0x81, 100);
  expect_burst(r->radio.bursts[1], 1, 0x04, 101);
  expect_burst(r->radio.bursts[2], 2, 0x81, 200);

  // Bounded queue: the 17th pending command is refused, with no event.
  r->send(command(6, 1, "open"));
  r->take();
  for (unsigned id = 7; id < 7 + TxQueue::CAPACITY; ++id) {
    r->send(command(id, 2, "close"));
    CHECK_EQ(r->one(), ack(id, 2));
  }
  r->send(command(40, 2, "close"));
  CHECK_EQ(r->one(), fail(40, "queue_full"));
  CHECK(r->take().empty());
}

static void disconnect_keeps_runtime_and_never_replays() {
  journal::MemoryFlash flash;
  auto r = make(flash);
  r->send(hello(1));
  r->take();
  r->send(command(2, 1, "open"));
  r->take();
  r->run(250);
  CHECK_EQ(r->one(), event(1, 2, 1, "emitted", 25));

  r->send(command(3, 1, "open"));  // on air
  r->send(command(4, 2, "close"));  // queued
  r->take();
  r->run(20);
  r->gateway.disconnected();
  CHECK(!r->gateway.handshaken() && !r->gateway.failed());
  // The new connection reuses request ids while the old burst still parks.
  r->send(hello(1));
  CHECK_EQ(r->one(), hello_ok());
  r->send(command(3, 1, "close"));
  CHECK_EQ(r->one(), ack(3, 1));
  r->run(10);
  CHECK(r->take().empty());  // the old burst's cancellation never reaches this connection
  r->run(250);
  CHECK_EQ(r->one(), event(2, 3, 1, "emitted", 25));  // seq continues; old id 3 was not mistaken

  CHECK(r->radio.bursts.size() == 3);  // the queued slot 2 close was never sent
  CHECK(r->radio.bursts[1].stopped && r->radio.bursts[1].completed == 3);
  expect_burst(r->radio.bursts[2], 1, 0x82, 102);
  CHECK(r->next(2) == 200);
  r->run(500);
  CHECK(r->take().empty() && r->radio.started == 3);

  // Idle disconnect is a safe repeat; the old connection's replies are gone.
  r->gateway.disconnected();
  r->gateway.disconnected();
  r->send(op(2, "status"));
  CHECK(r->take().empty());  // not handshaken
}

static void output_overflow_fails_closed() {
  journal::MemoryFlash flash;
  auto r = make(flash);
  r->send(hello(1));
  r->take();
  r->send(command(2, 1, "open"));
  r->take();
  unsigned id = 3;
  while (!r->gateway.failed() && id < 200) r->feed(op(id++, "status") + "\n", false);
  CHECK(r->gateway.failed() && r->gateway.output_size() <= OutputBuffer::CAPACITY);
  const size_t held = r->gateway.output_size();
  CHECK(r->gateway.feed(hello(1).c_str(), 3) == 0 && r->gateway.output_size() == held);
  for (int i = 0; i < 100; ++i) {  // no reader at all: work is still cancelled
    ++r->clock;
    r->gateway.tick();
  }
  CHECK(!r->gateway.runtime().active() && r->radio.stops == 1 && r->radio.bursts[0].stopped);
  CHECK(r->gateway.output_size() == held);  // events of a failed link are not queued

  r->gateway.disconnected();
  CHECK(!r->gateway.failed() && r->gateway.output_size() == 0);
  r->send(hello(1));
  CHECK_EQ(r->one(), hello_ok());
  r->send(command(2, 1, "close"));
  CHECK_EQ(r->one(), ack(2, 1));
}

static void policy_gates_are_dynamic_and_default_closed() {
  journal::MemoryFlash flash;
  {  // radio not verified at boot
    auto r = make(flash, sim::SESSION, true, false);
    r->send(hello(1));
    CHECK_EQ(r->one(), hello_ok(sim::SESSION, R"(["status","shutters"])"));
    r->send(op(2, "status"));
    CHECK(r->one().find(R"("tx_enabled":false)") != std::string::npos);
    r->send(command(3, 1, "open"));
    const std::string refused = r->one();
    CHECK_EQ(refused, fail(3, "profile_unverified"));
    r->send(command(3, 1, "open"));
    CHECK_EQ(r->one(), refused);
    r->run(50);
    CHECK(r->radio.started == 0 && r->next(1) == 100 && r->take().empty());
  }
  auto r = make(flash);  // same journal, now verified
  r->send(hello(1));
  r->take();
  r->hooks.tx = false;  // the radio failed after hello: admission and status follow
  r->send(command(2, 1, "open"));
  CHECK_EQ(r->one(), fail(2, "profile_unverified"));
  r->send(op(3, "status"));
  CHECK(r->one().find(R"("tx_enabled":false)") != std::string::npos);
  r->send(op(4, "shutters"));
  CHECK(r->one().find(R"("ok":true)") != std::string::npos);
  r->hooks.tx = true;
  r->send(command(5, 1, "open"));
  CHECK_EQ(r->one(), ack(5, 1));
  r->run(250);
  r->take();

  // Without enrollment: existing slots stay readable; nothing is created or confirmed.
  const uint32_t mutations = flash.mutations();
  r->send(slot_op(6, "provision", 1));
  CHECK_EQ(r->one(), ok(6, R"({"generation":")" + std::string(GEN) +
                           R"(","shutter_id":1,"state":"paired","last_command":"open"})"));
  r->send(slot_op(7, "provision", 3));
  CHECK_EQ(r->one(), fail(7, "profile_unverified"));
  r->send(slot_op(8, "pair", 1));
  CHECK_EQ(r->one(), fail(8, "profile_unverified"));
  r->send(slot_op(9, "confirm", 1));
  CHECK_EQ(r->one(), fail(9, "profile_unverified"));
  CHECK(flash.mutations() == mutations && r->radio.started == 1);
}

static void confirmation_requires_slot_authorization() {
  journal::MemoryFlash flash;
  journal::Journal seed(flash);
  CHECK(seed.open() == journal::StorageState::empty);
  CHECK(seed.provision(1, {0x1234AB, 0, sim::GENERATION}) == journal::Status::ok);
  CHECK(seed.provision(7, {0x9876CD, 100, sim::GENERATION}) == journal::Status::ok);
  auto r = make(flash, sim::SESSION, false, true, true);
  const std::vector<uint8_t> before(flash.raw(), flash.raw() + journal::REGION_BYTES);
  const uint32_t mutations = flash.mutations();
  r->send(hello(1));
  r->take();
  r->send(slot_op(2, "pair", 7));
  CHECK_EQ(r->one(), fail(2, "profile_unverified"));
  r->send(slot_op(3, "confirm", 7));
  CHECK_EQ(r->one(), fail(3, "profile_unverified"));
  r->send(slot_op(3, "confirm", 7));  // a duplicate rejection must stay inert
  CHECK_EQ(r->one(), fail(3, "profile_unverified"));
  r->send(command(4, 7, "open"));
  CHECK_EQ(r->one(), fail(4, "not_paired"));
  CHECK(r->journal.shutter(7).state == journal::SlotState::pending);
  CHECK(r->next(7) == 100 && r->radio.started == 0);
  CHECK(!memcmp(before.data(), flash.raw(), before.size()));
  CHECK(flash.mutations() == mutations);
  r->send(slot_op(5, "confirm", 1));
  CHECK(r->one().find(R"("state":"paired")") != std::string::npos);
  CHECK(r->journal.shutter(1).state == journal::SlotState::paired);
  CHECK(r->next(1) == 0 && r->radio.started == 0);
}

static void supervised_trial_enrollment() {
  journal::MemoryFlash flash;
  auto r = make(flash, sim::SESSION, false, true, true);
  r->send(hello(1));
  CHECK_EQ(r->one(), hello_ok(sim::SESSION,
                              R"(["status","shutters","command","provision","pair","confirm"])"));
  r->send(op(2, "status"));
  CHECK(r->one().find(R"("storage":{"state":"empty","generation":null})") != std::string::npos);
  r->send(op(3, "shutters"));
  CHECK_EQ(r->one(), ok(3, R"({"generation":null,"shutters":[]})"));

  r->send(slot_op(4, "provision", 2));  // the trial only allows slot 1
  CHECK_EQ(r->one(), fail(4, "profile_unverified"));
  r->send(slot_op(5, "provision", 1));
  CHECK_EQ(r->one(), ok(5, R"({"generation":"1122334455667788","shutter_id":1,)"
                           R"("state":"pending","last_command":null})"));
  uint32_t identity = 0;
  CHECK(r->journal.identity(1, &identity) && identity == 0x00ABCD5A);
  r->send(slot_op(6, "provision", 1));  // idempotent
  CHECK(r->one().find(R"("ok":true)") != std::string::npos);
  r->send(command(7, 1, "open"));  // pending is not paired
  CHECK_EQ(r->one(), fail(7, "not_paired"));

  r->send(slot_op(8, "pair", 1));
  CHECK_EQ(r->one(), ack(8, 1));
  r->send(slot_op(9, "confirm", 1));
  CHECK_EQ(r->one(), fail(9, "maintenance_pending"));
  r->send(slot_op(10, "pair", 1));
  CHECK_EQ(r->one(), fail(10, "maintenance_pending"));
  uint32_t guard = 0;
  while (r->pending.empty() && guard++ < 4000) r->run(1);
  CHECK_EQ(r->one(), event(1, 8, 1, "emitted", 48));  // 24 + 24 copies, one event
  CHECK(r->radio.bursts.size() == 2);
  CHECK(r->radio.bursts[1].started_ms - r->radio.bursts[0].started_ms == 2001);
  for (uint8_t phase = 0; phase < 2; ++phase) {
    radio::Body body;
    radio::Waveform expected;
    CHECK(radio::make_enrollment_body(identity, phase, phase, &body));
    CHECK(radio::encode_burst(body, 24, &expected));
    CHECK(same_wave(r->radio.bursts[phase].wave, expected));
  }
  CHECK(r->next(1) == 2);

  r->send(slot_op(11, "pair", 1));  // the expected counter is gone for good
  CHECK_EQ(r->one(), fail(11, "profile_unverified"));
  r->send(slot_op(12, "confirm", 1));
  CHECK_EQ(r->one(), ok(12, R"({"generation":"1122334455667788","shutter_id":1,)"
                            R"("state":"paired","last_command":null})"));
  r->send(command(13, 1, "open"));
  CHECK_EQ(r->one(), ack(13, 1));
  r->run(250);
  CHECK_EQ(r->one(), event(2, 13, 1, "emitted", 25));
  CHECK(r->next(1) == 3);
}

static void reboot_keeps_counters_and_never_replays() {
  journal::MemoryFlash flash;
  {
    auto before = make(flash);
    before->send(hello(1));
    before->take();
    before->send(command(2, 1, "open"));
    before->send(command(3, 2, "open"));  // still queued when power is lost
    before->take();
    before->run(20);
  }
  const char* session = "AAAAAAAAAAAAAAAA";
  auto r = make(flash, session);
  r->run(500);
  CHECK(r->take().empty() && r->radio.started == 0);  // no replay, no RF at boot
  CHECK(r->next(1) == 101 && r->next(2) == 200);

  r->send(op(1, "status"));  // silent until hello, even with a valid old session
  CHECK(r->take().empty());
  r->send(hello(1));
  CHECK_EQ(r->one(), hello_ok(session));
  r->send(op(2, "status", "{}", sim::SESSION));
  CHECK_EQ(r->one(), fail(2, "stale_session"));
  r->send(command(3, 1, "open", session));
  CHECK_EQ(r->one(), ack(3, 1));
  r->run(250);
  CHECK_EQ(r->one(), R"({"v":2,"session":"AAAAAAAAAAAAAAAA","seq":1,"event":"tx_result",)"
                     R"("request_id":3,"shutter_id":1,"result":"emitted","completed_copies":25})");
  expect_burst(r->radio.bursts[0], 1, 0x81, 101);  // never reuses 100
}

static void corrupt_storage_is_never_formatted() {
  journal::MemoryFlash flash;
  make(flash);  // a paired journal
  flash.raw()[30] ^= 0x01;  // body page no longer matches its commit
  uint8_t snapshot[journal::REGION_BYTES];
  memcpy(snapshot, flash.raw(), sizeof(snapshot));
  auto r = make(flash);
  CHECK(r->journal.state() == journal::StorageState::corrupt);
  r->send(hello(1));
  r->take();
  r->send(op(2, "status"));
  CHECK(r->one().find(R"("storage":{"state":"corrupt","generation":null})") != std::string::npos);
  r->send(op(3, "shutters"));
  const std::string refused = r->one();
  CHECK_EQ(refused, fail(3, "storage_corrupt"));
  r->send(op(3, "shutters"));
  CHECK_EQ(r->one(), refused);
  r->send(command(4, 1, "open"));
  CHECK_EQ(r->one(), fail(4, "storage_corrupt"));
  r->send(slot_op(5, "provision", 1));
  CHECK_EQ(r->one(), fail(5, "storage_corrupt"));
  CHECK(!memcmp(snapshot, flash.raw(), sizeof(snapshot)) && r->radio.started == 0);
}

static void invalid_identity_fails_closed() {
  for (const char* bad : {"0123", "0123456789abcdef", "0123456789ABCDEG", ""}) {
    uint32_t clock = 0;
    journal::MemoryFlash flash;
    journal::Journal journal(flash);
    sim::open_paired(journal);
    sim::Radio radio(clock);
    sim::Hooks hooks(clock);
    auto gateway = std::make_unique<sim::Gateway>(journal, radio, hooks, bad, sim::SESSION);
    const std::string line = hello(1) + "\n";
    CHECK(gateway->failed() && gateway->feed(line.data(), line.size()) == 0);
    gateway->tick();
    gateway->disconnected();
    CHECK(gateway->failed() && gateway->output_size() == 0);
  }
}

int main() {
  handshake_and_framing();
  command_ack_event_and_bodies();
  duplicates_staleness_and_cache();
  stop_preempts_in_event_order();
  disconnect_keeps_runtime_and_never_replays();
  output_overflow_fails_closed();
  policy_gates_are_dynamic_and_default_closed();
  confirmation_requires_slot_authorization();
  supervised_trial_enrollment();
  reboot_keeps_counters_and_never_replays();
  corrupt_storage_is_never_formatted();
  invalid_identity_fails_closed();
  puts("OK check_gateway: handshake, framing, events, STOP, cache, disconnect, overflow, gates, trial, reboot, storage");
}
