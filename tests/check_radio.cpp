// Host checks for the radio codec and the durable journal. No hardware.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

// Arduino core macros (Print.h, ArduinoCore-API) that the firmware headers
// must survive: a sketch includes them after Arduino.h.
#define HEX 16
#define DEC 10
#define OCT 8
#define BIN 2
#define HIGH 0x1
#define LOW 0x0
#define bit(b) (1UL << (b))
#define word(...) makeWord(__VA_ARGS__)
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define round(x) ((x) >= 0 ? (long)((x) + 0.5) : (long)((x) - 0.5))

#include "journal.h"
#include "radio_codec.h"

// Codec, journal, queue and runtime are plain C++17: ArduinoJson belongs to the gateway only.
#ifdef ARDUINOJSON_VERSION
#error "radio/journal/runtime headers must not include ArduinoJson"
#endif

using namespace ha_x2d;

#define CHECK(condition)                                                      \
  do {                                                                        \
    if (!(condition)) {                                                       \
      fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
      abort();                                                                \
    }                                                                         \
  } while (0)

static std::string hex(const uint8_t* data, size_t length) {
  static const char digits[] = "0123456789abcdef";
  std::string out;
  for (size_t i = 0; i < length; ++i) {
    out += digits[data[i] >> 4];
    out += digits[data[i] & 15];
  }
  return out;
}

// ---------------------------------------------------------------- codec

// Independent receiver-style decoder: chips -> biphase-mark check -> logical
// bits -> separators -> unstuff -> LSB-first bytes.
struct Decoded {
  bool ok = false;
  std::vector<std::string> bodies;  // hex
  std::vector<size_t> end_chips;
};

static Decoded decode_burst(const radio::Waveform& wave) {
  Decoded result;
  const size_t chips = wave.chips();
  if (chips % 2 || !wave.chip(0)) return result;  // starts from carrier off
  std::string logical;
  for (size_t i = 0; i < chips; i += 2) {
    if (i && wave.chip(i) == wave.chip(i - 1)) return result;  // boundary toggle
    logical += (wave.chip(i) != wave.chip(i + 1)) ? '1' : '0';
  }
  const std::string preamble(radio::PREAMBLE_ZERO_BITS, '0');
  if (logical.compare(0, preamble.size(), preamble) != 0) return result;
  size_t at = preamble.size();
  while (at < logical.size()) {
    if (logical.compare(at, 7, "1111110") != 0) return result;
    at += 7;
    const size_t stop = logical.find("111111110", at);
    if (stop == std::string::npos) return result;
    std::string bits;
    int ones = 0;
    for (size_t i = at; i < stop; ++i) {
      if (ones == 5) {
        if (logical[i] != '0') return result;  // sixth 1 is never data
        ones = 0;
        continue;
      }
      bits += logical[i];
      ones = logical[i] == '1' ? ones + 1 : 0;
    }
    if (bits.size() % 8 || bits.size() < 16 || bits.size() > radio::MAX_BODY_BYTES * 8)
      return result;
    uint8_t body[radio::MAX_BODY_BYTES] = {};
    for (size_t i = 0; i < bits.size(); ++i)
      if (bits[i] == '1') body[i / 8] |= static_cast<uint8_t>(1u << (i % 8));
    result.bodies.push_back(hex(body, bits.size() / 8));
    at = stop + 9;
    result.end_chips.push_back(at * 2);
  }
  result.ok = true;
  return result;
}

static void check_codec() {
  using namespace radio;
  for (const auto action : {Action::open, Action::close, Action::stop}) {
    Body body;
    ParsedBody parsed;
    const uint8_t expected = action == Action::open ? 0x81 : action == Action::close ? 0x82 : 0x04;
    CHECK(make_command_body(0x123456, action, 123, &body));
    CHECK(parse_body(body.bytes, body.length, &parsed));
    CHECK(parsed.identity == 0x123456 && parsed.counter == 123 && parsed.action == expected);
  }
  Body invalid;
  CHECK(!make_command_body(1, Action::none, 0, &invalid));
  CHECK(!make_command_body(1, static_cast<Action>(255), 0, &invalid));
  CHECK(!make_command_body(1, Action::open, 0, nullptr));
  // Public HACF export (identity F73192, counters 6641-6644) and public X3D
  // frames from diorcety/X2D raw_x3d.bin; Python reference transform.
  const struct { uint32_t id; uint16_t counter, word; } vectors[] = {
      {0xF73192, 6641, 0xE2DC}, {0xF73192, 6642, 0xC07A},
      {0xF73192, 6643, 0xE212}, {0xF73192, 6644, 0x1071},
      {0x186054, 140, 0x116A},  {0x186054, 141, 0x0BE8},
      {0x186054, 142, 0x1942},  {0x186091, 3032, 0x19A2},
      {0x186091, 3033, 0xEBB0}, {0x186091, 3034, 0x191E},
  };
  for (const auto& v : vectors) {
    CHECK(rolling_encode(v.counter, v.id) == v.word);
    CHECK(rolling_decode(v.word, v.id) == v.counter);
  }
  const uint32_t identities[] = {0xF73192, 0x186054, 0x000001, 0xFFFFFF};
  for (uint32_t id : identities)
    for (uint32_t value = 0; value < 65536; ++value)
      CHECK(rolling_decode(rolling_encode(static_cast<uint16_t>(value), id), id) == value);
  // The key is symmetric under reversal of the identity bytes.
  CHECK(rolling_encode(1234, 0x3B2F52) == rolling_encode(1234, 0x522F3B));

  // Bodies for the public identity with action 04: the checksum equals the
  // negated sum of the HACF header (the 00 pad byte adds nothing).
  const char* bodies[] = {"f731920105982204dce2fbc4", "f7319201059822047ac0fc48",
                          "f73192010598220412e2fc8e", "f7319201059822047110fd01"};
  for (int i = 0; i < 4; ++i) {
    Body body;
    CHECK(make_body(0xF73192, 0x04, static_cast<uint16_t>(6641 + i), &body));
    CHECK(hex(body.bytes, BODY_BYTES) == bodies[i]);
    ParsedBody parsed;
    CHECK(parse_body(body.bytes, BODY_BYTES, &parsed));
    CHECK(parsed.identity == 0xF73192 && parsed.action == 0x04 &&
          parsed.counter == 6641 + i);
  }
  Body body;
  CHECK(!make_body(0x1000000, 4, 0, &body) && !make_body(1, 4, 0, nullptr));
  CHECK(make_body(0xF73192, 0x04, 6641, &body));
  ParsedBody parsed;
  CHECK(!parse_body(body.bytes, BODY_BYTES - 1, &parsed));
  for (size_t i = 0; i < BODY_BYTES; ++i)
    for (int bit = 0; bit < 8; ++bit) {  // every single-bit error is rejected
      Body bad = body;
      bad.bytes[i] ^= static_cast<uint8_t>(1u << bit);
      CHECK(!parse_body(bad.bytes, BODY_BYTES, &parsed));
    }

  // Burst: decode with the independent decoder, for ordinary and extreme bodies.
  Body extremes[3];
  memset(extremes[0].bytes, 0xFF, BODY_BYTES);  // maximum stuffing
  memset(extremes[1].bytes, 0x00, BODY_BYTES);
  memset(extremes[2].bytes, 0x1F, BODY_BYTES);  // five 1s at the end of each byte
  extremes[2].bytes[11] = 0xF8;                  // data ends with five 1s
  static Waveform wave;
  // Synthetic public identity, counters 0/1, checked independently with the
  // Python rolling reference. These are not captured installation bodies.
  const char* enrollment[] = {"f731920185982202d410fc20", "f73192010598222007d610fc79"};
  for (uint8_t phase = 0; phase < 2; ++phase) {
    Body candidate;
    CHECK(make_enrollment_body(0xF73192, phase, phase, &candidate));
    CHECK(hex(candidate.bytes, candidate.length) == enrollment[phase]);
    CHECK(encode_burst(candidate, 24, &wave));
    const Decoded decoded = decode_burst(wave);
    CHECK(decoded.ok && decoded.bodies.size() == 24);
    for (const auto& item : decoded.bodies) CHECK(item == enrollment[phase]);
  }
  CHECK(!make_enrollment_body(1, 0, 2, &body));
  CHECK(!make_enrollment_body(1, 0, 0, nullptr));
  // Extended framing uses synthetic data, never an installation identity or
  // an assumed enrollment layout. Exercise maximum stuffing and every copy.
  for (uint8_t length : {uint8_t{13}, uint8_t{15}}) {
    Body extended;
    extended.length = length;
    memset(extended.bytes, 0xFF, length);
    CHECK(encode_burst(extended, MAX_COPIES, &wave));
    const Decoded decoded = decode_burst(wave);
    CHECK(decoded.ok && decoded.bodies.size() == MAX_COPIES);
    for (const auto& decoded_body : decoded.bodies)
      CHECK(decoded_body == hex(extended.bytes, length));
    CHECK(wave.frame_end(MAX_COPIES - 1) == wave.chips());
  }
  Body invalid_length;
  invalid_length.length = MAX_BODY_BYTES + 1;
  CHECK(!encode_burst(invalid_length, 1, &wave));
  for (uint8_t copies : {uint8_t{1}, uint8_t{2}, uint8_t{25}, uint8_t{32}}) {
    for (const Body& candidate : {body, extremes[0], extremes[1], extremes[2]}) {
      CHECK(encode_burst(candidate, copies, &wave));
      CHECK(wave.chips() <= MAX_CHIPS && wave.copies() == copies);
      const Decoded decoded = decode_burst(wave);
      CHECK(decoded.ok && decoded.bodies.size() == copies);
      for (uint8_t i = 0; i < copies; ++i) {
        CHECK(decoded.bodies[i] == hex(candidate.bytes, BODY_BYTES));
        CHECK(wave.frame_end(i) == decoded.end_chips[i]);
      }
      CHECK(wave.frame_end(copies - 1) == wave.chips() && wave.frame_end(copies) == 0);
    }
  }
  // Layout facts taken from the captures: 0^8, 1^6 0, data, 1^8 0, 1^6 0, ...
  CHECK(encode_burst(body, 2, &wave));
  std::string chips;
  for (size_t i = 0; i < wave.chips(); ++i) chips += wave.chip(i) ? '1' : '0';
  CHECK(chips.compare(0, 16, "1100110011001100") == 0);  // eight 0 bits
  CHECK(chips.compare(16, 14, "10101010101011") == 0);   // 1^6 then 0 (chips)
  // Copies are back to back: separator, data plus stuffed zeros, terminator
  // (captures: 114 bits = 228 chips apart for a body with two stuffed zeros).
  size_t stuffed = 0;
  int run = 0;
  for (size_t i = 0; i < 96; ++i) {
    run = ((body.bytes[i / 8] >> (i % 8)) & 1) ? run + 1 : 0;
    if (run == 5) {
      ++stuffed;
      run = 0;
    }
  }
  CHECK(wave.frame_end(1) - wave.frame_end(0) == 2 * (7 + 96 + stuffed + 9));

  // Packed form and pulses.
  for (size_t i = 0; i < wave.chips(); ++i)
    CHECK(((wave.packed()[i / 8] >> (7 - i % 8)) & 1) == wave.chip(i));
  CHECK(wave.packed_bytes() == (wave.chips() + 7) / 8);
  static uint8_t pulses[MAX_CHIPS];
  const size_t runs = to_pulses(wave, pulses, sizeof(pulses));
  CHECK(runs > 0 && to_pulses(wave, pulses, runs - 1) == 0);
  size_t position = 0;
  bool level = wave.chip(0);
  for (size_t i = 0; i < runs; ++i) {
    CHECK(pulses[i] == 1 || pulses[i] == 2);
    for (uint8_t k = 0; k < pulses[i]; ++k) CHECK(wave.chip(position++) == level);
    level = !level;
  }
  CHECK(position == wave.chips());

  // Bounds and convenience entry point.
  CHECK(!encode_burst(body, 0, &wave) && !encode_burst(body, MAX_COPIES + 1, &wave));
  CHECK(!encode_burst(body, 1, nullptr) && !encode(0x1000000, 4, 0, 1, &wave));
  static Waveform other;
  CHECK(encode(0xF73192, 0x04, 6641, 3, &other) && encode_burst(body, 3, &wave));
  CHECK(other.chips() == wave.chips() &&
        memcmp(other.packed(), wave.packed(), wave.packed_bytes()) == 0);
  CHECK(!wave.chip(wave.chips()));
}

// -------------------------------------------------------------- journal

using journal::Journal;
using journal::MemoryFlash;
using journal::NewController;
using journal::Reservation;
using journal::Shutter;
using journal::SlotState;
using journal::Status;
using journal::StorageState;

static const uint64_t GENERATION = 0x0123456789ABCDEFull;

static NewController fresh(uint32_t identity, uint16_t first = 100,
                           uint64_t generation = GENERATION) {
  NewController value;
  value.identity = identity;
  value.first_counter = first;
  value.generation = generation;
  return value;
}

static void check_journal_basics() {
  MemoryFlash flash;
  Journal j(flash);
  CHECK(j.state() == StorageState::corrupt);  // not opened: fail closed
  CHECK(j.provision(1, fresh(1)) == Status::corrupt);
  CHECK(j.open() == StorageState::empty && flash.mutations() == 0);
  char text[17];
  CHECK(!j.generation_hex(text) && j.shutter(1).state == SlotState::unused);
  Reservation reservation;
  CHECK(j.reserve(1, 3, true, &reservation) == Status::unknown_slot);
  CHECK(j.confirm(1) == Status::unknown_slot && j.maintain() == Status::ok);
  CHECK(flash.mutations() == 0);
  CHECK(j.provision(1, fresh(0xABCDE1, 100, 0)) == Status::bad_argument);
  CHECK(j.provision(0, fresh(1)) == Status::unknown_slot);
  CHECK(j.provision(17, fresh(1)) == Status::unknown_slot);
  CHECK(j.provision(1, fresh(0)) == Status::bad_argument);
  CHECK(j.provision(1, fresh(0x1000000)) == Status::bad_argument);
  CHECK(flash.mutations() == 0 && j.state() == StorageState::empty);

  Shutter shutter;
  bool created = false;
  CHECK(j.provision(3, fresh(0xABCDE1), &shutter, &created) == Status::ok);
  CHECK(created && shutter.shutter_id == 3 && shutter.state == SlotState::pending);
  CHECK(j.state() == StorageState::ready && j.generation_hex(text));
  CHECK(std::string(text) == "0123456789ABCDEF");
  const uint32_t writes = flash.mutations();
  CHECK(j.provision(3, fresh(0x123456, 7, 99), &shutter, &created) == Status::ok);
  CHECK(!created && shutter.state == SlotState::pending && flash.mutations() == writes);
  uint32_t identity = 0;
  CHECK(j.identity(3, &identity) && identity == 0xABCDE1 && j.find_identity(0xABCDE1) == 3);
  CHECK(j.find_identity(0x123456) == 0 && !j.identity(4, &identity));
  CHECK(j.provision(4, fresh(0xABCDE1)) == Status::identity_in_use);
  CHECK(j.provision(4, fresh(0xABCDE2, 5, 99)) == Status::ok);  // generation kept
  CHECK(j.generation_hex(text) && std::string(text) == "0123456789ABCDEF");

  CHECK(j.confirm(3, &shutter) == Status::ok && shutter.state == SlotState::paired);
  const uint32_t before = flash.mutations();
  CHECK(j.confirm(3) == Status::ok && flash.mutations() == before);

  // Counters: durable before use, abandoned reservations stay consumed.
  CHECK(j.reserve(3, 3, false, &reservation) == Status::ok);
  CHECK(reservation.shutter_id == 3 && reservation.identity == 0xABCDE1 &&
        reservation.counter == 100);
  CHECK(j.reserve(3, 1, false, &reservation) == Status::ok && reservation.counter == 101);
  CHECK(j.shutter(3).last_command == 1);
  Journal again(flash);
  CHECK(again.open() == StorageState::ready);
  CHECK(again.shutter(3).state == SlotState::paired && again.shutter(4).state == SlotState::pending);
  CHECK(again.shutter(3).last_command == 1 && again.find_identity(0xABCDE2) == 4);
  CHECK(again.reserve(3, 2, false, &reservation) == Status::ok && reservation.counter == 102);
  CHECK(again.reserve(4, 0, false, &reservation) == Status::ok && reservation.counter == 5);
  CHECK(flash.erases() == 0 && flash.violations() == 0);

  // No wrap: 0xFFFF is the last counter of an identity.
  CHECK(again.provision(5, fresh(0x555555, 0xFFFE)) == Status::ok);
  CHECK(again.reserve(5, 0, true, &reservation) == Status::ok && reservation.counter == 0xFFFE);
  CHECK(again.reserve(5, 0, true, &reservation) == Status::ok && reservation.counter == 0xFFFF);
  CHECK(again.reserve(5, 0, true, &reservation) == Status::exhausted);
  Journal third(flash);
  CHECK(third.open() == StorageState::ready);
  CHECK(third.reserve(5, 0, true, &reservation) == Status::exhausted);
  CHECK(third.reserve(3, 0, true, &reservation) == Status::ok && reservation.counter == 103);
  CHECK(std::string(journal::protocol_error(Status::exhausted)) == "counter_exhausted");
  CHECK(journal::protocol_error(Status::ok) == nullptr);
}

static void drain_maintenance(Journal& j, MemoryFlash& flash, int* steps = nullptr) {
  int count = 0;
  while (j.maintenance_due()) {
    const uint32_t before = flash.mutations();
    CHECK(j.maintain() == Status::ok);
    CHECK(flash.mutations() - before <= 2);  // one erase or one record
    CHECK(++count < 40);
  }
  if (steps) *steps = count;
}

static void check_journal_space() {
  MemoryFlash flash;
  Journal j(flash);
  CHECK(j.open() == StorageState::empty);
  CHECK(j.provision(1, fresh(0x111111, 0)) == Status::ok);
  Reservation reservation;
  uint32_t expected = 0;
  uint32_t programs_before = flash.programs();
  while (true) {
    const Status status = j.reserve(1, 3, false, &reservation);
    if (status == Status::no_space) break;
    CHECK(status == Status::ok && reservation.counter == expected++);
  }
  CHECK(j.state() == StorageState::ready);  // headroom left for STOP
  uint32_t critical = 0;
  while (j.reserve(1, 3, true, &reservation) == Status::ok) {
    CHECK(reservation.counter == expected++);
    ++critical;
  }
  CHECK(critical == journal::CRITICAL_PAIRS && j.state() == StorageState::full);
  CHECK(flash.programs() - programs_before == 2 * (expected + 0u));
  CHECK(flash.erases() == 0);  // nothing erased on the reserve path
  CHECK(j.confirm(1) == Status::no_space);

  // First rotation: the other bank is blank, so one record is enough.
  int steps = 0;
  drain_maintenance(j, flash, &steps);
  CHECK(steps == 1 && j.state() == StorageState::ready);
  char text[17];
  CHECK(j.generation_hex(text) && std::string(text) == "0123456789ABCDEF");
  CHECK(j.reserve(1, 3, false, &reservation) == Status::ok && reservation.counter == expected++);
  Journal reopened(flash);
  CHECK(reopened.open() == StorageState::ready);
  CHECK(reopened.reserve(1, 3, false, &reservation) == Status::ok && reservation.counter == expected++);

  // Second rotation must erase the stale bank, one sector per call.
  while (reopened.reserve(1, 3, true, &reservation) == Status::ok) CHECK(reservation.counter == expected++);
  const uint32_t erases = flash.erases();
  drain_maintenance(reopened, flash, &steps);
  CHECK(flash.erases() - erases == journal::SECTORS_PER_BANK && steps == 9);
  CHECK(reopened.reserve(1, 3, false, &reservation) == Status::ok && reservation.counter == expected++);
  Journal last(flash);
  CHECK(last.open() == StorageState::ready && last.shutter(1).state == SlotState::pending);
  CHECK(last.reserve(1, 3, false, &reservation) == Status::ok && reservation.counter == expected);
  CHECK(flash.violations() == 0);
}

static void patch_record_crcs(uint8_t* pair) {  // body at pair, commit at pair + 256
  using namespace journal::detail;
  put32(pair + CRC_AT, crc32(pair, CRC_AT));
  uint8_t* commit = pair + journal::PAGE_BYTES;
  put32(commit + 4, get32(pair + 16));
  put32(commit + 8, get32(pair + CRC_AT));
  put32(commit + 12, crc32(commit, 12));
}

static void check_journal_corruption() {
  auto prepare = [](MemoryFlash& flash, Journal& j) {
    CHECK(j.open() == StorageState::empty);
    CHECK(j.provision(1, fresh(0x111111, 10)) == Status::ok);
    CHECK(j.confirm(1) == Status::ok);
    Reservation reservation;
    for (int i = 0; i < 3; ++i) CHECK(j.reserve(1, 1, false, &reservation) == Status::ok);
    (void)flash;
  };
  // Records live at pair p: bytes [p * 512, p * 512 + 512). Five records exist.
  struct Case { const char* name; void (*damage)(MemoryFlash&); };
  const Case cases[] = {
      {"newest committed body bit flip", [](MemoryFlash& f) { f.raw()[4 * 512 + 40] ^= 1; }},
      {"older body bit flip under a valid commit", [](MemoryFlash& f) { f.raw()[2 * 512 + 40] ^= 1; }},
      {"middle commit page destroyed", [](MemoryFlash& f) { f.raw()[2 * 512 + 256 + 3] ^= 1; }},
      {"data after free space", [](MemoryFlash& f) { f.raw()[30 * 512] = 0x00; }},
      {"equal sequence in both banks", [](MemoryFlash& f) {
         memcpy(f.raw() + journal::BANK_BYTES, f.raw(), journal::BANK_BYTES); }},
      {"unknown version with valid CRCs", [](MemoryFlash& f) {
         uint8_t* pair = f.raw() + 4 * 512;
         pair[4] = 2;
         patch_record_crcs(pair); }},
      {"slot counter regression with valid CRCs", [](MemoryFlash& f) {
         uint8_t* next = f.raw() + 5 * 512;
         memcpy(next, f.raw() + 4 * 512, 512);
         using namespace journal::detail;
         put32(next + 16, get32(next + 16) + 1);
         put32(next + SLOTS_AT + 4, 3);  // slot 1 counter moves backwards
         patch_record_crcs(next); }},
      {"identity swapped with valid CRCs", [](MemoryFlash& f) {
         uint8_t* next = f.raw() + 5 * 512;
         memcpy(next, f.raw() + 4 * 512, 512);
         using namespace journal::detail;
         put32(next + 16, get32(next + 16) + 1);
         put32(next + SLOTS_AT, 0x222222);
         patch_record_crcs(next); }},
      {"commit without a body", [](MemoryFlash& f) { memset(f.raw() + 4 * 512, 0xFF, 256); }},
      {"intact body two sequences ahead, no commit", [](MemoryFlash& f) {
         uint8_t* next = f.raw() + 5 * 512;
         memcpy(next, f.raw() + 4 * 512, 256);
         using namespace journal::detail;
         put32(next + 16, get32(next + 16) + 2);
         put32(next + CRC_AT, crc32(next, CRC_AT)); }},
      {"intact body with another identity, no commit", [](MemoryFlash& f) {
         uint8_t* next = f.raw() + 5 * 512;
         memcpy(next, f.raw() + 4 * 512, 256);
         using namespace journal::detail;
         put32(next + 16, get32(next + 16) + 1);
         put32(next + SLOTS_AT, 0x222222);
         put32(next + CRC_AT, crc32(next, CRC_AT)); }},
  };
  for (const Case& test : cases) {
    MemoryFlash flash;
    {
      Journal j(flash);
      prepare(flash, j);
    }
    test.damage(flash);
    std::vector<uint8_t> snapshot(flash.raw(), flash.raw() + journal::REGION_BYTES);
    Journal j(flash);
    if (j.open() != StorageState::corrupt) {
      fprintf(stderr, "case not detected: %s\n", test.name);
      abort();
    }
    Reservation reservation;
    char text[17];
    CHECK(j.reserve(1, 1, true, &reservation) == Status::corrupt);
    CHECK(j.provision(2, fresh(0x222223)) == Status::corrupt && j.confirm(1) == Status::corrupt);
    CHECK(j.maintain() == Status::corrupt && !j.generation_hex(text));
    CHECK(j.shutter(1).state == SlotState::unused);  // nothing is exposed
    CHECK(memcmp(snapshot.data(), flash.raw(), snapshot.size()) == 0);  // never formatted
    CHECK(std::string(journal::protocol_error(Status::corrupt)) == "storage_corrupt");
  }
  // Blank is not corruption; an interrupted first write is recoverable.
  MemoryFlash blank;
  Journal empty(blank);
  CHECK(empty.open() == StorageState::empty && !empty.torn_tail_skipped());
  MemoryFlash torn;
  memset(torn.raw(), 0x00, 100);  // half-written first body, no commit anywhere
  Journal recovered(torn);
  CHECK(recovered.open() == StorageState::empty && recovered.torn_tail_skipped());
  CHECK(recovered.provision(1, fresh(0x111111, 0)) == Status::ok);
  Journal after(torn);
  CHECK(after.open() == StorageState::ready && after.shutter(1).state == SlotState::pending);
  CHECK(torn.violations() == 0);
  // An intact successor body without a commit burns its counters.
  MemoryFlash burnt;
  {
    Journal j(burnt);
    prepare(burnt, j);  // slot 1: counters 10, 11, 12 emitted, next is 13
    uint8_t* next = burnt.raw() + 5 * 512;
    memcpy(next, burnt.raw() + 4 * 512, 256);
    using namespace journal::detail;
    put32(next + 16, get32(next + 16) + 1);
    put32(next + SLOTS_AT + 4, 20);
    put32(next + CRC_AT, crc32(next, CRC_AT));
  }
  Journal forward(burnt);
  Reservation burnt_reservation;
  CHECK(forward.open() == StorageState::ready && forward.torn_tail_skipped());
  CHECK(forward.reserve(1, 1, true, &burnt_reservation) == Status::ok &&
        burnt_reservation.counter == 20);
  Journal forward_again(burnt);
  CHECK(forward_again.open() == StorageState::ready && !forward_again.torn_tail_skipped());
  CHECK(forward_again.reserve(1, 1, true, &burnt_reservation) == Status::ok &&
        burnt_reservation.counter == 21);
  // A stale bank full of garbage beside a good bank is ignored.
  MemoryFlash stale;
  {
    Journal j(stale);
    prepare(stale, j);
  }
  memset(stale.raw() + journal::BANK_BYTES, 0xA5, journal::BANK_BYTES);
  Journal beside(stale);
  CHECK(beside.open() == StorageState::ready);

  // A backend of the wrong size is refused.
  struct Small final : journal::Flash {
    uint32_t size() const override { return 4096; }
    bool read(uint32_t, void*, uint32_t) override { return false; }
    bool erase_sector(uint32_t) override { return false; }
    bool program_page(uint32_t, const uint8_t*) override { return false; }
  } small;
  Journal refused(small);
  CHECK(refused.open() == StorageState::corrupt);
}

static void check_journal_interruptions() {
  MemoryFlash flash;
  Journal j(flash);
  CHECK(j.open() == StorageState::empty && j.provision(1, fresh(0x111111, 5)) == Status::ok);
  Reservation reservation;
  CHECK(j.reserve(1, 1, true, &reservation) == Status::ok && reservation.counter == 5);
  CHECK(j.reserve(1, 1, true, &reservation) == Status::ok && reservation.counter == 6);
  flash.cut_after(0, 100);  // power lost while programming the body page
  CHECK(j.reserve(1, 1, true, &reservation) == Status::io_error);
  CHECK(j.state() == StorageState::corrupt && j.reserve(1, 1, true, &reservation) == Status::corrupt);
  flash.restore_power();
  Journal after(flash);
  CHECK(after.open() == StorageState::ready && after.torn_tail_skipped());
  CHECK(after.reserve(1, 1, true, &reservation) == Status::ok && reservation.counter == 7);
  flash.cut_after(1, 3);  // body page lands, commit page is torn
  CHECK(after.reserve(1, 1, true, &reservation) == Status::io_error);
  flash.restore_power();
  Journal next(flash);
  CHECK(next.open() == StorageState::ready && next.torn_tail_skipped());
  // The intact body may have been emitted by another failure mode, so its
  // counter (8) is burnt: the next reservation is 9, never 8.
  CHECK(next.reserve(1, 1, true, &reservation) == Status::ok && reservation.counter == 9);
  // A program that reports success but stored wrong bits is caught by the
  // read-back; the counter is not handed out.
  flash.flip_next_program();
  CHECK(next.reserve(1, 1, true, &reservation) == Status::io_error);
  Journal healed(flash);
  CHECK(healed.open() == StorageState::ready && healed.torn_tail_skipped());
  CHECK(healed.reserve(1, 1, true, &reservation) == Status::ok && reservation.counter == 10);
  CHECK(flash.violations() == 0);
}

// Power-cut matrix: cut at every erase/program of a long scenario with several
// tear sizes, recover, and require that no counter ever returned is handed out
// again, identities and generation survive, and operation continues.
struct Ledger {
  int64_t highest[journal::SLOTS + 1];
  bool confirmed[journal::SLOTS + 1];
  Ledger() {
    for (auto& value : highest) value = -1;
    for (auto& value : confirmed) value = false;
  }
};

static const uint32_t IDENTITY[4] = {0, 0x111111, 0x222222, 0x333333};
static const uint16_t FIRST[4] = {0, 10, 0xFFFD, 0};

static bool settle(Journal& j) {
  for (int i = 0; i < 40 && j.maintenance_due(); ++i)
    if (j.maintain() != Status::ok) return false;
  return true;
}

// False when a flash operation failed (power cut); CHECKs everything else.
static bool run_scenario(Journal& j, Ledger& ledger, int reserves) {
  for (uint8_t slot = 1; slot <= 3; ++slot) {
    const Status status = j.provision(slot, fresh(IDENTITY[slot], FIRST[slot]));
    if (status == Status::io_error) return false;
    CHECK(status == Status::ok);
  }
  for (uint8_t slot = 1; slot <= 2; ++slot) {
    const Status status = j.confirm(slot);
    if (status == Status::io_error) return false;
    CHECK(status == Status::ok);
    ledger.confirmed[slot] = true;
  }
  for (int i = 0; i < reserves; ++i) {
    const uint8_t slot = static_cast<uint8_t>(i % 3 + 1);
    Reservation reservation;
    const Status status = j.reserve(slot, static_cast<uint8_t>(i % 4), i % 5 == 0, &reservation);
    if (status == Status::io_error) return false;
    if (status == Status::exhausted) {
      CHECK(slot == 2);  // only the slot started near 0xFFFF; a cut may consume the last values
    } else {
      CHECK(status == Status::ok && reservation.identity == IDENTITY[slot]);
      CHECK(reservation.counter > ledger.highest[slot]);  // never reused
      ledger.highest[slot] = reservation.counter;
    }
    if (!settle(j)) return false;
  }
  return true;
}

static void check_recovered(Journal& j, const Ledger& ledger) {
  char text[17];
  if (j.state() != StorageState::empty) {
    CHECK(j.generation_hex(text) && std::string(text) == "0123456789ABCDEF");
  }
  for (uint8_t slot = 1; slot <= 3; ++slot) {
    if (ledger.confirmed[slot]) CHECK(j.shutter(slot).state == SlotState::paired);
    if (ledger.highest[slot] >= 0) {
      uint32_t identity = 0;
      CHECK(j.identity(slot, &identity) && identity == IDENTITY[slot]);
    }
  }
}

static void check_power_cuts() {
  uint32_t total = 0;
  {
    MemoryFlash flash;
    Journal j(flash);
    Ledger ledger;
    CHECK(j.open() == StorageState::empty && run_scenario(j, ledger, 220));
    CHECK(ledger.highest[2] == 0xFFFF && flash.erases() > journal::SECTORS_PER_BANK);  // two erasing rotations
    total = flash.mutations();
  }
  const uint32_t tears[] = {0, 1, 4, 9, 13, 24, 100, 251, 255, 256, 2000, 4096};
  uint32_t runs = 0;
  for (uint32_t cut = 0; cut < total; ++cut) {
    for (uint32_t torn : tears) {
      MemoryFlash flash;
      Ledger ledger;
      {
        Journal j(flash);
        CHECK(j.open() == StorageState::empty);
        flash.cut_after(cut, torn);
        CHECK(!run_scenario(j, ledger, 220));  // the cut always lands
      }
      CHECK(!flash.powered());
      flash.restore_power();
      Journal recovered(flash);
      CHECK(recovered.open() != StorageState::corrupt);
      check_recovered(recovered, ledger);
      CHECK(run_scenario(recovered, ledger, 60));  // continue, no reuse
      CHECK(flash.violations() == 0);
      Journal final_open(flash);
      CHECK(final_open.open() == StorageState::ready);
      check_recovered(final_open, ledger);
      ++runs;
    }
  }
  // Two interruptions in a row, deterministic pseudo-random cut points.
  uint32_t seed = 12345;
  for (int i = 0; i < 400; ++i) {
    seed = seed * 1664525u + 1013904223u;
    MemoryFlash flash;
    Ledger ledger;
    uint32_t cut = (seed >> 8) % total;
    uint32_t torn = tears[(seed >> 4) % 12];
    {
      Journal j(flash);
      CHECK(j.open() == StorageState::empty);
      flash.cut_after(cut, torn);
      CHECK(!run_scenario(j, ledger, 220));
    }
    flash.restore_power();
    for (int again = 0; again < 2; ++again) {
      seed = seed * 1664525u + 1013904223u;
      Journal j(flash);
      CHECK(j.open() != StorageState::corrupt);
      check_recovered(j, ledger);
      flash.cut_after((seed >> 8) % 40, tears[(seed >> 4) % 12]);
      if (run_scenario(j, ledger, 60)) flash.restore_power();  // cut may not land
      flash.restore_power();
    }
    Journal j(flash);
    CHECK(j.open() != StorageState::corrupt);
    CHECK(run_scenario(j, ledger, 60));
    CHECK(flash.violations() == 0);
  }
  printf("power cuts: %u single cuts over %u mutations, 400 double cuts\n", runs, total);
}

// Every single-bit flip in the newest pairs after counters were emitted must
// end as explicit corruption or as a state that never hands out an emitted
// counter again.
struct Sweep {
  uint32_t corrupt = 0, burnt = 0, unchanged = 0;
};

static uint32_t pair_of_sequence(MemoryFlash& flash, uint32_t sequence) {
  using namespace journal::detail;
  for (uint32_t offset = 0; offset < journal::REGION_BYTES; offset += 2 * journal::PAGE_BYTES) {
    Image image;
    if (classify(flash.raw() + offset, flash.raw() + offset + journal::PAGE_BYTES, &image) ==
            PairKind::committed && image.sequence == sequence)
      return offset;
  }
  CHECK(!"sequence not found");
  return 0;
}

static uint32_t newest_sequence(MemoryFlash& flash) {
  using namespace journal::detail;
  uint32_t newest = 0;
  for (uint32_t offset = 0; offset < journal::REGION_BYTES; offset += 2 * journal::PAGE_BYTES) {
    Image image;
    if (classify(flash.raw() + offset, flash.raw() + offset + journal::PAGE_BYTES, &image) ==
            PairKind::committed && image.sequence > newest)
      newest = image.sequence;
  }
  return newest;
}

static Sweep flip_sweep(MemoryFlash& source, uint32_t offset, int64_t highest) {
  static MemoryFlash copy;
  Sweep result;
  for (uint32_t byte = 0; byte < 2 * journal::PAGE_BYTES; ++byte) {
    for (int bit = 0; bit < 8; ++bit) {
      memcpy(copy.raw(), source.raw(), journal::REGION_BYTES);
      copy.raw()[offset + byte] ^= static_cast<uint8_t>(1u << bit);
      Journal j(copy);
      if (j.open() == StorageState::corrupt) {
        ++result.corrupt;
        continue;
      }
      if (j.torn_tail_skipped()) ++result.burnt; else ++result.unchanged;
      uint32_t identity = 0;
      CHECK(j.identity(1, &identity) && identity == IDENTITY[1]);
      CHECK(settle(j));
      Reservation reservation;
      CHECK(j.reserve(1, 0, true, &reservation) == Status::ok);
      CHECK(reservation.counter > highest);  // an emitted counter is never reused
    }
  }
  return result;
}

static void check_newest_page_bitflips() {
  // Plain log: provision, confirm, three emitted reservations (10, 11, 12).
  {
    MemoryFlash flash;
    Journal j(flash);
    CHECK(j.open() == StorageState::empty);
    CHECK(j.provision(1, fresh(IDENTITY[1], 10)) == Status::ok && j.confirm(1) == Status::ok);
    Reservation reservation;
    for (int i = 0; i < 3; ++i) CHECK(j.reserve(1, 1, false, &reservation) == Status::ok);
    CHECK(reservation.counter == 12);
    const uint32_t newest = newest_sequence(flash);
    // Newest record: body damage is corruption, commit damage burns its
    // counters, damage in the unused tail of the commit page changes nothing.
    const Sweep last = flip_sweep(flash, pair_of_sequence(flash, newest), 12);
    CHECK(last.corrupt == 8 * journal::PAGE_BYTES && last.burnt == 8 * 16 &&
          last.unchanged == 8 * (journal::PAGE_BYTES - 16));
    // Older records are covered by sequence continuity.
    for (uint32_t back = 1; back <= 2; ++back)
      flip_sweep(flash, pair_of_sequence(flash, newest - back), 12);
  }
  // Across a bank rotation: snapshot, then emitted reservations in the new bank.
  for (int emitted_after_rotation = 0; emitted_after_rotation <= 2; ++emitted_after_rotation) {
    MemoryFlash flash;
    Journal j(flash);
    CHECK(j.open() == StorageState::empty);
    CHECK(j.provision(1, fresh(IDENTITY[1], 0)) == Status::ok);
    Reservation reservation;
    int64_t highest = -1;
    while (!j.maintenance_due()) {
      CHECK(j.reserve(1, 1, false, &reservation) == Status::ok);
      highest = reservation.counter;
    }
    drain_maintenance(j, flash);
    for (int i = 0; i < emitted_after_rotation; ++i) {
      CHECK(j.reserve(1, 1, true, &reservation) == Status::ok);
      highest = reservation.counter;
    }
    const uint32_t newest = newest_sequence(flash);
    for (uint32_t back = 0; back <= 2; ++back)
      flip_sweep(flash, pair_of_sequence(flash, newest - back), highest);
  }
}

// STOP never needs an erase: 16 slots, movements until refused, then one STOP
// per slot. Maintenance is due long before movements are refused.
static void check_stop_headroom() {
  MemoryFlash flash;
  Journal j(flash);
  CHECK(j.open() == StorageState::empty);
  for (uint8_t slot = 1; slot <= journal::SLOTS; ++slot)
    CHECK(j.provision(slot, fresh(0x100000u + slot, 0)) == Status::ok);
  Reservation reservation;
  uint32_t movements = 0, after_due = 0;
  for (uint8_t slot = 1;; slot = static_cast<uint8_t>(slot % journal::SLOTS + 1)) {
    const bool due = j.maintenance_due();
    const Status status = j.reserve(slot, 1, false, &reservation);
    if (status == Status::no_space) break;
    CHECK(status == Status::ok);
    ++movements;
    if (due) ++after_due;
  }
  CHECK(movements > 0 && j.state() == StorageState::ready);
  CHECK(after_due == 16 && j.maintenance_due());  // 16 movements remain to finish maintenance
  CHECK(j.confirm(1) == Status::no_space);  // movements and confirms stop first
  for (uint8_t slot = 1; slot <= journal::SLOTS; ++slot)
    CHECK(j.reserve(slot, 3, true, &reservation) == Status::ok);  // 16 distinct STOPs
  CHECK(flash.erases() == 0 && flash.violations() == 0);
  CHECK(j.state() == StorageState::full);
  // Ceiling: no guarantee beyond 16 STOP records between maintenance runs.
  CHECK(j.reserve(1, 3, true, &reservation) == Status::no_space);
  Journal restarted(flash);
  CHECK(restarted.open() == StorageState::full);
  drain_maintenance(restarted, flash);
  for (uint8_t slot = 1; slot <= journal::SLOTS; ++slot)
    CHECK(restarted.reserve(slot, 3, true, &reservation) == Status::ok);
  CHECK(flash.violations() == 0);
}

// Counter 0xFFFF belongs to STOP: a movement is refused once next reaches it.
static void check_last_counter_for_stop() {
  MemoryFlash flash;
  Journal j(flash);
  CHECK(j.open() == StorageState::empty);
  CHECK(j.provision(1, fresh(0x111111, 0xFFFD)) == Status::ok);
  Reservation reservation;
  CHECK(j.reserve(1, 1, false, &reservation) == Status::ok && reservation.counter == 0xFFFD);
  CHECK(j.reserve(1, 1, false, &reservation) == Status::ok && reservation.counter == 0xFFFE);
  CHECK(j.reserve(1, 1, false, &reservation) == Status::exhausted);
  Journal again(flash);
  CHECK(again.open() == StorageState::ready);
  CHECK(again.reserve(1, 1, false, &reservation) == Status::exhausted);
  CHECK(again.reserve(1, 3, true, &reservation) == Status::ok && reservation.counter == 0xFFFF);
  CHECK(again.reserve(1, 3, true, &reservation) == Status::exhausted);
  // An identity provisioned at 0xFFFF can only ever send one STOP.
  CHECK(again.provision(2, fresh(0x222222, 0xFFFF)) == Status::ok);
  CHECK(again.reserve(2, 1, false, &reservation) == Status::exhausted);
  CHECK(again.reserve(2, 3, true, &reservation) == Status::ok && reservation.counter == 0xFFFF);
  CHECK(again.reserve(2, 3, true, &reservation) == Status::exhausted);
  CHECK(flash.erases() == 0);
}

int main() {
  using journal::detail::crc32;
  CHECK(crc32(reinterpret_cast<const uint8_t*>("123456789"), 9) == 0xCBF43926u);
  check_codec();
  check_journal_basics();
  check_journal_space();
  check_journal_corruption();
  check_journal_interruptions();
  check_newest_page_bitflips();
  check_stop_headroom();
  check_last_counter_for_stop();
  check_power_cuts();
  puts("OK");
  return 0;
}
