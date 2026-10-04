// Host checks for the radio codec and header portability. No hardware.
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

#include <x2d/journal.h>
#include <x2d/radio_codec.h>

// Codec, journal, queue and runtime are plain C++17, with no JSON dependency.
#ifdef ARDUINOJSON_VERSION
#error "radio/journal/runtime headers must not include ArduinoJson"
#endif

using namespace x2d;

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

// Lifecycle journal coverage is in check_journal.cpp, including v1 reset
// fixtures, power cuts, historical identity exclusions and STOP headroom.
int main() {
  CHECK(journal::detail::crc32(reinterpret_cast<const uint8_t*>("123456789"), 9) == 0xCBF43926u);
  check_codec();
  puts("radio codec: OK");
  return 0;
}
