#pragma once

// X2D-compatible radio codec: 12-byte body, rolling word, biphase-mark chips.
// Pure computation: no radio, USB, flash or counter state, and no dependency on
// protocol.h. The caller owns identity, action byte and counter, and emits only
// what it has authorized from captures.
//
// SPDX-License-Identifier: Apache-2.0
// rolling_transform() is adapted from x3d_enc_msg_id/x3d_dec_msg_id:
// Copyright (c) 2023 Sven Fabricius, mr-sven/x3d-rfm-esp32, x3d-lib/x3d.c
// https://github.com/mr-sven/x3d-rfm-esp32/blob/
// 92d9927462b93a2574986aa9e5c5b63dc5997ef4/x3d-lib/x3d.c
// Modifications: C++ translation of research/verify_public_samples.py (itself
// a Python translation); the transform never increments a counter. License
// text: research/LICENSE-APACHE.
//
// Body layout, observed in public and private captures (additive checksum):
//   [0..2]  identity in air order (big-endian 24 bits)
//   [3..6]  01 05 98 22 (fixed in every capture; meaning not established)
//   [7]     action byte, supplied by the caller
//   [8..9]  rolling word, little-endian
//   [10..11] -(sum of bytes 0..9) mod 65536, big-endian
// Burst layout, compared offline with receiver-side OOK captures:
//   0^8, then per copy: 1^6 0, data bits LSB first with a 0 stuffed after
//   every five 1s, 1^8 0. Every bit is two biphase-mark chips.
// NOT validated: the transmit waveform itself. Chip polarity (receiver
// captures read carrier-off as 1), chip duration, the number of preamble
// zeros (captures show 8), and the tail after the last copy are inferred from
// a receiver, not from a transmitter.

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace ha_x2d {
namespace radio {

constexpr size_t BODY_BYTES = 12;
constexpr size_t MAX_BODY_BYTES = 15;  // largest strictly observed body
constexpr uint32_t MAX_IDENTITY = 0xFFFFFF;
constexpr uint8_t BODY_FIXED[4] = {0x01, 0x05, 0x98, 0x22};
constexpr size_t PREAMBLE_ZERO_BITS = 8;
constexpr uint8_t MAX_COPIES = 32;  // captures show 22-25 copies per press
// Calibration knob: captures measure about 208.5 us per chip.
constexpr uint32_t NOMINAL_CHIP_US = 208;
// Frame encoding also accepts the observed extended bodies. Their field
// builders are separate: a valid length does not establish enrollment meaning.
constexpr size_t MAX_COPY_BITS = 7 + (MAX_BODY_BYTES * 8 + MAX_BODY_BYTES * 8 / 5) + 9;
constexpr size_t MAX_CHIPS =
    (2 * (PREAMBLE_ZERO_BITS + MAX_COPIES * MAX_COPY_BITS) + 7) & ~size_t{7};
static_assert(MAX_CHIPS >= 2 * (PREAMBLE_ZERO_BITS + MAX_COPIES * MAX_COPY_BITS),
              "chip buffer too small");
static_assert(MAX_CHIPS % 8 == 0, "packed chips use whole bytes");

struct Body {
  uint8_t bytes[MAX_BODY_BYTES]{};
  uint8_t length = BODY_BYTES;
};

inline uint16_t rolling_transform(uint16_t value, uint32_t identity,
                                  bool inverse) {
  static const uint8_t SBOX[16] = {1, 0, 12, 8, 10, 9, 14, 7,
                                   3, 5, 4,  11, 2, 15, 6, 13};
  const uint32_t key = (identity & 0xFF00) |
                       ((identity & 0xFF) ^ ((identity >> 16) & 0xFF));
  uint32_t state = value;
  for (int step = 0; step < 32; ++step) {
    const int shift = (inverse ? 31 - step : step) % 13;
    if (inverse) state ^= key;
    state = (state & ~(0xFu << shift)) |
            (uint32_t{SBOX[(state >> shift) & 0xF]} << shift);
    if (!inverse) state ^= key;
  }
  return static_cast<uint16_t>(state);
}

inline uint16_t rolling_encode(uint16_t counter, uint32_t identity) {
  return rolling_transform(counter, identity, false);
}

inline uint16_t rolling_decode(uint16_t rolling, uint32_t identity) {
  return rolling_transform(rolling, identity, true);
}

inline uint16_t body_checksum(const uint8_t* bytes, size_t length = BODY_BYTES) {
  if (!bytes || length < 2 || length > MAX_BODY_BYTES) return 0;
  uint16_t sum = 0;
  for (size_t i = 0; i < length - 2; ++i)
    sum = static_cast<uint16_t>(sum + bytes[i]);
  return static_cast<uint16_t>(0u - sum);
}

// One logical command: `counter` is the clear counter the journal reserved.
// Every radio copy of the command shares this body.
inline bool make_body(uint32_t identity, uint8_t action, uint16_t counter,
                      Body* out) {
  if (!out || identity > MAX_IDENTITY) return false;
  out->length = BODY_BYTES;
  uint8_t* b = out->bytes;
  b[0] = static_cast<uint8_t>(identity >> 16);
  b[1] = static_cast<uint8_t>(identity >> 8);
  b[2] = static_cast<uint8_t>(identity);
  memcpy(b + 3, BODY_FIXED, sizeof(BODY_FIXED));
  b[7] = action;
  const uint16_t rolling = rolling_encode(counter, identity);
  b[8] = static_cast<uint8_t>(rolling);
  b[9] = static_cast<uint8_t>(rolling >> 8);
  const uint16_t checksum = body_checksum(b);
  b[10] = static_cast<uint8_t>(checksum >> 8);
  b[11] = static_cast<uint8_t>(checksum);
  return true;
}

// Genuine B simultaneous UP+DOWN in N, two distinct command bodies with
// consecutive counters. The caller reserves both before RF, and supplies
// only its own new identity. This observed format is still motor-unqualified.
inline bool make_enrollment_body(uint32_t identity, uint16_t counter,
                                 uint8_t phase, Body* out) {
  if (phase > 1 || !make_body(identity, phase ? 0x20 : 0x02, counter, out))
    return false;
  if (!phase) out->bytes[4] = 0x85;
  else {
    out->length = 13;
    out->bytes[10] = out->bytes[9];
    out->bytes[9] = out->bytes[8];
    out->bytes[8] = 0x07;  // observed hold-gesture extension; semantics unknown
  }
  const uint16_t checksum = body_checksum(out->bytes, out->length);
  out->bytes[out->length - 2] = static_cast<uint8_t>(checksum >> 8);
  out->bytes[out->length - 1] = static_cast<uint8_t>(checksum);
  return true;
}

struct ParsedBody {
  uint32_t identity = 0;
  uint8_t action = 0;
  uint16_t rolling_word = 0;
  uint16_t counter = 0;  // inverse transform of rolling_word
};

// Accepts only 12 bytes with the fixed bytes and a correct checksum.
inline bool parse_body(const uint8_t* data, size_t length, ParsedBody* out) {
  if (!data || !out || length != BODY_BYTES ||
      memcmp(data + 3, BODY_FIXED, sizeof(BODY_FIXED)) != 0)
    return false;
  if (((data[10] << 8) | data[11]) != body_checksum(data)) return false;
  out->identity =
      (uint32_t{data[0]} << 16) | (uint32_t{data[1]} << 8) | data[2];
  out->action = data[7];
  out->rolling_word = static_cast<uint16_t>(data[8] | (data[9] << 8));
  out->counter = rolling_decode(out->rolling_word, out->identity);
  return true;
}

// Chip vector, one bit per chip, MSB first, unused bits zero. Chip 1 is
// carrier on; the burst starts from carrier off, so chip 0 is 1. The caller
// clocks chips at the calibrated chip duration and switches the carrier off
// after chips(). frame_end(i) is the chip index just after copy i, so a STOP
// can cut the burst at a complete-frame boundary. About 1.1 KB: keep static.
class Waveform {
 public:
  size_t chips() const { return count_; }
  bool chip(size_t index) const {
    return index < count_ && ((bits_[index >> 3] >> (7 - (index & 7))) & 1);
  }
  const uint8_t* packed() const { return bits_; }
  size_t packed_bytes() const { return (count_ + 7) / 8; }
  uint8_t copies() const { return copies_; }
  size_t frame_end(uint8_t copy) const { return copy < copies_ ? ends_[copy] : 0; }

  void clear() {
    memset(bits_, 0, sizeof(bits_));
    count_ = 0;
    copies_ = 0;
  }
  bool push(bool value) {
    if (count_ >= MAX_CHIPS) return false;
    if (value) bits_[count_ >> 3] |= static_cast<uint8_t>(0x80 >> (count_ & 7));
    ++count_;
    return true;
  }
  bool mark_frame_end() {
    if (copies_ >= MAX_COPIES) return false;
    ends_[copies_++] = static_cast<uint16_t>(count_);
    return true;
  }

 private:
  uint8_t bits_[MAX_CHIPS / 8]{};
  uint16_t ends_[MAX_COPIES]{};
  size_t count_ = 0;
  uint8_t copies_ = 0;
};

namespace detail {

// Biphase mark: the level toggles at every bit boundary and again mid-bit for
// a 1. The level before the burst is 0 (carrier off).
struct ChipWriter {
  Waveform& wave;
  bool level = false;
  void cell(bool value) {
    level = !level;
    wave.push(level);
    if (value) level = !level;
    wave.push(level);
  }
  void cells(bool value, size_t count) {
    while (count--) cell(value);
  }
};

}  // namespace detail

// `copies` identical frames behind one preamble. False for 0 or more than
// MAX_COPIES copies.
inline bool encode_burst(const Body& body, uint8_t copies, Waveform* out) {
  if (!out || copies == 0 || copies > MAX_COPIES ||
      body.length < 2 || body.length > MAX_BODY_BYTES) return false;
  out->clear();
  detail::ChipWriter writer{*out};
  writer.cells(false, PREAMBLE_ZERO_BITS);
  for (uint8_t copy = 0; copy < copies; ++copy) {
    writer.cells(true, 6);
    writer.cell(false);
    unsigned ones = 0;
    for (size_t i = 0; i < body.length; ++i) {
      for (int j = 0; j < 8; ++j) {
        const bool value = (body.bytes[i] >> j) & 1;
        writer.cell(value);
        ones = value ? ones + 1 : 0;
        if (ones == 5) {
          writer.cell(false);
          ones = 0;
        }
      }
    }
    writer.cells(true, 8);
    writer.cell(false);
    out->mark_frame_end();
  }
  return true;
}

// make_body + encode_burst. `action_byte` is not interpreted here.
inline bool encode(uint32_t identity, uint8_t action_byte, uint16_t counter,
                   uint8_t copies, Waveform* out) {
  Body body;
  return make_body(identity, action_byte, counter, &body) &&
         encode_burst(body, copies, out);
}

// Run lengths (1 or 2 chips) in order; the first run has the level of chip 0.
// Returns the run count, or 0 when `capacity` is too small.
inline size_t to_pulses(const Waveform& wave, uint8_t* out, size_t capacity) {
  size_t runs = 0;
  for (size_t i = 0; i < wave.chips();) {
    size_t length = 1;
    while (i + length < wave.chips() &&
           wave.chip(i + length) == wave.chip(i))
      ++length;
    if (runs >= capacity) return 0;
    out[runs++] = static_cast<uint8_t>(length);
    i += length;
  }
  return runs;
}

}  // namespace radio
}  // namespace ha_x2d
