#pragma once

// Dependency-free protocol vocabulary shared by the queue, runtime and gateway.
// Serial buffers are bounded; the radio and journal have no transport dependency.

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace ha_x2d {
constexpr size_t MAX_LINE_BYTES = 512;
constexpr uint8_t MAX_SHUTTERS = 16;

// Whole lines enter atomically; the transport drains only the bytes it can
// currently write. A slow host must not turn a write into a wait on STOP's path.
class OutputBuffer {
 public:
  static constexpr size_t CAPACITY = 4096;
  bool append(const char* data, size_t length) {
    if (!data || !length || length > CAPACITY - used_) return false;
    const size_t tail = (head_ + used_) % CAPACITY;
    const size_t first = length < CAPACITY - tail ? length : CAPACITY - tail;
    memcpy(bytes_ + tail, data, first);
    memcpy(bytes_, data + first, length - first);
    used_ += length;
    return true;
  }
  const char* data() const { return bytes_ + head_; }
  size_t contiguous() const {
    return used_ < CAPACITY - head_ ? used_ : CAPACITY - head_;
  }
  void consume(size_t length) {
    if (length > used_) length = used_;
    head_ = (head_ + length) % CAPACITY;
    used_ -= length;
  }
  void clear() { head_ = used_ = 0; }
  size_t size() const { return used_; }
 private:
  char bytes_[CAPACITY]{};
  size_t head_ = 0, used_ = 0;
};

class LineFramer {
 public:
  explicit LineFramer(size_t limit = MAX_LINE_BYTES)
      : limit_(limit > 1 && limit <= MAX_LINE_BYTES ? limit : MAX_LINE_BYTES) {}
  enum class Event { none, line, too_long };
  Event feed(char byte) {
    if (byte == '\n') {
      const bool overflow = dropping_;
      if (used_ && data_[used_ - 1] == '\r') --used_;
      length = used_; used_ = 0; dropping_ = false;
      return overflow ? Event::too_long : Event::line;
    }
    if (!dropping_) {
      if (used_ < limit_ - 1) data_[used_++] = byte;
      else dropping_ = true;
    }
    return Event::none;
  }
  void reset() { used_ = length = 0; dropping_ = false; }
  const char* data() const { return data_; }
  size_t length = 0;
 private:
  char data_[MAX_LINE_BYTES - 1]{};
  const size_t limit_;
  size_t used_ = 0;
  bool dropping_ = false;
};

enum class Action : uint8_t { none, open, close, stop };

// Uppercase hex16, the shape of device ids and journal generations.
inline bool hex16(const char* value) {
  if (!value || strlen(value) != 16) return false;
  for (size_t i = 0; i < 16; ++i)
    if (!((value[i] >= '0' && value[i] <= '9') || (value[i] >= 'A' && value[i] <= 'F'))) return false;
  return true;
}
}  // namespace ha_x2d
