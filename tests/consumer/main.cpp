#include <x2d/radio_codec.h>

static_assert(__cplusplus >= 201703L, "x2d::core must provide C++17");

int main() {
  // Offline example only: these inputs do not authorize physical transmission.
  x2d::radio::Body body;
  x2d::radio::ParsedBody parsed;
  x2d::radio::Waveform waveform;
  if (!x2d::radio::make_command_body(0x123456, x2d::Action::stop, 42, &body) ||
      !x2d::radio::parse_body(body.bytes, body.length, &parsed) ||
      parsed.identity != 0x123456 || parsed.counter != 42 || parsed.action != 0x04 ||
      !x2d::radio::encode_burst(body, 1, &waveform) ||
      waveform.copies() != 1 || waveform.frame_end(0) != waveform.chips()) return 1;
  return 0;
}
