#pragma once

#include <stddef.h>
#include <stdint.h>

namespace x2d {
constexpr uint8_t MAX_SHUTTERS = 16;
enum class Action : uint8_t { none, open, close, stop };
}  // namespace x2d
