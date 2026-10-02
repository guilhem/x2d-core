#pragma once

// Strict v2 request decoder. The only ArduinoJson use outside the gateway.
// Shared constants and request types live in types.h.

#include <ArduinoJson.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "types.h"

// The strict decoder and the gateway output are qualified against this release only.
static_assert(ARDUINOJSON_VERSION_MAJOR == 7 && ARDUINOJSON_VERSION_MINOR == 4 &&
                  ARDUINOJSON_VERSION_REVISION == 3,
              "x2d-core requires ArduinoJson 7.4.3");

namespace ha_x2d {

// ArduinoJson permits bare/single-quoted keys and coalesces duplicate keys.
// Requests contain only two object levels and integer/string values; compare
// colon counts to parsed sizes to reject duplicates at both trust boundaries.
inline bool strict_shape(const char* bytes, size_t length, size_t top, size_t args) {
  size_t colons[3]{};
  int depth = 0;
  bool quoted = false, escaped = false, started = false, finished = false;
  for (size_t i = 0; i < length; ++i) {
    const unsigned char c = bytes[i];
    if (quoted) {
      if (c < 0x20) return false;
      if (escaped) escaped = false;
      else if (c == '\\') escaped = true;
      else if (c == '"') quoted = false;
      continue;
    }
    if (c == ' ' || c == '\t' || c == '\r') continue;
    if (finished) return false;
    if (c == '"') { if (!depth) return false; quoted = true; }
    else if (c == '{') {
      if (!depth) { if (started) return false; started = true; }
      if (++depth > 2) return false;
    } else if (c == '}') {
      if (--depth < 0) return false;
      if (!depth) finished = true;
    } else if (c == ':') {
      if (!depth) return false;
      size_t j = i;
      while (j && (bytes[j-1] == ' ' || bytes[j-1] == '\t' || bytes[j-1] == '\r')) --j;
      if (!j || bytes[j-1] != '"') return false;
      ++colons[depth];
    } else if (c >= '0' && c <= '9') {
      if (!depth) return false;
      if (c == '0' && i + 1 < length && bytes[i+1] >= '0' && bytes[i+1] <= '9' &&
          (i == 0 || bytes[i-1] < '0' || bytes[i-1] > '9')) return false;
    } else if (c != ',' && c != '-') return false;
  }
  return finished && !quoted && colons[1] == top && colons[2] == args;
}

inline Request decode(const char* bytes, size_t length) {
  Request r;
  if (length >= MAX_LINE_BYTES) { r.error = "line_too_long"; return r; }
  JsonDocument doc;
  if (deserializeJson(doc, bytes, length, DeserializationOption::NestingLimit(2)) ||
      !doc.is<JsonObject>()) return r;
  const JsonObject obj = doc.as<JsonObject>();
  const JsonObject args = obj["args"].as<JsonObject>();
  if (!strict_shape(bytes, length, obj.size(), args.size())) return r;
  if (obj["id"].is<int32_t>() && obj["id"].as<int32_t>() > 0) {
    r.has_id = true; r.id = obj["id"].as<uint32_t>();
  }
  if (!r.has_id || !obj["v"].is<int>() || obj["v"].as<int>() != PROTOCOL_VERSION ||
      !obj["op"].is<const char*>()) return r;
  const char* op = obj["op"];
  if (!strcmp(op, "hello")) {
    if (obj.size() != 3) return r;
    r.op = Operation::hello; r.error = nullptr; return r;
  }
  if (obj.size() != 5 || !obj["session"].is<const char*>() ||
      !hex16(obj["session"]) || !obj["args"].is<JsonObject>()) return r;
  strcpy(r.session, obj["session"]);
  if (!strcmp(op, "status") || !strcmp(op, "shutters")) {
    if (args.size()) return r;
    r.op = !strcmp(op, "status") ? Operation::status : Operation::shutters;
  } else {
    if (!strcmp(op, "provision")) r.op = Operation::provision;
    else if (!strcmp(op, "pair")) r.op = Operation::pair;
    else if (!strcmp(op, "confirm")) r.op = Operation::confirm;
    else if (!strcmp(op, "command")) r.op = Operation::command;
    else { r.error = "unsupported_operation"; return r; }
    if (args.size() != (r.op == Operation::command ? 2u : 1u) ||
        !args["shutter_id"].is<int>() || args["shutter_id"].as<int>() < 1 ||
        args["shutter_id"].as<int>() > MAX_SHUTTERS) { r.op = Operation::none; return r; }
    r.shutter_id = args["shutter_id"];
    if (r.op == Operation::command) {
      if (!args["action"].is<const char*>()) { r.op = Operation::none; return r; }
      const char* action = args["action"];
      if (!strcmp(action, "open")) r.action = Action::open;
      else if (!strcmp(action, "close")) r.action = Action::close;
      else if (!strcmp(action, "stop")) r.action = Action::stop;
      else { r.op = Operation::none; return r; }
    }
  }
  r.error = nullptr;
  return r;
}
}  // namespace ha_x2d
