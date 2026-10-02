#pragma once

// Portable JSONL v2 gateway: decoding, replies, events, handshake, duplicate
// cache and bounded output, extracted from the RP2040 sketch with its wire
// behavior unchanged. No Arduino, Serial, USB or blocking I/O: the host adapter
// moves bytes.
//
// Adapter loop (single thread, nothing here blocks):
//   gateway.begin();                         // once, after journal.open() and
//                                            // the radio gate decision
//   each pass:  connected ? gateway.feed(...) (one line at most) :
//                           gateway.disconnected();
//               move output_data()/output_contiguous() out, then consume_output(n)
//               gateway.tick();              // always, also while disconnected
//               radio service (hardware poll) as the backend requires
//   if gateway.failed(): stop reading and close/ignore the link; tick() keeps
//   cancelling work. disconnected() recovers the link.
//
// ArduinoJson (7.4.3) is used only by this header and protocol.h.

#include <ArduinoJson.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "journal.h"
#include "protocol.h"
#include "radio_codec.h"
#include "radio_runtime.h"
#include "types.h"

namespace ha_x2d {

static_assert(journal::SLOTS == MAX_SHUTTERS, "slot contract mismatch");

// Hooks (firmware/host owned, must outlive the gateway, must not call back into
// it). All of them run on the adapter thread:
//   uint32_t now_ms();                 monotonic, wraps. Radio::start_burst's
//                                      started_ms uses this same basis.
//   bool tx_available();               DYNAMIC gate: the radio is configured and
//                                      its profile verified. Used by hello
//                                      (command capability), status (tx_enabled)
//                                      and command/pair admission.
//   bool enrollment_allowed();         static policy: advertises and allows
//                                      provision/pair/confirm.
//   RadioStatus probe_radio();         status; may be read during a TX frame.
//   const char* firmware();            static string for hello.
//   bool profile(const TxJob&, radio::TxProfile&);
//                                      the qualified burst: copies and chip
//                                      calibration (enrollment second start).
//                                      The gateway has already checked both
//                                      gates; the hook never decides policy.
//   bool authorize_provision(const journal::Journal&, uint8_t shutter_id);
//   bool authorize_pair(const journal::Journal&, uint8_t shutter_id);
//                                      supervised-trial gates (slot, expected
//                                      counter). false -> profile_unverified.
//   bool new_controller(const journal::Journal&, journal::NewController&);
//                                      fresh identity/counter/generation; false
//                                      -> profile_unverified. Entropy stays here.
// One shared implementation owns the protocol: the gateway itself builds the
// canonical command bodies (open 0x81, close 0x82, stop 0x04) and the two
// enrollment bodies from the journal reservation, so adapters never supply
// action bytes. It also owns the runtime's report hook, so no adapter can bypass
// connection routing of completion events.
template <class Radio, class Hooks>
class Gateway {
  struct RuntimeHooks_ {
    Gateway* owner;
    bool profile(const TxJob& job, radio::TxProfile& profile) {
      Hooks& hooks = owner->hooks_;
      return hooks.tx_available() && (!job.enrollment || hooks.enrollment_allowed()) &&
             hooks.profile(job, profile);
    }
    bool build(const TxJob& job, const journal::Reservation& reserved, uint8_t phase,
               radio::Body& body) {
      if (job.enrollment)
        return radio::make_enrollment_body(reserved.identity, reserved.counter, phase, &body);
      const uint8_t action = job.action == Action::open ? 0x81
                           : job.action == Action::close ? 0x82 : 0x04;
      return radio::make_body(reserved.identity, action, reserved.counter, &body);
    }
    void report(const radio::TxEvent& event) { owner->on_event(event); }
  };

 public:
  using Runtime = radio::RadioRuntime<Radio, RuntimeHooks_>;

  // device_id and session must be uppercase hex16 (copied). Anything else makes
  // the gateway permanently failed() and silent.
  Gateway(journal::Journal& journal, Radio& radio, Hooks& hooks,
          const char* device_id, const char* session)
      : journal_(journal), hooks_(hooks), adapter_{this},
        runtime_(journal, radio, adapter_) {
    invalid_ = !hex16(device_id) || !hex16(session);
    if (!invalid_) {
      memcpy(device_id_, device_id, sizeof(device_id_));
      memcpy(session_, session, sizeof(session_));
    }
  }
  Gateway(const Gateway&) = delete;  // the runtime hooks point back here
  Gateway& operator=(const Gateway&) = delete;

  // Applies the boot-time transmit/enrollment gates to the radio runtime.
  void begin() { runtime_.set_enabled(hooks_.tx_available(), hooks_.enrollment_allowed()); }

  // Consumes bytes up to and including the first complete line (also an
  // oversized one), answers it, and returns the count consumed so the caller can
  // tick/flush before offering the rest. Returns 0 while failed().
  size_t feed(const char* data, size_t length) {
    if (failed()) return 0;
    for (size_t used = 0; used < length;) {
      const auto event = framer_.feed(data[used++]);
      if (event == LineFramer::Event::none) continue;
      if (event == LineFramer::Event::too_long) {
        Request request;
        request.error = "line_too_long";
        respond(request);
      } else respond(decode(framer_.data(), framer_.length));
      return used;
    }
    return length;
  }

  // Runtime progress. A failed link keeps cancelling work; ACKs are queued by
  // feed() before the first tick can produce a terminal event.
  void tick() {
    if (failed()) runtime_.disconnect();
    runtime_.tick(hooks_.now_ms());
  }

  // Link loss; safe to repeat. Resets only link state. The runtime fault, the
  // active burst (cancelled at its next full frame, STOP included), journal,
  // boot session, event sequence and connection counter are kept, and the
  // journal is never reopened. Old events stay suppressed: handshaken_ is
  // cleared first, and a later hello gets a new connection id.
  void disconnected() {
    if (handshaken_) {
      handshaken_ = false;
      runtime_.disconnect();
    }
    framer_.reset();
    output_.clear();
    output_failed_ = false;
    highest_id_ = 0;
    for (CachedReply& cached : replies_) cached.length = 0;
  }

  // Output overflow, or a response that did not serialize, fails closed: no more
  // input is accepted until disconnected(). (The sketch dropped an unserializable
  // response silently and kept serving.)
  bool failed() const { return invalid_ || output_failed_; }
  bool handshaken() const { return handshaken_; }
  const Runtime& runtime() const { return runtime_; }

  const char* output_data() const { return output_.data(); }
  size_t output_contiguous() const { return output_.contiguous(); }
  size_t output_size() const { return output_.size(); }
  void consume_output(size_t length) { output_.consume(length); }

 private:
  struct CachedReply {
    Request request;
    size_t length = 0;
    char bytes[MAX_LINE_BYTES];
  };

  void queue(const char* bytes, size_t length) {
    if (output_failed_ || !output_.append(bytes, length)) output_failed_ = true;
  }

  void send(JsonDocument& doc, const Request* request = nullptr) {
    const size_t length = serializeJson(doc, line_, sizeof(line_) - 1);
    if (!length || length != measureJson(doc)) {
      output_failed_ = true;
      return;
    }
    line_[length] = '\n';
    if (request) {
      CachedReply& cached = replies_[reply_index_++ % 16];
      cached.request = *request;
      cached.length = length + 1;
      memcpy(cached.bytes, line_, cached.length);
      highest_id_ = request->id;
    }
    queue(line_, length + 1);
  }

  void on_event(const radio::TxEvent& event) {
    if (admitting_id_ && event.job.request_id == admitting_id_ &&
        event.job.connection_id == connection_id_) {
      admission_error_ = event.error ? event.error : "queue_full";
      return;  // rejected admission has a negative reply, no completion event
    }
    if (!event.job.request_id || !handshaken_ || output_failed_ ||
        event.job.connection_id != connection_id_) return;
    JsonDocument doc;
    doc["v"] = PROTOCOL_VERSION;
    doc["session"] = session_;
    doc["seq"] = ++event_sequence_;
    doc["event"] = "tx_result";
    doc["request_id"] = event.job.request_id;
    doc["shutter_id"] = event.job.shutter_id;
    doc["result"] = event.outcome;
    doc["completed_copies"] = event.completed_copies;
    if (event.error) doc["error"] = event.error;
    send(doc);
  }

  void error(const Request& request, const char* reason, bool cache = false) {
    JsonDocument doc;
    doc["v"] = PROTOCOL_VERSION;
    if (request.has_id) doc["id"] = request.id;
    else doc["id"] = nullptr;
    doc["ok"] = false;
    doc["error"] = reason;
    send(doc, cache ? &request : nullptr);
  }

  void generation(JsonObject object) const {
    char value[17];
    if (journal_.generation_hex(value)) object["generation"] = value;
    else object["generation"] = nullptr;
  }

  static void shutter_record(JsonObject object, const journal::Shutter& shutter) {
    object["shutter_id"] = shutter.shutter_id;
    object["state"] = shutter.state == journal::SlotState::paired ? "paired" : "pending";
    switch (static_cast<Action>(shutter.last_command)) {
      case Action::open: object["last_command"] = "open"; break;
      case Action::close: object["last_command"] = "close"; break;
      case Action::stop: object["last_command"] = "stop"; break;
      default: object["last_command"] = nullptr;
    }
  }

  void respond(const Request& request) {
    if (request.error) {
      if (handshaken_) error(request, request.error);
      return;
    }
    if (request.op != Operation::hello) {
      if (!handshaken_) return;
      if (strcmp(request.session, session_)) { error(request, "stale_session"); return; }
    }
    for (const CachedReply& cached : replies_) {
      if (!cached.length || cached.request.id != request.id) continue;
      if (cached.request.op != request.op || cached.request.shutter_id != request.shutter_id ||
          cached.request.action != request.action) { error(request, "duplicate_conflict"); return; }
      queue(cached.bytes, cached.length);
      return;
    }
    if (request.id <= highest_id_) { error(request, "stale_request"); return; }

    JsonDocument doc;
    doc["v"] = PROTOCOL_VERSION;
    doc["id"] = request.id;
    doc["ok"] = true;
    JsonObject result = doc["result"].to<JsonObject>();
    if (request.op == Operation::hello) {
      if (!handshaken_) ++connection_id_;
      handshaken_ = true;
      result["product"] = "ha-x2d";
      result["firmware"] = hooks_.firmware();
      result["device_id"] = device_id_;
      result["session"] = session_;
      result["max_line_bytes"] = MAX_LINE_BYTES;
      result["max_shutters"] = MAX_SHUTTERS;
      JsonArray caps = result["capabilities"].to<JsonArray>();
      caps.add("status"); caps.add("shutters");
      if (hooks_.tx_available()) {
        caps.add("command");
        if (hooks_.enrollment_allowed()) {
          caps.add("provision"); caps.add("pair"); caps.add("confirm");
        }
      }
    } else if (request.op == Operation::status) {
      const RadioStatus probe = hooks_.probe_radio();
      result["uptime_ms"] = hooks_.now_ms();
      result["tx_enabled"] = hooks_.tx_available();
      JsonObject r = result["radio"].to<JsonObject>();
      r["detected"] = probe.detected;
      if (probe.detected) {
        r["partnum"] = probe.partnum; r["version"] = probe.version; r["marcstate"] = probe.marcstate;
      } else { r["partnum"] = nullptr; r["version"] = nullptr; r["marcstate"] = nullptr; }
      JsonObject storage = result["storage"].to<JsonObject>();
      switch (journal_.state()) {
        case journal::StorageState::empty: storage["state"] = "empty"; break;
        case journal::StorageState::ready: storage["state"] = "ready"; break;
        case journal::StorageState::full: storage["state"] = "full"; break;
        default: storage["state"] = "corrupt";
      }
      generation(storage);
    } else {
      if (journal_.state() == journal::StorageState::corrupt) {
        error(request, "storage_corrupt", true); return;
      }
      if (request.op == Operation::shutters) {
        generation(result);
        JsonArray slots = result["shutters"].to<JsonArray>();
        for (uint8_t i = 1; i <= MAX_SHUTTERS; ++i) {
          const auto shutter = journal_.shutter(i);
          if (shutter.state != journal::SlotState::unused)
            shutter_record(slots.add<JsonObject>(), shutter);
        }
      } else if (request.op == Operation::provision &&
                 journal_.shutter(request.shutter_id).state != journal::SlotState::unused) {
        generation(result);
        shutter_record(result, journal_.shutter(request.shutter_id));
      } else if (request.op == Operation::command || request.op == Operation::pair) {
        if (!hooks_.tx_available() ||
            (request.op == Operation::pair && !hooks_.enrollment_allowed())) {
          error(request, "profile_unverified", true); return;
        }
        if (request.op == Operation::pair) {
          if (busy()) { error(request, "maintenance_pending", true); return; }
          // Private trial: authorize_pair owns the one-attempt counter rule.
          if (!hooks_.authorize_pair(journal_, request.shutter_id)) {
            error(request, "profile_unverified", true); return;
          }
        }
        TxJob job;
        job.request_id = request.id;
        job.shutter_id = request.shutter_id;
        job.action = request.action;
        job.enrollment = request.op == Operation::pair;
        job.connection_id = connection_id_;
        admitting_id_ = request.id;
        admission_error_ = nullptr;
        const bool accepted = runtime_.submit(job, hooks_.now_ms());
        admitting_id_ = 0;
        if (!accepted) { error(request, admission_error_ ? admission_error_ : "queue_full", true); return; }
        result["accepted"] = true;
        result["shutter_id"] = request.shutter_id;
      } else if (request.op == Operation::confirm) {
        if (!hooks_.enrollment_allowed()) { error(request, "profile_unverified", true); return; }
        if (busy()) { error(request, "maintenance_pending", true); return; }
        const auto status = journal_.confirm(request.shutter_id);
        if (status != journal::Status::ok) {
          error(request, journal::protocol_error(status), true); return;
        }
        generation(result);
        shutter_record(result, journal_.shutter(request.shutter_id));
      } else if (request.op == Operation::provision && hooks_.enrollment_allowed()) {
        if (!hooks_.authorize_provision(journal_, request.shutter_id)) {
          error(request, "profile_unverified", true); return;
        }
        if (busy()) { error(request, "maintenance_pending", true); return; }
        journal::NewController fresh;
        if (!hooks_.new_controller(journal_, fresh)) {
          error(request, "profile_unverified", true); return;
        }
        const auto status = journal_.provision(request.shutter_id, fresh);
        if (status != journal::Status::ok) {
          error(request, journal::protocol_error(status), true); return;
        }
        generation(result);
        shutter_record(result, journal_.shutter(request.shutter_id));
      } else {
        // Without enrollment the gateway neither allocates an identity nor emits RF.
        error(request, "profile_unverified", true); return;
      }
    }
    send(doc, &request);
  }

  bool busy() const { return runtime_.active() || runtime_.pending(); }

  journal::Journal& journal_;
  Hooks& hooks_;
  RuntimeHooks_ adapter_;
  Runtime runtime_;
  LineFramer framer_;
  OutputBuffer output_;
  CachedReply replies_[16];
  char line_[MAX_LINE_BYTES];
  char device_id_[17]{}, session_[17]{};
  const char* admission_error_ = nullptr;
  size_t reply_index_ = 0;
  uint32_t highest_id_ = 0, event_sequence_ = 0, admitting_id_ = 0, connection_id_ = 0;
  bool handshaken_ = false, output_failed_ = false, invalid_ = false;
};

}  // namespace ha_x2d
