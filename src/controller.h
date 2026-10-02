#pragma once

// Command admission, supervised association and RF authorization over the
// journal and RadioRuntime. One loop owner; no I/O, session or restart policy:
// adapters decide what a confirmed association, pause or disconnect means.

#include "radio_runtime.h"

namespace ha_x2d {

// This is a build-time, supervised trial authorization, not a qualified motor
// profile. An ordinary build has no authorization to generate RF identities.
struct PairingAuthorization {
  uint8_t slot = 0;
  uint8_t identity_suffix = 0;
  uint32_t expected_next_counter = 0;
};

// Radio is the RadioRuntime radio. Observer, called from the owning loop:
//   uint32_t random_u32();                            entropy for new identities
//   void status(const char* message, uint8_t slot);   diagnostic, slot 0 = global
//   void tx_result(const radio::TxEvent&);            terminal TX outcome
// A confirmed association reports status("paired", slot) and nothing else.
template<class Radio, class Observer> class Controller {
 public:
  static constexpr uint8_t MULTIPLE_PENDING = 255;

  Controller(journal::Journal &journal, Radio &radio, Observer &observer)
      : journal_(journal), radio_(radio), observer_(observer), runtime_(journal, radio, *this) {}

  bool begin(bool transmit, bool enrollment, uint32_t chip_ns, PairingAuthorization authorization = {}) {
    transmit_ = transmit;
    enrollment_ = transmit && enrollment;
    chip_ns_ = chip_ns;
    authorization_ = authorization;
    valid_ = journal_.open() != journal::StorageState::corrupt;
    runtime_.set_enabled(valid_ && transmit_ && radio_.available(), enrollment_);
    status(!valid_ ? "storage_corrupt" : !transmit_ ? "transmission_disabled" :
           !radio_.available() ? "radio_unavailable" : pending_slot() ? "association_pending" : "ready");
    return valid_;
  }

  void tick(uint32_t now) {
    if (valid_ && (!paused_ || runtime_.active())) runtime_.tick(now);
  }
  bool paired(uint8_t slot) const {
    return valid_ && journal_.shutter(slot).state == journal::SlotState::paired;
  }
  bool active() const { return runtime_.active(); }
  bool valid() const { return valid_; }
  // The only pending slot: 0 = none, MULTIPLE_PENDING = ambiguous.
  uint8_t pending_slot() const {
    uint8_t result = 0;
    for (uint8_t slot = 1; slot <= MAX_SHUTTERS; ++slot) {
      if (journal_.shutter(slot).state != journal::SlotState::pending) continue;
      if (result) return MULTIPLE_PENDING;
      result = slot;
    }
    return result;
  }

  bool command(uint8_t slot, Action action, uint32_t now) {
    if (!admit()) return false;
    if (!transmit_ || !radio_.available()) return refuse("transmission_disabled", slot);
    return runtime_.submit(job(slot, action, false), now);
  }

  bool associate(uint32_t now) {
    if (!admit_enrollment()) return false;
    if (runtime_.active() || runtime_.pending()) return refuse("radio_busy");
    if (journal_.maintenance_due()) return refuse("storage_maintenance");
    uint8_t slot = pending_slot();
    if (slot == MULTIPLE_PENDING) return refuse("multiple_pending_associations");
    if (!slot) {
      for (uint8_t i = 1; i <= MAX_SHUTTERS; ++i)
        if (journal_.shutter(i).state == journal::SlotState::unused) { slot = i; break; }
    }
    if (!slot) return refuse("inventory_full");
    if (slot != authorization_.slot) return refuse("association_profile_unqualified", slot);
    if (journal_.shutter(slot).state == journal::SlotState::unused) {
      if (authorization_.expected_next_counter != 0) return refuse("association_counter_mismatch", slot);
      journal::NewController fresh;
      for (unsigned attempt = 0; attempt < 64; ++attempt) {
        fresh.identity = ((observer_.random_u32() & 0xFFFF) << 8) | authorization_.identity_suffix;
        if ((fresh.identity & 0xFFFF00) && !journal_.find_identity(fresh.identity)) break;
        fresh.identity = 0;
      }
      if (!fresh.identity) return refuse("identity_generation_failed", slot);
      fresh.generation = (uint64_t{observer_.random_u32()} << 32) | observer_.random_u32();
      if (!fresh.generation) return refuse("identity_generation_failed", slot);
      fresh.first_counter = 0;
      if (!stored(journal_.provision(slot, fresh), slot)) return false;
    }
    uint32_t next;
    if (!journal_.next_counter(slot, &next) || next != authorization_.expected_next_counter)
      return refuse("association_counter_mismatch", slot);
    if (!runtime_.submit(job(slot, Action::none, true), now)) return false;
    status("associating", slot);
    return true;
  }

  // The human's assertion that the motor responded; RF never proves it. Reports
  // "paired" once the journal confirms the slot. Restarting is the caller's call.
  bool confirm() {
    if (!admit_enrollment()) return false;
    if (runtime_.active() || runtime_.pending()) return refuse("radio_busy");
    if (journal_.maintenance_due()) return refuse("storage_maintenance");
    const uint8_t slot = pending_slot();
    if (!slot) return refuse("no_pending_association");
    if (slot == MULTIPLE_PENDING) return refuse("multiple_pending_associations");
    if (slot != authorization_.slot) return refuse("association_profile_unqualified", slot);
    uint32_t next;
    // Reservations prove that an attempt was prepared, never motor reception.
    if (!journal_.next_counter(slot, &next) || next < 2) return refuse("no_association_attempt", slot);
    if (!stored(journal_.confirm(slot), slot)) return false;
    status("paired", slot);
    return true;
  }

  // Refuse new work, drop queued work and cancel the active burst at a frame
  // boundary until resume(). Reservations stay consumed; nothing is replayed.
  // reason is the diagnostic for refused requests and must be a static string.
  void pause(const char *reason = "paused") { paused_ = true; pause_reason_ = reason; runtime_.disconnect(); }
  void resume() { paused_ = false; }
  // Lost session: the same cancellation without refusing later requests.
  void disconnect() { runtime_.disconnect(); }

  // RadioRuntime hooks. It checks these again before reserving the counters.
  bool profile(const TxJob &job, radio::TxProfile &profile) {
    if (!valid_ || paused_ || !transmit_ || !radio_.available()) return false;
    if (job.enrollment) {
      uint32_t next;
      if (!enrollment_ || job.shutter_id != authorization_.slot ||
          !journal_.next_counter(job.shutter_id, &next) || next != authorization_.expected_next_counter)
        return false;
    }
    profile = {static_cast<uint8_t>(job.enrollment ? 24 : 25), chip_ns_, 2001};
    return true;
  }
  bool build(const TxJob &job, const journal::Reservation &reserved,
             uint8_t phase, radio::Body &body) {
    return job.enrollment
        ? radio::make_enrollment_body(reserved.identity, reserved.counter, phase, &body)
        : radio::make_command_body(reserved.identity, job.action, reserved.counter, &body);
  }
  void report(const radio::TxEvent &event) {
    if (event.storage_status == journal::Status::corrupt ||
        event.storage_status == journal::Status::io_error) valid_ = false;
    observer_.tx_result(event);
    status(event.job.enrollment && !strcmp(event.outcome, "emitted") ? "awaiting_confirmation" :
           event.error ? event.error : event.outcome, event.job.shutter_id);
  }

 private:
  void status(const char *message, uint8_t slot = 0) { observer_.status(message, slot); }
  bool refuse(const char *message, uint8_t slot = 0) { status(message, slot); return false; }
  bool admit() {
    if (!valid_) return refuse("storage_corrupt");
    if (paused_) return refuse(pause_reason_);
    return true;
  }
  bool admit_enrollment() {
    if (!admit()) return false;
    if (!enrollment_) return refuse("association_disabled");
    if (!authorization_.slot) return refuse("association_profile_unqualified");
    if (!radio_.available()) return refuse("radio_unavailable");
    return true;
  }
  bool stored(journal::Status result, uint8_t slot) {
    if (result == journal::Status::ok) return true;
    if (result == journal::Status::corrupt || result == journal::Status::io_error) valid_ = false;
    return refuse(journal::protocol_error(result), slot);
  }
  TxJob job(uint8_t slot, Action action, bool enrollment) {
    if (!++request_id_) ++request_id_;
    TxJob result{};
    result.request_id = request_id_;
    result.shutter_id = slot;
    result.action = action;
    result.enrollment = enrollment;
    return result;
  }
  journal::Journal &journal_;
  Radio &radio_;
  Observer &observer_;
  radio::RadioRuntime<Radio, Controller> runtime_;
  PairingAuthorization authorization_{};
  const char *pause_reason_ = "paused";
  uint32_t chip_ns_ = 208500, request_id_ = 0;
  bool transmit_ = false, enrollment_ = false, valid_ = false, paused_ = false;
};

}  // namespace ha_x2d
