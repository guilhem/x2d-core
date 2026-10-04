#pragma once

// One loop owner for RF admission and the durable shutter lifecycle. Transports
// supply explicit human intent; the controller never restores an emission permit.
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <x2d/radio_runtime.h>

namespace x2d {

// Public candidate for the currently demonstrated frame format. The suffix is
// hardware-unqualified; enabling enrollment does not establish motor support.
struct EnrollmentProfile { uint8_t identity_suffix = 1; };

// Observer supplies random_u32(), status(message, physical_slot), tx_result().
// Callbacks must not reenter; identities and counters never enter diagnostics.
template<class Radio, class Observer> class Controller {
 public:
  static constexpr uint8_t MULTIPLE_PENDING = 255;
  Controller(journal::Journal &journal, Radio &radio, Observer &observer)
      : journal_(journal), radio_(radio), observer_(observer), runtime_(journal, radio, *this) {}

  bool begin(bool transmit, bool enrollment, uint32_t chip_ns, EnrollmentProfile profile = {}) {
    transmit_ = transmit;
    enrollment_ = transmit && enrollment;
    chip_ns_ = chip_ns;
    profile_ = profile;
    const auto state = journal_.open();
    valid_ = state != journal::StorageState::corrupt;
    enable_runtime();
    if (!valid_) status("storage_corrupt");
    else if (state == journal::StorageState::empty) return initialize();
    else if (state == journal::StorageState::legacy) status("initialization_required");
    else if (state == journal::StorageState::initializing) status("initializing");
    else boot_status();
    return valid_;
  }

  bool initialize() {
    if (!admit() || !idle()) return false;
    const auto state = journal_.state();
    if (state != journal::StorageState::empty && state != journal::StorageState::legacy)
      return refuse("already_initialized");
    uint64_t generation = 0;
    for (unsigned i = 0; i < 64 && !generation; ++i)
      generation = (uint64_t{observer_.random_u32()} << 32) | observer_.random_u32();
    if (!generation) return refuse("identity_generation_failed");
    if (!stored(journal_.initialize(generation, static_cast<uint16_t>(observer_.random_u32()),
                                    profile_.identity_suffix), 0)) return false;
    enable_runtime();
    status(journal_.state() == journal::StorageState::initializing ? "initializing" : "ready");
    return true;
  }

  void tick(uint32_t now) {
    if (!valid_) return;
    if (journal_.state() == journal::StorageState::initializing) {
      if (paused_) return;
      if (!stored(journal_.maintain(), 0)) return;
      if (journal_.state() != journal::StorageState::initializing) {
        enable_runtime();
        boot_status();
      }
      return;
    }
    if (!paused_ || runtime_.active()) runtime_.tick(now, !paused_);
  }
  bool paired(uint8_t slot) const {
    return valid_ && journal_.shutter(slot).state == journal::SlotState::paired;
  }
  bool valid() const { return valid_; }
  bool active() const { return runtime_.active(); }
  bool busy() const { return runtime_.active() || runtime_.pending(); }
  uint8_t pending_slot() const {
    uint8_t result = 0;
    for (uint8_t slot = 1; slot <= MAX_SHUTTERS; ++slot) {
      if (!journal_.shutter(slot).has_candidate) continue;
      if (result) return MULTIPLE_PENDING;
      result = slot;
    }
    return result;
  }

  bool command(uint8_t slot, Action action, uint32_t now) {
    if (!admit_ready()) return false;
    if (!transmit_ || !radio_.available()) return refuse("transmission_disabled", slot);
    return runtime_.submit(job(slot, action, false), now);
  }
  bool associate(uint32_t now) {
    if (!admit_enrollment() || !idle() || !space()) return false;
    if (pending_slot()) return refuse("association_pending", pending_slot());
    uint8_t slot = 0;
    for (uint8_t i = 1; i <= MAX_SHUTTERS; ++i)
      if (journal_.shutter(i).state == journal::SlotState::unused ||
          journal_.shutter(i).state == journal::SlotState::retired) { slot = i; break; }
    if (!slot) return refuse("inventory_full");
    if (!stored(journal_.allocate_candidate(slot, false), slot)) return false;
    return attempt(slot, false, now);
  }
  bool replace(uint8_t slot, uint32_t now) {
    if (!admit_enrollment() || !idle(slot) || !space(slot)) return false;
    if (!paired(slot)) return refuse("unknown_shutter", slot);
    if (journal_.shutter(slot).in_service) return refuse("disable_first", slot);
    if (pending_slot()) return refuse("association_pending", pending_slot());
    if (!stored(journal_.allocate_candidate(slot, true), slot)) return false;
    return attempt(slot, false, now);
  }
  bool retry(uint8_t slot, uint32_t now) {
    if (!admit_enrollment() || !idle(slot) || !space(slot)) return false;
    if (!journal_.shutter(slot).has_candidate) return refuse("no_pending_association", slot);
    return attempt(slot, true, now);
  }
  // The user asserts motor response; preparation/emission is never reception.
  bool confirm(uint8_t slot) {
    if (!admit_ready() || !idle(slot) || !space(slot)) return false;
    if (!journal_.shutter(slot).has_candidate) return refuse("no_pending_association", slot);
    if (!stored(journal_.confirm_candidate(slot), slot)) return false;
    permit_request_ = 0;
    status("paired", slot);
    return true;
  }
  bool cancel(uint8_t slot) {
    if (!admit_ready()) return false;
    if (!journal_.shutter(slot).has_candidate) return refuse("no_pending_association", slot);
    if (busy()) { runtime_.cancel_slot(slot); return refuse("radio_busy", slot); }
    if (!space(slot)) return false;
    if (!stored(journal_.cancel_candidate(slot), slot)) return false;
    permit_request_ = 0;
    status("association_cancelled", slot);
    return true;
  }
  bool service(uint8_t slot, bool enabled) {
    if (!admit_ready()) return false;
    if (!paired(slot)) return refuse("unknown_shutter", slot);
    if (busy()) {
      if (!enabled) runtime_.cancel_slot(slot);
      return refuse("radio_busy", slot);
    }
    if (!space(slot)) return false;
    if (enabled && journal_.shutter(slot).has_candidate) return refuse("association_pending", slot);
    if (!stored(journal_.set_service(slot, enabled), slot)) return false;
    status(enabled ? "service_enabled" : "service_disabled", slot);
    return true;
  }
  bool retire(uint8_t slot) {
    if (!admit_ready() || !idle(slot) || !space(slot)) return false;
    const auto shutter = journal_.shutter(slot);
    if (!paired(slot)) return refuse("unknown_shutter", slot);
    if (shutter.in_service) return refuse("disable_first", slot);
    if (shutter.has_candidate) return refuse("association_pending", slot);
    if (!stored(journal_.retire(slot), slot)) return false;
    status("retired", slot);
    return true;
  }

  void pause(const char *reason = "paused") {
    paused_ = true;
    pause_reason_ = reason;
    disconnect();
  }
  void resume() { paused_ = false; }
  void disconnect() {
    permit_request_ = 0;
    runtime_.disconnect();
  }

  bool profile(const TxJob &job, radio::TxProfile &profile) {
    if (!valid_ || paused_ || !transmit_ || !radio_.available()) return false;
    if (job.enrollment) {
      uint32_t next = 0;
      const bool permitted = enrollment_ && job.request_id == permit_request_ &&
          job.shutter_id == permit_slot_ && job.incarnation == permit_incarnation_ &&
          journal_.next_counter(job.shutter_id, &next, true) && next == permit_next_;
      if (job.request_id == permit_request_) permit_request_ = 0;
      if (!permitted) return false;
    }
    profile = {static_cast<uint8_t>(job.enrollment ? 24 : 25), chip_ns_, 2001};
    return true;
  }
  bool build(const TxJob &job, const journal::Reservation &reserved,
             uint8_t phase, radio::Body &body) {
    if (reserved.incarnation != job.incarnation ||
        journal_.incarnation(job.shutter_id, job.enrollment) != job.incarnation) return false;
    return job.enrollment
        ? radio::make_enrollment_body(reserved.identity, reserved.counter, phase, &body)
        : radio::make_command_body(reserved.identity, job.action, reserved.counter, &body);
  }
  void report(const radio::TxEvent &event) {
    if (event.storage_status == journal::Status::corrupt ||
        event.storage_status == journal::Status::io_error) valid_ = false;
    if (event.job.request_id == permit_request_) permit_request_ = 0;
    if (event.storage_status == journal::Status::ok && event.job.request_id &&
        journal_.incarnation(event.job.shutter_id, event.job.enrollment) != event.job.incarnation)
      return;  // delayed outcomes cannot update a reused slot/replacement
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
  bool admit_ready() {
    if (!admit()) return false;
    const auto state = journal_.state();
    if (state != journal::StorageState::ready && state != journal::StorageState::full)
      return refuse(state == journal::StorageState::initializing ? "initializing" : "initialization_required");
    return true;
  }
  bool admit_enrollment() {
    if (!admit_ready()) return false;
    if (!enrollment_) return refuse("association_disabled");
    if (!journal_.enrollment_profile_matches(profile_.identity_suffix))
      return refuse("association_profile_unqualified");
    if (!radio_.available()) return refuse("radio_unavailable");
    return true;
  }
  bool idle(uint8_t slot = 0) { return !busy() || refuse("radio_busy", slot); }
  bool space(uint8_t slot = 0) {
    return !journal_.maintenance_due() || refuse("storage_maintenance", slot);
  }
  bool stored(journal::Status result, uint8_t slot) {
    if (result == journal::Status::ok) return true;
    if (result == journal::Status::corrupt || result == journal::Status::io_error) valid_ = false;
    return refuse(journal::protocol_error(result), slot);
  }
  bool attempt(uint8_t slot, bool retry, uint32_t now) {
    if (!stored(journal_.claim_attempt(slot, retry), slot)) return false;
    uint32_t next = 0;
    if (!journal_.next_counter(slot, &next, true)) return refuse("no_pending_association", slot);
    const auto request = job(slot, Action::none, true);
    permit_request_ = request.request_id;
    permit_slot_ = slot;
    permit_incarnation_ = request.incarnation;
    permit_next_ = next;
    if (!runtime_.submit(request, now)) { permit_request_ = 0; return false; }
    status("associating", slot);
    return true;
  }
  void enable_runtime() {
    const auto state = journal_.state();
    const bool ready = valid_ && (state == journal::StorageState::ready || state == journal::StorageState::full);
    // Gates reflect durable readiness and configured policy. Radio availability
    // is checked live at admission and profile selection, never latched at boot.
    runtime_.set_enabled(ready && transmit_, ready && enrollment_);
  }
  void boot_status() {
    status(!transmit_ ? "transmission_disabled" : !radio_.available() ? "radio_unavailable" :
           pending_slot() ? "association_pending" : "ready");
  }
  TxJob job(uint8_t slot, Action action, bool enrollment) {
    if (!++request_id_) ++request_id_;
    return {request_id_, 0, slot, action, enrollment, journal_.incarnation(slot, enrollment)};
  }
  journal::Journal &journal_;
  Radio &radio_;
  Observer &observer_;
  radio::RadioRuntime<Radio, Controller> runtime_;
  EnrollmentProfile profile_{};
  const char *pause_reason_ = "paused";
  uint32_t chip_ns_ = 208500, request_id_ = 0;
  uint32_t permit_request_ = 0, permit_incarnation_ = 0, permit_next_ = 0;
  uint8_t permit_slot_ = 0;
  bool transmit_ = false, enrollment_ = false, valid_ = false, paused_ = false;
};

}  // namespace x2d
