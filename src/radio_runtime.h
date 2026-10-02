#pragma once

#include "journal.h"
#include "radio_codec.h"
#include "tx_queue.h"

namespace ha_x2d {
namespace radio {

struct TxProfile {
  uint8_t copies = 0;  // caller-qualified; zero refuses transmission
  uint32_t chip_ns = 208500;  // caller can calibrate fractional microseconds
  uint32_t second_start_ms = 2001;  // enrollment only, from the first burst start
};

enum class FrameState : uint8_t { busy, complete, unknown };

struct TxEvent {
  TxJob job;
  const char* outcome;  // emitted, cancelled, expired, rejected, unknown
  const char* error = nullptr;
  uint8_t completed_copies = 0;  // whole copies, summed across stages (at most 64)
  journal::Status storage_status = journal::Status::ok;
};

// Single owner, called from the radio loop, never from an ISR/USB lock.
// Radio (nonblocking):
//   bool start_burst(const Waveform&, uint32_t chip_ns, uint32_t& started_ms):
//     all copies continuously, actual SM start observation returned as millis.
//     The waveform remains alive/unchanged until end_burst(). False guarantees
//     no chip was sent and no end_burst() is needed.
//   FrameState poll_burst(uint8_t& completed_copies): absolute whole-copy count
//     for the current burst, also on busy/unknown. Complete means the last
//     counted frame's final chip lasted fully AND the output is safely parked.
//   void request_stop(): park after the next full frame, never a partial copy.
//   void end_burst(): carrier off/release waveform, only after complete/fault.
// Hooks:
//   bool profile(const TxJob&, TxProfile&): only return qualified profiles.
//   bool build(const TxJob&, const journal::Reservation&, uint8_t phase, Body&):
//     phase 0 for normal commands, phases 0/1 for genuine B enrollment; dispatch to
//     caller's action/enrollment builder. No action bytes or enrollment format
//     are invented here. BOTH enrollment reservations and waveforms precede RF.
//   void report(const TxEvent&): record/enqueue promptly; never wait for USB.
// Hooks must not reenter the runtime. Terminal events occur once per request;
// Non-emitted outcomes can include completed copies (no RF rollback).
// Idle maintenance failures use request_id 0 as a storage-level event.
// No implicit provisioning, identity generation, confirmation or replay.
// Keep static (two waveforms). Dependencies and hooks must outlive the runtime.
template<class Radio, class Hooks>
class RadioRuntime {
 public:
  RadioRuntime(journal::Journal& journal, Radio& radio, Hooks& hooks)
      : journal_(journal), radio_(radio), hooks_(hooks) {}

  // Both gates default disabled. A gate change invalidates old requests;
  // an in-flight frame still finishes. Enabling is NOT physical qualification.
  void set_enabled(bool transmit, bool enrollment = false) {
    if (transmit != transmit_enabled_ || enrollment != enrollment_enabled_)
      disconnect();
    transmit_enabled_ = transmit;
    enrollment_enabled_ = enrollment;
  }

  bool submit(TxJob job, uint32_t now) {
    const char* error = nullptr;
    if (!job.request_id || job.shutter_id < 1 || job.shutter_id > MAX_SHUTTERS ||
        (job.enrollment ? job.action != Action::none
                        : job.action != Action::open && job.action != Action::close &&
                          job.action != Action::stop)) error = "invalid_request";
    else if (!transmit_enabled_ || (job.enrollment && !enrollment_enabled_))
      error = "tx_disabled";
    else if (fault_ != journal::Status::ok) {
      report(job, "rejected", storage_error(fault_), 0, fault_);
      return false;
    } else if (journal_.state() == journal::StorageState::corrupt) {
      report(job, "rejected", "storage_corrupt", 0, journal::Status::corrupt);
      storage_fault(journal::Status::corrupt);
      return false;
    } else if (journal_.state() == journal::StorageState::full) {
      report(job, "rejected", "storage_full", 0, journal::Status::no_space);
      return false;
    } else if (journal_.shutter(job.shutter_id).state == journal::SlotState::unused)
      error = "unknown_shutter";
    else if (!job.enrollment &&
             journal_.shutter(job.shutter_id).state != journal::SlotState::paired)
      error = "not_paired";
    else if (is_stop(job) && active_ && is_stop(current_) &&
             current_.shutter_id == job.shutter_id)
      error = "stop_in_progress";  // coalesce without a second reservation
    else if (!is_stop(job) && journal_.maintenance_due())
      error = "maintenance_pending";
    if (error) {
      report(job, "rejected", error);
      return false;
    }
    if (!queue_.push(job, now, queue_report())) {
      report(job, "rejected", "queue_full");
      return false;
    }
    if (is_stop(job) && active_ && !is_stop(current_))
      cancel("cancelled", "stop_preempted");
    return true;
  }

  // Session loss/clear: drop pending requests and stop the active burst at
  // its next full frame. Never undo reservations, and never resume old work.
  void disconnect() {
    queue_.clear(queue_report());
    if (active_) cancel("cancelled", "session_disconnected");
  }

  size_t pending() const { return queue_.size(); }
  bool active() const { return active_; }

  void tick(uint32_t now) {
    queue_.expire(now, queue_report());
    if (active_) {
      if (!cancel_outcome_ && expired(current_, now))
        cancel("expired", "deadline_expired");
      if (active_ && burst_running_) {
        uint8_t copies = 0;
        const FrameState state = radio_.poll_burst(copies);
        completed_ = static_cast<uint8_t>((phase_ ? profile_.copies : 0) + copies);
        if (state == FrameState::busy) return;
        radio_.end_burst();
        burst_running_ = false;
        if (state == FrameState::unknown) finish("unknown", "radio_fault");
        else if (cancel_outcome_) finish(cancel_outcome_, cancel_error_);
        else if (copies != profile_.copies) finish("unknown", "incomplete_burst");
        else if (!current_.enrollment || phase_ == 1) finish("emitted");
      }
      if (active_) {
        // Carrier idle, no journal maintenance or other command in this gap.
        if (static_cast<int32_t>(now - second_start_) >= 0) {
          phase_ = 1;
          start_burst(now);
          return;
        }
        return;
      }
    }
    if (fault_ != journal::Status::ok) return;
    // STOP bypasses maintenance, including a rotation partly erased at idle.
    // Every other command waits for idle maintenance BEFORE reserving a page.
    if (!queue_.stop_waiting() && journal_.maintenance_due()) {
      const journal::Status status = journal_.maintain();
      if (status != journal::Status::ok) {
        report(TxJob{}, "rejected", storage_error(status), 0, status);
        storage_fault(status);
      }
      return;  // at most one bounded maintenance step per tick
    }
    if (!queue_.pop(now, current_, queue_report())) return;
    profile_ = TxProfile{};
    if (!hooks_.profile(current_, profile_) || !profile_.copies ||
        profile_.copies > MAX_COPIES || !profile_.chip_ns ||
        (current_.enrollment && (!profile_.second_start_ms ||
                                profile_.second_start_ms >= ENROLLMENT_ACTIVE_MS))) {
      report(current_, "rejected", "unqualified_profile");
      return;
    }
    const uint8_t phases = current_.enrollment ? 2 : 1;
    journal::Reservation reservations[2];
    for (uint8_t phase = 0; phase < phases; ++phase) {
      const journal::Status status = journal_.reserve(
          current_.shutter_id, static_cast<uint8_t>(current_.action),
          is_stop(current_), &reservations[phase]);
      if (status != journal::Status::ok) {
        report(current_, "rejected", storage_error(status), 0, status);
        if (status == journal::Status::io_error || status == journal::Status::corrupt)
          storage_fault(status);
        return;
      }
    }
    for (uint8_t phase = 0; phase < phases; ++phase) {
      Body body{};
      if (!hooks_.build(current_, reservations[phase], phase, body) ||
          !encode_burst(body, profile_.copies, &waves_[phase])) {
        report(current_, "rejected", "body_refused");
        return;  // both reservations remain consumed
      }
    }
    if (current_.enrollment &&
        uint64_t{waves_[0].chips()} * profile_.chip_ns >=
            uint64_t{profile_.second_start_ms} * 1000000) {
      report(current_, "rejected", "enrollment_overlap");
      return;
    }
    completed_ = 0;
    phase_ = 0;
    cancel_outcome_ = cancel_error_ = nullptr;
    if (current_.enrollment) current_.deadline_ms = now + ENROLLMENT_ACTIVE_MS;
    active_ = true;
    start_burst(now);
  }

 private:
  static constexpr uint32_t ENROLLMENT_ACTIVE_MS = 6000;
  static bool is_stop(const TxJob& job) {
    return !job.enrollment && job.action == Action::stop;
  }
  static bool expired(const TxJob& job, uint32_t now) {
    return static_cast<int32_t>(now - job.deadline_ms) >= 0;
  }
  static const char* storage_error(journal::Status status) {
    if (status == journal::Status::io_error) return "storage_io_error";
    return journal::protocol_error(status);
  }
  void report(const TxJob& job, const char* outcome, const char* error = nullptr,
              uint8_t completed = 0, journal::Status status = journal::Status::ok) {
    hooks_.report(TxEvent{job, outcome, error, completed, status});
  }
  auto queue_report() {
    return [this](const TxJob& job, const char* outcome) {
      report(job, outcome, !strcmp(outcome, "expired") ? "queue_expired" : "queue_cancelled");
    };
  }
  void storage_fault(journal::Status status) {
    fault_ = status;  // caller must recover journal and create a fresh runtime
    queue_.clear([this, status](const TxJob& job, const char*) {
      report(job, "rejected", storage_error(status), 0, status);
    });
  }
  void start_burst(uint32_t now) {
    uint32_t started = now;
    if (!radio_.start_burst(waves_[phase_], profile_.chip_ns, started))
      finish("rejected", "radio_start_failed");
    else {
      burst_running_ = true;
      if (!phase_) second_start_ = started + profile_.second_start_ms;
    }
  }
  void cancel(const char* outcome, const char* error) {
    if (cancel_outcome_) return;
    cancel_outcome_ = outcome;
    cancel_error_ = error;
    if (burst_running_) radio_.request_stop();
    else finish(outcome, error);
  }
  void finish(const char* outcome, const char* error = nullptr) {
    active_ = false;
    cancel_outcome_ = cancel_error_ = nullptr;
    report(current_, outcome, error, completed_);
  }

  journal::Journal& journal_;
  Radio& radio_;
  Hooks& hooks_;
  TxQueue queue_;
  Waveform waves_[2];
  TxJob current_{};
  TxProfile profile_;
  journal::Status fault_ = journal::Status::ok;
  const char* cancel_outcome_ = nullptr;
  const char* cancel_error_ = nullptr;
  uint32_t second_start_ = 0;
  uint8_t phase_ = 0;
  uint8_t completed_ = 0;
  bool active_ = false, burst_running_ = false;
  bool transmit_enabled_ = false, enrollment_enabled_ = false;
};

}  // namespace radio
}  // namespace ha_x2d
