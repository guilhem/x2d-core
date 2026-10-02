#pragma once
#include "types.h"

namespace ha_x2d {
struct TxJob {
  uint32_t request_id = 0, deadline_ms = 0;
  uint8_t shutter_id = 0;
  Action action = Action::none;
  bool enrollment = false;
};
class TxQueue {
 public:
  static constexpr size_t CAPACITY = 16;
  // A group command can wait behind all 16 nominal ~1.3-second bursts.
  static constexpr uint32_t TTL_MS = 30000;
  template<class Report> bool push(TxJob job, uint32_t now, Report report) {
    expire(now, report);
    if (job.action == Action::stop) {
      for (size_t i = 0; i < size_;) {
        if (jobs_[i].shutter_id == job.shutter_id && !jobs_[i].enrollment) {
          report(jobs_[i], "cancelled"); remove(i);
        }
        else ++i;
      }
      // Keep STOP admissible when other shutters fill the movement queue.
      if (size_ == CAPACITY) {
        for (size_t i = size_; i > 0; --i) {
          if (jobs_[i-1].action != Action::stop) {
            report(jobs_[i-1], "cancelled"); remove(i-1); break;
          }
        }
      }
    }
    if (size_ == CAPACITY) return false;
    job.deadline_ms = now + TTL_MS;
    if (job.action == Action::stop) {
      size_t position = 0;
      while (position < size_ && jobs_[position].action == Action::stop) ++position;
      for (size_t i = size_; i > position; --i) jobs_[i] = jobs_[i-1];
      jobs_[position] = job;
    } else jobs_[size_] = job;
    ++size_; return true;
  }
  template<class Report> bool pop(uint32_t now, TxJob& job, Report report) {
    expire(now, report);
    if (!size_) return false;
    job = jobs_[0]; remove(0); return true;
  }
  bool stop_waiting() const { return size_ && jobs_[0].action == Action::stop; }
  size_t size() const { return size_; }
  template<class Report> void clear(Report report) {
    while (size_) { report(jobs_[0], "cancelled"); remove(0); }
  }
 private:
  void remove(size_t index) {
    for (size_t i = index + 1; i < size_; ++i) jobs_[i-1] = jobs_[i];
    --size_;
  }
 public:
  template<class Report> void expire(uint32_t now, Report report) {
    for (size_t i = 0; i < size_;) {
      if (static_cast<int32_t>(now - jobs_[i].deadline_ms) >= 0) {
        report(jobs_[i], "expired"); remove(i);
      } else ++i;
    }
  }
 private:
  TxJob jobs_[CAPACITY]{};
  size_t size_ = 0;
};
}  // namespace ha_x2d
