#pragma once

#include <stdint.h>
#include <stddef.h>

// Observation-only, independent of Arduino and motor control. All times are
// successful read-completion micros(), modulo 2^32. Native tests exercise the
// same implementation as the firmware.
class CurrentTimingAudit {
 public:
  static constexpr uint32_t LIMIT_US = 3333;
  static constexpr uint32_t TAIL_US = 10000;
  static constexpr size_t HISTORY_CAPACITY = 128;
  static constexpr size_t PULSE_CAPACITY = 256;
  static constexpr size_t VIOLATION_CAPACITY = 128;
  static constexpr size_t HISTOGRAM_BINS = 101;
  enum Task : uint8_t { IMU = 1, FULL_STATUS = 2, WEB = 4 };
  struct Stats {
    uint32_t count = 0, max_us = 0, gt3333 = 0, gt5000 = 0, gt10000 = 0;
    uint32_t bins[HISTOGRAM_BINS]{};  // [100*i,100*i+99]; last bin >=10000.
    void add(uint32_t gap) {
      ++count;
      if (gap > max_us) max_us = gap;
      gt3333 += gap > LIMIT_US; gt5000 += gap > 5000; gt10000 += gap > 10000;
      ++bins[gap / 100 < HISTOGRAM_BINS ? gap / 100 : HISTOGRAM_BINS - 1];
    }
  };
  struct Pulse {
    uint32_t pulse_id = 0, start_us = 0, end_us = 0;
    uint32_t current_read_count = 0, tail_current_read_count = 0;
    uint32_t interval_count = 0, current_gap_max_us = 0, current_gap_gt3333_count = 0;
    uint32_t tail_interval_count = 0, tail_current_gap_max_us = 0;
    uint32_t tail_current_gap_gt3333_count = 0;
    uint32_t pulse_end_current_age_us = UINT32_MAX;
    uint32_t failed_reads = 0, pre_imu_reads = 0, post_imu_reads = 0;
    uint32_t pre_status_reads = 0, post_status_reads = 0;
    bool tail_covered = false;
  };
  struct Violation {
    uint32_t gap_us = 0, gap_start_time = 0, gap_end_time = 0, pulse_id = 0;
    uint32_t time_to_pulse_end_us = UINT32_MAX;
    uint32_t imu_duration_us = 0, full_status_duration_us = 0, web_duration_us = 0;
    uint32_t previous_current_read_duration_us = 0, current_read_duration_us = 0;
    uint32_t failed_reads = 0;
    uint8_t tasks = 0;
    bool end_age_only = false; // censored interval: no successful read at end.
  };
  Stats whole, tail;
  Pulse pulses[PULSE_CAPACITY]{};
  Violation violations[VIOLATION_CAPACITY]{};
  uint32_t pulse_count = 0, violation_count = 0, pulse_overflow = 0, violation_overflow = 0;
  uint32_t started_pulse_count = 0;
  bool active = false;
  Pulse current;

  void reset() {
    whole = Stats{}; tail = Stats{}; current = Pulse{}; context_ = Violation{};
    pulse_count = violation_count = pulse_overflow = violation_overflow = started_pulse_count = 0;
    active = have_previous_ = false; history_head_ = history_count_ = 0;
    // Counts invalidate old slots; never allocate this large object on the stack.
  }
  void begin(uint32_t now) {
    current = Pulse{};
    current.pulse_id = ++started_pulse_count;
    current.start_us = now;
    active = true; have_previous_ = false; history_head_ = history_count_ = 0;
    context_ = Violation{}; first_violation_ = violation_count;
  }
  void noteTask(Task task, uint32_t duration) {
    if (!active || !have_previous_) return;
    context_.tasks |= task;
    if (task == IMU) context_.imu_duration_us += duration;
    if (task == FULL_STATUS) context_.full_status_duration_us += duration;
    if (task == WEB) context_.web_duration_us += duration;
  }
  void failedRead() {
    if (active) { ++current.failed_reads; ++context_.failed_reads; }
  }
  void success(uint32_t now, uint32_t read_duration) {
    if (!active) return;
    ++current.current_read_count;
    if (have_previous_) {
      const uint32_t gap = now - previous_us_;
      whole.add(gap);
      ++current.interval_count;
      if (gap > current.current_gap_max_us) current.current_gap_max_us = gap;
      if (gap > LIMIT_US) {
        ++current.current_gap_gt3333_count;
        recordViolation(now, gap, read_duration, false);
      }
      history_[history_head_] = {previous_us_, now, gap};
      history_head_ = (history_head_ + 1) % HISTORY_CAPACITY;
      if (history_count_ < HISTORY_CAPACITY) ++history_count_;
    } else {
      first_sample_us_ = now;
    }
    previous_us_ = now; previous_read_duration_ = read_duration;
    have_previous_ = true; context_ = Violation{};
  }
  void end(uint32_t now) {
    if (!active) return;
    current.end_us = now;
    const uint32_t start = now - TAIL_US;
    bool history_reaches_start = false;
    for (size_t i = 0; i < history_count_; ++i) {
      const Gap& gap = history_[(history_head_ + HISTORY_CAPACITY - history_count_ + i) % HISTORY_CAPACITY];
      // Count the FULL interval when it overlaps the tail, including the gap
      // crossing tail_start. Equality at tail_start is conservatively included.
      if (static_cast<int32_t>(gap.before - now) > 0 ||
          static_cast<int32_t>(gap.after - start) < 0) continue;
      if (static_cast<int32_t>(gap.before - start) <= 0) history_reaches_start = true;
      tail.add(gap.duration);
      ++current.tail_interval_count;
      if (gap.duration > current.tail_current_gap_max_us) current.tail_current_gap_max_us = gap.duration;
      current.tail_current_gap_gt3333_count += gap.duration > LIMIT_US;
      if (static_cast<int32_t>(gap.after - start) >= 0) ++current.tail_current_read_count;
    }
    if (have_previous_) {
      current.pulse_end_current_age_us = now - previous_us_;
      // A pre-tail last read still establishes known coverage up to pulse end;
      // the separate end-age criterion then rejects a long terminal gap.
      if (static_cast<int32_t>(previous_us_ - start) <= 0) history_reaches_start = true;
      if (static_cast<int32_t>(first_sample_us_ - start) >= 0 &&
          static_cast<int32_t>(first_sample_us_ - now) <= 0) ++current.tail_current_read_count;
      if (current.pulse_end_current_age_us > LIMIT_US)
        recordViolation(now, current.pulse_end_current_age_us, 0, true);
    }
    current.tail_covered = have_previous_ && (now - current.start_us >= TAIL_US) &&
        static_cast<int32_t>(first_sample_us_ - start) <= 0 && history_reaches_start;
    for (uint32_t i = first_violation_; i < violation_count; ++i)
      violations[i].time_to_pulse_end_us = now - violations[i].gap_end_time;
    if (pulse_count < PULSE_CAPACITY) pulses[pulse_count++] = current;
    else ++pulse_overflow;
    active = false;
  }

 private:
  struct Gap { uint32_t before, after, duration; };
  Gap history_[HISTORY_CAPACITY]{};
  size_t history_head_ = 0, history_count_ = 0;
  uint32_t previous_us_ = 0, first_sample_us_ = 0, previous_read_duration_ = 0;
  uint32_t first_violation_ = 0;
  bool have_previous_ = false;
  Violation context_;
  void recordViolation(uint32_t now, uint32_t gap, uint32_t read_duration, bool end_age) {
    if (violation_count >= VIOLATION_CAPACITY) { ++violation_overflow; return; }
    Violation& v = violations[violation_count++];
    v = context_;
    v.gap_us = gap; v.gap_start_time = previous_us_; v.gap_end_time = now;
    v.pulse_id = current.pulse_id; v.end_age_only = end_age;
    v.previous_current_read_duration_us = previous_read_duration_;
    v.current_read_duration_us = read_duration;
  }
};
