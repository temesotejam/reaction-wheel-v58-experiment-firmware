#pragma once
#include <stdint.h>
#include <math.h>

// Causal, sample-only stop policy. No model, observer interpolation or actuator.
class MeasuredQStop {
 public:
  static constexpr uint32_t GAP_US = 3333;
  static constexpr uint32_t HARD_US = 100000;
  static constexpr uint32_t RESERVE_US = 1500;
  static constexpr uint32_t CRITICAL_US = 3000;
  enum Reason : uint8_t { NONE, Q_TARGET_REACHED, Q_TARGET_NOT_REACHED,
    CURRENT_OBSERVATION_TIMEOUT, DRIVER_FAULT, SAFETY_STOP, EXPERIMENT_END,
    FIXED_START_KICK_COMPLETE, INVALID_Q_CONTEXT, HARD_WIDTH_LIMIT, FIXED_PROBE_COMPLETE };
  struct Record {
    bool enabled = false, active = false, has_sample = false, target_reached = false;
    bool fixed_duration = false;
    Reason reason = NONE;
    uint32_t start_us = 0, wait_limit_us = 0, last_sample_us = 0;
    uint32_t decision_us = 0, reach_us = 0;
    uint32_t hard_deadline_us = 0, forced_stop_deadline_us = 0;
    uint32_t planned_wait_deadline_us = 0, deadline_critical_enter_us = 0;
    bool deadline_critical_entered = false;
    float target = NAN, q = 0, previous_q = NAN, crossing_current_mA = NAN;
    float last_directed_mA = 0;
    int8_t direction = 0;
  } record;

  void reset() { record = Record{}; }
  void begin(uint32_t start, int8_t direction, float target, uint16_t planned_ms,
             uint16_t hard_ms, bool enabled) {
    reset(); auto& r = record;
    r.enabled = enabled; r.active = true; r.start_us = start;
    r.direction = direction; r.target = target;
    const uint32_t requested = static_cast<uint32_t>(planned_ms) + 10;
    r.wait_limit_us = (requested < hard_ms ? requested : hard_ms) * 1000UL;
    r.hard_deadline_us = start + HARD_US;
    r.forced_stop_deadline_us = r.hard_deadline_us - RESERVE_US;
    r.planned_wait_deadline_us = start + r.wait_limit_us;
    if (enabled && (!isfinite(target) || target <= 0 || planned_ms == 0 ||
        planned_ms > hard_ms || (direction != 1 && direction != -1)))
      stop(INVALID_Q_CONTEXT, start);
  }
  // Modular absolute timestamps: valid for these sub-2^31 us intervals.
  void beginFixed(uint32_t start, int8_t direction, uint16_t duration_ms) {
    begin(start, direction, 1.0f, duration_ms, HARD_US / 1000, true);
    record.fixed_duration = true; record.target = NAN;
    record.wait_limit_us = static_cast<uint32_t>(duration_ms) * 1000;
    record.planned_wait_deadline_us = start + record.wait_limit_us;
  }
  static bool due(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
  }
  bool deadlineCritical(uint32_t now) {
    auto& r = record;
    const uint32_t boundary = r.fixed_duration && r.wait_limit_us < HARD_US
        ? r.planned_wait_deadline_us : r.hard_deadline_us;
    if (!r.enabled || !r.active || !due(now, boundary - CRITICAL_US)) return false;
    if (!r.deadline_critical_entered) {
      r.deadline_critical_entered = true; r.deadline_critical_enter_us = now;
    }
    return true;
  }
  void stop(Reason why, uint32_t now) {
    if (!record.active || record.reason != NONE) return;
    record.reason = why; record.decision_us = now; record.active = false;
  }
  // Must run BEFORE a fresh read can erase evidence of excessive age.
  void pollObservation(uint32_t now, bool driver_ok = true) {
    auto& r = record;
    if (!r.enabled || !r.active) return;
    deadlineCritical(now);
    if (!driver_ok) { stop(DRIVER_FAULT, now); return; }
    const uint32_t reference = r.has_sample ? r.last_sample_us : r.start_us;
    if (static_cast<uint32_t>(now - reference) > GAP_US) {
      stop(CURRENT_OBSERVATION_TIMEOUT, now); return;
    }
  }
  void pollDeadline(uint32_t now) {
    if (!record.enabled || !record.active) return;
    if (due(now, record.forced_stop_deadline_us)) stop(HARD_WIDTH_LIMIT, now);
    else if (due(now, record.planned_wait_deadline_us)) stop(record.fixed_duration ? FIXED_PROBE_COMPLETE : Q_TARGET_NOT_REACHED, now);
  }
  void poll(uint32_t now, bool driver_ok = true) {
    pollObservation(now, driver_ok);
    pollDeadline(now);
  }
  void sample(uint32_t now, float raw_mA, bool valid) {
    auto& r = record;
    if (!r.enabled || !r.active) return;
    // Check completion as well as pre-read age: a slow read is not fresh enough.
    if (!valid || !isfinite(raw_mA)) {
      stop(CURRENT_OBSERVATION_TIMEOUT, now); return;
    }
    pollObservation(now);
    if (!r.active) return;
    const float directed = r.direction * raw_mA;
    r.previous_q = r.q;
    if (r.has_sample) {
      const uint32_t gap = now - r.last_sample_us;
      if (!gap) { stop(CURRENT_OBSERVATION_TIMEOUT, now); return; }
      r.q += 0.5f * (r.last_directed_mA + directed) * gap * 1.0e-6f;
    }
    r.has_sample = true; r.last_sample_us = now; r.last_directed_mA = directed;
    if (!r.fixed_duration && r.q >= r.target) {
      r.target_reached = true;
      r.crossing_current_mA = directed; r.reach_us = now;
      stop(Q_TARGET_REACHED, now);
    }
    // An already completed valid read is integrated before checking the time
    // limit. A NEW read is prohibited by pre-read poll at the reservation.
    pollDeadline(now);
  }
  static const char* name(Reason r) {
    switch (r) {
      case NONE: return "OPEN";
      case Q_TARGET_REACHED: return "Q_TARGET_REACHED";
      case Q_TARGET_NOT_REACHED: return "Q_TARGET_NOT_REACHED";
      case CURRENT_OBSERVATION_TIMEOUT: return "CURRENT_OBSERVATION_TIMEOUT";
      case DRIVER_FAULT: return "DRIVER_FAULT";
      case SAFETY_STOP: return "SAFETY_STOP";
      case EXPERIMENT_END: return "EXPERIMENT_END";
      case FIXED_START_KICK_COMPLETE: return "FIXED_START_KICK_COMPLETE";
      case INVALID_Q_CONTEXT: return "INVALID_Q_CONTEXT";
      case HARD_WIDTH_LIMIT: return "HARD_WIDTH_LIMIT";
      case FIXED_PROBE_COMPLETE: return "FIXED_PROBE_COMPLETE";
    }
    return "UNKNOWN";
  }
};
