#pragma once
#include <stdint.h>
#include <math.h>

// Discrete, bounded state preparation for an identification experiment only.
// No energy target, online Q model, or normal-mode actuator dependency.
struct WheelProbeV55 {
  static constexpr uint8_t TRIALS = 10, MAX_PREP = 16;
  static constexpr uint16_t CURRENT_MA = 300, PROBE_MS = 60;
  static constexpr uint32_t MAX_AGE_US = 2000, REST_US = 300000;
  static constexpr uint32_t TRIAL_LIMIT_US = 15000000, RUN_LIMIT_MS = 180000;
  static constexpr float BAND_RPM = 50, MAX_INITIAL_RPM = 650;
  enum Result : uint8_t { PENDING, COMPLETE, SPEED_READ_FAILED, SPEED_OUTSIDE_BAND,
    SPEED_AGE_EXCEEDED, SPEED_LIMIT, PREPARATION_LIMIT, TRIAL_TIMEOUT,
    CURRENT_FAILURE, DRIVER_FAILURE, STORAGE_LIMIT, USER_STOP, RUN_TIMEOUT };
  struct Trial {
    uint8_t id = 0, preparation_count = 0;
    int8_t direction = 0;
    int16_t target_aligned_rpm = 0;
    uint32_t start_us = 0, end_us = 0, probe_pulse_id = 0;
    Result result = PENDING;
  } trials[TRIALS];
  bool enabled = false, started = false, finished = false, aborted = false;
  uint8_t index = 0;
  uint32_t start_us = 0, end_us = 0;
  Result abort_reason = PENDING;
  void reset() { *this = WheelProbeV55{}; }
  void begin(uint16_t run_id, uint32_t now) {
    reset(); enabled = started = true; start_us = now;
    // Every run covers all 5 bands x both signs. Rotate bands and reverse the
    // first direction across runs, avoiding a monotonic speed/time schedule.
    const int16_t bands[5] = {0, 400, -200, 200, -400};
    for (uint8_t k = 0; k < TRIALS; ++k) {
      auto& t = trials[k]; t.id = k + 1;
      t.target_aligned_rpm = bands[(k / 2 + (run_id - 1) % 5) % 5];
      t.direction = ((k + run_id) & 1) ? 1 : -1;
    }
    trials[0].start_us = now;
  }
  Trial* current() { return started && index < TRIALS ? &trials[index] : nullptr; }
  static bool inBand(float rpm, int8_t d, int16_t target) {
    return isfinite(rpm) && fabsf(d * rpm - target) <= BAND_RPM;
  }
  static int8_t preparationDirection(float rpm, int8_t d, int16_t target) {
    const float error = d * target - rpm;
    return error > 0 ? 1 : -1;
  }
  static uint16_t preparationWidth(float rpm, int8_t d, int16_t target) {
    return fabsf(d * target - rpm) > 150 ? 20 : 5;
  }
  void advance(Result why, uint32_t now) {
    if (auto* t = current()) { t->result = why; t->end_us = now; ++index; }
    if (index >= TRIALS) { finished = true; end_us = now; }
    else trials[index].start_us = now;
  }
  void abort(Result why, uint32_t now) {
    if (auto* t = current()) { t->result = why; t->end_us = now; }
    aborted = finished = true; abort_reason = why; end_us = now;
  }
  static const char* name(Result why) {
    switch (why) {
      case PENDING: return "PENDING"; case COMPLETE: return "COMPLETE";
      case SPEED_READ_FAILED: return "SPEED_READ_FAILED";
      case SPEED_OUTSIDE_BAND: return "SPEED_OUTSIDE_BAND";
      case SPEED_AGE_EXCEEDED: return "SPEED_AGE_EXCEEDED";
      case SPEED_LIMIT: return "SPEED_LIMIT";
      case PREPARATION_LIMIT: return "PREPARATION_LIMIT";
      case TRIAL_TIMEOUT: return "TRIAL_TIMEOUT";
      case CURRENT_FAILURE: return "CURRENT_FAILURE";
      case DRIVER_FAILURE: return "DRIVER_FAILURE";
      case STORAGE_LIMIT: return "STORAGE_LIMIT";
      case USER_STOP: return "USER_STOP"; case RUN_TIMEOUT: return "RUN_TIMEOUT";
    }
    return "UNKNOWN";
  }
};
