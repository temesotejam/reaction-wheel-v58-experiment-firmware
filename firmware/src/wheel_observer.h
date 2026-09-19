#pragma once
#include <stdint.h>
#include <stddef.h>

// Diagnostics only: no actuator, Q estimator, or controller dependency.
struct WheelSample {
  uint32_t begin_us = 0, time_us = 0, sequence = 0;
  int32_t raw = 0;
  bool valid = false;
  double rpm() const { return raw / 100.0; }
  double radS() const { return rpm() * 0.10471975511965977; }
};

class WheelObserver {
 public:
  static constexpr size_t CAPACITY = 6144;
  static constexpr uint32_t PERIOD_US = 50000;
  WheelSample* samples = nullptr;
  WheelSample latest;
  uint32_t count = 0, overflow = 0, failures = 0, sequence = 0;
  bool recording = false, retain_samples = true;
  void reset(bool enable) {
    count = overflow = failures = 0;
    recording = enable;
    // Latest real attempt and global sequence retained; expose actual age.
  }
  void attempt(uint32_t begin, uint32_t end, int32_t raw, bool valid) {
    latest.begin_us = begin; latest.time_us = end;
    latest.sequence = ++sequence; latest.raw = raw; latest.valid = valid;
    if (!recording) return;
    if (!valid) ++failures;
    if (!retain_samples) return;
    if (samples && count < CAPACITY) samples[count++] = latest;
    else ++overflow;
  }
};

