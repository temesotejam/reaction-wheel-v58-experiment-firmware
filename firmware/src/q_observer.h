#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "measured_q_stop.h"
#include "wheel_observer.h"
#include "wheel_probe_v55.h"
#include "fixed_probe_v57.h"

// No Arduino, actuator or runner dependency. Observation cannot command output.
class QObserver {
 public:
  static constexpr uint32_t POST_US = 100000, GAP_US = 3333;
  static constexpr size_t SAMPLE_CAPACITY = 131072, PULSE_CAPACITY = 256;
  struct Sample {
    uint32_t time_us = 0, sequence = 0;
    int32_t raw = 0;
    uint32_t read_duration_us = 0;
    uint8_t valid = 0, padding[3]{};
  };
  static_assert(sizeof(Sample) == 20, "RWLOG v48 current attempt size");
  struct Context {
    float target = NAN, pred = NAN, goal_dir = NAN, tau_s = NAN;
    float initial_model_raw_mA = NAN;
    uint16_t width_ms = 0, battery_mV = 0;
    int8_t physical_side = 0;
    bool saturated = false;
    bool measured_stop_enabled = false;
    bool fixed_duration_v55 = false;
    uint8_t v55_role = 0, v55_trial_id = 0;
    int8_t v55_probe_direction = 0;
    int16_t v55_target_aligned_rpm = 0;
    uint32_t v55_status_time_us = 0;
    uint8_t v55_driver_mode = 0, v55_driver_status = 0, v55_driver_error = 0;
  };
  struct Reach { bool reached = false; uint32_t time_us = 0; };
  enum EndReason : uint8_t { OPEN = 0, WINDOW_100MS = 1, NEXT_COMMAND = 2,
                            RUN_END = 3, WRITE_FAILED = 4 };
  struct Pulse {
    uint32_t id = 0, command_start_us = 0, nonzero_write_begin_us = 0;
    uint32_t nonzero_write_end_us = 0, nonzero_output_end_us = 0;
    uint32_t stop_command_us = 0, zero_write_begin_us = 0, zero_write_end_us = 0;
    uint32_t zero_output_end_us = 0, post_end_us = 0;
    int8_t direction = 0;
    bool start_ok = false, stop_ok = false, has_first = false;
    bool active_bad = false, post_bad = false, linear_available = false, model_available = false;
    uint8_t phase = 1, end_reason = OPEN; // 1 active, 2 post, 3 closed
    Sample before, first, at_stop;
    Context context;
    float measured = 0, start_linear = NAN, start_model = NAN;
    float post_measured = 0;
    uint32_t active_covered_us = 0, post_covered_us = 0, active_samples = 0;
    uint32_t active_failures = 0, post_failures = 0, active_max_gap_us = 0, post_max_gap_us = 0;
    Reach a, b, c;
    MeasuredQStop::Record stopping;
    int8_t previous_direction = 0;
    uint16_t consecutive_same_direction = 0;
    WheelSample wheel_before, wheel_after;
  };
  Sample* samples = nullptr;
  Pulse* pulses = nullptr;
  uint32_t sample_count = 0, pulse_count = 0, sample_overflow = 0, pulse_overflow = 0;
  bool recording = false;
  // V57 enables raw storage only for the pre-probe, active, and post-probe
  // window. Observation and pulse integration continue while it is disabled.
  bool raw_sample_recording = true;
  float raw_per_mA = 100.0f;
  Context pending;
  WheelObserver wheel;
  WheelProbeV55 v55;
  FixedProbeV57 v57;

  void reset(bool enable) {
    v55.reset(); v57.reset();
    wheel.reset(enable);
    sample_count = pulse_count = sample_overflow = pulse_overflow = 0;
    recording = enable; raw_sample_recording = true; index_ = -1; pending = Context{};
    // Keep latest real current for start-edge diagnostics; timestamps expose age.
  }
  bool allocated() const { return samples && pulses; }
  void setRawSampleRecording(bool enable) { raw_sample_recording = enable; }
  bool rawSampleRecording() const { return raw_sample_recording; }
  bool observingPost() const { return index_ >= 0 && pulses[index_].phase == 2; }
  Pulse* currentPulse() { return index_ >= 0 ? &pulses[index_] : nullptr; }
  void close(uint32_t now, EndReason reason) {
    if (index_ < 0) return;
    Pulse& p = pulses[index_];
    if (p.phase == 3) return;
    if (p.phase == 2 && static_cast<uint32_t>(now - p.zero_write_end_us) >= POST_US) {
      p.post_end_us = p.zero_write_end_us + POST_US; p.end_reason = WINDOW_100MS;
    } else { p.post_end_us = now; p.end_reason = reason; }
    if (p.phase == 1) p.active_bad = true;
    p.phase = 3;
  }
  void finish(uint32_t now) { close(now, RUN_END); recording = false; wheel.recording = false; pending = Context{}; }
  void beforeNonzero(uint32_t write_begin) { close(write_begin, NEXT_COMMAND); }
  void begin(uint32_t command, uint32_t wb, uint32_t we, uint32_t oe, int8_t d, bool ok) {
    if (!recording || !allocated()) { pending = Context{}; return; }
    if (pulse_count >= PULSE_CAPACITY) { ++pulse_overflow; index_ = -1; pending = Context{}; return; }
    index_ = static_cast<int>(pulse_count++);
    Pulse& p = pulses[index_]; p = Pulse{};
    p.id = pulse_count; p.command_start_us = command;
    p.nonzero_write_begin_us = wb; p.nonzero_write_end_us = we; p.nonzero_output_end_us = oe;
    p.direction = d; p.start_ok = ok; p.active_bad = !ok;
    p.before = latest_; p.context = pending; pending = Context{};
    p.wheel_before = wheel.latest; // cache snapshot, never claimed as a new read
    previous_ = Sample{};
  }
  void stop(uint32_t command, uint32_t wb, uint32_t we, uint32_t oe, bool ok) {
    if (!recording || index_ < 0) return;
    Pulse& p = pulses[index_];
    if (p.phase != 1) return; // redundant zero writes do not reset the window
    p.stop_command_us = command; p.zero_write_begin_us = wb; p.zero_write_end_us = we;
    p.zero_output_end_us = oe; p.at_stop = latest_; p.stop_ok = ok;
    p.active_bad |= !ok; p.post_bad |= !ok; p.phase = 2; previous_ = Sample{};
  }
  void sample(uint32_t now, uint32_t seq, int32_t raw, uint32_t duration, bool valid) {
    Sample s; s.time_us = now; s.sequence = seq; s.raw = raw;
    s.read_duration_us = duration; s.valid = valid;
    if (recording && raw_sample_recording && allocated()) {
      if (sample_count < SAMPLE_CAPACITY) samples[sample_count++] = s;
      else ++sample_overflow;
    }
    if (recording && index_ >= 0) {
      Pulse& p = pulses[index_];
      if (p.phase == 2 && static_cast<uint32_t>(now - p.zero_write_end_us) >= POST_US) {
        // Retain the bracketing raw sample; post-boundary interpolation is offline.
        close(now, WINDOW_100MS);
      }
      if (p.phase == 1 || p.phase == 2) accumulate(p, s);
    }
    if (valid) latest_ = s;
  }
 private:
  int index_ = -1;
  Sample latest_, previous_;
  float directed(const Sample& s, int8_t d) const { return d * (static_cast<float>(s.raw) / raw_per_mA); }
  static void reach(Reach& r, float q, float target, uint32_t now) {
    if (!r.reached && isfinite(target) && target > 0 && isfinite(q) && q >= target) {
      r.reached = true; r.time_us = now;
    }
  }
  void accumulate(Pulse& p, const Sample& s) {
    const bool active = p.phase == 1;
    if (!s.valid) {
      if (active) { ++p.active_failures; p.active_bad = true; }
      else { ++p.post_failures; p.post_bad = true; }
      previous_ = Sample{}; // never bridge a failed read as measured data
      return;
    }
    if (active) {
      ++p.active_samples;
      if (!p.has_first) {
        p.has_first = true; p.first = s;
        const uint32_t missing = s.time_us - p.nonzero_write_end_us;
        const uint32_t bracket = s.time_us - p.before.time_us;
        if (p.before.valid && static_cast<int32_t>(p.nonzero_write_end_us - p.before.time_us) >= 0 &&
            bracket > 0 && bracket <= 50000) {
          const float end_i = directed(s, p.direction);
          const float before_i = directed(p.before, p.direction);
          const float edge_i = end_i + (before_i - end_i) * static_cast<float>(missing) / bracket;
          p.start_linear = 0.5f * (edge_i + end_i) * missing * 1.0e-6f;
          p.linear_available = true;
        }
        if (p.before.valid && static_cast<uint32_t>(p.nonzero_write_end_us - p.before.time_us) <= 50000 &&
            isfinite(p.context.goal_dir) && isfinite(p.context.tau_s) && p.context.tau_s > 0) {
          const float t = missing * 1.0e-6f, goal = p.context.goal_dir, tau = p.context.tau_s;
          p.start_model = goal * t + (directed(p.before, p.direction) - goal) * tau * (-expm1f(-t / tau));
          p.model_available = true;
        }
      }
    }
    if (previous_.valid) {
      const uint32_t gap = s.time_us - previous_.time_us;
      uint32_t& maximum = active ? p.active_max_gap_us : p.post_max_gap_us;
      if (gap > maximum) maximum = gap;
      if (gap > 0 && gap <= GAP_US) {
        const float q = 0.5f * (directed(previous_, p.direction) + directed(s, p.direction)) * gap * 1.0e-6f;
        if (active) { p.measured += q; p.active_covered_us += gap; }
        else { p.post_measured += q; p.post_covered_us += gap; }
      } else if (active) p.active_bad = true;
      else p.post_bad = true;
    }
    previous_ = s;
    if (active && !p.active_bad && !sample_overflow) {
      reach(p.a, p.measured, p.context.target, s.time_us);
      if (p.linear_available) reach(p.b, p.measured + p.start_linear, p.context.target, s.time_us);
      if (p.model_available) reach(p.c, p.measured + p.start_model, p.context.target, s.time_us);
    }
  }
};
