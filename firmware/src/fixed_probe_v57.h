#pragma once

#include <math.h>
#include <stdint.h>
#include "fixed_probe_baseline.h"

// V57 is measurement-only. It never selects a Q target or changes the fixed
// 300 mA / 60 ms probe from measured current, Q, or wheel speed.
struct FixedProbeV57 {
  static constexpr uint32_t RUN_LIMIT_MS = 600000;
  static constexpr uint32_t SETTLE_LIMIT_US = 10000000;
  static constexpr uint32_t BASELINE_US = 200000, PREP_LIMIT_US = 3000000;
  static constexpr uint16_t BASELINE_MIN_CURRENT_SAMPLES = 20;
  static constexpr uint32_t HOLD_US = 200000, SPEED_PERIOD_US = 10000;
  static constexpr uint32_t BASELINE_SPEED_PERIOD_US = 50000;
  static constexpr uint32_t PREPARATION_SLOPE_WINDOW_US = 100000;
  static constexpr uint32_t TRANSFER_LIMIT_US = 1500000;
  static constexpr uint32_t TRANSFER_CONTINUOUS_US = 30000;
  static constexpr float TRANSFER_TOLERANCE_MA = 0.2f;
  static constexpr uint32_t PROBE_US = 60000, ZERO_RESERVATION_US = 1500;
  static constexpr uint32_t MAX_SPEED_AGE_US = 2000;
  // A single CURRENT_READBACK miss outside the raw pulse/post window is
  // retried immediately. Two consecutive misses still terminate the run.
  // During the pulse or its post window every miss remains terminal so the
  // fixed-pulse current evidence is never silently bridged.
  static constexpr uint8_t NON_PULSE_CURRENT_READ_ATTEMPTS = 2;
  static constexpr double PREPARATION_MAX_SLOPE_RPM_PER_S = 100.0;
  static constexpr int CURRENT_LIMIT_MA = 300, PROBE_CURRENT_MA = 300;
  static constexpr int MAX_RPM = 650, QUIET_RPM = 5;
  static constexpr uint8_t SIMPLE_TRIAL_COUNT = 20, FULL_TRIAL_COUNT = 100;
  static constexpr uint32_t VIN_CAPACITY = 256;
  static constexpr uint8_t MAX_TRIAL_COUNT = 200, CONDITION_COUNT = 20;
  static constexpr uint32_t MIN_SETTLE_US = 500000;
  static constexpr int MAX_PREPARATION_RPM = 400;

  enum Phase : uint8_t { IDLE, BASELINE, PREPARING, TRANSFER_WAIT, PROBING, POST_SPEED, DONE };
  enum Result : uint8_t {
    PENDING, COMPLETE, BASELINE_NOT_STATIONARY, PREPARATION_TARGET_NOT_REACHED,
    TRANSFER_NOT_REACHED, CURRENT_READ_FAILED, CURRENT_STALE, SPEED_READ_FAILED,
    SPEED_LIMIT, SPEED_STALE_AT_PROBE, DRIVER_FAILURE, BATTERY_GUARD,
    STORAGE_LIMIT, USER_STOP, RUN_TIMEOUT, ZERO_DEADLINE_MISSED,
    BASELINE_UNSTABLE, PROBE_SPEED_REGION_MISSED, PREPROBE_CURRENT_NOT_SETTLED,
    CURRENT_QUALITY_INVALID, COVERAGE_INCOMPLETE, SPEED_OBSERVATION_INVALID
  };
  static constexpr uint32_t SPEED_TRACE_PERIOD_US=10000, SPEED_50_LATE_LIMIT_US=2000;
  struct Trace { uint32_t time_us=0;float rpm=NAN; };
  struct Trial {
    bool is_coast=false, speed_50ms_valid=false;
    uint16_t pair_id=0;
    uint32_t event_start_us=0, coast_event_start_us=0, event_write_begin_us=0, event_write_end_us=0;
    uint32_t speed_50ms_time_us=0, speed_observation_dt_us=0;
    float speed_50ms_rpm=NAN, aligned_speed_delta_50_rpm=NAN;
    Trace speed_trace[6];uint8_t trace_count=0;

    uint8_t id = 0, condition_repeat = 0;
    int16_t target_speed_rpm = 0;
    int8_t probe_direction = 0;
    uint8_t condition_id = 0;
    int16_t requested_aligned_speed_rpm = 0;
    float baseline_current_slope_mA_s = NAN, baseline_current_spread_mA = NAN;
    float baseline_speed_median_rpm = NAN, transfer_threshold_mA = NAN;
    float current_before_probe_aligned_mA = NAN;
    uint32_t current_age_at_nonzero_end_us = UINT32_MAX;
    bool baseline_valid = false, fresh_current_valid = false, valid_probe = false;
    float transfer_last_current_mA = NAN, transfer_residual_min_mA = NAN, transfer_residual_max_mA = NAN;
    uint32_t start_us = 0, end_us = 0, baseline_begin_us = 0, baseline_end_us = 0;
    uint32_t speed_mode_enter_us = 0, speed_stable_begin_us = 0, speed_target_reached_us = 0;
    uint32_t speed_mode_exit_begin_us = 0, speed_mode_exit_end_us = 0;
    uint32_t transfer_begin_us = 0, transfer_ready_begin_us = 0, transfer_ready_us = 0;
    uint32_t rw_speed_timestamp_us = 0, current_timestamp_us = 0, bus_voltage_timestamp_us = 0;
    float baseline_current_mA = NAN, baseline_current_mad_mA = NAN;
    uint32_t baseline_current_samples = 0;
    float rw_speed_before_probe_rpm = NAN, current_before_probe_mA = NAN;
    uint16_t bus_voltage_before_probe_mV = 0;
    uint32_t rw_speed_age_at_nonzero_end_us = UINT32_MAX;
    uint32_t nonzero_write_begin_us = 0, nonzero_write_end_us = 0;
    uint32_t planned_probe_us = PROBE_US, probe_deadline_us = 0;
    uint32_t zero_write_begin_us = 0, zero_write_end_us = 0;
    uint32_t actual_probe_width_us = 0;
    int32_t zero_deadline_margin_us = INT32_MIN;
    float q_probe_60ms_on_device_mA_s = NAN;
    uint32_t active_current_samples = 0, active_current_failures = 0, active_current_max_gap_us = 0;
    uint32_t rw_speed_after_timestamp_us = 0;
    float rw_speed_after_probe_rpm = NAN, aligned_speed_delta_rpm = NAN;
    bool target_reached = false, transition_ok = false, transfer_ready = false;
    bool speed_fresh_at_probe = false, zero_deadline_met = false;
    Result result = PENDING;
  };
  struct Voltage { uint32_t begin_us = 0, time_us = 0; int32_t raw = 0; bool valid = false; };
  struct SpeedSample { uint32_t time_us = 0; double rpm = NAN; };

  bool enabled = false, started = false, finished = false, aborted = false, full = false;
  Phase phase = IDLE;
  Result reason = PENDING;
  uint32_t start_us = 0, end_us = 0, stable_begin_us = 0, last_speed_sequence = 0;
  bool stable = false;
  uint8_t index = 0, trial_count = 0, failed_trial_count = 0;
  Trial* trials=nullptr; // Allocated in PSRAM before a measurement can start.
  FixedProbeBaseline baseline;
  uint32_t order_seed = 0, rng = 1;
  uint16_t block_index=0;
  uint8_t order[CONDITION_COUNT], order_pos = CONDITION_COUNT;
  uint8_t valid_by_condition[CONDITION_COUNT] = {}, attempts_by_condition[CONDITION_COUNT] = {};
  int16_t preparation_magnitude[CONDITION_COUNT] = {};
  uint8_t valid_probe_count = 0, valid_goal_each = 1, attempt_limit_each = 2;
  Voltage voltages[VIN_CAPACITY];
  uint32_t voltage_count = 0, voltage_overflow = 0;
  SpeedSample speed_history[16];
  uint8_t speed_history_count = 0, speed_history_next = 0;
  double preparation_slope_rpm_per_s = NAN;
  double baseline_sum_mA = 0, baseline_sum_sq_mA = 0;
  uint32_t baseline_count = 0;

  void reset() {
    enabled = started = finished = aborted = full = stable = false;
    phase = IDLE; reason = PENDING; start_us = end_us = stable_begin_us = last_speed_sequence = 0;
    index = trial_count = failed_trial_count = 0; voltage_count = voltage_overflow = 0;
    clearPreparationSlope(); clearBaselineCurrent(); baseline.clear(); valid_probe_count=0;
    for(uint8_t i=0;i<CONDITION_COUNT;++i) { valid_by_condition[i]=attempts_by_condition[i]=0; }
    if(trials) for(uint16_t n=0;n<MAX_TRIAL_COUNT;++n) trials[n]=Trial{};
  }
  bool active() const { return enabled && started && !finished; }
  Trial* current() { return index < trial_count ? &trials[index] : nullptr; }
  uint32_t randomNext() { rng ^= rng<<13; rng ^= rng>>17; rng ^= rng<<5; return rng; }
  void shuffle() {
    uint8_t base[10];for(uint8_t i=0;i<10;++i)base[i]=i;
    for(uint8_t i=9;i>0;--i){const uint8_t j=randomNext()%(i+1),t=base[i];base[i]=base[j];base[j]=t;}
    for(uint8_t i=0;i<10;++i){const uint8_t first=(block_index+base[i]+(order_seed&1))&1;order[2*i]=2*base[i]+first;order[2*i+1]=2*base[i]+1-first;}
    ++block_index;order_pos=0;
  }
  static int alignedCenter(uint8_t condition) { return (int(condition/4)-2)*150; }
  bool coverageMet() const {
    for(auto n:valid_by_condition) if(n<valid_goal_each) return false;
    return true;
  }
  bool inProbeRegion(const Trial& t, float speed) const {
    const float aligned=t.probe_direction*speed;
    if(t.requested_aligned_speed_rpm==0) return fabsf(aligned)<=20;
    // Shared boundary at 225 rpm belongs to the moderate region.
    if(std::abs(t.requested_aligned_speed_rpm)==300 && fabsf(aligned)==225) return false;
    return fabsf(aligned-t.requested_aligned_speed_rpm)<=75;
  }
  void scheduleNext(uint32_t now) {
    for(uint8_t n=0;n<2*CONDITION_COUNT;++n) {
      if(order_pos>=CONDITION_COUNT) shuffle();
      const uint8_t c=order[order_pos++];
      if(valid_by_condition[c]>=valid_goal_each || attempts_by_condition[c]>=attempt_limit_each) continue;
      if(index>=MAX_TRIAL_COUNT) break;
      auto& t=trials[index]; t=Trial{};t.id=index+1;t.condition_id=c;t.is_coast=c%2;t.pair_id=(block_index-1)*10+c/2+1;
      t.condition_repeat=++attempts_by_condition[c];t.requested_aligned_speed_rpm=alignedCenter(c);
      t.probe_direction=((c/2)%2) ? -1:1;
      const int sign=t.requested_aligned_speed_rpm>0 ? 1 : t.requested_aligned_speed_rpm<0 ? -1:0;
      t.target_speed_rpm=sign*t.probe_direction*preparation_magnitude[c];
      t.start_us=now;trial_count=index+1;phase=BASELINE;stable=false;stable_begin_us=0;
      clearPreparationSlope();clearBaselineCurrent();baseline.clear();return;
    }
    finished=true;phase=DONE;end_us=now;reason=coverageMet()?COMPLETE:COVERAGE_INCOMPLETE;
  }
  void begin(bool all, uint32_t now, uint16_t run_id) {
    reset();if(!trials){reason=STORAGE_LIMIT;finished=aborted=true;return;} enabled=started=true;full=all;start_us=now;block_index=0;
    valid_goal_each=all?5:1;attempt_limit_each=all?10:2;
    order_seed=now^(uint32_t(run_id)*2654435761UL);rng=order_seed?order_seed:1;order_pos=CONDITION_COUNT;
    for(uint8_t c=0;c<CONDITION_COUNT;++c) preparation_magnitude[c]=std::abs(alignedCenter(c))==300?400:std::abs(alignedCenter(c))==150?200:0;
    scheduleNext(now);
  }
  static bool inBand(double speed, int target) {
    return isfinite(speed) && fabs(speed - target) <= fmax(20.0, 0.05 * fabs(target));
  }
  void clearPreparationSlope() { preparation_slope_rpm_per_s = NAN; speed_history_count = speed_history_next = 0; }
  double notePreparationSpeed(uint32_t time_us, double rpm) {
    bool have = false; uint32_t age = 0; double anchor = NAN;
    for (uint8_t i = 0; i < speed_history_count; ++i) {
      const uint32_t candidate_age = time_us - speed_history[i].time_us;
      if (candidate_age >= PREPARATION_SLOPE_WINDOW_US && (!have || candidate_age < age)) {
        have = true; age = candidate_age; anchor = speed_history[i].rpm;
      }
    }
    preparation_slope_rpm_per_s = have && isfinite(rpm) && isfinite(anchor)
        ? (rpm - anchor) * 1000000.0 / age : NAN;
    speed_history[speed_history_next].time_us = time_us; speed_history[speed_history_next].rpm = rpm;
    speed_history_next = (speed_history_next + 1) % 16;
    if (speed_history_count < 16) ++speed_history_count;
    return preparation_slope_rpm_per_s;
  }
  void clearBaselineCurrent() { baseline_sum_mA = baseline_sum_sq_mA = 0; baseline_count = 0; }
  void abort(Result why, uint32_t now) {
    if (finished) return;
    if (auto* t = current()) { t->result = why; t->end_us = now; }
    reason = why; finished = aborted = true; phase = DONE; end_us = now;
  }
  void beginNextTrial(uint32_t now) { ++index;scheduleNext(now); }
  void advance(uint32_t now) {
    if(auto* t=current()) {
      t->valid_probe=t->baseline_valid && t->transfer_ready && t->fresh_current_valid && t->speed_fresh_at_probe &&
        t->speed_50ms_valid && t->zero_deadline_met && t->actual_probe_width_us>50000 && t->active_current_samples>=2 &&
        t->active_current_failures==0 && t->active_current_max_gap_us<=3333;
      t->result=t->valid_probe?COMPLETE:!t->speed_50ms_valid?SPEED_OBSERVATION_INVALID:CURRENT_QUALITY_INVALID;t->end_us=now;
      if(t->valid_probe) { ++valid_by_condition[t->condition_id];++valid_probe_count; }
      else ++failed_trial_count;
    }
    beginNextTrial(now);
  }
  void failCurrentTrial(Result why, uint32_t now) {
    if(auto* t=current()) {
      t->result=why;t->end_us=now;
      if(why==PROBE_SPEED_REGION_MISSED && fabsf(t->rw_speed_before_probe_rpm)<std::abs(t->requested_aligned_speed_rpm)-75) {
        auto& prep=preparation_magnitude[t->condition_id]; if(prep>0 && prep<MAX_PREPARATION_RPM) prep+=25;
        preparation_magnitude[t->condition_id^1]=prep;
      }
    }
    if(failed_trial_count<UINT8_MAX) ++failed_trial_count;
    beginNextTrial(now);
  }
  void voltage(uint32_t begin, uint32_t end, int32_t raw, bool valid) {
    if (voltage_count < VIN_CAPACITY) { Voltage& v = voltages[voltage_count++]; v.begin_us = begin; v.time_us = end; v.raw = raw; v.valid = valid; }
    else ++voltage_overflow;
  }
  static const char* name(Result r) {
    switch (r) {
#define V57_NAME(n) case n: return #n;
      V57_NAME(PENDING) V57_NAME(COMPLETE) V57_NAME(BASELINE_NOT_STATIONARY)
      V57_NAME(PREPARATION_TARGET_NOT_REACHED) V57_NAME(TRANSFER_NOT_REACHED)
      V57_NAME(CURRENT_READ_FAILED) V57_NAME(CURRENT_STALE) V57_NAME(SPEED_READ_FAILED)
      V57_NAME(SPEED_LIMIT) V57_NAME(SPEED_STALE_AT_PROBE) V57_NAME(DRIVER_FAILURE)
      V57_NAME(BATTERY_GUARD) V57_NAME(STORAGE_LIMIT) V57_NAME(USER_STOP)
      V57_NAME(RUN_TIMEOUT) V57_NAME(ZERO_DEADLINE_MISSED)
      V57_NAME(BASELINE_UNSTABLE) V57_NAME(PROBE_SPEED_REGION_MISSED)
      V57_NAME(PREPROBE_CURRENT_NOT_SETTLED) V57_NAME(CURRENT_QUALITY_INVALID) V57_NAME(COVERAGE_INCOMPLETE) V57_NAME(SPEED_OBSERVATION_INVALID)
#undef V57_NAME
    }
    return "UNKNOWN";
  }
};