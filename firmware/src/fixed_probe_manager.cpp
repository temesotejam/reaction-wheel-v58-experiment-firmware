#include "roller485_manager.h"
#include "config.h"
#include <Wire.h>

namespace {
constexpr uint8_t REG_OUTPUT = 0x00, REG_MODE = 0x01, REG_VIN = 0x34;
constexpr uint8_t REG_SPEED = 0x40, REG_CURRENT_LIMIT = 0x50;
constexpr uint8_t REG_SPEED_READBACK = 0x60, REG_CURRENT = 0xB0;
}

bool Roller485Manager::beginFixedProbe(bool full, uint16_t run_id) {
  if (v57_acquisition_ || currentCommandActive() || q_observer_.observingPost() || !ok() ||
      !q_observer_.v57.trials || !q_observer_.recording || !q_observer_.allocated() || !q_observer_.wheel.samples) return false;
  Wire.setTimeOut(1);
  v57_acquisition_ = true; measured_q_stop_ = MeasuredQStop{};
  q_observer_.v57.begin(full, micros(), run_id);
  // Baseline, preparation, and transfer values are retained as trial metadata.
  // Preserve raw CURRENT only for pulse windows so a 50-trial run cannot
  // exhaust the fixed PSRAM observer buffer before completion.
  q_observer_.setRawSampleRecording(false);
  bool good = writeU8(REG_OUTPUT, 0);
  good = writeI32(REG_CURRENT, 0) && good;
  good = writeU8(REG_MODE, 3) && good;
  int32_t zero = -1; good = readI32(REG_CURRENT, zero) && good && zero == 0;
  if (good) good = writeU8(REG_OUTPUT, 1);  // Current Mode at 0 mA is live transfer state.
  telemetry_.mode_raw = 3; telemetry_.output_raw = 1; telemetry_.current_valid = false;
  command_mA_ = 0; last_fast_current_due_us_ = last_read_due_us_ = 0; has_wheel_attempt_ = false;
  if (!good) { abortFixedProbe(FixedProbeV57::DRIVER_FAILURE); return false; }
  return v57FreshCurrent();
}

bool Roller485Manager::endFixedProbeOutput() {
  auto& r = q_observer_.v57;
  if (v57_acquisition_ && r.phase == FixedProbeV57::PROBING && r.current() &&
      r.current()->zero_write_end_us == 0) zeroFixedProbePulse(false);
  bool good = writeU8(REG_OUTPUT, 0);
  good = writeI32(REG_CURRENT, 0) && good;
  good = writeU8(REG_MODE, 3) && good;
  v57_acquisition_ = false; command_mA_ = 0; telemetry_.output_raw = 0; telemetry_.mode_raw = 3;
  Wire.setTimeOut(Config::I2C_TIMEOUT_MS);
  if (!good) { output_write_fault_ = true; last_error_ = "v57_stop_write_failed"; }
  return good;
}

void Roller485Manager::abortFixedProbe(FixedProbeV57::Result why) {
  auto& r = q_observer_.v57;
  if (v57_acquisition_ && r.phase == FixedProbeV57::PROBING && r.current() &&
      r.current()->zero_write_end_us == 0) zeroFixedProbePulse(false);
  r.abort(why, micros());
  q_observer_.setRawSampleRecording(false);
  if (v57_acquisition_ && !endFixedProbeOutput()) {
    r.reason = FixedProbeV57::DRIVER_FAILURE;
    if (auto* t = r.current()) t->result = FixedProbeV57::DRIVER_FAILURE;
  }
}
bool Roller485Manager::failFixedProbeTrial(FixedProbeV57::Result why) {
  auto& r = q_observer_.v57;
  if (!v57_acquisition_ || !r.active() || r.phase == FixedProbeV57::PROBING) return false;

  // A reachability failure is recorded with the commanded current at zero. It
  // is not a hardware safety failure, so the remaining scheduled trials run.
  bool good = writeU8(REG_OUTPUT, 0);
  good = writeI32(REG_CURRENT, 0) && good;
  good = writeU8(REG_MODE, 3) && good;
  int32_t zero = -1;
  good = readI32(REG_CURRENT, zero) && good && zero == 0;
  if (good) good = writeU8(REG_OUTPUT, 1);
  command_mA_ = 0; telemetry_.mode_raw = 3; telemetry_.output_raw = good ? 1 : 0;
  telemetry_.current_valid = false; last_fast_current_due_us_ = last_read_due_us_ = 0;
  if (!good) { abortFixedProbe(FixedProbeV57::DRIVER_FAILURE); return false; }

  q_observer_.setRawSampleRecording(false);
  r.failCurrentTrial(why, micros());
  if (r.finished) {
    if (!endFixedProbeOutput()) { r.aborted = true; r.reason = FixedProbeV57::DRIVER_FAILURE; }
    return !r.aborted;
  }
  return v57FreshCurrent();
}

void Roller485Manager::serviceFixedProbeSafety() {
  auto& r = q_observer_.v57;
  if (!v57_acquisition_ || !r.active()) return;
  const uint32_t now = micros();
  if (!ok()) { abortFixedProbe(FixedProbeV57::DRIVER_FAILURE); return; }
  if (now - r.start_us >= FixedProbeV57::RUN_LIMIT_MS * 1000UL && r.phase != FixedProbeV57::PROBING) {
    if (auto* t=r.current()) { t->result=FixedProbeV57::RUN_TIMEOUT;t->end_us=now;++r.failed_trial_count; }
    q_observer_.setRawSampleRecording(false);
    if(!endFixedProbeOutput()) { r.abort(FixedProbeV57::DRIVER_FAILURE,micros());return; }
    r.finished=true;r.phase=FixedProbeV57::DONE;r.end_us=micros();r.reason=FixedProbeV57::COVERAGE_INCOMPLETE;return;
  }
  if (r.phase == FixedProbeV57::PREPARING && r.current() &&
      now - r.current()->speed_mode_enter_us >= FixedProbeV57::PREP_LIMIT_US) {
    failFixedProbeTrial(FixedProbeV57::PREPARATION_TARGET_NOT_REACHED); return;
  }
  if (r.phase == FixedProbeV57::TRANSFER_WAIT && r.current() &&
      now - r.current()->transfer_begin_us >= FixedProbeV57::TRANSFER_LIMIT_US) {
    failFixedProbeTrial(FixedProbeV57::TRANSFER_NOT_REACHED); return;
  }
  // The reservation gives the I2C zero write 1.5 ms to complete. This uses no
  // measured-Q branch and is the sole normal stop decision for the fixed pulse.
  if (r.phase == FixedProbeV57::PROBING && r.current() &&
      now - r.current()->event_start_us >= FixedProbeV57::PROBE_US - FixedProbeV57::ZERO_RESERVATION_US) {
    zeroFixedProbePulse(true); return;
  }
  if (telemetry_.current_valid && currentAgeUs(now) > QObserver::GAP_US) {
    abortFixedProbe(FixedProbeV57::CURRENT_STALE); return;
  }
  if (q_observer_.sample_overflow || q_observer_.pulse_overflow || q_observer_.wheel.overflow || r.voltage_overflow)
    abortFixedProbe(FixedProbeV57::STORAGE_LIMIT);
}

bool Roller485Manager::v57FreshCurrent() {
  if (!v57_acquisition_) return false;
  const uint8_t index = q_observer_.v57.index;
  const FixedProbeV57::Phase phase = q_observer_.v57.phase;
  // A normal deadline stop can advance the trial inside readCurrentFresh.
  // Yield the old call before interpreting its skipped read as an I2C failure.
  const auto context_changed = [&]() {
    const auto& r = q_observer_.v57;
    return !v57_acquisition_ || !r.active() || r.index != index || r.phase != phase;
  };
  bool good = readCurrentFresh(true);
  if (context_changed()) return false;
  // Normal trial transitions above are not read failures. Preserve the
  // existing single retry outside pulse/post windows for actual failures.
  // A real probe/post-window read failure remains terminal in readCurrentFresh().
  const bool retry_allowed = phase != FixedProbeV57::PROBING && !q_observer_.observingPost();
  for (uint8_t attempt = 1; !good && v57_acquisition_ && retry_allowed &&
                              attempt < FixedProbeV57::NON_PULSE_CURRENT_READ_ATTEMPTS; ++attempt) {
    good = readCurrentFresh(true);
    if (context_changed()) return false;
  }
  if (!good && v57_acquisition_) abortFixedProbe(FixedProbeV57::CURRENT_READ_FAILED);
  serviceFixedProbeSafety();
  const auto& r = q_observer_.v57;
  return good && v57_acquisition_ && r.active() && r.index == index && r.phase == phase;
}

bool Roller485Manager::readFixedProbeWheel(FixedProbeV57::Trial& t, bool after_probe) {
  const uint32_t begin = micros(); int32_t raw = 0;
  const bool good = readI32(REG_SPEED_READBACK, raw); const uint32_t end = micros();
  q_observer_.wheel.retain_samples = after_probe || q_observer_.v57.phase == FixedProbeV57::TRANSFER_WAIT;
  q_observer_.wheel.attempt(begin, end, raw, good);
  q_observer_.wheel.retain_samples = true; last_wheel_attempt_us_ = begin; has_wheel_attempt_ = true;
  if (!good) { abortFixedProbe(FixedProbeV57::SPEED_READ_FAILED); return false; }
  const double rpm = q_observer_.wheel.latest.rpm();
  if (fabs(rpm) > FixedProbeV57::MAX_RPM) { abortFixedProbe(FixedProbeV57::SPEED_LIMIT); return false; }
  if (after_probe) {
    t.rw_speed_after_timestamp_us = end; t.rw_speed_after_probe_rpm = rpm;
    t.aligned_speed_delta_rpm = t.probe_direction * (t.rw_speed_after_probe_rpm - t.rw_speed_before_probe_rpm);
  } else {
    t.rw_speed_timestamp_us = end; t.rw_speed_before_probe_rpm = rpm;
  }
  return true;
}

void Roller485Manager::updateFixedProbeStatus() {
  if (!v57FreshCurrent()) return;
  sampleV58Speed(); if(!q_observer_.v57.active())return;
  auto& r = q_observer_.v57;
  int32_t vin = 0; const uint32_t begin = micros(); const bool vin_ok = readI32(REG_VIN, vin); const uint32_t end = micros();

  if (!vin_ok) { abortFixedProbe(FixedProbeV57::DRIVER_FAILURE); return; }
  telemetry_.battery_mV = static_cast<uint16_t>(max<int32_t>(0, vin * 10)); telemetry_.bus_voltage_sample_time_us = end;
  if (telemetry_.battery_mV < Config::Q_IDENT_BATTERY_MIN_MV || telemetry_.battery_mV > Config::Q_IDENT_BATTERY_MAX_MV) {
    abortFixedProbe(FixedProbeV57::BATTERY_GUARD); return;
  }
  const uint8_t regs[4] = {0x01, 0x00, 0x0C, 0x0D}; uint8_t data[4] = {};
  for (uint8_t i = 0; i < 4; ++i) {
    if (!v57FreshCurrent()) return;
  sampleV58Speed(); if(!q_observer_.v57.active())return;
    if (!readU8(regs[i], data[i])) { abortFixedProbe(FixedProbeV57::DRIVER_FAILURE); return; }
  }
  telemetry_.mode_raw = data[0]; telemetry_.output_raw = data[1]; telemetry_.status_raw = data[2]; telemetry_.error_raw = data[3];
  telemetry_.status_sample_time_us = micros();
  const uint8_t expected_mode = r.phase == FixedProbeV57::PREPARING ? 1 : 3;
  if (data[0] != expected_mode || data[1] != 1 || data[3] != 0) abortFixedProbe(FixedProbeV57::DRIVER_FAILURE);
}

bool Roller485Manager::startFixedProbePreparation() {
  auto& r = q_observer_.v57; auto& t = *r.current();
  if (t.target_speed_rpm == 0) {
    t.transition_ok = true; t.transfer_begin_us = micros(); r.phase = FixedProbeV57::TRANSFER_WAIT; return true;
  }
  if (!v57FreshCurrent()) return false;
  bool good = writeU8(REG_OUTPUT, 0); good = writeI32(REG_CURRENT, 0) && good;
  if (!good) { abortFixedProbe(FixedProbeV57::DRIVER_FAILURE); return false; }
  if (!v57FreshCurrent()) return false;
  good = writeU8(REG_MODE, 1); t.speed_mode_enter_us = micros();
  uint8_t mode = 0;
  if (!good || !v57FreshCurrent() || !readU8(REG_MODE, mode) || mode != 1) { abortFixedProbe(FixedProbeV57::DRIVER_FAILURE); return false; }
  if (!v57FreshCurrent()) return false;
  good = writeI32(REG_CURRENT_LIMIT, FixedProbeV57::CURRENT_LIMIT_MA * 100);
  good = writeI32(REG_SPEED, t.target_speed_rpm * 100) && good;
  int32_t value = 0;
  if (!good || !readI32(REG_CURRENT, value) || value != 0 || !readI32(REG_CURRENT_LIMIT, value) ||
      value != FixedProbeV57::CURRENT_LIMIT_MA * 100 || !readI32(REG_SPEED, value) || value != t.target_speed_rpm * 100) {
    abortFixedProbe(FixedProbeV57::DRIVER_FAILURE); return false;
  }
  good = writeU8(REG_OUTPUT, 1);
  if (!good) { abortFixedProbe(FixedProbeV57::DRIVER_FAILURE); return false; }
  r.phase = FixedProbeV57::PREPARING; r.stable = false; r.stable_begin_us = 0; r.clearPreparationSlope();
  telemetry_.mode_raw = 1; telemetry_.output_raw = 1; return v57FreshCurrent();
}

bool Roller485Manager::switchFixedProbeToZero() {
  auto& r = q_observer_.v57; auto& t = *r.current();
  if (!v57FreshCurrent()) return false;
  t.speed_mode_exit_begin_us = micros(); bool good = writeU8(REG_MODE, 3); t.speed_mode_exit_end_us = micros();
  t.transfer_begin_us = micros(); good = writeI32(REG_CURRENT, 0) && good; const uint32_t zero_end = micros();
  t.transition_ok = good; telemetry_.mode_raw = 3;
  if (!good) { abortFixedProbe(FixedProbeV57::DRIVER_FAILURE); return false; }
  t.transfer_begin_us = zero_end; r.phase = FixedProbeV57::TRANSFER_WAIT; return v57FreshCurrent();
}

bool Roller485Manager::beginFixedProbePulse() {
  auto& r = q_observer_.v57; auto& t = *r.current();
  if(!t.baseline_valid || !t.transfer_ready) { failFixedProbeTrial(FixedProbeV57::PREPROBE_CURRENT_NOT_SETTLED);return false; }
  // Capture Vbus, then fresh CURRENT, then a forced speed read immediately
  // before CURRENT. All completion timestamps are exported without claiming
  // internal sensor sampling times.
  int32_t vin = 0; const uint32_t vin_begin = micros(); const bool vin_ok = readI32(REG_VIN, vin); const uint32_t vin_end = micros();
  r.voltage(vin_begin, vin_end, vin, vin_ok);
  if (!vin_ok) { abortFixedProbe(FixedProbeV57::DRIVER_FAILURE); return false; }
  telemetry_.battery_mV = static_cast<uint16_t>(max<int32_t>(0, vin * 10)); telemetry_.bus_voltage_sample_time_us = vin_end;
  if(telemetry_.battery_mV<Config::Q_IDENT_BATTERY_MIN_MV || telemetry_.battery_mV>Config::Q_IDENT_BATTERY_MAX_MV) {
    abortFixedProbe(FixedProbeV57::BATTERY_GUARD);return false;
  }
  // Retain the fresh pre-probe sample and every active/post sample. Baseline,
  // preparation, and transfer current remain in trial-level aggregates.
  q_observer_.setRawSampleRecording(true);
  if (!v57FreshCurrent()) return false;
  t.current_before_probe_mA = telemetry_.current_raw_mA; t.current_timestamp_us = telemetry_.current_sample_time_us;
  t.bus_voltage_before_probe_mV = telemetry_.battery_mV; t.bus_voltage_timestamp_us = vin_end;
  t.current_before_probe_aligned_mA=t.probe_direction*t.current_before_probe_mA;
  if (!readFixedProbeWheel(t, false)) return false;
  if(!r.inProbeRegion(t,t.rw_speed_before_probe_rpm)) {
    failFixedProbeTrial(FixedProbeV57::PROBE_SPEED_REGION_MISSED);return false;
  }
  const uint32_t admit=micros();
  t.fresh_current_valid=telemetry_.current_valid && uint32_t(admit-t.current_timestamp_us)<=QObserver::GAP_US;
  if(!t.fresh_current_valid || fabsf(t.current_before_probe_mA-t.baseline_current_mA)>t.transfer_threshold_mA) {
    failFixedProbeTrial(FixedProbeV57::PREPROBE_CURRENT_NOT_SETTLED);return false;
  }
  if(uint32_t(admit-t.rw_speed_timestamp_us)>FixedProbeV57::MAX_SPEED_AGE_US) {
    failFixedProbeTrial(FixedProbeV57::SPEED_STALE_AT_PROBE);return false;
  }
  // Symmetric register transaction: COAST writes zero, never a nonzero command.
  t.event_write_begin_us=micros();q_observer_.beforeNonzero(t.event_write_begin_us);
  const int32_t command=t.is_coast?0:t.probe_direction*FixedProbeV57::PROBE_CURRENT_MA;
  const bool good=writeI32(REG_CURRENT,command*100);t.event_write_end_us=micros();
  t.event_start_us=t.event_write_end_us;command_mA_=good?command:0;
  if(t.is_coast)t.coast_event_start_us=t.event_start_us;
  else {t.nonzero_write_begin_us=t.event_write_begin_us;t.nonzero_write_end_us=t.event_start_us;}
  t.current_age_at_nonzero_end_us=t.event_start_us-t.current_timestamp_us;
  t.fresh_current_valid=t.fresh_current_valid && t.current_age_at_nonzero_end_us<=QObserver::GAP_US;
  t.rw_speed_age_at_nonzero_end_us=t.event_start_us-t.rw_speed_timestamp_us;
  t.speed_fresh_at_probe=t.rw_speed_age_at_nonzero_end_us<=FixedProbeV57::MAX_SPEED_AGE_US;
  if(!good||!t.speed_fresh_at_probe||!t.fresh_current_valid){abortFixedProbe(!good?FixedProbeV57::DRIVER_FAILURE:!t.fresh_current_valid?FixedProbeV57::CURRENT_STALE:FixedProbeV57::SPEED_STALE_AT_PROBE);return false;}
  t.speed_trace[0].time_us=t.rw_speed_timestamp_us;t.speed_trace[0].rpm=t.rw_speed_before_probe_rpm;t.trace_count=1;
  q_observer_.pending=QObserver::Context{};q_observer_.pending.width_ms=60;q_observer_.pending.battery_mV=t.bus_voltage_before_probe_mV;
  // QObserver only records; for COAST its internal pulse is an observation window.
  q_observer_.begin(t.event_write_begin_us,t.event_write_begin_us,t.event_start_us,t.event_start_us,t.probe_direction,true);
  beginCurrentAuditPulse();t.probe_deadline_us=t.event_start_us+FixedProbeV57::PROBE_US;
  r.phase=FixedProbeV57::PROBING;
  return v57FreshCurrent();
}

bool Roller485Manager::zeroFixedProbePulse(bool advance) {
  auto& r = q_observer_.v57; auto* t = r.current();
  if (!t || t->zero_write_end_us) return true;
  t->zero_write_begin_us = micros(); const bool good = writeI32(REG_CURRENT, 0); t->zero_write_end_us = micros();
  command_mA_ = 0; q_observer_.stop(t->zero_write_begin_us, t->zero_write_begin_us, t->zero_write_end_us,
                                    t->zero_write_end_us, good); endCurrentAuditPulse(t->zero_write_end_us);
  t->actual_probe_width_us = t->zero_write_end_us - t->event_start_us;
  t->zero_deadline_margin_us = static_cast<int32_t>(t->probe_deadline_us - t->zero_write_end_us);
  t->zero_deadline_met = good && t->zero_deadline_margin_us >= 0;
  if (auto* p = q_observer_.currentPulse()) {
    t->q_probe_60ms_on_device_mA_s = p->measured; t->active_current_samples = p->active_samples;
    t->active_current_failures = p->active_failures; t->active_current_max_gap_us = p->active_max_gap_us;
  }
  if (!good) { abortFixedProbe(FixedProbeV57::DRIVER_FAILURE); return false; }
  if (!readFixedProbeWheel(*t, true)) return false;
  if (!t->zero_deadline_met) { abortFixedProbe(FixedProbeV57::ZERO_DEADLINE_MISSED); return false; }
  if (advance) {
    r.advance(micros());
    if (r.finished && !endFixedProbeOutput()) { r.aborted = true; r.reason = FixedProbeV57::DRIVER_FAILURE; }
  }
  return true;
}

void Roller485Manager::updateFixedProbe() {
  serviceFixedProbeSafety();
  auto& r = q_observer_.v57; if (!v57_acquisition_ || !r.active()) return;
  // Keep raw samples through QObserver's post window, then return to
  // metadata-only current collection between scheduled pulses.
  if (r.phase != FixedProbeV57::PROBING && !q_observer_.observingPost())
    q_observer_.setRawSampleRecording(false);
  auto* t = r.current(); if (!t) { abortFixedProbe(FixedProbeV57::DRIVER_FAILURE); return; }
  if (r.phase == FixedProbeV57::PROBING) { if(v57FreshCurrent()) sampleV58Speed(); return; }
  if (!v57FreshCurrent()) return;
  uint32_t now = micros();
  const uint32_t period = r.phase == FixedProbeV57::BASELINE ? FixedProbeV57::BASELINE_SPEED_PERIOD_US : FixedProbeV57::SPEED_PERIOD_US;
  if ((r.phase == FixedProbeV57::BASELINE || r.phase == FixedProbeV57::PREPARING) &&
      (!has_wheel_attempt_ || now - last_wheel_attempt_us_ >= period)) {
    if (!readFixedProbeWheel(*t, false)) return;
  }
  // Wheel reads complete after the time captured above. Refresh now so a new
  // stable_begin_us can never be subtracted from an earlier timestamp.
  now = micros();
  const auto& w = q_observer_.wheel.latest;
  const bool fresh = w.sequence != r.last_speed_sequence;
  if (fresh) r.last_speed_sequence = w.sequence;
  if (r.phase == FixedProbeV57::BASELINE) {
    // Never reuse the preceding pulse tail as the next trial's baseline.
    if(uint32_t(now-t->start_us)<FixedProbeV57::MIN_SETTLE_US) r.baseline.clear();
    else if(w.valid && uint32_t(now-w.time_us)<=2*FixedProbeV57::BASELINE_SPEED_PERIOD_US) {
      const bool accepted=r.baseline.note(telemetry_.current_sample_time_us,telemetry_.current_raw_mA,w.rpm());
      if(r.baseline.count==FixedProbeBaseline::N) {
        t->baseline_begin_us=r.baseline.begin_us;t->baseline_end_us=r.baseline.end_us;
        t->baseline_current_mA=r.baseline.median_mA;t->baseline_current_mad_mA=r.baseline.mad_mA;
        t->baseline_current_slope_mA_s=r.baseline.slope_mA_s;t->baseline_current_spread_mA=r.baseline.spread_mA;
        t->baseline_speed_median_rpm=r.baseline.speed_median;t->baseline_current_samples=r.baseline.count;
      }
      if(accepted) {
        t->baseline_valid=true;t->transfer_threshold_mA=fmaxf(3*t->baseline_current_mad_mA,FixedProbeV57::TRANSFER_TOLERANCE_MA);
        startFixedProbePreparation();return;
      }
    } else r.baseline.clear();
    if(now-t->start_us>=FixedProbeV57::SETTLE_LIMIT_US) failFixedProbeTrial(FixedProbeV57::BASELINE_UNSTABLE);
  } else if (r.phase == FixedProbeV57::PREPARING && fresh) {
    const double slope = r.notePreparationSpeed(w.time_us, w.rpm());
    const bool settled = FixedProbeV57::inBand(w.rpm(), t->target_speed_rpm) && isfinite(slope) &&
                         fabs(slope) <= FixedProbeV57::PREPARATION_MAX_SLOPE_RPM_PER_S;
    if (!settled) { r.stable = false; r.stable_begin_us = 0; }
    else if (!r.stable) { r.stable = true; r.stable_begin_us = w.time_us; t->speed_stable_begin_us = w.time_us; }
    if (r.stable && w.time_us - r.stable_begin_us >= FixedProbeV57::HOLD_US) {
      t->target_reached = true; t->speed_target_reached_us = w.time_us; switchFixedProbeToZero();
    }
  } else if (r.phase == FixedProbeV57::TRANSFER_WAIT) {
    // Threshold derives only from an independently accepted quiet baseline.
    t->transfer_last_current_mA=telemetry_.current_raw_mA;
    const float residual=telemetry_.current_raw_mA-t->baseline_current_mA;
    t->transfer_residual_min_mA=isfinite(t->transfer_residual_min_mA)?fminf(t->transfer_residual_min_mA,residual):residual;
    t->transfer_residual_max_mA=isfinite(t->transfer_residual_max_mA)?fmaxf(t->transfer_residual_max_mA,residual):residual;
    if (isfinite(t->baseline_current_mA) && isfinite(telemetry_.current_raw_mA) &&
        fabsf(telemetry_.current_raw_mA - t->baseline_current_mA) <= t->transfer_threshold_mA) {
      if (!t->transfer_ready_begin_us) t->transfer_ready_begin_us = telemetry_.current_sample_time_us;
      if (telemetry_.current_sample_time_us - t->transfer_ready_begin_us >= FixedProbeV57::TRANSFER_CONTINUOUS_US) {
        t->transfer_ready = true; t->transfer_ready_us = telemetry_.current_sample_time_us; beginFixedProbePulse();
      }
    } else t->transfer_ready_begin_us = 0;
  }
}
void Roller485Manager::sampleV58Speed() {
  auto& r=q_observer_.v57;auto* t=r.current();
  if(!v57_acquisition_||r.phase!=FixedProbeV57::PROBING||!t||t->trace_count>=6)return;
  const uint8_t slot=t->trace_count;
  if(uint32_t(micros()-t->event_start_us)<slot*FixedProbeV57::SPEED_TRACE_PERIOD_US)return;
  // Caller just completed a fresh current read and its Q update.
  const uint32_t begin=micros();int32_t raw=0;const bool good=readI32(REG_SPEED_READBACK,raw);const uint32_t end=micros();
  q_observer_.wheel.attempt(begin,end,raw,good);
  if(!good){abortFixedProbe(FixedProbeV57::SPEED_READ_FAILED);return;}
  const float rpm=raw/100.0f;if(fabsf(rpm)>FixedProbeV57::MAX_RPM){abortFixedProbe(FixedProbeV57::SPEED_LIMIT);return;}
  t->speed_trace[slot].time_us=end;t->speed_trace[slot].rpm=rpm;++t->trace_count;
  if(slot==5){
    t->speed_50ms_time_us=end;t->speed_50ms_rpm=rpm;t->speed_observation_dt_us=end-t->rw_speed_timestamp_us;
    t->aligned_speed_delta_50_rpm=t->probe_direction*(rpm-t->rw_speed_before_probe_rpm);
    t->speed_50ms_valid=uint32_t(end-t->event_start_us)>=50000 && uint32_t(end-t->event_start_us)<=50000+FixedProbeV57::SPEED_50_LATE_LIMIT_US;
  }
  // Account for the wheel transaction immediately in the current-gap audit.
  v57FreshCurrent();
}
