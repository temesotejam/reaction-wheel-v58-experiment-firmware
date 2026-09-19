#include "roller485_manager.h"

#include <Wire.h>
#include <new>

#include "config.h"

static constexpr uint8_t REG_OUTPUT = 0x00;
static constexpr uint8_t REG_MODE = 0x01;
static constexpr uint8_t REG_SYS_STATUS = 0x0C;
static constexpr uint8_t REG_ERROR_CODE = 0x0D;
static constexpr uint8_t REG_VIN = 0x34;
static constexpr uint8_t REG_CURRENT = 0xB0;
static constexpr uint8_t REG_CURRENT_READBACK = 0xC0;
static constexpr uint8_t REG_SPEED_READBACK = 0x60;

bool Roller485Manager::begin() {
  q_observer_.wheel.samples = static_cast<WheelSample*>(ps_malloc(sizeof(WheelSample) * WheelObserver::CAPACITY));
  q_observer_.samples = static_cast<QObserver::Sample*>(ps_malloc(sizeof(QObserver::Sample) * QObserver::SAMPLE_CAPACITY));
  q_observer_.pulses = static_cast<QObserver::Pulse*>(ps_malloc(sizeof(QObserver::Pulse) * QObserver::PULSE_CAPACITY));
  q_observer_.v57.trials=static_cast<FixedProbeV57::Trial*>(ps_malloc(sizeof(FixedProbeV57::Trial)*FixedProbeV57::MAX_TRIAL_COUNT));
  if(q_observer_.v57.trials) for(unsigned n=0;n<FixedProbeV57::MAX_TRIAL_COUNT;++n) new(&q_observer_.v57.trials[n]) FixedProbeV57::Trial{};
  q_observer_.raw_per_mA = Config::ROLLER_CURRENT_RAW_PER_MA;
  Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN);
  Wire.setClock(Config::I2C_HZ);
  Wire.setTimeOut(Config::I2C_TIMEOUT_MS);

  Wire.beginTransmission(Config::ROLLER_ADDR);
  const bool present = Wire.endTransmission() == 0;
  if (!present) {
    telemetry_.roller_ok = false;
    last_error_ = "roller_not_found";
    return false;
  }

  bool ok = true;
  ok &= writeU8(REG_MODE, Config::ROLLER_MODE_CURRENT);
  ok &= writeI32(REG_CURRENT, 0);
  ok &= writeU8(REG_OUTPUT, 0);
  telemetry_.roller_ok = ok;
  telemetry_.mode_raw = Config::ROLLER_MODE_CURRENT;
  telemetry_.output_raw = 0;
  command_mA_ = 0;
  last_error_ = ok ? "" : "roller_zero_failed";
  return ok;
}

void Roller485Manager::update() {
  updatePulseCurrent();
  updateFullStatus();
  updateWheelIdle();
}

void Roller485Manager::updateWheelIdle() {
  // V52 active and post-current schedules stay untouched. Shared Wire timeout
  // is 20ms: an optional speed transaction must never block an active pulse.
  if (currentObservationPriority() || deadlineCritical()) return;
  const uint32_t now = micros();
  if (has_wheel_attempt_ && static_cast<uint32_t>(now - last_wheel_attempt_us_) < WheelObserver::PERIOD_US) return;
  last_wheel_attempt_us_ = now;
  has_wheel_attempt_ = true;
  int32_t raw = 0;
  const bool valid = readI32(REG_SPEED_READBACK, raw);
  q_observer_.wheel.attempt(now, micros(), raw, valid);
  // A diagnostic failure is visible in its own counters, not a new motor gate.
  // Existing current/status/write faults remain handled by their original paths.
}

bool Roller485Manager::readWheelForV55() {
  if (currentCommandActive()) return false;
  const uint32_t begin = micros(); int32_t raw = 0;
  const bool valid = readI32(REG_SPEED_READBACK, raw);
  q_observer_.wheel.attempt(begin, micros(), raw, valid);
  last_wheel_attempt_us_ = begin; has_wheel_attempt_ = true;
  return valid;
}
void Roller485Manager::captureWheelAfterV55() {
  auto* p = q_observer_.currentPulse();
  if (!p || !p->context.fixed_duration_v55 || p->phase != 2 ||
      p->wheel_after.sequence || currentCommandActive() || !telemetry_.current_valid ||
      telemetry_.current_sequence == p->at_stop.sequence) return;
  // First post-stop current is already saved to bracket the zero boundary.
  readWheelForV55();
  p->wheel_after = q_observer_.wheel.latest;
}
void Roller485Manager::updatePulseCurrent() {
  updatePulseCurrentInternal(false);
}

bool Roller485Manager::updatePulseCurrentInternal(bool force_now) {
  const bool was_commanded = currentCommandActive();
  serviceMeasuredQStop();
  // No trailing read in the call that has just forced CURRENT=0.
  if (was_commanded && !currentCommandActive()) return false;
  const uint32_t now_us = micros();

  // V47 keeps CURRENT_READBACK independent of the six-register full status
  // snapshot so the main loop can run this first during a pulse. Its result is
  // V51 checks measured Q immediately at every successful read completion.
  if (currentObservationPriority() &&
      (force_now || last_fast_current_due_us_ == 0 ||
       static_cast<uint32_t>(now_us - last_fast_current_due_us_) >=
           Config::CURRENT_AUDIT_FAST_READ_PERIOD_US)) {
    last_fast_current_due_us_ = now_us;
    const uint32_t fast_current_start_us = micros();
    const bool ok = readCurrentFresh(true);
    if (ok) captureWheelAfterV55();
    if (timing_audit_) timing_audit_->fast_current_read_us = micros() - fast_current_start_us;
    return ok;
  }
  return false;
}

void Roller485Manager::servicePulseCurrentBeforeImu() {
  if (!currentObservationPriority()) return;
  if (current_audit_active_ &&
      telemetry_.current_audit_imu_deadline_guard_count < UINT16_MAX) {
    ++telemetry_.current_audit_imu_deadline_guard_count;
  }
  if (current_timing_audit_.active) ++current_timing_audit_.current.pre_imu_reads;
  updatePulseCurrentInternal(true);
}

void Roller485Manager::servicePulseCurrentAfterImu() {
  if (!currentObservationPriority()) return;
  if (current_audit_active_ && telemetry_.current_audit_post_imu_service_count < UINT16_MAX) {
    ++telemetry_.current_audit_post_imu_service_count;
  }
  if (current_timing_audit_.active) ++current_timing_audit_.current.post_imu_reads;
  updatePulseCurrentInternal(true);
}

void Roller485Manager::servicePulseCurrentBeforeStatus() {
  if (!currentObservationPriority()) return;
  if (current_timing_audit_.active) ++current_timing_audit_.current.pre_status_reads;
  updatePulseCurrentInternal(true);
}

void Roller485Manager::servicePulseCurrentAfterStatus() {
  if (!currentObservationPriority()) return;
  if (current_timing_audit_.active) ++current_timing_audit_.current.post_status_reads;
  updatePulseCurrentInternal(true);
}

bool Roller485Manager::fullStatusDue(uint32_t now_us) const {
  return last_read_due_us_ == 0 || static_cast<uint32_t>(now_us - last_read_due_us_) >=
      Config::ROLLER_READ_PERIOD_MS * 1000UL;
}

void Roller485Manager::updateFullStatus() {
  if (deadlineCritical()) { serviceMeasuredQStop(); return; }
  const uint32_t now_us = micros();
  const uint32_t period_us = Config::ROLLER_READ_PERIOD_MS * 1000UL;
  if (last_read_due_us_ != 0 && static_cast<uint32_t>(now_us - last_read_due_us_) < period_us) return;
  last_read_due_us_ = last_read_due_us_ == 0 ? now_us : last_read_due_us_ + period_us;

  if(v57_acquisition_){updateFixedProbeStatus();return;}
  const uint32_t full_status_start_us = micros();
  if (timing_audit_) {
    timing_audit_->roller_full_status_start_us = full_status_start_us;
    timing_audit_->roller_full_status_sequence = ++timing_full_status_sequence_;
  }
  int32_t vin_raw = 0;
  uint8_t mode = 0;
  uint8_t output = 0;
  uint8_t status = 0;
  uint8_t error = 0;

  // During output the caller already acquired CURRENT_READBACK and serviced
  // safety immediately before this due status task. Idle behavior is unchanged.
  bool ok = currentObservationPriority() ? telemetry_.current_valid : readCurrentFresh(false);
  const bool v55_active = currentCommandActive() && measured_q_stop_.record.fixed_duration;
  auto continueV55Status = [this, v55_active, &ok]() {
    if (!v55_active) return true;
    if (!ok) { recordIo(false); stop(MeasuredQStop::DRIVER_FAULT); return false; }
    serviceMeasuredQStop();
    return currentCommandActive() && !deadlineCritical();
  };
  if (!continueV55Status()) return;
  ok &= readI32(REG_VIN, vin_raw);
  if (!continueV55Status()) return;
  ok &= readU8(REG_MODE, mode);
  if (!continueV55Status()) return;
  ok &= readU8(REG_OUTPUT, output);
  if (!continueV55Status()) return;
  ok &= readU8(REG_SYS_STATUS, status);
  if (!continueV55Status()) return;
  ok &= readU8(REG_ERROR_CODE, error);
  if (!continueV55Status()) return;
  recordIo(ok);
  current_timing_audit_.noteTask(CurrentTimingAudit::FULL_STATUS, micros() - full_status_start_us);
  if (timing_audit_) timing_audit_->roller_full_status_us = micros() - full_status_start_us;
  if (!ok) return;

  telemetry_.battery_mV = static_cast<uint16_t>(max<int32_t>(0, vin_raw * 10));
  telemetry_.mode_raw = mode;
  telemetry_.output_raw = output;
  telemetry_.status_raw = status;
  telemetry_.error_raw = error;
  telemetry_.status_sample_time_us = micros();
  telemetry_.roller_ok = error == 0;
  last_error_ = error == 0 ? "" : "roller_error_raw";
}

bool Roller485Manager::setCurrentMa(int16_t current_mA) {
  if(v57_acquisition_) {
    if(current_mA!=0){last_error_="v57_no_current_probe";return false;}
    abortFixedProbe(FixedProbeV57::USER_STOP);
    return !output_write_fault_;
  }
  const bool fixed_v55 = current_mA != 0 && q_observer_.pending.fixed_duration_v55;
  v55_start_result_ = WheelProbeV55::PENDING;
  if (fixed_v55) {
    const auto& c = q_observer_.pending;
    const bool width_ok = c.v55_role == 2 ? c.width_ms == WheelProbeV55::PROBE_MS
        : c.v55_role == 1 && (c.width_ms == 5 || c.width_ms == 20);
    if (!width_ok || (current_mA != 300 && current_mA != -300) ||
        !q_observer_.v55.enabled || !q_observer_.v55.started || q_observer_.v55.finished ||
        (c.v55_role == 2 && current_mA != c.v55_probe_direction * 300) || !q_observer_.allocated() || !q_observer_.wheel.samples ||
        !q_observer_.recording || currentObservationPriority() || !ok() ||
        q_observer_.sample_overflow || q_observer_.pulse_overflow || q_observer_.wheel.overflow ||
        q_observer_.pulse_count >= QObserver::PULSE_CAPACITY ||
        q_observer_.sample_count + 150 >= QObserver::SAMPLE_CAPACITY) {
      q_observer_.pending = QObserver::Context{};
      v55_start_result_ = WheelProbeV55::STORAGE_LIMIT;
      last_error_ = "v55_invalid_context_state_or_storage"; return false;
    }
    // Initial current -> MODE -> fresh speed -> immediately CURRENT.
    if (!readCurrentFresh(false)) {
      q_observer_.pending = QObserver::Context{};
      v55_start_result_ = WheelProbeV55::CURRENT_FAILURE; return false;
    }
    if (!writeU8(REG_MODE, Config::ROLLER_MODE_CURRENT)) {
      q_observer_.pending = QObserver::Context{}; output_write_fault_ = true;
      v55_start_result_ = WheelProbeV55::DRIVER_FAILURE; return false;
    }
    if (!readWheelForV55()) v55_start_result_ = WheelProbeV55::SPEED_READ_FAILED;
    else if (fabs(q_observer_.wheel.latest.rpm()) > WheelProbeV55::MAX_INITIAL_RPM)
      v55_start_result_ = WheelProbeV55::SPEED_LIMIT;
    else if (c.v55_role == 2 && !WheelProbeV55::inBand(q_observer_.wheel.latest.rpm(),
        c.v55_probe_direction, c.v55_target_aligned_rpm))
      v55_start_result_ = WheelProbeV55::SPEED_OUTSIDE_BAND;
    if (v55_start_result_ != WheelProbeV55::PENDING) {
      q_observer_.pending = QObserver::Context{};
      last_error_ = WheelProbeV55::name(v55_start_result_); return false;
    }
  }
  if (fixed_v55) { Wire.setTimeOut(1); v55_bus_timeout_active_ = true; }
  // A normal V51 pulse requires its independently supplied, valid Q context.
  // Reject before any nonzero write; never fall back to planned-width control.
  if (current_mA != 0 && q_observer_.pending.measured_stop_enabled &&
      (!isfinite(q_observer_.pending.target) || q_observer_.pending.target <= 0 ||
       q_observer_.pending.width_ms == 0 ||
       q_observer_.pending.width_ms > Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS ||
       !q_observer_.allocated() || !q_observer_.recording ||
       q_observer_.pulse_count >= QObserver::PULSE_CAPACITY)) {
    q_observer_.pending = QObserver::Context{};
    last_error_ = "invalid_measured_q_context_or_storage";
    return false;
  }
  const uint32_t command_us = micros();
  const bool was_commanded = command_mA_ != 0;
  command_mA_ = current_mA;
  const int32_t raw = static_cast<int32_t>(current_mA) * Config::ROLLER_CURRENT_RAW_PER_MA;
  bool ok = true;
  // CURRENT=0 is the first bus transaction when stopping; mode was set at start.
  if (current_mA != 0 && !fixed_v55) ok &= writeU8(REG_MODE, Config::ROLLER_MODE_CURRENT);
  const uint32_t write_begin_us = micros();
  ok &= writeI32(REG_CURRENT, raw);
  const uint32_t write_end_us = micros();
  ok &= writeU8(REG_OUTPUT, current_mA == 0 ? 0 : 1);
  const uint32_t output_end_us = micros();
  if (current_mA == 0 && v55_bus_timeout_active_) {
    Wire.setTimeOut(Config::I2C_TIMEOUT_MS); v55_bus_timeout_active_ = false;
  }
  // No reads occur within this synchronous write sequence. Finalize the prior
  // post window at the captured begin without adding observer work inside it.
  if (current_mA != 0) q_observer_.beforeNonzero(write_begin_us);
  if (current_mA != 0) {
    const auto context = q_observer_.pending;
    measured_q_stop_.begin(write_end_us, current_mA > 0 ? 1 : -1,
        context.target, context.width_ms, Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS,
        context.measured_stop_enabled);
    if (fixed_v55) measured_q_stop_.beginFixed(write_end_us,
        current_mA > 0 ? 1 : -1, context.width_ms);
    q_observer_.begin(command_us, write_begin_us, write_end_us,
        output_end_us, current_mA > 0 ? 1 : -1, ok);
    if (auto* p = q_observer_.currentPulse()) {
      if (p->id > 1) {
        const auto& previous = q_observer_.pulses[p->id - 2];
        p->previous_direction = previous.direction;
        p->consecutive_same_direction = previous.direction == p->direction
            ? previous.consecutive_same_direction + 1 : 1;
      } else p->consecutive_same_direction = 1;
    }
  } else {
    measured_q_stop_.stop(MeasuredQStop::SAFETY_STOP, command_us);
    q_observer_.stop(command_us, write_begin_us, write_end_us, output_end_us, ok);
    if (was_commanded) {
      if (!ok) measured_q_stop_.record.reason = MeasuredQStop::DRIVER_FAULT;
      if (auto* p = q_observer_.currentPulse()) p->stopping = measured_q_stop_.record;
    }
  }
  recordIo(ok);
  if (!ok) {
    output_write_fault_ = true; // An unconfirmed actuator write is not cleared by reads.
    measured_q_stop_.stop(MeasuredQStop::DRIVER_FAULT, micros());
    last_error_ = "roller_current_write_failed";
    return false;
  }
  if (!was_commanded && current_mA != 0) beginCurrentAuditPulse();
  else if (was_commanded && current_mA == 0) endCurrentAuditPulse(micros());
  if (fixed_v55 && static_cast<uint32_t>(write_end_us - q_observer_.wheel.latest.time_us) > WheelProbeV55::MAX_AGE_US) {
    stop(MeasuredQStop::SAFETY_STOP);
    v55_start_result_ = WheelProbeV55::SPEED_AGE_EXCEEDED;
    last_error_ = "v55_speed_age_exceeded"; return false;
  }
  return true;
}

bool Roller485Manager::stop(MeasuredQStop::Reason reason) {
  measured_q_stop_.stop(reason, micros());
  return setCurrentMa(0);
}

void Roller485Manager::applyMeasuredQStop() {
  if (currentCommandActive() && measured_q_stop_.record.enabled &&
      measured_q_stop_.record.reason != MeasuredQStop::NONE) setCurrentMa(0);
}

void Roller485Manager::serviceMeasuredQStop() {
  if(v57_acquisition_){serviceFixedProbeSafety();return;}
  measured_q_stop_.poll(micros(), ok());
  applyMeasuredQStop();
}

uint32_t Roller485Manager::currentAgeUs(uint32_t now_us) const {
  if (telemetry_.current_sample_time_us == 0) return UINT32_MAX;
  return static_cast<uint32_t>(now_us - telemetry_.current_sample_time_us);
}

bool Roller485Manager::readCurrentFresh(bool audit_sample) {
  // Recheck at the actual bus entry, not only at scheduler admission.
  const bool was_commanded = currentCommandActive();
  serviceMeasuredQStop();
  if (was_commanded && !currentCommandActive()) return false;
  int32_t current_raw = 0;
  ++telemetry_.current_sequence;
  const uint32_t read_start_us = micros();
  if (!readI32(REG_CURRENT_READBACK, current_raw)) {
    const uint32_t failed_us = micros();
    q_observer_.sample(failed_us, telemetry_.current_sequence, 0, failed_us - read_start_us, false);
    recordCurrentReadFailure(audit_sample);
    // Outside the V57 raw-current pulse/post window, the V57 wrapper makes
    // one immediate retry. While pulse evidence is being collected, retain
    // the strict stop: a missing sample must not be hidden by interpolation.
    if (v57_acquisition_ && (q_observer_.v57.phase == FixedProbeV57::PROBING ||
                             q_observer_.observingPost())) {
      abortFixedProbe(FixedProbeV57::CURRENT_READ_FAILED);
    }
    measured_q_stop_.sample(failed_us, 0, false);
    applyMeasuredQStop();
    return false;
  }
  const uint32_t completed_us = micros();
  current_read_duration_us_ = completed_us - read_start_us;
  q_observer_.sample(completed_us, telemetry_.current_sequence, current_raw, current_read_duration_us_, true);
  recordFreshCurrent(current_raw, completed_us, audit_sample);
  measured_q_stop_.sample(completed_us,
      static_cast<float>(current_raw) / Config::ROLLER_CURRENT_RAW_PER_MA, true);
  applyMeasuredQStop(); // Before IMU, status, Web or any return to the scheduler.
  return true;
}

void Roller485Manager::recordFreshCurrent(int32_t current_raw, uint32_t sample_time_us,
                                          bool audit_sample) {
  telemetry_.current_raw_mA=static_cast<float>(current_raw)/Config::ROLLER_CURRENT_RAW_PER_MA;
  telemetry_.actual_current_mA = static_cast<int16_t>(
      current_raw / Config::ROLLER_CURRENT_RAW_PER_MA);
  telemetry_.current_sample_time_us = sample_time_us;
  telemetry_.current_valid = true;
  if (!audit_sample || !current_audit_active_) return;
  current_timing_audit_.success(sample_time_us, current_read_duration_us_);

  const int16_t current_mA = telemetry_.actual_current_mA;
  if (current_audit_has_previous_sample_) {
    const uint32_t dt_us = static_cast<uint32_t>(sample_time_us - current_audit_previous_sample_us_);
    if (dt_us > 0) {
      recordCurrentAuditGap(current_audit_previous_sample_us_, sample_time_us, dt_us);
    }
    if (dt_us > 0 && dt_us <= 50000UL && isfinite(telemetry_.q_meas_observed_mA_s)) {
      telemetry_.q_meas_observed_mA_s += 0.5f *
          (fabsf(static_cast<float>(current_audit_previous_mA_)) + fabsf(static_cast<float>(current_mA))) *
          static_cast<float>(dt_us) * 1.0e-6f;
    }
  } else {
    telemetry_.q_meas_observed_mA_s = 0.0f;
    current_audit_has_previous_sample_ = true;
  }
  current_audit_previous_mA_ = current_mA;
  current_audit_previous_sample_us_ = sample_time_us;
  if (telemetry_.current_audit_sample_count < UINT16_MAX) {
    ++telemetry_.current_audit_sample_count;
  }
  telemetry_.q_meas_observed_valid = telemetry_.current_audit_sample_count >= 2 &&
      !current_audit_read_failed_ && isfinite(telemetry_.q_meas_observed_mA_s);
}

void Roller485Manager::recordCurrentReadFailure(bool audit_sample) {
  telemetry_.current_valid = false;
  ++telemetry_.current_read_failure_count;
  if (audit_sample && current_audit_active_) {
    current_timing_audit_.failedRead();
    current_audit_read_failed_ = true;
    telemetry_.q_meas_observed_valid = false;
  }
}

void Roller485Manager::beginCurrentAuditPulse() {
  current_timing_audit_.begin(micros());
  current_audit_active_ = true;
  current_audit_has_previous_sample_ = false;
  current_audit_read_failed_ = false;
  current_audit_previous_mA_ = 0;
  current_audit_previous_sample_us_ = 0;
  last_fast_current_due_us_ = 0;
  telemetry_.current_sample_time_us = 0;
  telemetry_.current_audit_sample_count = 0;
  telemetry_.q_meas_observed_mA_s = NAN;
  telemetry_.current_valid = false;
  telemetry_.q_meas_observed_valid = false;
  telemetry_.current_audit_max_gap_us = 0;
  telemetry_.current_audit_gt_3333_count = 0;
  telemetry_.current_audit_interval_count = 0;
  telemetry_.current_audit_tail_max_gap_us = 0;
  telemetry_.current_audit_tail_gt_3333_count = 0;
  telemetry_.current_audit_tail_interval_count = 0;
  telemetry_.current_audit_pulse_end_current_age_us = UINT32_MAX;
  telemetry_.current_audit_imu_deadline_guard_count = 0;
  telemetry_.current_audit_post_imu_service_count = 0;
  telemetry_.current_audit_finalized = false;
  telemetry_.current_audit_tail_covered = false;
  current_audit_gap_history_head_ = 0;
  current_audit_gap_history_count_ = 0;
}

void Roller485Manager::endCurrentAuditPulse(uint32_t pulse_end_us) {
  finalizeCurrentAuditTail(pulse_end_us);
  current_timing_audit_.end(pulse_end_us);
  // Existing binary fields now use the strict V49 coverage audit too. Detailed
  // completed records live in metadata and cannot be lost by row decimation.
  const auto& audit = current_timing_audit_.current;
  telemetry_.current_audit_tail_max_gap_us = audit.tail_current_gap_max_us;
  telemetry_.current_audit_tail_gt_3333_count = audit.tail_current_gap_gt3333_count;
  telemetry_.current_audit_tail_interval_count = audit.tail_interval_count;
  telemetry_.current_audit_pulse_end_current_age_us = audit.pulse_end_current_age_us;
  telemetry_.current_audit_tail_covered = audit.tail_covered;
  current_audit_active_ = false;
}

void Roller485Manager::recordCurrentAuditGap(uint32_t previous_sample_us,
                                              uint32_t sample_time_us, uint32_t gap_us) {
  if (gap_us > telemetry_.current_audit_max_gap_us) {
    telemetry_.current_audit_max_gap_us = gap_us;
  }
  if (gap_us > Config::CURRENT_AUDIT_MAX_GAP_US &&
      telemetry_.current_audit_gt_3333_count < UINT16_MAX) {
    ++telemetry_.current_audit_gt_3333_count;
  }
  if (telemetry_.current_audit_interval_count < UINT16_MAX) {
    ++telemetry_.current_audit_interval_count;
  }
  current_audit_gap_history_[current_audit_gap_history_head_] = {
      previous_sample_us, sample_time_us, gap_us};
  current_audit_gap_history_head_ =
      (current_audit_gap_history_head_ + 1) % CURRENT_AUDIT_GAP_HISTORY_CAPACITY;
  if (current_audit_gap_history_count_ < CURRENT_AUDIT_GAP_HISTORY_CAPACITY) {
    ++current_audit_gap_history_count_;
  }
}

void Roller485Manager::finalizeCurrentAuditTail(uint32_t pulse_end_us) {
  telemetry_.current_audit_tail_max_gap_us = 0;
  telemetry_.current_audit_tail_gt_3333_count = 0;
  telemetry_.current_audit_tail_interval_count = 0;
  telemetry_.current_audit_pulse_end_current_age_us = currentAgeUs(pulse_end_us);
  telemetry_.current_audit_tail_covered = false;
  const uint32_t tail_start_us = pulse_end_us - Config::CURRENT_AUDIT_TAIL_WINDOW_US;
  const uint8_t oldest = (current_audit_gap_history_head_ + CURRENT_AUDIT_GAP_HISTORY_CAPACITY -
                          current_audit_gap_history_count_) % CURRENT_AUDIT_GAP_HISTORY_CAPACITY;
  for (uint8_t offset = 0; offset < current_audit_gap_history_count_; ++offset) {
    const CurrentAuditGap& gap = current_audit_gap_history_[
        (oldest + offset) % CURRENT_AUDIT_GAP_HISTORY_CAPACITY];
    if (static_cast<int32_t>(gap.previous_sample_us - pulse_end_us) >= 0 ||
        static_cast<int32_t>(gap.sample_time_us - tail_start_us) <= 0) continue;
    telemetry_.current_audit_tail_covered = true;
    if (gap.gap_us > telemetry_.current_audit_tail_max_gap_us) {
      telemetry_.current_audit_tail_max_gap_us = gap.gap_us;
    }
    if (gap.gap_us > Config::CURRENT_AUDIT_MAX_GAP_US &&
        telemetry_.current_audit_tail_gt_3333_count < UINT16_MAX) {
      ++telemetry_.current_audit_tail_gt_3333_count;
    }
    if (telemetry_.current_audit_tail_interval_count < UINT16_MAX) {
      ++telemetry_.current_audit_tail_interval_count;
    }
  }
  telemetry_.current_audit_finalized = true;
}

bool Roller485Manager::writeU8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(Config::ROLLER_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool Roller485Manager::writeI32(uint8_t reg, int32_t value) {
  uint8_t* p = reinterpret_cast<uint8_t*>(&value);
  Wire.beginTransmission(Config::ROLLER_ADDR);
  Wire.write(reg);
  Wire.write(p, 4);
  return Wire.endTransmission() == 0;
}

bool Roller485Manager::readBytes(uint8_t reg, uint8_t* buffer, size_t len) {
  Wire.beginTransmission(Config::ROLLER_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    memset(buffer, 0, len);
    return false;
  }
  const uint8_t got = Wire.requestFrom(Config::ROLLER_ADDR, static_cast<uint8_t>(len));
  if (got != len) {
    memset(buffer, 0, len);
    while (Wire.available()) Wire.read();
    return false;
  }
  for (size_t i = 0; i < len; ++i) buffer[i] = static_cast<uint8_t>(Wire.read());
  return true;
}

bool Roller485Manager::readI32(uint8_t reg, int32_t& value) {
  value = 0;
  return readBytes(reg, reinterpret_cast<uint8_t*>(&value), 4);
}

bool Roller485Manager::readU8(uint8_t reg, uint8_t& value) {
  value = 0;
  return readBytes(reg, &value, 1);
}

void Roller485Manager::recordIo(bool ok) {
  if (ok) {
    telemetry_.consecutive_errors = 0;
    telemetry_.roller_ok = true;
    return;
  }
  telemetry_.i2c_error_count++;
  if (telemetry_.consecutive_errors < 255) telemetry_.consecutive_errors++;
  telemetry_.roller_ok = false;
}
