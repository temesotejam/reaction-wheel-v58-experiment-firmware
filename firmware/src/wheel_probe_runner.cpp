#include "experiment_runner.h"

bool ExperimentRunner::startWheelProbeCapture() {
  if (!logger_ || !imu_ || !roller_) return false;
  if (!logger_->ready()) {
    status_.last_error = "psram_not_ready";
    return false;
  }
  if (!imu_->ok()) {
    status_.last_error = "imu_not_ready";
    return false;
  }
  if (status_.state != ExperimentState::READY_TO_MEASURE && status_.state != ExperimentState::FINISHED) {
    status_.last_error = "not_ready";
    return false;
  }

  if (!roller_->ok() || !roller_->qObserver().allocated() || !roller_->qObserver().wheel.samples) {
    status_.last_error = "v55_driver_or_storage_not_ready"; return false;
  }
  energy_control_v0_mode_ = energy_control_autonomous_mode_ = false;
  energy_control_v0_pulse_authorized_ = energy_control_autonomous_pulse_authorized_ = false;
  // Dedicated preparation/probe path; no manual-release gate or start kick.
  q1_shadow_run_target_peak_abs_deg_ = q1_shadow_target_peak_abs_deg_;
  q_ident_mode_ = false;
  q_ident_pulse_authorized_ = false;
  resetQIdentTracker();
  resetQ1ShadowZeroCrossTracker();
  passive_capture_mode_ = false;
  zero_cross_mode_ = false;
  single_trial_mode_ = true;
  selected_trial_index_ = 0;
  identification_mode_ = false;
  q_run_mode_ = QRunMode::NONE;
  q_probe_schedule_id_ = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A;
  control_target_peak_deg_ = 0.0f;
  zero_cross_fixed_current_mA_ = 0;
  zero_cross_fixed_pulse_ms_ = 0;
  stopMotor();
  status_.run_id++;
  if (status_.run_id == 0) status_.run_id = 1;
  last_log_us_ = 0;
  last_pulse_end_ms_ = 0;
  status_.pulse_id = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  status_.motor_cmd_mA = 0;
  status_.measure_elapsed_ms = 0;
  status_.remaining_ms = WheelProbeV55::RUN_LIMIT_MS;
  status_.trial_index = 1;
  status_.trial_count = 1;
  status_.trial_elapsed_ms = 0;
  status_.trial_duration_ms = WheelProbeV55::RUN_LIMIT_MS;
  status_.current_mA_setting = 0;
  status_.pulse_width_ms_setting = 0;
  status_.input_interval_ms = 0;
  status_.beta_hold_after_input_ms_setting = 0;
  status_.beta_recovery_tau_s_setting = 0.0f;
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.sync_event_id = 0;
  beginStartSync(millis(), true);
  return true;
}


void ExperimentRunner::serviceWheelProbeFast() {
  if (!roller_ || !imu_ || !logger_) return;
  auto& o = roller_->qObserver(); auto& run = o.v55;
  if (status_.emergency_stop || !roller_->ok() || !imu_->ok() || imu_->stale(millis())) {
    if (!run.finished) run.abort(!roller_->ok() ? WheelProbeV55::DRIVER_FAILURE :
        WheelProbeV55::USER_STOP, micros());
    requestEmergencyStop("v55_safety_or_sensor_fault"); return;
  }
  if (static_cast<uint32_t>(millis() - trial_start_ms_) >= WheelProbeV55::RUN_LIMIT_MS) {
    run.abort(WheelProbeV55::RUN_TIMEOUT, micros()); finishRun(); return;
  }
  if (o.sample_overflow || o.pulse_overflow || o.wheel.overflow) {
    run.abort(WheelProbeV55::STORAGE_LIMIT, micros());
    requestEmergencyStop("v55_storage_overflow"); return;
  }
  roller_->serviceMeasuredQStop();
  if (status_.pulse_active && !roller_->currentCommandActive()) {
    status_.pulse_active = false; status_.motor_cmd_mA = 0; status_.pulse_direction = 0;
    last_pulse_end_ms_ = millis();
    const auto why = roller_->measuredQStopReason();
    if (why != MeasuredQStop::FIXED_PROBE_COMPLETE) {
      run.abort(why == MeasuredQStop::DRIVER_FAULT ? WheelProbeV55::DRIVER_FAILURE :
          WheelProbeV55::CURRENT_FAILURE, micros());
      requestEmergencyStop(MeasuredQStop::name(why));
    }
  }
}

void ExperimentRunner::updateWheelProbe() {
  serviceWheelProbeFast();
  if (status_.state != ExperimentState::RUNNING_BATCH_SWEEP) return;
  auto& o = roller_->qObserver(); auto& run = o.v55;
  const uint32_t now = micros();
  if (run.finished) { finishRun(); return; }
  if (wheel_probe_waiting_) {
    if (roller_->currentObservationPriority()) return;
    auto& p = o.pulses[wheel_probe_pulse_id_ - 1];
    if (!p.wheel_after.valid) {
      // Preserve the emitted probe plus explicit failure; no silent retry.
      run.advance(WheelProbeV55::SPEED_READ_FAILED, now);
    } else if (fabs(p.wheel_after.rpm()) > WheelProbeV55::MAX_INITIAL_RPM) {
      run.abort(WheelProbeV55::SPEED_LIMIT, now);
    } else if (p.context.v55_role == 2) run.advance(WheelProbeV55::COMPLETE, now);
    wheel_probe_waiting_ = false;
    wheel_probe_next_us_ = now + WheelProbeV55::REST_US;
    return;
  }
  if (!MeasuredQStop::due(now, wheel_probe_next_us_)) return;
  auto* trial = run.current();
  if (!trial) { finishRun(); return; }
  if (now - trial->start_us > WheelProbeV55::TRIAL_LIMIT_US) {
    run.advance(WheelProbeV55::TRIAL_TIMEOUT, now); return;
  }
  if (roller_->currentObservationPriority()) return;
  const auto& telem = roller_->telemetry();
  if (telem.battery_mV < Config::Q_IDENT_BATTERY_MIN_MV ||
      telem.battery_mV > Config::Q_IDENT_BATTERY_MAX_MV) {
    run.abort(WheelProbeV55::DRIVER_FAILURE, now);
    requestEmergencyStop("v55_battery_guard"); return;
  }
  if (!roller_->readWheelForV55()) {
    run.advance(WheelProbeV55::SPEED_READ_FAILED, micros()); return;
  }
  const float rpm = o.wheel.latest.rpm();
  if (fabsf(rpm) > WheelProbeV55::MAX_INITIAL_RPM) {
    run.abort(WheelProbeV55::SPEED_LIMIT, micros()); finishRun(); return;
  }
  const bool probe = WheelProbeV55::inBand(rpm, trial->direction, trial->target_aligned_rpm);
  if (!probe && trial->preparation_count >= WheelProbeV55::MAX_PREP) {
    run.advance(WheelProbeV55::PREPARATION_LIMIT, micros()); return;
  }
  QObserver::Context c;
  c.fixed_duration_v55 = true; c.v55_role = probe ? 2 : 1; c.v55_trial_id = trial->id;
  c.v55_probe_direction = trial->direction; c.v55_target_aligned_rpm = trial->target_aligned_rpm;
  c.width_ms = probe ? WheelProbeV55::PROBE_MS :
      WheelProbeV55::preparationWidth(rpm, trial->direction, trial->target_aligned_rpm);
  c.battery_mV = telem.battery_mV;
  c.v55_status_time_us = telem.status_sample_time_us;
  c.v55_driver_mode = telem.mode_raw; c.v55_driver_status = telem.status_raw;
  c.v55_driver_error = telem.error_raw;
  const int8_t d = probe ? trial->direction :
      WheelProbeV55::preparationDirection(rpm, trial->direction, trial->target_aligned_rpm);
  o.pending = c;
  const uint32_t previous_count = o.pulse_count;
  if (!roller_->setCurrentMa(d * WheelProbeV55::CURRENT_MA)) {
    const auto reason = roller_->v55StartResult();
    if (o.pulse_count > previous_count && probe) trial->probe_pulse_id = o.pulse_count;
    if (o.pulse_count == previous_count && (reason == WheelProbeV55::SPEED_READ_FAILED ||
        reason == WheelProbeV55::SPEED_OUTSIDE_BAND)) run.advance(reason, micros());
    else {
      run.abort(reason == WheelProbeV55::PENDING ? WheelProbeV55::DRIVER_FAILURE : reason, micros());
      requestEmergencyStop("v55_pulse_start_failed");
    }
    return;
  }
  wheel_probe_waiting_ = true; wheel_probe_pulse_id_ = o.currentPulse()->id;
  if (probe) trial->probe_pulse_id = wheel_probe_pulse_id_;
  else ++trial->preparation_count;
  status_.pulse_id = wheel_probe_pulse_id_; status_.pulse_active = true;
  status_.pulse_direction = d; status_.motor_cmd_mA = d * WheelProbeV55::CURRENT_MA;
  status_.current_mA_setting = WheelProbeV55::CURRENT_MA; status_.pulse_width_ms_setting = c.width_ms;
  active_pulse_start_ms_ = millis(); active_pulse_start_test_ms_ = millis() - run_start_ms_;
}
