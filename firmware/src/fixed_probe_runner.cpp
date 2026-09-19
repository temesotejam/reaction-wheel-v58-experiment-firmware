#include "experiment_runner.h"

bool ExperimentRunner::startFixedProbeCapture(bool full) {
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

  if (!roller_->ok() || !roller_->qObserver().allocated() || !roller_->qObserver().wheel.samples || !roller_->qObserver().v57.trials) {
    status_.last_error = "v57_driver_or_storage_not_ready"; return false;
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
  status_.remaining_ms = FixedProbeV57::RUN_LIMIT_MS;
  status_.trial_index = 1;
  status_.trial_count = full ? FixedProbeV57::FULL_TRIAL_COUNT : FixedProbeV57::SIMPLE_TRIAL_COUNT;
  status_.trial_elapsed_ms = 0;
  status_.trial_duration_ms = FixedProbeV57::RUN_LIMIT_MS;
  status_.current_mA_setting = 0;
  status_.pulse_width_ms_setting = 0;
  status_.input_interval_ms = 0;
  status_.beta_hold_after_input_ms_setting = 0;
  status_.beta_recovery_tau_s_setting = 0.0f;
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.sync_event_id = 0;
  fixed_probe_full_=full;
  beginStartSync(millis(), false, true);
  return true;
}
void ExperimentRunner::serviceFixedProbeFast() {
  if(!roller_||!imu_||!logger_)return;
  auto&r=roller_->qObserver().v57;
  if(status_.emergency_stop||!imu_->ok()||imu_->stale(millis())) {
    roller_->abortFixedProbe(FixedProbeV57::USER_STOP);
    requestEmergencyStop("v57_safety_or_imu_fault");return;
  }
  if(!roller_->ok()) {
    roller_->abortFixedProbe(FixedProbeV57::DRIVER_FAILURE);
    requestEmergencyStop("v57_driver_fault");return;
  }
  roller_->serviceMeasuredQStop();
  if(r.aborted)requestEmergencyStop(FixedProbeV57::name(r.reason));
}
void ExperimentRunner::updateFixedProbeRun() {
  serviceFixedProbeFast();
  if(status_.state!=ExperimentState::RUNNING_BATCH_SWEEP)return;
  roller_->updateFixedProbe();
  auto&r=roller_->qObserver().v57;
  if(r.aborted){requestEmergencyStop(FixedProbeV57::name(r.reason));return;}
  status_.trial_index=r.index<r.trial_count?r.index+1:r.trial_count;
  status_.trial_count=r.trial_count;
  if(r.finished){finishRun();return;}
}

