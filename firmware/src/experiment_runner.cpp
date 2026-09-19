#include "experiment_runner.h"

#include <math.h>

namespace {
bool isSelectableZeroCrossCurrent(int16_t current_mA) {
  return current_mA >= Config::ZERO_CROSS_CURRENT_MIN_MA &&
         current_mA <= Config::ZERO_CROSS_CURRENT_MAX_MA;
}

bool isSelectableZeroCrossPulseMs(uint16_t pulse_width_ms) {
  return pulse_width_ms >= Config::ZERO_CROSS_TIME_SWEEP_MIN_PULSE_MS &&
         pulse_width_ms <= Config::ZERO_CROSS_TIME_SWEEP_MAX_PULSE_MS;
}

int8_t signedSide(float value) {
  return value > 0.0f ? 1 : (value < 0.0f ? -1 : 0);
}}  // namespace

void ExperimentRunner::begin(PsramLogger& logger, ImuManager& imu, Roller485Manager& roller) {
  logger_ = &logger;
  imu_ = &imu;
  roller_ = &roller;
  boot_start_ms_ = millis();
  calib_start_ms_ = boot_start_ms_;
  beginFilters();
  pinMode(Config::SYNC_LED_PIN, OUTPUT);
  setSyncLed(false);
  stopMotor();
  status_.state = ExperimentState::STARTUP_GYRO_CALIB;
}

void ExperimentRunner::beginFilters() {
  constexpr float imu_hz = 1000.0f / Config::IMU_PERIOD_MS;
  filter_beta1_raw_.begin(imu_hz);
  filter_beta1_bias_.begin(imu_hz);
  filter_beta1_raw_.setBeta(Config::MADGWICK_BETA_ONE);
  filter_beta1_bias_.setBeta(Config::MADGWICK_BETA_ONE);
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    filter_dynamic_raw_[i].begin(imu_hz);
    filter_dynamic_bias_[i].begin(imu_hz);
    beta_smooth_[i] = betaCeilingForStrategy(i);
    filter_dynamic_raw_[i].setBeta(beta_smooth_[i]);
    filter_dynamic_bias_[i].setBeta(beta_smooth_[i]);
    status_.beta_target_series[i] = beta_smooth_[i];
    status_.beta_smooth_series[i] = beta_smooth_[i];
  }
  status_.beta_target = beta_smooth_[Config::FILTER_ADOPTED_INDEX];
  status_.beta_smooth = beta_smooth_[Config::FILTER_ADOPTED_INDEX];
}

void ExperimentRunner::serviceFast() {
  if(fixed_probe_mode_ && status_.state==ExperimentState::RUNNING_BATCH_SWEEP) {
    serviceFixedProbeFast(); return;
  }
  if (wheel_probe_mode_ && status_.state == ExperimentState::RUNNING_BATCH_SWEEP) {
    serviceWheelProbeFast(); return;
  }
  // Actual output is permitted only for one explicit Energy Control path.
  const bool energy_control_v0_pulse_live = energy_control_v0_mode_ &&
      energy_control_v0_pulse_authorized_ && status_.state == ExperimentState::RUNNING_BATCH_SWEEP &&
      !status_.emergency_stop && status_.pulse_active;
  const bool energy_control_autonomous_pulse_live = energy_control_autonomous_mode_ &&
      energy_control_autonomous_pulse_authorized_ && status_.state == ExperimentState::RUNNING_BATCH_SWEEP &&
      !status_.emergency_stop && status_.pulse_active;
  if (!energy_control_v0_pulse_live && !energy_control_autonomous_pulse_live) stopMotor();
  if (energy_control_autonomous_pulse_live && roller_ && roller_->measuredQStopEnabled()) {
    // Keep the existing experiment-end condition live while heavy updates defer.
    if (roller_->deadlineCritical() &&
        static_cast<uint32_t>(millis() - trial_start_ms_) >= status_.trial_duration_ms) {
      finishRun(); // CURRENT=0 precedes end logging.
      return;
    }
    roller_->serviceMeasuredQStop();
    updateEnergyControlAutonomousPulse(millis());
  }
}

void ExperimentRunner::updateImuDynamicBetaContext() {
  const uint32_t now_ms = millis();
  uint32_t since_last_pulse_ms = 65535;
  if (last_pulse_end_ms_ != 0) since_last_pulse_ms = now_ms - last_pulse_end_ms_;
  if (imu_) {
    imu_->setDynamicBetaContext(status_.pulse_active, since_last_pulse_ms,
                                status_.state != ExperimentState::RUNNING_BATCH_SWEEP);
    imu_->setAutonomousExperimentActive(autonomousExperimentRunning());
  }
}

void ExperimentRunner::update() {
  if (!logger_ || !imu_ || !roller_) return;
  const uint32_t now_ms = millis();
  status_.boot_elapsed_ms = now_ms - boot_start_ms_;
  status_.roller_actual_current_mA = roller_->telemetry().actual_current_mA;
  status_.roller_battery_mV = roller_->telemetry().battery_mV;

  const ImuReading& r = imu_->reading();
  if (r.last_update_us != 0 && r.last_update_us != last_imu_update_us_) {
    const uint32_t filter_start_us = micros();
    if (timing_audit_) {
      timing_audit_->runner_filter_start_us = filter_start_us;
      timing_audit_->runner_filter_sequence = ++timing_runner_filter_sequence_;
    }
    updateFilterSeries(r);
    if (timing_audit_) timing_audit_->update_filter_series_us = micros() - filter_start_us;

    const uint32_t displayed_angles_start_us = micros();
    updateDisplayedAngles(r);
    if (timing_audit_) timing_audit_->update_displayed_angles_us = micros() - displayed_angles_start_us;

    if (status_.state == ExperimentState::STARTUP_GYRO_CALIB) updateStartupCalibration(r);

    const uint32_t current_roll_start_us = micros();
    updateCurrentRollState(r, now_ms);
    if (timing_audit_) timing_audit_->update_current_roll_us = micros() - current_roll_start_us;

    if ((passive_capture_mode_ || q_ident_mode_ || energy_control_v0_mode_ || energy_control_autonomous_mode_) && status_.state == ExperimentState::RUNNING_BATCH_SWEEP) {
      const uint32_t q1_shadow_start_us = micros();
      updateQ1ShadowAtZeroCross(now_ms);
      if (timing_audit_) timing_audit_->q1_shadow_us = micros() - q1_shadow_start_us;
      if (energy_control_autonomous_mode_) {
        const uint32_t autonomous_motion_start_us = micros();
        updateEnergyControlAutonomousMotion(now_ms);
        if (timing_audit_) timing_audit_->autonomous_motion_us = micros() - autonomous_motion_start_us;
      }
    }
    last_imu_update_us_ = r.last_update_us;
  }

  if (status_.state == ExperimentState::MADGWICK_SETTLING &&
      static_cast<uint32_t>(now_ms - settling_start_ms_) >= Config::MADGWICK_SETTLING_MS) {
    status_.state = ExperimentState::READY_TO_MEASURE;
  }

  if (status_.state == ExperimentState::START_SYNC) {
    status_.measure_elapsed_ms = 0;
    status_.remaining_ms = measurementTotalDurationMs();
    updateStartSync(now_ms);
    logSampleIfDue();
  } else if (status_.state == ExperimentState::RUNNING_BATCH_SWEEP) {
    status_.measure_elapsed_ms = now_ms - run_start_ms_;
    status_.remaining_ms = status_.measure_elapsed_ms >= measurementTotalDurationMs()
                                ? 0
                                : measurementTotalDurationMs() - status_.measure_elapsed_ms;
    status_.trial_elapsed_ms = now_ms - trial_start_ms_;
    updateMidSyncLed(now_ms);
    if(fixed_probe_mode_) {
      updateFixedProbeRun();
    } else if (wheel_probe_mode_) {
      updateWheelProbe();
    } else if (passive_capture_mode_) {
      // The passive recording path has no pulse, Q, rebuild, or cooldown action.
      stopMotor();
    } else if (energy_control_autonomous_mode_) {
      updateEnergyControlAutonomousPulse(now_ms);
    } else if (energy_control_v0_mode_) {
      // Sole V0 actual-output route. It cannot enter the legacy scheduler.
      updateEnergyControlV0Pulse(now_ms);
    } else if (q_ident_mode_) {
      // Q_IDENT is frozen; retain only a fail-closed status path.
      updateQIdentPulse(now_ms);
    } else {
      updateInputPulse(now_ms);
    }
    logSampleIfDue();
    if (status_.trial_elapsed_ms >= status_.trial_duration_ms) {
      if (passive_capture_mode_ || q_ident_mode_ || energy_control_v0_mode_ || energy_control_autonomous_mode_) finishRun();
      else finishTrial(now_ms);
    }
  } else if (status_.state == ExperimentState::TRIAL_REST) {
    status_.measure_elapsed_ms = now_ms - run_start_ms_;
    status_.remaining_ms = status_.measure_elapsed_ms >= measurementTotalDurationMs()
                                ? 0
                                : measurementTotalDurationMs() - status_.measure_elapsed_ms;
    status_.trial_elapsed_ms = 0;
    updateMidSyncLed(now_ms);
    logSampleIfDue();
    updateTrialRest(now_ms);
  } else if (status_.state == ExperimentState::END_SYNC) {
    if(!fixed_probe_mode_) status_.measure_elapsed_ms = measurementTotalDurationMs();
    status_.remaining_ms = 0;
    updateEndSync(now_ms);
    logSampleIfDue();
  } else {
    setSyncLed(false);
    stopMotor();
  }
}

void ExperimentRunner::updateFilterSeries(const ImuReading& r) {
  const float dt_s = r.update_dt_us > 0 ? static_cast<float>(r.update_dt_us) / 1000000.0f
                                        : static_cast<float>(Config::IMU_PERIOD_MS) / 1000.0f;
  const float pitch_rate = Config::GYRO_PITCH_RATE_SIGN * r.gy_dps;
  const float gx_bias = bias_ready_ ? status_.gyro_bias_x_dps : 0.0f;
  const float gy_bias = bias_ready_ ? status_.gyro_bias_y_dps : 0.0f;
  const float gz_bias = bias_ready_ ? status_.gyro_bias_z_dps : 0.0f;
  // HOLD_073 is the adopted control/observation series. The beta1 and all
  // other parallel Madgwick series are held during an Autonomous run to keep
  // the current-observation path free of comparison computation.
  const bool hold_comparison_madgwick = Config::AUTONOMOUS_SKIP_COMPARISON_MADGWICK &&
      (energy_control_autonomous_mode_ || wheel_probe_mode_ || fixed_probe_mode_) && status_.state == ExperimentState::RUNNING_BATCH_SWEEP;
  if (!hold_comparison_madgwick) {
    const uint32_t beta1_filters_start_us = micros();
    filter_beta1_raw_.setBeta(Config::MADGWICK_BETA_ONE);
    filter_beta1_bias_.setBeta(Config::MADGWICK_BETA_ONE);
    filter_beta1_raw_.updateIMU(r.gx_dps, r.gy_dps, r.gz_dps, r.ax_g, r.ay_g, r.az_g);
    filter_beta1_bias_.updateIMU(r.gx_dps - gx_bias, r.gy_dps - gy_bias, r.gz_dps - gz_bias, r.ax_g, r.ay_g, r.az_g);
    raw_beta1_raw_pitch_deg_ = Config::PITCH_SIGN * filter_beta1_raw_.getPitch();
    raw_beta1_bias_pitch_deg_ = Config::PITCH_SIGN * filter_beta1_bias_.getPitch();
    if (timing_audit_) timing_audit_->runner_beta1_filters_us = micros() - beta1_filters_start_us;
  }
  const bool adopted_hold_after_input =
      !status_.pulse_active && last_pulse_end_ms_ != 0 &&
      static_cast<uint32_t>(millis() - last_pulse_end_ms_) <
          betaHoldAfterInputMsForStrategy(Config::FILTER_ADOPTED_INDEX);
  const bool input_protected = status_.pulse_active || adopted_hold_after_input;
  const float phase_rate_dps = pitch_rate - pitchBiasFromGyroBias();
  if (!hold_comparison_madgwick) {
    if (Config::BETA_PHASE_RETURN_TEST_ENABLED) beta_phase_.update(phase_rate_dps, dt_s, input_protected);
    const uint32_t beta_turn_fast_start_us = micros();
    beta_turn_fast_.update(phase_rate_dps, dt_s, input_protected);
    if (timing_audit_) timing_audit_->beta_turn_fast_us = micros() - beta_turn_fast_start_us;
  }
  const uint32_t dynamic_all_start_us = micros();
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    const float beta_ceiling = betaCeilingForStrategy(i);
    float beta_target = beta_ceiling;
    if (i == Config::FILTER_FIXED_B100_INDEX || i == Config::FILTER_FIXED_B000_INDEX) {
      beta_smooth_[i] = betaCeilingForStrategy(i);
    } else if (i == Config::FILTER_DYNAMIC_TURN_FAST_INDEX) {
      beta_target = beta_turn_fast_.betaTarget(betaFloorForStrategy(i), beta_ceiling);
      beta_smooth_[i] = beta_target;
    } else {
      const uint16_t hold_after_input_ms = betaHoldAfterInputMsForStrategy(i);
      const bool hold_after_input =
          !status_.pulse_active && last_pulse_end_ms_ != 0 &&
          static_cast<uint32_t>(millis() - last_pulse_end_ms_) < hold_after_input_ms;
      if (status_.pulse_active || hold_after_input) {
        beta_target = betaFloorForStrategy(i);
        beta_smooth_[i] = beta_target;
      } else if (beta_smooth_[i] > beta_ceiling) {
        beta_smooth_[i] = beta_ceiling;
      } else if (beta_smooth_[i] < beta_ceiling) {
        // After the input floor, use the configured quadratic soft-start followed by
        // the configured linear ramp. Dynamic strategies are capped at beta_ceiling.
        const uint32_t elapsed_after_input_ms = static_cast<uint32_t>(millis() - last_pulse_end_ms_);
        const float recovery_ms = static_cast<float>(elapsed_after_input_ms - hold_after_input_ms);
        const float beta_floor = betaFloorForStrategy(i);
        // Scale the ramp so the reduced ceiling is reached at the same time that
        // the prior beta=0.100 profile would have reached its ceiling.
        const float reference_span = Config::BETA_TIME_MATCH_REFERENCE_MAX - beta_floor;
        const float target_span = beta_ceiling - beta_floor;
        const float recovery_rate_per_ms =
            (reference_span > 0.0f && target_span > 0.0f)
                ? Config::BETA_TIME_MATCH_REFERENCE_RECOVERY_PER_MS * target_span / reference_span
                : 0.0f;
        if (recovery_ms < static_cast<float>(Config::BETA_SOFT_START_MS)) {
          beta_target = beta_floor + recovery_rate_per_ms * recovery_ms * recovery_ms /
                                         (2.0f * static_cast<float>(Config::BETA_SOFT_START_MS));
        } else {
          beta_target = beta_floor + recovery_rate_per_ms *
                                         (recovery_ms - 0.5f * static_cast<float>(Config::BETA_SOFT_START_MS));
        }
        if (beta_target > beta_ceiling) beta_target = beta_ceiling;
        beta_smooth_[i] = beta_target;
      }
    }
    if (i == Config::FILTER_ADOPTED_INDEX && Config::BETA_PHASE_RETURN_TEST_ENABLED) {
      const float phase_ceiling = beta_phase_.betaCeiling(betaFloorForStrategy(i), beta_ceiling);
      if (beta_target > phase_ceiling) beta_target = phase_ceiling;
      beta_smooth_[i] = beta_target;
    }
    const bool advance_series = !hold_comparison_madgwick || i == Config::FILTER_ADOPTED_INDEX;
    if (advance_series) {
      filter_dynamic_raw_[i].setBeta(beta_smooth_[i]);
      filter_dynamic_bias_[i].setBeta(beta_smooth_[i]);
      filter_dynamic_raw_[i].updateIMU(r.gx_dps, r.gy_dps, r.gz_dps, r.ax_g, r.ay_g, r.az_g);
      const uint32_t adopted_filter_start_us =
          i == Config::FILTER_ADOPTED_INDEX ? micros() : 0;
      filter_dynamic_bias_[i].updateIMU(r.gx_dps - gx_bias, r.gy_dps - gy_bias, r.gz_dps - gz_bias, r.ax_g, r.ay_g,
                                         r.az_g);
      if (timing_audit_ && i == Config::FILTER_ADOPTED_INDEX) {
        timing_audit_->adopted_filter_only_us = micros() - adopted_filter_start_us;
      }
      raw_dynamic_raw_pitch_deg_[i] = Config::PITCH_SIGN * filter_dynamic_raw_[i].getPitch();
      raw_dynamic_bias_pitch_deg_[i] = Config::PITCH_SIGN * filter_dynamic_bias_[i].getPitch();
    }
    status_.beta_target_series[i] = beta_target;
    status_.beta_smooth_series[i] = beta_smooth_[i];
  }
  if (timing_audit_) timing_audit_->runner_dynamic_all_us = micros() - dynamic_all_start_us;
  raw_accel_pitch_deg_ = accelPitchDeg(r);

  if (dt_s > 0.0f && dt_s < 1.0f) {
    gyro_raw_deg_ += pitch_rate * dt_s;
    gyro_bias_corrected_deg_ += (pitch_rate - pitchBiasFromGyroBias()) * dt_s;
  }

  status_.gyro_pitch_rate_dps = pitch_rate;
  status_.beta_target = status_.beta_target_series[Config::FILTER_ADOPTED_INDEX];
  status_.beta_smooth = status_.beta_smooth_series[Config::FILTER_ADOPTED_INDEX];
  status_.beta_phase_state = static_cast<uint8_t>(beta_turn_fast_.state());
  status_.beta_phase_progress = beta_turn_fast_.recoveryProgress();
  status_.beta_phase_peak_angle_deg = beta_turn_fast_.peakAbsAngleDeg();
  status_.beta_phase_angle_deg = beta_turn_fast_.phaseAngleDeg();
  status_.beta_phase_ceiling = beta_turn_fast_.betaTarget(
      betaFloorForStrategy(Config::FILTER_DYNAMIC_TURN_FAST_INDEX),
      betaCeilingForStrategy(Config::FILTER_DYNAMIC_TURN_FAST_INDEX));
  status_.ax_g = r.ax_g;
  status_.ay_g = r.ay_g;
  status_.az_g = r.az_g;
  status_.gx_dps = r.gx_dps;
  status_.gy_dps = r.gy_dps;
  status_.gz_dps = r.gz_dps;
  status_.acc_norm_g = r.acc_norm_g;
}

void ExperimentRunner::updateStartupCalibration(const ImuReading& r) {
  bias_sum_x_ += r.gx_dps;
  bias_sum_y_ += r.gy_dps;
  bias_sum_z_ += r.gz_dps;
  bias_sample_count_++;
  status_.calibration_sample_count = bias_sample_count_;

  if (static_cast<uint32_t>(millis() - calib_start_ms_) < Config::STARTUP_GYRO_CALIB_MS) return;
  if (bias_sample_count_ > 0) {
    status_.gyro_bias_x_dps = static_cast<float>(bias_sum_x_ / bias_sample_count_);
    status_.gyro_bias_y_dps = static_cast<float>(bias_sum_y_ / bias_sample_count_);
    status_.gyro_bias_z_dps = static_cast<float>(bias_sum_z_ / bias_sample_count_);
  }
  status_.gyro_bias_pitch_dps = pitchBiasFromGyroBias();
  bias_ready_ = true;
  settling_start_ms_ = millis();
  status_.state = ExperimentState::MADGWICK_SETTLING;
}

void ExperimentRunner::updateDisplayedAngles(const ImuReading&) {
  if (passive_capture_mode_) {
    // Preserve the continuous IMU gravity frame: no start-of-run subtraction.
    status_.pitch_madgwick_beta1_raw_deg = raw_beta1_raw_pitch_deg_;
    status_.pitch_madgwick_beta1_bias_deg = raw_beta1_bias_pitch_deg_;
    for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
      status_.pitch_dynamic_beta_deg[i] = raw_dynamic_bias_pitch_deg_[i];
    }
    status_.pitch_madgwick_dynamic_raw_deg = raw_dynamic_raw_pitch_deg_[Config::FILTER_ADOPTED_INDEX];
    status_.pitch_madgwick_dynamic_bias_deg = raw_dynamic_bias_pitch_deg_[Config::FILTER_ADOPTED_INDEX];
    status_.pitch_accel_only_deg = raw_accel_pitch_deg_;
  } else {
    status_.pitch_madgwick_beta1_raw_deg = raw_beta1_raw_pitch_deg_ - offset_beta1_raw_deg_;
    status_.pitch_madgwick_beta1_bias_deg = raw_beta1_bias_pitch_deg_ - offset_beta1_bias_deg_;
    for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
      status_.pitch_dynamic_beta_deg[i] = raw_dynamic_bias_pitch_deg_[i] - offset_dynamic_bias_deg_[i];
    }
    status_.pitch_madgwick_dynamic_raw_deg =
        raw_dynamic_raw_pitch_deg_[Config::FILTER_ADOPTED_INDEX] -
        offset_dynamic_raw_deg_[Config::FILTER_ADOPTED_INDEX];
    status_.pitch_madgwick_dynamic_bias_deg = status_.pitch_dynamic_beta_deg[Config::FILTER_ADOPTED_INDEX];
    status_.pitch_accel_only_deg = raw_accel_pitch_deg_ - offset_accel_deg_;
  }
  status_.pitch_gyro_raw_deg = gyro_raw_deg_;
  status_.pitch_gyro_bias_corrected_deg = gyro_bias_corrected_deg_;
}

void ExperimentRunner::updateCurrentRollState(const ImuReading& r, uint32_t now_ms) {
  status_.physical_roll_candidate_deg = physicalRollCandidateDeg(r);
  status_.physical_roll_abs_deg =
      Config::PHYSICAL_ROLL_AFFINE_SLOPE * status_.physical_roll_candidate_deg +
      Config::PHYSICAL_ROLL_AFFINE_OFFSET_DEG;
  status_.physical_roll_rate_raw_dps = r.gy_dps;
  status_.physical_roll_rate_dps =
      bias_ready_ ? r.gy_dps - status_.gyro_bias_y_dps : 0.0f;
  status_.display_zero_offset_deg = display_zero_offset_deg_;
  status_.current_roll_deg = status_.physical_roll_abs_deg - display_zero_offset_deg_;
  status_.target_roll_deg = target_roll_deg_;
  status_.target_error_deg = status_.current_roll_deg - target_roll_deg_;

  const bool rate_is_static = bias_ready_ &&
      fabsf(status_.physical_roll_rate_dps) <= Config::STATIC_RATE_THRESHOLD_DPS;
  if (!rate_is_static) {
    static_rate_since_ms_ = 0;
    status_.static_confirmed = false;
  } else {
    if (static_rate_since_ms_ == 0) static_rate_since_ms_ = now_ms;
    status_.static_confirmed =
        static_cast<uint32_t>(now_ms - static_rate_since_ms_) >= Config::STATIC_HOLD_TIME_MS;
  }
  status_.ready = status_.static_confirmed &&
      fabsf(status_.target_error_deg) <= Config::TARGET_TOLERANCE_DEG;
}

bool ExperimentRunner::startBatchSweepTest() {
  energy_control_v0_mode_ = false;
  energy_control_autonomous_mode_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  energy_control_v0_pulse_authorized_ = false;
  zero_cross_mode_ = false;
  single_trial_mode_ = false;
  passive_capture_mode_ = false;
  selected_trial_index_ = 0;
  if (!logger_ || !imu_ || !roller_) return false;
  if (!logger_->ready()) {
    status_.last_error = "psram_not_ready";
    return false;
  }
  if (!imu_->ok()) {
    status_.last_error = "imu_not_ready";
    return false;
  }
  if (status_.state != ExperimentState::READY_TO_MEASURE && status_.state != ExperimentState::FINISHED &&
      status_.state != ExperimentState::ESTOP) {
    status_.last_error = "not_ready";
    return false;
  }
  stopMotor();
  status_.run_id++;
  if (status_.run_id == 0) status_.run_id = 1;
  last_log_us_ = 0;
  last_pulse_end_ms_ = 0;
  status_.pulse_id = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  status_.measure_elapsed_ms = 0;
  status_.remaining_ms = measurementTotalDurationMs();
  status_.trial_index = 0;
  status_.trial_count = Config::BETA_SWEEP_TRIAL_COUNT;
  status_.trial_elapsed_ms = 0;
  status_.trial_duration_ms = Config::BETA_SWEEP_TRIAL_DURATION_MS;
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.sync_event_id = 0;
  beginStartSync(millis());
  return true;
}

bool ExperimentRunner::startSingleTrialTest(uint8_t trial_number) {
  energy_control_v0_mode_ = false;
  energy_control_autonomous_mode_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  energy_control_v0_pulse_authorized_ = false;
  zero_cross_mode_ = false;
  passive_capture_mode_ = false;
  if (!logger_ || !imu_ || !roller_) return false;
  if (trial_number < 1 || trial_number > Config::BETA_SWEEP_TRIAL_COUNT) {
    status_.last_error = "bad_trial";
    return false;
  }
  if (!logger_->ready()) {
    status_.last_error = "psram_not_ready";
    return false;
  }
  if (!imu_->ok()) {
    status_.last_error = "imu_not_ready";
    return false;
  }
  if (status_.state != ExperimentState::READY_TO_MEASURE && status_.state != ExperimentState::FINISHED &&
      status_.state != ExperimentState::ESTOP) {
    status_.last_error = "not_ready";
    return false;
  }

  single_trial_mode_ = true;
  selected_trial_index_ = trial_number - 1;
  const auto& trial = Config::BETA_SWEEP_TRIALS[selected_trial_index_];

  stopMotor();
  status_.run_id++;
  if (status_.run_id == 0) status_.run_id = 1;
  last_log_us_ = 0;
  last_pulse_end_ms_ = 0;
  status_.pulse_id = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  status_.measure_elapsed_ms = 0;
  status_.trial_index = trial_number;
  status_.trial_count = Config::BETA_SWEEP_TRIAL_COUNT;
  status_.trial_elapsed_ms = 0;
  status_.trial_duration_ms = trial.duration_ms;
  status_.current_mA_setting = trial.current_mA;
  status_.pulse_width_ms_setting = trial.pulse_width_ms;
  status_.input_interval_ms = trial.input_interval_ms;
  status_.beta_hold_after_input_ms_setting = Config::BETA_HOLD_AFTER_INPUT_MS;
  status_.beta_recovery_tau_s_setting = Config::BETA_RECOVERY_TAU_S;
  updatePulseModelPrediction();
  status_.remaining_ms = measurementTotalDurationMs();
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.sync_event_id = 0;
  beginStartSync(millis());
  return true;
}


bool ExperimentRunner::startZeroCrossTest(int16_t fixed_current_mA, uint16_t fixed_pulse_width_ms,
                                           QRunMode q_run_mode, float control_target_deg,
                                           uint8_t q_probe_schedule_id) {
  energy_control_v0_mode_ = false;
  energy_control_autonomous_mode_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  energy_control_v0_pulse_authorized_ = false;
  if (q_run_mode == QRunMode::CONTROL &&
      (control_target_deg < Config::ZERO_CROSS_CONTROL_TARGET_MIN_DEG ||
       control_target_deg > Config::ZERO_CROSS_CONTROL_TARGET_MAX_DEG)) {
    status_.last_error = "invalid_control_target";
    return false;
  }
  if (q_run_mode == QRunMode::CONTROL &&
      q_probe_schedule_id >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_COUNT) {
    status_.last_error = "invalid_q_probe_schedule";
    return false;
  }
  if (!isSelectableZeroCrossCurrent(fixed_current_mA)) {
    status_.last_error = "invalid_zero_cross_current";
    return false;
  }
  if (!isSelectableZeroCrossPulseMs(fixed_pulse_width_ms)) {
    status_.last_error = "invalid_zero_cross_pulse_ms";
    return false;
  }
  if (!logger_ || !imu_ || !roller_) return false;
  if (!logger_->ready()) {
    status_.last_error = "psram_not_ready";
    return false;
  }
  if (!imu_->ok()) {
    status_.last_error = "imu_not_ready";
    return false;
  }
  if (status_.state != ExperimentState::READY_TO_MEASURE && status_.state != ExperimentState::FINISHED &&
      status_.state != ExperimentState::ESTOP) {
    status_.last_error = "not_ready";
    return false;
  }

  zero_cross_fixed_current_mA_ = fixed_current_mA;
  zero_cross_fixed_pulse_ms_ = fixed_pulse_width_ms;
  zero_cross_mode_ = true;
  passive_capture_mode_ = false;
  q_run_mode_ = q_run_mode;
  q_probe_schedule_id_ = q_run_mode_ == QRunMode::CONTROL ? q_probe_schedule_id :
      Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A;
  identification_mode_ = q_run_mode_ != QRunMode::NONE;
  control_target_peak_deg_ = q_run_mode_ == QRunMode::CONTROL ? control_target_deg : 0.0f;
  identification_target_index_ = 0;
  identification_event_id_ = 0;
  identification_peak_waiting_ = false;
  identification_outbound_rate_sign_ = 0;
  identification_reverse_samples_ = 0;
  identification_peak_angle_deg_ = 0.0f;
  predicted_signed_current_end_mA_ = 0.0f;
  predicted_current_end_ms_ = 0;
  single_trial_mode_ = true;
  selected_trial_index_ = 0;
  stopMotor();
  status_.run_id++;
  if (status_.run_id == 0) status_.run_id = 1;
  last_log_us_ = 0;
  last_pulse_end_ms_ = 0;
  last_zero_cross_pulse_start_ms_ = 0;
  zero_cross_start_refractory_until_ms_ = 0;
  zero_cross_half_cycle_peak_abs_deg_ = 0.0f;
  status_.pulse_id = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  status_.measure_elapsed_ms = 0;
  status_.trial_index = 1;
  status_.trial_count = 1;
  status_.trial_elapsed_ms = 0;
  status_.trial_duration_ms = measurementTotalDurationMs();
  status_.current_mA_setting = zero_cross_fixed_current_mA_;
  status_.pulse_width_ms_setting = zero_cross_fixed_pulse_ms_;
  status_.input_interval_ms = 0;
  status_.beta_hold_after_input_ms_setting = Config::BETA_HOLD_AFTER_INPUT_MS;
  status_.beta_recovery_tau_s_setting = Config::BETA_RECOVERY_TAU_S;
  updatePulseModelPrediction();
  status_.remaining_ms = measurementTotalDurationMs();
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.sync_event_id = 0;
  beginStartSync(millis());
  return true;
}
bool ExperimentRunner::startZeroCrossIdentificationTest() {
  return startZeroCrossTest(Config::ZERO_CROSS_OPERATING_CURRENT_MA, 20, QRunMode::VALIDATION);
}

bool ExperimentRunner::startZeroCrossControlTest(float target_peak_deg, uint8_t q_probe_schedule_id) {
  return startZeroCrossTest(Config::ZERO_CROSS_OPERATING_CURRENT_MA, 20,
                            QRunMode::CONTROL, target_peak_deg, q_probe_schedule_id);
}

bool ExperimentRunner::startPassiveCapture() {
  energy_control_v0_mode_ = false;
  energy_control_autonomous_mode_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  energy_control_v0_pulse_authorized_ = false;
  if (!logger_ || !imu_ || !roller_) return false;
  if (!logger_->ready()) {
    status_.last_error = "psram_not_ready";
    return false;
  }
  if (!imu_->ok()) {
    status_.last_error = "imu_not_ready";
    return false;
  }
  if (status_.state != ExperimentState::READY_TO_MEASURE && status_.state != ExperimentState::FINISHED &&
      status_.state != ExperimentState::ESTOP) {
    status_.last_error = "not_ready";
    return false;
  }

  // Do not enter the zero-cross setup: this is a manual-release logger only.
  // The signed shadow target is snapshotted now, before the log is locked.
  q1_shadow_run_target_peak_abs_deg_ = q1_shadow_target_peak_abs_deg_;
  q_ident_mode_ = false;
  q_ident_pulse_authorized_ = false;
  resetQIdentTracker();
  resetQ1ShadowZeroCrossTracker();
  passive_capture_mode_ = true;
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
  status_.remaining_ms = Config::PASSIVE_CAPTURE_DURATION_MS;
  status_.trial_index = 1;
  status_.trial_count = 1;
  status_.trial_elapsed_ms = 0;
  status_.trial_duration_ms = Config::PASSIVE_CAPTURE_DURATION_MS;
  status_.current_mA_setting = 0;
  status_.pulse_width_ms_setting = 0;
  status_.input_interval_ms = 0;
  status_.beta_hold_after_input_ms_setting = 0;
  status_.beta_recovery_tau_s_setting = 0.0f;
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.sync_event_id = 0;
  beginStartSync(millis());
  return true;
}

bool ExperimentRunner::startEnergyControlV0Capture() {
  if (!logger_ || !imu_ || !roller_) return false;
  if (!logger_->ready()) { status_.last_error = "psram_not_ready"; return false; }
  if (!imu_->ok()) { status_.last_error = "imu_not_ready"; return false; }
  if (status_.state != ExperimentState::READY_TO_MEASURE && status_.state != ExperimentState::FINISHED &&
      status_.state != ExperimentState::ESTOP) {
    status_.last_error = "not_ready";
    return false;
  }

  // V0 is a manual one-release capture. No Q_IDENT arm, schedule, legacy
  // control, start-kick, rebuild, E2 or inverse-Q state can initialize here.
  passive_capture_mode_ = false;
  q_ident_mode_ = false;
  q_ident_pulse_authorized_ = false;
  energy_control_v0_mode_ = true;
  energy_control_autonomous_mode_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  resetEnergyControlV0OutputGate();
  energy_control_v0_pulse_authorized_ = false;
  q_ident_run_schedule_id_ = 0;
  resetQIdentTracker();
  q1_shadow_run_target_peak_abs_deg_ = q1_shadow_target_peak_abs_deg_;  // diagnostic only.
  resetQ1ShadowZeroCrossTracker();
  zero_cross_mode_ = false;
  single_trial_mode_ = true;
  selected_trial_index_ = 0;
  identification_mode_ = false;
  q_run_mode_ = QRunMode::NONE;
  q_probe_schedule_id_ = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A;
  control_target_peak_deg_ = Config::ENERGY_CONTROL_V0_TARGET_PEAK_DEG;
  // The frozen validated 300 mA rise/decay model is used only for this V0
  // pulse solver.  The stored UI settings remain zero until a pulse is issued.
  zero_cross_fixed_current_mA_ = Config::Q_IDENT_CURRENT_MA;
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
  status_.remaining_ms = Config::ENERGY_CONTROL_V0_DURATION_MS;
  status_.trial_index = 1;
  status_.trial_count = 1;
  status_.trial_elapsed_ms = 0;
  status_.trial_duration_ms = Config::ENERGY_CONTROL_V0_DURATION_MS;
  status_.current_mA_setting = 0;
  status_.pulse_width_ms_setting = 0;
  status_.input_interval_ms = 0;
  status_.beta_hold_after_input_ms_setting = 0;
  status_.beta_recovery_tau_s_setting = 0.0f;
  updatePulseModelPrediction();
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.sync_event_id = 0;
  beginStartSync(millis());
  return true;
}
bool ExperimentRunner::setEnergyControlAutonomousTarget(float target_deg) {
  if (running()) { status_.last_error = "energy_target_while_running"; return false; }
  bool selected = false;
  for (float allowed : Config::ENERGY_CONTROL_AUTONOMOUS_TARGET_CHOICES_DEG) {
    if (fabsf(target_deg - allowed) <= 0.001f) { selected = true; break; }
  }
  if (!selected) { status_.last_error = "energy_target_not_selectable"; return false; }
  energy_control_autonomous_target_peak_deg_ = target_deg;
  status_.last_error = "";
  return true;
}

const char* ExperimentRunner::energyControlAutonomousPhaseName() const {
  switch (energy_control_autonomous_phase_) {
    case EnergyControlAutonomousPhase::STRONG_START_KICK: return "STRONG_START_KICK";
    case EnergyControlAutonomousPhase::WAIT_FIRST_PEAK: return "WAIT_FIRST_PEAK";
    case EnergyControlAutonomousPhase::ENERGY_CONTROL: return "ENERGY_CONTROL";
    case EnergyControlAutonomousPhase::HOLD: return "HOLD";
    case EnergyControlAutonomousPhase::STOP: return "STOP";
    default: return "IDLE";
  }
}

bool ExperimentRunner::startEnergyControlAutonomousCapture() {
  if (!logger_ || !imu_ || !roller_) return false;
  if (!logger_->ready()) { status_.last_error = "psram_not_ready"; return false; }
  if (!imu_->ok()) { status_.last_error = "imu_not_ready"; return false; }
  // ESTOP must be cleared through the existing explicit reset path first.
  if (status_.state != ExperimentState::READY_TO_MEASURE && status_.state != ExperimentState::FINISHED) {
    status_.last_error = "not_ready";
    return false;
  }

  // New path: it does not inherit V0's manual-release gate, Q_IDENT schedule,
  // E2 shadow tracker, or legacy scheduler.
  passive_capture_mode_ = false;
  q_ident_mode_ = false;
  q_ident_pulse_authorized_ = false;
  energy_control_v0_mode_ = false;
  energy_control_autonomous_mode_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  energy_control_v0_pulse_authorized_ = false;
  energy_control_autonomous_mode_ = true;
  energy_control_autonomous_pulse_authorized_ = false;
  resetEnergyControlAutonomous();
  q_ident_run_schedule_id_ = 0;
  resetQIdentTracker();
  q1_shadow_run_target_peak_abs_deg_ = q1_shadow_target_peak_abs_deg_;
  resetQ1ShadowZeroCrossTracker();
  zero_cross_mode_ = false;
  single_trial_mode_ = true;
  selected_trial_index_ = 0;
  identification_mode_ = false;
  q_run_mode_ = QRunMode::NONE;
  q_probe_schedule_id_ = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A;
  control_target_peak_deg_ = energy_control_autonomous_target_peak_deg_;
  // Autonomous current is intentionally independent of Q_IDENT scheduling.
  zero_cross_fixed_current_mA_ = Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA;
  zero_cross_fixed_pulse_ms_ = 0;
  stopMotor();
  status_.run_id++;
  if (status_.run_id == 0) status_.run_id = 1;
  last_log_us_ = 0;
  last_pulse_end_ms_ = 0;
  predicted_signed_current_end_mA_ = 0.0f;
  predicted_current_end_ms_ = 0;
  status_.pulse_id = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  status_.motor_cmd_mA = 0;
  status_.measure_elapsed_ms = 0;
  status_.remaining_ms = Config::ENERGY_CONTROL_AUTONOMOUS_DURATION_MS;
  status_.trial_index = 1;
  status_.trial_count = 1;
  status_.trial_elapsed_ms = 0;
  status_.trial_duration_ms = Config::ENERGY_CONTROL_AUTONOMOUS_DURATION_MS;
  status_.current_mA_setting = 0;
  status_.pulse_width_ms_setting = 0;
  status_.input_interval_ms = 0;
  status_.beta_hold_after_input_ms_setting = 0;
  status_.beta_recovery_tau_s_setting = 0.0f;
  updatePulseModelPrediction();
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.sync_event_id = 0;
  beginStartSync(millis());
  return true;
}
bool ExperimentRunner::startQIdentCapture(uint8_t q_ident_run_schedule_id) {
  (void)q_ident_run_schedule_id;
  q_ident_mode_ = false;
  q_ident_pulse_authorized_ = false;
  energy_control_v0_mode_ = false;
  energy_control_autonomous_mode_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  energy_control_v0_pulse_authorized_ = false;
  stopMotor();
  status_.last_error = "q_ident_frozen_use_energy_control_v0";
  return false;
  if (!logger_ || !imu_ || !roller_) return false;
  if (!logger_->ready()) {
    status_.last_error = "psram_not_ready";
    return false;
  }
  if (!imu_->ok()) {
    status_.last_error = "imu_not_ready";
    return false;
  }
  if (q_ident_run_schedule_id >= Config::Q_IDENT_SCHEDULE_COUNT) {
    status_.last_error = "q_ident_schedule_invalid";
    return false;
  }
  if (status_.state != ExperimentState::READY_TO_MEASURE && status_.state != ExperimentState::FINISHED &&
      status_.state != ExperimentState::ESTOP) {
    status_.last_error = "not_ready";
    return false;
  }

  // This is the only start function allowed to enable actual fixed-Q output.
  passive_capture_mode_ = false;
  q_ident_mode_ = true;
  q_ident_pulse_authorized_ = false;
  q_ident_run_schedule_id_ = q_ident_run_schedule_id;
  resetQIdentTracker();
  q1_shadow_run_target_peak_abs_deg_ = q1_shadow_target_peak_abs_deg_;  // diagnostic only.
  resetQ1ShadowZeroCrossTracker();
  zero_cross_mode_ = false;
  single_trial_mode_ = true;
  selected_trial_index_ = 0;
  identification_mode_ = false;
  q_run_mode_ = QRunMode::NONE;
  q_probe_schedule_id_ = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A;
  control_target_peak_deg_ = 0.0f;
  // The validated 300 mA current/pulse model is used only by qIdentSolvePulse.
  zero_cross_fixed_current_mA_ = Config::Q_IDENT_CURRENT_MA;
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
  status_.remaining_ms = Config::Q_IDENT_DURATION_MS;
  status_.trial_index = 1;
  status_.trial_count = 1;
  status_.trial_elapsed_ms = 0;
  status_.trial_duration_ms = Config::Q_IDENT_DURATION_MS;
  status_.current_mA_setting = 0;
  status_.pulse_width_ms_setting = 0;
  status_.input_interval_ms = 0;
  status_.beta_hold_after_input_ms_setting = 0;
  status_.beta_recovery_tau_s_setting = 0.0f;
  updatePulseModelPrediction();
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.sync_event_id = 0;
  beginStartSync(millis());
  return true;
}
const char* ExperimentRunner::qRunModeName() const {
  switch (q_run_mode_) {
    case QRunMode::VALIDATION: return "validation";
    case QRunMode::CONTROL: return "control";
    default: return "none";
  }
}

const char* ExperimentRunner::qProbeScheduleName() const {
  switch (q_probe_schedule_id_) {
    case Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A: return "A";
    case Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_B: return "B";
    case Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_C: return "C";
    default: return "invalid";
  }
}

uint8_t ExperimentRunner::calibrationProbeQLevelIndex(uint8_t plan_index) const {
  if (q_probe_schedule_id_ >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_COUNT ||
      plan_index >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_COUNT) {
    return 0;
  }
  return Config::ZERO_CROSS_CALIBRATION_Q_PROBE_Q_LEVEL_INDEX_BY_SCHEDULE[
      q_probe_schedule_id_][plan_index];
}

uint8_t ExperimentRunner::classifyV59StateGate(float hprev_deg, float cprev_deg,
                                                 float abs_rate_dps, int8_t next_peak_side,
                                                 bool dynamic_h_in_support) const {
  if (!Config::ZERO_CROSS_V59_STATE_WAIT_FIXED_Q_ENABLED) {
    return Config::ZERO_CROSS_V59_GATE_REASON_DISABLED;
  }
  const bool state_valid = calibration_last_half_range_valid_ &&
      calibration_last_center_dynamic_valid_ && isfinite(hprev_deg) && isfinite(cprev_deg) &&
      isfinite(abs_rate_dps) && (next_peak_side == -1 || next_peak_side == 1);
  if (!state_valid) return Config::ZERO_CROSS_V59_GATE_REASON_STATE_INVALID;
  if (!dynamic_h_in_support) return Config::ZERO_CROSS_V59_GATE_REASON_DYNAMIC_H_OUT_OF_SUPPORT;
  if (hprev_deg < Config::ZERO_CROSS_V59_GATE_HPREV_MIN_DEG) return Config::ZERO_CROSS_V59_GATE_REASON_HPREV_BELOW;
  if (hprev_deg > Config::ZERO_CROSS_V59_GATE_HPREV_MAX_DEG) return Config::ZERO_CROSS_V59_GATE_REASON_HPREV_ABOVE;
  if (cprev_deg < Config::ZERO_CROSS_V59_GATE_CPREV_MIN_DEG) return Config::ZERO_CROSS_V59_GATE_REASON_CPREV_BELOW;
  if (cprev_deg > Config::ZERO_CROSS_V59_GATE_CPREV_MAX_DEG) return Config::ZERO_CROSS_V59_GATE_REASON_CPREV_ABOVE;
  if (abs_rate_dps < Config::ZERO_CROSS_V59_GATE_ABS_RATE_MIN_DPS) return Config::ZERO_CROSS_V59_GATE_REASON_RATE_BELOW;
  if (abs_rate_dps > Config::ZERO_CROSS_V59_GATE_ABS_RATE_MAX_DPS) return Config::ZERO_CROSS_V59_GATE_REASON_RATE_ABOVE;
  if (next_peak_side != Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE) return Config::ZERO_CROSS_V59_GATE_REASON_DIRECTION_MISMATCH;
  return Config::ZERO_CROSS_V59_GATE_REASON_PASSED;
}

void ExperimentRunner::recordV59StateGateEvent(uint32_t crossing_ms, float hprev_deg,
                                                 float cprev_deg, float abs_rate_dps,
                                                 int8_t next_peak_side, bool state_valid,
                                                 bool dynamic_h_in_support, uint8_t reason,
                                                 uint8_t action,
                                                 bool v60_cooldown_free_decay,
                                                 bool v60_gate_skipped_by_decay) {
  if (!logger_) return;
  PsramLogger::CalibrationStateGateEvent event;
  event.planned_probe_index = calibration_probe_plan_index_ + 1;
  event.q_level_index = calibrationProbeQLevelIndex(calibration_probe_plan_index_);
  event.reason = reason;
  event.action = action;
  event.wait_halfcycle_count = calibration_probe_wait_halfcycle_count_;
  event.rebuild_total_count = calibration_rebuild_total_count_;
  event.crossing_ms = crossing_ms;
  event.hprev_cdeg = state_valid ? centi(hprev_deg) : LOG_NAN_I16;
  event.cprev_cdeg = state_valid ? centi(cprev_deg) : LOG_NAN_I16;
  event.rate_abs_cdps = state_valid ? centi(abs_rate_dps) : LOG_NAN_I16;
  event.next_peak_side = next_peak_side;
  event.state_valid = state_valid;
  event.dynamic_h_in_support = dynamic_h_in_support;
  event.v60_cooldown_free_decay = v60_cooldown_free_decay;
  event.v60_gate_skipped_by_decay = v60_gate_skipped_by_decay;
  logger_->addCalibrationStateGateEvent(event);
}

void ExperimentRunner::beginStartSync(uint32_t now_ms, bool wheel_probe, bool fixed_probe) {
  fixed_probe_mode_=fixed_probe;
  wheel_probe_mode_ = wheel_probe;
  run_start_us_ = micros();
  logger_->startRun(status_.run_id, run_start_us_, status_.current_mA_setting, status_.pulse_width_ms_setting,
                    status_.input_interval_ms, identification_mode_,
                    static_cast<uint8_t>(q_run_mode_), centi(control_target_peak_deg_), q_probe_schedule_id_,
                    passive_capture_mode_, q1_shadow_run_target_peak_abs_deg_, q_ident_mode_,
                    q_ident_mode_ ? static_cast<uint8_t>(q_ident_run_schedule_id_ + 1) : 0,
                     energy_control_v0_mode_, energy_control_autonomous_mode_);
  roller_->qObserver().v55.enabled = wheel_probe_mode_;
  roller_->qObserver().v57.enabled=fixed_probe_mode_;
  last_log_us_ = 0;
  status_.state = ExperimentState::START_SYNC;
  status_.sync_event_id = 1;
  sync_step_ = 0;
  sync_step_start_ms_ = now_ms;
  sync_led_until_ms_ = 0;
  setSyncLed(Config::START_SYNC_PATTERN[0].led_on);
  logSampleNow();
}

void ExperimentRunner::beginMeasurementRun() {
  if (!passive_capture_mode_ && !q_ident_mode_ && !energy_control_v0_mode_ && !wheel_probe_mode_ && !fixed_probe_mode_) {
    captureAngleOffsets();
    updateDisplayedAngles(imu_->reading());
  }
  run_start_ms_ = millis();
  if (passive_capture_mode_ || q_ident_mode_ || energy_control_v0_mode_ || energy_control_autonomous_mode_) resetQ1ShadowZeroCrossTracker();
  if (energy_control_v0_mode_) resetEnergyControlV0OutputGate();
  if (energy_control_autonomous_mode_) resetEnergyControlAutonomous();
  if (q_ident_mode_) resetQIdentTracker();
  next_pulse_start_test_ms_ = 0;
  active_pulse_start_ms_ = 0;
  active_pulse_start_test_ms_ = 0;
  last_pulse_end_ms_ = 0;
  next_pulse_direction_ = 1;
  sync_led_until_ms_ = 0;
  status_.measure_elapsed_ms = 0;
  status_.remaining_ms = measurementTotalDurationMs();
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.state = ExperimentState::RUNNING_BATCH_SWEEP;
  status_.sync_event_id = 2;
  setSyncLed(false);
  // Select the configured comparison betas before emitting the t_test=0 row.
  // This preserves the shared quaternion while making the logged transition exact.
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    beta_smooth_[i] = betaCeilingForStrategy(i);
    status_.beta_target_series[i] = beta_smooth_[i];
    status_.beta_smooth_series[i] = beta_smooth_[i];
    filter_dynamic_raw_[i].setBeta(beta_smooth_[i]);
    filter_dynamic_bias_[i].setBeta(beta_smooth_[i]);
  }
  status_.beta_target = status_.beta_target_series[Config::FILTER_ADOPTED_INDEX];
  status_.beta_smooth = status_.beta_smooth_series[Config::FILTER_ADOPTED_INDEX];
  updateDisplayedAngles(imu_->reading());
  logSampleNow();
  if(fixed_probe_mode_) {
    trial_start_ms_=run_start_ms_;
    status_.trial_duration_ms=FixedProbeV57::RUN_LIMIT_MS;
    if(!roller_->beginFixedProbe(fixed_probe_full_,status_.run_id))
      requestEmergencyStop("v57_start_failed");
    return;
  }
  if (wheel_probe_mode_) {
    trial_start_ms_ = run_start_ms_;
    status_.trial_duration_ms = WheelProbeV55::RUN_LIMIT_MS;
    status_.current_mA_setting = status_.pulse_width_ms_setting = status_.input_interval_ms = 0;
    roller_->qObserver().v55.begin(status_.run_id, micros());
    wheel_probe_waiting_ = false; wheel_probe_pulse_id_ = 0;
    wheel_probe_next_us_ = micros() + WheelProbeV55::REST_US;
    stopMotor(); return;
  }
  if (passive_capture_mode_ || q_ident_mode_ || energy_control_v0_mode_ || energy_control_autonomous_mode_) {
    // Dedicated capture modes skip beginTrial(), isolating legacy scheduling.
    status_.trial_index = 1;
    status_.trial_count = 1;
    status_.trial_elapsed_ms = 0;
    status_.trial_duration_ms = energy_control_autonomous_mode_ ? Config::ENERGY_CONTROL_AUTONOMOUS_DURATION_MS :
        (energy_control_v0_mode_ ? Config::ENERGY_CONTROL_V0_DURATION_MS :
        (q_ident_mode_ ? Config::Q_IDENT_DURATION_MS : Config::PASSIVE_CAPTURE_DURATION_MS));
    status_.current_mA_setting = 0;
    status_.pulse_width_ms_setting = 0;
    status_.input_interval_ms = 0;
    status_.motor_cmd_mA = 0;
    trial_start_ms_ = run_start_ms_;
    stopMotor();
    if (energy_control_autonomous_mode_) beginEnergyControlAutonomousStartKick(millis());
    return;
  }
  beginTrial(single_trial_mode_ ? selected_trial_index_ : 0);
}

void ExperimentRunner::beginTrial(uint8_t trial_index) {
  if (!zero_cross_mode_ && trial_index >= Config::BETA_SWEEP_TRIAL_COUNT) {
    finishRun();
    return;
  }
  stopMotor();
  status_.state = ExperimentState::RUNNING_BATCH_SWEEP;
  status_.trial_elapsed_ms = 0;
  if (zero_cross_mode_) {
    status_.trial_index = 1;
    status_.trial_count = 1;
    status_.trial_duration_ms = Config::ZERO_CROSS_TEST_DURATION_MS;
    status_.current_mA_setting = zero_cross_fixed_current_mA_;
    status_.pulse_width_ms_setting = zero_cross_fixed_pulse_ms_;
    status_.input_interval_ms = 0;  // Event-driven zero-cross mode.
  } else {
    const auto& trial = Config::BETA_SWEEP_TRIALS[trial_index];
    status_.trial_index = trial_index + 1;
    status_.trial_count = Config::BETA_SWEEP_TRIAL_COUNT;
    status_.trial_duration_ms = trial.duration_ms;
    status_.current_mA_setting = trial.current_mA;
    status_.pulse_width_ms_setting = trial.pulse_width_ms;
    status_.input_interval_ms = trial.input_interval_ms;
  }
  status_.beta_hold_after_input_ms_setting = Config::BETA_HOLD_AFTER_INPUT_MS;
  status_.beta_recovery_tau_s_setting = Config::BETA_RECOVERY_TAU_S;
  updatePulseModelPrediction();
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    beta_smooth_[i] = betaCeilingForStrategy(i);
    status_.beta_target_series[i] = beta_smooth_[i];
    status_.beta_smooth_series[i] = beta_smooth_[i];
    filter_dynamic_raw_[i].setBeta(beta_smooth_[i]);
    filter_dynamic_bias_[i].setBeta(beta_smooth_[i]);
  }
  status_.beta_target = status_.beta_target_series[Config::FILTER_ADOPTED_INDEX];
  status_.beta_smooth = status_.beta_smooth_series[Config::FILTER_ADOPTED_INDEX];
  trial_start_ms_ = millis();
  active_pulse_start_ms_ = 0;
  active_pulse_start_test_ms_ = 0;
  last_pulse_end_ms_ = 0;
  last_zero_cross_pulse_start_ms_ = 0;
  zero_cross_start_refractory_until_ms_ = 0;
  zero_cross_half_cycle_peak_abs_deg_ = 0.0f;
  zero_cross_next_direction_ = -Config::ZERO_CROSS_BOOTSTRAP_DIRECTION;
  next_pulse_start_test_ms_ = 0;
  next_pulse_direction_ = 1;
  zero_cross_bootstrap_pending_ = zero_cross_mode_;
  zero_cross_armed_ = false;
  zero_cross_has_previous_angle_ = false;
  zero_cross_previous_angle_deg_ = 0.0f;
  resetCalibrationShadow();
  beta_phase_.reset();
  beta_turn_fast_.reset();
  status_.pulse_id = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  captureAngleOffsets();
  updateDisplayedAngles(imu_->reading());
  status_.sync_event_id = 6;
  logSampleNow();
}

void ExperimentRunner::zeroAngleNow() {
  if (passive_capture_mode_) {
    status_.last_error = "zero_disabled_in_passive_mode";
    return;
  }
  captureAngleOffsets();
  if (imu_) updateDisplayedAngles(imu_->reading());
}

bool ExperimentRunner::zeroCurrentRollDisplay() {
  if (running()) {
    status_.last_error = "current_roll_zero_while_running";
    return false;
  }
  if (!status_.static_confirmed) {
    status_.last_error = "current_roll_zero_requires_static";
    return false;
  }
  display_zero_offset_deg_ = status_.physical_roll_abs_deg;
  status_.display_zero_offset_deg = display_zero_offset_deg_;
  status_.current_roll_deg = 0.0f;
  status_.target_error_deg = -target_roll_deg_;
  status_.ready = fabsf(status_.target_error_deg) <= Config::TARGET_TOLERANCE_DEG;
  status_.last_error = "";
  return true;
}

bool ExperimentRunner::setCurrentRollTarget(float target_deg) {
  if (!isfinite(target_deg) || target_deg < Config::CURRENT_ROLL_TARGET_MIN_DEG ||
      target_deg > Config::CURRENT_ROLL_TARGET_MAX_DEG) {
    status_.last_error = "current_roll_target_out_of_range";
    return false;
  }
  target_roll_deg_ = target_deg;
  status_.target_roll_deg = target_roll_deg_;
  status_.target_error_deg = status_.current_roll_deg - target_roll_deg_;
  status_.ready = status_.static_confirmed &&
      fabsf(status_.target_error_deg) <= Config::TARGET_TOLERANCE_DEG;
  status_.last_error = "";
  return true;
}

bool ExperimentRunner::setQ1ShadowTargetPeakAbs(float target_deg) {
  if (running()) {
    status_.last_error = "q1_shadow_target_while_running";
    return false;
  }
  if (!isfinite(target_deg) || target_deg < Config::Q1_SHADOW_TARGET_MIN_DEG ||
      target_deg > Config::Q1_SHADOW_TARGET_MAX_DEG) {
    status_.last_error = "q1_shadow_target_out_of_range";
    return false;
  }
  q1_shadow_target_peak_abs_deg_ = target_deg;
  status_.last_error = "";
  return true;
}

void ExperimentRunner::resetE2ShadowPeakTracker() {
  e2_shadow_armed_ = false;
  e2_shadow_release_detected_ms_ = 0;
  e2_shadow_gyro_integrator_ready_ = false;
  e2_shadow_last_gyro_sample_us_ = 0;
  e2_shadow_last_gyro_rate_dps_ = 0.0f;
  e2_shadow_gyro_relative_deg_ = 0.0f;
  e2_shadow_pending_static_anchor_abs_deg_ = status_.static_confirmed &&
      isfinite(status_.physical_roll_abs_deg) ? status_.physical_roll_abs_deg : NAN;
  e2_shadow_static_anchor_abs_deg_ = NAN;
  e2_shadow_candidate_motion_sign_ = 0;
  e2_shadow_reverse_samples_ = 0;
  e2_shadow_candidate_gyro_relative_deg_ = 0.0f;
  e2_shadow_candidate_accel_abs_diag_deg_ = NAN;
  e2_shadow_candidate_peak_ms_ = 0;
  e2_shadow_last_peak_valid_ = false;
  e2_shadow_last_turn_side_ = 0;
  e2_shadow_last_peak_gyro_relative_deg_ = NAN;
  e2_shadow_turn_index_ = 0;
}

void ExperimentRunner::updateE2ShadowPeakTracker(uint32_t now_ms) {
  if (!logger_ || !imu_) return;
  const float accel_abs_diag_deg = status_.physical_roll_abs_deg;
  const float rate_dps = status_.physical_roll_rate_dps;
  if (!isfinite(rate_dps)) return;
  const uint32_t t_test_ms = now_ms - run_start_ms_;
  const uint32_t sample_us = imu_->reading().last_update_us;

  // Retain only a static acceleration angle as the absolute-angle anchor. It
  // is a video-comparison diagnostic; it never contributes to H or Q validity.
  if (!e2_shadow_armed_ && status_.static_confirmed && isfinite(accel_abs_diag_deg)) {
    e2_shadow_pending_static_anchor_abs_deg_ = accel_abs_diag_deg;
    e2_shadow_gyro_relative_deg_ = 0.0f;
    e2_shadow_gyro_integrator_ready_ = true;
    e2_shadow_last_gyro_sample_us_ = sample_us;
    e2_shadow_last_gyro_rate_dps_ = rate_dps;
  } else if (!e2_shadow_gyro_integrator_ready_) {
    e2_shadow_gyro_integrator_ready_ = true;
    e2_shadow_last_gyro_sample_us_ = sample_us;
    e2_shadow_last_gyro_rate_dps_ = rate_dps;
  } else {
    const uint32_t dt_us = sample_us - e2_shadow_last_gyro_sample_us_;
    if (dt_us > 0 && dt_us <= Config::E2_SHADOW_GYRO_INTEGRATION_MAX_DT_US) {
      e2_shadow_gyro_relative_deg_ += 0.5f *
          (e2_shadow_last_gyro_rate_dps_ + rate_dps) * (static_cast<float>(dt_us) / 1000000.0f);
    }
    e2_shadow_last_gyro_sample_us_ = sample_us;
    e2_shadow_last_gyro_rate_dps_ = rate_dps;
  }

  if (!e2_shadow_armed_) {
    if (fabsf(rate_dps) < Config::E2_SHADOW_RELEASE_ARM_RATE_THRESHOLD_DPS) return;
    e2_shadow_armed_ = true;
    e2_shadow_release_detected_ms_ = t_test_ms;
    e2_shadow_static_anchor_abs_deg_ = e2_shadow_pending_static_anchor_abs_deg_;
    // Arm clears all pre-release peak history. The first accepted turn is a
    // baseline only; the first H is formed by the following opposite-side turn.
    e2_shadow_candidate_motion_sign_ = rate_dps > 0.0f ? 1 : -1;
    e2_shadow_reverse_samples_ = 0;
    e2_shadow_candidate_gyro_relative_deg_ = e2_shadow_gyro_relative_deg_;
    e2_shadow_candidate_accel_abs_diag_deg_ = accel_abs_diag_deg;
    e2_shadow_candidate_peak_ms_ = t_test_ms;
    e2_shadow_last_peak_valid_ = false;
    e2_shadow_last_turn_side_ = 0;
    e2_shadow_last_peak_gyro_relative_deg_ = NAN;
    e2_shadow_turn_index_ = 0;
    return;
  }

  const int8_t motion_sign = rate_dps >= Config::E2_SHADOW_PEAK_RATE_THRESHOLD_DPS ? 1 :
      (rate_dps <= -Config::E2_SHADOW_PEAK_RATE_THRESHOLD_DPS ? -1 : 0);
  if (motion_sign == 0) return;
  if (motion_sign == e2_shadow_candidate_motion_sign_) {
    e2_shadow_reverse_samples_ = 0;
    // Candidate time and gyro integral are frozen at the final sample before
    // the rate reversal; confirmation samples never enter the half-cycle H.
    e2_shadow_candidate_gyro_relative_deg_ = e2_shadow_gyro_relative_deg_;
    e2_shadow_candidate_accel_abs_diag_deg_ = accel_abs_diag_deg;
    e2_shadow_candidate_peak_ms_ = t_test_ms;
    return;
  }

  if (++e2_shadow_reverse_samples_ < Config::E2_SHADOW_PEAK_CONFIRM_SAMPLES) return;
  recordE2ShadowPeak(e2_shadow_candidate_peak_ms_, t_test_ms,
                     e2_shadow_candidate_motion_sign_,
                     e2_shadow_candidate_gyro_relative_deg_,
                     e2_shadow_candidate_accel_abs_diag_deg_);
  e2_shadow_candidate_motion_sign_ = motion_sign;
  e2_shadow_candidate_gyro_relative_deg_ = e2_shadow_gyro_relative_deg_;
  e2_shadow_candidate_accel_abs_diag_deg_ = accel_abs_diag_deg;
  e2_shadow_candidate_peak_ms_ = t_test_ms;
  e2_shadow_reverse_samples_ = 0;
}

void ExperimentRunner::recordE2ShadowPeak(uint32_t candidate_peak_ms, uint32_t confirmed_ms,
                                            int8_t turn_side_from_rate,
                                            float candidate_gyro_relative_deg,
                                            float candidate_accel_abs_diag_deg) {
  if (!logger_) return;
  PsramLogger::E2ShadowPeakEvent event;
  event.e2_armed = e2_shadow_armed_;
  event.release_detected_ms = e2_shadow_release_detected_ms_;
  event.turn_index = ++e2_shadow_turn_index_;
  event.candidate_peak_ms = candidate_peak_ms;
  event.confirmed_ms = confirmed_ms;
  event.turn_side_from_rate = turn_side_from_rate;
  event.physical_next_peak_side = turn_side_from_rate == 0 ? 0 : -turn_side_from_rate;
  event.static_anchor_abs_deg = e2_shadow_static_anchor_abs_deg_;
  event.peak_accel_abs_diag_deg = candidate_accel_abs_diag_deg;
  event.peak_gyro_integrated_abs_deg = isfinite(e2_shadow_static_anchor_abs_deg_) &&
      isfinite(candidate_gyro_relative_deg)
      ? e2_shadow_static_anchor_abs_deg_ + candidate_gyro_relative_deg : NAN;
  event.previous_peak_deg = e2_shadow_last_peak_valid_ && isfinite(e2_shadow_static_anchor_abs_deg_)
      ? e2_shadow_static_anchor_abs_deg_ + e2_shadow_last_peak_gyro_relative_deg_ : NAN;
  event.current_peak_deg = event.peak_gyro_integrated_abs_deg;
  event.target_peak_deg = NAN;
  event.alternating_side_ok = e2_shadow_last_peak_valid_ &&
      turn_side_from_rate != 0 && turn_side_from_rate != e2_shadow_last_turn_side_;

  uint8_t reason = Config::E2_SHADOW_INVALID_INSUFFICIENT_TURNS;
  if (!e2_shadow_last_peak_valid_) {
    // Establish the first post-release turn. No half-cycle H exists yet.
    reason = Config::E2_SHADOW_INVALID_INSUFFICIENT_TURNS;
  } else if (!event.alternating_side_ok) {
    // Never bridge a chatter/nonalternating turn into a later H interval.
    reason = Config::E2_SHADOW_INVALID_NONALTERNATING_TURN;
  } else {
    event.halfcycle_gyro_integral_deg = candidate_gyro_relative_deg -
        e2_shadow_last_peak_gyro_relative_deg_;
    event.h_prev_gyro_deg = 0.5f * fabsf(event.halfcycle_gyro_integral_deg);
    event.h_prev_deg = event.h_prev_gyro_deg;  // legacy H_prev alias
    if (!isfinite(event.h_prev_gyro_deg) ||
        event.h_prev_gyro_deg < Config::E2_SHADOW_H_PREV_MIN_DEG ||
        event.h_prev_gyro_deg > Config::E2_SHADOW_H_PREV_MAX_DEG) {
      reason = Config::E2_SHADOW_INVALID_H_OUT_OF_CALIBRATION_RANGE;
    } else {
      event.h_next_e2_deg = Config::E2_SHADOW_E2_SLOPE * event.h_prev_gyro_deg +
          Config::E2_SHADOW_E2_OFFSET_DEG;
      event.h_next_e2_explicit_deg = event.h_next_e2_deg;
      if (!isfinite(event.h_next_e2_deg) || event.h_next_e2_deg <= 0.0f) {
        reason = Config::E2_SHADOW_INVALID_NONFINITE_STATE;
      } else {
        // This revision validates E2 only in H-space. The static-anchor gyro
        // absolute peak is logged for video comparison, but it is not yet a
        // validated coordinate for passive absolute peak or inverse-Q logic.
        reason = Config::E2_SHADOW_INVALID_ABSOLUTE_PEAK_ESTIMATOR_UNVALIDATED;
      }
    }
  }
  event.shadow_valid = false;
  event.shadow_invalid_reason = reason;
  logger_->addE2ShadowPeakEvent(event);

  if (e2_shadow_last_peak_valid_ && !event.alternating_side_ok) {
    e2_shadow_last_peak_valid_ = false;
    e2_shadow_last_turn_side_ = 0;
    e2_shadow_last_peak_gyro_relative_deg_ = NAN;
    return;
  }
  e2_shadow_last_peak_valid_ = turn_side_from_rate != 0 && isfinite(candidate_gyro_relative_deg);
  e2_shadow_last_turn_side_ = turn_side_from_rate;
  e2_shadow_last_peak_gyro_relative_deg_ = candidate_gyro_relative_deg;
}
void ExperimentRunner::resetQ1ShadowZeroCrossTracker() {
  q1_shadow_has_previous_angle_ = false;
  q1_shadow_zero_cross_armed_ = false;
  q1_shadow_angle_zero_deg_ = status_.pitch_dynamic_beta_deg[Config::FILTER_ADOPTED_INDEX];
  if (!isfinite(q1_shadow_angle_zero_deg_)) q1_shadow_angle_zero_deg_ = 0.0f;
  q1_shadow_previous_relative_angle_deg_ = 0.0f;
  q1_shadow_previous_time_ms_ = 0;
  q1_shadow_previous_rate_dps_ = 0.0f;
  q1_shadow_last_zero_cross_ms_ = 0;
}

void ExperimentRunner::updateQ1ShadowAtZeroCross(uint32_t now_ms) {
  if (!logger_) return;
  // The continuous filter angle is only a detector coordinate. Q1 itself uses
  // +gy physical_roll_rate_dps at the crossing, not an IMU absolute angle.
  const float detector_angle_deg =
      status_.pitch_dynamic_beta_deg[Config::FILTER_ADOPTED_INDEX] - q1_shadow_angle_zero_deg_;
  const float rate_dps = status_.physical_roll_rate_dps;
  if (!isfinite(detector_angle_deg) || !isfinite(rate_dps)) return;
  if (energy_control_v0_mode_) updateEnergyControlV0OutputGate(detector_angle_deg);
  const uint32_t t_test_ms = now_ms - run_start_ms_;
  if (!q1_shadow_has_previous_angle_) {
    q1_shadow_has_previous_angle_ = true;
    q1_shadow_previous_relative_angle_deg_ = detector_angle_deg;
    q1_shadow_previous_time_ms_ = t_test_ms;
    q1_shadow_previous_rate_dps_ = rate_dps;
    return;
  }

  if (fabsf(detector_angle_deg) >= Config::Q1_SHADOW_ZERO_CROSS_REARM_ANGLE_DEG) {
    q1_shadow_zero_cross_armed_ = true;
  }
  const bool detector_neg_to_pos = q1_shadow_previous_relative_angle_deg_ < 0.0f &&
      detector_angle_deg >= 0.0f;
  const bool detector_pos_to_neg = q1_shadow_previous_relative_angle_deg_ > 0.0f &&
      detector_angle_deg <= 0.0f;
  // The adopted detector coordinate is anti-correlated with official +gy.
  // Keep the detector, start reference, rearm, and interval unchanged; only
  // this sign-consistency condition is reversed from the previous build.
  const bool sign_gate_passed =
      (detector_neg_to_pos && rate_dps <= -Config::Q1_SHADOW_ZERO_CROSS_MIN_ABS_RATE_DPS) ||
      (detector_pos_to_neg && rate_dps >= Config::Q1_SHADOW_ZERO_CROSS_MIN_ABS_RATE_DPS);
  const bool interval_ok = q1_shadow_last_zero_cross_ms_ == 0 ||
      t_test_ms - q1_shadow_last_zero_cross_ms_ >= Config::Q1_SHADOW_ZERO_CROSS_MIN_INTERVAL_MS;
  if (q1_shadow_zero_cross_armed_ && interval_ok && sign_gate_passed) {
    // The sign of +gy after the crossing is the physical side of the next peak.
    Q1ZeroCrossDiagnostics diagnostics;
    diagnostics.detector_crossing_direction = detector_neg_to_pos ? 1 : -1;
    diagnostics.detector_angle_before_deg = q1_shadow_previous_relative_angle_deg_;
    diagnostics.detector_angle_after_deg = detector_angle_deg;
    const float denominator = q1_shadow_previous_relative_angle_deg_ - detector_angle_deg;
    if (isfinite(denominator) && fabsf(denominator) > 1.0e-6f) {
      diagnostics.crossing_interpolation_alpha =
          q1_shadow_previous_relative_angle_deg_ / denominator;
      if (diagnostics.crossing_interpolation_alpha < 0.0f) {
        diagnostics.crossing_interpolation_alpha = 0.0f;
      } else if (diagnostics.crossing_interpolation_alpha > 1.0f) {
        diagnostics.crossing_interpolation_alpha = 1.0f;
      }
      const uint32_t interval_ms = t_test_ms - q1_shadow_previous_time_ms_;
      diagnostics.interpolated_zero_cross_time_ms = q1_shadow_previous_time_ms_ +
          static_cast<uint32_t>(lroundf(diagnostics.crossing_interpolation_alpha * interval_ms));
      diagnostics.interpolated_physical_roll_rate_dps = q1_shadow_previous_rate_dps_ +
          diagnostics.crossing_interpolation_alpha * (rate_dps - q1_shadow_previous_rate_dps_);
    }
    diagnostics.physical_roll_rate_before_dps = q1_shadow_previous_rate_dps_;
    diagnostics.physical_roll_rate_after_dps = rate_dps;
    diagnostics.sign_gate_passed = true;
    // Do not substitute interpolation diagnostics into the Q1 model yet.
    recordQ1ShadowZeroCross(t_test_ms, rate_dps, diagnostics);
    if (energy_control_v0_mode_) updateEnergyControlV0AtZeroCross(t_test_ms, rate_dps, diagnostics);
    if (q_ident_mode_) updateQIdentAtZeroCross(t_test_ms, rate_dps, diagnostics);
    q1_shadow_last_zero_cross_ms_ = t_test_ms;
    q1_shadow_zero_cross_armed_ = false;
  }
  q1_shadow_previous_relative_angle_deg_ = detector_angle_deg;
  q1_shadow_previous_time_ms_ = t_test_ms;
  q1_shadow_previous_rate_dps_ = rate_dps;
}

void ExperimentRunner::recordQ1ShadowZeroCross(
    uint32_t t_test_ms, float rate_dps, const Q1ZeroCrossDiagnostics& diagnostics) {
  if (!logger_) return;
  PsramLogger::Q1ShadowEvent event;
  event.zero_cross_time_ms = t_test_ms;
  event.zero_cross_rate_dps = rate_dps;
  event.zero_cross_abs_rate_dps = fabsf(rate_dps);
  event.physical_next_peak_side = rate_dps >= 0.0f ? 1 : -1;
  event.detector_crossing_direction = diagnostics.detector_crossing_direction;
  event.detector_angle_before_deg = diagnostics.detector_angle_before_deg;
  event.detector_angle_after_deg = diagnostics.detector_angle_after_deg;
  event.crossing_interpolation_alpha = diagnostics.crossing_interpolation_alpha;
  event.interpolated_zero_cross_time_ms = diagnostics.interpolated_zero_cross_time_ms;
  event.physical_roll_rate_before_dps = diagnostics.physical_roll_rate_before_dps;
  event.physical_roll_rate_after_dps = diagnostics.physical_roll_rate_after_dps;
  event.interpolated_physical_roll_rate_dps = diagnostics.interpolated_physical_roll_rate_dps;
  event.sign_gate_passed = diagnostics.sign_gate_passed;
  event.q1_intercept_deg = Config::Q1_SHADOW_INTERCEPT_DEG;
  event.q1_rate_term_deg = Config::Q1_SHADOW_RATE_GAIN_DEG_PER_DPS *
      event.zero_cross_abs_rate_dps;
  event.q1_side_term_deg = Config::Q1_SHADOW_SIDE_TERM_DEG *
      static_cast<float>(event.physical_next_peak_side);
  event.q1_baseline_next_peak_abs_deg = event.q1_intercept_deg +
      event.q1_rate_term_deg + event.q1_side_term_deg;
  event.target_next_peak_abs_deg = q1_shadow_run_target_peak_abs_deg_;
  event.q1_gain_deg_per_mA_s = event.physical_next_peak_side > 0
      ? Config::Q1_SHADOW_GAIN_PHYSICAL_PLUS_DEG_PER_MAS
      : Config::Q1_SHADOW_GAIN_PHYSICAL_MINUS_DEG_PER_MAS;

  uint8_t reason = Config::Q1_SHADOW_INVALID_NONE;
  if (!isfinite(event.zero_cross_abs_rate_dps) || !isfinite(event.q1_baseline_next_peak_abs_deg) ||
      !isfinite(event.target_next_peak_abs_deg) || !isfinite(event.q1_gain_deg_per_mA_s) ||
      event.q1_gain_deg_per_mA_s <= 0.0f) {
    reason = Config::Q1_SHADOW_INVALID_NONFINITE_STATE;
  } else if (event.zero_cross_abs_rate_dps < Config::Q1_SHADOW_RATE_SUPPORT_MIN_DPS) {
    reason = Config::Q1_SHADOW_INVALID_RATE_BELOW_SUPPORT;
  } else if (event.zero_cross_abs_rate_dps > Config::Q1_SHADOW_RATE_SUPPORT_MAX_DPS) {
    reason = Config::Q1_SHADOW_INVALID_RATE_ABOVE_SUPPORT;
  } else {
    event.delta_peak_required_deg = event.target_next_peak_abs_deg -
        event.q1_baseline_next_peak_abs_deg;
    if (event.delta_peak_required_deg <= 0.0f) {
      reason = Config::Q1_SHADOW_INVALID_BRAKING_NOT_IDENTIFIED;
    } else {
      event.q_req_shadow_mA_s = event.delta_peak_required_deg /
          event.q1_gain_deg_per_mA_s;
      // Q_req uses the exact canonical q_target_mA_s coordinate; the same
      // value is logged under its model-axis name for independent replay.
      event.q_model_axis_mA_s = event.q_req_shadow_mA_s;
      if (!isfinite(event.q_req_shadow_mA_s)) {
        reason = Config::Q1_SHADOW_INVALID_NONFINITE_STATE;
      } else if (event.q_req_shadow_mA_s < Config::Q1_SHADOW_Q_SUPPORT_MIN_MAS) {
        reason = Config::Q1_SHADOW_INVALID_Q_BELOW_SUPPORT;
      } else if (event.q_req_shadow_mA_s > Config::Q1_SHADOW_Q_SUPPORT_MAX_MAS) {
        reason = Config::Q1_SHADOW_INVALID_Q_ABOVE_SUPPORT;
      }
    }
  }
  event.q1_shadow_valid = reason == Config::Q1_SHADOW_INVALID_NONE;
  event.q1_shadow_invalid_reason = reason;
  logger_->addQ1ShadowEvent(event);
}
void ExperimentRunner::resetQIdentTracker() {
  q_ident_armed_ = false;
  q_ident_pulse_authorized_ = false;
  q_ident_arm_consecutive_count_ = 0;
  q_ident_last_arm_side_ = 0;
  for (uint8_t i = 0; i < Config::Q_IDENT_SIDE_COUNT; ++i) {
    q_ident_side_occurrence_count_[i] = 0;
  }
}

bool ExperimentRunner::qIdentRateInSupport(float abs_rate_dps) const {
  return isfinite(abs_rate_dps) && abs_rate_dps >= Config::Q_IDENT_RATE_SUPPORT_MIN_DPS &&
      abs_rate_dps <= Config::Q_IDENT_RATE_SUPPORT_MAX_DPS;
}

float ExperimentRunner::qIdentRequiredWidthMs(float q_target_mA_s, float signed_i0_mA,
                                               int8_t direction) const {
  if (!isfinite(q_target_mA_s) || !isfinite(signed_i0_mA) || q_target_mA_s <= 0.0f ||
      (direction != -1 && direction != 1)) return NAN;
  const float upper_ms = static_cast<float>(Config::Q_IDENT_SOLVER_DIAGNOSTIC_MAX_WIDTH_MS);
  const float q_at_upper = fabsf(identificationChargeMaS(signed_i0_mA, direction, upper_ms));
  if (!isfinite(q_at_upper) || q_at_upper + 0.0001f < q_target_mA_s) return NAN;
  float lo_ms = 0.0f;
  float hi_ms = upper_ms;
  for (uint8_t i = 0; i < 32; ++i) {
    const float mid_ms = 0.5f * (lo_ms + hi_ms);
    if (fabsf(identificationChargeMaS(signed_i0_mA, direction, mid_ms)) < q_target_mA_s) {
      lo_ms = mid_ms;
    } else {
      hi_ms = mid_ms;
    }
  }
  return hi_ms;
}

bool ExperimentRunner::qIdentSolvePulse(float q_target_mA_s, float signed_i0_mA, int8_t direction,
                                        float* required_width_ms,
                                        uint16_t* selected_integer_width_ms, uint16_t* width_ms,
                                        float* q_effective_pred_mA_s) const {
  if (!required_width_ms || !selected_integer_width_ms || !width_ms || !q_effective_pred_mA_s ||
      !isfinite(q_target_mA_s) || !isfinite(signed_i0_mA) || q_target_mA_s <= 0.0f ||
      (direction != -1 && direction != 1)) return false;
  *required_width_ms = qIdentRequiredWidthMs(q_target_mA_s, signed_i0_mA, direction);
  *selected_integer_width_ms = isfinite(*required_width_ms)
      ? static_cast<uint16_t>(ceilf(*required_width_ms - 0.0001f)) : 0;
  if (!isfinite(*required_width_ms) || *selected_integer_width_ms < Config::Q_IDENT_MIN_PULSE_MS ||
      *selected_integer_width_ms > Config::Q_IDENT_MAX_PULSE_MS) return false;
  const float q_effective = fabsf(identificationChargeMaS(
      signed_i0_mA, direction, static_cast<float>(*selected_integer_width_ms)));
  if (!isfinite(q_effective)) return false;
  *width_ms = *selected_integer_width_ms;
  *q_effective_pred_mA_s = q_effective;
  return true;
}

bool ExperimentRunner::beginQIdentPulse(uint32_t now_ms, uint32_t t_test_ms, int8_t direction,
                                         uint16_t pulse_width_ms) {
  if (!q_ident_mode_ || !q_ident_armed_ || status_.emergency_stop ||
      status_.state != ExperimentState::RUNNING_BATCH_SWEEP || status_.pulse_active ||
      !roller_ || !roller_->ok() || (direction != -1 && direction != 1) ||
      pulse_width_ms < Config::Q_IDENT_MIN_PULSE_MS || pulse_width_ms > Config::Q_IDENT_MAX_PULSE_MS) {
    return false;
  }
  q_ident_pulse_authorized_ = true;
  const int16_t command_current_mA = static_cast<int16_t>(direction * Config::Q_IDENT_CURRENT_MA);
  if (!roller_->setCurrentMa(command_current_mA)) {
    q_ident_pulse_authorized_ = false;
    stopMotor();
    return false;
  }
  status_.current_mA_setting = Config::Q_IDENT_CURRENT_MA;
  status_.pulse_width_ms_setting = pulse_width_ms;
  status_.motor_cmd_mA = command_current_mA;
  status_.pulse_direction = direction;
  status_.pulse_active = true;
  status_.pulse_id++;
  active_pulse_start_ms_ = now_ms;
  active_pulse_start_test_ms_ = t_test_ms;
  const float i0 = predicted_current_end_ms_ == 0 ? 0.0f :
      predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  const float v = status_.beta_model_vbat_mV > 0 ? status_.beta_model_vbat_mV / 1000.0f :
      Config::MODEL_VBAT_REFERENCE_V;
  const float target_current_mA = direction * predictCurrentGoalMa(Config::Q_IDENT_CURRENT_MA, v);
  const float tau_s = predictRiseTauS(Config::Q_IDENT_CURRENT_MA);
  const float width_s = static_cast<float>(pulse_width_ms) / 1000.0f;
  predicted_signed_current_end_mA_ = target_current_mA +
      (i0 - target_current_mA) * expf(-width_s / tau_s);
  predicted_current_end_ms_ = now_ms + pulse_width_ms;
  updatePulseModelPrediction();
  return true;
}

void ExperimentRunner::updateQIdentPulse(uint32_t now_ms) {
  if (!q_ident_mode_) {
    stopMotor();
    return;
  }
  if (status_.pulse_active &&
      static_cast<uint32_t>(now_ms - active_pulse_start_ms_) >= status_.pulse_width_ms_setting) {
    stopActivePulse(now_ms);
    status_.current_mA_setting = 0;
    status_.pulse_width_ms_setting = 0;
  }
}

void ExperimentRunner::updateQIdentAtZeroCross(uint32_t t_test_ms, float rate_dps,
                                                const Q1ZeroCrossDiagnostics& diagnostics) {
  if (!q_ident_mode_ || !logger_) return;
  PsramLogger::QIdentEvent event;
  event.q_ident_run_schedule_id = q_ident_run_schedule_id_ + 1;
  event.zero_cross_time_ms = t_test_ms;
  event.zero_cross_rate_dps = rate_dps;
  event.zero_cross_abs_rate_dps = fabsf(rate_dps);
  event.interpolated_zero_cross_time_ms = diagnostics.interpolated_zero_cross_time_ms;
  event.interpolated_physical_roll_rate_dps = diagnostics.interpolated_physical_roll_rate_dps;
  event.detector_crossing_direction = diagnostics.detector_crossing_direction;
  event.detector_angle_before_deg = diagnostics.detector_angle_before_deg;
  event.detector_angle_after_deg = diagnostics.detector_angle_after_deg;
  event.sign_gate_passed = diagnostics.sign_gate_passed;
  event.physical_roll_abs_diag_deg = status_.physical_roll_abs_deg;
  event.current_roll_deg = status_.current_roll_deg;
  event.physical_roll_rate_dps = rate_dps;
  event.physical_next_peak_side = rate_dps >= 0.0f ? 1 : -1;
  event.vbat_mV = status_.roller_battery_mV;
  event.pulse_width_guard_max_ms = Config::Q_IDENT_MAX_PULSE_MS;
  const uint32_t now_ms = run_start_ms_ + t_test_ms;
  event.i0_estimated_mA = predicted_current_end_ms_ == 0 ? 0.0f :
      predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);

  if (!q_ident_armed_) {
    const bool qualifies = event.sign_gate_passed &&
        event.zero_cross_abs_rate_dps >= Config::Q_IDENT_ARM_MIN_ABS_RATE_DPS &&
        (q_ident_last_arm_side_ == 0 || event.physical_next_peak_side != q_ident_last_arm_side_);
    if (qualifies) {
      q_ident_arm_consecutive_count_++;
      q_ident_last_arm_side_ = event.physical_next_peak_side;
      if (q_ident_arm_consecutive_count_ >= Config::Q_IDENT_ARM_CONSECUTIVE_CROSSES) {
        q_ident_armed_ = true;
        event.q_ident_armed = true;
        event.arm_consecutive_count = q_ident_arm_consecutive_count_;
        event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_ARMED_EVENT_NO_OUTPUT;
      } else {
        event.arm_consecutive_count = q_ident_arm_consecutive_count_;
        event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_ARM_WAITING;
      }
    } else {
      q_ident_arm_consecutive_count_ = 0;
      q_ident_last_arm_side_ = 0;
      event.arm_consecutive_count = 0;
      event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_ARM_WAITING;
    }
    logger_->addQIdentEvent(event);
    return;
  }

  event.q_ident_armed = true;
  event.arm_consecutive_count = q_ident_arm_consecutive_count_;
  if (event.zero_cross_abs_rate_dps < Config::Q_IDENT_RATE_SUPPORT_MIN_DPS) {
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_RATE_BELOW_SUPPORT;
    logger_->addQIdentEvent(event);
    return;
  }
  if (event.zero_cross_abs_rate_dps > Config::Q_IDENT_RATE_SUPPORT_MAX_DPS) {
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_RATE_ABOVE_SUPPORT;
    logger_->addQIdentEvent(event);
    return;
  }

  const uint8_t side_index = event.physical_next_peak_side > 0 ? 0 : 1;
  if (q_ident_side_occurrence_count_[side_index] >= Config::Q_IDENT_OCCURRENCES_PER_SIDE) {
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_SCHEDULE_EXHAUSTED;
    logger_->addQIdentEvent(event);
    return;
  }
  const uint8_t occurrence = q_ident_side_occurrence_count_[side_index]++;
  const float q_target = Config::Q_IDENT_SCHEDULE_Q_MAS[q_ident_run_schedule_id_][side_index][occurrence];
  event.side_occurrence_index = occurrence + 1;
  event.q_schedule_target_mA_s = q_target;
  event.q_target_mA_s = q_target;
  event.q_command_direction = -event.physical_next_peak_side;

  bool q_level_valid = false;
  for (float level : Config::Q_IDENT_Q_LEVELS_MAS) {
    if (fabsf(q_target - level) <= 0.0001f) q_level_valid = true;
  }
  if (!q_level_valid || !isfinite(q_target)) {
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_SOLVER_NONFINITE;
    logger_->addQIdentEvent(event);
    return;
  }
  if (q_target == 0.0f) {
    event.q_ident_valid = true;
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_Q_ZERO_NO_PULSE;
    logger_->addQIdentEvent(event);
    return;
  }
  if (status_.emergency_stop || status_.state != ExperimentState::RUNNING_BATCH_SWEEP) {
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_ESTOP_OR_STATE;
    logger_->addQIdentEvent(event);
    return;
  }
  if (!roller_ || !roller_->ok()) {
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_ROLLER_NOT_READY;
    logger_->addQIdentEvent(event);
    return;
  }
  const uint16_t battery_mV = status_.roller_battery_mV;
  if (battery_mV < Config::Q_IDENT_BATTERY_MIN_MV || battery_mV > Config::Q_IDENT_BATTERY_MAX_MV) {
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_BATTERY_GUARD;
    logger_->addQIdentEvent(event);
    return;
  }
  const float signed_i0_mA = event.i0_estimated_mA;
  float solver_required_width_ms = NAN;
  uint16_t solver_selected_integer_width_ms = 0;
  uint16_t width_ms = 0;
  float q_effective_pred_mA_s = NAN;
  if (!qIdentSolvePulse(q_target, signed_i0_mA, event.q_command_direction,
                        &solver_required_width_ms, &solver_selected_integer_width_ms, &width_ms,
                        &q_effective_pred_mA_s)) {
    event.solver_required_width_ms = solver_required_width_ms;
    event.solver_selected_integer_width_ms = solver_selected_integer_width_ms;
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_PULSE_WIDTH_GUARD;
    logger_->addQIdentEvent(event);
    return;
  }
  event.solver_required_width_ms = solver_required_width_ms;
  event.solver_selected_integer_width_ms = solver_selected_integer_width_ms;
  event.command_current_mA = Config::Q_IDENT_CURRENT_MA;
  event.pulse_width_ms = width_ms;
  event.q_effective_pred_mA_s = q_effective_pred_mA_s;
  event.pulse_start_ms = t_test_ms;
  event.pulse_end_ms = t_test_ms + width_ms;
  if (!beginQIdentPulse(now_ms, t_test_ms, event.q_command_direction, width_ms)) {
    event.command_current_mA = 0;
    event.pulse_width_ms = 0;
    event.pulse_start_ms = 0;
    event.pulse_end_ms = 0;
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_CURRENT_WRITE_FAILED;
    logger_->addQIdentEvent(event);
    return;
  }
  event.q_ident_valid = true;
  event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_NONE;
  logger_->addQIdentEvent(event);
}
void ExperimentRunner::resetEnergyControlV0OutputGate() {
  energy_control_v0_output_gate_state_ =
      EnergyControlV0OutputGateState::WAIT_INITIAL_EXCURSION;
  energy_control_v0_previous_accepted_next_peak_side_ = 0;
  energy_control_v0_rearm_excursion_seen_ = false;
  energy_control_v0_max_abs_detector_excursion_deg_ = 0.0f;
}

void ExperimentRunner::updateEnergyControlV0OutputGate(float detector_relative_angle_deg) {
  if (!energy_control_v0_mode_ || !isfinite(detector_relative_angle_deg)) return;
  const float abs_detector_angle_deg = fabsf(detector_relative_angle_deg);
  energy_control_v0_max_abs_detector_excursion_deg_ = fmaxf(
      energy_control_v0_max_abs_detector_excursion_deg_, abs_detector_angle_deg);
  if (energy_control_v0_output_gate_state_ ==
      EnergyControlV0OutputGateState::WAIT_INITIAL_EXCURSION) {
    if (abs_detector_angle_deg >= Config::ENERGY_CONTROL_V0_REARM_EXCURSION_DEG) {
      energy_control_v0_rearm_excursion_seen_ = true;
      energy_control_v0_output_gate_state_ =
          EnergyControlV0OutputGateState::ARMED_FOR_ZERO_CROSS;
    }
    return;
  }
  if (energy_control_v0_output_gate_state_ !=
      EnergyControlV0OutputGateState::WAIT_OPPOSITE_EXCURSION) return;

  // Detector is anti-correlated with +gy.  The return excursion after side s
  // has detector sign -s; V0.2 accidentally accepted the opposite sign.
  const int8_t previous_physical_side = energy_control_v0_previous_accepted_next_peak_side_;
  if (previous_physical_side != 0 &&
      detector_relative_angle_deg * static_cast<float>(previous_physical_side) <=
          -Config::ENERGY_CONTROL_V0_REARM_EXCURSION_DEG) {
    energy_control_v0_rearm_excursion_seen_ = true;
    energy_control_v0_output_gate_state_ =
        EnergyControlV0OutputGateState::ARMED_FOR_ZERO_CROSS;
  }
}

void ExperimentRunner::disarmEnergyControlV0AfterAcceptedCross(
    int8_t physical_next_peak_side) {
  energy_control_v0_previous_accepted_next_peak_side_ = physical_next_peak_side;
  energy_control_v0_rearm_excursion_seen_ = false;
  energy_control_v0_max_abs_detector_excursion_deg_ = 0.0f;
  energy_control_v0_output_gate_state_ =
      EnergyControlV0OutputGateState::WAIT_OPPOSITE_EXCURSION;
}
void ExperimentRunner::updateEnergyControlV0AtZeroCross(
    uint32_t t_test_ms, float rate_dps, const Q1ZeroCrossDiagnostics& diagnostics) {
  (void)diagnostics;  // Logged by the parallel Q1 shadow event.
  if (!energy_control_v0_mode_ || !logger_) return;

  PsramLogger::EnergyControlV0Event event;
  event.zero_cross_time_ms = t_test_ms;
  event.zero_cross_rate_dps = rate_dps;
  event.zero_cross_abs_rate_dps = fabsf(rate_dps);
  event.physical_next_peak_side = rate_dps >= 0.0f ? 1 : -1;
  event.candidate_physical_next_peak_side = event.physical_next_peak_side;
  event.previous_accepted_physical_next_peak_side =
      energy_control_v0_previous_accepted_next_peak_side_;
  event.output_gate_state = static_cast<uint8_t>(energy_control_v0_output_gate_state_);
  event.rearm_excursion_seen = energy_control_v0_rearm_excursion_seen_;
  event.maximum_excursion_since_previous_cross_deg =
      energy_control_v0_max_abs_detector_excursion_deg_;
  event.rearm_threshold_deg = Config::ENERGY_CONTROL_V0_REARM_EXCURSION_DEG;
  event.q_command_direction = -event.physical_next_peak_side;

  auto recordEnergyControlV0Event = [&]() {
    if (!event.output_executed && !event.valid &&
        event.output_blocked_reason == Config::ENERGY_CONTROL_V0_INVALID_NONE) {
      event.output_blocked_reason = event.reason;
    }
    logger_->addEnergyControlV0Event(event);
  };

  // If the corresponding audit record cannot be retained, no actual V0 pulse
  // may occur.  This keeps overflow fail-closed rather than silently losing
  // causality between a motor action and its RWLOG decision.
  if (logger_->energyControlV0EventCapacityReached() ||
      logger_->q1ShadowEventCapacityReached()) {
    logger_->markEnergyControlV0EventOverflow();
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_EVENT_LOG_OVERFLOW;
    event.output_blocked_reason = event.reason;
    if (!logger_->energyControlV0EventCapacityReached()) {
      recordEnergyControlV0Event();
    }
    return;
  }

  if (energy_control_v0_output_gate_state_ ==
      EnergyControlV0OutputGateState::WAIT_INITIAL_EXCURSION) {
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_WAIT_INITIAL_EXCURSION;
    recordEnergyControlV0Event();
    return;
  }
  if (energy_control_v0_output_gate_state_ ==
      EnergyControlV0OutputGateState::WAIT_OPPOSITE_EXCURSION) {
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_WAIT_REARM_EXCURSION;
    recordEnergyControlV0Event();
    return;
  }
  event.side_alternation_passed =
      event.previous_accepted_physical_next_peak_side == 0 ||
      event.candidate_physical_next_peak_side ==
          -event.previous_accepted_physical_next_peak_side;
  if (!event.side_alternation_passed) {
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_NONALTERNATING_SIDE;
    recordEnergyControlV0Event();
    return;
  }
  event.output_authorized = true;
  // A physically accepted crossing is consumed even when it later coasts or
  // fails a hardware guard.  The following candidate must re-establish the
  // opposite-side excursion before any new output can be authorized.
  disarmEnergyControlV0AfterAcceptedCross(event.physical_next_peak_side);
  event.target_peak_abs_deg = Config::ENERGY_CONTROL_V0_TARGET_PEAK_DEG;
  event.vbat_mV = status_.roller_battery_mV;
  event.rate_support_status = event.zero_cross_abs_rate_dps < Config::Q1_SHADOW_RATE_SUPPORT_MIN_DPS ? 0 :
      (event.zero_cross_abs_rate_dps <= Config::Q1_SHADOW_RATE_SUPPORT_MAX_DPS ? 1 : 2);
  event.q1_gain_deg_per_mA_s = event.physical_next_peak_side > 0
      ? Config::Q1_SHADOW_GAIN_PHYSICAL_PLUS_DEG_PER_MAS
      : Config::Q1_SHADOW_GAIN_PHYSICAL_MINUS_DEG_PER_MAS;
  event.passive_next_peak_abs_deg = Config::Q1_SHADOW_INTERCEPT_DEG +
      Config::Q1_SHADOW_RATE_GAIN_DEG_PER_DPS * event.zero_cross_abs_rate_dps +
      Config::Q1_SHADOW_SIDE_TERM_DEG * static_cast<float>(event.physical_next_peak_side);

  const auto potentialJ = [](float amplitude_deg) -> float {
    const float amplitude_rad = fabsf(amplitude_deg) * 0.01745329251994329577f;
    const float theta_inner = asinf(Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M /
                                    Config::ENERGY_CONTROL_V0_RADIUS_M);
    const float theta_outer = asinf(Config::ENERGY_CONTROL_V0_OUTER_EDGE_X_M /
                                    Config::ENERGY_CONTROL_V0_RADIUS_M);
    if (!isfinite(amplitude_rad) || amplitude_rad > theta_outer) return NAN;
    const float center_height = sqrtf(Config::ENERGY_CONTROL_V0_RADIUS_M *
                                      Config::ENERGY_CONTROL_V0_RADIUS_M -
                                      Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M *
                                      Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M);
    const float height_m = amplitude_rad <= theta_inner
        ? Config::ENERGY_CONTROL_V0_CG_HEIGHT_M * cosf(amplitude_rad) +
              Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M * sinf(amplitude_rad)
        : Config::ENERGY_CONTROL_V0_RADIUS_M +
              (Config::ENERGY_CONTROL_V0_CG_HEIGHT_M - center_height) * cosf(amplitude_rad);
    return Config::ENERGY_CONTROL_V0_MASS_KG * Config::ENERGY_CONTROL_V0_GRAVITY_M_S2 *
        (height_m - Config::ENERGY_CONTROL_V0_CG_HEIGHT_M);
  };

  event.passive_energy_j = potentialJ(event.passive_next_peak_abs_deg);
  event.target_energy_j = potentialJ(event.target_peak_abs_deg);
  if (!isfinite(event.zero_cross_abs_rate_dps) || !isfinite(event.q1_gain_deg_per_mA_s) ||
      event.q1_gain_deg_per_mA_s <= 0.0f || !isfinite(event.passive_energy_j) ||
      !isfinite(event.target_energy_j)) {
    event.reason = !isfinite(event.passive_energy_j) || !isfinite(event.target_energy_j)
        ? Config::ENERGY_CONTROL_V0_INVALID_POTENTIAL_DOMAIN
        : Config::ENERGY_CONTROL_V0_INVALID_NONFINITE_STATE;
    recordEnergyControlV0Event();
    return;
  }

  event.delta_energy_required_j = event.target_energy_j - event.passive_energy_j;
  event.q_selected_mA_s = 0.0f;
  event.q_effective_pred_mA_s = 0.0f;
  event.predicted_next_peak_abs_deg = event.passive_next_peak_abs_deg;
  event.predicted_next_energy_j = event.passive_energy_j;
  event.q_support_status = 0;
  if (event.delta_energy_required_j <= 0.0f) {
    // Braking is not identified: coast safely and record a valid no-output event.
    event.valid = true;
    event.reason = Config::ENERGY_CONTROL_V0_VALID_PASSIVE_NO_OUTPUT;
    recordEnergyControlV0Event();
    return;
  }

  float best_q = 0.0f;
  float best_peak = event.passive_next_peak_abs_deg;
  float best_energy = event.passive_energy_j;
  float best_error = fabsf(event.target_energy_j - best_energy);
  const uint16_t steps = static_cast<uint16_t>(lroundf(
      Config::ENERGY_CONTROL_V0_Q_MAX_MAS / Config::ENERGY_CONTROL_V0_Q_SEARCH_STEP_MAS));
  for (uint16_t i = 1; i <= steps; ++i) {
    const float q = static_cast<float>(i) * Config::ENERGY_CONTROL_V0_Q_SEARCH_STEP_MAS;
    const float candidate_peak = event.passive_next_peak_abs_deg + event.q1_gain_deg_per_mA_s * q;
    const float candidate_energy = potentialJ(candidate_peak);
    if (!isfinite(candidate_energy)) break;
    const float error = fabsf(event.target_energy_j - candidate_energy);
    if (error < best_error) {
      best_q = q;
      best_peak = candidate_peak;
      best_energy = candidate_energy;
      best_error = error;
    }
  }
  event.q_selected_mA_s = best_q;
  event.predicted_next_peak_abs_deg = best_peak;
  event.predicted_next_energy_j = best_energy;
  event.q_saturated_at_max = best_q >= Config::ENERGY_CONTROL_V0_Q_MAX_MAS -
      0.5f * Config::ENERGY_CONTROL_V0_Q_SEARCH_STEP_MAS &&
      best_energy + 1.0e-9f < event.target_energy_j;
  event.q_support_status = best_q <= 0.0f ? 0 :
      (best_q < Config::Q1_SHADOW_Q_SUPPORT_MIN_MAS ? 1 :
       (best_q <= Config::Q1_SHADOW_Q_SUPPORT_MAX_MAS ? 2 : 3));
  if (best_q <= 0.0f) {
    event.valid = true;
    event.reason = Config::ENERGY_CONTROL_V0_VALID_PASSIVE_NO_OUTPUT;
    recordEnergyControlV0Event();
    return;
  }
  if (status_.emergency_stop || status_.state != ExperimentState::RUNNING_BATCH_SWEEP) {
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_ESTOP_OR_STATE;
    recordEnergyControlV0Event();
    return;
  }
  if (!roller_ || !roller_->ok()) {
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_ROLLER_NOT_READY;
    recordEnergyControlV0Event();
    return;
  }
  if (event.vbat_mV < Config::Q_IDENT_BATTERY_MIN_MV || event.vbat_mV > Config::Q_IDENT_BATTERY_MAX_MV) {
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_BATTERY_GUARD;
    recordEnergyControlV0Event();
    return;
  }

  const uint32_t now_ms = run_start_ms_ + t_test_ms;
  event.i0_estimated_mA = predicted_current_end_ms_ == 0 ? 0.0f :
      predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  float required_width_ms = NAN;
  uint16_t selected_integer_width_ms = 0;
  uint16_t width_ms = 0;
  float q_effective_pred_mA_s = NAN;
  if (!qIdentSolvePulse(best_q, event.i0_estimated_mA, event.q_command_direction,
                         &required_width_ms, &selected_integer_width_ms, &width_ms,
                         &q_effective_pred_mA_s)) {
    event.solver_required_width_ms = required_width_ms;
    event.solver_selected_integer_width_ms = selected_integer_width_ms;
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_PULSE_WIDTH_GUARD;
    recordEnergyControlV0Event();
    return;
  }
  event.solver_required_width_ms = required_width_ms;
  event.solver_selected_integer_width_ms = selected_integer_width_ms;
  event.q_effective_pred_mA_s = q_effective_pred_mA_s;
  event.predicted_next_peak_abs_deg = event.passive_next_peak_abs_deg +
      event.q1_gain_deg_per_mA_s * q_effective_pred_mA_s;
  event.predicted_next_energy_j = potentialJ(event.predicted_next_peak_abs_deg);
  event.command_current_mA = Config::Q_IDENT_CURRENT_MA;
  event.pulse_width_ms = width_ms;
  event.pulse_start_ms = t_test_ms;
  event.pulse_end_ms = t_test_ms + width_ms;
  if (!beginEnergyControlV0Pulse(now_ms, t_test_ms, event.q_command_direction, width_ms)) {
    event.command_current_mA = 0;
    event.pulse_width_ms = 0;
    event.pulse_start_ms = 0;
    event.pulse_end_ms = 0;
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_CURRENT_WRITE_FAILED;
    recordEnergyControlV0Event();
    return;
  }
  event.output_executed = true;
  event.valid = true;
  event.reason = Config::ENERGY_CONTROL_V0_INVALID_NONE;
  recordEnergyControlV0Event();
}

bool ExperimentRunner::beginEnergyControlV0Pulse(uint32_t now_ms, uint32_t t_test_ms,
                                                   int8_t direction, uint16_t pulse_width_ms) {
  if (!energy_control_v0_mode_ || status_.emergency_stop ||
      status_.state != ExperimentState::RUNNING_BATCH_SWEEP || status_.pulse_active ||
      !roller_ || !roller_->ok() || (direction != -1 && direction != 1) ||
      pulse_width_ms < Config::Q_IDENT_MIN_PULSE_MS || pulse_width_ms > Config::Q_IDENT_MAX_PULSE_MS) {
    return false;
  }
  energy_control_v0_pulse_authorized_ = true;
  const int16_t command_current_mA = static_cast<int16_t>(direction * Config::Q_IDENT_CURRENT_MA);
  if (!roller_->setCurrentMa(command_current_mA)) {
    energy_control_v0_pulse_authorized_ = false;
    stopMotor();
    return false;
  }
  status_.current_mA_setting = Config::Q_IDENT_CURRENT_MA;
  status_.pulse_width_ms_setting = pulse_width_ms;
  status_.motor_cmd_mA = command_current_mA;
  status_.pulse_direction = direction;
  status_.pulse_active = true;
  status_.pulse_id++;
  active_pulse_start_ms_ = now_ms;
  active_pulse_start_test_ms_ = t_test_ms;
  const float i0 = predicted_current_end_ms_ == 0 ? 0.0f :
      predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  const float v = status_.beta_model_vbat_mV > 0 ? status_.beta_model_vbat_mV / 1000.0f :
      Config::MODEL_VBAT_REFERENCE_V;
  const float target_current_mA = direction * predictCurrentGoalMa(Config::Q_IDENT_CURRENT_MA, v);
  const float tau_s = predictRiseTauS(Config::Q_IDENT_CURRENT_MA);
  const float width_s = static_cast<float>(pulse_width_ms) / 1000.0f;
  predicted_signed_current_end_mA_ = target_current_mA +
      (i0 - target_current_mA) * expf(-width_s / tau_s);
  predicted_current_end_ms_ = now_ms + pulse_width_ms;
  updatePulseModelPrediction();
  return true;
}

void ExperimentRunner::updateEnergyControlV0Pulse(uint32_t now_ms) {
  if (!energy_control_v0_mode_) {
    stopMotor();
    return;
  }
  if (status_.pulse_active &&
      static_cast<uint32_t>(now_ms - active_pulse_start_ms_) >= status_.pulse_width_ms_setting) {
    stopActivePulse(now_ms);
    status_.current_mA_setting = 0;
    status_.pulse_width_ms_setting = 0;
  }
}
void ExperimentRunner::resetEnergyControlAutonomous() {
  energy_control_autonomous_pulse_authorized_ = false;
  energy_control_autonomous_phase_ = EnergyControlAutonomousPhase::IDLE;
  energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::WAIT_PEAK;
  energy_control_autonomous_integral_plus_mA_s_ = 0.0f;
  energy_control_autonomous_integral_minus_mA_s_ = 0.0f;
  energy_control_autonomous_gyro_integrator_ready_ = false;
  energy_control_autonomous_last_gyro_sample_us_ = 0;
  energy_control_autonomous_last_gyro_rate_dps_ = 0.0f;
  energy_control_autonomous_gyro_relative_deg_ = 0.0f;
  energy_control_autonomous_detector_has_previous_angle_ = false;
  energy_control_autonomous_detector_zero_angle_deg_ =
      status_.pitch_dynamic_beta_deg[Config::FILTER_ADOPTED_INDEX];
  if (!isfinite(energy_control_autonomous_detector_zero_angle_deg_)) {
    energy_control_autonomous_detector_zero_angle_deg_ = 0.0f;
  }
  energy_control_autonomous_previous_detector_relative_angle_deg_ = 0.0f;
  energy_control_autonomous_previous_detector_rate_dps_ = 0.0f;
  energy_control_autonomous_previous_detector_test_ms_ = 0;
  energy_control_autonomous_zero_cross_consumed_for_peak_ = false;
  energy_control_autonomous_last_accepted_zero_cross_valid_ = false;
  energy_control_autonomous_last_accepted_zero_cross_ms_ = 0;
  resetEnergyControlAutonomousPeakTracker(false);
  energy_control_autonomous_last_peak_valid_ = false;
  energy_control_autonomous_last_peak_amplitude_deg_ = NAN;
  energy_control_autonomous_last_peak_side_ = 0;
  energy_control_autonomous_last_peak_ms_ = 0;
  energy_control_autonomous_pending_peak_ = false;
  energy_control_autonomous_pending_next_side_ = 0;
  energy_control_autonomous_pending_q_command_mA_s_ = 0.0f;
  energy_control_autonomous_pending_saturated_upper_ = false;
  energy_control_autonomous_pending_saturated_lower_ = false;
  energy_control_autonomous_pending_zero_event_index_ = 0;
}

void ExperimentRunner::resetEnergyControlAutonomousPeakTracker(bool enable) {
  energy_control_autonomous_peak_tracker_enabled_ = enable;
  energy_control_autonomous_peak_tracker_started_ = false;
  energy_control_autonomous_candidate_detector_side_ = 0;
  energy_control_autonomous_return_samples_ = 0;
  energy_control_autonomous_candidate_detector_peak_abs_deg_ = 0.0f;
  energy_control_autonomous_candidate_peak_amplitude_deg_ = 0.0f;
  energy_control_autonomous_candidate_peak_ms_ = 0;
}
float ExperimentRunner::energyControlPotentialJ(float amplitude_deg) const {
  const float amplitude_rad = fabsf(amplitude_deg) * 0.01745329251994329577f;
  const float theta_inner = asinf(Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M /
                                  Config::ENERGY_CONTROL_V0_RADIUS_M);
  const float theta_outer = asinf(Config::ENERGY_CONTROL_V0_OUTER_EDGE_X_M /
                                  Config::ENERGY_CONTROL_V0_RADIUS_M);
  if (!isfinite(amplitude_rad) || amplitude_rad > theta_outer) return NAN;
  const float center_height = sqrtf(Config::ENERGY_CONTROL_V0_RADIUS_M *
                                    Config::ENERGY_CONTROL_V0_RADIUS_M -
                                    Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M *
                                    Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M);
  const float height_m = amplitude_rad <= theta_inner
      ? Config::ENERGY_CONTROL_V0_CG_HEIGHT_M * cosf(amplitude_rad) +
            Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M * sinf(amplitude_rad)
      : Config::ENERGY_CONTROL_V0_RADIUS_M +
            (Config::ENERGY_CONTROL_V0_CG_HEIGHT_M - center_height) * cosf(amplitude_rad);
  return Config::ENERGY_CONTROL_V0_MASS_KG * Config::ENERGY_CONTROL_V0_GRAVITY_M_S2 *
      (height_m - Config::ENERGY_CONTROL_V0_CG_HEIGHT_M);
}

float ExperimentRunner::energyControlAutonomousFreeNextPeakAmplitude(float amplitude_deg) const {
  const float current_energy = energyControlPotentialJ(amplitude_deg);
  if (!isfinite(current_energy) || current_energy < 0.0f) return NAN;
  const float next_energy = Config::ENERGY_CONTROL_AUTONOMOUS_P1_FREE_DECAY_ALPHA * current_energy -
      Config::ENERGY_CONTROL_AUTONOMOUS_P1_FREE_DECAY_EC_J;
  if (!isfinite(next_energy)) return NAN;
  if (next_energy <= 0.0f) return 0.0f;
  const float maximum_deg = asinf(Config::ENERGY_CONTROL_V0_OUTER_EDGE_X_M /
                                  Config::ENERGY_CONTROL_V0_RADIUS_M) * 57.295779513082320876f;
  const float maximum_energy = energyControlPotentialJ(maximum_deg);
  if (!isfinite(maximum_energy) || next_energy > maximum_energy) return NAN;
  float lo = 0.0f;
  float hi = maximum_deg;
  for (uint8_t i = 0; i < 32; ++i) {
    const float mid = 0.5f * (lo + hi);
    const float mid_energy = energyControlPotentialJ(mid);
    if (!isfinite(mid_energy)) return NAN;
    if (mid_energy < next_energy) lo = mid; else hi = mid;
  }
  return 0.5f * (lo + hi);
}
float ExperimentRunner::energyControlAutonomousGainForSide(int8_t physical_side) const {
  return physical_side > 0 ? Config::Q1_SHADOW_GAIN_PHYSICAL_PLUS_DEG_PER_MAS :
      Config::Q1_SHADOW_GAIN_PHYSICAL_MINUS_DEG_PER_MAS;
}

void ExperimentRunner::energyControlAutonomousCorrectionParameters(
    int8_t physical_side, float* c_side_used_deg, float* g_side_corrected_deg_per_mA_s) const {
  const float base_gain = energyControlAutonomousGainForSide(physical_side);
  float c_used_deg = 0.0f;
  float g_used_deg_per_mA_s = base_gain;
  if (Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_CORRECTION_ENABLED) {
    const float c_fit_deg = physical_side > 0
        ? Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_C_PLUS_DEG
        : Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_C_MINUS_DEG;
    const float g_fit_deg_per_mA_s = physical_side > 0
        ? Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_G_PLUS_DEG_PER_MAS
        : Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_G_MINUS_DEG_PER_MAS;
    c_used_deg = Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_BLEND_LAMBDA * c_fit_deg;
    g_used_deg_per_mA_s = base_gain +
        Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_BLEND_LAMBDA *
            (g_fit_deg_per_mA_s - base_gain);
  }
  if (c_side_used_deg) *c_side_used_deg = c_used_deg;
  if (g_side_corrected_deg_per_mA_s) *g_side_corrected_deg_per_mA_s = g_used_deg_per_mA_s;
}

float ExperimentRunner::energyControlAutonomousCorrectedPrediction(
    float free_next_peak_deg, int8_t physical_side, float q_mA_s, float* correction_deg) const {
  const float base_gain = energyControlAutonomousGainForSide(physical_side);
  float c_used_deg = 0.0f;
  float g_used_deg_per_mA_s = base_gain;
  energyControlAutonomousCorrectionParameters(physical_side, &c_used_deg, &g_used_deg_per_mA_s);
  const float base_prediction_deg = free_next_peak_deg + base_gain * q_mA_s;
  const float raw_correction_deg = c_used_deg + (g_used_deg_per_mA_s - base_gain) * q_mA_s;
  const float bounded_correction_deg = fmaxf(-Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_MAX_ABS_DEG,
      fminf(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_MAX_ABS_DEG, raw_correction_deg));
  if (correction_deg) *correction_deg = bounded_correction_deg;
  return fmaxf(0.0f, base_prediction_deg + bounded_correction_deg);
}

bool ExperimentRunner::beginEnergyControlAutonomousPulse(uint32_t now_ms, uint32_t t_test_ms,
                                                            int8_t direction, uint16_t pulse_width_ms) {
  if (!energy_control_autonomous_mode_ || status_.emergency_stop ||
      status_.state != ExperimentState::RUNNING_BATCH_SWEEP || status_.pulse_active ||
      !roller_ || !roller_->ok() || (direction != -1 && direction != 1) ||
      pulse_width_ms < 1 || pulse_width_ms > Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS ||
      (energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::ENERGY_CONTROL &&
       energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::HOLD) ||
      energy_control_autonomous_half_cycle_state_ !=
          EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS) {
    return false;
  }
  energy_control_autonomous_pulse_authorized_ = true;
  const int16_t command_current_mA = static_cast<int16_t>(
      direction * Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA);
  // V50_OBSERVER_BEGIN: copies only; no observer value is read by control.
  {
    auto& context = roller_->qObserver().pending;
    context = QObserver::Context{};
    context.target = status_.current_audit_q_target_mA_s;
    context.measured_stop_enabled = true; // V51: normal autonomous pulses only.
    context.pred = status_.current_audit_q_pred_mA_s;
    context.width_ms = pulse_width_ms;
    context.battery_mV = status_.beta_model_vbat_mV;
    context.goal_dir = predictCurrentGoalMa(Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA,
        context.battery_mV > 0 ? context.battery_mV / 1000.0f : Config::MODEL_VBAT_REFERENCE_V);
    context.tau_s = predictRiseTauS(Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA);
    context.physical_side = energy_control_autonomous_pending_next_side_;
    context.saturated = energy_control_autonomous_pending_saturated_upper_;
    context.initial_model_raw_mA = predicted_current_end_ms_ == 0 ? 0.0f :
        predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  }
  // V50_OBSERVER_END
  if (!roller_->setCurrentMa(command_current_mA)) {
    energy_control_autonomous_pulse_authorized_ = false;
    stopMotor();
    return false;
  }
  status_.current_mA_setting = Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA;
  status_.pulse_width_ms_setting = pulse_width_ms;
  status_.motor_cmd_mA = command_current_mA;
  status_.pulse_direction = direction;
  status_.pulse_active = true;
  energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::PULSE_ACTIVE;
  status_.pulse_id++;
  active_pulse_start_ms_ = now_ms;
  active_pulse_start_test_ms_ = t_test_ms;
  const float i0 = predicted_current_end_ms_ == 0 ? 0.0f :
      predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  const float v = status_.beta_model_vbat_mV > 0 ? status_.beta_model_vbat_mV / 1000.0f :
      Config::MODEL_VBAT_REFERENCE_V;
  const float target_current_mA = direction * predictCurrentGoalMa(
      Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA, v);
  const float tau_s = predictRiseTauS(Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA);
  const float width_s = static_cast<float>(pulse_width_ms) / 1000.0f;
  predicted_signed_current_end_mA_ = target_current_mA +
      (i0 - target_current_mA) * expf(-width_s / tau_s);
  predicted_current_end_ms_ = now_ms + pulse_width_ms;
  updatePulseModelPrediction();
  return true;
}
bool ExperimentRunner::beginEnergyControlAutonomousStartKickPulse(uint32_t now_ms, int8_t direction) {
  if (!energy_control_autonomous_mode_ ||
      energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::STRONG_START_KICK ||
      status_.emergency_stop || status_.state != ExperimentState::RUNNING_BATCH_SWEEP ||
      status_.pulse_active || !roller_ || !roller_->ok() ||
      direction != Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_DIRECTION) return false;
  energy_control_autonomous_pulse_authorized_ = true;
  const int16_t command_current_mA = static_cast<int16_t>(
      direction * Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA);
  // V50_OBSERVER_BEGIN: START_KICK deliberately has no Q_target or Q_pred.
  {
    auto& context = roller_->qObserver().pending;
    context = QObserver::Context{};
    context.width_ms = Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS;
    context.battery_mV = status_.beta_model_vbat_mV;
    context.goal_dir = predictCurrentGoalMa(Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA,
        context.battery_mV > 0 ? context.battery_mV / 1000.0f : Config::MODEL_VBAT_REFERENCE_V);
    context.tau_s = predictRiseTauS(Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA);
  }
  // V50_OBSERVER_END
  if (!roller_->setCurrentMa(command_current_mA)) {
    energy_control_autonomous_pulse_authorized_ = false;
    stopMotor();
    return false;
  }
  status_.current_mA_setting = Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA;
  status_.pulse_width_ms_setting = Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS;
  status_.motor_cmd_mA = command_current_mA;
  status_.pulse_direction = direction;
  status_.pulse_active = true;
  status_.pulse_id++;
  active_pulse_start_ms_ = now_ms;
  active_pulse_start_test_ms_ = 0;
  const float i0 = predicted_current_end_ms_ == 0 ? 0.0f :
      predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  const float v = status_.beta_model_vbat_mV > 0 ? status_.beta_model_vbat_mV / 1000.0f :
      Config::MODEL_VBAT_REFERENCE_V;
  const float target_current_mA = direction * predictCurrentGoalMa(
      Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA, v);
  const float tau_s = predictRiseTauS(Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA);
  const float width_s = static_cast<float>(Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS) / 1000.0f;
  predicted_signed_current_end_mA_ = target_current_mA +
      (i0 - target_current_mA) * expf(-width_s / tau_s);
  predicted_current_end_ms_ = now_ms + Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS;
  updatePulseModelPrediction();
  return true;
}
void ExperimentRunner::beginEnergyControlAutonomousStartKick(uint32_t now_ms) {
  if (!energy_control_autonomous_mode_ || !logger_) return;
  // START_KICK is deliberately outside the V7 Q selector, so it has no
  // Q_target/Q_pred label in the current audit.
  status_.current_audit_q_target_mA_s = NAN;
  status_.current_audit_q_pred_mA_s = NAN;
  energy_control_autonomous_phase_ = EnergyControlAutonomousPhase::STRONG_START_KICK;
  PsramLogger::EnergyControlAutonomousZeroCrossEvent event;
  event.event_kind = 1;
  event.zero_cross_time_ms = 0;
  event.phase = static_cast<uint8_t>(energy_control_autonomous_phase_);
  event.target_peak_deg = energy_control_autonomous_target_peak_deg_;
  event.physical_next_peak_side = 1;
  event.q_command_direction = Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_DIRECTION;
  event.vbat_mV = status_.roller_battery_mV;
  event.command_current_mA = Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA;
  event.pulse_width_ms = Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS;
  event.pulse_start_ms = 0;
  event.pulse_end_ms = Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS;
  if (logger_->energyControlAutonomousEventCapacityReached()) {
    logger_->markEnergyControlAutonomousEventOverflow();
  }
  if (status_.emergency_stop || status_.state != ExperimentState::RUNNING_BATCH_SWEEP) {
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_ESTOP_OR_STATE;
  } else if (!roller_ || !roller_->ok()) {
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_ROLLER_NOT_READY;
  } else if (!beginEnergyControlAutonomousStartKickPulse(now_ms,
               Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_DIRECTION)) {
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_CURRENT_WRITE_FAILED;
  } else {
    event.output_executed = true;
    event.valid = true;
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_STRONG_START_KICK_EXECUTED;
  }
  logger_->addEnergyControlAutonomousZeroCrossEvent(event);
  if (!event.output_executed) {
    energy_control_autonomous_phase_ = EnergyControlAutonomousPhase::STOP;
    status_.last_error = "energy_strong_start_kick_failed";
  }
}
void ExperimentRunner::updateEnergyControlAutonomousPulse(uint32_t now_ms) {
  if (!energy_control_autonomous_mode_) { stopMotor(); return; }
  if (!status_.pulse_active) return;
  if (roller_ && roller_->measuredQStopEnabled()) {
    roller_->serviceMeasuredQStop();
    if (roller_->currentCommandActive()) return; // Never stop at planned width.
  } else if (static_cast<uint32_t>(now_ms - active_pulse_start_ms_) <
             status_.pulse_width_ms_setting) return;
  const bool completed_start_kick =
      energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::STRONG_START_KICK;
  const bool completed_normal_pulse =
      energy_control_autonomous_half_cycle_state_ == EnergyControlAutonomousHalfCycleState::PULSE_ACTIVE;
  if (completed_start_kick && roller_)
    roller_->stop(MeasuredQStop::FIXED_START_KICK_COMPLETE);
  const auto q_stop_reason = roller_ ? roller_->measuredQStopReason() : MeasuredQStop::NONE;
  stopActivePulse(now_ms);
  status_.current_mA_setting = 0;
  status_.pulse_width_ms_setting = 0;
  energy_control_autonomous_pulse_authorized_ = false;
  if (completed_normal_pulse && q_stop_reason != MeasuredQStop::Q_TARGET_REACHED &&
      q_stop_reason != MeasuredQStop::Q_TARGET_NOT_REACHED &&
      q_stop_reason != MeasuredQStop::HARD_WIDTH_LIMIT) {
    requestEmergencyStop(MeasuredQStop::name(q_stop_reason));
    return;
  }
  if (completed_start_kick) {
    energy_control_autonomous_phase_ = EnergyControlAutonomousPhase::WAIT_FIRST_PEAK;
    energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::WAIT_PEAK;
    resetEnergyControlAutonomousPeakTracker(true);
  } else if (completed_normal_pulse) {
    // Do not carry a detector extremum formed by the 100 ms pulse transient
    // into the next physical half-cycle.
    energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::WAIT_PEAK;
    resetEnergyControlAutonomousPeakTracker(true);
  }
}
bool ExperimentRunner::recordEnergyControlAutonomousPeak(uint32_t peak_ms, int8_t physical_side,
                                                            float amplitude_deg, float detector_peak_angle_deg) {
  if (!logger_ || physical_side == 0 || !isfinite(amplitude_deg) || !isfinite(detector_peak_angle_deg) ||
      (energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::WAIT_FIRST_PEAK &&
       energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::ENERGY_CONTROL &&
       energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::HOLD) ||
      status_.pulse_active ||
      energy_control_autonomous_half_cycle_state_ != EnergyControlAutonomousHalfCycleState::WAIT_PEAK) return false;
  const bool first_peak = energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::WAIT_FIRST_PEAK;
  if (!first_peak && energy_control_autonomous_last_accepted_zero_cross_valid_ &&
      static_cast<uint32_t>(peak_ms - energy_control_autonomous_last_accepted_zero_cross_ms_) <
          Config::ENERGY_CONTROL_AUTONOMOUS_MIN_ZERO_TO_PEAK_MS) return false;
  if (logger_->energyControlAutonomousEventCapacityReached()) {
    // Logging capacity is diagnostic-only. The physical half-cycle controller
    // continues and the timeseries remains available.
    logger_->markEnergyControlAutonomousEventOverflow();
  }
  PsramLogger::EnergyControlAutonomousPeakEvent event;
  event.peak_time_ms = peak_ms;
  event.physical_peak_side = physical_side;
  event.peak_amplitude_deg = amplitude_deg;
  event.detector_peak_angle_deg = detector_peak_angle_deg;
  event.target_peak_deg = energy_control_autonomous_target_peak_deg_;
  event.peak_error_deg = event.target_peak_deg - event.peak_amplitude_deg;
  event.first_peak = first_peak;
  event.pending_command_matched = energy_control_autonomous_pending_peak_ &&
      physical_side == energy_control_autonomous_pending_next_side_;
  event.pending_q_command_mA_s = event.pending_command_matched
      ? energy_control_autonomous_pending_q_command_mA_s_ : NAN;
  event.antiwindup_upper_hold = event.pending_command_matched &&
      energy_control_autonomous_pending_saturated_upper_ && event.peak_error_deg > 0.0f;
  event.antiwindup_lower_hold = event.pending_command_matched &&
      energy_control_autonomous_pending_saturated_lower_ && event.peak_error_deg < 0.0f;
  if (!event.antiwindup_upper_hold && !event.antiwindup_lower_hold) {
    float* integral = physical_side > 0 ? &energy_control_autonomous_integral_plus_mA_s_ :
        &energy_control_autonomous_integral_minus_mA_s_;
    *integral += Config::ENERGY_CONTROL_AUTONOMOUS_INTEGRAL_KI_MAS_PER_DEG * event.peak_error_deg;
  }
  energy_control_autonomous_pending_peak_ = false;
  energy_control_autonomous_last_peak_valid_ = true;
  energy_control_autonomous_last_peak_amplitude_deg_ = amplitude_deg;
  energy_control_autonomous_last_peak_side_ = physical_side;
  energy_control_autonomous_last_peak_ms_ = peak_ms;
  energy_control_autonomous_zero_cross_consumed_for_peak_ = false;
  energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS;
  resetEnergyControlAutonomousPeakTracker(false);
  energy_control_autonomous_phase_ = fabsf(event.peak_error_deg) <= Config::TARGET_TOLERANCE_DEG
      ? EnergyControlAutonomousPhase::HOLD : EnergyControlAutonomousPhase::ENERGY_CONTROL;
  event.phase = static_cast<uint8_t>(energy_control_autonomous_phase_);
  event.integral_plus_mA_s = energy_control_autonomous_integral_plus_mA_s_;
  event.integral_minus_mA_s = energy_control_autonomous_integral_minus_mA_s_;
  logger_->addEnergyControlAutonomousPeakEvent(event);
  return true;
}
void ExperimentRunner::updateEnergyControlAutonomousMotion(uint32_t now_ms) {
  if (!energy_control_autonomous_mode_ ||
      energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::IDLE ||
      energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::STOP || !imu_) return;
  const float rate_dps = status_.physical_roll_rate_dps;
  const uint32_t sample_us = imu_->reading().last_update_us;
  if (!isfinite(rate_dps) || sample_us == 0) return;
  if (!energy_control_autonomous_gyro_integrator_ready_) {
    energy_control_autonomous_gyro_integrator_ready_ = true;
    energy_control_autonomous_last_gyro_sample_us_ = sample_us;
    energy_control_autonomous_last_gyro_rate_dps_ = rate_dps;
    return;
  }
  const uint32_t dt_us = sample_us - energy_control_autonomous_last_gyro_sample_us_;
  if (dt_us > 0 && dt_us <= Config::ENERGY_CONTROL_AUTONOMOUS_GYRO_INTEGRATION_MAX_DT_US) {
    energy_control_autonomous_gyro_relative_deg_ +=
        Config::ENERGY_CONTROL_AUTONOMOUS_GYRO_TO_VIDEO_PEAK_SCALE *
        0.5f * (energy_control_autonomous_last_gyro_rate_dps_ + rate_dps) *
        static_cast<float>(dt_us) * 1.0e-6f;
  }
  energy_control_autonomous_last_gyro_sample_us_ = sample_us;
  energy_control_autonomous_last_gyro_rate_dps_ = rate_dps;
  if (!isfinite(energy_control_autonomous_gyro_relative_deg_)) {
    energy_control_autonomous_phase_ = EnergyControlAutonomousPhase::STOP;
    stopMotor();
    status_.last_error = "energy_nonfinite_gyro_coordinate";
    return;
  }
  const uint32_t t_test_ms = now_ms - run_start_ms_;
  const float detector_relative_angle_deg = status_.pitch_dynamic_beta_deg[Config::FILTER_ADOPTED_INDEX] -
      energy_control_autonomous_detector_zero_angle_deg_;
  if (!isfinite(detector_relative_angle_deg)) return;
  if (!energy_control_autonomous_detector_has_previous_angle_) {
    energy_control_autonomous_detector_has_previous_angle_ = true;
    energy_control_autonomous_previous_detector_relative_angle_deg_ = detector_relative_angle_deg;
    energy_control_autonomous_previous_detector_rate_dps_ = rate_dps;
    energy_control_autonomous_previous_detector_test_ms_ = t_test_ms;
    if (!status_.pulse_active &&
        energy_control_autonomous_half_cycle_state_ == EnergyControlAutonomousHalfCycleState::WAIT_PEAK) {
      updateEnergyControlAutonomousPeakTracker(now_ms, detector_relative_angle_deg, rate_dps);
    }
    return;
  }
  const float before_deg = energy_control_autonomous_previous_detector_relative_angle_deg_;
  const bool crossing = (before_deg < 0.0f && detector_relative_angle_deg >= 0.0f) ||
      (before_deg > 0.0f && detector_relative_angle_deg <= 0.0f);
  const bool physical_event_suppressed = status_.pulse_active ||
      energy_control_autonomous_half_cycle_state_ == EnergyControlAutonomousHalfCycleState::PULSE_ACTIVE;
  // IMU, Madgwick, gyro integration, and detector history stay live during a
  // pulse. Only promotion to a physical control event is suppressed.
  if (!physical_event_suppressed &&
      energy_control_autonomous_half_cycle_state_ == EnergyControlAutonomousHalfCycleState::WAIT_PEAK) {
    updateEnergyControlAutonomousPeakTracker(now_ms, detector_relative_angle_deg, rate_dps);
  }
  if (!physical_event_suppressed && crossing &&
      energy_control_autonomous_half_cycle_state_ == EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS &&
      (energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::ENERGY_CONTROL ||
       energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::HOLD) &&
      energy_control_autonomous_last_peak_valid_ &&
      !energy_control_autonomous_zero_cross_consumed_for_peak_) {
    const float denominator = fabsf(before_deg) + fabsf(detector_relative_angle_deg);
    const float alpha = denominator > 0.0f ? fabsf(before_deg) / denominator : 0.5f;
    const float interpolated_time_ms = static_cast<float>(energy_control_autonomous_previous_detector_test_ms_) +
        alpha * static_cast<float>(t_test_ms - energy_control_autonomous_previous_detector_test_ms_);
    const float interpolated_rate_dps = energy_control_autonomous_previous_detector_rate_dps_ +
        alpha * (rate_dps - energy_control_autonomous_previous_detector_rate_dps_);
    updateEnergyControlAutonomousAtZeroCross(t_test_ms, interpolated_rate_dps, before_deg,
        detector_relative_angle_deg, alpha, interpolated_time_ms);
  }
  energy_control_autonomous_previous_detector_relative_angle_deg_ = detector_relative_angle_deg;
  energy_control_autonomous_previous_detector_rate_dps_ = rate_dps;
  energy_control_autonomous_previous_detector_test_ms_ = t_test_ms;
}
void ExperimentRunner::updateEnergyControlAutonomousPeakTracker(uint32_t now_ms,
                                                                  float detector_relative_angle_deg,
                                                                  float rate_dps) {
  if (!energy_control_autonomous_mode_ || !energy_control_autonomous_peak_tracker_enabled_ ||
      energy_control_autonomous_half_cycle_state_ != EnergyControlAutonomousHalfCycleState::WAIT_PEAK ||
      status_.pulse_active ||
      energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::IDLE ||
      energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::STRONG_START_KICK ||
      energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::STOP ||
      !isfinite(detector_relative_angle_deg) || !isfinite(rate_dps)) return;
  const int8_t detector_side = detector_relative_angle_deg > 0.0f ? 1 :
      (detector_relative_angle_deg < 0.0f ? -1 : 0);
  if (detector_side == 0) return;
  const float detector_abs_deg = fabsf(detector_relative_angle_deg);
  const uint32_t t_test_ms = now_ms - run_start_ms_;
  if (!energy_control_autonomous_peak_tracker_started_) {
    energy_control_autonomous_peak_tracker_started_ = true;
    energy_control_autonomous_candidate_detector_side_ = detector_side;
    energy_control_autonomous_candidate_detector_peak_abs_deg_ = detector_abs_deg;
    energy_control_autonomous_candidate_peak_amplitude_deg_ = fabsf(energy_control_autonomous_gyro_relative_deg_);
    energy_control_autonomous_candidate_peak_ms_ = t_test_ms;
    energy_control_autonomous_return_samples_ = 0;
    return;
  }
  if (detector_side == energy_control_autonomous_candidate_detector_side_ &&
      detector_abs_deg >= energy_control_autonomous_candidate_detector_peak_abs_deg_) {
    energy_control_autonomous_candidate_detector_peak_abs_deg_ = detector_abs_deg;
    energy_control_autonomous_candidate_peak_amplitude_deg_ = fabsf(energy_control_autonomous_gyro_relative_deg_);
    energy_control_autonomous_candidate_peak_ms_ = t_test_ms;
    energy_control_autonomous_return_samples_ = 0;
    return;
  }
  const int8_t rate_sign = rate_dps > 0.0f ? 1 : (rate_dps < 0.0f ? -1 : 0);
  const bool returning_toward_centre = detector_abs_deg <
      energy_control_autonomous_candidate_detector_peak_abs_deg_;
  // The adopted detector is empirically opposite in sign to +gy. At a
  // detector-side extremum, return-to-centre +gy has detector-side sign.
  const bool rate_confirms_return = rate_sign == energy_control_autonomous_candidate_detector_side_;
  if (!returning_toward_centre || !rate_confirms_return) {
    energy_control_autonomous_return_samples_ = 0;
    return;
  }
  if (++energy_control_autonomous_return_samples_ <
      Config::ENERGY_CONTROL_AUTONOMOUS_PEAK_CONFIRM_SAMPLES) return;
  const float detector_peak_angle_deg = static_cast<float>(
      energy_control_autonomous_candidate_detector_side_) *
      energy_control_autonomous_candidate_detector_peak_abs_deg_;
  const bool accepted = recordEnergyControlAutonomousPeak(
      energy_control_autonomous_candidate_peak_ms_,
      -energy_control_autonomous_candidate_detector_side_,
      energy_control_autonomous_candidate_peak_amplitude_deg_, detector_peak_angle_deg);
  if (!accepted) resetEnergyControlAutonomousPeakTracker(true);
}
void ExperimentRunner::updateEnergyControlAutonomousAtZeroCross(uint32_t t_test_ms, float rate_dps,
                                                                  float detector_before_deg,
                                                                  float detector_after_deg,
                                                                  float crossing_alpha,
                                                                  float interpolated_time_ms) {
  if (!energy_control_autonomous_mode_ || !logger_ || status_.pulse_active ||
      (energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::ENERGY_CONTROL &&
       energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::HOLD) ||
      energy_control_autonomous_half_cycle_state_ !=
          EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS ||
      !energy_control_autonomous_last_peak_valid_ ||
      energy_control_autonomous_zero_cross_consumed_for_peak_) return;
  const uint32_t accepted_cross_ms = static_cast<uint32_t>(lroundf(interpolated_time_ms));
  if (energy_control_autonomous_last_accepted_zero_cross_valid_ &&
      static_cast<uint32_t>(accepted_cross_ms - energy_control_autonomous_last_accepted_zero_cross_ms_) <
          Config::ENERGY_CONTROL_AUTONOMOUS_MIN_HALF_CYCLE_MS) return;
  if (logger_->energyControlAutonomousEventCapacityReached()) {
    logger_->markEnergyControlAutonomousEventOverflow();
  }
  // One accepted peak owns one following accepted central passage. Do not arm
  // the next peak tracker until a normal pulse has finished (or width is zero).
  energy_control_autonomous_zero_cross_consumed_for_peak_ = true;
  energy_control_autonomous_last_accepted_zero_cross_valid_ = true;
  energy_control_autonomous_last_accepted_zero_cross_ms_ = accepted_cross_ms;
  resetEnergyControlAutonomousPeakTracker(false);
  auto rearm_for_next_peak = [this]() {
    energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::WAIT_PEAK;
    resetEnergyControlAutonomousPeakTracker(true);
  };
  PsramLogger::EnergyControlAutonomousZeroCrossEvent event;
  event.zero_cross_time_ms = accepted_cross_ms;
  event.zero_cross_rate_dps = rate_dps;
  event.zero_cross_abs_rate_dps = fabsf(rate_dps);
  event.detector_angle_before_deg = detector_before_deg;
  event.detector_angle_after_deg = detector_after_deg;
  event.detector_crossing_alpha = crossing_alpha;
  event.interpolated_crossing_time_ms = interpolated_time_ms;
  event.physical_next_peak_side = rate_dps >= 0.0f ? 1 : -1;
  event.side_mismatch_diagnostic =
      event.physical_next_peak_side != -energy_control_autonomous_last_peak_side_;
  event.rate_support_diagnostic = event.zero_cross_abs_rate_dps >= Config::Q1_SHADOW_RATE_SUPPORT_MIN_DPS &&
      event.zero_cross_abs_rate_dps <= Config::Q1_SHADOW_RATE_SUPPORT_MAX_DPS;
  event.phase = static_cast<uint8_t>(energy_control_autonomous_phase_);
  event.previous_peak_time_ms = energy_control_autonomous_last_peak_ms_;
  event.previous_peak_side = energy_control_autonomous_last_peak_side_;
  event.previous_peak_amplitude_deg = energy_control_autonomous_last_peak_amplitude_deg_;
  event.target_peak_deg = energy_control_autonomous_target_peak_deg_;
  // V6 normal excitation: command in the current zero-cross motion direction.
  // Physical side remains the sole selector of the side-specific Q gain.
  event.q_command_direction = event.physical_next_peak_side;
  event.command_matches_zero_cross_motion =
      event.q_command_direction == event.physical_next_peak_side;
  event.vbat_mV = status_.roller_battery_mV;
  event.free_next_peak_amplitude_deg = energyControlAutonomousFreeNextPeakAmplitude(
      energy_control_autonomous_last_peak_amplitude_deg_);
  event.passive_energy_j = energyControlPotentialJ(event.free_next_peak_amplitude_deg);
  event.target_energy_j = energyControlPotentialJ(event.target_peak_deg);
  event.q1_gain_deg_per_mA_s = energyControlAutonomousGainForSide(event.physical_next_peak_side);
  event.g_side_base_deg_per_mA_s = event.q1_gain_deg_per_mA_s;
  energyControlAutonomousCorrectionParameters(event.physical_next_peak_side,
      &event.c_side_used_deg, &event.g_side_corrected_deg_per_mA_s);
  event.correction_blend_lambda =
      Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_CORRECTION_ENABLED
          ? Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_BLEND_LAMBDA : 0.0f;
  const float zero_q_corrected_prediction_deg = energyControlAutonomousCorrectedPrediction(
      event.free_next_peak_amplitude_deg, event.physical_next_peak_side, 0.0f, nullptr);
  const uint32_t now_ms = run_start_ms_ + t_test_ms;
  event.i0_estimated_mA = predicted_current_end_ms_ == 0 ? 0.0f :
      predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  event.q_available_mA_s = fabsf(predictedChargeMaS(event.i0_estimated_mA,
      event.q_command_direction, static_cast<float>(Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS),
      Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA));
  if (!isfinite(event.free_next_peak_amplitude_deg) || !isfinite(event.passive_energy_j) ||
      !isfinite(event.target_energy_j) || !isfinite(event.q1_gain_deg_per_mA_s) ||
      event.q1_gain_deg_per_mA_s <= 0.0f || !isfinite(event.q_available_mA_s) ||
      !isfinite(zero_q_corrected_prediction_deg)) {
    event.reason = (!isfinite(event.passive_energy_j) || !isfinite(event.target_energy_j))
        ? Config::ENERGY_CONTROL_AUTONOMOUS_REASON_POTENTIAL_DOMAIN
        : Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONFINITE_STATE;
    logger_->addEnergyControlAutonomousZeroCrossEvent(event);
    rearm_for_next_peak();
    return;
  }
  event.delta_energy_required_j = event.target_energy_j - event.passive_energy_j;
  uint16_t ff_width_ms = 0;
  float ff_q_mA_s = 0.0f;
  float ff_energy_j = energyControlPotentialJ(zero_q_corrected_prediction_deg);
  float ff_error_j = fabsf(event.target_energy_j - ff_energy_j);
  for (uint16_t width_ms = Config::ENERGY_CONTROL_AUTONOMOUS_MIN_PULSE_MS;
       width_ms <= Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS; ++width_ms) {
    const float q_mA_s = width_ms == 0 ? 0.0f : fabsf(predictedChargeMaS(event.i0_estimated_mA,
        event.q_command_direction, static_cast<float>(width_ms), Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA));
    const float predicted_peak_deg = energyControlAutonomousCorrectedPrediction(
        event.free_next_peak_amplitude_deg, event.physical_next_peak_side, q_mA_s, nullptr);
    const float energy_j = energyControlPotentialJ(predicted_peak_deg);
    if (!isfinite(q_mA_s) || !isfinite(energy_j)) break;
    const float error_j = fabsf(event.target_energy_j - energy_j);
    if (error_j < ff_error_j) {
      ff_width_ms = width_ms;
      ff_q_mA_s = q_mA_s;
      ff_energy_j = energy_j;
      ff_error_j = error_j;
    }
  }
  event.q_ff_energy_mA_s = ff_q_mA_s;
  event.q_angle_diagnostic_mA_s = fmaxf(0.0f, (event.target_peak_deg -
      event.free_next_peak_amplitude_deg) / event.q1_gain_deg_per_mA_s);
  event.integral_side_mA_s = event.physical_next_peak_side > 0
      ? energy_control_autonomous_integral_plus_mA_s_ : energy_control_autonomous_integral_minus_mA_s_;
  event.q_unclamped_mA_s = event.q_ff_energy_mA_s + event.integral_side_mA_s;
  event.q_saturated_upper = event.q_unclamped_mA_s > event.q_available_mA_s ||
      (ff_width_ms == Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS &&
       ff_energy_j + 1.0e-8f < event.target_energy_j);
  event.q_saturated_lower = event.q_unclamped_mA_s < 0.0f;
  const float corrected_q_target_mA_s = fmaxf(0.0f, fminf(event.q_available_mA_s,
      event.q_unclamped_mA_s));
  const float corrected_target_prediction_deg = energyControlAutonomousCorrectedPrediction(
      event.free_next_peak_amplitude_deg, event.physical_next_peak_side,
      corrected_q_target_mA_s, nullptr);
  const float corrected_target_energy_j = energyControlPotentialJ(corrected_target_prediction_deg);
  if (!isfinite(corrected_target_energy_j)) {
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_POTENTIAL_DOMAIN;
    logger_->addEnergyControlAutonomousZeroCrossEvent(event);
    rearm_for_next_peak();
    return;
  }
  uint16_t selected_width_ms = 0;
  float selected_q_mA_s = 0.0f;
  float selected_energy_j = event.passive_energy_j;
  float selected_error_j = fabsf(corrected_target_energy_j - selected_energy_j);
  for (uint16_t width_ms = Config::ENERGY_CONTROL_AUTONOMOUS_MIN_PULSE_MS;
       width_ms <= Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS; ++width_ms) {
    const float q_mA_s = width_ms == 0 ? 0.0f : fabsf(predictedChargeMaS(event.i0_estimated_mA,
        event.q_command_direction, static_cast<float>(width_ms), Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA));
    const float predicted_peak_deg = energyControlAutonomousCorrectedPrediction(
        event.free_next_peak_amplitude_deg, event.physical_next_peak_side, q_mA_s, nullptr);
    const float energy_j = energyControlPotentialJ(predicted_peak_deg);
    if (!isfinite(q_mA_s) || !isfinite(energy_j)) break;
    const float error_j = fabsf(corrected_target_energy_j - energy_j);
    if (error_j < selected_error_j) {
      selected_width_ms = width_ms;
      selected_q_mA_s = q_mA_s;
      selected_energy_j = energy_j;
      selected_error_j = error_j;
    }
  }
  event.q_command_mA_s = selected_q_mA_s;
  event.q_effective_pred_mA_s = selected_q_mA_s;
  event.a_pred_base_deg = event.free_next_peak_amplitude_deg +
      event.q1_gain_deg_per_mA_s * selected_q_mA_s;
  event.a_pred_corrected_deg = energyControlAutonomousCorrectedPrediction(
      event.free_next_peak_amplitude_deg, event.physical_next_peak_side,
      selected_q_mA_s, &event.correction_deg);
  event.predicted_next_peak_amplitude_deg = event.a_pred_corrected_deg;
  event.predicted_energy_j = selected_energy_j;
  event.q_gain_extrapolated = selected_width_ms > 0 &&
      (selected_q_mA_s < Config::Q1_SHADOW_Q_SUPPORT_MIN_MAS ||
       selected_q_mA_s > Config::Q1_SHADOW_Q_SUPPORT_MAX_MAS);
  event.solver_required_width_ms = static_cast<float>(selected_width_ms);
  event.solver_selected_integer_width_ms = selected_width_ms;
  energy_control_autonomous_pending_peak_ = true;
  energy_control_autonomous_pending_next_side_ = event.physical_next_peak_side;
  energy_control_autonomous_pending_q_command_mA_s_ = selected_q_mA_s;
  energy_control_autonomous_pending_saturated_upper_ = event.q_saturated_upper;
  energy_control_autonomous_pending_saturated_lower_ = event.q_saturated_lower;
  if (selected_width_ms == 0) {
    event.valid = true;
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_VALID_NO_OUTPUT;
    logger_->addEnergyControlAutonomousZeroCrossEvent(event);
    rearm_for_next_peak();
    return;
  }
  if (status_.emergency_stop || status_.state != ExperimentState::RUNNING_BATCH_SWEEP) {
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_ESTOP_OR_STATE;
    logger_->addEnergyControlAutonomousZeroCrossEvent(event);
    rearm_for_next_peak();
    return;
  }
  if (!roller_ || !roller_->ok()) {
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_ROLLER_NOT_READY;
    logger_->addEnergyControlAutonomousZeroCrossEvent(event);
    rearm_for_next_peak();
    return;
  }
  event.pulse_width_ms = selected_width_ms;
  event.command_current_mA = Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA;
  event.pulse_start_ms = t_test_ms;
  event.pulse_end_ms = t_test_ms + selected_width_ms;
  // v45 labels are copied from the already-selected V7 command. They are
  // diagnostic only; neither field is read by the pulse/control path.
  status_.current_audit_q_target_mA_s = event.q_command_mA_s;
  status_.current_audit_q_pred_mA_s = event.q_effective_pred_mA_s;
  if (!beginEnergyControlAutonomousPulse(now_ms, t_test_ms, event.q_command_direction, selected_width_ms)) {
    event.command_current_mA = 0;
    event.pulse_width_ms = 0;
    event.pulse_start_ms = 0;
    event.pulse_end_ms = 0;
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_CURRENT_WRITE_FAILED;
    logger_->addEnergyControlAutonomousZeroCrossEvent(event);
    rearm_for_next_peak();
    return;
  }
  event.output_executed = true;
  event.valid = true;
  event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONE;
  logger_->addEnergyControlAutonomousZeroCrossEvent(event);
}
void ExperimentRunner::captureAngleOffsets() {
  offset_beta1_raw_deg_ = raw_beta1_raw_pitch_deg_;
  offset_beta1_bias_deg_ = raw_beta1_bias_pitch_deg_;
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    offset_dynamic_raw_deg_[i] = raw_dynamic_raw_pitch_deg_[i];
    offset_dynamic_bias_deg_[i] = raw_dynamic_bias_pitch_deg_[i];
  }
  offset_accel_deg_ = raw_accel_pitch_deg_;
  gyro_raw_deg_ = 0.0f;
  gyro_bias_corrected_deg_ = 0.0f;
}

void ExperimentRunner::requestEmergencyStop(const char* reason) {
  if(fixed_probe_mode_ && roller_) roller_->abortFixedProbe(FixedProbeV57::USER_STOP);
  if (wheel_probe_mode_ && roller_ && !roller_->qObserver().v55.finished)
    roller_->qObserver().v55.abort(WheelProbeV55::USER_STOP, micros());
  setSyncLed(false);
  stopMotor();
  if (status_.state == ExperimentState::RUNNING_BATCH_SWEEP || status_.state == ExperimentState::TRIAL_REST ||
      status_.state == ExperimentState::END_SYNC) {
    logger_->markMeasurementDone();
  }
  status_.state = ExperimentState::ESTOP;
  status_.running = false;
  status_.emergency_stop = true;
  status_.last_error = reason ? reason : "estop";
}

void ExperimentRunner::clearFinishedOrEstop() {
  if (running()) return;
  if (logger_) logger_->clear();
  setSyncLed(false);
  status_.state = bias_ready_ ? ExperimentState::READY_TO_MEASURE : ExperimentState::STARTUP_GYRO_CALIB;
  status_.running = false;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.measure_elapsed_ms = 0;
  status_.remaining_ms = measurementTotalDurationMs();
  status_.sync_event_id = 0;
}

void ExperimentRunner::setInputSettings(int16_t current_mA, uint16_t pulse_width_ms, uint16_t input_interval_ms) {
  if (running()) return;
  if (current_mA < 0) current_mA = -current_mA;
  if (current_mA > Config::MAX_ABS_INPUT_CURRENT_MA) current_mA = Config::MAX_ABS_INPUT_CURRENT_MA;
  pulse_width_ms = constrain(pulse_width_ms, Config::MIN_PULSE_WIDTH_MS, Config::MAX_PULSE_WIDTH_MS);
  input_interval_ms = constrain(input_interval_ms, Config::MIN_INPUT_INTERVAL_MS, Config::MAX_INPUT_INTERVAL_MS);
  if (pulse_width_ms > input_interval_ms) pulse_width_ms = input_interval_ms;
  status_.current_mA_setting = current_mA;
  status_.pulse_width_ms_setting = pulse_width_ms;
  status_.input_interval_ms = input_interval_ms;
}

void ExperimentRunner::updateMidSyncLed(uint32_t now_ms) {
  const uint32_t t_test_ms = now_ms - run_start_ms_;
  if (t_test_ms < Config::MID_SYNC_FIRST_MS) {
    status_.sync_event_id = 0;
    setSyncLed(false);
    return;
  }
  const uint32_t phase_ms = (t_test_ms - Config::MID_SYNC_FIRST_MS) % Config::MID_SYNC_INTERVAL_MS;
  if (phase_ms < Config::MID_SYNC_SHORT_ON_MS) {
    status_.sync_event_id = 5;
    setSyncLed(true);
  } else if (phase_ms < Config::MID_SYNC_SHORT_ON_MS + Config::MID_SYNC_GAP_MS) {
    status_.sync_event_id = 5;
    setSyncLed(false);
  } else if (phase_ms < Config::MID_SYNC_TOTAL_MS) {
    status_.sync_event_id = 5;
    setSyncLed(true);
  } else {
    status_.sync_event_id = 0;
    setSyncLed(false);
  }
}

void ExperimentRunner::updateStartSync(uint32_t now_ms) {
  stopMotor();
  if (updateSyncPattern(now_ms, Config::START_SYNC_PATTERN, Config::START_SYNC_PATTERN_STEP_COUNT)) {
    setSyncLed(false);
    beginMeasurementRun();
  }
}

void ExperimentRunner::beginEndSync(uint32_t now_ms) {
  stopMotor();
  status_.state = ExperimentState::END_SYNC;
  status_.running = true;
  status_.sync_event_id = 4;
  sync_step_ = 0;
  sync_step_start_ms_ = now_ms;
  sync_led_until_ms_ = 0;
  setSyncLed(Config::END_SYNC_PATTERN[0].led_on);
  logSampleNow();
}

void ExperimentRunner::updateEndSync(uint32_t now_ms) {
  stopMotor();
  if (!updateSyncPattern(now_ms, Config::END_SYNC_PATTERN, Config::END_SYNC_PATTERN_STEP_COUNT)) return;

  setSyncLed(false);
  logger_->markMeasurementDone();
  status_.state = ExperimentState::FINISHED;
  status_.running = false;
  status_.sync_event_id = 0;
}

bool ExperimentRunner::updateSyncPattern(uint32_t now_ms, const Config::LedSyncStep* pattern, uint8_t step_count) {
  if (!pattern || step_count == 0) return true;
  if (sync_step_ >= step_count) return true;

  while (sync_step_ < step_count) {
    const uint16_t duration_ms = pattern[sync_step_].duration_ms;
    if (static_cast<uint32_t>(now_ms - sync_step_start_ms_) < duration_ms) break;
    sync_step_start_ms_ += duration_ms;
    ++sync_step_;
    if (sync_step_ >= step_count) return true;
    setSyncLed(pattern[sync_step_].led_on);
  }
  return false;
}

void ExperimentRunner::finishTrial(uint32_t now_ms) {
  stopMotor();
  status_.trial_elapsed_ms = status_.trial_duration_ms;
  status_.sync_event_id = 7;
  logSampleNow();
  if (single_trial_mode_ || status_.trial_index >= Config::BETA_SWEEP_TRIAL_COUNT) {
    finishRun();
    return;
  }
  status_.state = ExperimentState::TRIAL_REST;
  rest_start_ms_ = now_ms;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
}

void ExperimentRunner::updateTrialRest(uint32_t now_ms) {
  stopMotor();
  if (static_cast<uint32_t>(now_ms - rest_start_ms_) < Config::BETA_SWEEP_INTER_TRIAL_REST_MS) return;
  beginTrial(status_.trial_index);
}

void ExperimentRunner::updateInputPulse(uint32_t now_ms) {
  if (status_.state != ExperimentState::RUNNING_BATCH_SWEEP) return;
  const uint32_t t_ms = now_ms - trial_start_ms_;

  if (status_.pulse_active &&
      static_cast<uint32_t>(now_ms - active_pulse_start_ms_) >= status_.pulse_width_ms_setting) {
    stopActivePulse(now_ms);
  }
  if (status_.pulse_active || t_ms >= status_.trial_duration_ms) return;

  if (!zero_cross_mode_) {
    if (t_ms >= next_pulse_start_test_ms_) {
      beginPulse(now_ms, t_ms, next_pulse_direction_);
      next_pulse_direction_ = -next_pulse_direction_;
    }
    return;
  }

  const float angle_deg = status_.pitch_dynamic_beta_deg[Config::FILTER_ADOPTED_INDEX];
  const float rate_dps = status_.gyro_pitch_rate_dps - pitchBiasFromGyroBias();
  if (calibration_enabled_) updateCalibrationPeak(now_ms, t_ms, angle_deg, rate_dps);
  if (calibration_enabled_ && !calibrationIsMainControl() && t_ms >= Config::ZERO_CROSS_CALIBRATION_TIMEOUT_MS) {
    finishCalibrationShadow(4);
  }
  // V59 may finish from the calibration state machine; never issue another
  // zero-cross pulse in the same IMU update after the run has entered END_SYNC.
  if (status_.state != ExperimentState::RUNNING_BATCH_SWEEP) return;
  if (identification_mode_) updateIdentificationPeak(now_ms, angle_deg, rate_dps);
  if (!zero_cross_has_previous_angle_) {
    zero_cross_previous_angle_deg_ = angle_deg;
    zero_cross_has_previous_angle_ = true;
  }

  // Identification uses a strong bootstrap, then either rotates validation Q targets or selects a control Q.
  if (zero_cross_bootstrap_pending_) {
    status_.current_mA_setting = zero_cross_fixed_current_mA_;
    const int8_t direction = Config::ZERO_CROSS_BOOTSTRAP_DIRECTION;
    const float i0 = predicted_signed_current_end_mA_ *
        expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
    CalibrationBuildUpCommand calibration_command;
    const bool use_calibration_build_up = calibration_enabled_;
    const float q_command = use_calibration_build_up
        ? (calibration_command = calibrationBuildUpCommand(
              angle_deg, rate_dps, i0, direction,
              Config::ZERO_CROSS_CALIBRATION_INITIAL_TARGET_PEAK_DEG)).q_command_mA_s
        : Config::ZERO_CROSS_IDENTIFICATION_BOOTSTRAP_Q_MAS;
    const uint16_t width = use_calibration_build_up
        ? calibration_command.width_ms
        : (identification_mode_ ? identificationWidthForTarget(q_command, i0, direction)
                                : zero_cross_fixed_pulse_ms_);
    status_.pulse_width_ms_setting = width;
    if (identification_mode_) {
      const float q_requested = use_calibration_build_up ? calibration_command.q_required_mA_s
          : (q_run_mode_ == QRunMode::CONTROL ? requiredControlQMaS(angle_deg, rate_dps, direction) : q_command);
      const float a_target = use_calibration_build_up ? Config::ZERO_CROSS_CALIBRATION_INITIAL_TARGET_PEAK_DEG
          : (q_run_mode_ == QRunMode::CONTROL ? control_target_peak_deg_ : 0.0f);
      const float a_pred = use_calibration_build_up ? calibration_command.predicted_peak_deg
          : predictNextPeakAbsDeg(angle_deg, rate_dps, q_command, direction);
      const ControlQEvaluation evaluation = q_run_mode_ == QRunMode::CONTROL && !use_calibration_build_up
          ? evaluateControlQ(angle_deg, rate_dps, direction) : ControlQEvaluation{};
      beginIdentificationEvent(now_ms, t_ms, direction, q_command, q_requested, q_command,
                               a_target, a_pred, evaluation.min_predicted_peak_deg,
                               evaluation.max_predicted_peak_deg, evaluation.target_reachable,
                               true, width, false);
    }
    beginPulse(now_ms, t_ms, direction);
    if (use_calibration_build_up) {
      calibration_initial_kick_count_ = 1;
      calibration_initial_kick_q_effective_pred_mA_s_ = calibration_command.q_effective_mA_s;
      recordCalibrationBuildUpPulse(t_ms, angle_deg, rate_dps, direction, 0,
                                    Config::ZERO_CROSS_CALIBRATION_INITIAL_TARGET_PEAK_DEG,
                                    calibration_command);
      calibration_phase_ = CalibrationPhase::INITIAL_EXCITE;
    }
    zero_cross_bootstrap_pending_ = false;
    zero_cross_armed_ = false;
    zero_cross_start_refractory_until_ms_ = now_ms + Config::ZERO_CROSS_START_REFRACTORY_MS;
    zero_cross_half_cycle_peak_abs_deg_ = fabsf(angle_deg);
    zero_cross_previous_angle_deg_ = angle_deg;
    return;
  }

  zero_cross_half_cycle_peak_abs_deg_ =
      fmaxf(zero_cross_half_cycle_peak_abs_deg_, fabsf(angle_deg));
  if (static_cast<int32_t>(now_ms - zero_cross_start_refractory_until_ms_) < 0) {
    zero_cross_previous_angle_deg_ = angle_deg;
    return;
  }

  if (fabsf(angle_deg) >= Config::ZERO_CROSS_REARM_ANGLE_DEG) zero_cross_armed_ = true;
  const bool interval_ok = last_zero_cross_pulse_start_ms_ == 0 ||
                           static_cast<uint32_t>(now_ms - last_zero_cross_pulse_start_ms_) >=
                               Config::ZERO_CROSS_MIN_PULSE_INTERVAL_MS;
  bool zero_cross_detected = false;
  if (zero_cross_armed_ && interval_ok) {
    zero_cross_detected =
        (zero_cross_previous_angle_deg_ < 0.0f && angle_deg >= 0.0f &&
         rate_dps >= Config::ZERO_CROSS_MIN_RATE_DPS) ||
        (zero_cross_previous_angle_deg_ > 0.0f && angle_deg <= 0.0f &&
         rate_dps <= -Config::ZERO_CROSS_MIN_RATE_DPS);
  }
  zero_cross_previous_angle_deg_ = angle_deg;
  if (!zero_cross_detected) return;

  // The angle estimate only decides when a pulse is allowed. Its sign is not
  // trusted to decide motor direction: every accepted crossing alternates the
  // physical excitation direction after the one-time bootstrap kick.
  int8_t direction = zero_cross_next_direction_;
  zero_cross_next_direction_ = -zero_cross_next_direction_;
  const int8_t next_peak_side = rate_dps >= 0.0f ? 1 : -1;

  if (calibration_enabled_ && !calibrationIsMainControl()) {
    zero_cross_armed_ = false;
    zero_cross_half_cycle_peak_abs_deg_ = fabsf(angle_deg);
    if (calibration_phase_ == CalibrationPhase::INITIAL_EXCITE) {
      if (calibration_initial_kick_count_ >= Config::ZERO_CROSS_CALIBRATION_INITIAL_MAX_KICKS) {
        finishCalibrationShadow(1);
        return;
      }
      status_.current_mA_setting = zero_cross_fixed_current_mA_;
      const float i0_initial = predicted_signed_current_end_mA_ *
          expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
      const CalibrationBuildUpCommand command =
          calibrationBuildUpCommand(angle_deg, rate_dps, i0_initial, direction,
                                    Config::ZERO_CROSS_CALIBRATION_INITIAL_TARGET_PEAK_DEG);
      status_.pulse_width_ms_setting = command.width_ms;
      calibration_initial_kick_q_effective_pred_mA_s_ = command.q_effective_mA_s;
      ++calibration_initial_kick_count_;
      recordCalibrationBuildUpPulse(t_ms, angle_deg, rate_dps, direction, 0,
                                    Config::ZERO_CROSS_CALIBRATION_INITIAL_TARGET_PEAK_DEG, command);
      beginPulse(now_ms, t_ms, direction);
      return;
    }
    if (calibration_phase_ == CalibrationPhase::FREE_DECAY &&
        next_peak_side == Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE &&
        calibration_last_half_range_valid_ && calibration_last_center_dynamic_valid_) {
      // V62 observes this unforced positive arrival only; it cannot command a
      // pulse or alter the fixed-Q sequence.
      addV62RateStateSample(calibration_last_half_range_dynamic_deg_,
                            calibration_last_center_dynamic_deg_, fabsf(rate_dps));
    }
    if (calibration_phase_ == CalibrationPhase::FREE_DECAY) return;
    if (calibration_phase_ == CalibrationPhase::Q_REBUILD) {
      // Rebuild only until the confirmed predecessor is inside the common
      // margin-reduced fitted input domain. Every pulse remains subject to the
      // existing Q cap and prediction guard used by calibrationBuildUpCommand.
      if (calibration_reverse_samples_ != 0 || !calibration_last_peak_valid_) return;
      const float support_min = fmaxf(calibration_result_.free_input_min_pos_deg,
                                      calibration_result_.free_input_min_neg_deg);
      const float support_max = fminf(calibration_result_.free_input_max_pos_deg,
                                      calibration_result_.free_input_max_neg_deg);
      const float margin = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SUPPORT_MARGIN_DEG;
      const bool legacy_a_domain_valid = support_min + margin <= support_max - margin;
      // The dynamic-H model is an independently fitted, measured input domain.
      // If the legacy A domains have no common interior, only an invalid H
      // model is terminal; a valid H model may still be reached by a rebuild.
      if (!legacy_a_domain_valid && !calibration_result_.half_range_dynamic_valid) {
        finishCalibrationShadow(7);
        return;
      }
      bool predecessor_in_legacy_a_support = legacy_a_domain_valid &&
          calibration_last_peak_abs_deg_ >= support_min + margin &&
          calibration_last_peak_abs_deg_ <= support_max - margin;
      bool predecessor_in_dynamic_h_support = dynamicHalfRangePredecessorInSupport();
      if (calibration_probe_plan_index_ >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_COUNT) {
        finishCalibrationShadow(0);
        return;
      }

      // V59 makes this a pre-pulse admission check. A skipped crossing never
      // advances the fixed-Q plan. High or mismatched states coast freely;
      // low states use the existing bounded rebuild below.
      const float v59_hprev = calibration_last_half_range_dynamic_deg_;
      const float v59_cprev = calibration_last_center_dynamic_deg_;
      const float v59_abs_rate = fabsf(rate_dps);
      const bool v59_state_valid = calibration_last_half_range_valid_ &&
          calibration_last_center_dynamic_valid_ && isfinite(v59_hprev) && isfinite(v59_cprev) &&
          isfinite(v59_abs_rate);
      const uint8_t v59_reason = classifyV59StateGate(v59_hprev, v59_cprev, v59_abs_rate,
                                                        next_peak_side, predecessor_in_dynamic_h_support);
      const bool v59_gate_passed = v59_reason == Config::ZERO_CROSS_V59_GATE_REASON_PASSED;
      const bool v59_low_state = v59_reason == Config::ZERO_CROSS_V59_GATE_REASON_DYNAMIC_H_OUT_OF_SUPPORT ||
          v59_reason == Config::ZERO_CROSS_V59_GATE_REASON_HPREV_BELOW ||
          v59_reason == Config::ZERO_CROSS_V59_GATE_REASON_STATE_INVALID;
      // A V60 rebuild is followed by one whole unforced oscillation (two
      // consecutive peak-pair H updates) before the gate may consume a fixed Q.
      // Log these candidates with their full H/C/rate state, but do not count
      // them as failed gate admissions.
      if (calibration_v60_cooldown_halfcycles_remaining_ > 0) {
        --calibration_v60_cooldown_halfcycles_remaining_;
        ++calibration_result_.v60_cooldown_free_decay_count;
        recordV59StateGateEvent(t_ms, v59_hprev, v59_cprev, v59_abs_rate, next_peak_side,
                                v59_state_valid, predecessor_in_dynamic_h_support, v59_reason, 3,
                                true, false);
        if (calibration_v60_cooldown_halfcycles_remaining_ == 0) {
          calibration_v61_rebuild_plan_active_ = false;
        }
        if (logger_) logger_->setCalibrationResult(calibration_result_);
        return;
      }
      // A V61 rebuild drives only the peak side opposite to the fixed-Q
      // arrival.  This avoids the V60 alternation where a negative command
      // was immediately followed by an unrelated positive rebuild.  If the
      // measured state is already admissible, abandon any pending rebuild plan
      // and let the unchanged V59 gate enter the deterministic Q plan.
      if (v59_gate_passed) {
        calibration_v61_rebuild_plan_active_ = false;
      }
      const bool v61_rebuild_required = calibration_result_.v61_state_target_valid &&
          !v59_gate_passed && (calibration_v61_rebuild_plan_active_ || v59_low_state);
      if (v61_rebuild_required) {
        if (!calibration_v61_rebuild_plan_active_) {
          calibration_v61_rebuild_plan_active_ = true;
          calibration_v61_command_peak_target_deg_ =
              calibration_v61_nominal_controlled_peak_target_deg_;
          calibration_result_.v61_feedback_command_peak_deg =
              calibration_v61_command_peak_target_deg_;
        }
        const int8_t controlled_side = -Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE;
        if (next_peak_side != controlled_side) {
          recordV59StateGateEvent(t_ms, v59_hprev, v59_cprev, v59_abs_rate, next_peak_side,
                                  v59_state_valid, predecessor_in_dynamic_h_support,
                                  Config::ZERO_CROSS_V59_GATE_REASON_V61_STATE_FEEDBACK, 0);
          if (logger_) logger_->setCalibrationResult(calibration_result_);
          return;
        }
        recordV59StateGateEvent(t_ms, v59_hprev, v59_cprev, v59_abs_rate, next_peak_side,
                                v59_state_valid, predecessor_in_dynamic_h_support,
                                Config::ZERO_CROSS_V59_GATE_REASON_V61_STATE_FEEDBACK, 2);
        status_.current_mA_setting = zero_cross_fixed_current_mA_;
        const float i0_rebuild = predicted_signed_current_end_mA_ *
            expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
        const CalibrationBuildUpCommand command = calibrationBuildUpCommand(
            angle_deg, rate_dps, i0_rebuild, direction,
            calibration_v61_command_peak_target_deg_,
            Config::ZERO_CROSS_V60_REBUILD_UNRESTRICTED);
        status_.pulse_width_ms_setting = command.width_ms;
        recordCalibrationBuildUpPulse(t_ms, angle_deg, rate_dps, direction, 1,
                                      calibration_v61_command_peak_target_deg_, command);
        if (calibration_rebuild_attempt_in_episode_ == 0) {
          ++calibration_rebuild_episode_id_;
        }
        ++calibration_rebuild_attempt_in_episode_;
        ++calibration_rebuild_total_count_;
        calibration_result_.rebuild_total_count = calibration_rebuild_total_count_;
        calibration_result_.rebuild_episode_id = calibration_rebuild_episode_id_;
        calibration_result_.rebuild_attempt_in_episode = calibration_rebuild_attempt_in_episode_;
        calibration_rebuild_pulse_start_test_ms_ = t_ms;
        calibration_v60_cooldown_halfcycles_remaining_ = 0;
        calibration_phase_ = CalibrationPhase::WAIT_Q_REBUILD;
        beginPulse(now_ms, t_ms, direction);
        calibration_rebuild_pulse_id_ = status_.pulse_id;
        return;
      }
      if (!v59_gate_passed) {
        ++calibration_probe_wait_halfcycle_count_;
        ++calibration_probe_wait_halfcycle_total_;
        ++calibration_v59_gate_event_count_;
        ++calibration_v59_gate_skip_count_;
        calibration_result_.probe_wait_halfcycles = calibration_probe_wait_halfcycle_total_;
        calibration_result_.v59_gate_event_count = calibration_v59_gate_event_count_;
        calibration_result_.v59_gate_skip_count = calibration_v59_gate_skip_count_;
        if (calibration_probe_wait_halfcycle_count_ >= Config::ZERO_CROSS_V59_MAX_CONSECUTIVE_GATE_SKIPS) {
          recordV59StateGateEvent(t_ms, v59_hprev, v59_cprev, v59_abs_rate, next_peak_side,
                                  v59_state_valid, predecessor_in_dynamic_h_support, v59_reason, 0);
          finishCalibrationShadow(10);
          return;
        }
        if (!v59_low_state) {
          if (next_peak_side == Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE &&
              v59_reason == Config::ZERO_CROSS_V59_GATE_REASON_HPREV_ABOVE) {
            calibration_v60_high_desired_wait_pending_ = true;
          }
          recordV59StateGateEvent(t_ms, v59_hprev, v59_cprev, v59_abs_rate, next_peak_side,
                                  v59_state_valid, predecessor_in_dynamic_h_support, v59_reason, 0);
          if (logger_) logger_->setCalibrationResult(calibration_result_);
          return;
        }
        ++calibration_v59_rebuild_from_low_state_count_;
        calibration_result_.v59_rebuild_from_low_state_count = calibration_v59_rebuild_from_low_state_count_;
        const bool v60_gate_skipped_by_decay =
            calibration_v60_high_desired_wait_pending_ &&
            next_peak_side == Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE;
        if (v60_gate_skipped_by_decay) {
          ++calibration_result_.v60_gate_skipped_by_decay_count;
        }
        if (next_peak_side == Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE) {
          calibration_v60_high_desired_wait_pending_ = false;
        }
        recordV59StateGateEvent(t_ms, v59_hprev, v59_cprev, v59_abs_rate, next_peak_side,
                                v59_state_valid, predecessor_in_dynamic_h_support, v59_reason, 2,
                                false, v60_gate_skipped_by_decay);
        // Force the known-safe rebuild branch even where the broader fitted
        // support domain still happens to contain this low V59 state.
        predecessor_in_legacy_a_support = false;
        predecessor_in_dynamic_h_support = false;
      } else {
        calibration_v60_high_desired_wait_pending_ = false;
        ++calibration_v59_gate_event_count_;
        ++calibration_v59_gate_pass_count_;
        calibration_result_.v59_gate_event_count = calibration_v59_gate_event_count_;
        calibration_result_.v59_gate_pass_count = calibration_v59_gate_pass_count_;
        recordV59StateGateEvent(t_ms, v59_hprev, v59_cprev, v59_abs_rate, next_peak_side,
                                v59_state_valid, predecessor_in_dynamic_h_support, v59_reason, 1);
      }

      if (predecessor_in_legacy_a_support || predecessor_in_dynamic_h_support) {
        const int8_t desired_side = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_DESIRED_SIDE[
            calibration_probe_plan_index_];
        if (next_peak_side == desired_side) {
          calibration_probe_rebuild_entry_source_ = predecessor_in_legacy_a_support ? 0 : 1;
          calibration_probe_rebuild_target_reached_ = calibration_result_.v60_rebuild_target_valid &&
              calibration_last_peak_abs_deg_ >= calibration_result_.v60_rebuild_target_peak_deg;
          calibration_probe_dynamic_h_in_support_at_command_ = predecessor_in_dynamic_h_support;
          calibration_phase_ = desired_side > 0 ? CalibrationPhase::Q_CAL_POS :
                                                  CalibrationPhase::Q_CAL_NEG;
        } else {
          // This cannot occur in enabled V59 (direction is part of the gate),
          // but retain the historical free-half-cycle behavior as a guard.
          ++calibration_probe_wait_halfcycle_count_;
          ++calibration_probe_wait_halfcycle_total_;
          calibration_result_.probe_wait_halfcycles = calibration_probe_wait_halfcycle_total_;
          if (logger_) logger_->setCalibrationResult(calibration_result_);
          return;
        }
      } else {
        if (Config::ZERO_CROSS_CALIBRATION_Q_REBUILD_MAX_ATTEMPTS != 0 &&
            calibration_rebuild_attempt_in_episode_ >=
                Config::ZERO_CROSS_CALIBRATION_Q_REBUILD_MAX_ATTEMPTS) {
          finishCalibrationShadow(8);
          return;
        }
        status_.current_mA_setting = zero_cross_fixed_current_mA_;
        const float i0_rebuild = predicted_signed_current_end_mA_ *
            expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
        const CalibrationBuildUpCommand command = calibrationBuildUpCommand(
            angle_deg, rate_dps, i0_rebuild, direction,
            calibration_result_.v60_rebuild_target_peak_deg,
            Config::ZERO_CROSS_V60_REBUILD_UNRESTRICTED);
        status_.pulse_width_ms_setting = command.width_ms;
        recordCalibrationBuildUpPulse(t_ms, angle_deg, rate_dps, direction, 1,
                                      calibration_result_.v60_rebuild_target_peak_deg, command);
        if (calibration_rebuild_attempt_in_episode_ == 0) {
          ++calibration_rebuild_episode_id_;
        }
        ++calibration_rebuild_attempt_in_episode_;
        ++calibration_rebuild_total_count_;
        calibration_result_.rebuild_total_count = calibration_rebuild_total_count_;
        calibration_result_.rebuild_episode_id = calibration_rebuild_episode_id_;
        calibration_result_.rebuild_attempt_in_episode = calibration_rebuild_attempt_in_episode_;
        calibration_rebuild_pulse_start_test_ms_ = t_ms;
        // Cooldown is scheduled only after a measured peak reaches the V60
        // rebuild target, not between below-target build-up pulses.
        calibration_v60_cooldown_halfcycles_remaining_ = 0;
        calibration_phase_ = CalibrationPhase::WAIT_Q_REBUILD;
        beginPulse(now_ms, t_ms, direction);
        calibration_rebuild_pulse_id_ = status_.pulse_id;
        return;
      }
    }
    const bool want_pos = calibration_phase_ == CalibrationPhase::Q_CAL_POS;
    const bool want_neg = calibration_phase_ == CalibrationPhase::Q_CAL_NEG;
    if (want_pos || want_neg) {
      if (calibration_probe_attempt_count_ >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_MAX_ATTEMPTS) {
        finishCalibrationShadow(5);
        return;
      }
      // Do not command from a stale pre-turn state. A confirmed extremum is
      // required before a low-Q probe can be associated with its next peak.
      if (calibration_reverse_samples_ != 0 || !calibration_last_peak_valid_) return;
      const int8_t requested_side = want_pos ? 1 : -1;
      if (calibration_probe_plan_index_ >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_COUNT ||
          requested_side != Config::ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_DESIRED_SIDE[
              calibration_probe_plan_index_]) {
        finishCalibrationShadow(3);
        return;
      }
      // The probe can arrive on either side, so its predecessor must lie in
      // the intersection of both selected A_prev domains, after margin.
      const float support_min = fmaxf(calibration_result_.free_input_min_pos_deg,
                                      calibration_result_.free_input_min_neg_deg);
      const float support_max = fminf(calibration_result_.free_input_max_pos_deg,
                                      calibration_result_.free_input_max_neg_deg);
      const float margin = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SUPPORT_MARGIN_DEG;
      const bool predecessor_in_legacy_a_support = support_min + margin <= support_max - margin &&
          calibration_last_peak_abs_deg_ >= support_min + margin &&
          calibration_last_peak_abs_deg_ <= support_max - margin;
      const bool predecessor_in_dynamic_h_support = dynamicHalfRangePredecessorInSupport();
      if (!predecessor_in_legacy_a_support && !predecessor_in_dynamic_h_support) {
        // Never extrapolate either free-decay model. The v50 H path is allowed
        // only inside the measured dynamic-H input domain.
        finishCalibrationShadow(7);
        return;
      }
      // v46 base polarity is fixed and explicit: direction = -desired arrival
      // side. A measured non-positive gain is data, never a reason to flip it.
      direction = -requested_side;
      const uint8_t plan_index = calibration_probe_plan_index_;
      const uint8_t q_level_index = calibrationProbeQLevelIndex(plan_index);
      const float q_target_mA_s = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_Q_LEVEL_MAS[q_level_index];
      status_.current_mA_setting = zero_cross_fixed_current_mA_;
      const float i0_cal = predicted_signed_current_end_mA_ *
                           expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
      const uint16_t width_cal = calibrationProbeWidthForTarget(q_target_mA_s, i0_cal, direction);
      status_.pulse_width_ms_setting = width_cal;
      calibration_probe_previous_peak_abs_deg_ = calibration_last_peak_abs_deg_;
      calibration_probe_previous_peak_signed_dynamic_deg_ = calibration_last_peak_signed_dynamic_deg_;
      calibration_probe_previous_peak_signed_fixed_deg_ = calibration_last_peak_signed_fixed_deg_;
      calibration_probe_previous_half_range_dynamic_deg_ = calibration_last_half_range_dynamic_deg_;
      calibration_probe_previous_half_range_fixed_deg_ = calibration_last_half_range_fixed_deg_;
      calibration_probe_previous_half_range_valid_ = calibration_last_half_range_valid_;
      calibration_probe_previous_center_dynamic_deg_ = calibration_last_center_dynamic_deg_;
      calibration_probe_previous_center_dynamic_valid_ = calibration_last_center_dynamic_valid_;
      calibration_probe_previous_peak_candidate_ms_ = calibration_last_peak_candidate_ms_;
      calibration_probe_q_target_mA_s_ = q_target_mA_s;
      calibration_probe_q_effective_pred_mA_s_ = fabsf(identificationChargeMaS(i0_cal, direction, width_cal));
      calibration_probe_command_dynamic_angle_deg_ =
          status_.pitch_dynamic_beta_deg[Config::FILTER_ADOPTED_INDEX];
      calibration_probe_command_rate_raw_dps_ = status_.gyro_pitch_rate_dps;
      calibration_probe_command_rate_bias_corrected_dps_ =
          status_.gyro_pitch_rate_dps - pitchBiasFromGyroBias();
      calibration_probe_requested_side_ = requested_side;
      calibration_probe_predecessor_side_ = calibration_last_peak_side_;
      calibration_probe_direction_ = direction;
      calibration_probe_active_plan_index_ = plan_index + 1;
      calibration_probe_q_level_index_ = q_level_index;
      calibration_probe_rebuild_total_count_ = calibration_rebuild_total_count_;
      calibration_probe_rebuild_episode_id_ = calibration_rebuild_episode_id_;
      calibration_probe_rebuild_attempt_in_episode_ = calibration_rebuild_attempt_in_episode_;
      calibration_probe_rebuild_entry_source_ = predecessor_in_legacy_a_support ? 0 : 1;
      calibration_probe_rebuild_target_reached_ =
          calibration_last_peak_abs_deg_ >= Config::ZERO_CROSS_CALIBRATION_Q_REBUILD_TARGET_PEAK_DEG;
      calibration_probe_dynamic_h_in_support_at_command_ = predecessor_in_dynamic_h_support;
      calibration_probe_pulse_start_test_ms_ = t_ms;
      ++calibration_probe_attempt_count_;
      ++calibration_probe_plan_index_;
      calibration_result_.probe_plan_count = calibration_probe_plan_index_;
      calibration_phase_ = requested_side > 0 ? CalibrationPhase::WAIT_Q_CAL_POS :
                                                  CalibrationPhase::WAIT_Q_CAL_NEG;
      beginPulse(now_ms, t_ms, direction);
      calibration_probe_pulse_id_ = status_.pulse_id;
    }
    return;
  }

  status_.current_mA_setting = zero_cross_fixed_current_mA_;
  float i0 = predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  float q_target = 0.0f;
  float q_requested = 0.0f;
  float q_command = 0.0f;
  float a_target = 0.0f;
  float a_pred = 0.0f;
  float a_min = 0.0f;
  float a_max = 0.0f;
  bool target_reachable = false;
  if (q_run_mode_ == QRunMode::VALIDATION) {
    const uint8_t target_index = (identification_target_index_ / 2) %
                                 Config::ZERO_CROSS_IDENTIFICATION_TARGET_COUNT;
    q_target = Config::ZERO_CROSS_IDENTIFICATION_TARGET_Q_MAS[target_index];
    q_requested = q_target;
    q_command = q_target;
    ++identification_target_index_;
    a_pred = predictNextPeakAbsDeg(angle_deg, rate_dps, q_command, direction);
  } else if (q_run_mode_ == QRunMode::CONTROL) {
    q_requested = requiredControlQMaS(angle_deg, rate_dps, direction);
    const ControlQEvaluation evaluation = evaluateControlQ(angle_deg, rate_dps, direction);
    q_command = evaluation.q_command_mA_s;
    a_pred = evaluation.predicted_peak_deg;
    a_min = evaluation.min_predicted_peak_deg;
    a_max = evaluation.max_predicted_peak_deg;
    target_reachable = evaluation.target_reachable;
    q_target = q_command;  // retained for compatibility with existing Q tooling.
    a_target = control_target_peak_deg_;
  }
  const bool control_mode = q_run_mode_ == QRunMode::CONTROL;
  const float min_control_q_mA_s = control_mode
      ? fabsf(identificationChargeMaS(i0, direction, Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS))
      : 0.0f;
  // Below the physical 5 ms minimum, a zero-energy command is more faithful
  // to the inverse-Q request than rounding it up to a nonzero pulse.
  const bool suppressed = identification_mode_ &&
      (q_command <= 0.0f || (control_mode && q_command < min_control_q_mA_s));
  uint16_t width = identification_mode_
      ? (suppressed ? 0 : (control_mode ? controlWidthForTarget(q_command, i0, direction)
                                        : identificationWidthForTarget(q_command, i0, direction)))
      : zero_cross_fixed_pulse_ms_;
  if (control_mode && !suppressed) {
    // Width inversion initially returns the first integer-ms value reaching Q.
    // Round downward when necessary: normal control must never inject more
    // energy than the bounded continuous-Q request.
    while (width > Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS &&
           fabsf(identificationChargeMaS(i0, direction, width)) > q_command + 0.0001f) {
      --width;
    }
    // Record the physically attainable integer-ms Q prediction, not just the
    // continuous inverse target used before pulse-width quantization.
    a_pred = predictNextPeakAbsDeg(angle_deg, rate_dps,
                                   fabsf(identificationChargeMaS(i0, direction, width)), direction);
  }
  status_.pulse_width_ms_setting = width;
  zero_cross_armed_ = false;
  zero_cross_half_cycle_peak_abs_deg_ = fabsf(angle_deg);
  if (identification_mode_) {
    beginIdentificationEvent(now_ms, t_ms, direction, q_target, q_requested, q_command,
                             a_target, a_pred, a_min, a_max, target_reachable,
                             false, width, suppressed);
  }
  if (!suppressed) beginPulse(now_ms, t_ms, direction);
}

void ExperimentRunner::beginPulse(uint32_t now_ms, uint32_t t_test_ms, int8_t direction) {
  // This derived firmware has no current/pulse output path. Retain the method
  // only so a legacy scheduler call fails closed without any actuator action.
  (void)now_ms;
  (void)t_test_ms;
  (void)direction;
  stopMotor();
  status_.last_error = "motor_off_only";
}

void ExperimentRunner::stopActivePulse(uint32_t now_ms) {
  q_ident_pulse_authorized_ = false;
  energy_control_v0_pulse_authorized_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  if (roller_) roller_->stop();
  status_.motor_cmd_mA = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  last_pulse_end_ms_ = now_ms;
  if (!zero_cross_mode_) {
    next_pulse_start_test_ms_ = active_pulse_start_test_ms_ + status_.input_interval_ms;
  }
}

float ExperimentRunner::predictedChargeMaS(float signed_i0_mA, int8_t direction, float width_ms,
                                            int16_t commanded_current_mA) const {
  if (!isfinite(signed_i0_mA) || !isfinite(width_ms) || width_ms < 0.0f ||
      (direction != -1 && direction != 1) || commanded_current_mA <= 0) return NAN;
  const float v = status_.beta_model_vbat_mV > 0 ? status_.beta_model_vbat_mV / 1000.0f :
      Config::MODEL_VBAT_REFERENCE_V;
  const float target = direction * predictCurrentGoalMa(commanded_current_mA, v);
  const float tau = predictRiseTauS(commanded_current_mA);
  const float t = width_ms / 1000.0f;
  return target * t + (signed_i0_mA - target) * tau * (1.0f - expf(-t / tau));
}

float ExperimentRunner::identificationChargeMaS(float signed_i0_mA, int8_t direction, float width_ms) const {
  return predictedChargeMaS(signed_i0_mA, direction, width_ms, zero_cross_fixed_current_mA_);
}
uint16_t ExperimentRunner::widthForChargeTarget(float target_q_mA_s, float signed_i0_mA,
                                                  int8_t direction, uint16_t min_width_ms,
                                                  uint16_t max_width_ms) const {
  uint16_t lo = min_width_ms;
  uint16_t hi = max_width_ms < min_width_ms ? min_width_ms : max_width_ms;
  for (uint8_t i = 0; i < 16 && lo < hi; ++i) {
    const uint16_t mid = static_cast<uint16_t>((lo + hi) / 2);
    if (fabsf(identificationChargeMaS(signed_i0_mA, direction, mid)) < target_q_mA_s) lo = mid + 1;
    else hi = mid;
  }
  // A target may be unattainable within the supplied interval; use its upper
  // bound and preserve the resulting Q gap in metadata.
  return lo > max_width_ms ? max_width_ms : lo;
}

uint16_t ExperimentRunner::identificationWidthForTarget(float target_q_mA_s, float signed_i0_mA,
                                                          int8_t direction) const {
  return widthForChargeTarget(target_q_mA_s, signed_i0_mA, direction,
                              Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS,
                              Config::ZERO_CROSS_IDENTIFICATION_MAX_PULSE_MS);
}

uint16_t ExperimentRunner::calibrationProbeWidthForTarget(float target_q_mA_s, float signed_i0_mA,
                                                            int8_t direction) const {
  return widthForChargeTarget(target_q_mA_s, signed_i0_mA, direction,
                              Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS,
                              Config::ZERO_CROSS_CALIBRATION_Q_PROBE_MAX_PULSE_MS);
}

uint16_t ExperimentRunner::calibrationWidthForTarget(float target_q_mA_s, float signed_i0_mA,
                                                       int8_t direction) const {
  return widthForChargeTarget(target_q_mA_s, signed_i0_mA, direction,
                              Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS,
                              Config::ZERO_CROSS_CALIBRATION_INITIAL_MAX_PULSE_MS);
}

uint16_t ExperimentRunner::controlWidthForTarget(float target_q_mA_s, float signed_i0_mA,
                                                   int8_t direction) const {
  return widthForChargeTarget(target_q_mA_s, signed_i0_mA, direction,
                              Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS,
                              Config::ZERO_CROSS_CONTROL_MAX_PULSE_MS);
}

float ExperimentRunner::predictNextPeakAbsDeg(float angle_deg, float rate_dps,
                                                  float q_command_mA_s, int8_t direction) const {
  const float d = direction >= 0 ? 1.0f : -1.0f;
  return Config::ZERO_CROSS_Q_MODEL_INTERCEPT_DEG +
         Config::ZERO_CROSS_Q_MODEL_RATE_GAIN_DEG_PER_DPS * fabsf(rate_dps) +
         Config::ZERO_CROSS_Q_MODEL_ANGLE_GAIN_DEG_PER_DEG * fabsf(angle_deg) +
         Config::ZERO_CROSS_Q_MODEL_GAIN_DEG_PER_MAS * q_command_mA_s +
         Config::ZERO_CROSS_Q_MODEL_DIRECTION_OFFSET_DEG * d +
         Config::ZERO_CROSS_Q_MODEL_DIRECTION_GAIN_DEG_PER_MAS * q_command_mA_s * d;
}

float ExperimentRunner::requiredQForTargetPeakMaS(float target_peak_deg, float angle_deg,
                                                    float rate_dps, int8_t direction) const {
  const float d = direction >= 0 ? 1.0f : -1.0f;
  const float base = Config::ZERO_CROSS_Q_MODEL_INTERCEPT_DEG +
                     Config::ZERO_CROSS_Q_MODEL_RATE_GAIN_DEG_PER_DPS * fabsf(rate_dps) +
                     Config::ZERO_CROSS_Q_MODEL_ANGLE_GAIN_DEG_PER_DEG * fabsf(angle_deg) +
                     Config::ZERO_CROSS_Q_MODEL_DIRECTION_OFFSET_DEG * d;
  const float gain = Config::ZERO_CROSS_Q_MODEL_GAIN_DEG_PER_MAS +
                     Config::ZERO_CROSS_Q_MODEL_DIRECTION_GAIN_DEG_PER_MAS * d;
  return gain > 0.0f ? (target_peak_deg - base) / gain : 0.0f;
}

float ExperimentRunner::requiredControlQMaS(float angle_deg, float rate_dps, int8_t direction) const {
  return requiredQForTargetPeakMaS(control_target_peak_deg_, angle_deg, rate_dps, direction);
}

ExperimentRunner::CalibrationBuildUpCommand ExperimentRunner::calibrationBuildUpCommand(
    float angle_deg, float rate_dps, float signed_i0_mA, int8_t direction,
    float target_peak_deg, bool unrestricted) const {
  CalibrationBuildUpCommand command;
  command.q_required_mA_s = requiredQForTargetPeakMaS(
      target_peak_deg, angle_deg, rate_dps, direction);
  command.q_command_mA_s = fmaxf(0.0f, command.q_required_mA_s);
  if (!unrestricted) {
    const float q_guard = requiredQForTargetPeakMaS(
        Config::ZERO_CROSS_CALIBRATION_INITIAL_PREDICT_GUARD_DEG, angle_deg, rate_dps, direction);
    if (q_guard >= 0.0f && command.q_command_mA_s > q_guard) {
      command.q_command_mA_s = q_guard;
      command.guard_limited = true;
    }
    if (command.q_command_mA_s > Config::ZERO_CROSS_CALIBRATION_INITIAL_Q_MAX_MAS) {
      command.q_command_mA_s = Config::ZERO_CROSS_CALIBRATION_INITIAL_Q_MAX_MAS;
      command.q_cap_limited = true;
    }
  }
  const uint16_t max_width_ms = unrestricted ? static_cast<uint16_t>(65535U) :
      Config::ZERO_CROSS_CALIBRATION_INITIAL_MAX_PULSE_MS;
  command.width_ms = widthForChargeTarget(command.q_command_mA_s, signed_i0_mA, direction,
                                          Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS,
                                          max_width_ms);
  command.q_effective_mA_s = fabsf(identificationChargeMaS(signed_i0_mA, direction, command.width_ms));
  command.width_limited = command.q_effective_mA_s + 0.0001f < command.q_command_mA_s;
  command.predicted_peak_deg = predictNextPeakAbsDeg(angle_deg, rate_dps, command.q_effective_mA_s, direction);
  if (!unrestricted) {
    // Integer-ms quantization must neither exceed the requested bounded energy
    // nor turn the 18 deg prediction guard into an exceedance.
    while (command.width_ms > Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS &&
           (command.q_effective_mA_s > command.q_command_mA_s + 0.0001f ||
            command.predicted_peak_deg > Config::ZERO_CROSS_CALIBRATION_INITIAL_PREDICT_GUARD_DEG)) {
      const bool guard_before = command.predicted_peak_deg > Config::ZERO_CROSS_CALIBRATION_INITIAL_PREDICT_GUARD_DEG;
      --command.width_ms;
      command.q_effective_mA_s = fabsf(identificationChargeMaS(signed_i0_mA, direction, command.width_ms));
      command.predicted_peak_deg = predictNextPeakAbsDeg(angle_deg, rate_dps, command.q_effective_mA_s, direction);
      if (guard_before) command.guard_limited = true;
    }
  }
  return command;
}

void ExperimentRunner::recordCalibrationBuildUpPulse(uint32_t t_test_ms, float angle_deg, float rate_dps,
                                                       int8_t direction, uint8_t phase, float target_peak_deg,
                                                       const CalibrationBuildUpCommand& command) {
  if (!logger_) return;
  PsramLogger::CalibrationBuildUpEvent event;
  event.start_ms = t_test_ms;
  event.theta0_cdeg = centi(angle_deg);
  event.rate0_cdps = centi(rate_dps);
  event.direction = direction;
  event.phase = phase;
  event.target_peak_cdeg = centi(target_peak_deg);
  event.vbat_mV = status_.beta_model_vbat_mV;
  event.pulse_width_ms = command.width_ms;
  event.q_required_mAms = lroundf(command.q_required_mA_s * 1000.0f);
  event.q_command_mAms = lroundf(command.q_command_mA_s * 1000.0f);
  event.q_effective_mAms = lroundf(command.q_effective_mA_s * 1000.0f);
  event.predicted_peak_cdeg = centi(command.predicted_peak_deg);
  event.q_cap_limited = command.q_cap_limited;
  event.guard_limited = command.guard_limited;
  event.width_limited = command.width_limited;
  logger_->addCalibrationBuildUpEvent(event);
}

ExperimentRunner::ControlQEvaluation ExperimentRunner::evaluateControlQ(float angle_deg, float rate_dps,
                                                                          int8_t direction) const {
  ControlQEvaluation evaluation;
  const float q_requested = requiredControlQMaS(angle_deg, rate_dps, direction);
  // Normal zero-cross control is continuous-Q. The identical 3.80 mA*s cap
  // used for calibration is an experimentally observed near-limit input, not
  // a raw width cap; its physical width is solved from I0 and Vbat below.
  evaluation.q_command_mA_s = fminf(Config::ZERO_CROSS_CONTROL_Q_MAX_MAS,
                                     fmaxf(0.0f, q_requested));
  evaluation.predicted_peak_deg = predictNextPeakAbsDeg(
      angle_deg, rate_dps, evaluation.q_command_mA_s, direction);
  evaluation.min_predicted_peak_deg = predictNextPeakAbsDeg(angle_deg, rate_dps, 0.0f, direction);
  evaluation.max_predicted_peak_deg = predictNextPeakAbsDeg(
      angle_deg, rate_dps, Config::ZERO_CROSS_CONTROL_Q_MAX_MAS, direction);
  evaluation.target_reachable = q_requested >= 0.0f &&
                                q_requested <= Config::ZERO_CROSS_CONTROL_Q_MAX_MAS;
  return evaluation;
}

void ExperimentRunner::resetCalibrationShadow() {
  calibration_enabled_ = Config::ZERO_CROSS_CALIBRATION_SHADOW_ENABLED && q_run_mode_ == QRunMode::CONTROL;
  calibration_phase_ = calibration_enabled_ ? CalibrationPhase::INITIAL_EXCITE : CalibrationPhase::INACTIVE;
  calibration_peak_tracker_ready_ = false;
  calibration_expected_candidate_side_ = 0;
  calibration_outbound_rate_sign_ = 0;
  calibration_reverse_samples_ = 0;
  calibration_peak_candidate_deg_ = 0.0f;
  calibration_peak_candidate_fixed_deg_ = 0.0f;
  calibration_peak_candidate_ms_ = 0;
  calibration_last_peak_valid_ = false;
  calibration_last_peak_side_ = 0;
  calibration_last_peak_abs_deg_ = 0.0f;
  calibration_last_peak_signed_dynamic_deg_ = 0.0f;
  calibration_last_peak_signed_fixed_deg_ = 0.0f;
  calibration_last_half_range_dynamic_deg_ = 0.0f;
  calibration_last_half_range_fixed_deg_ = 0.0f;
  calibration_last_half_range_valid_ = false;
  calibration_last_center_dynamic_deg_ = 0.0f;
  calibration_last_center_dynamic_valid_ = false;
  calibration_last_peak_candidate_ms_ = 0;
  calibration_fit_pos_ = CalibrationFit{};
  calibration_fit_neg_ = CalibrationFit{};
  calibration_half_range_dynamic_fit_ = HalfRangeFit{};
  calibration_half_range_fixed_fit_ = HalfRangeFit{};
  calibration_free_last_half_range_dynamic_deg_ = 0.0f;
  calibration_free_last_half_range_fixed_deg_ = 0.0f;
  calibration_free_last_half_range_valid_ = false;
  calibration_result_ = PsramLogger::CalibrationResult{};
  calibration_result_.enabled = calibration_enabled_;
  calibration_probe_previous_peak_abs_deg_ = 0.0f;
  calibration_probe_previous_peak_signed_dynamic_deg_ = 0.0f;
  calibration_probe_previous_peak_signed_fixed_deg_ = 0.0f;
  calibration_probe_previous_half_range_dynamic_deg_ = 0.0f;
  calibration_probe_previous_half_range_fixed_deg_ = 0.0f;
  calibration_probe_previous_half_range_valid_ = false;
  calibration_probe_previous_center_dynamic_deg_ = 0.0f;
  calibration_probe_previous_center_dynamic_valid_ = false;
  calibration_probe_previous_peak_candidate_ms_ = 0;
  calibration_probe_q_target_mA_s_ = 0.0f;
  calibration_probe_q_effective_pred_mA_s_ = 0.0f;
  calibration_probe_command_dynamic_angle_deg_ = 0.0f;
  calibration_probe_command_rate_raw_dps_ = 0.0f;
  calibration_probe_command_rate_bias_corrected_dps_ = 0.0f;
  calibration_probe_pulse_start_test_ms_ = 0;
  calibration_probe_pulse_id_ = 0;
  calibration_probe_requested_side_ = 0;
  calibration_probe_predecessor_side_ = 0;
  calibration_probe_direction_ = 0;
  calibration_probe_plan_index_ = 0;
  calibration_probe_active_plan_index_ = 0;
  calibration_probe_q_level_index_ = 0;
  calibration_probe_rebuild_total_count_ = 0;
  calibration_probe_rebuild_episode_id_ = 0;
  calibration_probe_rebuild_attempt_in_episode_ = 0;
  calibration_probe_rebuild_entry_source_ = 0;
  calibration_probe_rebuild_target_reached_ = false;
  calibration_probe_dynamic_h_in_support_at_command_ = false;
  calibration_probe_wait_halfcycle_count_ = 0;
  calibration_probe_wait_halfcycle_total_ = 0;
  calibration_probe_attempt_count_ = 0;
  calibration_v59_gate_event_count_ = 0;
  calibration_v59_gate_pass_count_ = 0;
  calibration_v59_gate_skip_count_ = 0;
  calibration_v59_rebuild_from_low_state_count_ = 0;
  calibration_v60_cooldown_halfcycles_remaining_ = 0;
  calibration_v60_high_desired_wait_pending_ = false;
  calibration_v61_rebuild_plan_active_ = false;
  calibration_v61_nominal_controlled_peak_target_deg_ = 0.0f;
  calibration_v61_command_peak_target_deg_ = 0.0f;
  calibration_v62_rate_fit_ = RateStateFit{};
  calibration_gain_sum_pos_ = 0.0f;
  calibration_gain_sum_neg_ = 0.0f;
  calibration_gain_sum_sq_pos_ = 0.0f;
  calibration_gain_sum_sq_neg_ = 0.0f;
  calibration_initial_kick_count_ = 0;
  calibration_initial_kick_q_effective_pred_mA_s_ = 0.0f;
  calibration_rebuild_pulse_id_ = 0;
  calibration_rebuild_pulse_start_test_ms_ = 0;
  calibration_rebuild_total_count_ = 0;
  calibration_rebuild_episode_id_ = 0;
  calibration_rebuild_attempt_in_episode_ = 0;
  if (logger_) logger_->setCalibrationResult(calibration_result_);
}

void ExperimentRunner::recordCalibrationPeak(uint32_t candidate_peak_ms, uint32_t confirmed_ms,
                                              int8_t side, float peak_signed_dynamic_deg,
                                              float peak_signed_fixed_deg, uint8_t phase,
                                              float previous_peak_abs_deg,
                                              float q_effective_pred_mA_s) {
  if (!logger_) return;
  PsramLogger::CalibrationPeakEvent event;
  event.candidate_peak_ms = candidate_peak_ms;
  event.confirmed_ms = confirmed_ms;
  event.peak_side = side;
  event.phase = phase;
  event.peak_abs_cdeg = centi(fabsf(peak_signed_dynamic_deg));
  event.prev_peak_abs_cdeg = previous_peak_abs_deg >= 0.0f ? centi(previous_peak_abs_deg) : LOG_NAN_I16;
  event.peak_signed_dynamic_cdeg = centi(peak_signed_dynamic_deg);
  event.peak_signed_fixed_cdeg = centi(peak_signed_fixed_deg);
  if (calibration_last_peak_valid_) {
    event.prev_peak_signed_dynamic_cdeg = centi(calibration_last_peak_signed_dynamic_deg_);
    event.center_dynamic_cdeg = centi(0.5f * (calibration_last_peak_signed_dynamic_deg_ +
                                               peak_signed_dynamic_deg));
    event.half_range_dynamic_cdeg = centi(0.5f * fabsf(peak_signed_dynamic_deg -
                                                        calibration_last_peak_signed_dynamic_deg_));
    event.prev_peak_signed_fixed_cdeg = centi(calibration_last_peak_signed_fixed_deg_);
    event.center_fixed_cdeg = centi(0.5f * (calibration_last_peak_signed_fixed_deg_ + peak_signed_fixed_deg));
    event.half_range_fixed_cdeg = centi(0.5f * fabsf(peak_signed_fixed_deg -
                                                      calibration_last_peak_signed_fixed_deg_));
  }
  event.q_effective_pred_mAms = lroundf(q_effective_pred_mA_s * 1000.0f);
  logger_->addCalibrationPeakEvent(event);
}

void ExperimentRunner::addHalfRangeFreePair(HalfRangeFit& fit, float previous_half_range_deg,
                                             float half_range_deg) {
  if (!isfinite(previous_half_range_deg) || !isfinite(half_range_deg) ||
      previous_half_range_deg <= 0.0f || half_range_deg <= 0.0f ||
      fit.count >= HalfRangeFit::kMaxPairs) return;
  if (fit.count == 0) {
    fit.input_min_deg = previous_half_range_deg;
    fit.input_max_deg = previous_half_range_deg;
  } else {
    fit.input_min_deg = fminf(fit.input_min_deg, previous_half_range_deg);
    fit.input_max_deg = fmaxf(fit.input_max_deg, previous_half_range_deg);
  }
  fit.x_deg[fit.count] = previous_half_range_deg;
  fit.y_deg[fit.count] = half_range_deg;
  ++fit.count;
  fit.sum_x += previous_half_range_deg; fit.sum_y += half_range_deg;
  fit.sum_x2 += previous_half_range_deg * previous_half_range_deg;
  fit.sum_xy += previous_half_range_deg * half_range_deg;
}

bool ExperimentRunner::fitHalfRangeShadow(HalfRangeFit& fit, bool dynamic) {
  const float n = static_cast<float>(fit.count);
  if (fit.count < 3) return false;
  const float det = n * fit.sum_x2 - fit.sum_x * fit.sum_x;
  if (!isfinite(det) || fabsf(det) < 1.0e-6f) return false;
  const float r = (n * fit.sum_xy - fit.sum_x * fit.sum_y) / det;
  const float c = (fit.sum_y - r * fit.sum_x) / n;
  float squared_error = 0.0f;
  for (uint8_t i = 0; i < fit.count; ++i) {
    const float error = fit.y_deg[i] - (r * fit.x_deg[i] + c);
    squared_error += error * error;
  }
  float loocv_squared_error = 0.0f;
  uint8_t loocv_count = 0;
  for (uint8_t omit = 0; omit < fit.count; ++omit) {
    const float nn = n - 1.0f;
    const float sx = fit.sum_x - fit.x_deg[omit];
    const float sy = fit.sum_y - fit.y_deg[omit];
    const float sx2 = fit.sum_x2 - fit.x_deg[omit] * fit.x_deg[omit];
    const float sxy = fit.sum_xy - fit.x_deg[omit] * fit.y_deg[omit];
    const float loo_det = nn * sx2 - sx * sx;
    if (nn < 2.0f || !isfinite(loo_det) || fabsf(loo_det) < 1.0e-6f) continue;
    const float loo_r = (nn * sxy - sx * sy) / loo_det;
    const float loo_c = (sy - loo_r * sx) / nn;
    const float error = fit.y_deg[omit] - (loo_r * fit.x_deg[omit] + loo_c);
    loocv_squared_error += error * error;
    ++loocv_count;
  }
  const float rmse = sqrtf(squared_error / n);
  const float loocv = loocv_count ? sqrtf(loocv_squared_error / loocv_count) : NAN;
  const bool valid = isfinite(r) && isfinite(c) && isfinite(rmse) && isfinite(loocv) &&
                     r > 0.0f && fit.input_min_deg > 0.0f && fit.input_max_deg >= fit.input_min_deg;
  if (dynamic) {
    calibration_result_.half_range_dynamic_r = r;
    calibration_result_.half_range_dynamic_c_deg = c;
    calibration_result_.half_range_dynamic_rmse_deg = rmse;
    calibration_result_.half_range_dynamic_loocv_rmse_deg = loocv;
    calibration_result_.half_range_dynamic_input_min_deg = fit.input_min_deg;
    calibration_result_.half_range_dynamic_input_max_deg = fit.input_max_deg;
    calibration_result_.half_range_dynamic_valid = valid;
  } else {
    calibration_result_.half_range_fixed_r = r;
    calibration_result_.half_range_fixed_c_deg = c;
    calibration_result_.half_range_fixed_rmse_deg = rmse;
    calibration_result_.half_range_fixed_loocv_rmse_deg = loocv;
    calibration_result_.half_range_fixed_input_min_deg = fit.input_min_deg;
    calibration_result_.half_range_fixed_input_max_deg = fit.input_max_deg;
    calibration_result_.half_range_fixed_valid = valid;
  }
  return valid;
}

bool ExperimentRunner::v60GateCenterInDynamicHInputSupport() const {
  const float h_gate = 0.5f * (Config::ZERO_CROSS_V59_GATE_HPREV_MIN_DEG +
                               Config::ZERO_CROSS_V59_GATE_HPREV_MAX_DEG);
  return calibration_result_.half_range_dynamic_valid &&
         isfinite(calibration_result_.half_range_dynamic_r) &&
         isfinite(calibration_result_.half_range_dynamic_c_deg) &&
         isfinite(calibration_result_.half_range_dynamic_input_min_deg) &&
         isfinite(calibration_result_.half_range_dynamic_input_max_deg) &&
         h_gate >= calibration_result_.half_range_dynamic_input_min_deg &&
         h_gate <= calibration_result_.half_range_dynamic_input_max_deg;
}

bool ExperimentRunner::configureV60RebuildTarget() {
  // V61 targets a *state*, not a raw peak.  It commands the side opposite to
  // the fixed-Q arrival, then predicts the next two unforced peaks with the
  // measured side-specific maps.  The predicted H/C must land in the unchanged
  // narrow V59 gate before the cooldown is allowed.
  const float h_gate = 0.5f * (Config::ZERO_CROSS_V59_GATE_HPREV_MIN_DEG +
                               Config::ZERO_CROSS_V59_GATE_HPREV_MAX_DEG);
  const float c_gate = 0.5f * (Config::ZERO_CROSS_V59_GATE_CPREV_MIN_DEG +
                               Config::ZERO_CROSS_V59_GATE_CPREV_MAX_DEG);
  calibration_result_.v60_gate_center_h_deg = h_gate;
  calibration_result_.v60_gate_center_c_deg = c_gate;
  calibration_result_.v60_rebuild_target_valid = false;
  calibration_result_.v60_rebuild_target_observed = false;
  calibration_result_.v61_state_target_valid = false;
  calibration_result_.v61_predicted_gate_h_deg = 0.0f;
  calibration_result_.v61_predicted_gate_c_deg = 0.0f;
  calibration_result_.v61_feedback_command_peak_deg = 0.0f;
  calibration_result_.v61_feedback_correction_count = 0;
  calibration_v61_rebuild_plan_active_ = false;
  calibration_v61_nominal_controlled_peak_target_deg_ = 0.0f;
  calibration_v61_command_peak_target_deg_ = 0.0f;
  calibration_result_.v60_gate_center_in_dynamic_h_input_support =
      v60GateCenterInDynamicHInputSupport();
  if (!calibration_result_.v60_gate_center_in_dynamic_h_input_support ||
      calibration_result_.free_model_pos == 0 || calibration_result_.free_model_neg == 0) {
    return false;
  }
  const float r_pos = calibration_result_.r_pos;
  const float c_pos = calibration_result_.c_pos_deg;
  const float r_neg = calibration_result_.r_neg;
  const float c_neg = calibration_result_.c_neg_deg;
  if (!isfinite(r_pos) || !isfinite(c_pos) || !isfinite(r_neg) || !isfinite(c_neg) ||
      r_pos <= 0.0f || r_neg <= 0.0f) return false;

  float a_pos = 0.0f;
  float a_neg = 0.0f;
  float controlled_peak = 0.0f;
  if (Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE > 0) {
    // controlled negative -> free positive -> free negative -> fixed-Q positive
    a_pos = (2.0f * h_gate - c_neg) / (1.0f + r_neg);
    controlled_peak = (a_pos - c_pos) / r_pos;
    a_neg = r_neg * a_pos + c_neg;
  } else {
    // controlled positive -> free negative -> free positive -> fixed-Q negative
    a_neg = (2.0f * h_gate - c_pos) / (1.0f + r_pos);
    controlled_peak = (a_neg - c_neg) / r_neg;
    a_pos = r_pos * a_neg + c_pos;
  }
  const float predicted_h = 0.5f * (a_pos + a_neg);
  const float predicted_c = 0.5f * (a_pos - a_neg);
  const bool controlled_in_support =
      (Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE > 0 &&
       controlled_peak >= calibration_result_.free_input_min_pos_deg &&
       controlled_peak <= calibration_result_.free_input_max_pos_deg &&
       a_pos >= calibration_result_.free_input_min_neg_deg &&
       a_pos <= calibration_result_.free_input_max_neg_deg) ||
      (Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE < 0 &&
       controlled_peak >= calibration_result_.free_input_min_neg_deg &&
       controlled_peak <= calibration_result_.free_input_max_neg_deg &&
       a_neg >= calibration_result_.free_input_min_pos_deg &&
       a_neg <= calibration_result_.free_input_max_pos_deg);
  const bool predicted_in_gate = predicted_h >= Config::ZERO_CROSS_V59_GATE_HPREV_MIN_DEG &&
      predicted_h <= Config::ZERO_CROSS_V59_GATE_HPREV_MAX_DEG &&
      predicted_c >= Config::ZERO_CROSS_V59_GATE_CPREV_MIN_DEG &&
      predicted_c <= Config::ZERO_CROSS_V59_GATE_CPREV_MAX_DEG;
  if (!controlled_in_support || !predicted_in_gate || !isfinite(controlled_peak) ||
      controlled_peak <= 0.0f || !isfinite(predicted_h) || !isfinite(predicted_c)) return false;

  const float h_r = calibration_result_.half_range_dynamic_r;
  const float h_c = calibration_result_.half_range_dynamic_c_deg;
  const float h_after_cycle = h_r * (h_r * h_gate + h_c) + h_c;
  calibration_result_.v60_free_decay_after_one_cycle_h_deg = h_after_cycle;
  calibration_result_.v60_decay_per_cycle_h_deg = h_gate - h_after_cycle;
  calibration_result_.v60_rebuild_target_h_deg = predicted_h;
  calibration_result_.v60_rebuild_target_peak_deg = controlled_peak;
  calibration_result_.v60_rebuild_target_valid = true;
  calibration_result_.v61_state_target_valid = true;
  calibration_result_.v61_predicted_gate_h_deg = predicted_h;
  calibration_result_.v61_predicted_gate_c_deg = predicted_c;
  calibration_result_.v61_feedback_command_peak_deg = controlled_peak;
  calibration_v61_nominal_controlled_peak_target_deg_ = controlled_peak;
  calibration_v61_command_peak_target_deg_ = controlled_peak;
  calibration_v61_rebuild_plan_active_ = true;
  return true;
}

bool ExperimentRunner::predictV61GateState(float controlled_peak_abs_deg, float* predicted_h_deg,
                                            float* predicted_c_deg) const {
  if (predicted_h_deg) *predicted_h_deg = NAN;
  if (predicted_c_deg) *predicted_c_deg = NAN;
  if (!calibration_result_.v61_state_target_valid || !isfinite(controlled_peak_abs_deg) ||
      controlled_peak_abs_deg <= 0.0f) return false;
  const float r_pos = calibration_result_.r_pos;
  const float c_pos = calibration_result_.c_pos_deg;
  const float r_neg = calibration_result_.r_neg;
  const float c_neg = calibration_result_.c_neg_deg;
  if (!isfinite(r_pos) || !isfinite(c_pos) || !isfinite(r_neg) || !isfinite(c_neg) ||
      r_pos <= 0.0f || r_neg <= 0.0f) return false;
  float a_pos = 0.0f;
  float a_neg = 0.0f;
  if (Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE > 0) {
    a_neg = controlled_peak_abs_deg;
    if (a_neg < calibration_result_.free_input_min_pos_deg ||
        a_neg > calibration_result_.free_input_max_pos_deg) return false;
    a_pos = r_pos * a_neg + c_pos;
    if (a_pos < calibration_result_.free_input_min_neg_deg ||
        a_pos > calibration_result_.free_input_max_neg_deg) return false;
    a_neg = r_neg * a_pos + c_neg;
  } else {
    a_pos = controlled_peak_abs_deg;
    if (a_pos < calibration_result_.free_input_min_neg_deg ||
        a_pos > calibration_result_.free_input_max_neg_deg) return false;
    a_neg = r_neg * a_pos + c_neg;
    if (a_neg < calibration_result_.free_input_min_pos_deg ||
        a_neg > calibration_result_.free_input_max_pos_deg) return false;
    a_pos = r_pos * a_neg + c_pos;
  }
  const float h = 0.5f * (a_pos + a_neg);
  const float c = 0.5f * (a_pos - a_neg);
  if (!isfinite(h) || !isfinite(c) || h <= 0.0f) return false;
  if (predicted_h_deg) *predicted_h_deg = h;
  if (predicted_c_deg) *predicted_c_deg = c;
  return true;
}

void ExperimentRunner::addV62RateStateSample(float hprev_deg, float cprev_deg, float abs_rate_dps) {
  if (!isfinite(hprev_deg) || !isfinite(cprev_deg) || !isfinite(abs_rate_dps) || hprev_deg <= 0.0f || abs_rate_dps <= 0.0f) return;
  RateStateFit& fit = calibration_v62_rate_fit_;
  if (fit.count == 0) {
    fit.h_min_deg = fit.h_max_deg = hprev_deg;
    fit.c_min_deg = fit.c_max_deg = cprev_deg;
  } else {
    fit.h_min_deg = fminf(fit.h_min_deg, hprev_deg); fit.h_max_deg = fmaxf(fit.h_max_deg, hprev_deg);
    fit.c_min_deg = fminf(fit.c_min_deg, cprev_deg); fit.c_max_deg = fmaxf(fit.c_max_deg, cprev_deg);
  }
  ++fit.count;
  fit.sum_h += hprev_deg; fit.sum_c += cprev_deg; fit.sum_rate += abs_rate_dps;
  fit.sum_h2 += hprev_deg * hprev_deg; fit.sum_hc += hprev_deg * cprev_deg; fit.sum_c2 += cprev_deg * cprev_deg;
  fit.sum_hrate += hprev_deg * abs_rate_dps; fit.sum_crate += cprev_deg * abs_rate_dps; fit.sum_rate2 += abs_rate_dps * abs_rate_dps;
  calibration_result_.v62_rate_sample_count = fit.count;
}

bool ExperimentRunner::fitV62RateStateModel() {
  RateStateFit& fit = calibration_v62_rate_fit_;
  calibration_result_.v62_rate_model_valid = false;
  calibration_result_.v62_rate_sample_count = fit.count;
  if (fit.count < Config::ZERO_CROSS_V62_RATE_MODEL_MIN_SAMPLES) return false;
  float m[3][4] = {{fit.sum_h2 + Config::ZERO_CROSS_V62_RATE_MODEL_RIDGE, fit.sum_hc, fit.sum_h, fit.sum_hrate},
                   {fit.sum_hc, fit.sum_c2 + Config::ZERO_CROSS_V62_RATE_MODEL_RIDGE, fit.sum_c, fit.sum_crate},
                   {fit.sum_h, fit.sum_c, static_cast<float>(fit.count) + Config::ZERO_CROSS_V62_RATE_MODEL_RIDGE, fit.sum_rate}};
  for (uint8_t col = 0; col < 3; ++col) {
    uint8_t pivot = col;
    for (uint8_t row = col + 1; row < 3; ++row) if (fabsf(m[row][col]) > fabsf(m[pivot][col])) pivot = row;
    if (!isfinite(m[pivot][col]) || fabsf(m[pivot][col]) < 1.0e-5f) return false;
    if (pivot != col) for (uint8_t j = col; j < 4; ++j) { const float t = m[col][j]; m[col][j] = m[pivot][j]; m[pivot][j] = t; }
    const float divisor = m[col][col];
    for (uint8_t j = col; j < 4; ++j) m[col][j] /= divisor;
    for (uint8_t row = 0; row < 3; ++row) if (row != col) { const float factor = m[row][col]; for (uint8_t j = col; j < 4; ++j) m[row][j] -= factor * m[col][j]; }
  }
  fit.a_per_s = m[0][3]; fit.b_per_s = m[1][3]; fit.offset_dps = m[2][3];
  const float sse = fit.sum_rate2 - 2.0f * (fit.a_per_s * fit.sum_hrate + fit.b_per_s * fit.sum_crate + fit.offset_dps * fit.sum_rate) + fit.a_per_s * fit.a_per_s * fit.sum_h2 + fit.b_per_s * fit.b_per_s * fit.sum_c2 + fit.offset_dps * fit.offset_dps * static_cast<float>(fit.count) + 2.0f * fit.a_per_s * fit.b_per_s * fit.sum_hc + 2.0f * fit.a_per_s * fit.offset_dps * fit.sum_h + 2.0f * fit.b_per_s * fit.offset_dps * fit.sum_c;
  const float mean = fit.sum_rate / static_cast<float>(fit.count);
  const float sst = fit.sum_rate2 - static_cast<float>(fit.count) * mean * mean;
  fit.r2 = sst > 1.0e-4f ? 1.0f - fmaxf(0.0f, sse) / sst : NAN;
  calibration_result_.v62_rate_a_per_s = fit.a_per_s; calibration_result_.v62_rate_b_per_s = fit.b_per_s;
  calibration_result_.v62_rate_offset_dps = fit.offset_dps; calibration_result_.v62_rate_r2 = fit.r2;
  calibration_result_.v62_rate_model_valid = isfinite(fit.a_per_s) && isfinite(fit.b_per_s) && isfinite(fit.offset_dps) && isfinite(fit.r2) && fit.r2 >= Config::ZERO_CROSS_V62_RATE_MODEL_MIN_R2;
  return calibration_result_.v62_rate_model_valid;
}

bool ExperimentRunner::v62HasFeasibleHCTarget() const {
  const float r_pos = calibration_result_.r_pos, c_pos = calibration_result_.c_pos_deg;
  const float r_neg = calibration_result_.r_neg, c_neg = calibration_result_.c_neg_deg;
  if (calibration_result_.free_model_pos == 0 || calibration_result_.free_model_neg == 0 || !calibration_result_.half_range_dynamic_valid || !isfinite(r_pos) || !isfinite(c_pos) || !isfinite(r_neg) || !isfinite(c_neg) || r_pos <= 0.0f || r_neg <= 0.0f) return false;
  const bool desired_positive = Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE > 0;
  const float low = desired_positive ? calibration_result_.free_input_min_pos_deg : calibration_result_.free_input_min_neg_deg;
  const float high = desired_positive ? calibration_result_.free_input_max_pos_deg : calibration_result_.free_input_max_neg_deg;
  for (float controlled = low; controlled <= high + 0.0001f; controlled += Config::ZERO_CROSS_V62_TARGET_SCAN_STEP_DEG) {
    float a_pos = 0.0f, a_neg = 0.0f; bool supported = false;
    if (desired_positive) { a_neg = controlled; a_pos = r_pos * a_neg + c_pos; supported = a_pos >= calibration_result_.free_input_min_neg_deg && a_pos <= calibration_result_.free_input_max_neg_deg; a_neg = r_neg * a_pos + c_neg; }
    else { a_pos = controlled; a_neg = r_neg * a_pos + c_neg; supported = a_neg >= calibration_result_.free_input_min_pos_deg && a_neg <= calibration_result_.free_input_max_pos_deg; a_pos = r_pos * a_neg + c_pos; }
    const float h = 0.5f * (a_pos + a_neg), c = 0.5f * (a_pos - a_neg);
    if (supported && isfinite(h) && isfinite(c) && h >= Config::ZERO_CROSS_V59_GATE_HPREV_MIN_DEG && h <= Config::ZERO_CROSS_V59_GATE_HPREV_MAX_DEG && c >= Config::ZERO_CROSS_V59_GATE_CPREV_MIN_DEG && c <= Config::ZERO_CROSS_V59_GATE_CPREV_MAX_DEG && h >= calibration_result_.half_range_dynamic_input_min_deg && h <= calibration_result_.half_range_dynamic_input_max_deg) return true;
  }
  return false;
}

bool ExperimentRunner::configureV62RebuildTarget() {
  calibration_result_.v62_hc_target_feasible = v62HasFeasibleHCTarget();
  calibration_result_.v62_state_target_valid = false;
  if (!calibration_result_.v62_hc_target_feasible || !calibration_result_.v62_rate_model_valid) return false;
  const RateStateFit& fit = calibration_v62_rate_fit_;
  const float r_pos = calibration_result_.r_pos, c_pos = calibration_result_.c_pos_deg;
  const float r_neg = calibration_result_.r_neg, c_neg = calibration_result_.c_neg_deg;
  const bool desired_positive = Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE > 0;
  const float low = desired_positive ? calibration_result_.free_input_min_pos_deg : calibration_result_.free_input_min_neg_deg;
  const float high = desired_positive ? calibration_result_.free_input_max_pos_deg : calibration_result_.free_input_max_neg_deg;
  float best_score = -1.0f, best_controlled = 0.0f, best_h = 0.0f, best_c = 0.0f, best_rate = 0.0f;
  for (float controlled = low; controlled <= high + 0.0001f; controlled += Config::ZERO_CROSS_V62_TARGET_SCAN_STEP_DEG) {
    float a_pos = 0.0f, a_neg = 0.0f; bool supported = false;
    if (desired_positive) { a_neg = controlled; a_pos = r_pos * a_neg + c_pos; supported = a_pos >= calibration_result_.free_input_min_neg_deg && a_pos <= calibration_result_.free_input_max_neg_deg; a_neg = r_neg * a_pos + c_neg; }
    else { a_pos = controlled; a_neg = r_neg * a_pos + c_neg; supported = a_neg >= calibration_result_.free_input_min_pos_deg && a_neg <= calibration_result_.free_input_max_pos_deg; a_pos = r_pos * a_neg + c_pos; }
    const float h = 0.5f * (a_pos + a_neg), c = 0.5f * (a_pos - a_neg), rate = fit.a_per_s * h + fit.b_per_s * c + fit.offset_dps;
    const bool inside = supported && isfinite(h) && isfinite(c) && isfinite(rate) && h >= Config::ZERO_CROSS_V59_GATE_HPREV_MIN_DEG && h <= Config::ZERO_CROSS_V59_GATE_HPREV_MAX_DEG && c >= Config::ZERO_CROSS_V59_GATE_CPREV_MIN_DEG && c <= Config::ZERO_CROSS_V59_GATE_CPREV_MAX_DEG && rate >= Config::ZERO_CROSS_V59_GATE_ABS_RATE_MIN_DPS && rate <= Config::ZERO_CROSS_V59_GATE_ABS_RATE_MAX_DPS && h >= calibration_result_.half_range_dynamic_input_min_deg && h <= calibration_result_.half_range_dynamic_input_max_deg && h >= fit.h_min_deg && h <= fit.h_max_deg && c >= fit.c_min_deg && c <= fit.c_max_deg;
    if (!inside) continue;
    const float score = fminf(fminf(h - Config::ZERO_CROSS_V59_GATE_HPREV_MIN_DEG, Config::ZERO_CROSS_V59_GATE_HPREV_MAX_DEG - h) / 0.125f, fminf(c - Config::ZERO_CROSS_V59_GATE_CPREV_MIN_DEG, Config::ZERO_CROSS_V59_GATE_CPREV_MAX_DEG - c) / 0.30f);
    if (score > best_score) { best_score = score; best_controlled = controlled; best_h = h; best_c = c; best_rate = rate; }
  }
  if (best_score < 0.0f) return false;
  calibration_result_.v62_state_target_valid = true;
  calibration_result_.v62_target_h_deg = best_h; calibration_result_.v62_target_c_deg = best_c; calibration_result_.v62_target_rate_dps = best_rate; calibration_result_.v62_target_controlled_peak_deg = best_controlled;
  calibration_result_.v60_rebuild_target_valid = true; calibration_result_.v60_rebuild_target_h_deg = best_h; calibration_result_.v60_rebuild_target_peak_deg = best_controlled;
  calibration_result_.v61_state_target_valid = true; calibration_result_.v61_predicted_gate_h_deg = best_h; calibration_result_.v61_predicted_gate_c_deg = best_c; calibration_result_.v61_feedback_command_peak_deg = best_controlled;
  calibration_v61_nominal_controlled_peak_target_deg_ = best_controlled; calibration_v61_command_peak_target_deg_ = best_controlled; calibration_v61_rebuild_plan_active_ = true;
  return true;
}
float ExperimentRunner::halfRangeShadowPrediction(float previous_half_range_deg, bool dynamic,
                                                   bool* in_support) const {
  if (in_support) *in_support = false;
  const bool valid = dynamic ? calibration_result_.half_range_dynamic_valid :
                               calibration_result_.half_range_fixed_valid;
  const float min_input = dynamic ? calibration_result_.half_range_dynamic_input_min_deg :
                                    calibration_result_.half_range_fixed_input_min_deg;
  const float max_input = dynamic ? calibration_result_.half_range_dynamic_input_max_deg :
                                    calibration_result_.half_range_fixed_input_max_deg;
  if (!valid || !isfinite(previous_half_range_deg) || previous_half_range_deg < min_input ||
      previous_half_range_deg > max_input) return NAN;
  if (in_support) *in_support = true;
  const float r = dynamic ? calibration_result_.half_range_dynamic_r : calibration_result_.half_range_fixed_r;
  const float c = dynamic ? calibration_result_.half_range_dynamic_c_deg : calibration_result_.half_range_fixed_c_deg;
  return r * previous_half_range_deg + c;
}

bool ExperimentRunner::dynamicHalfRangePredecessorInSupport() const {
  if (!calibration_last_half_range_valid_) return false;
  bool in_support = false;
  (void)halfRangeShadowPrediction(calibration_last_half_range_dynamic_deg_, true, &in_support);
  return in_support;
}

bool ExperimentRunner::fitCalibrationFreeDecay() {
  // Half-cycle amplitudes may be asymmetric. Each arrival-side map must stay
  // positive and monotone over its measured A_prev domain; passive decay is
  // tested on the composed same-side full cycle, not on either half cycle alone.
  auto evaluate = [&](const CalibrationFit& input, bool positive) {
    const float n = static_cast<float>(input.count);
    const bool enough = input.count >= Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_PER_SIDE;
    const float input_min = input.input_min_deg;
    const float input_max = input.input_max_deg;
    const float sst = n > 0.0f ? input.sum_y2 - input.sum_y * input.sum_y / n : 0.0f;
    const bool usable_input = isfinite(input_min) && isfinite(input_max) &&
                              input_min > 0.0f && input_max >= input_min;
    float& prop_r = positive ? calibration_result_.free_proportional_r_pos : calibration_result_.free_proportional_r_neg;
    float& prop_rmse = positive ? calibration_result_.free_proportional_rmse_pos_deg : calibration_result_.free_proportional_rmse_neg_deg;
    float& prop_r2 = positive ? calibration_result_.free_proportional_r2_pos : calibration_result_.free_proportional_r2_neg;
    float& prop_loocv = positive ? calibration_result_.free_proportional_loocv_rmse_pos_deg : calibration_result_.free_proportional_loocv_rmse_neg_deg;
    bool& prop_valid = positive ? calibration_result_.free_proportional_valid_pos : calibration_result_.free_proportional_valid_neg;
    float& affine_r = positive ? calibration_result_.free_affine_r_pos : calibration_result_.free_affine_r_neg;
    float& affine_c = positive ? calibration_result_.free_affine_c_pos_deg : calibration_result_.free_affine_c_neg_deg;
    float& affine_rmse = positive ? calibration_result_.free_affine_rmse_pos_deg : calibration_result_.free_affine_rmse_neg_deg;
    float& affine_r2 = positive ? calibration_result_.free_affine_r2_pos : calibration_result_.free_affine_r2_neg;
    float& affine_loocv = positive ? calibration_result_.free_affine_loocv_rmse_pos_deg : calibration_result_.free_affine_loocv_rmse_neg_deg;
    bool& affine_valid = positive ? calibration_result_.free_affine_valid_pos : calibration_result_.free_affine_valid_neg;
    uint8_t& model = positive ? calibration_result_.free_model_pos : calibration_result_.free_model_neg;
    float& selected_r = positive ? calibration_result_.r_pos : calibration_result_.r_neg;
    float& selected_c = positive ? calibration_result_.c_pos_deg : calibration_result_.c_neg_deg;

    auto loocvRmse = [&](bool affine) {
      if (input.count < 3) return NAN;
      float total_squared_error = 0.0f;
      for (uint8_t held_out = 0; held_out < input.count; ++held_out) {
        float sx = 0.0f, sy = 0.0f, sx2 = 0.0f, sxy = 0.0f;
        for (uint8_t i = 0; i < input.count; ++i) {
          if (i == held_out) continue;
          const float x = input.x_deg[i], y = input.y_deg[i];
          sx += x; sy += y; sx2 += x * x; sxy += x * y;
        }
        const float train_n = static_cast<float>(input.count - 1);
        float r = 0.0f, c = 0.0f;
        if (affine) {
          const float denom = train_n * sx2 - sx * sx;
          if (fabsf(denom) < 1.0e-6f) return NAN;
          r = (train_n * sxy - sx * sy) / denom;
          c = (sy - r * sx) / train_n;
        } else {
          if (fabsf(sx2) < 1.0e-6f) return NAN;
          r = sxy / sx2;
        }
        const float error = r * input.x_deg[held_out] + c - input.y_deg[held_out];
        if (!isfinite(error)) return NAN;
        total_squared_error += error * error;
      }
      return sqrtf(total_squared_error / n);
    };

    model = 0; selected_r = 0.0f; selected_c = 0.0f;
    prop_r = 0.0f; prop_rmse = 0.0f; prop_r2 = 0.0f; prop_loocv = 0.0f; prop_valid = false;
    affine_r = 0.0f; affine_c = 0.0f; affine_rmse = 0.0f; affine_r2 = 0.0f; affine_loocv = 0.0f; affine_valid = false;
    if (n <= 0.0f) return false;

    if (fabsf(input.sum_x2) > 1.0e-6f) {
      prop_r = input.sum_xy / input.sum_x2;
      const float sse = input.sum_y2 - 2.0f * prop_r * input.sum_xy + prop_r * prop_r * input.sum_x2;
      prop_rmse = sqrtf(fmaxf(0.0f, sse / n));
      prop_r2 = sst > 1.0e-6f ? 1.0f - sse / sst : 0.0f;
      prop_loocv = loocvRmse(false);
      const float low = prop_r * input_min;
      const float high = prop_r * input_max;
      prop_valid = enough && usable_input && isfinite(prop_r) && isfinite(prop_rmse) && isfinite(prop_loocv) &&
                   prop_r > 0.0f && low > 0.0f && high > 0.0f;
    }

    const float denom = n * input.sum_x2 - input.sum_x * input.sum_x;
    const float variance = n > 0.0f ? denom / (n * n) : 0.0f;
    if (fabsf(denom) > 1.0e-6f && variance >= Config::ZERO_CROSS_CALIBRATION_MIN_X_VARIANCE_DEG2) {
      affine_r = (n * input.sum_xy - input.sum_x * input.sum_y) / denom;
      affine_c = (input.sum_y - affine_r * input.sum_x) / n;
      const float sse = input.sum_y2 - 2.0f * affine_r * input.sum_xy - 2.0f * affine_c * input.sum_y +
                        affine_r * affine_r * input.sum_x2 + 2.0f * affine_r * affine_c * input.sum_x + n * affine_c * affine_c;
      affine_rmse = sqrtf(fmaxf(0.0f, sse / n));
      affine_r2 = sst > 1.0e-6f ? 1.0f - sse / sst : 0.0f;
      affine_loocv = loocvRmse(true);
      const float low = affine_r * input_min + affine_c;
      const float high = affine_r * input_max + affine_c;
      affine_valid = enough && usable_input && isfinite(affine_r) && isfinite(affine_c) &&
                     isfinite(affine_rmse) && isfinite(affine_loocv) && affine_r > 0.0f &&
                     low > 0.0f && high > 0.0f;
    }

    if (affine_valid && (!prop_valid || affine_loocv < prop_loocv)) {
      model = 2; selected_r = affine_r; selected_c = affine_c;
    } else if (prop_valid) {
      model = 1; selected_r = prop_r; selected_c = 0.0f;
    }
    return model != 0;
  };

  auto evaluateFullCycle = [&](float first_r, float first_c, float first_input_min, float first_input_max,
                               float second_r, float second_c, float second_input_min, float second_input_max,
                               float& out_r, float& out_c, float& out_input_min, float& out_input_max,
                               bool& out_valid) {
    out_r = 0.0f; out_c = 0.0f; out_input_min = 0.0f; out_input_max = 0.0f; out_valid = false;
    if (!isfinite(first_r) || !isfinite(first_c) || !isfinite(second_r) || !isfinite(second_c) ||
        first_r <= 0.0f || second_r <= 0.0f || first_input_min <= 0.0f ||
        first_input_max < first_input_min || second_input_min <= 0.0f ||
        second_input_max < second_input_min) return false;
    const float composition_min = fmaxf(first_input_min, (second_input_min - first_c) / first_r);
    const float composition_max = fminf(first_input_max, (second_input_max - first_c) / first_r);
    const float r = second_r * first_r;
    const float c = second_r * first_c + second_c;
    const float low = r * composition_min + c;
    const float high = r * composition_max + c;
    out_r = r; out_c = c; out_input_min = composition_min; out_input_max = composition_max;
    out_valid = isfinite(composition_min) && isfinite(composition_max) && isfinite(r) && isfinite(c) &&
                composition_min > 0.0f && composition_max > composition_min && r > 0.0f &&
                isfinite(low) && isfinite(high) && low > 0.0f && high > 0.0f &&
                low < composition_min && high < composition_max;
    return out_valid;
  };

  calibration_result_.initial_kick_count = calibration_initial_kick_count_;
  calibration_result_.free_count_pos = calibration_fit_pos_.count;
  calibration_result_.free_count_neg = calibration_fit_neg_.count;
  calibration_result_.free_support_min_pos_deg = calibration_fit_pos_.support_min_deg;
  calibration_result_.free_support_max_pos_deg = calibration_fit_pos_.support_max_deg;
  calibration_result_.free_support_min_neg_deg = calibration_fit_neg_.support_min_deg;
  calibration_result_.free_support_max_neg_deg = calibration_fit_neg_.support_max_deg;
  calibration_result_.free_input_min_pos_deg = calibration_fit_pos_.input_min_deg;
  calibration_result_.free_input_max_pos_deg = calibration_fit_pos_.input_max_deg;
  calibration_result_.free_input_min_neg_deg = calibration_fit_neg_.input_min_deg;
  calibration_result_.free_input_max_neg_deg = calibration_fit_neg_.input_max_deg;
  const bool pos_ok = evaluate(calibration_fit_pos_, true);
  const bool neg_ok = evaluate(calibration_fit_neg_, false);
  calibration_result_.free_halfcycle_monotone_pos = pos_ok;
  calibration_result_.free_halfcycle_monotone_neg = neg_ok;
  const bool full_pos = pos_ok && neg_ok && evaluateFullCycle(
      calibration_result_.r_neg, calibration_result_.c_neg_deg,
      calibration_result_.free_input_min_neg_deg, calibration_result_.free_input_max_neg_deg,
      calibration_result_.r_pos, calibration_result_.c_pos_deg,
      calibration_result_.free_input_min_pos_deg, calibration_result_.free_input_max_pos_deg,
      calibration_result_.free_full_cycle_r_pos, calibration_result_.free_full_cycle_c_pos_deg,
      calibration_result_.free_full_cycle_input_min_pos_deg, calibration_result_.free_full_cycle_input_max_pos_deg,
      calibration_result_.free_full_cycle_valid_pos);
  const bool full_neg = pos_ok && neg_ok && evaluateFullCycle(
      calibration_result_.r_pos, calibration_result_.c_pos_deg,
      calibration_result_.free_input_min_pos_deg, calibration_result_.free_input_max_pos_deg,
      calibration_result_.r_neg, calibration_result_.c_neg_deg,
      calibration_result_.free_input_min_neg_deg, calibration_result_.free_input_max_neg_deg,
      calibration_result_.free_full_cycle_r_neg, calibration_result_.free_full_cycle_c_neg_deg,
      calibration_result_.free_full_cycle_input_min_neg_deg, calibration_result_.free_full_cycle_input_max_neg_deg,
      calibration_result_.free_full_cycle_valid_neg);
  return pos_ok && neg_ok && full_pos && full_neg;
}
void ExperimentRunner::finishCalibrationShadow(uint8_t failure_reason) {
  if (!calibration_enabled_) return;
  calibration_result_.initial_kick_count = calibration_initial_kick_count_;
  calibration_result_.rebuild_total_count = calibration_rebuild_total_count_;
  calibration_result_.rebuild_episode_id = calibration_rebuild_episode_id_;
  calibration_result_.rebuild_attempt_in_episode = calibration_rebuild_attempt_in_episode_;
  calibration_result_.free_count_pos = calibration_fit_pos_.count;
  calibration_result_.free_count_neg = calibration_fit_neg_.count;
  calibration_result_.failure_reason = failure_reason;
  calibration_result_.probe_plan_count = calibration_probe_plan_index_;
  calibration_result_.probe_wait_halfcycles = calibration_probe_wait_halfcycle_total_;
  calibration_result_.v59_gate_event_count = calibration_v59_gate_event_count_;
  calibration_result_.v59_gate_pass_count = calibration_v59_gate_pass_count_;
  calibration_result_.v59_gate_skip_count = calibration_v59_gate_skip_count_;
  calibration_result_.v59_rebuild_from_low_state_count = calibration_v59_rebuild_from_low_state_count_;
  calibration_result_.protocol_complete = failure_reason == 0 &&
      calibration_probe_plan_index_ == Config::ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_COUNT;
  // The V59 response set intentionally contains only +arrival events, so the
  // historical signed two-side gain summary remains invalid by design.
  calibration_result_.valid = calibration_result_.protocol_complete &&
                              calibration_result_.q_count_pos >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SAMPLES_PER_SIDE &&
                              calibration_result_.q_count_neg >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SAMPLES_PER_SIDE &&
                              isfinite(calibration_result_.g_pos) && isfinite(calibration_result_.g_neg) &&
                              calibration_result_.g_pos > Config::ZERO_CROSS_CALIBRATION_Q_PROBE_MIN_POSITIVE_GAIN_DEG_PER_MAS &&
                              calibration_result_.g_neg > Config::ZERO_CROSS_CALIBRATION_Q_PROBE_MIN_POSITIVE_GAIN_DEG_PER_MAS;
  calibration_phase_ = CalibrationPhase::MAIN_CONTROL;
  if (logger_) logger_->setCalibrationResult(calibration_result_);
  // V59 is a measurement protocol, not a handoff to the normal controller.
  // Preserve the completed/failed RWLOG and end its LED signature immediately.
  if (Config::ZERO_CROSS_V59_STATE_WAIT_FIXED_Q_ENABLED) finishRun();
}

float ExperimentRunner::calibratedPredictionDeg(float q_effective_pred_mA_s, int8_t next_peak_side) const {
  if (!calibration_result_.valid || !calibration_last_peak_valid_) return NAN;
  const bool positive = next_peak_side >= 0;
  const float support_min = positive ? calibration_result_.free_input_min_pos_deg : calibration_result_.free_input_min_neg_deg;
  const float support_max = positive ? calibration_result_.free_input_max_pos_deg : calibration_result_.free_input_max_neg_deg;
  // The selected free-decay fit is identification data, not an extrapolating
  // controller. Outside its observed amplitude support, do not emit a shadow prediction.
  if (calibration_last_peak_abs_deg_ < support_min || calibration_last_peak_abs_deg_ > support_max) return NAN;
  const float r = positive ? calibration_result_.r_pos : calibration_result_.r_neg;
  const float g = positive ? calibration_result_.g_pos : calibration_result_.g_neg;
  const float c = positive ? calibration_result_.c_pos_deg : calibration_result_.c_neg_deg;
  return r * calibration_last_peak_abs_deg_ + g * q_effective_pred_mA_s + c;
}

void ExperimentRunner::updateCalibrationPeak(uint32_t now_ms, uint32_t t_test_ms, float angle_deg, float rate_dps) {
  (void)now_ms;
  if (!calibration_enabled_) return;
  if (!calibration_peak_tracker_ready_) {
    // The first candidate can be armed from either moving side. After every
    // confirmed peak, however, only the opposite-side *outbound* leg may arm
    // the next candidate. This prevents the confirmation sample from being
    // reused as a stale same-side extremum.
    if (calibration_expected_candidate_side_ != 0) {
      const float expected = static_cast<float>(calibration_expected_candidate_side_);
      if (expected * angle_deg <= 0.0f ||
          expected * rate_dps < Config::ZERO_CROSS_MIN_RATE_DPS) return;
    } else if (fabsf(rate_dps) < Config::ZERO_CROSS_MIN_RATE_DPS) {
      return;
    }
    calibration_peak_tracker_ready_ = true;
    calibration_outbound_rate_sign_ = rate_dps >= 0.0f ? 1 : -1;
    calibration_peak_candidate_deg_ = angle_deg;
    calibration_peak_candidate_fixed_deg_ = status_.pitch_dynamic_beta_deg[Config::FILTER_FIXED_B100_INDEX];
    calibration_peak_candidate_ms_ = t_test_ms;
    calibration_expected_candidate_side_ = 0;
    return;
  }

  // Once the rate has reversed, freeze the extremum candidate.  It remains
  // frozen through the confirmation window so the following half-cycle cannot
  // overwrite either its value, sign, or physical occurrence time.
  const bool reversing = rate_dps * calibration_outbound_rate_sign_ <=
                         -Config::ZERO_CROSS_MIN_RATE_DPS;
  if (!reversing) {
    calibration_reverse_samples_ = 0;
    if (fabsf(angle_deg) > fabsf(calibration_peak_candidate_deg_)) {
      calibration_peak_candidate_deg_ = angle_deg;
      calibration_peak_candidate_fixed_deg_ = status_.pitch_dynamic_beta_deg[Config::FILTER_FIXED_B100_INDEX];
      calibration_peak_candidate_ms_ = t_test_ms;
    }
    return;
  }
  if (++calibration_reverse_samples_ < Config::ZERO_CROSS_IDENTIFICATION_PEAK_CONFIRM_SAMPLES) return;

  calibration_reverse_samples_ = 0;
  const float peak_signed_dynamic = calibration_peak_candidate_deg_;
  const float peak_signed_fixed = calibration_peak_candidate_fixed_deg_;
  const float peak_abs = fabsf(peak_signed_dynamic);
  const int8_t side = peak_signed_dynamic >= 0.0f ? 1 : -1;
  const uint32_t candidate_peak_ms = calibration_peak_candidate_ms_;
  const uint32_t confirmed_ms = t_test_ms;
  // Do not reuse the confirmation-time sample as a candidate for the next
  // turn. It still lies on this peak's side and can be larger than the next
  // opposite-side extremum. Clear it, then arm only after a genuine opposite
  // outbound leg is observed.
  calibration_peak_tracker_ready_ = false;
  calibration_expected_candidate_side_ = -side;
  calibration_outbound_rate_sign_ = 0;
  calibration_peak_candidate_deg_ = 0.0f;
  calibration_peak_candidate_fixed_deg_ = 0.0f;
  calibration_peak_candidate_ms_ = 0;

  auto isConsecutive = [&]() {
    return calibration_last_peak_valid_ && candidate_peak_ms > calibration_last_peak_candidate_ms_ &&
           candidate_peak_ms - calibration_last_peak_candidate_ms_ <=
               Config::ZERO_CROSS_CALIBRATION_MAX_CONSECUTIVE_PEAK_GAP_MS;
  };
  auto setLastPeak = [&]() {
    if (calibration_last_peak_valid_) {
      calibration_last_half_range_dynamic_deg_ = 0.5f * fabsf(
          peak_signed_dynamic - calibration_last_peak_signed_dynamic_deg_);
      calibration_last_half_range_fixed_deg_ = 0.5f * fabsf(
          peak_signed_fixed - calibration_last_peak_signed_fixed_deg_);
      calibration_last_half_range_valid_ = true;
      // This is the center of the two already confirmed peaks. It is the
      // online C_prev audit state, distinct from the post-pulse event center.
      calibration_last_center_dynamic_deg_ = 0.5f * (
          peak_signed_dynamic + calibration_last_peak_signed_dynamic_deg_);
      calibration_last_center_dynamic_valid_ = true;
    } else {
      calibration_last_half_range_valid_ = false;
      calibration_last_center_dynamic_valid_ = false;
    }
    calibration_last_peak_valid_ = true;
    calibration_last_peak_side_ = side;
    calibration_last_peak_abs_deg_ = peak_abs;
    calibration_last_peak_signed_dynamic_deg_ = peak_signed_dynamic;
    calibration_last_peak_signed_fixed_deg_ = peak_signed_fixed;
    calibration_last_peak_candidate_ms_ = candidate_peak_ms;
  };
  auto addProbeEvent = [&](bool post_pulse, bool in_support, bool side_matched,
                            bool positive_gain, bool gain_stored, uint8_t result_code, float gain) {
    if (!logger_) return;
    PsramLogger::CalibrationProbeEvent event;
    event.planned_probe_index = calibration_probe_active_plan_index_;
    event.q_level_index = calibration_probe_q_level_index_;
    event.q_probe_schedule_id = q_probe_schedule_id_;
    event.polarity_mode = 0;  // BASE: direction = -requested_side.
    event.rebuild_count = calibration_probe_rebuild_total_count_;
    event.rebuild_episode_id = calibration_probe_rebuild_episode_id_;
    event.rebuild_attempt_in_episode = calibration_probe_rebuild_attempt_in_episode_;
    event.rebuild_entry_source = calibration_probe_rebuild_entry_source_;
    event.wait_halfcycle_count = calibration_probe_wait_halfcycle_count_;
    event.result_code = result_code;
    event.pulse_start_ms = calibration_probe_pulse_start_test_ms_;
    event.candidate_peak_ms = candidate_peak_ms;
    event.confirmed_ms = confirmed_ms;
    event.previous_peak_candidate_ms = calibration_probe_previous_peak_candidate_ms_;
    event.requested_side = calibration_probe_requested_side_;
    event.predecessor_side = calibration_probe_predecessor_side_;
    event.observed_side = side;
    event.pulse_direction = calibration_probe_direction_;
    event.pulse_id = calibration_probe_pulse_id_;
    event.pulse_width_ms = status_.pulse_width_ms_setting;
    event.command_dynamic_angle_cdeg = centi(calibration_probe_command_dynamic_angle_deg_);
    event.command_rate_raw_cdps = centi(calibration_probe_command_rate_raw_dps_);
    event.command_rate_bias_corrected_cdps = centi(calibration_probe_command_rate_bias_corrected_dps_);
    event.previous_peak_abs_cdeg = centi(calibration_probe_previous_peak_abs_deg_);
    event.observed_peak_abs_cdeg = centi(peak_abs);
    event.previous_peak_signed_dynamic_cdeg = centi(calibration_probe_previous_peak_signed_dynamic_deg_);
    event.observed_peak_signed_dynamic_cdeg = centi(peak_signed_dynamic);
    event.center_dynamic_cdeg = centi(0.5f * (calibration_probe_previous_peak_signed_dynamic_deg_ +
                                              peak_signed_dynamic));
    const float observed_half_range_dynamic = 0.5f * fabsf(
        peak_signed_dynamic - calibration_probe_previous_peak_signed_dynamic_deg_);
    event.half_range_previous_dynamic_cdeg = calibration_probe_previous_half_range_valid_ ?
        centi(calibration_probe_previous_half_range_dynamic_deg_) : LOG_NAN_I16;
    event.half_range_observed_dynamic_cdeg = centi(observed_half_range_dynamic);
    bool dynamic_half_range_in_support = false;
    const float predicted_half_range_dynamic = post_pulse && calibration_probe_previous_half_range_valid_ ?
        halfRangeShadowPrediction(calibration_probe_previous_half_range_dynamic_deg_, true,
                                  &dynamic_half_range_in_support) : NAN;
    const float delta_half_range_dynamic = isfinite(predicted_half_range_dynamic) ?
        observed_half_range_dynamic - predicted_half_range_dynamic : NAN;
    const float gain_half_range_dynamic = isfinite(delta_half_range_dynamic) &&
        calibration_probe_q_effective_pred_mA_s_ > 0.0f ?
        delta_half_range_dynamic / calibration_probe_q_effective_pred_mA_s_ : NAN;
    event.half_range_free_pred_dynamic_cdeg = isfinite(predicted_half_range_dynamic) ?
        centi(predicted_half_range_dynamic) : LOG_NAN_I16;
    event.delta_half_range_dynamic_cdeg = isfinite(delta_half_range_dynamic) ?
        centi(delta_half_range_dynamic) : LOG_NAN_I16;
    event.gain_half_range_dynamic_cdeg_per_mAs = isfinite(gain_half_range_dynamic) ?
        centi(gain_half_range_dynamic) : LOG_NAN_I16;
    event.previous_peak_signed_fixed_cdeg = centi(calibration_probe_previous_peak_signed_fixed_deg_);
    event.observed_peak_signed_fixed_cdeg = centi(peak_signed_fixed);
    event.center_fixed_cdeg = centi(0.5f * (calibration_probe_previous_peak_signed_fixed_deg_ + peak_signed_fixed));
    const float observed_half_range_fixed = 0.5f * fabsf(
        peak_signed_fixed - calibration_probe_previous_peak_signed_fixed_deg_);
    event.half_range_previous_fixed_cdeg = calibration_probe_previous_half_range_valid_ ?
        centi(calibration_probe_previous_half_range_fixed_deg_) : LOG_NAN_I16;
    event.half_range_observed_fixed_cdeg = centi(observed_half_range_fixed);
    bool fixed_half_range_in_support = false;
    const float predicted_half_range_fixed = post_pulse && calibration_probe_previous_half_range_valid_ ?
        halfRangeShadowPrediction(calibration_probe_previous_half_range_fixed_deg_, false,
                                  &fixed_half_range_in_support) : NAN;
    const float delta_half_range_fixed = isfinite(predicted_half_range_fixed) ?
        observed_half_range_fixed - predicted_half_range_fixed : NAN;
    const float gain_half_range_fixed = isfinite(delta_half_range_fixed) &&
        calibration_probe_q_effective_pred_mA_s_ > 0.0f ?
        delta_half_range_fixed / calibration_probe_q_effective_pred_mA_s_ : NAN;
    event.half_range_free_pred_fixed_cdeg = isfinite(predicted_half_range_fixed) ?
        centi(predicted_half_range_fixed) : LOG_NAN_I16;
    event.delta_half_range_fixed_cdeg = isfinite(delta_half_range_fixed) ?
        centi(delta_half_range_fixed) : LOG_NAN_I16;
    event.gain_half_range_fixed_cdeg_per_mAs = isfinite(gain_half_range_fixed) ?
        centi(gain_half_range_fixed) : LOG_NAN_I16;
    event.half_range_dynamic_in_support = dynamic_half_range_in_support;
    event.half_range_fixed_in_support = fixed_half_range_in_support;
    event.rebuild_target_reached = calibration_probe_rebuild_target_reached_;
    event.dynamic_h_in_support_at_command = calibration_probe_dynamic_h_in_support_at_command_;
    event.q_target_mAms = lroundf(calibration_probe_q_target_mA_s_ * 1000.0f);
    event.q_effective_pred_mAms = lroundf(calibration_probe_q_effective_pred_mA_s_ * 1000.0f);
    event.gain_cdeg_per_mAs = isfinite(gain) ? centi(gain) : LOG_NAN_I16;

    // v53 remains disconnected from every Q selector and motor path. It keeps
    // the v52 forward model fixed, then records inverse-shadow diagnostics only.
    const float shadow_hprev = calibration_probe_previous_half_range_dynamic_deg_;
    const float shadow_c_prev = calibration_probe_previous_center_dynamic_deg_;
    const float shadow_abs_rate = fabsf(calibration_probe_command_rate_bias_corrected_dps_);
    const int8_t shadow_next_peak_side = calibration_probe_requested_side_;
    const float shadow_q_effective = calibration_probe_q_effective_pred_mA_s_;
    const bool shadow_state_valid = Config::ZERO_CROSS_V52_SHADOW_ENABLED &&
        Config::ZERO_CROSS_V53_INVERSE_SHADOW_ENABLED &&
        calibration_probe_previous_half_range_valid_ &&
        calibration_probe_previous_center_dynamic_valid_ &&
        isfinite(shadow_hprev) && isfinite(shadow_c_prev) && isfinite(shadow_abs_rate) &&
        (shadow_next_peak_side == -1 || shadow_next_peak_side == 1);
    const bool shadow_h_in_support = shadow_state_valid &&
        shadow_hprev >= Config::ZERO_CROSS_V53_STATE_HPREV_MIN_DEG &&
        shadow_hprev <= Config::ZERO_CROSS_V53_STATE_HPREV_MAX_DEG;
    const bool shadow_rate_in_support = shadow_state_valid &&
        shadow_abs_rate >= Config::ZERO_CROSS_V53_STATE_ABS_RATE_MIN_DPS &&
        shadow_abs_rate <= Config::ZERO_CROSS_V53_STATE_ABS_RATE_MAX_DPS;
    const bool shadow_state_in_support = shadow_h_in_support && shadow_rate_in_support;
    const bool shadow_q_in_model_support = isfinite(shadow_q_effective) &&
        shadow_q_effective >= Config::ZERO_CROSS_V52_SHADOW_Q_SUPPORT_MIN_MAS &&
        shadow_q_effective <= Config::ZERO_CROSS_V52_SHADOW_Q_SUPPORT_MAX_MAS;
    event.shadow_hprev_imu_cdeg = shadow_state_valid ? centi(shadow_hprev) : LOG_NAN_I16;
    event.shadow_c_prev_imu_cdeg = shadow_state_valid ? centi(shadow_c_prev) : LOG_NAN_I16;
    event.shadow_rate_abs_cdps = shadow_state_valid ? centi(shadow_abs_rate) : LOG_NAN_I16;
    event.shadow_next_peak_side = shadow_next_peak_side;
    event.shadow_q_effective_pred_mA_s = shadow_q_effective;
    event.shadow_state_valid = shadow_state_valid;
    event.shadow_h_in_support = shadow_h_in_support;
    event.shadow_rate_in_support = shadow_rate_in_support;
    event.shadow_state_in_support = shadow_state_in_support;
    event.shadow_q_in_model_support = shadow_q_in_model_support;
    // v53 never becomes a command source, including inside the support range.
    event.shadow_control_candidate = false;

    const float shadow_base = shadow_state_valid
        ? Config::ZERO_CROSS_V52_SHADOW_INTERCEPT_DEG +
              Config::ZERO_CROSS_V52_SHADOW_HPREV_GAIN * shadow_hprev +
              Config::ZERO_CROSS_V52_SHADOW_ABS_RATE_GAIN_DEG_PER_DPS * shadow_abs_rate +
              Config::ZERO_CROSS_V52_SHADOW_NEXT_PEAK_SIDE_OFFSET_DEG * shadow_next_peak_side
        : NAN;
    const bool shadow_forward_inputs_valid = isfinite(shadow_base) &&
        Config::ZERO_CROSS_V52_SHADOW_Q_GAIN_DEG_PER_MAS > 0.0f;
    const bool shadow_delta_h_valid = shadow_forward_inputs_valid && shadow_q_in_model_support &&
        post_pulse && side_matched;
    const float shadow_delta_h = shadow_delta_h_valid
        ? shadow_base + Config::ZERO_CROSS_V52_SHADOW_Q_GAIN_DEG_PER_MAS * shadow_q_effective
        : NAN;
    event.shadow_delta_h_video_pred_deg = shadow_delta_h;
    event.shadow_delta_h_valid = isfinite(shadow_delta_h);

    // This is the on-device dynamic-H proxy. Video validation remains offline.
    event.shadow_delta_h_imu_actual_deg = isfinite(delta_half_range_dynamic)
        ? delta_half_range_dynamic
        : NAN;
    event.shadow_imu_proxy_residual_valid = event.shadow_delta_h_valid &&
        isfinite(event.shadow_delta_h_imu_actual_deg);
    event.shadow_delta_h_pred_minus_imu_dynamic_deg = event.shadow_imu_proxy_residual_valid
        ? shadow_delta_h - event.shadow_delta_h_imu_actual_deg
        : NAN;

    bool shadow_h_free_imu_in_support = false;
    const float shadow_h_free_imu = shadow_state_valid
        ? halfRangeShadowPrediction(shadow_hprev, true, &shadow_h_free_imu_in_support) : NAN;
    event.shadow_h_free_imu_pred_deg = shadow_h_free_imu;
    event.shadow_h_free_valid = isfinite(shadow_h_free_imu) && shadow_h_free_imu_in_support;
    const float shadow_h_free_video = event.shadow_h_free_valid
        ? Config::ZERO_CROSS_V53_H_FREE_VIDEO_INTERCEPT_DEG +
              Config::ZERO_CROSS_V53_H_FREE_VIDEO_FROM_IMU_GAIN * shadow_h_free_imu
        : NAN;
    event.shadow_h_free_video_pred_deg = shadow_h_free_video;
    event.shadow_h_free_video_valid = isfinite(shadow_h_free_video);

    // Candidate predictions are retained even when the v51 state support is
    // false. They are diagnostics, not commands. Q=0 is explicitly extrapolated.
    const bool shadow_h_post_candidates_valid = shadow_forward_inputs_valid &&
        event.shadow_h_free_video_valid;
    const float shadow_h_post_q0 = shadow_h_post_candidates_valid
        ? shadow_h_free_video + shadow_base + Config::ZERO_CROSS_V52_SHADOW_Q_GAIN_DEG_PER_MAS *
              Config::ZERO_CROSS_V53_Q_CANDIDATE_MAS[0] : NAN;
    const float shadow_h_post_q05 = shadow_h_post_candidates_valid
        ? shadow_h_free_video + shadow_base + Config::ZERO_CROSS_V52_SHADOW_Q_GAIN_DEG_PER_MAS *
              Config::ZERO_CROSS_V53_Q_CANDIDATE_MAS[1] : NAN;
    const float shadow_h_post_q10 = shadow_h_post_candidates_valid
        ? shadow_h_free_video + shadow_base + Config::ZERO_CROSS_V52_SHADOW_Q_GAIN_DEG_PER_MAS *
              Config::ZERO_CROSS_V53_Q_CANDIDATE_MAS[2] : NAN;
    const float shadow_h_post_q15 = shadow_h_post_candidates_valid
        ? shadow_h_free_video + shadow_base + Config::ZERO_CROSS_V52_SHADOW_Q_GAIN_DEG_PER_MAS *
              Config::ZERO_CROSS_V53_Q_CANDIDATE_MAS[3] : NAN;
    event.shadow_h_post_pred_q0_deg = shadow_h_post_q0;
    event.shadow_h_post_pred_q05_deg = shadow_h_post_q05;
    event.shadow_h_post_pred_q10_deg = shadow_h_post_q10;
    event.shadow_h_post_pred_q15_deg = shadow_h_post_q15;
    event.shadow_h_post_candidates_valid = shadow_h_post_candidates_valid;
    event.shadow_q0_extrapolated = shadow_h_post_candidates_valid;
    // These bounds are the current discrete control-candidate endpoints, not
    // the wider forward-model measurement support. They are recorded even in
    // this log-only build so the target-reachability policy can be audited.
    event.shadow_h_supported_min_pred_deg = shadow_h_post_q05;
    event.shadow_h_supported_max_pred_deg = shadow_h_post_q15;
    const float shadow_h_post_actual_q = event.shadow_h_free_video_valid &&
        event.shadow_delta_h_valid ? shadow_h_free_video + shadow_delta_h : NAN;
    event.shadow_h_post_pred_deg = shadow_h_post_actual_q;
    event.shadow_h_post_valid = isfinite(shadow_h_post_actual_q);

    event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_REFERENCE_UNCONFIGURED;
    event.shadow_q_req_region = Config::ZERO_CROSS_V53_Q_REQ_REGION_UNAVAILABLE;
    event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_UNAVAILABLE;
    event.shadow_recommended_action = Config::ZERO_CROSS_V54_SHADOW_ACTION_UNAVAILABLE;
    event.shadow_q_support_reason = Config::ZERO_CROSS_V54_Q_SUPPORT_UNAVAILABLE;
    if (Config::ZERO_CROSS_V53_H_REF_CONFIGURED) {
      event.shadow_h_ref_deg = Config::ZERO_CROSS_V53_H_REF_VIDEO_DEG;
      if (!shadow_forward_inputs_valid) {
        event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_STATE_INVALID;
      } else if (!event.shadow_h_free_video_valid) {
        event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_H_FREE_INVALID;
      } else {
        const float delta_h_required = event.shadow_h_ref_deg - shadow_h_free_video;
        const float q_raw = (delta_h_required - shadow_base) /
            Config::ZERO_CROSS_V52_SHADOW_Q_GAIN_DEG_PER_MAS;
        event.shadow_q_req_raw_mA_s = q_raw;
        event.shadow_q_req_clamped_mA_s = isfinite(q_raw)
            ? fminf(Config::ZERO_CROSS_V53_Q_REQUEST_MAX_MAS, fmaxf(0.0f, q_raw))
            : NAN;
        if (!isfinite(q_raw)) {
          event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_STATE_INVALID;
        } else if (q_raw < 0.0f) {
          event.shadow_q_req_region = Config::ZERO_CROSS_V53_Q_REQ_REGION_BELOW_ZERO;
          event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_BELOW_ZERO;
        } else if (q_raw > Config::ZERO_CROSS_V53_Q_REQUEST_MAX_MAS) {
          event.shadow_q_req_region = Config::ZERO_CROSS_V53_Q_REQ_REGION_ABOVE_QMAX;
          event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_ABOVE_QMAX;
        } else {
          // Region remains 0--1.5 mA*s even below the fitted Q minimum. The
          // reason and valid flag separately expose that lower-support limit.
          event.shadow_q_req_region = Config::ZERO_CROSS_V53_Q_REQ_REGION_WITHIN_ZERO_TO_QMAX;
          if (q_raw < Config::ZERO_CROSS_V52_SHADOW_Q_SUPPORT_MIN_MAS) {
            event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_OUTSIDE_MODEL_SUPPORT;
          } else if (shadow_state_in_support) {
            event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_WITHIN_RANGE;
            event.shadow_q_req_valid = true;
            event.shadow_future_control_eligible = true;
          } else {
            event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_STATE_OUT_OF_SUPPORT;
          }
        }

        // v54 reports Q support independently of the state support. This
        // makes the old v53 `outside_model_support` label unambiguous without
        // rewriting its historical code or meaning.
        if (isfinite(q_raw)) {
          if (q_raw < 0.0f) {
            event.shadow_q_support_reason = Config::ZERO_CROSS_V54_Q_SUPPORT_NEGATIVE;
          } else if (q_raw < Config::ZERO_CROSS_V52_SHADOW_Q_SUPPORT_MIN_MAS) {
            event.shadow_q_support_reason = Config::ZERO_CROSS_V54_Q_SUPPORT_BELOW_FORWARD_MODEL;
          } else if (q_raw <= Config::ZERO_CROSS_V54_CONTROL_CANDIDATE_Q_MAX_MAS) {
            event.shadow_q_support_reason = Config::ZERO_CROSS_V54_Q_SUPPORT_WITHIN_CURRENT_CANDIDATES;
          } else if (q_raw <= Config::ZERO_CROSS_V52_SHADOW_Q_SUPPORT_MAX_MAS) {
            event.shadow_q_support_reason = Config::ZERO_CROSS_V54_Q_SUPPORT_ABOVE_CANDIDATES_WITHIN_FORWARD_MODEL;
          } else {
            event.shadow_q_support_reason = Config::ZERO_CROSS_V54_Q_SUPPORT_ABOVE_FORWARD_MODEL;
          }
        }
      }
    }

    // v54 target-reachability policy. It is deliberately evaluated after the
    // diagnostics above and remains isolated from every motor-command path.
    if (!Config::ZERO_CROSS_V53_H_REF_CONFIGURED) {
      event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_REFERENCE_UNCONFIGURED;
    } else if (!shadow_state_valid) {
      event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_STATE_INVALID;
      event.shadow_recommended_action = Config::ZERO_CROSS_V54_SHADOW_ACTION_NO_PULSE;
    } else if (!shadow_state_in_support) {
      event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_STATE_OUT_OF_SUPPORT;
      event.shadow_recommended_action = Config::ZERO_CROSS_V54_SHADOW_ACTION_NO_PULSE;
    } else if (!event.shadow_h_free_video_valid) {
      event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_H_FREE_INVALID;
      event.shadow_recommended_action = Config::ZERO_CROSS_V54_SHADOW_ACTION_NO_PULSE;
    } else if (!shadow_h_post_candidates_valid || !isfinite(event.shadow_h_supported_min_pred_deg) ||
               !isfinite(event.shadow_h_supported_max_pred_deg)) {
      event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_CANDIDATES_INVALID;
      event.shadow_recommended_action = Config::ZERO_CROSS_V54_SHADOW_ACTION_NO_PULSE;
    } else {
      event.shadow_reachability_valid = true;
      const float h_ref = event.shadow_h_ref_deg;
      const float h_min = event.shadow_h_supported_min_pred_deg;
      const float h_max = event.shadow_h_supported_max_pred_deg;
      if (h_ref < h_min) {
        event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_TARGET_BELOW_RANGE;
        event.shadow_recommended_action = Config::ZERO_CROSS_V54_SHADOW_ACTION_NO_PULSE;
        event.shadow_recommended_q_mA_s = 0.0f;
      } else if (h_ref > h_max) {
        event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_TARGET_ABOVE_RANGE;
        event.shadow_recommended_action = Config::ZERO_CROSS_V54_SHADOW_ACTION_SATURATE_QMAX;
        event.shadow_recommended_q_mA_s = Config::ZERO_CROSS_V54_CONTROL_CANDIDATE_Q_MAX_MAS;
      } else {
        event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_TARGET_WITHIN_RANGE;
        event.shadow_recommended_action = Config::ZERO_CROSS_V54_SHADOW_ACTION_INVERSE_Q;
        event.shadow_recommended_q_mA_s = event.shadow_q_req_raw_mA_s;
        event.shadow_href_in_current_control_candidate_reachable_range = true;
      }
    }

    const bool shadow_q_req_finite = isfinite(event.shadow_q_req_raw_mA_s);
    event.shadow_q_req_in_model_support = shadow_q_req_finite &&
        event.shadow_q_req_raw_mA_s >= Config::ZERO_CROSS_V52_SHADOW_Q_SUPPORT_MIN_MAS &&
        event.shadow_q_req_raw_mA_s <= Config::ZERO_CROSS_V52_SHADOW_Q_SUPPORT_MAX_MAS;
    event.shadow_q_req_in_control_candidate_range = shadow_q_req_finite &&
        event.shadow_q_req_raw_mA_s >= Config::ZERO_CROSS_V54_CONTROL_CANDIDATE_Q_MIN_MAS &&
        event.shadow_q_req_raw_mA_s <= Config::ZERO_CROSS_V54_CONTROL_CANDIDATE_Q_MAX_MAS;

    event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_UNAVAILABLE;
    if (!Config::ZERO_CROSS_V57_SUPPORT_AWARE_INVERSE_SHADOW_ENABLED) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_UNAVAILABLE;
    } else if (!Config::ZERO_CROSS_V53_H_REF_CONFIGURED) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_REFERENCE_UNCONFIGURED;
    } else if (!shadow_state_valid) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_STATE_INVALID;
    } else if (!shadow_state_in_support) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_STATE_OUT_OF_SUPPORT;
    } else if (!event.shadow_h_free_video_valid) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_H_FREE_INVALID;
    } else if (!shadow_q_req_finite || !event.shadow_reachability_valid) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_REFERENCE_UNREACHABLE;
    } else if (event.shadow_q_req_raw_mA_s < Config::ZERO_CROSS_V54_CONTROL_CANDIDATE_Q_MIN_MAS) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_Q_BELOW_CANDIDATE_RANGE;
    } else if (event.shadow_q_req_raw_mA_s > Config::ZERO_CROSS_V54_CONTROL_CANDIDATE_Q_MAX_MAS) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_Q_ABOVE_CANDIDATE_RANGE;
    } else if (event.shadow_reachability_reason != Config::ZERO_CROSS_V54_REACHABILITY_TARGET_WITHIN_RANGE) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_REFERENCE_UNREACHABLE;
    } else {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_VALID;
    }
    event.shadow_q_req_candidate_valid =
        event.shadow_v57_q_req_reason == Config::ZERO_CROSS_V57_Q_REQ_REASON_VALID;

    // v58: retain every v57 audit record, and separately label the narrowly
    // targeted state neighborhood for inverse-Q repeatability analysis. This
    // is intentionally evaluated after every command decision and cannot feed
    // any physical control path.
    event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_UNAVAILABLE;
    if (!Config::ZERO_CROSS_V58_REPEATABILITY_STATE_GATE_ENABLED) {
      event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_DISABLED;
    } else if (!shadow_state_valid) {
      event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_STATE_INVALID;
    } else if (shadow_hprev < Config::ZERO_CROSS_V58_GATE_HPREV_MIN_DEG) {
      event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_HPREV_BELOW;
    } else if (shadow_hprev > Config::ZERO_CROSS_V58_GATE_HPREV_MAX_DEG) {
      event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_HPREV_ABOVE;
    } else if (shadow_abs_rate < Config::ZERO_CROSS_V58_GATE_ABS_RATE_MIN_DPS) {
      event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_RATE_BELOW;
    } else if (shadow_abs_rate > Config::ZERO_CROSS_V58_GATE_ABS_RATE_MAX_DPS) {
      event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_RATE_ABOVE;
    } else if (shadow_next_peak_side != Config::ZERO_CROSS_V58_GATE_NEXT_PEAK_SIDE) {
      event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_DIRECTION_MISMATCH;
    } else {
      event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_PASSED;
    }
    event.shadow_v58_repeatability_gate_passed =
        event.shadow_v58_repeatability_gate_reason == Config::ZERO_CROSS_V58_GATE_REASON_PASSED;
    event.v59_block_index = calibration_probe_active_plan_index_ == 0 ? 0 :
        static_cast<uint8_t>((calibration_probe_active_plan_index_ - 1) / 3 + 1);
    event.v59_block_order = calibration_probe_active_plan_index_ == 0 ? 0 :
        static_cast<uint8_t>((calibration_probe_active_plan_index_ - 1) % 3 + 1);
    // Reaching Q_CAL is possible only after this online state gate passed.
    event.v59_state_gate_reason = Config::ZERO_CROSS_V59_GATE_REASON_PASSED;
    event.v59_state_gate_passed = Config::ZERO_CROSS_V59_STATE_WAIT_FIXED_Q_ENABLED;

    event.post_pulse = post_pulse;
    event.in_support = in_support;
    event.side_matched = side_matched;
    event.direction_matches_base = calibration_probe_direction_ == -calibration_probe_requested_side_;
    event.positive_gain = positive_gain;
    event.gain_stored = gain_stored;
    logger_->addCalibrationProbeEvent(event);
  };

  if (calibration_phase_ == CalibrationPhase::INITIAL_EXCITE) {
    recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 0, -1.0f,
                          calibration_initial_kick_q_effective_pred_mA_s_);
    if (peak_abs >= Config::ZERO_CROSS_CALIBRATION_INITIAL_ABORT_DEG) {
      finishCalibrationShadow(1);
    } else if (peak_abs >= Config::ZERO_CROSS_CALIBRATION_INITIAL_MIN_DEG) {
      setLastPeak();
      calibration_phase_ = CalibrationPhase::FREE_DECAY;
      calibration_result_.initial_kick_count = calibration_initial_kick_count_;
      if (logger_) logger_->setCalibrationResult(calibration_result_);
    } else if (calibration_initial_kick_count_ >= Config::ZERO_CROSS_CALIBRATION_INITIAL_MAX_KICKS) {
      finishCalibrationShadow(1);
    }
    return;
  }

  if (calibrationIsMainControl()) {
    setLastPeak();
    return;
  }
  if (peak_abs < Config::ZERO_CROSS_CALIBRATION_MIN_PEAK_DEG) {
    finishCalibrationShadow(1);
    return;
  }

  if (calibration_phase_ == CalibrationPhase::FREE_DECAY) {
    if (!isConsecutive() || side == calibration_last_peak_side_) {
      recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 4,
                            calibration_last_peak_abs_deg_, 0.0f);
      // A missing or non-alternating turn must never be bridged by a later one.
      finishCalibrationShadow(6);
      return;
    }
    CalibrationFit& fit = side > 0 ? calibration_fit_pos_ : calibration_fit_neg_;
    if (fit.count >= Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_MAX_PER_SIDE) {
      // The paired arrival side has already used the bounded support-wait
      // allowance. Do not overwrite a retained sample or extrapolate V60.
      finishCalibrationShadow(9);
      return;
    }
    if (fit.count == 0) {
      fit.support_min_deg = fminf(calibration_last_peak_abs_deg_, peak_abs);
      fit.support_max_deg = fmaxf(calibration_last_peak_abs_deg_, peak_abs);
      fit.input_min_deg = calibration_last_peak_abs_deg_;
      fit.input_max_deg = calibration_last_peak_abs_deg_;
    } else {
      fit.support_min_deg = fminf(fit.support_min_deg, fminf(calibration_last_peak_abs_deg_, peak_abs));
      fit.support_max_deg = fmaxf(fit.support_max_deg, fmaxf(calibration_last_peak_abs_deg_, peak_abs));
      fit.input_min_deg = fminf(fit.input_min_deg, calibration_last_peak_abs_deg_);
      fit.input_max_deg = fmaxf(fit.input_max_deg, calibration_last_peak_abs_deg_);
    }
    fit.x_deg[fit.count] = calibration_last_peak_abs_deg_;
    fit.y_deg[fit.count] = peak_abs;
    fit.count++; fit.sum_x += calibration_last_peak_abs_deg_; fit.sum_y += peak_abs;
    fit.sum_x2 += calibration_last_peak_abs_deg_ * calibration_last_peak_abs_deg_;
    fit.sum_y2 += peak_abs * peak_abs;
    fit.sum_xy += calibration_last_peak_abs_deg_ * peak_abs;
    recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 1,
                          calibration_last_peak_abs_deg_, 0.0f);
    const float current_half_range_dynamic = 0.5f * fabsf(
        peak_signed_dynamic - calibration_last_peak_signed_dynamic_deg_);
    const float current_half_range_fixed = 0.5f * fabsf(
        peak_signed_fixed - calibration_last_peak_signed_fixed_deg_);
    if (calibration_free_last_half_range_valid_) {
      addHalfRangeFreePair(calibration_half_range_dynamic_fit_,
                           calibration_free_last_half_range_dynamic_deg_, current_half_range_dynamic);
      addHalfRangeFreePair(calibration_half_range_fixed_fit_,
                           calibration_free_last_half_range_fixed_deg_, current_half_range_fixed);
    }
    calibration_free_last_half_range_dynamic_deg_ = current_half_range_dynamic;
    calibration_free_last_half_range_fixed_deg_ = current_half_range_fixed;
    calibration_free_last_half_range_valid_ = true;
    setLastPeak();
    if (calibration_fit_pos_.count >= Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_PER_SIDE &&
        calibration_fit_neg_.count >= Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_PER_SIDE) {
      if (!fitCalibrationFreeDecay()) {
        finishCalibrationShadow(2);
      } else {
        calibration_result_.half_range_free_count = calibration_half_range_dynamic_fit_.count;
        const bool dynamic_half_range_valid = fitHalfRangeShadow(calibration_half_range_dynamic_fit_, true);
        fitHalfRangeShadow(calibration_half_range_fixed_fit_, false);
        if (dynamic_half_range_valid) {
          calibration_result_.v62_hc_target_feasible = v62HasFeasibleHCTarget();
          if (!calibration_result_.v62_hc_target_feasible) {
            finishCalibrationShadow(Config::ZERO_CROSS_CAL_FAILURE_V62_NO_FEASIBLE_HC_TARGET);
          } else if (!fitV62RateStateModel()) {
            finishCalibrationShadow(Config::ZERO_CROSS_CAL_FAILURE_V62_NO_FEASIBLE_HC_RATE_TARGET);
          } else if (!configureV62RebuildTarget()) {
            finishCalibrationShadow(calibration_result_.v62_hc_target_feasible
                ? Config::ZERO_CROSS_CAL_FAILURE_V62_NO_FEASIBLE_HC_RATE_TARGET
                : Config::ZERO_CROSS_CAL_FAILURE_V62_NO_FEASIBLE_HC_TARGET);
          } else {
            calibration_phase_ = CalibrationPhase::Q_REBUILD;
          }
          if (logger_) logger_->setCalibrationResult(calibration_result_);
          return;
        }
        if (!dynamic_half_range_valid) {
          finishCalibrationShadow(2);
        } else if (configureV60RebuildTarget()) {
          calibration_phase_ = CalibrationPhase::Q_REBUILD;
        } else if (!calibration_result_.v60_gate_center_in_dynamic_h_input_support) {
          // The fit is valid but its measured predecessor domain does not yet
          // contain the unchanged H-gate center. Continue unforced free decay
          // and refit at the next eligible peak; never widen the fixed-Q gate.
          calibration_result_.v60_free_decay_support_wait_count++;
          if (calibration_fit_pos_.count >= Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_MAX_PER_SIDE &&
              calibration_fit_neg_.count >= Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_MAX_PER_SIDE) {
            finishCalibrationShadow(9);
          }
        } else {
          // H_gate is supported but the composed loss is not physically usable.
          finishCalibrationShadow(2);
        }
      }
      if (logger_) logger_->setCalibrationResult(calibration_result_);
    }
    return;
  }

  if (calibration_phase_ == CalibrationPhase::WAIT_Q_REBUILD) {
    const bool exact_rebuild = calibration_rebuild_pulse_id_ != 0 &&
                               status_.pulse_id == calibration_rebuild_pulse_id_;
    const bool post_rebuild = exact_rebuild &&
                              candidate_peak_ms > calibration_rebuild_pulse_start_test_ms_;
    if (!isConsecutive() || side == calibration_last_peak_side_) {
      recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 4,
                            calibration_last_peak_abs_deg_, 0.0f);
      finishCalibrationShadow(6);
      return;
    }
    if (!post_rebuild) {
      recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 5,
                            calibration_last_peak_abs_deg_, 0.0f);
      setLastPeak();
      return;
    }
    recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 6,
                          calibration_last_peak_abs_deg_, 0.0f);
    bool v61_state_observed = false;
    if (calibration_v61_rebuild_plan_active_) {
      const int8_t controlled_side = -Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE;
      float predicted_h = NAN;
      float predicted_c = NAN;
      const bool state_predicted = side == controlled_side &&
          predictV61GateState(peak_abs, &predicted_h, &predicted_c);
      if (state_predicted) {
        calibration_result_.v61_predicted_gate_h_deg = predicted_h;
        calibration_result_.v61_predicted_gate_c_deg = predicted_c;
        v61_state_observed = predicted_h >= Config::ZERO_CROSS_V59_GATE_HPREV_MIN_DEG &&
            predicted_h <= Config::ZERO_CROSS_V59_GATE_HPREV_MAX_DEG &&
            predicted_c >= Config::ZERO_CROSS_V59_GATE_CPREV_MIN_DEG &&
            predicted_c <= Config::ZERO_CROSS_V59_GATE_CPREV_MAX_DEG;
      }
      if (!v61_state_observed) {
        // Peak-error feedback remains unrestricted as requested.  It corrects
        // only the next command on the same controlled side; no positive-side
        // rebuild is inserted between attempts.
        calibration_v61_command_peak_target_deg_ +=
            calibration_v61_nominal_controlled_peak_target_deg_ - peak_abs;
        if (!isfinite(calibration_v61_command_peak_target_deg_) ||
            calibration_v61_command_peak_target_deg_ <= 0.0f) {
          calibration_v61_command_peak_target_deg_ =
              calibration_v61_nominal_controlled_peak_target_deg_;
        }
        calibration_result_.v61_feedback_command_peak_deg =
            calibration_v61_command_peak_target_deg_;
        ++calibration_result_.v61_feedback_correction_count;
      }
    } else {
      // Compatibility fallback: V61 configuration was not available, so retain
      // the old V60 raw-peak criterion rather than using a model outside support.
      v61_state_observed = calibration_result_.v60_rebuild_target_valid &&
          peak_abs >= calibration_result_.v60_rebuild_target_peak_deg;
    }
    if (v61_state_observed) {
      calibration_result_.v60_rebuild_target_observed = true;
      // Only a state-confirmed rebuild is followed by the required unforced
      // pair. The actual rate is still checked by the unchanged V59 gate.
      calibration_v60_cooldown_halfcycles_remaining_ =
          Config::ZERO_CROSS_V60_REBUILD_COOLDOWN_HALF_CYCLES;
    }
    setLastPeak();
    calibration_phase_ = CalibrationPhase::Q_REBUILD;
    if (logger_) logger_->setCalibrationResult(calibration_result_);
    return;
  }

  const bool wait_pos = calibration_phase_ == CalibrationPhase::WAIT_Q_CAL_POS;
  const bool wait_neg = calibration_phase_ == CalibrationPhase::WAIT_Q_CAL_NEG;
  if (!wait_pos && !wait_neg) {
    // A Q probe was deferred at a zero crossing. Keep the true free-decay
    // sequence current while waiting for the requested arrival side.
    if (!isConsecutive() || side == calibration_last_peak_side_) {
      recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 4,
                            calibration_last_peak_abs_deg_, 0.0f);
      finishCalibrationShadow(6);
      return;
    }
    recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 5,
                          calibration_last_peak_abs_deg_, 0.0f);
    setLastPeak();
    return;
  }

  const bool exact_probe = calibration_probe_pulse_id_ != 0 &&
                           status_.pulse_id == calibration_probe_pulse_id_;
  const bool post_pulse = exact_probe && candidate_peak_ms > calibration_probe_pulse_start_test_ms_;
  const bool consecutive = isConsecutive() && side != calibration_last_peak_side_;
  if (!consecutive) {
    recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 4,
                          calibration_last_peak_abs_deg_, 0.0f);
    finishCalibrationShadow(6);
    return;
  }

  if (!post_pulse) {
    // This physical peak predates the pulse, even though confirmation happened
    // later. It becomes the correct pre-pulse state; it cannot identify g.
    recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 5,
                          calibration_last_peak_abs_deg_, 0.0f);
    addProbeEvent(false, false, side == calibration_probe_requested_side_, false, false, 1, NAN);
    setLastPeak();
    calibration_probe_previous_peak_abs_deg_ = peak_abs;
    calibration_probe_previous_peak_signed_dynamic_deg_ = peak_signed_dynamic;
    calibration_probe_previous_peak_signed_fixed_deg_ = peak_signed_fixed;
    calibration_probe_previous_half_range_dynamic_deg_ = calibration_last_half_range_dynamic_deg_;
    calibration_probe_previous_half_range_fixed_deg_ = calibration_last_half_range_fixed_deg_;
    calibration_probe_previous_half_range_valid_ = calibration_last_half_range_valid_;
    calibration_probe_previous_center_dynamic_deg_ = calibration_last_center_dynamic_deg_;
    calibration_probe_previous_center_dynamic_valid_ = calibration_last_center_dynamic_valid_;
    calibration_probe_previous_peak_candidate_ms_ = candidate_peak_ms;
    return;
  }

  const float support_min = side > 0 ? calibration_result_.free_input_min_pos_deg :
                                       calibration_result_.free_input_min_neg_deg;
  const float support_max = side > 0 ? calibration_result_.free_input_max_pos_deg :
                                       calibration_result_.free_input_max_neg_deg;
  const float margin = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SUPPORT_MARGIN_DEG;
  const bool in_support = calibration_probe_previous_peak_candidate_ms_ < calibration_probe_pulse_start_test_ms_ &&
                          calibration_probe_previous_peak_abs_deg_ >= support_min + margin &&
                          calibration_probe_previous_peak_abs_deg_ <= support_max - margin;
  const float gain = in_support && calibration_probe_q_effective_pred_mA_s_ > 0.0f
      ? (peak_abs - ( (side > 0 ? calibration_result_.r_pos : calibration_result_.r_neg) *
                      calibration_probe_previous_peak_abs_deg_ +
                      (side > 0 ? calibration_result_.c_pos_deg : calibration_result_.c_neg_deg))) /
            calibration_probe_q_effective_pred_mA_s_
      : NAN;
  const bool positive_gain = isfinite(gain) &&
      gain > Config::ZERO_CROSS_CALIBRATION_Q_PROBE_MIN_POSITIVE_GAIN_DEG_PER_MAS;
  bool gain_stored = false;
  if (positive_gain && (side > 0
      ? calibration_result_.q_count_pos < Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SAMPLES_PER_SIDE
      : calibration_result_.q_count_neg < Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SAMPLES_PER_SIDE)) {
    if (side > 0) {
      calibration_gain_sum_pos_ += gain;
      calibration_gain_sum_sq_pos_ += gain * gain;
      calibration_result_.q_count_pos++;
      const float n = calibration_result_.q_count_pos;
      calibration_result_.g_pos = calibration_gain_sum_pos_ / n;
      calibration_result_.g_pos_stddev = sqrtf(fmaxf(0.0f, calibration_gain_sum_sq_pos_ / n -
          calibration_result_.g_pos * calibration_result_.g_pos));
      calibration_result_.q_used_pos_mA_s = calibration_probe_q_effective_pred_mA_s_;
    } else {
      calibration_gain_sum_neg_ += gain;
      calibration_gain_sum_sq_neg_ += gain * gain;
      calibration_result_.q_count_neg++;
      const float n = calibration_result_.q_count_neg;
      calibration_result_.g_neg = calibration_gain_sum_neg_ / n;
      calibration_result_.g_neg_stddev = sqrtf(fmaxf(0.0f, calibration_gain_sum_sq_neg_ / n -
          calibration_result_.g_neg * calibration_result_.g_neg));
      calibration_result_.q_used_neg_mA_s = calibration_probe_q_effective_pred_mA_s_;
    }
    gain_stored = true;
  }

  recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, side > 0 ? 2 : 3,
                        calibration_probe_previous_peak_abs_deg_, calibration_probe_q_effective_pred_mA_s_);
  const bool side_matched = side == calibration_probe_requested_side_;
  const uint8_t result_code = !side_matched ? 2 : !in_support ? 3 :
      !isfinite(gain) ? 4 : !positive_gain ? 5 : gain_stored ? 0 : 6;
  addProbeEvent(true, in_support, side_matched, positive_gain, gain_stored, result_code, gain);
  // A post-pulse peak completes this support-entry episode. A later support
  // loss starts a new bounded episode without hiding its run-wide total.
  calibration_rebuild_attempt_in_episode_ = 0;
  calibration_result_.rebuild_total_count = calibration_rebuild_total_count_;
  calibration_result_.rebuild_episode_id = calibration_rebuild_episode_id_;
  calibration_result_.rebuild_attempt_in_episode = 0;
  calibration_probe_wait_halfcycle_count_ = 0;
  setLastPeak();
  if (calibration_probe_plan_index_ >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_COUNT) {
    // Complete the deterministic protocol even if a signed response is not
    // positive. protocol_complete and calibration_valid are deliberately
    // separate: only the latter can enable a log-only calibrated prediction.
    finishCalibrationShadow(0);
  } else {
    // A probe can legitimately leave the following predecessor below the
    // fitted input domain. Return through Q_REBUILD so the planned next side
    // is either reached after an unforced half-cycle or rebuilt safely.
    calibration_phase_ = CalibrationPhase::Q_REBUILD;
  }
  if (logger_) logger_->setCalibrationResult(calibration_result_);
}
void ExperimentRunner::beginIdentificationEvent(uint32_t now_ms, uint32_t t_test_ms, int8_t direction,
                                                  float target_q_mA_s, float requested_q_mA_s,
                                                  float command_q_mA_s, float target_peak_deg,
                                                  float predicted_peak_deg, float min_predicted_peak_deg,
                                                  float max_predicted_peak_deg, bool target_reachable, bool bootstrap,
                                                  uint16_t width_ms, bool suppressed) {
  const float i0 = predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  const float q_effective_pred_mA_s = fabsf(identificationChargeMaS(i0, direction, width_ms));
  const float rate_dps = status_.gyro_pitch_rate_dps - pitchBiasFromGyroBias();
  const int8_t next_peak_side = rate_dps >= 0.0f ? 1 : -1;
  const float a_pred_calibrated = calibratedPredictionDeg(q_effective_pred_mA_s, next_peak_side);
  PsramLogger::IdentificationEvent e;
  e.start_ms = t_test_ms;
  e.theta0_cdeg = centi(status_.pitch_dynamic_beta_deg[Config::FILTER_ADOPTED_INDEX]);
  e.rate0_cdps = centi(status_.gyro_pitch_rate_dps - pitchBiasFromGyroBias());
  e.direction = direction;
  e.vbat_mV = status_.beta_model_vbat_mV;
  e.i0_mA = lroundf(i0);
  e.pulse_width_ms = width_ms;
  e.q_target_mAms = lroundf(target_q_mA_s * 1000.0f);
  e.q_requested_mAms = lroundf(requested_q_mA_s * 1000.0f);
  e.q_command_mAms = lroundf(command_q_mA_s * 1000.0f);
  e.q_estimated_mAms = lroundf(identificationChargeMaS(i0, direction, width_ms) * 1000.0f);
  e.a_target_cdeg = centi(target_peak_deg);
  e.a_pred_cdeg = centi(predicted_peak_deg);
  e.a_pred_shadow_cdeg = centi(predicted_peak_deg + Config::ZERO_CROSS_Q_MODEL_SHADOW_OFFSET_DEG);
  e.a_pred_calibrated_cdeg = isfinite(a_pred_calibrated) ? centi(a_pred_calibrated) : LOG_NAN_I16;
  e.prev_peak_abs_cdeg = calibration_last_peak_valid_ ? centi(calibration_last_peak_abs_deg_) : LOG_NAN_I16;
  e.q_effective_pred_mAms = lroundf(q_effective_pred_mA_s * 1000.0f);
  e.next_peak_side = next_peak_side;
  e.calibrated_prediction_valid = isfinite(a_pred_calibrated);
  e.a_min_cdeg = centi(min_predicted_peak_deg);
  e.a_max_cdeg = centi(max_predicted_peak_deg);
  e.target_reachable = target_reachable;
  e.bootstrap = bootstrap;
  e.pulse_suppressed = suppressed;
  identification_event_id_ = logger_->beginIdentificationEvent(e);
  identification_peak_waiting_ = identification_event_id_ != 0;
  identification_outbound_rate_sign_ = 0;
  identification_reverse_samples_ = 0;
  identification_peak_angle_deg_ = status_.pitch_dynamic_beta_deg[Config::FILTER_ADOPTED_INDEX];
}

void ExperimentRunner::updateIdentificationPeak(uint32_t now_ms, float angle_deg, float rate_dps) {
  if (!identification_peak_waiting_) return; if (fabsf(angle_deg)>fabsf(identification_peak_angle_deg_)) identification_peak_angle_deg_=angle_deg;
  if (identification_outbound_rate_sign_==0) { if (fabsf(rate_dps)>=Config::ZERO_CROSS_MIN_RATE_DPS) identification_outbound_rate_sign_=rate_dps>=0?1:-1; return; }
  if (rate_dps*identification_outbound_rate_sign_<=-Config::ZERO_CROSS_MIN_RATE_DPS) { if (++identification_reverse_samples_>=Config::ZERO_CROSS_IDENTIFICATION_PEAK_CONFIRM_SAMPLES) { logger_->finishIdentificationEvent(identification_event_id_, now_ms-trial_start_ms_, centi(identification_peak_angle_deg_)); identification_peak_waiting_=false; } } else identification_reverse_samples_=0;
}
void ExperimentRunner::updatePulseModelPrediction() {
  uint16_t source_vbat_mV = status_.roller_battery_mV;
  float model_vbat_v = Config::MODEL_VBAT_REFERENCE_V;
  status_.beta_model_vbat_status = 0;
  if (source_vbat_mV < 1000) {
    status_.beta_model_vbat_status = 2;
  } else {
    model_vbat_v = static_cast<float>(source_vbat_mV) / 1000.0f;
    if (model_vbat_v < Config::MODEL_VBAT_MIN_V) {
      model_vbat_v = Config::MODEL_VBAT_MIN_V;
      status_.beta_model_vbat_status = 1;
    } else if (model_vbat_v > Config::MODEL_VBAT_MAX_V) {
      model_vbat_v = Config::MODEL_VBAT_MAX_V;
      status_.beta_model_vbat_status = 1;
    }
  }
  status_.beta_model_vbat_mV = static_cast<uint16_t>(lroundf(model_vbat_v * 1000.0f));
  const float goal_mA = predictCurrentGoalMa(status_.current_mA_setting, model_vbat_v);
  const float tau_rise_s = predictRiseTauS(status_.current_mA_setting);
  const float width_s = static_cast<float>(status_.pulse_width_ms_setting) / 1000.0f;
  const float peak_mA = goal_mA * (1.0f - expf(-width_s / tau_rise_s));
  status_.predicted_i_goal_mA = static_cast<int16_t>(lroundf(goal_mA));
  status_.predicted_peak_current_mA = static_cast<int16_t>(lroundf(peak_mA));
  status_.predicted_beta_min = predictBetaMin(peak_mA);
}

float ExperimentRunner::predictCurrentGoalMa(float command_mA, float model_vbat_v) const {
  const float u_mA = fabsf(command_mA);
  if (u_mA <= 0.0f) return 0.0f;
  const float i_sat_mA = Config::MODEL_I_SAT_AT_REFERENCE_MA +
                         Config::MODEL_I_SAT_SLOPE_MA_PER_V *
                             (model_vbat_v - Config::MODEL_VBAT_REFERENCE_V);
  const float ratio = u_mA / i_sat_mA;
  return u_mA / powf(1.0f + powf(ratio, Config::MODEL_I_SAT_EXPONENT),
                      1.0f / Config::MODEL_I_SAT_EXPONENT);
}

float ExperimentRunner::predictRiseTauS(float command_mA) const {
  const float u_mA = fabsf(command_mA);
  const float ratio = u_mA / Config::MODEL_TAU_RISE_U_MA;
  const float tau_ms = Config::MODEL_TAU_RISE_MIN_MS +
                       (Config::MODEL_TAU_RISE_MAX_MS - Config::MODEL_TAU_RISE_MIN_MS) /
                           (1.0f + powf(ratio, Config::MODEL_TAU_RISE_EXPONENT));
  return tau_ms / 1000.0f;
}

float ExperimentRunner::predictBetaMin(float peak_current_mA) const {
  float ratio = peak_current_mA / Config::BETA_MIN_REFERENCE_CURRENT_MA;
  if (ratio < 0.0f) ratio = 0.0f;
  if (ratio > 1.0f) ratio = 1.0f;
  return Config::BETA_MIN_AT_ZERO_CURRENT +
         (Config::BETA_MIN_AT_REFERENCE_CURRENT - Config::BETA_MIN_AT_ZERO_CURRENT) * ratio;
}

float ExperimentRunner::betaFloorForStrategy(uint8_t index) const {
  return (index == Config::FILTER_FIXED_B100_INDEX || index == Config::FILTER_FIXED_B000_INDEX) ? betaCeilingForStrategy(index) : status_.predicted_beta_min;
}

float ExperimentRunner::betaCeilingForStrategy(uint8_t index) const {
  if (index >= Config::DYNAMIC_BETA_COUNT) return Config::MADGWICK_BETA_NORMAL;
  const float requested = Config::DYNAMIC_BETA_CEILINGS[index];
  // Start every series from the same gravity-converged quaternion. During
  // calibration, settling, ready, and start-sync, use beta=0.100 for all of
  // them. At t_test=0, RUNNING_BATCH_SWEEP selects each configured beta.
  if (status_.state != ExperimentState::RUNNING_BATCH_SWEEP &&
      status_.state != ExperimentState::TRIAL_REST) {
    return Config::MADGWICK_BETA_NORMAL;
  }
  return (index == Config::FILTER_FIXED_B100_INDEX || index == Config::FILTER_FIXED_B000_INDEX) ? requested : fmaxf(requested, status_.predicted_beta_min);
}

uint16_t ExperimentRunner::betaHoldAfterInputMsForStrategy(uint8_t index) const {
  if (index >= Config::DYNAMIC_BETA_COUNT) return Config::BETA_HOLD_AFTER_INPUT_MS;
  return Config::DYNAMIC_BETA_HOLD_AFTER_INPUT_MS[index];
}
void ExperimentRunner::logSampleIfDue() {
  const uint32_t now_us = micros();
  // Preserve the normal 20 ms time series. While an already-authorized pulse
  // is live, add rows at the current-audit period so fresh-read gaps can be
  // evaluated offline. Logging rate cannot alter the motor command.
  const uint32_t period_us = fixed_probe_mode_ ? 100000UL : status_.pulse_active
      ? Config::CURRENT_AUDIT_LOG_PERIOD_US : Config::LOG_PERIOD_MS * 1000UL;
  if (last_log_us_ != 0 && static_cast<uint32_t>(now_us - last_log_us_) < period_us) return;
  logSampleNow();
}

void ExperimentRunner::logSampleNow() {
  if (!logger_ || !logger_->ready()) return;
  const uint32_t timing_log_start_us = micros();
  if (timing_audit_) {
    timing_audit_->log_task_start_us = timing_log_start_us;
    timing_audit_->log_task_sequence = ++timing_log_sequence_;
  }
  const uint32_t now_us = micros();
  if (logger_->full()) {
    requestEmergencyStop("log_buffer_full");
    return;
  }
  LogSample row{};
  row.time_us = static_cast<uint32_t>(now_us - run_start_us_);
  if (status_.sync_event_id == 2) {
    row.time_us = 0;
    row.t_test_ms = 0;
  } else if (status_.state == ExperimentState::RUNNING_BATCH_SWEEP || status_.state == ExperimentState::TRIAL_REST) {
    row.t_test_ms = millis() - run_start_ms_;
  } else {
    row.t_test_ms = status_.measure_elapsed_ms;
  }
  row.state_id = static_cast<uint8_t>(status_.state);
  row.pulse_id = status_.pulse_id;
  row.pulse_active = status_.pulse_active ? 1 : 0;
  row.pulse_direction = status_.pulse_direction;
  row.motor_cmd_mA = status_.motor_cmd_mA;
  row.current_mA_setting = status_.current_mA_setting;
  row.pulse_width_ms_setting = status_.pulse_width_ms_setting;
  row.input_interval_ms = status_.input_interval_ms;
  row.trial_index = status_.trial_index;
  row.trial_count = status_.trial_count;
  row.trial_elapsed_ms = status_.trial_elapsed_ms;
  row.trial_duration_ms = status_.trial_duration_ms;
  row.trial_current_mA = status_.current_mA_setting;
  row.trial_pulse_width_ms = status_.pulse_width_ms_setting;
  row.trial_input_interval_ms = status_.input_interval_ms;
  row.trial_predicted_beta_min_x10000 = betaScaled(status_.predicted_beta_min);
  row.beta_recovery_tau_ms = static_cast<uint16_t>(lroundf(status_.beta_recovery_tau_s_setting * 1000.0f));
  row.beta_model_vbat_mV = status_.beta_model_vbat_mV;
  row.predicted_i_goal_mA = status_.predicted_i_goal_mA;
  row.predicted_peak_current_mA = status_.predicted_peak_current_mA;
  row.beta_model_vbat_status = status_.beta_model_vbat_status;
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    row.beta_ceiling_series_x10000[i] = betaScaled(betaCeilingForStrategy(i));
    row.pitch_dynamic_series_cdeg[i] = centi(status_.pitch_dynamic_beta_deg[i]);
  }
  row.pitch_madgwick_beta1_raw_cdeg = centi(status_.pitch_madgwick_beta1_raw_deg);
  row.pitch_madgwick_beta1_bias_cdeg = centi(status_.pitch_madgwick_beta1_bias_deg);
  row.pitch_gyro_raw_cdeg = centi(status_.pitch_gyro_raw_deg);
  row.pitch_gyro_bias_corrected_cdeg = centi(status_.pitch_gyro_bias_corrected_deg);
  row.pitch_accel_only_cdeg = centi(status_.pitch_accel_only_deg);
  row.gyro_bias_x_cdps = centi(status_.gyro_bias_x_dps);
  row.gyro_bias_y_cdps = centi(status_.gyro_bias_y_dps);
  row.gyro_bias_z_cdps = centi(status_.gyro_bias_z_dps);
  row.gyro_pitch_rate_cdps = centi(status_.gyro_pitch_rate_dps);
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    row.beta_target_series_x10000[i] = betaScaled(status_.beta_target_series[i]);
    row.beta_applied_series_x10000[i] = betaScaled(status_.beta_smooth_series[i]);
  }
  row.ax_mg = milli(status_.ax_g);
  row.ay_mg = milli(status_.ay_g);
  row.az_mg = milli(status_.az_g);
  row.gx_cdps = centi(status_.gx_dps);
  row.gy_cdps = centi(status_.gy_dps);
  row.gz_cdps = centi(status_.gz_dps);
  row.acc_norm_mg = milli(status_.acc_norm_g);
  const RollerTelemetry& roller_telemetry = roller_->telemetry();
  row.roller_actual_current_mA = roller_telemetry.actual_current_mA;
  row.roller_battery_mV = roller_telemetry.battery_mV;
  row.roller_current_sample_time_us = roller_telemetry.current_sample_time_us;
  row.roller_current_sequence = roller_telemetry.current_sequence;
  row.roller_current_age_us = roller_->currentAgeUs(now_us);
  row.roller_current_read_failure_count = roller_telemetry.current_read_failure_count;
  row.roller_q_meas_observed_mAms = isfinite(roller_telemetry.q_meas_observed_mA_s)
      ? static_cast<int32_t>(lroundf(roller_telemetry.q_meas_observed_mA_s * 1000.0f)) : LOG_NAN_I32;
  row.pulse_q_target_mAms = isfinite(status_.current_audit_q_target_mA_s)
      ? static_cast<int32_t>(lroundf(status_.current_audit_q_target_mA_s * 1000.0f)) : LOG_NAN_I32;
  row.pulse_q_pred_mAms = isfinite(status_.current_audit_q_pred_mA_s)
      ? static_cast<int32_t>(lroundf(status_.current_audit_q_pred_mA_s * 1000.0f)) : LOG_NAN_I32;
  row.roller_current_sample_count = roller_telemetry.current_audit_sample_count;
  row.roller_current_valid = roller_telemetry.current_valid ? 1 : 0;
  row.roller_q_meas_observed_valid = roller_telemetry.q_meas_observed_valid ? 1 : 0;
  row.roller_current_audit_max_gap_us = roller_telemetry.current_audit_max_gap_us;
  row.roller_current_audit_gt_3333_count = roller_telemetry.current_audit_gt_3333_count;
  row.roller_current_audit_interval_count = roller_telemetry.current_audit_interval_count;
  row.roller_current_audit_tail_max_gap_us = roller_telemetry.current_audit_tail_max_gap_us;
  row.roller_current_audit_tail_gt_3333_count = roller_telemetry.current_audit_tail_gt_3333_count;
  row.roller_current_audit_tail_interval_count = roller_telemetry.current_audit_tail_interval_count;
  row.roller_current_audit_pulse_end_current_age_us =
      roller_telemetry.current_audit_pulse_end_current_age_us;
  row.roller_current_audit_imu_deadline_guard_count =
      roller_telemetry.current_audit_imu_deadline_guard_count;
  row.roller_current_audit_post_imu_service_count =
      roller_telemetry.current_audit_post_imu_service_count;
  row.roller_current_audit_finalized = roller_telemetry.current_audit_finalized ? 1 : 0;
  row.roller_current_audit_tail_covered = roller_telemetry.current_audit_tail_covered ? 1 : 0;
  row.led_state = status_.led_state ? 1 : 0;
  row.sync_event_id = status_.sync_event_id;
  row.log_active =
      (status_.state == ExperimentState::START_SYNC || status_.state == ExperimentState::RUNNING_BATCH_SWEEP ||
       status_.state == ExperimentState::TRIAL_REST || status_.state == ExperimentState::END_SYNC) ? 1 : 0;
  row.beta_phase_state = status_.beta_phase_state;
  row.beta_phase_progress_x10000 = betaScaled(status_.beta_phase_progress);
  row.beta_phase_peak_angle_cdeg = centi(status_.beta_phase_peak_angle_deg);
  row.beta_phase_angle_cdeg = centi(status_.beta_phase_angle_deg);
  row.beta_phase_ceiling_x10000 = betaScaled(status_.beta_phase_ceiling);
  row.physical_roll_abs_cdeg = centi(status_.physical_roll_abs_deg);
  row.current_roll_cdeg = centi(status_.current_roll_deg);
  row.physical_roll_rate_cdps = centi(status_.physical_roll_rate_dps);
  row.target_roll_cdeg = centi(status_.target_roll_deg);
  row.target_error_cdeg = centi(status_.target_error_deg);
  row.static_confirmed = status_.static_confirmed ? 1 : 0;
  row.ready = status_.ready ? 1 : 0;
  const TimingAudit& timing = completed_timing_audit_;
  row.timing_loop_sequence = timing.loop_sequence;
  row.timing_loop_total_us = timing.loop_total_us;
  row.timing_m5_update_us = timing.m5_update_us;
  row.timing_service_fast_us = timing.service_fast_us;
  row.timing_beta_context_us = timing.beta_context_us;
  row.timing_imu_update_total_us = timing.imu_update_total_us;
  row.timing_roller_update_total_us = timing.roller_update_total_us;
  row.timing_runner_update_total_us = timing.runner_update_total_us;
  row.timing_web_update_us = timing.web_update_us;
  row.timing_imu_hw_update_us = timing.imu_hw_update_us;
  row.timing_imu_get_data_us = timing.imu_get_data_us;
  row.timing_imu_manager_filters_us = timing.imu_manager_filters_us;
  row.timing_runner_beta1_filters_us = timing.runner_beta1_filters_us;
  row.timing_runner_dynamic_all_us = timing.runner_dynamic_all_us;
  row.timing_adopted_filter_only_us = timing.adopted_filter_only_us;
  row.timing_beta_turn_fast_us = timing.beta_turn_fast_us;
  row.timing_update_filter_series_us = timing.update_filter_series_us;
  row.timing_update_displayed_angles_us = timing.update_displayed_angles_us;
  row.timing_update_current_roll_us = timing.update_current_roll_us;
  row.timing_q1_shadow_us = timing.q1_shadow_us;
  row.timing_autonomous_motion_us = timing.autonomous_motion_us;
  row.timing_fast_current_read_us = timing.fast_current_read_us;
  row.timing_roller_full_status_us = timing.roller_full_status_us;
  row.timing_log_sample_us = timing.log_sample_us;
  row.timing_imu_task_start_us = timing.imu_task_start_us;
  row.timing_imu_task_sequence = timing.imu_task_sequence;
  row.timing_roller_full_status_start_us = timing.roller_full_status_start_us;
  row.timing_roller_full_status_sequence = timing.roller_full_status_sequence;
  row.timing_runner_filter_start_us = timing.runner_filter_start_us;
  row.timing_runner_filter_sequence = timing.runner_filter_sequence;
  row.timing_log_task_start_us = timing.log_task_start_us;
  row.timing_log_task_sequence = timing.log_task_sequence;
  row.timing_web_task_start_us = timing.web_task_start_us;
  row.timing_web_task_sequence = timing.web_task_sequence;
  logger_->addSample(row);
  if (timing_audit_) timing_audit_->log_sample_us = micros() - timing_log_start_us;
  if (status_.sync_event_id == 2 || status_.sync_event_id == 3 || status_.sync_event_id == 6 ||
      status_.sync_event_id == 7) {
    status_.sync_event_id = 0;
  }
  if (last_log_us_ != 0) status_.log_dt_us = now_us - last_log_us_;
  last_log_us_ = now_us;
}

void ExperimentRunner::finishRun() {
  if (roller_) roller_->stop(MeasuredQStop::EXPERIMENT_END);
  stopMotor();
  status_.measure_elapsed_ms = fixed_probe_mode_ ? millis()-run_start_ms_ : measurementTotalDurationMs();
  status_.remaining_ms = 0;
  status_.sync_event_id = 3;
  sync_led_until_ms_ = 0;
  setSyncLed(false);
  logSampleNow();
  beginEndSync(millis());
}

void ExperimentRunner::stopMotor() {
  q_ident_pulse_authorized_ = false;
  energy_control_v0_pulse_authorized_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  if (roller_) roller_->stop();
  status_.motor_cmd_mA = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  if (roller_) status_.roller_actual_current_mA = roller_->telemetry().actual_current_mA;
}

void ExperimentRunner::setSyncLed(bool on) {
  digitalWrite(Config::SYNC_LED_PIN, on ? HIGH : LOW);
  status_.led_state = on;
}

uint32_t ExperimentRunner::measurementTotalDurationMs() const {
  if(fixed_probe_mode_)return FixedProbeV57::RUN_LIMIT_MS;
  if (wheel_probe_mode_) return WheelProbeV55::RUN_LIMIT_MS;
  if (energy_control_autonomous_mode_) return Config::ENERGY_CONTROL_AUTONOMOUS_DURATION_MS;
  if (energy_control_v0_mode_) return Config::ENERGY_CONTROL_V0_DURATION_MS;
  if (q_ident_mode_) return Config::Q_IDENT_DURATION_MS;
  if (passive_capture_mode_) return Config::PASSIVE_CAPTURE_DURATION_MS;
  if (zero_cross_mode_) {
    return q_run_mode_ == QRunMode::CONTROL && Config::ZERO_CROSS_V59_STATE_WAIT_FIXED_Q_ENABLED
        ? Config::ZERO_CROSS_V59_TEST_DURATION_MS : Config::ZERO_CROSS_TEST_DURATION_MS;
  }
  if (single_trial_mode_ && selected_trial_index_ < Config::BETA_SWEEP_TRIAL_COUNT) {
    return Config::BETA_SWEEP_TRIALS[selected_trial_index_].duration_ms;
  }
  return Config::BETA_SWEEP_TOTAL_DURATION_MS;
}
float ExperimentRunner::accelPitchDeg(const ImuReading& r) const {
  return Config::PITCH_SIGN * atan2f(-r.ax_g, sqrtf(r.ay_g * r.ay_g + r.az_g * r.az_g)) * 57.2957795f;
}

float ExperimentRunner::physicalRollCandidateDeg(const ImuReading& r) const {
  return atan2f(r.ax_g, sqrtf(r.ay_g * r.ay_g + r.az_g * r.az_g)) * 57.2957795f;
}

float ExperimentRunner::pitchBiasFromGyroBias() const {
  return Config::GYRO_PITCH_RATE_SIGN * status_.gyro_bias_y_dps;
}

int16_t ExperimentRunner::centi(float value) const {
  if (!isfinite(value)) return LOG_NAN_I16;
  value *= 100.0f;
  if (value > 32767.0f) return 32767;
  if (value < -32767.0f) return -32767;
  return static_cast<int16_t>(lroundf(value));
}

int16_t ExperimentRunner::milli(float value) const {
  if (!isfinite(value)) return LOG_NAN_I16;
  value *= 1000.0f;
  if (value > 32767.0f) return 32767;
  if (value < -32767.0f) return -32767;
  return static_cast<int16_t>(lroundf(value));
}

int16_t ExperimentRunner::betaScaled(float value) const {
  if (!isfinite(value)) return LOG_NAN_I16;
  value *= 10000.0f;
  if (value > 32767.0f) return 32767;
  if (value < -32767.0f) return -32767;
  return static_cast<int16_t>(lroundf(value));
}

const char* ExperimentRunner::stateName() const {
  switch (status_.state) {
    case ExperimentState::STARTUP_GYRO_CALIB: return "STARTUP_GYRO_CALIB";
    case ExperimentState::MADGWICK_SETTLING: return "MADGWICK_SETTLING";
    case ExperimentState::READY_TO_MEASURE: return "READY_TO_MEASURE";
    case ExperimentState::RUNNING_BATCH_SWEEP: return "RUNNING_BATCH_SWEEP";
    case ExperimentState::FINISHED: return "FINISHED";
    case ExperimentState::ESTOP: return "ESTOP";
    case ExperimentState::START_SYNC: return "START_SYNC";
    case ExperimentState::END_SYNC: return "END_SYNC";
    case ExperimentState::TRIAL_REST: return "TRIAL_REST";
  }
  return "UNKNOWN";
}









