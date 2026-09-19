#pragma once

#include <Adafruit_AHRS.h>
#include <Arduino.h>

#include "beta_phase_controller.h"
#include "beta_turn_fast_controller.h"
#include "config.h"
#include "imu_manager.h"
#include "log_types.h"
#include "timing_audit.h"
#include "psram_logger.h"
#include "roller485_manager.h"

struct ExperimentStatus {
  ExperimentState state = ExperimentState::STARTUP_GYRO_CALIB;
  uint16_t run_id = 0;
  bool running = false;
  bool emergency_stop = false;
  uint32_t boot_elapsed_ms = 0;
  uint32_t measure_elapsed_ms = 0;
  uint32_t remaining_ms = Config::BETA_SWEEP_TOTAL_DURATION_MS;
  uint8_t trial_index = 0;
  uint8_t trial_count = Config::BETA_SWEEP_TRIAL_COUNT;
  uint32_t trial_elapsed_ms = 0;
  uint32_t trial_duration_ms = Config::BETA_SWEEP_TRIAL_DURATION_MS;
  uint32_t calibration_sample_count = 0;
  uint32_t pulse_id = 0;
  bool pulse_active = false;
  int8_t pulse_direction = 0;
  int16_t motor_cmd_mA = 0;
  bool led_state = false;
  uint8_t sync_event_id = 0;
  int16_t current_mA_setting = Config::DEFAULT_INPUT_CURRENT_MA;
  uint16_t pulse_width_ms_setting = Config::DEFAULT_PULSE_WIDTH_MS;
  uint16_t input_interval_ms = Config::DEFAULT_INPUT_INTERVAL_MS;
  float predicted_beta_min = Config::BETA_MIN_AT_ZERO_CURRENT;
  uint16_t beta_hold_after_input_ms_setting = Config::BETA_HOLD_AFTER_INPUT_MS;
  float beta_recovery_tau_s_setting = Config::BETA_RECOVERY_TAU_S;
  uint16_t beta_model_vbat_mV = 0;
  int16_t predicted_i_goal_mA = 0;
  int16_t predicted_peak_current_mA = 0;
  uint8_t beta_model_vbat_status = 2;
  int16_t roller_actual_current_mA = 0;
  uint16_t roller_battery_mV = 0;
  // v45 current telemetry labels only; no control path reads these values.
  float current_audit_q_target_mA_s = NAN;
  float current_audit_q_pred_mA_s = NAN;
  float gyro_bias_x_dps = 0.0f;
  float gyro_bias_y_dps = 0.0f;
  float gyro_bias_z_dps = 0.0f;
  float gyro_bias_pitch_dps = 0.0f;
  float pitch_madgwick_beta1_raw_deg = 0.0f;
  float pitch_madgwick_dynamic_raw_deg = 0.0f;
  float pitch_madgwick_beta1_bias_deg = 0.0f;
  float pitch_madgwick_dynamic_bias_deg = 0.0f;
  float pitch_dynamic_beta_deg[Config::DYNAMIC_BETA_COUNT] = {};
  float pitch_gyro_raw_deg = 0.0f;
  float pitch_gyro_bias_corrected_deg = 0.0f;
  float pitch_accel_only_deg = 0.0f;
  float gyro_pitch_rate_dps = 0.0f;
  float beta_target = Config::MADGWICK_BETA_NORMAL;
  float beta_smooth = Config::MADGWICK_BETA_NORMAL;
  uint8_t beta_phase_state = 0;
  float beta_phase_progress = 0.0f;
  float beta_phase_peak_angle_deg = 0.0f;
  float beta_phase_angle_deg = 0.0f;
  float beta_phase_ceiling = Config::MADGWICK_BETA_DYNAMIC_MAX;
  float beta_target_series[Config::DYNAMIC_BETA_COUNT] = {};
  float beta_smooth_series[Config::DYNAMIC_BETA_COUNT] = {};
  float ax_g = 0.0f;
  float ay_g = 0.0f;
  float az_g = 0.0f;
  float gx_dps = 0.0f;
  float gy_dps = 0.0f;
  float gz_dps = 0.0f;
  float acc_norm_g = 0.0f;
  // Static calibrated physical-roll display. These do not replace the
  // fixed-horizon/video coordinate used as the Model B teacher value.
  float physical_roll_candidate_deg = 0.0f;
  float physical_roll_abs_deg = 0.0f;
  float current_roll_deg = 0.0f;
  float physical_roll_rate_raw_dps = 0.0f;
  float physical_roll_rate_dps = 0.0f;
  float display_zero_offset_deg = 0.0f;
  float target_roll_deg = 0.0f;
  float target_error_deg = 0.0f;
  bool static_confirmed = false;
  bool ready = false;
  uint32_t loop_dt_us = 0;
  uint32_t log_dt_us = 0;
  const char* last_error = "";
};

class ExperimentRunner {
public:
  enum class QRunMode : uint8_t { NONE = 0, VALIDATION = 1, CONTROL = 2 };
  void begin(PsramLogger& logger, ImuManager& imu, Roller485Manager& roller);
  void serviceFast();
  void update();
  void updateImuDynamicBetaContext();
  void setLoopDt(uint32_t dt_us) { status_.loop_dt_us = dt_us; }
  void setTimingAudit(TimingAudit* timing_audit) { timing_audit_ = timing_audit; }
  void commitTimingAudit(const TimingAudit& timing_audit) { completed_timing_audit_ = timing_audit; }

  bool startBatchSweepTest();
  bool startSingleTrialTest(uint8_t trial_number);
  bool startZeroCrossTest(int16_t fixed_current_mA, uint16_t fixed_pulse_width_ms,
                          QRunMode q_run_mode = QRunMode::NONE,
                          float control_target_deg = 0.0f,
                          uint8_t q_probe_schedule_id = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A);
  bool startZeroCrossIdentificationTest();
  bool startZeroCrossControlTest(float target_peak_deg, uint8_t q_probe_schedule_id);
  bool startPassiveCapture();
  bool startWheelProbeCapture();
  bool startFixedProbeCapture(bool full=false);
  bool fixedProbeMode() const{return fixed_probe_mode_;}
  bool wheelProbeMode() const { return wheel_probe_mode_; }
  // Dedicated actual-output fixed-Q protocol. No Q1/E2/legacy control caller
  // can enter this path.
  bool startQIdentCapture(uint8_t q_ident_run_schedule_id);  // Frozen: fail closed in this revision.
  bool startEnergyControlV0Capture();
  bool startEnergyControlAutonomousCapture();
  bool setEnergyControlAutonomousTarget(float target_deg);
  void zeroAngleNow();
  bool zeroCurrentRollDisplay();
  bool setCurrentRollTarget(float target_deg);
  // Absolute next-peak target for Q1 direct shadow. It is snapshotted at
  // passive-run start and never feeds a motor command.
  bool setQ1ShadowTargetPeakAbs(float target_deg);
  float q1ShadowTargetPeakAbsDeg() const { return q1_shadow_target_peak_abs_deg_; }
  float q1ShadowActiveTargetPeakAbsDeg() const { return q1_shadow_run_target_peak_abs_deg_; }
  void requestEmergencyStop(const char* reason);
  void clearFinishedOrEstop();
  void setInputSettings(int16_t current_mA, uint16_t pulse_width_ms, uint16_t input_interval_ms);

  const ExperimentStatus& status() const { return status_; }
  bool zeroCrossMode() const { return zero_cross_mode_; }
  bool identificationMode() const { return identification_mode_; }
  bool controlMode() const { return q_run_mode_ == QRunMode::CONTROL; }
  bool passiveCaptureMode() const { return passive_capture_mode_; }
  bool qIdentMode() const { return q_ident_mode_; }
  bool energyControlV0Mode() const { return energy_control_v0_mode_; }
  bool energyControlAutonomousMode() const { return energy_control_autonomous_mode_; }
  bool autonomousExperimentRunning() const {
    return (energy_control_autonomous_mode_ || wheel_probe_mode_ || fixed_probe_mode_) && status_.state == ExperimentState::RUNNING_BATCH_SWEEP;
  }
  float energyControlAutonomousTargetPeakDeg() const { return energy_control_autonomous_target_peak_deg_; }
  const char* energyControlAutonomousPhaseName() const;
  bool qIdentArmed() const { return q_ident_armed_; }
  uint8_t qIdentRunScheduleId() const { return q_ident_run_schedule_id_ + 1; }
  uint8_t qIdentPlusOccurrenceCount() const { return q_ident_side_occurrence_count_[0]; }
  uint8_t qIdentMinusOccurrenceCount() const { return q_ident_side_occurrence_count_[1]; }
  const char* qRunModeName() const;
  uint8_t qProbeScheduleId() const { return q_probe_schedule_id_; }
  const char* qProbeScheduleName() const;
  float controlTargetPeakDeg() const { return control_target_peak_deg_; }
  int16_t zeroCrossFixedCurrentMa() const { return zero_cross_fixed_current_mA_; }
  bool running() const {
    return status_.state == ExperimentState::START_SYNC || status_.state == ExperimentState::RUNNING_BATCH_SWEEP ||
           status_.state == ExperimentState::TRIAL_REST || status_.state == ExperimentState::END_SYNC;
  }
  const char* stateName() const;
  const char* presetName() const { return "DYNAMIC_BETA_VBAT_HOLD_TIME_COMPARE"; }

private:
  bool fixed_probe_mode_=false, fixed_probe_full_=false;
  void serviceFixedProbeFast();
  void updateFixedProbeRun();
  bool wheel_probe_mode_ = false, wheel_probe_waiting_ = false;
  uint32_t wheel_probe_pulse_id_ = 0, wheel_probe_next_us_ = 0;
  void serviceWheelProbeFast();
  void updateWheelProbe();
  TimingAudit* timing_audit_ = nullptr;
  TimingAudit completed_timing_audit_{};
  uint32_t timing_runner_filter_sequence_ = 0;
  uint32_t timing_log_sequence_ = 0;
  void beginFilters();
  void updateFilterSeries(const ImuReading& r);
  void updateStartupCalibration(const ImuReading& r);
  void beginStartSync(uint32_t now_ms, bool wheel_probe = false, bool fixed_probe = false);
  void updateStartSync(uint32_t now_ms);
  void updateEndSync(uint32_t now_ms);
  bool updateSyncPattern(uint32_t now_ms, const Config::LedSyncStep* pattern, uint8_t step_count);
  void beginMeasurementRun();
  void beginTrial(uint8_t trial_index);
  void finishTrial(uint32_t now_ms);
  void updateTrialRest(uint32_t now_ms);
  void updateMidSyncLed(uint32_t now_ms);
  void beginEndSync(uint32_t now_ms);
  void updateInputPulse(uint32_t now_ms);
  void beginPulse(uint32_t now_ms, uint32_t t_test_ms, int8_t direction);
  void stopActivePulse(uint32_t now_ms);
  void updatePulseModelPrediction();
  float predictedChargeMaS(float signed_i0_mA, int8_t direction, float width_ms,
                            int16_t commanded_current_mA) const;
  float identificationChargeMaS(float signed_i0_mA, int8_t direction, float width_ms) const;
  uint16_t widthForChargeTarget(float target_q_mA_s, float signed_i0_mA, int8_t direction,
                                uint16_t min_width_ms, uint16_t max_width_ms) const;
  uint16_t identificationWidthForTarget(float target_q_mA_s, float signed_i0_mA, int8_t direction) const;
  uint16_t calibrationProbeWidthForTarget(float target_q_mA_s, float signed_i0_mA, int8_t direction) const;
  uint16_t calibrationWidthForTarget(float target_q_mA_s, float signed_i0_mA, int8_t direction) const;
  uint16_t controlWidthForTarget(float target_q_mA_s, float signed_i0_mA, int8_t direction) const;
  float predictNextPeakAbsDeg(float angle_deg, float rate_dps, float q_command_mA_s, int8_t direction) const;
  float requiredQForTargetPeakMaS(float target_peak_deg, float angle_deg, float rate_dps, int8_t direction) const;
  float requiredControlQMaS(float angle_deg, float rate_dps, int8_t direction) const;
  struct CalibrationBuildUpCommand {
    float q_required_mA_s = 0.0f; float q_command_mA_s = 0.0f; float q_effective_mA_s = 0.0f;
    float predicted_peak_deg = 0.0f; uint16_t width_ms = 0;
    bool q_cap_limited = false; bool guard_limited = false; bool width_limited = false;
  };
  CalibrationBuildUpCommand calibrationBuildUpCommand(float angle_deg, float rate_dps,
                                                        float signed_i0_mA, int8_t direction,
                                                        float target_peak_deg,
                                                        bool unrestricted = false) const;
  void recordCalibrationBuildUpPulse(uint32_t t_test_ms, float angle_deg, float rate_dps,
                                     int8_t direction, uint8_t phase, float target_peak_deg,
                                     const CalibrationBuildUpCommand& command);
  uint8_t calibrationProbeQLevelIndex(uint8_t plan_index) const;
  uint8_t classifyV59StateGate(float hprev_deg, float cprev_deg, float abs_rate_dps,
                               int8_t next_peak_side, bool dynamic_h_in_support) const;
  void recordV59StateGateEvent(uint32_t crossing_ms, float hprev_deg, float cprev_deg,
                               float abs_rate_dps, int8_t next_peak_side,
                               bool state_valid, bool dynamic_h_in_support,
                               uint8_t reason, uint8_t action,
                               bool v60_cooldown_free_decay = false,
                               bool v60_gate_skipped_by_decay = false);
  struct ControlQEvaluation {
    float q_command_mA_s = 0.0f;
    float predicted_peak_deg = 0.0f;
    float min_predicted_peak_deg = 0.0f;
    float max_predicted_peak_deg = 0.0f;
    bool target_reachable = false;
  };
  ControlQEvaluation evaluateControlQ(float angle_deg, float rate_dps, int8_t direction) const;
  enum class CalibrationPhase : uint8_t { INACTIVE, INITIAL_EXCITE, FREE_DECAY, Q_REBUILD,
                                           WAIT_Q_REBUILD, Q_CAL_POS, WAIT_Q_CAL_POS,
                                           Q_CAL_NEG, WAIT_Q_CAL_NEG, MAIN_CONTROL };
  struct CalibrationFit {
    uint8_t count = 0;
    float sum_x = 0.0f; float sum_y = 0.0f; float sum_x2 = 0.0f; float sum_y2 = 0.0f; float sum_xy = 0.0f;
    // Keep the individual pairs for leave-one-out cross-validation. The count
    // is bounded by the V60 free-decay support-wait limit per arrival side.
    float x_deg[Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_MAX_PER_SIDE] = {};
    float y_deg[Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_MAX_PER_SIDE] = {};
    // The selected model is valid only over this measured union of predecessor
    // and arrival amplitudes; it is never extrapolated beyond it.
    float support_min_deg = 0.0f; float support_max_deg = 0.0f;
    // Input domain is predecessor amplitude A_prev, separate from the
    // predecessor/arrival union used by the physical fit.
    float input_min_deg = 0.0f; float input_max_deg = 0.0f;
  };
  struct HalfRangeFit {
    static constexpr uint8_t kMaxPairs = 2 * Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_MAX_PER_SIDE;
    uint8_t count = 0;
    float x_deg[kMaxPairs] = {}; float y_deg[kMaxPairs] = {};
    float sum_x = 0.0f; float sum_y = 0.0f; float sum_x2 = 0.0f; float sum_xy = 0.0f;
    float input_min_deg = 0.0f; float input_max_deg = 0.0f;
  };
  struct RateStateFit {
    uint8_t count = 0;
    float sum_h = 0.0f; float sum_c = 0.0f; float sum_rate = 0.0f;
    float sum_h2 = 0.0f; float sum_hc = 0.0f; float sum_c2 = 0.0f;
    float sum_hrate = 0.0f; float sum_crate = 0.0f; float sum_rate2 = 0.0f;
    float h_min_deg = 0.0f; float h_max_deg = 0.0f;
    float c_min_deg = 0.0f; float c_max_deg = 0.0f;
    float a_per_s = 0.0f; float b_per_s = 0.0f; float offset_dps = 0.0f;
    float r2 = 0.0f;
  };
  void resetCalibrationShadow();
  void updateCalibrationPeak(uint32_t now_ms, uint32_t t_test_ms, float angle_deg, float rate_dps);
  void recordCalibrationPeak(uint32_t candidate_peak_ms, uint32_t confirmed_ms,
                             int8_t side, float peak_signed_dynamic_deg, float peak_signed_fixed_deg, uint8_t phase,
                             float previous_peak_abs_deg, float q_effective_pred_mA_s);
  bool fitCalibrationFreeDecay();
  void addHalfRangeFreePair(HalfRangeFit& fit, float previous_half_range_deg, float half_range_deg);
  bool fitHalfRangeShadow(HalfRangeFit& fit, bool dynamic);
  bool configureV60RebuildTarget();
  void addV62RateStateSample(float hprev_deg, float cprev_deg, float abs_rate_dps);
  bool fitV62RateStateModel();
  bool v62HasFeasibleHCTarget() const;
  bool configureV62RebuildTarget();
  bool predictV61GateState(float controlled_peak_abs_deg, float* predicted_h_deg,
                           float* predicted_c_deg) const;
  bool v60GateCenterInDynamicHInputSupport() const;
  float halfRangeShadowPrediction(float previous_half_range_deg, bool dynamic, bool* in_support) const;
  bool dynamicHalfRangePredecessorInSupport() const;
  void finishCalibrationShadow(uint8_t failure_reason);
  float calibratedPredictionDeg(float q_effective_pred_mA_s, int8_t next_peak_side) const;
  bool calibrationIsMainControl() const { return calibration_phase_ == CalibrationPhase::MAIN_CONTROL || calibration_phase_ == CalibrationPhase::INACTIVE; }
  void beginIdentificationEvent(uint32_t now_ms, uint32_t t_test_ms, int8_t direction,
                                float q_target_mA_s, float q_requested_mA_s,
                                float q_command_mA_s, float target_peak_deg,
                                float predicted_peak_deg, float min_predicted_peak_deg,
                                float max_predicted_peak_deg, bool target_reachable, bool bootstrap,
                                uint16_t width_ms, bool suppressed);
  void updateIdentificationPeak(uint32_t now_ms, float angle_deg, float rate_dps);
  float predictCurrentGoalMa(float command_mA, float model_vbat_v) const;
  float predictRiseTauS(float command_mA) const;
  float predictBetaMin(float peak_current_mA) const;
  float betaFloorForStrategy(uint8_t index) const;
  float betaCeilingForStrategy(uint8_t index) const;
  uint16_t betaHoldAfterInputMsForStrategy(uint8_t index) const;
  void captureAngleOffsets();
  void updateDisplayedAngles(const ImuReading& r);
  void updateCurrentRollState(const ImuReading& r, uint32_t now_ms);
  // Legacy E2 is compiled only as an offline diagnostic reference and is not
  // called by the passive Q1 runtime.
  void resetE2ShadowPeakTracker();
  void updateE2ShadowPeakTracker(uint32_t now_ms);
  void recordE2ShadowPeak(uint32_t candidate_peak_ms, uint32_t confirmed_ms,
                          int8_t turn_side_from_rate,
                          float candidate_gyro_relative_deg,
                          float candidate_accel_abs_diag_deg);
  // Diagnostic values derived while accepting a Q1 zero-cross. They are not
  // inputs to the Q1 equation; the official Q1 inputs remain the current
  // sample's time and +gy rate.
  struct Q1ZeroCrossDiagnostics {
    int8_t detector_crossing_direction = 0;  // +1: detector - to +; -1: + to -.
    float detector_angle_before_deg = NAN;
    float detector_angle_after_deg = NAN;
    float crossing_interpolation_alpha = NAN;
    uint32_t interpolated_zero_cross_time_ms = 0;
    float physical_roll_rate_before_dps = NAN;
    float physical_roll_rate_after_dps = NAN;
    float interpolated_physical_roll_rate_dps = NAN;
    bool sign_gate_passed = false;
  };
  void resetQ1ShadowZeroCrossTracker();
  void updateQ1ShadowAtZeroCross(uint32_t now_ms);
  void recordQ1ShadowZeroCross(uint32_t t_test_ms, float rate_dps,
                               const Q1ZeroCrossDiagnostics& diagnostics);
  void resetQIdentTracker();
  void updateQIdentAtZeroCross(uint32_t t_test_ms, float rate_dps,
                                const Q1ZeroCrossDiagnostics& diagnostics);
  void updateQIdentPulse(uint32_t now_ms);
  void updateEnergyControlV0AtZeroCross(uint32_t t_test_ms, float rate_dps,
                                         const Q1ZeroCrossDiagnostics& diagnostics);
  enum class EnergyControlV0OutputGateState : uint8_t {
    WAIT_INITIAL_EXCURSION = 0,
    ARMED_FOR_ZERO_CROSS = 1,
    WAIT_OPPOSITE_EXCURSION = 2,
  };
  void resetEnergyControlV0OutputGate();
  void updateEnergyControlV0OutputGate(float detector_relative_angle_deg);
  void disarmEnergyControlV0AfterAcceptedCross(int8_t physical_next_peak_side);
  void updateEnergyControlV0Pulse(uint32_t now_ms);
  bool beginEnergyControlV0Pulse(uint32_t now_ms, uint32_t t_test_ms, int8_t direction,
                                 uint16_t pulse_width_ms);
  enum class EnergyControlAutonomousPhase : uint8_t {
    IDLE = 0, STRONG_START_KICK = 1, WAIT_FIRST_PEAK = 2, ENERGY_CONTROL = 3, HOLD = 4, STOP = 5,
  };
  // Separate from the high-level target phase: one accepted physical peak, one
  // following accepted central passage, then at most one normal pulse.
  enum class EnergyControlAutonomousHalfCycleState : uint8_t {
    WAIT_PEAK = 0, WAIT_ZERO_CROSS = 1, PULSE_ACTIVE = 2,
  };
  void resetEnergyControlAutonomous();
  void resetEnergyControlAutonomousPeakTracker(bool enable);
  void beginEnergyControlAutonomousStartKick(uint32_t now_ms);
  void updateEnergyControlAutonomousMotion(uint32_t now_ms);
  void updateEnergyControlAutonomousPeakTracker(uint32_t now_ms,
                                                float detector_relative_angle_deg,
                                                float rate_dps);
  bool recordEnergyControlAutonomousPeak(uint32_t peak_ms, int8_t physical_side,
                                         float amplitude_deg, float detector_peak_angle_deg);
  void updateEnergyControlAutonomousAtZeroCross(uint32_t t_test_ms, float rate_dps,
                                                 float detector_before_deg,
                                                 float detector_after_deg,
                                                 float crossing_alpha,
                                                 float interpolated_time_ms);
  void updateEnergyControlAutonomousPulse(uint32_t now_ms);
  bool beginEnergyControlAutonomousPulse(uint32_t now_ms, uint32_t t_test_ms,
                                         int8_t direction, uint16_t pulse_width_ms);
  bool beginEnergyControlAutonomousStartKickPulse(uint32_t now_ms, int8_t direction);
  float energyControlPotentialJ(float amplitude_deg) const;
  float energyControlAutonomousFreeNextPeakAmplitude(float amplitude_deg) const;
  float energyControlAutonomousGainForSide(int8_t physical_side) const;
  void energyControlAutonomousCorrectionParameters(int8_t physical_side,
                                                    float* c_side_used_deg,
                                                    float* g_side_corrected_deg_per_mA_s) const;
  float energyControlAutonomousCorrectedPrediction(float free_next_peak_deg, int8_t physical_side,
                                                    float q_mA_s, float* correction_deg) const;
  bool qIdentRateInSupport(float abs_rate_dps) const;
  float qIdentRequiredWidthMs(float q_target_mA_s, float signed_i0_mA, int8_t direction) const;
  bool qIdentSolvePulse(float q_target_mA_s, float signed_i0_mA, int8_t direction,
                         float* required_width_ms, uint16_t* selected_integer_width_ms,
                         uint16_t* width_ms, float* q_effective_pred_mA_s) const;
  bool beginQIdentPulse(uint32_t now_ms, uint32_t t_test_ms, int8_t direction,
                        uint16_t pulse_width_ms);
  void logSampleIfDue();
  void logSampleNow();
  void finishRun();
  void stopMotor();
  void setSyncLed(bool on);
  uint32_t measurementTotalDurationMs() const;
  float accelPitchDeg(const ImuReading& r) const;
  float physicalRollCandidateDeg(const ImuReading& r) const;
  float pitchBiasFromGyroBias() const;
  int16_t centi(float value) const;
  int16_t milli(float value) const;
  int16_t betaScaled(float value) const;

  PsramLogger* logger_ = nullptr;
  ImuManager* imu_ = nullptr;
  Roller485Manager* roller_ = nullptr;
  ExperimentStatus status_;

  uint32_t boot_start_ms_ = 0;
  uint32_t calib_start_ms_ = 0;
  uint32_t settling_start_ms_ = 0;
  uint32_t run_start_ms_ = 0;
  uint32_t trial_start_ms_ = 0;
  uint32_t rest_start_ms_ = 0;
  uint64_t run_start_us_ = 0;
  uint32_t last_imu_update_us_ = 0;
  uint32_t last_log_us_ = 0;
  uint32_t static_rate_since_ms_ = 0;
  float display_zero_offset_deg_ = 0.0f;
  float target_roll_deg_ = 0.0f;
  // Separate from Current Roll / READY. A zero target is intentionally a
  // braking request and is logged INVALID_BRAKING_NOT_IDENTIFIED.
  float q1_shadow_target_peak_abs_deg_ = 0.0f;
  float q1_shadow_run_target_peak_abs_deg_ = 0.0f;
  bool e2_shadow_armed_ = false;
  uint32_t e2_shadow_release_detected_ms_ = 0;
  bool e2_shadow_gyro_integrator_ready_ = false;
  uint32_t e2_shadow_last_gyro_sample_us_ = 0;
  float e2_shadow_last_gyro_rate_dps_ = 0.0f;
  float e2_shadow_gyro_relative_deg_ = 0.0f;
  float e2_shadow_pending_static_anchor_abs_deg_ = NAN;
  float e2_shadow_static_anchor_abs_deg_ = NAN;
  int8_t e2_shadow_candidate_motion_sign_ = 0;
  uint8_t e2_shadow_reverse_samples_ = 0;
  float e2_shadow_candidate_gyro_relative_deg_ = 0.0f;
  float e2_shadow_candidate_accel_abs_diag_deg_ = NAN;
  uint32_t e2_shadow_candidate_peak_ms_ = 0;
  bool e2_shadow_last_peak_valid_ = false;
  int8_t e2_shadow_last_turn_side_ = 0;
  float e2_shadow_last_peak_gyro_relative_deg_ = NAN;
  uint16_t e2_shadow_turn_index_ = 0;
  // Q1 detects a zero-cross from the adopted-filter relative angle. The model
  // state itself is official +gy physical rate; the angle only detects crossing.
  bool q1_shadow_has_previous_angle_ = false;
  bool q1_shadow_zero_cross_armed_ = false;
  float q1_shadow_angle_zero_deg_ = 0.0f;
  float q1_shadow_previous_relative_angle_deg_ = 0.0f;
  uint32_t q1_shadow_previous_time_ms_ = 0;
  float q1_shadow_previous_rate_dps_ = 0.0f;
  uint32_t q1_shadow_last_zero_cross_ms_ = 0;
  // Q_IDENT state has no shared scheduler, target, or controller state.
  bool q_ident_mode_ = false;
  bool q_ident_armed_ = false;
  bool q_ident_pulse_authorized_ = false;
  uint8_t q_ident_run_schedule_id_ = 0;
  uint8_t q_ident_arm_consecutive_count_ = 0;
  int8_t q_ident_last_arm_side_ = 0;
  uint8_t q_ident_side_occurrence_count_[Config::Q_IDENT_SIDE_COUNT] = {};
  // Separate from Q_IDENT: only this V0 mode can authorize an actual pulse.
  bool energy_control_v0_mode_ = false;
  bool energy_control_v0_pulse_authorized_ = false;
  EnergyControlV0OutputGateState energy_control_v0_output_gate_state_ =
      EnergyControlV0OutputGateState::WAIT_INITIAL_EXCURSION;
  int8_t energy_control_v0_previous_accepted_next_peak_side_ = 0;
  bool energy_control_v0_rearm_excursion_seen_ = false;
  float energy_control_v0_max_abs_detector_excursion_deg_ = 0.0f;
  // Dedicated autonomous path.  None of these fields are shared with the
  // Q_IDENT schedule or the manual-release V0 output gate.
  bool energy_control_autonomous_mode_ = false;
  bool energy_control_autonomous_pulse_authorized_ = false;
  EnergyControlAutonomousPhase energy_control_autonomous_phase_ = EnergyControlAutonomousPhase::IDLE;
  EnergyControlAutonomousHalfCycleState energy_control_autonomous_half_cycle_state_ =
      EnergyControlAutonomousHalfCycleState::WAIT_PEAK;
  float energy_control_autonomous_target_peak_deg_ = Config::ENERGY_CONTROL_AUTONOMOUS_DEFAULT_TARGET_PEAK_DEG;
  float energy_control_autonomous_integral_plus_mA_s_ = 0.0f;
  float energy_control_autonomous_integral_minus_mA_s_ = 0.0f;
  bool energy_control_autonomous_gyro_integrator_ready_ = false;
  uint32_t energy_control_autonomous_last_gyro_sample_us_ = 0;
  float energy_control_autonomous_last_gyro_rate_dps_ = 0.0f;
  float energy_control_autonomous_gyro_relative_deg_ = 0.0f;
  // The adopted dynamic filter detects a zero crossing only. Its baseline is
  // fixed at RUN start and it is never used as the energy/peak coordinate.
  bool energy_control_autonomous_detector_has_previous_angle_ = false;
  float energy_control_autonomous_detector_zero_angle_deg_ = 0.0f;
  float energy_control_autonomous_previous_detector_relative_angle_deg_ = 0.0f;
  float energy_control_autonomous_previous_detector_rate_dps_ = 0.0f;
  uint32_t energy_control_autonomous_previous_detector_test_ms_ = 0;
  bool energy_control_autonomous_zero_cross_consumed_for_peak_ = false;
  bool energy_control_autonomous_last_accepted_zero_cross_valid_ = false;
  uint32_t energy_control_autonomous_last_accepted_zero_cross_ms_ = 0;
  bool energy_control_autonomous_peak_tracker_enabled_ = false;
  bool energy_control_autonomous_peak_tracker_started_ = false;
  int8_t energy_control_autonomous_candidate_detector_side_ = 0;
  uint8_t energy_control_autonomous_return_samples_ = 0;
  float energy_control_autonomous_candidate_detector_peak_abs_deg_ = 0.0f;
  float energy_control_autonomous_candidate_peak_amplitude_deg_ = 0.0f;
  uint32_t energy_control_autonomous_candidate_peak_ms_ = 0;
  bool energy_control_autonomous_last_peak_valid_ = false;
  float energy_control_autonomous_last_peak_amplitude_deg_ = NAN;
  int8_t energy_control_autonomous_last_peak_side_ = 0;
  uint32_t energy_control_autonomous_last_peak_ms_ = 0;
  bool energy_control_autonomous_pending_peak_ = false;
  int8_t energy_control_autonomous_pending_next_side_ = 0;
  float energy_control_autonomous_pending_q_command_mA_s_ = 0.0f;
  bool energy_control_autonomous_pending_saturated_upper_ = false;
  bool energy_control_autonomous_pending_saturated_lower_ = false;
  uint16_t energy_control_autonomous_pending_zero_event_index_ = 0;  uint32_t next_pulse_start_test_ms_ = 0;  uint32_t active_pulse_start_ms_ = 0;
  uint32_t active_pulse_start_test_ms_ = 0;
  bool passive_capture_mode_ = false;
  uint32_t last_pulse_end_ms_ = 0;
  int8_t next_pulse_direction_ = 1;
  bool zero_cross_mode_ = false;
  bool zero_cross_bootstrap_pending_ = false;
  bool zero_cross_armed_ = false;
  bool zero_cross_has_previous_angle_ = false;
  float zero_cross_previous_angle_deg_ = 0.0f;
  uint32_t last_zero_cross_pulse_start_ms_ = 0;
  uint32_t zero_cross_start_refractory_until_ms_ = 0;
  float zero_cross_half_cycle_peak_abs_deg_ = 0.0f;
  int8_t zero_cross_next_direction_ = -Config::ZERO_CROSS_BOOTSTRAP_DIRECTION;
  int16_t zero_cross_fixed_current_mA_ = Config::ZERO_CROSS_DEFAULT_FIXED_CURRENT_MA;
  uint16_t zero_cross_fixed_pulse_ms_ = Config::ZERO_CROSS_DEFAULT_FIXED_PULSE_MS;
  bool identification_mode_ = false;
  QRunMode q_run_mode_ = QRunMode::NONE;
  uint8_t q_probe_schedule_id_ = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A;
  float control_target_peak_deg_ = 0.0f;
  CalibrationPhase calibration_phase_ = CalibrationPhase::INACTIVE;
  bool calibration_enabled_ = false;
  bool calibration_peak_tracker_ready_ = false;
  // After a confirmed turn, candidate tracking stays disarmed until motion has
  // crossed into the opposite half-cycle and is moving outward again.
  int8_t calibration_expected_candidate_side_ = 0;
  int8_t calibration_outbound_rate_sign_ = 0;
  uint8_t calibration_reverse_samples_ = 0;
  float calibration_peak_candidate_deg_ = 0.0f;
  float calibration_peak_candidate_fixed_deg_ = 0.0f;
  uint32_t calibration_peak_candidate_ms_ = 0;
  bool calibration_last_peak_valid_ = false;
  int8_t calibration_last_peak_side_ = 0;
  float calibration_last_peak_abs_deg_ = 0.0f;
  float calibration_last_peak_signed_dynamic_deg_ = 0.0f;
  float calibration_last_peak_signed_fixed_deg_ = 0.0f;
  float calibration_last_half_range_dynamic_deg_ = 0.0f;
  float calibration_last_half_range_fixed_deg_ = 0.0f;
  bool calibration_last_half_range_valid_ = false;
  float calibration_last_center_dynamic_deg_ = 0.0f;
  bool calibration_last_center_dynamic_valid_ = false;
  uint32_t calibration_last_peak_candidate_ms_ = 0;
  CalibrationFit calibration_fit_pos_;
  CalibrationFit calibration_fit_neg_;
  HalfRangeFit calibration_half_range_dynamic_fit_;
  HalfRangeFit calibration_half_range_fixed_fit_;
  float calibration_free_last_half_range_dynamic_deg_ = 0.0f;
  float calibration_free_last_half_range_fixed_deg_ = 0.0f;
  bool calibration_free_last_half_range_valid_ = false;
  PsramLogger::CalibrationResult calibration_result_;
  float calibration_probe_previous_peak_abs_deg_ = 0.0f;
  float calibration_probe_previous_peak_signed_dynamic_deg_ = 0.0f;
  float calibration_probe_previous_peak_signed_fixed_deg_ = 0.0f;
  float calibration_probe_previous_half_range_dynamic_deg_ = 0.0f;
  float calibration_probe_previous_half_range_fixed_deg_ = 0.0f;
  bool calibration_probe_previous_half_range_valid_ = false;
  float calibration_probe_previous_center_dynamic_deg_ = 0.0f;
  bool calibration_probe_previous_center_dynamic_valid_ = false;
  uint32_t calibration_probe_previous_peak_candidate_ms_ = 0;
  float calibration_probe_q_target_mA_s_ = 0.0f;
  float calibration_probe_q_effective_pred_mA_s_ = 0.0f;
  float calibration_probe_command_dynamic_angle_deg_ = 0.0f;
  float calibration_probe_command_rate_raw_dps_ = 0.0f;
  float calibration_probe_command_rate_bias_corrected_dps_ = 0.0f;
  uint32_t calibration_probe_pulse_start_test_ms_ = 0;
  uint16_t calibration_probe_pulse_id_ = 0;
  int8_t calibration_probe_requested_side_ = 0;
  int8_t calibration_probe_predecessor_side_ = 0;
  int8_t calibration_probe_direction_ = 0;
  uint8_t calibration_probe_plan_index_ = 0;
  uint8_t calibration_probe_active_plan_index_ = 0;
  uint8_t calibration_probe_q_level_index_ = 0;
  uint8_t calibration_probe_rebuild_total_count_ = 0;
  uint8_t calibration_probe_rebuild_episode_id_ = 0;
  uint8_t calibration_probe_rebuild_attempt_in_episode_ = 0;
  // Audit how the predecessor was allowed to enter a Q probe: 0=legacy A
  // input-domain support, 1=dynamic-H support. V60 logs its per-run
  // free-decay-derived rebuild target separately in CalibrationResult.
  uint8_t calibration_probe_rebuild_entry_source_ = 0;
  bool calibration_probe_rebuild_target_reached_ = false;
  bool calibration_probe_dynamic_h_in_support_at_command_ = false;
  uint8_t calibration_probe_wait_halfcycle_count_ = 0;
  uint8_t calibration_probe_wait_halfcycle_total_ = 0;
  uint8_t calibration_probe_attempt_count_ = 0;
  uint8_t calibration_v59_gate_event_count_ = 0;
  uint8_t calibration_v59_gate_pass_count_ = 0;
  uint8_t calibration_v59_gate_skip_count_ = 0;
  uint8_t calibration_v59_rebuild_from_low_state_count_ = 0;
  uint8_t calibration_v60_cooldown_halfcycles_remaining_ = 0;
  bool calibration_v60_high_desired_wait_pending_ = false;
  // V61 always commands the side opposite to the first fixed-Q arrival, then
  // evaluates the two subsequently unforced peaks as an H/C state.
  bool calibration_v61_rebuild_plan_active_ = false;
  float calibration_v61_nominal_controlled_peak_target_deg_ = 0.0f;
  float calibration_v61_command_peak_target_deg_ = 0.0f;
  RateStateFit calibration_v62_rate_fit_;
  float calibration_gain_sum_pos_ = 0.0f;
  float calibration_gain_sum_neg_ = 0.0f;
  float calibration_gain_sum_sq_pos_ = 0.0f;
  float calibration_gain_sum_sq_neg_ = 0.0f;
  uint8_t calibration_initial_kick_count_ = 0;
  float calibration_initial_kick_q_effective_pred_mA_s_ = 0.0f;
  uint16_t calibration_rebuild_pulse_id_ = 0;
  uint32_t calibration_rebuild_pulse_start_test_ms_ = 0;
  uint8_t calibration_rebuild_total_count_ = 0;
  uint8_t calibration_rebuild_episode_id_ = 0;
  uint8_t calibration_rebuild_attempt_in_episode_ = 0;
  uint8_t identification_target_index_ = 0;
  uint16_t identification_event_id_ = 0;
  bool identification_peak_waiting_ = false;
  int8_t identification_outbound_rate_sign_ = 0;
  uint8_t identification_reverse_samples_ = 0;
  float identification_peak_angle_deg_ = 0.0f;
  float predicted_signed_current_end_mA_ = 0.0f;
  uint32_t predicted_current_end_ms_ = 0;
  uint8_t sync_step_ = 0;
  uint32_t sync_step_start_ms_ = 0;
  uint32_t sync_led_until_ms_ = 0;
  bool single_trial_mode_ = false;
  uint8_t selected_trial_index_ = 0;

  double bias_sum_x_ = 0.0;
  double bias_sum_y_ = 0.0;
  double bias_sum_z_ = 0.0;
  uint32_t bias_sample_count_ = 0;
  bool bias_ready_ = false;

  float beta_smooth_[Config::DYNAMIC_BETA_COUNT] = {};
  BetaPhaseController beta_phase_;
  BetaTurnFastController beta_turn_fast_;
  float raw_beta1_raw_pitch_deg_ = 0.0f;
  float raw_dynamic_raw_pitch_deg_[Config::DYNAMIC_BETA_COUNT] = {};
  float raw_beta1_bias_pitch_deg_ = 0.0f;
  float raw_dynamic_bias_pitch_deg_[Config::DYNAMIC_BETA_COUNT] = {};
  float raw_accel_pitch_deg_ = 0.0f;
  float offset_beta1_raw_deg_ = 0.0f;
  float offset_dynamic_raw_deg_[Config::DYNAMIC_BETA_COUNT] = {};
  float offset_beta1_bias_deg_ = 0.0f;
  float offset_dynamic_bias_deg_[Config::DYNAMIC_BETA_COUNT] = {};
  float offset_accel_deg_ = 0.0f;
  float gyro_raw_deg_ = 0.0f;
  float gyro_bias_corrected_deg_ = 0.0f;

  Adafruit_Madgwick filter_beta1_raw_;
  Adafruit_Madgwick filter_dynamic_raw_[Config::DYNAMIC_BETA_COUNT];
  Adafruit_Madgwick filter_beta1_bias_;
  Adafruit_Madgwick filter_dynamic_bias_[Config::DYNAMIC_BETA_COUNT];
};



