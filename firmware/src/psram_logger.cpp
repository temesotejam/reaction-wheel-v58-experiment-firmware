#include "psram_logger.h"
#include "current_timing_json.h"
#include "q_observer_json.h"

#include <string.h>

#include "config.h"

static constexpr uint16_t RWLOG_FORMAT_VERSION = 48;
static constexpr uint32_t RWLOG_FLAG_CRC32 = 1U << 0;
static constexpr size_t STREAM_CHUNK_BYTES = 4096;

namespace {
const char* e2ShadowInvalidReasonName(uint8_t reason) {
  switch (reason) {
    case Config::E2_SHADOW_INVALID_NONE: return "VALID";
    case Config::E2_SHADOW_INVALID_H_OUT_OF_CALIBRATION_RANGE:
      return "INVALID_OUT_OF_CALIBRATION_RANGE";
    case Config::E2_SHADOW_INVALID_BRAKING_NOT_IDENTIFIED:
      return "INVALID_BRAKING_NOT_IDENTIFIED";
    case Config::E2_SHADOW_INVALID_SIDE_MISMATCH: return "INVALID_SIDE_MISMATCH";
    case Config::E2_SHADOW_INVALID_Q_OUT_OF_CALIBRATION_RANGE:
      return "INVALID_OUT_OF_CALIBRATION_RANGE";
    case Config::E2_SHADOW_INVALID_ABSOLUTE_PEAK_ESTIMATOR_UNVALIDATED:
      return "INVALID_ABSOLUTE_PEAK_ESTIMATOR_UNVALIDATED";
    case Config::E2_SHADOW_INVALID_NONALTERNATING_TURN:
      return "INVALID_NONALTERNATING_TURN";
    case Config::E2_SHADOW_INVALID_INSUFFICIENT_TURNS:
      return "INVALID_INSUFFICIENT_TURNS";
    default: return "INVALID_NONFINITE_STATE";
  }
}
const char* q1ShadowInvalidReasonName(uint8_t reason) {
  switch (reason) {
    case Config::Q1_SHADOW_INVALID_NONE: return "VALID";
    case Config::Q1_SHADOW_INVALID_NONFINITE_STATE: return "INVALID_NONFINITE_STATE";
    case Config::Q1_SHADOW_INVALID_RATE_BELOW_SUPPORT: return "INVALID_RATE_BELOW_SUPPORT";
    case Config::Q1_SHADOW_INVALID_RATE_ABOVE_SUPPORT: return "INVALID_RATE_ABOVE_SUPPORT";
    case Config::Q1_SHADOW_INVALID_BRAKING_NOT_IDENTIFIED:
      return "INVALID_BRAKING_NOT_IDENTIFIED";
    case Config::Q1_SHADOW_INVALID_Q_BELOW_SUPPORT: return "INVALID_Q_BELOW_SUPPORT";
    case Config::Q1_SHADOW_INVALID_Q_ABOVE_SUPPORT: return "INVALID_Q_ABOVE_SUPPORT";
    default: return "INVALID_NONFINITE_STATE";
  }
}
const char* qIdentInvalidReasonName(uint8_t reason) {
  switch (reason) {
    case Config::Q_IDENT_INVALID_NONE: return "VALID";
    case Config::Q_IDENT_INVALID_ARM_WAITING: return "ARM_WAITING";
    case Config::Q_IDENT_INVALID_ARMED_EVENT_NO_OUTPUT: return "ARMED_EVENT_NO_OUTPUT";
    case Config::Q_IDENT_INVALID_RATE_BELOW_SUPPORT: return "INVALID_RATE_BELOW_SUPPORT";
    case Config::Q_IDENT_INVALID_RATE_ABOVE_SUPPORT: return "INVALID_RATE_ABOVE_SUPPORT";
    case Config::Q_IDENT_INVALID_SCHEDULE_EXHAUSTED: return "INVALID_SCHEDULE_EXHAUSTED";
    case Config::Q_IDENT_INVALID_Q_ZERO_NO_PULSE: return "VALID_Q_ZERO_NO_PULSE";
    case Config::Q_IDENT_INVALID_ROLLER_NOT_READY: return "INVALID_ROLLER_NOT_READY";
    case Config::Q_IDENT_INVALID_BATTERY_GUARD: return "INVALID_BATTERY_GUARD";
    case Config::Q_IDENT_INVALID_PULSE_WIDTH_GUARD: return "INVALID_PULSE_WIDTH_GUARD";
    case Config::Q_IDENT_INVALID_SOLVER_NONFINITE: return "INVALID_SOLVER_NONFINITE";
    case Config::Q_IDENT_INVALID_CURRENT_WRITE_FAILED: return "INVALID_CURRENT_WRITE_FAILED";
    case Config::Q_IDENT_INVALID_ESTOP_OR_STATE: return "INVALID_ESTOP_OR_STATE";
    default: return "INVALID_UNKNOWN";
  }
}
const char* energyControlV0ReasonName(uint8_t reason) {
  switch (reason) {
    case Config::ENERGY_CONTROL_V0_INVALID_NONE: return "VALID";
    case Config::ENERGY_CONTROL_V0_VALID_PASSIVE_NO_OUTPUT: return "VALID_PASSIVE_NO_OUTPUT";
    case Config::ENERGY_CONTROL_V0_INVALID_NONFINITE_STATE: return "INVALID_NONFINITE_STATE";
    case Config::ENERGY_CONTROL_V0_INVALID_POTENTIAL_DOMAIN: return "INVALID_POTENTIAL_DOMAIN";
    case Config::ENERGY_CONTROL_V0_INVALID_ROLLER_NOT_READY: return "INVALID_ROLLER_NOT_READY";
    case Config::ENERGY_CONTROL_V0_INVALID_BATTERY_GUARD: return "INVALID_BATTERY_GUARD";
    case Config::ENERGY_CONTROL_V0_INVALID_PULSE_WIDTH_GUARD: return "INVALID_PULSE_WIDTH_GUARD";
    case Config::ENERGY_CONTROL_V0_INVALID_CURRENT_WRITE_FAILED: return "INVALID_CURRENT_WRITE_FAILED";
    case Config::ENERGY_CONTROL_V0_INVALID_ESTOP_OR_STATE: return "INVALID_ESTOP_OR_STATE";
    case Config::ENERGY_CONTROL_V0_INVALID_WAIT_INITIAL_EXCURSION: return "WAIT_INITIAL_EXCURSION";
    case Config::ENERGY_CONTROL_V0_INVALID_WAIT_REARM_EXCURSION: return "WAIT_REARM_EXCURSION";
    case Config::ENERGY_CONTROL_V0_INVALID_NONALTERNATING_SIDE: return "NONALTERNATING_SIDE";
    case Config::ENERGY_CONTROL_V0_INVALID_EVENT_LOG_OVERFLOW: return "INVALID_EVENT_LOG_OVERFLOW";
    default: return "INVALID_UNKNOWN";
  }
}
const char* energyControlAutonomousReasonName(uint8_t reason) {
  switch (reason) {
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONE: return "VALID";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_VALID_NO_OUTPUT: return "VALID_NO_OUTPUT";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_STRONG_START_KICK_EXECUTED: return "STRONG_START_KICK_EXECUTED";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_EVENT_LOG_OVERFLOW: return "INVALID_EVENT_LOG_OVERFLOW";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONFINITE_STATE: return "INVALID_NONFINITE_STATE";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_POTENTIAL_DOMAIN: return "INVALID_POTENTIAL_DOMAIN";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_ROLLER_NOT_READY: return "INVALID_ROLLER_NOT_READY";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_BATTERY_GUARD: return "INVALID_BATTERY_GUARD";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_PULSE_WIDTH_GUARD: return "INVALID_PULSE_WIDTH_GUARD";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_CURRENT_WRITE_FAILED: return "INVALID_CURRENT_WRITE_FAILED";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_Q_BELOW_MIN_PULSE: return "VALID_Q_BELOW_MIN_PULSE";
    case Config::ENERGY_CONTROL_AUTONOMOUS_REASON_ESTOP_OR_STATE: return "INVALID_ESTOP_OR_STATE";
    default: return "INVALID_UNKNOWN";
  }
}
}  // namespace
bool PsramLogger::begin() {
  ready_ = false;
  if (!psramFound()) {
    last_error_ = "psram_not_found";
    return false;
  }

  sample_capacity_ = Config::LOG_BUFFER_BYTES / sizeof(LogSample);
  samples_ = static_cast<LogSample*>(ps_malloc(sample_capacity_ * sizeof(LogSample)));
  if (!samples_) {
    last_error_ = "psram_allocation_failed";
    return false;
  }

  clear();
  ready_ = true;
  last_error_ = "";
  return true;
}

void PsramLogger::clear() {
  if (downloading_) return;
  sample_count_ = 0;
  if (current_timing_audit_) current_timing_audit_->reset();
  if (q_observer_) q_observer_->reset(false);
  current_run_id_ = 0;
  run_start_us_ = 0;
  run_current_mA_ = 0;
  run_pulse_width_ms_ = 0;
  run_input_interval_ms_ = 0;
  identification_mode_ = false;
  q_run_mode_ = 0;
  q_probe_schedule_id_ = 0;
  control_target_cdeg_ = 0;
  passive_capture_ = false;
  q1_shadow_target_peak_abs_deg_ = NAN;
  q_ident_mode_ = false;
  q_ident_run_schedule_id_ = 0;
  energy_control_v0_mode_ = false;
  energy_control_autonomous_mode_ = false;
  identification_event_count_ = 0;
  calibration_peak_event_count_ = 0;
  calibration_probe_event_count_ = 0;
  calibration_state_gate_event_count_ = 0;
  calibration_build_up_event_count_ = 0;
  e2_shadow_peak_event_count_ = 0;
  e2_shadow_peak_event_overflow_ = false;
  q1_shadow_event_count_ = 0;
  q1_shadow_event_overflow_ = false;
  q_ident_event_count_ = 0;
  q_ident_event_overflow_ = false;
  energy_control_v0_event_count_ = 0;
  energy_control_v0_event_overflow_ = false;
  energy_control_autonomous_peak_event_count_ = 0;
  energy_control_autonomous_zero_cross_event_count_ = 0;
  energy_control_autonomous_event_overflow_ = false;
  calibration_result_ = CalibrationResult{};
  last_measurement_done_ = false;
}

void PsramLogger::startRun(uint16_t run_id, uint64_t run_start_us, int16_t current_mA, uint16_t pulse_width_ms,
                             uint16_t input_interval_ms, bool identification_mode,
                             uint8_t q_run_mode, int16_t control_target_cdeg,
                             uint8_t q_probe_schedule_id, bool passive_capture,
                             float q1_shadow_target_peak_abs_deg, bool q_ident_mode,
                             uint8_t q_ident_run_schedule_id, bool energy_control_v0_mode,
                             bool energy_control_autonomous_mode) {
  if (downloading_) return;
  sample_count_ = 0;
  current_run_id_ = run_id;
  if (current_timing_audit_) current_timing_audit_->reset();
  if (q_observer_) q_observer_->reset(true);
  run_start_us_ = run_start_us;
  run_current_mA_ = current_mA;
  run_pulse_width_ms_ = pulse_width_ms;
  run_input_interval_ms_ = input_interval_ms;
  identification_mode_ = identification_mode;
  q_run_mode_ = q_run_mode;
  q_probe_schedule_id_ = q_probe_schedule_id;
  control_target_cdeg_ = control_target_cdeg;
  passive_capture_ = passive_capture;
  q1_shadow_target_peak_abs_deg_ = q1_shadow_target_peak_abs_deg;
  q_ident_mode_ = q_ident_mode;
  q_ident_run_schedule_id_ = q_ident_run_schedule_id;
  energy_control_v0_mode_ = energy_control_v0_mode;
  energy_control_autonomous_mode_ = energy_control_autonomous_mode;
  identification_event_count_ = 0;
  calibration_peak_event_count_ = 0;
  calibration_probe_event_count_ = 0;
  calibration_state_gate_event_count_ = 0;
  calibration_build_up_event_count_ = 0;
  e2_shadow_peak_event_count_ = 0;
  e2_shadow_peak_event_overflow_ = false;
  q1_shadow_event_count_ = 0;
  q1_shadow_event_overflow_ = false;
  q_ident_event_count_ = 0;
  q_ident_event_overflow_ = false;
  energy_control_v0_event_count_ = 0;
  energy_control_v0_event_overflow_ = false;
  energy_control_autonomous_peak_event_count_ = 0;
  energy_control_autonomous_zero_cross_event_count_ = 0;
  energy_control_autonomous_event_overflow_ = false;
  calibration_result_ = CalibrationResult{};
  last_measurement_done_ = false;
}

uint16_t PsramLogger::beginIdentificationEvent(const IdentificationEvent& event) {
  if (!identification_mode_ || identification_event_count_ >= kMaxIdentificationEvents) return 0;
  IdentificationEvent stored = event;
  stored.event_id = identification_event_count_ + 1;
  identification_events_[identification_event_count_++] = stored;
  return stored.event_id;
}

void PsramLogger::finishIdentificationEvent(uint16_t event_id, uint32_t peak_ms, int16_t theta_peak_cdeg) {
  if (event_id == 0 || event_id > identification_event_count_) return;
  IdentificationEvent& event = identification_events_[event_id - 1];
  event.peak_ms = peak_ms;
  event.theta_peak_cdeg = theta_peak_cdeg;
  event.peak_detected = true;
}

void PsramLogger::addCalibrationPeakEvent(const CalibrationPeakEvent& event) {
  if (calibration_peak_event_count_ >= kMaxCalibrationPeakEvents) return;
  CalibrationPeakEvent stored = event;
  stored.peak_index = calibration_peak_event_count_ + 1;
  calibration_peak_events_[calibration_peak_event_count_++] = stored;
}

void PsramLogger::addCalibrationProbeEvent(const CalibrationProbeEvent& event) {
  if (calibration_probe_event_count_ >= kMaxCalibrationProbeEvents) return;
  CalibrationProbeEvent stored = event;
  stored.probe_index = calibration_probe_event_count_ + 1;
  calibration_probe_events_[calibration_probe_event_count_++] = stored;
}

void PsramLogger::addCalibrationStateGateEvent(const CalibrationStateGateEvent& event) {
  if (calibration_state_gate_event_count_ >= kMaxCalibrationStateGateEvents) return;
  CalibrationStateGateEvent stored = event;
  stored.event_index = calibration_state_gate_event_count_ + 1;
  calibration_state_gate_events_[calibration_state_gate_event_count_++] = stored;
}

void PsramLogger::addCalibrationBuildUpEvent(const CalibrationBuildUpEvent& event) {
  if (calibration_build_up_event_count_ >= kMaxCalibrationBuildUpEvents) return;
  CalibrationBuildUpEvent stored = event;
  stored.pulse_index = calibration_build_up_event_count_ + 1;
  calibration_build_up_events_[calibration_build_up_event_count_++] = stored;
}

void PsramLogger::addE2ShadowPeakEvent(const E2ShadowPeakEvent& event) {
  if (e2_shadow_peak_event_count_ >= kMaxE2ShadowPeakEvents) {
    e2_shadow_peak_event_overflow_ = true;
    return;
  }
  E2ShadowPeakEvent stored = event;
  stored.event_index = e2_shadow_peak_event_count_ + 1;
  e2_shadow_peak_events_[e2_shadow_peak_event_count_++] = stored;
}

void PsramLogger::addQ1ShadowEvent(const Q1ShadowEvent& event) {
  if (q1_shadow_event_count_ >= kMaxQ1ShadowEvents) {
    q1_shadow_event_overflow_ = true;
    return;
  }
  Q1ShadowEvent stored = event;
  stored.q1_shadow_event_index = q1_shadow_event_count_ + 1;
  q1_shadow_events_[q1_shadow_event_count_++] = stored;
}
void PsramLogger::addQIdentEvent(const QIdentEvent& event) {
  if (q_ident_event_count_ >= kMaxQIdentEvents) {
    q_ident_event_overflow_ = true;
    return;
  }
  QIdentEvent stored = event;
  stored.q_ident_event_index = q_ident_event_count_ + 1;
  q_ident_events_[q_ident_event_count_++] = stored;
}
void PsramLogger::addEnergyControlV0Event(const EnergyControlV0Event& event) {
  if (energy_control_v0_event_count_ >= kMaxEnergyControlV0Events) {
    energy_control_v0_event_overflow_ = true;
    return;
  }
  EnergyControlV0Event stored = event;
  stored.event_index = energy_control_v0_event_count_ + 1;
  energy_control_v0_events_[energy_control_v0_event_count_++] = stored;
}
void PsramLogger::addEnergyControlAutonomousPeakEvent(const EnergyControlAutonomousPeakEvent& event) {
  if (energy_control_autonomous_peak_event_count_ >= kMaxEnergyControlAutonomousEvents) {
    energy_control_autonomous_event_overflow_ = true;
    return;
  }
  EnergyControlAutonomousPeakEvent stored = event;
  stored.peak_index = energy_control_autonomous_peak_event_count_ + 1;
  energy_control_autonomous_peak_events_[energy_control_autonomous_peak_event_count_++] = stored;
}
void PsramLogger::addEnergyControlAutonomousZeroCrossEvent(const EnergyControlAutonomousZeroCrossEvent& event) {
  if (energy_control_autonomous_zero_cross_event_count_ >= kMaxEnergyControlAutonomousEvents) {
    energy_control_autonomous_event_overflow_ = true;
    return;
  }
  EnergyControlAutonomousZeroCrossEvent stored = event;
  stored.event_index = energy_control_autonomous_zero_cross_event_count_ + 1;
  energy_control_autonomous_zero_cross_events_[energy_control_autonomous_zero_cross_event_count_++] = stored;
}
void PsramLogger::setCalibrationResult(const CalibrationResult& result) {
  calibration_result_ = result;
}
void PsramLogger::markMeasurementDone() {
  if (q_observer_) q_observer_->finish(micros());
  last_measurement_done_ = true;
}

bool PsramLogger::addSample(const LogSample& row) {
  if (!ready_ || downloading_ || sample_count_ >= sample_capacity_) return false;
  samples_[sample_count_++] = row;
  return true;
}

uint8_t PsramLogger::usagePercent() const {
  if (sample_capacity_ == 0) return 0;
  return static_cast<uint8_t>((sample_count_ * 100ULL) / sample_capacity_);
}

bool PsramLogger::warningLevel() const {
  return usagePercent() >= Config::BUFFER_WARNING_PERCENT;
}

bool PsramLogger::rwlogDownloadable() const {
  return ready_ && run_start_us_ != 0 && sample_count_ > 0;
}

void PsramLogger::downloadFilename(char* out, size_t out_len) const {
  if (!out || out_len == 0) return;
  const char* prefix = q_observer_ && q_observer_->v57.enabled ? "paired_probe_v58_run" : q_observer_ && q_observer_->v55.enabled ? "wheel_state_probe_v55_run" : energy_control_autonomous_mode_ ? "energy_control_autonomous_run" :
      (energy_control_v0_mode_ ? "energy_control_v0_run" :
      (q_ident_mode_ ? "q_ident_fixed_schedule_run" :
       (passive_capture_ ? "passive_absolute_roll_run" : "dynamic_beta_holdcompare_zerocross_run")));
  snprintf(out, out_len, "%s_%u_%llu.rwlog", prefix, current_run_id_,
           static_cast<unsigned long long>(run_start_us_));
}

size_t PsramLogger::psramTotal() const {
  return ESP.getPsramSize();
}

size_t PsramLogger::psramFree() const {
  return ESP.getFreePsram();
}

String PsramLogger::buildMetadataJson() const {
  // V57 has no autonomous-control event detail. Its required metadata is the
  // per-trial fixed-probe record plus references to the raw-current event
  // block. The legacy 2 MiB reservation below is for old V0.2 event logs and
  // exceeds the contiguous heap available after the V57 raw-current buffer is
  // allocated. Build a bounded V57 document before entering that legacy path.
  if (q_observer_ && q_observer_->v57.enabled) {
    static constexpr size_t kV57MetadataJsonReserveBytes = 256U * 1024U;
    String json;
    if (!json.reserve(kV57MetadataJsonReserveBytes)) {
      return String("{\"format\":\"rwlog_paired_probe_v58\",\"measurement_mode\":\"V58_PAIRED_PROBE_COAST\",\"metadata_error\":\"v58_metadata_reserve_failed\",\"v58_metadata_reservation_bytes\":262144}");
    }
    json += "{\"format\":\"rwlog_paired_probe_v58\",";
    json += "\"firmware_revision\":\"" + String(Config::PASSIVE_CAPTURE_FIRMWARE_REVISION) + "\",";
    json += "\"measurement_mode\":\"V58_PAIRED_PROBE_COAST\",";
    json += "\"v58_metadata_reservation_bytes\":" + String(kV57MetadataJsonReserveBytes) + ",";
    json += "\"v58_metadata_complete\":true";
    appendQObserverJson(json, *q_observer_);
    const String final_size_key = ",\"metadata_json_final_bytes\":";
    size_t final_size = json.length() + final_size_key.length() + 2U;
    for (uint8_t i = 0; i < 4; ++i) {
      const size_t candidate_size = json.length() + final_size_key.length() +
          String(final_size).length() + 1U;
      if (candidate_size == final_size) break;
      final_size = candidate_size;
    }
    json += final_size_key + String(final_size) + "}";
    return json;
  }
  // v40 has up to 96 detailed Q events plus calibration peaks. Reserve the
  // complete JSON up front: growing an 11 kB String past about 70 kB during
  // download corrupted the first v40 metadata payload.
  // V0.2 can hold 128 Q1 candidates and 128 detailed V0 events concurrently.
  // The worst-case renderer is exercised by test_energy_control_v01_protocol.py
  // and must remain below 80% of this reservation. Reserving once avoids the
  // historic String-growth corruption observed during RWLOG download.
  static constexpr size_t kMetadataJsonReserveBytes = 2U * 1024U * 1024U;
  // Preserve headroom above the measured V4 602,388-byte metadata and never
  // let detailed Autonomous events force a String reallocation.
  static constexpr size_t kMetadataJsonHardBudgetBytes = 8256U * 1024U;
  static constexpr size_t kMetadataJsonTailReserveBytes = 320U * 1024U;
  String json;
  if (!json.reserve(kMetadataJsonReserveBytes)) {
    return String("{\"metadata_error\":\"reserve_failed\"}");
  }
  json += "{";
  json += "\"format\":\"" + String(q_observer_ && q_observer_->v57.enabled ? "rwlog_paired_probe_v58" : q_observer_ && q_observer_->v55.enabled ? "rwlog_wheel_state_probe_v55" : energy_control_autonomous_mode_ ? "rwlog_energy_control_autonomous" :
      (energy_control_v0_mode_ ? "rwlog_energy_control_v0" :
      (q_ident_mode_ ? "rwlog_q_ident_fixed_schedule" :
       (passive_capture_ ? "rwlog_passive_absolute_roll_free_decay" : "rwlog_dynamic_beta_vbat_hold_time_compare")))) + "\",";
  json += "\"firmware_revision\":\"" + String(Config::PASSIVE_CAPTURE_FIRMWARE_REVISION) + "\",";
  json += "\"measurement_mode\":\"" + String(q_observer_ && q_observer_->v57.enabled ? "V58_PAIRED_PROBE_COAST" : q_observer_ && q_observer_->v55.enabled ? "WHEEL_STATE_PROBE" : energy_control_autonomous_mode_ ? Config::ENERGY_CONTROL_AUTONOMOUS_MEASUREMENT_MODE :
      (energy_control_v0_mode_ ? Config::ENERGY_CONTROL_V0_MEASUREMENT_MODE :
      (q_ident_mode_ ? Config::Q_IDENT_MEASUREMENT_MODE :
       (passive_capture_ ? "manual_passive_free_decay" : "legacy")))) + "\",";
  json += "\"passive_capture_mode\":" + String(passive_capture_ ? "true" : "false") + ",";
  json += "\"q_ident_mode\":" + String(q_ident_mode_ ? "true" : "false") + ",";
  json += "\"q_ident_run_schedule_id\":" + String(q_ident_run_schedule_id_) + ",";
  json += "\"q_ident_fixed_schedule\":" + String(Config::Q_IDENT_FIXED_SCHEDULE ? "true" : "false") + ",";
  json += "\"q_ident_inverse_q_enabled\":" + String(Config::Q_IDENT_INVERSE_Q_ENABLED ? "true" : "false") + ",";
  json += "\"q_ident_arm_rule\":\"two_consecutive_alternating_zero_crosses_abs_rate_ge_30dps\",";
  json += "\"q_ident_rate_support_dps\":\"1.68-27.80 inclusive\",";
  json += "\"q_ident_battery_guard_mV\":\"6180-8100 inclusive\",";
  json += "\"q_ident_q_axis\":\"q_target_mA_s\",";
  json += "\"q_ident_q_levels_mA_s\":\"0,0.454,0.786,0.900\",";
  json += "\"q_ident_command_direction_rule\":\"-physical_next_peak_side\",";
  json += "\"q_ident_direction_status\":\"PREVIOUS_DATA_SUPPORTED_NOT_YET_VERIFIED_IN_Q_IDENT_PATH\",";
  json += "\"q_ident_output_policy\":\"frozen_no_actual_output_in_energy_control_v0_firmware\",";
  json += "\"energy_control_v0_policy\":\"sole_actual_output_path;frozen_q1_passive_plus_side_gain_and_p1_step_energy;Q_IDENT_and_all_legacy_paths_fail_closed\",";
  json += "\"energy_control_v0_revision\":\"V0.2\",";
  json += "\"energy_control_v0_output_gate\":\"independent_from_Q1_detector;WAIT_INITIAL_EXCURSION->ARMED_FOR_ZERO_CROSS->WAIT_OPPOSITE_EXCURSION\",";
  json += "\"energy_control_v0_rearm_angle_coordinate\":\"q1_detector_relative_angle_deg;continuous_detector_coordinate_only;physical_side_is_sign_of_plus_gy\",";
  json += "\"energy_control_v0_rearm_threshold_deg\":" +
      String(Config::ENERGY_CONTROL_V0_REARM_EXCURSION_DEG, 3) + ",";
  json += "\"energy_control_v0_gate_state_codes\":\"0=WAIT_INITIAL_EXCURSION,1=ARMED_FOR_ZERO_CROSS,2=WAIT_OPPOSITE_EXCURSION\",";
  json += "\"energy_control_v0_gate_block_reason_codes\":\"9=WAIT_INITIAL_EXCURSION,10=WAIT_REARM_EXCURSION,11=NONALTERNATING_SIDE,12=INVALID_EVENT_LOG_OVERFLOW\",";
  json += "\"energy_control_v0_event_log_policy\":\"V0_output_fail_closed_when_Q1_or_V0_event_capacity_reached\",";
  json += "\"energy_control_v0_target_peak_abs_deg\":" + String(Config::ENERGY_CONTROL_V0_TARGET_PEAK_DEG, 3) + ",";
  json += "\"energy_control_v0_q_search_mA_s\":\"0.000-0.900 step 0.001\",";
  json += "\"energy_control_v0_q_support_policy\":\"diagnostic_only_not_an_output_gate\",";
  json += "\"energy_control_v0_command_direction_rule\":\"-physical_next_peak_side\",";
  json += "\"energy_control_v0_direction_status\":\"PREVIOUS_DATA_SUPPORTED_NOT_YET_VERIFIED_IN_V0_PATH\",";
  json += "\"energy_control_v0_potential\":\"P1_STEP_gap;R=0.150m;inner_edge=0.005m;outer_edge=0.045m;cg_height=0.120m;mass=0.1997kg;g=9.80665m/s2\",";
  json += "\"energy_control_v0_potential_role\":\"geometry_potential_only;not_a_J_eff_or_dissipation_fit\",";
  json += "\"passive_motor_command_policy\":\"passive_capture_only:all_recorded_samples_command_0mA_and_output_off\",";
  json += "\"energy_control_v0_output_policy\":\"sole_actual_output_path;accepted_zero_cross_only;no_braking;all_other_paths_fail_closed\",";
  json += "\"energy_control_autonomous_policy\":\"independent_actual_output_path;one_300mA_100ms_start_kick_then_direct_P1_Q1_energy_control;Q_IDENT_E2_and_legacy_paths_fail_closed\",";
  json += "\"energy_control_autonomous_revision\":\"V7\",";
  json += "\"energy_control_autonomous_base_revision\":\"V6\",";
  json += "\"normal_excitation_direction_policy\":\"same_as_zero_cross_roll_rate\",";
  json += "\"normal_direction_sign_fix\":true,";
  json += "\"start_kick_direction_changed\":false,";
  json += "\"side_response_correction_enabled\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_CORRECTION_ENABLED ? "true" : "false") + ",";
  json += "\"side_response_correction_source\":\"" + String(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_CORRECTION_SOURCE) + "\",";
  json += "\"side_response_correction_model\":\"A_pred=A_free+c_side+g_side*Q;runtime_fit_coordinate=firmware_scaled_gyro_peak\",";
  json += "\"side_response_correction_blend_lambda\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_BLEND_LAMBDA, 3) + ",";
  json += "\"side_response_correction_max_abs_deg\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_MAX_ABS_DEG, 3) + ",";
  json += "\"side_response_correction_fit_c_plus_deg\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_C_PLUS_DEG, 6) + ",";
  json += "\"side_response_correction_fit_g_plus_deg_per_mA_s\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_G_PLUS_DEG_PER_MAS, 6) + ",";
  json += "\"side_response_correction_fit_c_minus_deg\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_C_MINUS_DEG, 6) + ",";
  json += "\"side_response_correction_fit_g_minus_deg_per_mA_s\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_G_MINUS_DEG_PER_MAS, 6) + ",";
  json += "\"side_response_correction_base_P1_unchanged\":true,";
  json += "\"side_response_correction_base_Q1_gains_unchanged\":true,";
  json += "\"side_response_correction_Ki_unchanged\":true,";
  json += "\"side_response_correction_direction_policy_unchanged\":true,";
  json += "\"energy_control_autonomous_state_machine\":\"IDLE->STRONG_START_KICK->WAIT_FIRST_PEAK->ENERGY_CONTROL->HOLD;physical_halfcycle=WAIT_PEAK->WAIT_ZERO_CROSS->PULSE_ACTIVE->WAIT_PEAK;STOP_only_abnormal\",";
  json += "\"physical_event_policy\":\"peak_zero_pulse_halfcycle_v5\",";
  json += "\"min_half_cycle_ms\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_MIN_HALF_CYCLE_MS) + ",";
  json += "\"min_zero_to_peak_ms\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_MIN_ZERO_TO_PEAK_MS) + ",";
  json += "\"events_accepted_during_pulse\":false,";
  json += "\"autonomous_duration_ms\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_DURATION_MS) + ",";
  json += "\"energy_control_autonomous_free_model\":\"F_s(A)=U_P1_inverse(0.8706716644111074*U_P1(A)-0J);A_is_nonnegative_scaled_gyro_absolute_peak_deg;F_plus_equals_F_minus;P1_20260828\",";
  json += "\"energy_control_autonomous_peak_policy\":\"continuous_adopted_angle_extremum_plus_3_returning_rate_samples;gyro_sign_change_alone_never_generates_peak;candidate_events_during_pulse_never_accepted;no_peak_rate_or_amplitude_minimum\",";
  json += "\"energy_control_autonomous_target_peak_deg\":" + String(control_target_cdeg_ / 100.0f, 3) + ",";
  json += "\"energy_control_autonomous_target_choices_deg\":\"8.0,10.0,12.0\",";
  json += "\"energy_control_autonomous_start_kick\":\"startup_only;300mA;100ms;command_direction=-1\",";
  json += "\"energy_control_autonomous_startup_pump\":false,";
  json += "\"energy_control_autonomous_peak_coordinate\":\"A=abs(0.908911*integral(physical_roll_rate_dps dt));measurement_start_reference;detector_extremum_qualifies_peak\",";
  json += "\"energy_control_autonomous_zero_cross_detector\":\"pitch_dynamic_beta_deg_at_FILTER_ADOPTED_INDEX_relative_to_RUN_start_baseline;detector_only_not_energy_coordinate;one_consumed_cross_per_accepted_peak\",";
  json += "\"energy_control_autonomous_side_policy\":\"next_side_from_interpolated_rate;peak_side_mismatch_logged_diagnostic_only\",";
  json += "\"energy_control_autonomous_integral_enable_rule\":\"every_accepted_peak;per_physical_peak_side;100ms_available_Q_antiwindup\",";
  json += "\"energy_control_autonomous_integral_Ki_mA_s_per_deg\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_INTEGRAL_KI_MAS_PER_DEG, 3) + ",";
  json += "\"energy_control_autonomous_normal_current_mA\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA) + ",";
  json += "\"energy_control_autonomous_normal_width_min_ms\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_MIN_PULSE_MS) + ",";
  json += "\"energy_control_autonomous_normal_width_max_ms\":" + String(Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS) + ",";
  json += "\"autonomous_uses_q_ident_limits\":false,";
  json += "\"energy_control_autonomous_q_policy\":\"direct_width_search_0_to_100ms;Q_available_from_current_rise_model;Q_GAIN_EXTRAPOLATED_logged_not_blocked;no_braking\",";
  json += "\"energy_control_autonomous_event_overflow\":" + String(energy_control_autonomous_event_overflow_ ? "true" : "false") + ",";
  json += "\"energy_control_autonomous_event_overflow_policy\":\"diagnostic_only;no_control_stop\",";
  json += "\"q1_shadow_enabled\":true,";
  json += "\"q1_model_name\":\"" + String(Config::Q1_SHADOW_MODEL_NAME) + "\",";
  json += "\"q_model_axis_type\":\"" + String(Config::Q1_SHADOW_Q_MODEL_AXIS_TYPE) + "\",";
  json += "\"q_model_axis_field\":\"" + String(Config::Q1_SHADOW_Q_MODEL_AXIS_FIELD) + "\",";
  json += "\"q1_formula\":\"A_next_abs=0.01304+0.08812*abs(physical_roll_rate_dps)+0.11858*physical_next_peak_side+g_side*q_target_mA_s\",";
  json += "\"q1_intercept\":" + String(Config::Q1_SHADOW_INTERCEPT_DEG, 5) + ",";
  json += "\"q1_rate_gain\":" + String(Config::Q1_SHADOW_RATE_GAIN_DEG_PER_DPS, 5) + ",";
  json += "\"q1_side_term\":" + String(Config::Q1_SHADOW_SIDE_TERM_DEG, 5) + ",";
  json += "\"q1_gain_physical_plus\":" + String(Config::Q1_SHADOW_GAIN_PHYSICAL_PLUS_DEG_PER_MAS, 5) + ",";
  json += "\"q1_gain_physical_minus\":" + String(Config::Q1_SHADOW_GAIN_PHYSICAL_MINUS_DEG_PER_MAS, 5) + ",";
  json += "\"q1_q_support_mA_s\":\"0.454-1.197\",";
  json += "\"q1_rate_support_dps\":\"1.68-27.80\",";
  json += "\"q1_zero_cross_angle_detector\":\"adopted_filter_angle_relative_to_measurement_start;detector_only\",";
  json += "\"q1_zero_cross_rate_axis\":\"physical_roll_rate_dps=+gy_dps-startup_y_bias\",";
  json += "\"q1_physical_next_peak_side_rule\":\"sign(physical_roll_rate_dps) after accepted zero-cross;positive_rate_means_positive_physical_next_peak\",";
  json += "\"q1_policy\":\"baseline_is_Q_equals_0;inverse_Q_augmentation_only;nonpositive_delta_is_braking_invalid;never_clip_support\",";
  json += "\"q1_motor_connected\":false,";
  json += "\"q1_target_next_peak_abs_deg\":";
  if (isfinite(q1_shadow_target_peak_abs_deg_)) json += String(q1_shadow_target_peak_abs_deg_, 4);
  else json += "null";
  json += ",";
  json += "\"q1_shadow_event_overflow\":" + String(q1_shadow_event_overflow_ ? "true" : "false") + ",";
  json += "\"q_ident_event_overflow\":" + String(q_ident_event_overflow_ ? "true" : "false") + ",";
  json += "\"energy_control_v0_event_overflow\":" + String(energy_control_v0_event_overflow_ ? "true" : "false") + ",";
  json += "\"e2_shadow_enabled\":false,";
  json += "\"e2_shadow_role\":\"offline_diagnostic_only;not_called_by_Q1_or_its_validity_logic\",";
  json += "\"angle_reference_policy\":\"imu_gravity_frame_continuous_without_per_run_zero_subtraction;video_must_use_body_line_minus_fixed_horizon\",";
  json += "\"physical_roll_axis\":\"IMU_y_axis; positive physical rate is gy_dps after startup y-bias subtraction\",";
  json += "\"physical_roll_candidate_formula\":\"atan2(ax_g,hypot(ay_g,az_g))*180/pi\",";
  json += "\"physical_roll_abs_formula\":\"0.9278941864271074*candidate_deg-0.49830848087090135\",";
  json += "\"physical_roll_confidence_policy\":\"accelerometer-derived angle is static-only; STATIC=NO suppresses static confidence during motion\",";
  json += "\"physical_roll_rate_formula\":\"gy_dps-startup_gyro_bias_y_dps\",";
  json += "\"static_rate_threshold_dps\":" + String(Config::STATIC_RATE_THRESHOLD_DPS, 3) + ",";
  json += "\"static_hold_time_ms\":" + String(Config::STATIC_HOLD_TIME_MS) + ",";
  json += "\"target_tolerance_deg\":" + String(Config::TARGET_TOLERANCE_DEG, 3) + ",";
  json += "\"current_roll_zero_policy\":\"display_only_current_roll=physical_roll_abs-display_zero_offset;never_changes_raw_imu_physical_abs_video_horizon_or_model_b_teacher\",";
  json += "\"target_ready_policy\":\"display_only_READY=STATIC_and_abs(current_roll-target_roll)<=target_tolerance;never_changes_motor_current_pulse_Q_or_control\",";
  json += "\"current_roll_ui_revision\":\"" + String(Config::CURRENT_ROLL_UI_REVISION) + "\",";
  json += "\"passive_static_window_t_test_ms\":\"0-" + String(Config::PASSIVE_STATIC_WINDOW_MS) + "\",";
  json += "\"v62_base_commit\":\"" + String(Config::V62_BASE_COMMIT) + "\",";
  json += "\"resolved_m5unified_version\":\"" + String(Config::RESOLVED_M5UNIFIED_VERSION) + "\",";
  json += "\"resolved_m5gfx_version\":\"" + String(Config::RESOLVED_M5GFX_VERSION) + "\",";
  json += "\"resolved_adafruit_ahrs_version\":\"" + String(Config::RESOLVED_ADAFRUIT_AHRS_VERSION) + "\",";
  json += "\"actual_current_audit_revision\":\"v49_forced_reads_strict_tail_end_age_violation_diagnostics_observation_only\",";
  json += "\"actual_current_audit_policy\":\"CURRENT_READBACK_timestamp_sequence_validity_and_observed_Q_are_diagnostic_only;they_never_select_stop_extend_or_change_motor_output\",";
  json += "\"roller_current_full_status_period_ms\":" + String(Config::ROLLER_READ_PERIOD_MS) + ",";
  json += "\"roller_current_fast_read_period_us\":" + String(Config::CURRENT_AUDIT_FAST_READ_PERIOD_US) + ",";
  json += "\"roller_current_max_gap_us\":" + String(Config::CURRENT_AUDIT_MAX_GAP_US) + ",";
  json += "\"roller_current_tail_window_us\":" + String(Config::CURRENT_AUDIT_TAIL_WINDOW_US) + ",";
  json += "\"roller_current_imu_guard_budget_us\":null,";
  json += "\"roller_current_fast_read_scope\":\"during_nonzero_command_only;CURRENT_READBACK_precedes_safety_and_the_other_five_full_status_registers\",";
  json += "\"roller_current_sample_time_us_semantics\":\"micros_modulo_2pow32_at_successful_CURRENT_READBACK_completion;zero_means_no_fresh_sample_since_pulse_start\",";
  json += "\"roller_current_sequence_semantics\":\"monotonic_CURRENT_READBACK_attempt_counter;increments_on_success_and_failure\",";
  json += "\"roller_current_on_device_gap_semantics\":\"gap_us_is_measured_between_every_adjacent_successful_CURRENT_READBACK_inside_each_pulse;it_is_not_reconstructed_from_RWLOG_rows_or_sequence_deltas\",";
  json += "\"roller_current_on_device_tail_semantics\":\"actual_zero_write_completion_end;full_gaps_overlapping_inclusive_last_10000us;coverage_requires_pre_tail_sample_and_retained_boundary_history;GO_also_requires_end_age_le3333\",";
  json += "\"roller_current_valid_semantics\":\"true_only_if_the_most_recent_CURRENT_READBACK_attempt_succeeded\",";
  json += "\"roller_current_age_us_semantics\":\"time_since_last_successful_CURRENT_READBACK;4294967295_means_unknown\",";
  json += "\"roller_q_meas_observed_semantics\":\"absolute_current_trapezoid_over_adjacent_fresh_samples_inside_active_pulse_only;edge_intervals_are_not_estimated;diagnostic_not_total_physical_Q\",";
  json += "\"roller_q_meas_observed_valid_semantics\":\"at_least_two_fresh_active_pulse_samples_and_no_fast_current_read_failure_in_that_pulse;does_not_authorize_control\",";
  json += "\"pulse_q_target_pred_semantics\":\"normal_V7_selected_q_command_and_q_effective_pred_only;START_KICK_is_null\",";
  json += "\"execution_timing_audit_revision\":\"v49_completed_loop_snapshot_plus_run_pulse_and_violation_metadata\",";
  json += "\"execution_timing_audit_policy\":\"diagnostic_only;timing_values_never_select_or_change_current_pulse_Q_target_safety_or_control\",";
  json += "\"mode_a_observation_scheduling_revision\":\"" + String(Config::MODE_A_OBSERVATION_SCHEDULING_REVISION) + "\",";
  json += "\"mode_a_pulse_current_priority\":" + String(Config::MODE_A_PULSE_CURRENT_PRIORITY_ENABLED ? "true" : "false") + ",";
  json += "\"mode_a_web_policy\":\"WebServer_handleClient_deferred_while_nonzero_motor_command_active\",";
  json += "\"mode_a_roller_order\":\"forced_current_then_safety_then_due_IMU_then_forced_current;forced_current_then_safety_then_due_full_status_then_forced_current\",";
  json += "\"autonomous_comparison_madgwick_policy\":\"nonadopted_series_held_during_autonomous_run;adopted_dynamic_hold073_continues\",";
  json += "\"execution_timing_audit_snapshot_semantics\":\"each_LogSample_carries_the_previous_fully_completed_main_loop;durations_are_micros_deltas;start_times_are_micros_modulo_2pow32;zero_duration_or_sequence_means_task_not_executed\",";
  json += "\"execution_timing_audit_tasks\":\"main_loop,M5.update,runner.serviceFast,beta_context,ImuManager,Roller485,ExperimentRunner,WebUi,IMU_hardware_get_data_and_filters,runner_filter_components,fast_current_read,full_status_read,log_sample\",";
  json += "\"format_version\":" + String(RWLOG_FORMAT_VERSION) + ",";
  json += "\"calibration_algorithm_revision\":\"v61_state_feedback_rebuild_fixed_q_v51_video_hfree\",";
  json += "\"shadow_forward_model_version\":\"" + String(Config::ZERO_CROSS_V57_FORWARD_MODEL_VERSION) + "\",";
  json += "\"shadow_model_version\":\"" + String(Config::ZERO_CROSS_V57_INVERSE_SHADOW_VERSION) + "\",";
  json += "\"shadow_reachability_audit_version\":\"" + String(Config::ZERO_CROSS_V54_REACHABILITY_AUDIT_VERSION) + "\",";
  json += "\"shadow_model_policy\":\"log_only_never_used_for_q_selection_pulse_width_or_motor_command\",";
  json += "\"shadow_v57_policy\":\"support_aware_inverse_shadow_log_only\",";
  json += "\"shadow_v58_repeatability_gate_version\":\"" + String(Config::ZERO_CROSS_V58_REPEATABILITY_GATE_VERSION) + "\",";
  json += "\"shadow_v58_repeatability_gate_policy\":\"log_only_marks_analysis_subset_never_changes_q_pulse_or_motor\",";
  json += "\"v59_state_wait_fixed_q_version\":\"" + String(Config::ZERO_CROSS_V59_STATE_WAIT_FIXED_Q_VERSION) + "\",";
  json += "\"v59_state_wait_fixed_q_policy\":\"state_gate_admits_only_preplanned_fixed_q;inverse_q_and_hfree_are_diagnostic_only\",";
  json += "\"v60_rebuild_phase_version\":\"" + String(Config::ZERO_CROSS_V60_REBUILD_PHASE_VERSION) + "\",";
  json += "\"v60_rebuild_phase_policy\":\"same_fixed_q_gate_and_order;minimum_six_free_transitions_per_side_then_wait_for_measured_h_gate_support;unrestricted_rebuild_commands_only_the_opposite_arrival_side;the_observed_controlled_peak_must_predict_post_cooldown_H_and_C_inside_the_unchanged_gate\",";
  json += "\"v62_state_feasibility_version\":\"" + String(Config::ZERO_CROSS_V62_STATE_FEASIBILITY_VERSION) + "\",";
  json += "\"v62_state_feasibility_policy\":\"free_decay_rate=aH+bC+c_selects_only_supported_H_C_rate_intersection;fixed_Q_gate_plan_and_motor_command_policy_unchanged\",";
  json += "\"v62_calibration_algorithm_revision\":\"v62_joint_h_c_rate_feasibility_fixed_q_v51_video_hfree\",";
  json += "\"shadow_reachability_policy\":\"log_only_no_pulse_below_or_invalid,inverse_q_within,saturate_qmax_above\",";
  json += "\"shadow_delta_h_model\":\"DeltaH_video=-1.1393659-0.4714266*Hprev+0.0823895*abs(rate)-0.0961605*next_peak_side+0.1919561*Q_effective_pred\",";
  json += "\"shadow_model_state_timing\":\"Hprev_Cprev_rate_and_next_peak_side_are_captured_before_pulse; post_pulse_center_is_not_a_model_input\",";
  json += "\"shadow_q_effective_support_mA_s\":\"0.50-1.60\",";
  json += "\"shadow_current_control_candidate_q_range_mA_s\":\"0.50-1.50\",";
  json += "\"shadow_state_hprev_support_deg\":\"5.98-8.67\",";
  json += "\"shadow_state_abs_rate_support_dps\":\"51.97-73.15\",";
  json += "\"shadow_h_free_video_model\":\"Hfree_video=-0.1960+0.94286*Hfree_imu\",";
  json += "\"shadow_h_post_coordinate\":\"video_half_range\",";
  json += "\"shadow_h_ref_configured\":" + String(Config::ZERO_CROSS_V53_H_REF_CONFIGURED ? "true" : "false") + ",";
  json += "\"shadow_h_ref_video_deg\":" + String(Config::ZERO_CROSS_V53_H_REF_VIDEO_DEG, 3) + ",";
  json += "\"shadow_q_candidate_mA_s\":\"0,0.5,1.0,1.5\",";
  json += "\"shadow_q0_is_extrapolated\":true,";
  json += "\"shadow_q_req_region_codes\":\"0=unavailable,1=below_zero,2=within_zero_to_qmax1p5,3=above_qmax1p5\",";
  json += "\"shadow_q_req_reason_codes\":\"0=within_range,1=reference_unconfigured,2=state_invalid,3=below_zero,4=above_qmax,5=outside_model_support,6=state_out_of_support,7=h_free_invalid\",";
  json += "\"shadow_reachability_reason_codes\":\"0=unavailable,1=reference_unconfigured,2=state_invalid,3=state_out_of_support,4=h_free_invalid,5=candidates_invalid,6=target_below_candidate_range,7=target_within_candidate_range,8=target_above_candidate_range\",";
  json += "\"shadow_recommended_action_codes\":\"0=unavailable,1=no_pulse,2=inverse_q,3=saturate_qmax\",";
  json += "\"shadow_q_support_reason_codes\":\"0=unavailable,1=negative,2=below_forward_model_support,3=within_current_candidates,4=above_candidates_within_forward_model_support,5=above_forward_model_support\",";
  json += "\"shadow_v57_observed_hprev_envelope_deg\":\"5.69-9.01\",";
  json += "\"shadow_v57_observed_abs_rate_envelope_dps\":\"55.01-74.38\",";
  json += "\"shadow_v57_q_req_reason_codes\":\"0=unavailable,1=valid,2=reference_unconfigured,3=state_invalid,4=state_out_of_support,5=h_free_invalid,6=q_below_candidate_range,7=q_above_candidate_range,8=reference_unreachable\",";
  json += "\"shadow_v58_repeatability_gate\":\"Hprev=6.40-6.80deg,abs_rate=57.50-60.50dps,next_peak_side=+1\",";
  json += "\"shadow_v58_repeatability_gate_reason_codes\":\"0=unavailable,1=passed,2=disabled,3=state_invalid,4=hprev_below,5=hprev_above,6=rate_below,7=rate_above,8=direction_mismatch\",";
  json += "\"v59_state_gate\":\"Hprev=6.50-6.75deg,Cprev=0.70-1.30deg,abs_rate=58.00-60.50dps,next_peak_side=+1,dynamic_H_support_required\",";
  json += "\"v59_state_gate_reason_codes\":\"0=unavailable,1=passed,2=disabled,3=state_invalid,4=dynamic_h_out_of_support,5=hprev_below,6=hprev_above,7=cprev_below,8=cprev_above,9=rate_below,10=rate_above,11=direction_mismatch,12=v61_state_feedback_wait_or_rebuild\",";
  json += "\"v59_state_gate_action_codes\":\"0=wait_no_fixed_q,1=accepted_fixed_q,2=low_state_rebuild_or_v61_controlled_side_rebuild,3=v60_cooldown_free_decay\",";
  json += "\"v60_state_gate_action_codes\":\"0=wait_no_fixed_q,1=accepted_fixed_q,2=low_state_rebuild,3=cooldown_free_decay\",";
  json += "\"v60_gate_skipped_by_decay_definition\":\"same_desired_arrival_hprev_above_then_hprev_below_without_accepted_gate;diagnostic_not_proof_of_C_or_rate_match\",";
  json += "\"shadow_v57_imu_proxy_residual_definition\":\"shadow_delta_h_video_pred_deg-delta_half_range_dynamic_hold073_deg; video validation is offline only\",";
  json += "\"sample_size\":" + String(sizeof(LogSample)) + ",";
  json += "\"log_period_ms\":" + String(Config::LOG_PERIOD_MS) + ",";
  json += "\"imu_period_ms\":" + String(Config::IMU_PERIOD_MS) + ",";
  json += "\"measurement_goal\":\"" + String(q_observer_ && q_observer_->v57.enabled ? "reaction_wheel_speed_vs_fixed_300mA_60ms_pulse_capability" : q_observer_ && q_observer_->v55.enabled ? "initial_wheel_speed_vs_fixed_300mA_60ms_current_and_Q" : passive_capture_ ?
      "absolute_angle_motor_inhibited_manual_free_decay" :
      "compare_fixed_beta_0p100_and_fixed_beta_0p000_with_dynamic_beta_max025_hold073_hold120_hold170_zero_cross_input") + "\",";
  json += "\"startup_gyro_calib_ms\":" + String(Config::STARTUP_GYRO_CALIB_MS) + ",";
  json += "\"madgwick_settling_ms\":" + String(Config::MADGWICK_SETTLING_MS) + ",";
  json += "\"sync_led_pin\":" + String(Config::SYNC_LED_PIN) + ",";
  json += "\"led_sync_pattern_id\":\"" + String(Config::LED_SYNC_PATTERN_ID) + "\",";
  json += "\"led_sync_start_pattern\":\"OFF_1000_ON_600_OFF_300_ON_600_OFF_300_ON_1200_OFF_1000\",";
  json += "\"led_sync_end_pattern\":\"OFF_1000_ON_1200_OFF_300_ON_600_OFF_300_ON_600_OFF_1000\",";
  json += "\"led_sync_short_on_ms\":" + String(Config::LED_SYNC_SHORT_ON_MS) + ",";
  json += "\"led_sync_long_on_ms\":" + String(Config::LED_SYNC_LONG_ON_MS) + ",";
  json += "\"led_sync_half_off_ms\":" + String(Config::LED_SYNC_HALF_OFF_MS) + ",";
  json += "\"led_sync_boundary_off_ms\":" + String(Config::LED_SYNC_BOUNDARY_OFF_MS) + ",";
  json += "\"led_sync_start_total_ms\":" + String(Config::START_SYNC_PATTERN_TOTAL_MS) + ",";
  json += "\"led_sync_end_total_ms\":" + String(Config::END_SYNC_PATTERN_TOTAL_MS) + ",";
  json += "\"led_sync_mid_pattern\":\"ON_300_OFF_300_ON_600\",";
  json += "\"led_sync_mid_first_ms\":" + String(Config::MID_SYNC_FIRST_MS) + ",";
  json += "\"led_sync_mid_interval_ms\":" + String(Config::MID_SYNC_INTERVAL_MS) + ",";
  json += "\"led_sync_mid_total_ms\":" + String(Config::MID_SYNC_TOTAL_MS) + ",";
  json += "\"video_sync_method\":\"fit_video_time_to_logged_start_mid_end_led_transitions\",";
  json += "\"experiment_start_definition\":\"t_test_zero_is_after_start_signature_final_off_all_led_states_logged\",";
  json += "\"trial_duration_ms\":" + String(Config::BETA_SWEEP_TRIAL_DURATION_MS) + ",";
  json += "\"inter_trial_rest_ms\":" + String(Config::BETA_SWEEP_INTER_TRIAL_REST_MS) + ",";
  json += "\"batch_total_duration_ms\":" + String(Config::BETA_SWEEP_TOTAL_DURATION_MS) + ",";
  json += "\"current_mA\":" + String(run_current_mA_) + ",";
  json += "\"pulse_width_ms\":" + String(run_pulse_width_ms_) + ",";
  json += "\"input_interval_ms\":" + String(run_input_interval_ms_) + ",";
  json += "\"input_pattern\":\"" + String(q_observer_ && q_observer_->v57.enabled ? "stationary_baseline_speed_prepare_transfer_ready_abs_current_minus_baseline_le_0p7mA_for_20ms_then_fixed_300mA_60ms" : q_observer_ && q_observer_->v55.enabled ? "measured_speed_preparation_then_fixed_probe_5bands_2directions" : passive_capture_ ?
      "manual_release_no_motor_command" :
      "zero_cross_gated_alternating_fixed_current_after_bootstrap") + "\",";
  json += "\"beta_recovery_rule\":\"series_specific_input_hold_softstart_time_matched_linear__zero_cross_event_resets_each_timer\",";
  json += "\"beta_hold_after_input_ms_reference\":" +
          String(Config::DYNAMIC_BETA_HOLD_AFTER_INPUT_MS[Config::FILTER_ADOPTED_INDEX]) + ",";
  json += "\"beta_hold_series_ms\":\"73,120,170\",";
  json += "\"turn_fast_series\":\"low_beta_until_confirmed_turn_then_25ms_smooth_recovery\",";
  json += "\"turn_fast_confirm_samples\":" + String(Config::BETA_TURN_FAST_CONFIRM_SAMPLES) + ",";
  json += "\"turn_fast_recovery_ms\":" + String(Config::BETA_TURN_FAST_RECOVERY_MS) + ",";
  json += "\"turn_fast_fallback_after_input_ms\":" + String(Config::BETA_TURN_FAST_FALLBACK_MS) + ",";
  json += "\"fixed_beta_series\":\"0.100,0.000\",";
  json += "\"beta_soft_start_ms\":" + String(Config::BETA_SOFT_START_MS) + ",";
  json += "\"beta_time_match_reference_recovery_per_ms\":" + String(Config::BETA_TIME_MATCH_REFERENCE_RECOVERY_PER_MS, 7) + ",";
  json += "\"beta_dynamic_ceiling\":0.025,";
  const uint32_t zero_cross_duration_ms = q_run_mode_ == 2 && Config::ZERO_CROSS_V59_STATE_WAIT_FIXED_Q_ENABLED
      ? Config::ZERO_CROSS_V59_TEST_DURATION_MS : Config::ZERO_CROSS_TEST_DURATION_MS;
  json += "\"zero_cross_test_duration_ms\":" + String(zero_cross_duration_ms) + ",";
  json += "\"zero_cross_amplitude_control\":\"disabled_fixed_current\",";
  json += "\"zero_cross_allowed_current_range_mA\":\"300 (fixed)\",";
  json += "\"zero_cross_allowed_pulse_width_range_ms\":\"5-25 (integer)\",";
  json += "\"zero_cross_fixed_current_mA\":" + String(run_current_mA_) + ",";
  json += "\"zero_cross_fixed_pulse_ms\":" + String(run_pulse_width_ms_) + ",";
  json += "\"zero_cross_start_kick_current_mA\":" + String(run_current_mA_) + ",";
  json += "\"zero_cross_start_kick_ms\":" + String(run_pulse_width_ms_) + ",";
  json += "\"zero_cross_start_refractory_ms\":" + String(Config::ZERO_CROSS_START_REFRACTORY_MS) + ",";
  json += "\"zero_cross_rearm_angle_deg\":" + String(Config::ZERO_CROSS_REARM_ANGLE_DEG, 3) + ",";
  json += "\"zero_cross_min_rate_dps\":" + String(Config::ZERO_CROSS_MIN_RATE_DPS, 3) + ",";
  json += "\"zero_cross_min_pulse_interval_ms\":" + String(Config::ZERO_CROSS_MIN_PULSE_INTERVAL_MS) + ",";
  json += "\"zero_cross_bootstrap_direction\":" + String(Config::ZERO_CROSS_BOOTSTRAP_DIRECTION) + ",";
  json += "\"zero_cross_direction_rule\":\"alternate_each_accepted_crossing_after_bootstrap\",";
  json += "\"beta_phase_return_test_enabled\":" + String(Config::BETA_PHASE_RETURN_TEST_ENABLED ? "true" : "false") + ",";
  json += "\"beta_phase_exponential_k\":" + String(Config::BETA_PHASE_EXPONENTIAL_K, 3) + ",";
  json += "\"beta_phase_outbound_rate_min_dps\":" + String(Config::BETA_PHASE_OUTBOUND_RATE_MIN_DPS, 3) + ",";
  json += "\"beta_phase_turn_confirm_rate_dps\":" + String(Config::BETA_PHASE_TURN_CONFIRM_RATE_DPS, 3) + ",";
  json += "\"beta_phase_min_peak_angle_deg\":" + String(Config::BETA_PHASE_MIN_PEAK_ANGLE_DEG, 3) + ",";
  json += "\"beta_phase_peak_source\":\"bias_corrected_gyro_integral_relative_to_pulse_start\",";
  json += "\"beta_recovery_tau_s\":" + String(Config::BETA_RECOVERY_TAU_S, 5) + ",";
  json += "\"beta_normal\":" + String(Config::MADGWICK_BETA_NORMAL, 5) + ",";
  json += "\"beta_min_at_zero_current\":" + String(Config::BETA_MIN_AT_ZERO_CURRENT, 5) + ",";
  json += "\"beta_min_at_reference_current\":" + String(Config::BETA_MIN_AT_REFERENCE_CURRENT, 5) + ",";
  json += "\"beta_min_reference_current_mA\":" + String(Config::BETA_MIN_REFERENCE_CURRENT_MA, 3) + ",";
  json += "\"model_vbat_reference_v\":" + String(Config::MODEL_VBAT_REFERENCE_V, 3) + ",";
  json += "\"model_vbat_min_v\":" + String(Config::MODEL_VBAT_MIN_V, 3) + ",";
  json += "\"model_vbat_max_v\":" + String(Config::MODEL_VBAT_MAX_V, 3) + ",";
  json += "\"model_i_sat_at_reference_mA\":" + String(Config::MODEL_I_SAT_AT_REFERENCE_MA, 6) + ",";
  json += "\"model_i_sat_slope_mA_per_v\":" + String(Config::MODEL_I_SAT_SLOPE_MA_PER_V, 6) + ",";
  json += "\"model_i_sat_exponent\":" + String(Config::MODEL_I_SAT_EXPONENT, 3) + ",";
  json += "\"model_tau_rise_min_ms\":" + String(Config::MODEL_TAU_RISE_MIN_MS, 3) + ",";
  json += "\"model_tau_rise_max_ms\":" + String(Config::MODEL_TAU_RISE_MAX_MS, 3) + ",";
  json += "\"model_tau_rise_u_mA\":" + String(Config::MODEL_TAU_RISE_U_MA, 3) + ",";
  json += "\"model_tau_rise_exponent\":" + String(Config::MODEL_TAU_RISE_EXPONENT, 3) + ",";
  json += "\"filter_strategies\":[";
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    if (i) json += ",";
    json += "\"" + String(Config::DYNAMIC_BETA_LABELS[i]) + "\"";
  }
  json += "],";
  json += "\"trials\":[{";
  json += "\"trial_index\":1,";
  json += "\"name\":\"" + String(passive_capture_ ? "manual_passive_release" : "manual_zero_cross_selected_condition") + "\",";
  json += "\"current_mA\":" + String(run_current_mA_) + ",";
  json += "\"pulse_width_ms\":" + String(run_pulse_width_ms_) + ",";
  json += "\"input_interval_ms\":" + String(run_input_interval_ms_) + ",";
  json += "\"duration_ms\":" + String(passive_capture_ ? Config::PASSIVE_CAPTURE_DURATION_MS : zero_cross_duration_ms);
  json += "}],";
  json += "\"identification_mode\":" + String(identification_mode_ ? "true" : "false") + ",";
  const char* q_mode = q_run_mode_ == 2 ? "control" : (q_run_mode_ == 1 ? "validation" : "none");
  const uint8_t q_schedule = q_probe_schedule_id_ <
      Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_COUNT ? q_probe_schedule_id_ :
      Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A;
  const char* q_schedule_name = q_schedule == Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_B ? "B" :
      (q_schedule == Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_C ? "C" : "A");
  json += "\"q_run_mode\":\"" + String(q_mode) + "\",";
  json += "\"q_probe_schedule_id\":" + String(q_schedule) + ",";
  json += "\"q_probe_schedule_name\":\"" + String(q_schedule_name) + "\",";
  json += "\"q_control_target_peak_deg\":" + String(control_target_cdeg_ / 100.0f, 3) + ",";
  json += "\"q_response_model\":\"A_next=0.022622+0.088146*abs(rate0)-0.073766*abs(theta0)+0.269799*Q_command-0.124417*d-0.022127*Q_command*d\",";
  json += "\"q_control_selection\":\"continuous_inverse_target_peak_Q\",";
  json += "\"q_control_q_max_mA_s\":" + String(Config::ZERO_CROSS_CONTROL_Q_MAX_MAS, 4) + ",";
  json += "\"q_control_solver_min_pulse_ms\":" + String(Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS) + ",";
  json += "\"q_control_solver_max_pulse_ms\":" + String(Config::ZERO_CROSS_CONTROL_MAX_PULSE_MS) + ",";
  json += "\"shadow_offset_deg\":" + String(Config::ZERO_CROSS_Q_MODEL_SHADOW_OFFSET_DEG, 3) + ",";
  json += "\"identification_event_q_units\":\"mA*s (signed estimate stored separately)\",";
  json += "\"identification_event_amplitude_fields\":\"a_min_pred_deg/a_max_pred_deg are the four-Q predicted range; a_pred_shadow_deg and a_pred_calibrated_deg are log-only; calibrated_prediction_valid gates the latter; bootstrap=true marks the forced start kick; a_next_imu_abs_deg is the on-device observed peak\",";
  json += "\"calibration_shadow_enabled\":" + String(calibration_result_.enabled ? "true" : "false") + ",";
  json += "\"calibration_model\":\"A_next=r_next_side*A_prev+g_next_side*Q_effective_pred+c_next_side\",";
  json += "\"cal_initial_target_min_deg\":" + String(Config::ZERO_CROSS_CALIBRATION_INITIAL_MIN_DEG, 3) + ",";
  json += "\"cal_initial_target_max_deg\":" + String(Config::ZERO_CROSS_CALIBRATION_INITIAL_MAX_DEG, 3) + ",";
  json += "\"cal_initial_abort_deg\":" + String(Config::ZERO_CROSS_CALIBRATION_INITIAL_ABORT_DEG, 3) + ",";
  json += "\"cal_initial_max_kicks\":" + String(Config::ZERO_CROSS_CALIBRATION_INITIAL_MAX_KICKS) + ",";
  json += "\"cal_initial_target_peak_deg\":" + String(Config::ZERO_CROSS_CALIBRATION_INITIAL_TARGET_PEAK_DEG, 3) + ",";
  json += "\"cal_initial_predict_guard_deg\":" + String(Config::ZERO_CROSS_CALIBRATION_INITIAL_PREDICT_GUARD_DEG, 3) + ",";
  json += "\"cal_initial_q_max_mA_s\":" + String(Config::ZERO_CROSS_CALIBRATION_INITIAL_Q_MAX_MAS, 4) + ",";
  json += "\"cal_initial_solver_max_pulse_ms\":" + String(Config::ZERO_CROSS_CALIBRATION_INITIAL_MAX_PULSE_MS) + ",";
  json += "\"calibration_timeout_ms\":" + String(Config::ZERO_CROSS_CALIBRATION_TIMEOUT_MS) + ",";
  json += "\"v59_gate_event_count\":" + String(calibration_result_.v59_gate_event_count) + ",";
  json += "\"v59_gate_pass_count\":" + String(calibration_result_.v59_gate_pass_count) + ",";
  json += "\"v59_gate_skip_count\":" + String(calibration_result_.v59_gate_skip_count) + ",";
  json += "\"v59_rebuild_from_low_state_count\":" + String(calibration_result_.v59_rebuild_from_low_state_count) + ",";
  json += "\"v60_rebuild_target_valid\":" + String(calibration_result_.v60_rebuild_target_valid ? "true" : "false") + ",";
  json += "\"v60_rebuild_target_observed\":" + String(calibration_result_.v60_rebuild_target_observed ? "true" : "false") + ",";
  json += "\"v60_gate_center_in_dynamic_h_input_support\":" + String(calibration_result_.v60_gate_center_in_dynamic_h_input_support ? "true" : "false") + ",";
  json += "\"v61_state_target_valid\":" + String(calibration_result_.v61_state_target_valid ? "true" : "false") + ",";
  json += "\"v61_predicted_gate_h_deg\":" + String(calibration_result_.v61_predicted_gate_h_deg, 4) + ",";
  json += "\"v61_predicted_gate_c_deg\":" + String(calibration_result_.v61_predicted_gate_c_deg, 4) + ",";
  json += "\"v61_feedback_command_peak_deg\":" + String(calibration_result_.v61_feedback_command_peak_deg, 4) + ",";
  json += "\"v61_feedback_correction_count\":" + String(calibration_result_.v61_feedback_correction_count) + ",";
  json += "\"v62_rate_model_valid\":" + String(calibration_result_.v62_rate_model_valid ? "true" : "false") + ",";
  json += "\"v62_hc_target_feasible\":" + String(calibration_result_.v62_hc_target_feasible ? "true" : "false") + ",";
  json += "\"v62_state_target_valid\":" + String(calibration_result_.v62_state_target_valid ? "true" : "false") + ",";
  json += "\"v62_rate_sample_count\":" + String(calibration_result_.v62_rate_sample_count) + ",";
  json += "\"v62_rate_a_per_s\":" + String(calibration_result_.v62_rate_a_per_s, 5) + ",";
  json += "\"v62_rate_b_per_s\":" + String(calibration_result_.v62_rate_b_per_s, 5) + ",";
  json += "\"v62_rate_offset_dps\":" + String(calibration_result_.v62_rate_offset_dps, 5) + ",";
  json += "\"v62_rate_r2\":" + String(calibration_result_.v62_rate_r2, 5) + ",";
  json += "\"v62_target_h_deg\":" + String(calibration_result_.v62_target_h_deg, 4) + ",";
  json += "\"v62_target_c_deg\":" + String(calibration_result_.v62_target_c_deg, 4) + ",";
  json += "\"v62_target_rate_dps\":" + String(calibration_result_.v62_target_rate_dps, 4) + ",";
  json += "\"v62_target_controlled_peak_deg\":" + String(calibration_result_.v62_target_controlled_peak_deg, 4) + ",";
  json += "\"v60_free_decay_support_wait_count\":" + String(calibration_result_.v60_free_decay_support_wait_count) + ",";
  json += "\"v60_cooldown_free_decay_count\":" + String(calibration_result_.v60_cooldown_free_decay_count) + ",";
  json += "\"v60_gate_skipped_by_decay_count\":" + String(calibration_result_.v60_gate_skipped_by_decay_count) + ",";
  json += "\"v60_gate_center_h_deg\":" + String(calibration_result_.v60_gate_center_h_deg, 4) + ",";
  json += "\"v60_gate_center_c_deg\":" + String(calibration_result_.v60_gate_center_c_deg, 4) + ",";
  if (calibration_result_.v60_rebuild_target_valid) {
    json += "\"v60_free_decay_after_one_cycle_h_deg\":" + String(calibration_result_.v60_free_decay_after_one_cycle_h_deg, 4) + ",";
    json += "\"v60_decay_per_cycle_h_deg\":" + String(calibration_result_.v60_decay_per_cycle_h_deg, 4) + ",";
    json += "\"v60_rebuild_target_h_deg\":" + String(calibration_result_.v60_rebuild_target_h_deg, 4) + ",";
    json += "\"v60_rebuild_target_peak_deg\":" + String(calibration_result_.v60_rebuild_target_peak_deg, 4) + ",";
  } else {
    json += "\"v60_free_decay_after_one_cycle_h_deg\":null,\"v60_decay_per_cycle_h_deg\":null,";
    json += "\"v60_rebuild_target_h_deg\":null,\"v60_rebuild_target_peak_deg\":null,";
  }
  json += "\"cal_free_transitions_per_side\":" + String(Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_PER_SIDE) + ",";
  json += "\"cal_free_transitions_max_per_side\":" + String(Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_MAX_PER_SIDE) + ",";
  json += "\"cal_max_consecutive_peak_gap_ms\":" + String(Config::ZERO_CROSS_CALIBRATION_MAX_CONSECUTIVE_PEAK_GAP_MS) + ",";
  json += "\"cal_q_probe_samples_per_side\":" + String(Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SAMPLES_PER_SIDE) + ",";
  const char* q_probe_plan = q_schedule == Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_B ?
      "B:+(0.5,1.5,1.0);+(1.5,1.0,0.5);+(1.0,0.5,1.5);+(1.0,1.5,0.5) mA*s" :
      (q_schedule == Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_C ?
      "C:+(1.0,0.5,1.5);+(0.5,1.5,1.0);+(1.5,1.0,0.5);+(0.5,1.0,1.5) mA*s" :
      "A:+(0.5,1.0,1.5);+(1.0,1.5,0.5);+(1.5,0.5,1.0);+(1.5,1.0,0.5) mA*s");  json += "\"cal_q_probe_plan_count\":" + String(Config::ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_COUNT) + ",";
  json += "\"cal_q_probe_schedule_count\":" + String(Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_COUNT) + ",";
  json += "\"cal_q_probe_schedule_id\":" + String(q_schedule) + ",";
  json += "\"cal_q_probe_schedule_name\":\"" + String(q_schedule_name) + "\",";
  json += "\"cal_q_probe_solver_max_pulse_ms\":" + String(Config::ZERO_CROSS_CALIBRATION_Q_PROBE_MAX_PULSE_MS) + ",";
  json += "\"cal_q_probe_plan\":\"" + String(q_probe_plan) + "\",";
  json += "\"cal_q_probe_schedule_legend\":\"V59: all arrivals are +; each schedule has four 3-level blocks and four observations per Q level\",";
  json += "\"cal_q_probe_polarity_mode\":\"BASE_only_no_automatic_direction_flip\",";
  json += "\"cal_q_rebuild_target_peak_deg\":" + String(Config::ZERO_CROSS_CALIBRATION_Q_REBUILD_TARGET_PEAK_DEG, 3) + ",";
  json += "\"cal_q_rebuild_max_attempts_per_episode\":" + String(Config::ZERO_CROSS_CALIBRATION_Q_REBUILD_MAX_ATTEMPTS) + ",";
  json += "\"v60_rebuild_unrestricted\":" + String(Config::ZERO_CROSS_V60_REBUILD_UNRESTRICTED ? "true" : "false") + ",";
  json += "\"cal_q_rebuild_entry_rule\":\"enter next Q probe when either the legacy A_prev common input domain or the valid dynamic-H predecessor input domain contains the confirmed predecessor; no A or H model is extrapolated\",";
  json += "\"cal_q_rebuild_entry_source_legend\":\"0=legacy_A_prev_common_input_support,1=dynamic_H_predecessor_input_support\",";
  json += "\"cal_q_rebuild_target_audit\":\"per-probe rebuild_target_reached records whether the confirmed predecessor magnitude was at or above cal_q_rebuild_target_peak_deg; the target remains the rebuild-pulse setpoint, not an H-support requirement\",";
  json += "\"cal_q_probe_support_margin_deg\":" + String(Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SUPPORT_MARGIN_DEG, 3) + ",";
  json += "\"cal_q_probe_min_positive_gain_deg_per_mA_s\":" + String(Config::ZERO_CROSS_CALIBRATION_Q_PROBE_MIN_POSITIVE_GAIN_DEG_PER_MAS, 4) + ",";
  json += "\"cal_free_model_legend\":\"0=none,1=proportional_rA,2=affine_rA_plus_c\",";
  json += "\"cal_free_selection_rule\":\"select lower-LOOCV-RMSE positive monotone half-cycle maps over measured A_prev domains; require each composed same-side full cycle 0<F(A)<A over its composition domain; no extrapolation\",";
  json += "\"cal_q_probe_input_rule\":\"use A_prev input-domain intersection across both arrival-side models, reduced by cal_q_probe_support_margin_deg; issue only the next deterministic desired arrival side; if the immediate arrival is the other side, send no Q pulse for one half-cycle then re-check support; out-of-support waits return through the existing bounded rebuild; BASE direction=-desired_arrival_side and is never auto-flipped\",";
  json += "\"cal_free_model_pos\":" + String(calibration_result_.free_model_pos) + ",";
  json += "\"cal_free_model_neg\":" + String(calibration_result_.free_model_neg) + ",";
  json += "\"cal_free_halfcycle_monotone_pos\":" + String(calibration_result_.free_halfcycle_monotone_pos ? "true" : "false") + ",";
  json += "\"cal_free_halfcycle_monotone_neg\":" + String(calibration_result_.free_halfcycle_monotone_neg ? "true" : "false") + ",";
  json += "\"cal_free_full_cycle_valid_pos\":" + String(calibration_result_.free_full_cycle_valid_pos ? "true" : "false") + ",";
  json += "\"cal_free_full_cycle_valid_neg\":" + String(calibration_result_.free_full_cycle_valid_neg ? "true" : "false") + ",";
  json += "\"cal_free_full_cycle_r_pos\":" + String(calibration_result_.free_full_cycle_r_pos, 6) + ",";
  json += "\"cal_free_full_cycle_r_neg\":" + String(calibration_result_.free_full_cycle_r_neg, 6) + ",";
  json += "\"cal_free_full_cycle_c_pos_deg\":" + String(calibration_result_.free_full_cycle_c_pos_deg, 6) + ",";
  json += "\"cal_free_full_cycle_c_neg_deg\":" + String(calibration_result_.free_full_cycle_c_neg_deg, 6) + ",";
  json += "\"cal_free_full_cycle_input_min_pos_deg\":" + String(calibration_result_.free_full_cycle_input_min_pos_deg, 4) + ",";
  json += "\"cal_free_full_cycle_input_max_pos_deg\":" + String(calibration_result_.free_full_cycle_input_max_pos_deg, 4) + ",";
  json += "\"cal_free_full_cycle_input_min_neg_deg\":" + String(calibration_result_.free_full_cycle_input_min_neg_deg, 4) + ",";
  json += "\"cal_free_full_cycle_input_max_neg_deg\":" + String(calibration_result_.free_full_cycle_input_max_neg_deg, 4) + ",";
  json += "\"cal_free_support_min_pos_deg\":" + String(calibration_result_.free_support_min_pos_deg, 4) + ",";
  json += "\"cal_free_support_max_pos_deg\":" + String(calibration_result_.free_support_max_pos_deg, 4) + ",";
  json += "\"cal_free_support_min_neg_deg\":" + String(calibration_result_.free_support_min_neg_deg, 4) + ",";
  json += "\"cal_free_support_max_neg_deg\":" + String(calibration_result_.free_support_max_neg_deg, 4) + ",";
  json += "\"cal_free_input_min_pos_deg\":" + String(calibration_result_.free_input_min_pos_deg, 4) + ",";
  json += "\"cal_free_input_max_pos_deg\":" + String(calibration_result_.free_input_max_pos_deg, 4) + ",";
  json += "\"cal_free_input_min_neg_deg\":" + String(calibration_result_.free_input_min_neg_deg, 4) + ",";
  json += "\"cal_free_input_max_neg_deg\":" + String(calibration_result_.free_input_max_neg_deg, 4) + ",";
  json += "\"cal_free_proportional_r_pos\":" + String(calibration_result_.free_proportional_r_pos, 6) + ",";
  json += "\"cal_free_proportional_r_neg\":" + String(calibration_result_.free_proportional_r_neg, 6) + ",";
  json += "\"cal_free_proportional_rmse_pos_deg\":" + String(calibration_result_.free_proportional_rmse_pos_deg, 6) + ",";
  json += "\"cal_free_proportional_rmse_neg_deg\":" + String(calibration_result_.free_proportional_rmse_neg_deg, 6) + ",";
  json += "\"cal_free_proportional_r2_pos\":" + String(calibration_result_.free_proportional_r2_pos, 6) + ",";
  json += "\"cal_free_proportional_r2_neg\":" + String(calibration_result_.free_proportional_r2_neg, 6) + ",";
  json += "\"cal_free_proportional_loocv_rmse_pos_deg\":" + String(calibration_result_.free_proportional_loocv_rmse_pos_deg, 6) + ",";
  json += "\"cal_free_proportional_loocv_rmse_neg_deg\":" + String(calibration_result_.free_proportional_loocv_rmse_neg_deg, 6) + ",";
  json += "\"cal_free_proportional_valid_pos\":" + String(calibration_result_.free_proportional_valid_pos ? "true" : "false") + ",";
  json += "\"cal_free_proportional_valid_neg\":" + String(calibration_result_.free_proportional_valid_neg ? "true" : "false") + ",";
  json += "\"cal_free_affine_r_pos\":" + String(calibration_result_.free_affine_r_pos, 6) + ",";
  json += "\"cal_free_affine_r_neg\":" + String(calibration_result_.free_affine_r_neg, 6) + ",";
  json += "\"cal_free_affine_c_pos_deg\":" + String(calibration_result_.free_affine_c_pos_deg, 6) + ",";
  json += "\"cal_free_affine_c_neg_deg\":" + String(calibration_result_.free_affine_c_neg_deg, 6) + ",";
  json += "\"cal_free_affine_rmse_pos_deg\":" + String(calibration_result_.free_affine_rmse_pos_deg, 6) + ",";
  json += "\"cal_free_affine_rmse_neg_deg\":" + String(calibration_result_.free_affine_rmse_neg_deg, 6) + ",";
  json += "\"cal_free_affine_r2_pos\":" + String(calibration_result_.free_affine_r2_pos, 6) + ",";
  json += "\"cal_free_affine_r2_neg\":" + String(calibration_result_.free_affine_r2_neg, 6) + ",";
  json += "\"cal_free_affine_loocv_rmse_pos_deg\":" + String(calibration_result_.free_affine_loocv_rmse_pos_deg, 6) + ",";
  json += "\"cal_free_affine_loocv_rmse_neg_deg\":" + String(calibration_result_.free_affine_loocv_rmse_neg_deg, 6) + ",";
  json += "\"cal_free_affine_valid_pos\":" + String(calibration_result_.free_affine_valid_pos ? "true" : "false") + ",";
  json += "\"cal_free_affine_valid_neg\":" + String(calibration_result_.free_affine_valid_neg ? "true" : "false") + ",";
  json += "\"calibration_protocol_complete\":" + String(calibration_result_.protocol_complete ? "true" : "false") + ",";
  json += "\"calibration_probe_plan_count_completed\":" + String(calibration_result_.probe_plan_count) + ",";
  json += "\"calibration_wait_halfcycles\":" + String(calibration_result_.probe_wait_halfcycles) + ",";
  json += "\"calibration_valid\":" + String(calibration_result_.valid ? "true" : "false") + ",";
  json += "\"cal_failure_reason_code\":" + String(calibration_result_.failure_reason) + ",";
  json += "\"cal_failure_reason_legend\":\"0=none,1=initial_band_or_amplitude,2=no_physical_free_decay_model,3=nonfinite_fit,4=timeout,5=q_probe_attempt_limit,6=nonconsecutive_or_nonalternating_peak,7=reserved_v60_pre_support,8=reserved_v60_rebuild_attempt_limit,9=v60_h_gate_not_covered_before_free_decay_limit,10=v59_consecutive_gate_skip_limit\",";
  json += "\"cal_failure_reason_v62_legend\":\"11=v62_no_feasible_hc_target,12=v62_rate_model_unavailable_or_no_feasible_hc_rate_target\",";
  json += "\"cal_initial_kick_count\":" + String(calibration_result_.initial_kick_count) + ",";
  json += "\"cal_q_rebuild_total_count\":" + String(calibration_result_.rebuild_total_count) + ",";
  json += "\"cal_q_rebuild_episode_id\":" + String(calibration_result_.rebuild_episode_id) + ",";
  json += "\"cal_q_rebuild_attempt_in_episode\":" + String(calibration_result_.rebuild_attempt_in_episode) + ",";
  json += "\"calibration_peak_phase_legend\":\"0=initial_build_up,1=free_decay,2=q_probe_pos_accepted_or_rejected,3=q_probe_neg_accepted_or_rejected,4=rejected_nonconsecutive_or_nonalternating,5=unforced_or_pre_pulse_peak,6=q_probe_rebuild_arrival\",";
  json += "\"calibration_probe_result_code_legend\":\"0=positive_gain_stored,1=pre_pulse_peak,2=side_mismatch,3=outside_support,4=nonfinite_gain,5=nonpositive_gain,6=positive_gain_not_stored\",";
  json += "\"cal_r_pos\":" + String(calibration_result_.r_pos, 6) + ",";
  json += "\"cal_r_neg\":" + String(calibration_result_.r_neg, 6) + ",";
  json += "\"cal_g_pos\":" + String(calibration_result_.g_pos, 6) + ",";
  json += "\"cal_g_neg\":" + String(calibration_result_.g_neg, 6) + ",";
  json += "\"cal_g_pos_stddev\":" + String(calibration_result_.g_pos_stddev, 6) + ",";
  json += "\"cal_g_neg_stddev\":" + String(calibration_result_.g_neg_stddev, 6) + ",";
  json += "\"cal_c_pos_deg\":" + String(calibration_result_.c_pos_deg, 6) + ",";
  json += "\"cal_c_neg_deg\":" + String(calibration_result_.c_neg_deg, 6) + ",";
  json += "\"cal_free_count_pos\":" + String(calibration_result_.free_count_pos) + ",";
  json += "\"cal_free_count_neg\":" + String(calibration_result_.free_count_neg) + ",";
  json += "\"cal_q_count_pos\":" + String(calibration_result_.q_count_pos) + ",";
  json += "\"cal_q_count_neg\":" + String(calibration_result_.q_count_neg) + ",";
  json += "\"cal_q_used_pos_mA_s\":" + String(calibration_result_.q_used_pos_mA_s, 4) + ",";
  json += "\"cal_q_used_neg_mA_s\":" + String(calibration_result_.q_used_neg_mA_s, 4) + ",";
  json += "\"cal_half_range_shadow_coordinate\":\"H=abs(theta_k-theta_k_minus_1)/2; C=(theta_k+theta_k_minus_1)/2; peak timing is dynamic_hold073 and fixed_b100 is sampled at that same time\",";
  json += "\"cal_half_range_free_count\":" + String(calibration_result_.half_range_free_count) + ",";
  json += "\"cal_half_range_dynamic_r\":" + String(calibration_result_.half_range_dynamic_r, 6) + ",";
  json += "\"cal_half_range_dynamic_c_deg\":" + String(calibration_result_.half_range_dynamic_c_deg, 6) + ",";
  json += "\"cal_half_range_dynamic_rmse_deg\":" + String(calibration_result_.half_range_dynamic_rmse_deg, 6) + ",";
  json += "\"cal_half_range_dynamic_loocv_rmse_deg\":" + String(calibration_result_.half_range_dynamic_loocv_rmse_deg, 6) + ",";
  json += "\"cal_half_range_dynamic_input_min_deg\":" + String(calibration_result_.half_range_dynamic_input_min_deg, 6) + ",";
  json += "\"cal_half_range_dynamic_input_max_deg\":" + String(calibration_result_.half_range_dynamic_input_max_deg, 6) + ",";
  json += "\"cal_half_range_dynamic_valid\":" + String(calibration_result_.half_range_dynamic_valid ? "true" : "false") + ",";
  json += "\"cal_half_range_fixed_r\":" + String(calibration_result_.half_range_fixed_r, 6) + ",";
  json += "\"cal_half_range_fixed_c_deg\":" + String(calibration_result_.half_range_fixed_c_deg, 6) + ",";
  json += "\"cal_half_range_fixed_rmse_deg\":" + String(calibration_result_.half_range_fixed_rmse_deg, 6) + ",";
  json += "\"cal_half_range_fixed_loocv_rmse_deg\":" + String(calibration_result_.half_range_fixed_loocv_rmse_deg, 6) + ",";
  json += "\"cal_half_range_fixed_input_min_deg\":" + String(calibration_result_.half_range_fixed_input_min_deg, 6) + ",";
  json += "\"cal_half_range_fixed_input_max_deg\":" + String(calibration_result_.half_range_fixed_input_max_deg, 6) + ",";
  json += "\"cal_half_range_fixed_valid\":" + String(calibration_result_.half_range_fixed_valid ? "true" : "false") + ",";
  json += "\"calibration_build_up_event_fields\":\"phase 0=initial build-up,1=post-fit Q-probe rebuild (one event per bounded attempt); target_peak_deg is the requested peak; q_required=unclamped inverse-model Q; q_command=after guard/Q cap; q_effective=current-model Q at the selected integer width; predicted_peak uses q_effective; q_cap_limited/guard_limited/width_limited mark active constraints\",";
  json += "\"calibration_build_up_events\":[";
  for (uint8_t i = 0; i < calibration_build_up_event_count_; ++i) {
    const CalibrationBuildUpEvent& e = calibration_build_up_events_[i];
    if (i) json += ",";
    json += "{\"pulse_index\":" + String(e.pulse_index);
    json += ",\"start_ms\":" + String(e.start_ms);
    json += ",\"theta0_deg\":" + String(e.theta0_cdeg / 100.0f, 3);
    json += ",\"rate0_dps\":" + String(e.rate0_cdps / 100.0f, 3);
    json += ",\"direction\":" + String(e.direction);
    json += ",\"phase\":" + String(e.phase);
    json += ",\"target_peak_deg\":" + String(e.target_peak_cdeg / 100.0f, 3);
    json += ",\"vbat_mV\":" + String(e.vbat_mV);
    json += ",\"pulse_width_ms\":" + String(e.pulse_width_ms);
    json += ",\"q_required_mA_s\":" + String(e.q_required_mAms / 1000.0f, 4);
    json += ",\"q_command_mA_s\":" + String(e.q_command_mAms / 1000.0f, 4);
    json += ",\"q_effective_mA_s\":" + String(e.q_effective_mAms / 1000.0f, 4);
    json += ",\"predicted_peak_deg\":" + String(e.predicted_peak_cdeg / 100.0f, 3);
    json += ",\"q_cap_limited\":" + String(e.q_cap_limited ? "true" : "false");
    json += ",\"guard_limited\":" + String(e.guard_limited ? "true" : "false");
    json += ",\"width_limited\":" + String(e.width_limited ? "true" : "false") + "}";
  }
  json += "],";
  json += "\"calibration_peak_event_fields\":\"candidate_peak_ms is the physical extremum time found by dynamic_hold073; confirmed_ms is the later reversal-confirmation time; signed, center, and half_range preserve dynamic_hold073 and fixed_b100 at the same candidate time\",";
  json += "\"calibration_peak_events\":[";
  for (uint8_t i = 0; i < calibration_peak_event_count_; ++i) {
    const CalibrationPeakEvent& p = calibration_peak_events_[i];
    if (i) json += ",";
    json += "{\"peak_index\":" + String(p.peak_index);
    json += ",\"candidate_peak_ms\":" + String(p.candidate_peak_ms);
    json += ",\"confirmed_ms\":" + String(p.confirmed_ms);
    json += ",\"peak_side\":" + String(p.peak_side);
    json += ",\"phase\":" + String(p.phase);
    json += ",\"peak_abs_deg\":" + String(p.peak_abs_cdeg / 100.0f, 3);
    if (p.prev_peak_abs_cdeg == LOG_NAN_I16) json += ",\"prev_peak_abs_deg\":null";
    else json += ",\"prev_peak_abs_deg\":" + String(p.prev_peak_abs_cdeg / 100.0f, 3);
    if (p.peak_signed_dynamic_cdeg == LOG_NAN_I16) json += ",\"peak_signed_dynamic_hold073_deg\":null";
    else json += ",\"peak_signed_dynamic_hold073_deg\":" + String(p.peak_signed_dynamic_cdeg / 100.0f, 3);
    if (p.prev_peak_signed_dynamic_cdeg == LOG_NAN_I16) json += ",\"prev_peak_signed_dynamic_hold073_deg\":null";
    else json += ",\"prev_peak_signed_dynamic_hold073_deg\":" + String(p.prev_peak_signed_dynamic_cdeg / 100.0f, 3);
    if (p.center_dynamic_cdeg == LOG_NAN_I16) json += ",\"center_dynamic_hold073_deg\":null";
    else json += ",\"center_dynamic_hold073_deg\":" + String(p.center_dynamic_cdeg / 100.0f, 3);
    if (p.half_range_dynamic_cdeg == LOG_NAN_I16) json += ",\"half_range_dynamic_hold073_deg\":null";
    else json += ",\"half_range_dynamic_hold073_deg\":" + String(p.half_range_dynamic_cdeg / 100.0f, 3);
    if (p.peak_signed_fixed_cdeg == LOG_NAN_I16) json += ",\"peak_signed_fixed_b100_deg\":null";
    else json += ",\"peak_signed_fixed_b100_deg\":" + String(p.peak_signed_fixed_cdeg / 100.0f, 3);
    if (p.prev_peak_signed_fixed_cdeg == LOG_NAN_I16) json += ",\"prev_peak_signed_fixed_b100_deg\":null";
    else json += ",\"prev_peak_signed_fixed_b100_deg\":" + String(p.prev_peak_signed_fixed_cdeg / 100.0f, 3);
    if (p.center_fixed_cdeg == LOG_NAN_I16) json += ",\"center_fixed_b100_deg\":null";
    else json += ",\"center_fixed_b100_deg\":" + String(p.center_fixed_cdeg / 100.0f, 3);
    if (p.half_range_fixed_cdeg == LOG_NAN_I16) json += ",\"half_range_fixed_b100_deg\":null";
    else json += ",\"half_range_fixed_b100_deg\":" + String(p.half_range_fixed_cdeg / 100.0f, 3);
    json += ",\"q_effective_pred_mA_s\":" + String(p.q_effective_pred_mAms / 1000.0f, 4) + "}";
  }
  json += "],";
  json += "\"calibration_probe_event_fields\":\"v51 q_probe_schedule_id, planned_probe_index, and q_level_index identify the balanced signed-Q order; command_dynamic_angle and command_rate fields are sampled immediately before the Q pulse (raw and gyro-bias-corrected pitch rate); q_effective_pred is the current-model prediction from residual I0, not a sensor-integrated charge; half_range_previous, center, free_pred, observed, and delta fields are the dynamic-H H_prev, post-pulse C, H_free_pred, H_post, and Delta_H audit tuple. v52 shadow_* fields use only pre-pulse Hprev/C_prev/abs(rate)/expected next-peak side/Q_effective_pred; the post-pulse center is explicitly excluded. v57 independently records H/rate support, raw inverse-Q support and candidate validity, and an IMU dynamic-H proxy residual; video validation is joined offline only. All shadow fields are audit-only and cannot alter normal control, calibration, pulse width, or motor command\",";
  json += "\"calibration_probe_events\":[";
  for (uint8_t i = 0; i < calibration_probe_event_count_; ++i) {
    const CalibrationProbeEvent& e = calibration_probe_events_[i];
    if (i) json += ",";
    json += "{\"probe_index\":" + String(e.probe_index);
    json += ",\"planned_probe_index\":" + String(e.planned_probe_index);
    json += ",\"q_level_index\":" + String(e.q_level_index);
    json += ",\"q_probe_schedule_id\":" + String(e.q_probe_schedule_id);
    json += ",\"polarity_mode\":" + String(e.polarity_mode);
    json += ",\"rebuild_count\":" + String(e.rebuild_count);
    json += ",\"rebuild_episode_id\":" + String(e.rebuild_episode_id);
    json += ",\"rebuild_attempt_in_episode\":" + String(e.rebuild_attempt_in_episode);
    json += ",\"rebuild_entry_source\":" + String(e.rebuild_entry_source);
    json += ",\"wait_halfcycle_count\":" + String(e.wait_halfcycle_count);
    json += ",\"result_code\":" + String(e.result_code);
    json += ",\"pulse_start_ms\":" + String(e.pulse_start_ms);
    json += ",\"candidate_peak_ms\":" + String(e.candidate_peak_ms);
    json += ",\"confirmed_ms\":" + String(e.confirmed_ms);
    json += ",\"previous_peak_candidate_ms\":" + String(e.previous_peak_candidate_ms);
    json += ",\"requested_side\":" + String(e.requested_side);
    json += ",\"predecessor_side\":" + String(e.predecessor_side);
    json += ",\"observed_side\":" + String(e.observed_side);
    json += ",\"pulse_direction\":" + String(e.pulse_direction);
    json += ",\"pulse_id\":" + String(e.pulse_id);
    json += ",\"pulse_width_ms\":" + String(e.pulse_width_ms);
    if (e.command_dynamic_angle_cdeg == LOG_NAN_I16) json += ",\"command_dynamic_angle_deg\":null";
    else json += ",\"command_dynamic_angle_deg\":" + String(e.command_dynamic_angle_cdeg / 100.0f, 3);
    if (e.command_rate_raw_cdps == LOG_NAN_I16) json += ",\"command_rate_raw_dps\":null";
    else json += ",\"command_rate_raw_dps\":" + String(e.command_rate_raw_cdps / 100.0f, 3);
    if (e.command_rate_bias_corrected_cdps == LOG_NAN_I16) json += ",\"command_rate_bias_corrected_dps\":null";
    else json += ",\"command_rate_bias_corrected_dps\":" + String(e.command_rate_bias_corrected_cdps / 100.0f, 3);
    if (e.previous_peak_abs_cdeg == LOG_NAN_I16) json += ",\"previous_peak_abs_deg\":null";
    else json += ",\"previous_peak_abs_deg\":" + String(e.previous_peak_abs_cdeg / 100.0f, 3);
    if (e.observed_peak_abs_cdeg == LOG_NAN_I16) json += ",\"observed_peak_abs_deg\":null";
    else json += ",\"observed_peak_abs_deg\":" + String(e.observed_peak_abs_cdeg / 100.0f, 3);
    if (e.previous_peak_signed_dynamic_cdeg == LOG_NAN_I16) json += ",\"previous_peak_signed_dynamic_hold073_deg\":null";
    else json += ",\"previous_peak_signed_dynamic_hold073_deg\":" + String(e.previous_peak_signed_dynamic_cdeg / 100.0f, 3);
    if (e.observed_peak_signed_dynamic_cdeg == LOG_NAN_I16) json += ",\"observed_peak_signed_dynamic_hold073_deg\":null";
    else json += ",\"observed_peak_signed_dynamic_hold073_deg\":" + String(e.observed_peak_signed_dynamic_cdeg / 100.0f, 3);
    if (e.center_dynamic_cdeg == LOG_NAN_I16) json += ",\"center_dynamic_hold073_deg\":null";
    else json += ",\"center_dynamic_hold073_deg\":" + String(e.center_dynamic_cdeg / 100.0f, 3);
    if (e.half_range_previous_dynamic_cdeg == LOG_NAN_I16) json += ",\"half_range_previous_dynamic_hold073_deg\":null";
    else json += ",\"half_range_previous_dynamic_hold073_deg\":" + String(e.half_range_previous_dynamic_cdeg / 100.0f, 3);
    if (e.half_range_observed_dynamic_cdeg == LOG_NAN_I16) json += ",\"half_range_observed_dynamic_hold073_deg\":null";
    else json += ",\"half_range_observed_dynamic_hold073_deg\":" + String(e.half_range_observed_dynamic_cdeg / 100.0f, 3);
    if (e.half_range_free_pred_dynamic_cdeg == LOG_NAN_I16) json += ",\"half_range_free_pred_dynamic_hold073_deg\":null";
    else json += ",\"half_range_free_pred_dynamic_hold073_deg\":" + String(e.half_range_free_pred_dynamic_cdeg / 100.0f, 3);
    if (e.delta_half_range_dynamic_cdeg == LOG_NAN_I16) json += ",\"delta_half_range_dynamic_hold073_deg\":null";
    else json += ",\"delta_half_range_dynamic_hold073_deg\":" + String(e.delta_half_range_dynamic_cdeg / 100.0f, 3);
    if (e.gain_half_range_dynamic_cdeg_per_mAs == LOG_NAN_I16) json += ",\"gain_half_range_dynamic_hold073_deg_per_mA_s\":null";
    else json += ",\"gain_half_range_dynamic_hold073_deg_per_mA_s\":" + String(e.gain_half_range_dynamic_cdeg_per_mAs / 100.0f, 3);
    if (e.previous_peak_signed_fixed_cdeg == LOG_NAN_I16) json += ",\"previous_peak_signed_fixed_b100_deg\":null";
    else json += ",\"previous_peak_signed_fixed_b100_deg\":" + String(e.previous_peak_signed_fixed_cdeg / 100.0f, 3);
    if (e.observed_peak_signed_fixed_cdeg == LOG_NAN_I16) json += ",\"observed_peak_signed_fixed_b100_deg\":null";
    else json += ",\"observed_peak_signed_fixed_b100_deg\":" + String(e.observed_peak_signed_fixed_cdeg / 100.0f, 3);
    if (e.center_fixed_cdeg == LOG_NAN_I16) json += ",\"center_fixed_b100_deg\":null";
    else json += ",\"center_fixed_b100_deg\":" + String(e.center_fixed_cdeg / 100.0f, 3);
    if (e.half_range_previous_fixed_cdeg == LOG_NAN_I16) json += ",\"half_range_previous_fixed_b100_deg\":null";
    else json += ",\"half_range_previous_fixed_b100_deg\":" + String(e.half_range_previous_fixed_cdeg / 100.0f, 3);
    if (e.half_range_observed_fixed_cdeg == LOG_NAN_I16) json += ",\"half_range_observed_fixed_b100_deg\":null";
    else json += ",\"half_range_observed_fixed_b100_deg\":" + String(e.half_range_observed_fixed_cdeg / 100.0f, 3);
    if (e.half_range_free_pred_fixed_cdeg == LOG_NAN_I16) json += ",\"half_range_free_pred_fixed_b100_deg\":null";
    else json += ",\"half_range_free_pred_fixed_b100_deg\":" + String(e.half_range_free_pred_fixed_cdeg / 100.0f, 3);
    if (e.delta_half_range_fixed_cdeg == LOG_NAN_I16) json += ",\"delta_half_range_fixed_b100_deg\":null";
    else json += ",\"delta_half_range_fixed_b100_deg\":" + String(e.delta_half_range_fixed_cdeg / 100.0f, 3);
    if (e.gain_half_range_fixed_cdeg_per_mAs == LOG_NAN_I16) json += ",\"gain_half_range_fixed_b100_deg_per_mA_s\":null";
    else json += ",\"gain_half_range_fixed_b100_deg_per_mA_s\":" + String(e.gain_half_range_fixed_cdeg_per_mAs / 100.0f, 3);
    json += ",\"half_range_dynamic_in_support\":" + String(e.half_range_dynamic_in_support ? "true" : "false");
    json += ",\"half_range_fixed_in_support\":" + String(e.half_range_fixed_in_support ? "true" : "false");
    json += ",\"rebuild_target_reached\":" + String(e.rebuild_target_reached ? "true" : "false");
    json += ",\"dynamic_h_in_support_at_command\":" + String(e.dynamic_h_in_support_at_command ? "true" : "false");
    json += ",\"q_target_mA_s\":" + String(e.q_target_mAms / 1000.0f, 4);
    json += ",\"q_effective_pred_mA_s\":" + String(e.q_effective_pred_mAms / 1000.0f, 4);
    if (e.shadow_hprev_imu_cdeg == LOG_NAN_I16) json += ",\"shadow_Hprev_imu_deg\":null";
    else json += ",\"shadow_Hprev_imu_deg\":" + String(e.shadow_hprev_imu_cdeg / 100.0f, 3);
    if (e.shadow_c_prev_imu_cdeg == LOG_NAN_I16) json += ",\"shadow_C_prev_imu_deg\":null";
    else json += ",\"shadow_C_prev_imu_deg\":" + String(e.shadow_c_prev_imu_cdeg / 100.0f, 3);
    if (e.shadow_rate_abs_cdps == LOG_NAN_I16) json += ",\"shadow_rate_abs_dps\":null";
    else json += ",\"shadow_rate_abs_dps\":" + String(e.shadow_rate_abs_cdps / 100.0f, 3);
    json += ",\"shadow_next_peak_side\":" + String(e.shadow_next_peak_side);
    json += ",\"shadow_direction\":" + String(e.shadow_next_peak_side);
    if (!isfinite(e.shadow_q_effective_pred_mA_s)) json += ",\"shadow_q_effective_pred_mA_s\":null";
    else json += ",\"shadow_q_effective_pred_mA_s\":" + String(e.shadow_q_effective_pred_mA_s, 4);
    if (!isfinite(e.shadow_delta_h_video_pred_deg)) json += ",\"shadow_delta_h_video_pred_deg\":null";
    else json += ",\"shadow_delta_h_video_pred_deg\":" + String(e.shadow_delta_h_video_pred_deg, 4);
    if (!isfinite(e.shadow_delta_h_imu_actual_deg)) json += ",\"shadow_delta_h_imu_actual_deg\":null";
    else json += ",\"shadow_delta_h_imu_actual_deg\":" + String(e.shadow_delta_h_imu_actual_deg, 4);
    if (!isfinite(e.shadow_delta_h_pred_minus_imu_dynamic_deg)) json += ",\"shadow_delta_h_pred_minus_imu_dynamic_deg\":null";
    else json += ",\"shadow_delta_h_pred_minus_imu_dynamic_deg\":" + String(e.shadow_delta_h_pred_minus_imu_dynamic_deg, 4);
    if (!isfinite(e.shadow_h_free_imu_pred_deg)) json += ",\"shadow_H_free_imu_pred_deg\":null";
    else json += ",\"shadow_H_free_imu_pred_deg\":" + String(e.shadow_h_free_imu_pred_deg, 4);
    if (!isfinite(e.shadow_h_free_video_pred_deg)) json += ",\"shadow_H_free_video_pred_deg\":null";
    else json += ",\"shadow_H_free_video_pred_deg\":" + String(e.shadow_h_free_video_pred_deg, 4);
    if (!isfinite(e.shadow_h_post_pred_deg)) json += ",\"shadow_H_post_pred_deg\":null";
    else json += ",\"shadow_H_post_pred_deg\":" + String(e.shadow_h_post_pred_deg, 4);
    if (!isfinite(e.shadow_h_post_pred_q0_deg)) json += ",\"shadow_H_post_pred_q0_deg\":null";
    else json += ",\"shadow_H_post_pred_q0_deg\":" + String(e.shadow_h_post_pred_q0_deg, 4);
    if (!isfinite(e.shadow_h_post_pred_q05_deg)) json += ",\"shadow_H_post_pred_q05_deg\":null";
    else json += ",\"shadow_H_post_pred_q05_deg\":" + String(e.shadow_h_post_pred_q05_deg, 4);
    if (!isfinite(e.shadow_h_post_pred_q10_deg)) json += ",\"shadow_H_post_pred_q10_deg\":null";
    else json += ",\"shadow_H_post_pred_q10_deg\":" + String(e.shadow_h_post_pred_q10_deg, 4);
    if (!isfinite(e.shadow_h_post_pred_q15_deg)) json += ",\"shadow_H_post_pred_q15_deg\":null";
    else json += ",\"shadow_H_post_pred_q15_deg\":" + String(e.shadow_h_post_pred_q15_deg, 4);
    if (!isfinite(e.shadow_h_supported_min_pred_deg)) json += ",\"shadow_H_supported_min_pred_deg\":null";
    else json += ",\"shadow_H_supported_min_pred_deg\":" + String(e.shadow_h_supported_min_pred_deg, 4);
    if (!isfinite(e.shadow_h_supported_max_pred_deg)) json += ",\"shadow_H_supported_max_pred_deg\":null";
    else json += ",\"shadow_H_supported_max_pred_deg\":" + String(e.shadow_h_supported_max_pred_deg, 4);
    if (!isfinite(e.shadow_h_ref_deg)) json += ",\"shadow_H_ref_deg\":null";
    else json += ",\"shadow_H_ref_deg\":" + String(e.shadow_h_ref_deg, 4);
    if (!isfinite(e.shadow_q_req_raw_mA_s)) json += ",\"shadow_q_req_raw_mA_s\":null";
    else json += ",\"shadow_q_req_raw_mA_s\":" + String(e.shadow_q_req_raw_mA_s, 4);
    if (!isfinite(e.shadow_q_req_clamped_mA_s)) json += ",\"shadow_q_req_clamped_mA_s\":null";
    else json += ",\"shadow_q_req_clamped_mA_s\":" + String(e.shadow_q_req_clamped_mA_s, 4);
    if (!isfinite(e.shadow_recommended_q_mA_s)) json += ",\"shadow_recommended_q_mA_s\":null";
    else json += ",\"shadow_recommended_q_mA_s\":" + String(e.shadow_recommended_q_mA_s, 4);
    json += ",\"shadow_q_req_region\":" + String(e.shadow_q_req_region);
    json += ",\"shadow_q_req_reason\":" + String(e.shadow_q_req_reason);
    json += ",\"shadow_v57_q_req_reason\":" + String(e.shadow_v57_q_req_reason);
    json += ",\"shadow_v58_repeatability_gate_reason\":" + String(e.shadow_v58_repeatability_gate_reason);
    json += ",\"v59_block_index\":" + String(e.v59_block_index);
    json += ",\"v59_block_order\":" + String(e.v59_block_order);
    json += ",\"v59_state_gate_reason\":" + String(e.v59_state_gate_reason);
    json += ",\"shadow_reachability_reason\":" + String(e.shadow_reachability_reason);
    json += ",\"shadow_recommended_action\":" + String(e.shadow_recommended_action);
    json += ",\"shadow_q_support_reason\":" + String(e.shadow_q_support_reason);
    json += ",\"shadow_state_valid\":" + String(e.shadow_state_valid ? "true" : "false");
    json += ",\"shadow_h_in_support\":" + String(e.shadow_h_in_support ? "true" : "false");
    json += ",\"shadow_rate_in_support\":" + String(e.shadow_rate_in_support ? "true" : "false");
    json += ",\"shadow_state_in_support\":" + String(e.shadow_state_in_support ? "true" : "false");
    json += ",\"shadow_q_in_model_support\":" + String(e.shadow_q_in_model_support ? "true" : "false");
    json += ",\"shadow_q_req_in_model_support\":" + String(e.shadow_q_req_in_model_support ? "true" : "false");
    json += ",\"shadow_q_req_in_control_candidate_range\":" + String(e.shadow_q_req_in_control_candidate_range ? "true" : "false");
    json += ",\"shadow_q_req_candidate_valid\":" + String(e.shadow_q_req_candidate_valid ? "true" : "false");
    json += ",\"shadow_v58_repeatability_gate_passed\":" + String(e.shadow_v58_repeatability_gate_passed ? "true" : "false");
    json += ",\"v59_state_gate_passed\":" + String(e.v59_state_gate_passed ? "true" : "false");
    json += ",\"shadow_imu_proxy_residual_valid\":" + String(e.shadow_imu_proxy_residual_valid ? "true" : "false");
    json += ",\"shadow_delta_h_valid\":" + String(e.shadow_delta_h_valid ? "true" : "false");
    json += ",\"shadow_H_free_valid\":" + String(e.shadow_h_free_valid ? "true" : "false");
    json += ",\"shadow_H_free_video_valid\":" + String(e.shadow_h_free_video_valid ? "true" : "false");
    json += ",\"shadow_H_post_valid\":" + String(e.shadow_h_post_valid ? "true" : "false");
    json += ",\"shadow_H_post_candidates_valid\":" + String(e.shadow_h_post_candidates_valid ? "true" : "false");
    json += ",\"shadow_q0_extrapolated\":" + String(e.shadow_q0_extrapolated ? "true" : "false");
    json += ",\"shadow_q_req_valid\":" + String(e.shadow_q_req_valid ? "true" : "false");
    json += ",\"shadow_href_in_current_control_candidate_reachable_range\":" + String(e.shadow_href_in_current_control_candidate_reachable_range ? "true" : "false");
    json += ",\"shadow_reachability_valid\":" + String(e.shadow_reachability_valid ? "true" : "false");
    json += ",\"shadow_future_control_eligible\":" + String(e.shadow_future_control_eligible ? "true" : "false");
    json += ",\"shadow_control_candidate\":" + String(e.shadow_control_candidate ? "true" : "false");
    if (e.gain_cdeg_per_mAs == LOG_NAN_I16) json += ",\"gain_deg_per_mA_s\":null";
    else json += ",\"gain_deg_per_mA_s\":" + String(e.gain_cdeg_per_mAs / 100.0f, 3);
    json += ",\"post_pulse\":" + String(e.post_pulse ? "true" : "false");
    json += ",\"in_support\":" + String(e.in_support ? "true" : "false");
    json += ",\"side_matched\":" + String(e.side_matched ? "true" : "false");
    json += ",\"direction_matches_base\":" + String(e.direction_matches_base ? "true" : "false");
    json += ",\"positive_gain\":" + String(e.positive_gain ? "true" : "false");
    json += ",\"gain_stored\":" + String(e.gain_stored ? "true" : "false") + "}";
  }
  json += "],";
  json += "\"calibration_state_gate_events\":[";
  for (uint8_t i = 0; i < calibration_state_gate_event_count_; ++i) {
    const CalibrationStateGateEvent& e = calibration_state_gate_events_[i];
    if (i) json += ",";
    json += "{\"event_index\":" + String(e.event_index);
    json += ",\"planned_probe_index\":" + String(e.planned_probe_index);
    json += ",\"q_level_index\":" + String(e.q_level_index);
    json += ",\"reason\":" + String(e.reason);
    json += ",\"action\":" + String(e.action);
    json += ",\"wait_halfcycle_count\":" + String(e.wait_halfcycle_count);
    json += ",\"rebuild_total_count\":" + String(e.rebuild_total_count);
    json += ",\"crossing_ms\":" + String(e.crossing_ms);
    if (e.hprev_cdeg == LOG_NAN_I16) json += ",\"Hprev_imu_deg\":null";
    else json += ",\"Hprev_imu_deg\":" + String(e.hprev_cdeg / 100.0f, 3);
    if (e.cprev_cdeg == LOG_NAN_I16) json += ",\"Cprev_imu_deg\":null";
    else json += ",\"Cprev_imu_deg\":" + String(e.cprev_cdeg / 100.0f, 3);
    if (e.rate_abs_cdps == LOG_NAN_I16) json += ",\"rate_abs_dps\":null";
    else json += ",\"rate_abs_dps\":" + String(e.rate_abs_cdps / 100.0f, 3);
    json += ",\"next_peak_side\":" + String(e.next_peak_side);
    json += ",\"state_valid\":" + String(e.state_valid ? "true" : "false");
    json += ",\"dynamic_h_in_support\":" + String(e.dynamic_h_in_support ? "true" : "false");
    json += ",\"v60_cooldown_free_decay\":" + String(e.v60_cooldown_free_decay ? "true" : "false");
    json += ",\"v60_gate_skipped_by_decay\":" + String(e.v60_gate_skipped_by_decay ? "true" : "false") + "}";
  }
  json += "],";  json += "\"identification_events\":[";
  for (uint16_t i = 0; i < identification_event_count_; ++i) {
    const IdentificationEvent& e = identification_events_[i];
    if (i) json += ",";
    json += "{\"event_id\":" + String(e.event_id);
    json += ",\"start_ms\":" + String(e.start_ms) + ",\"peak_ms\":" + String(e.peak_ms);
    json += ",\"theta0_deg\":" + String(e.theta0_cdeg / 100.0f, 3);
    json += ",\"rate0_dps\":" + String(e.rate0_cdps / 100.0f, 3);
    json += ",\"theta_peak_deg\":" + String(e.theta_peak_cdeg / 100.0f, 3);
    json += ",\"direction\":" + String(e.direction) + ",\"vbat_mV\":" + String(e.vbat_mV);
    json += ",\"i0_estimated_mA\":" + String(e.i0_mA) + ",\"pulse_width_ms\":" + String(e.pulse_width_ms);
    json += ",\"q_target_mA_s\":" + String(e.q_target_mAms / 1000.0f, 4);
    json += ",\"q_requested_mA_s\":" + String(e.q_requested_mAms / 1000.0f, 4);
    json += ",\"q_command_mA_s\":" + String(e.q_command_mAms / 1000.0f, 4);
    json += ",\"q_estimated_signed_mA_s\":" + String(e.q_estimated_mAms / 1000.0f, 4);
    json += ",\"q_predicted_signed_mA_s\":" + String(e.q_estimated_mAms / 1000.0f, 4);
    json += ",\"a_target_deg\":" + String(e.a_target_cdeg / 100.0f, 3);
    json += ",\"a_pred_deg\":" + String(e.a_pred_cdeg / 100.0f, 3);
    json += ",\"a_predicted_next_deg\":" + String(e.a_pred_cdeg / 100.0f, 3);
    json += ",\"a_pred_shadow_deg\":" + String(e.a_pred_shadow_cdeg / 100.0f, 3);
    if (e.a_pred_calibrated_cdeg == LOG_NAN_I16) json += ",\"a_pred_calibrated_deg\":null";
    else json += ",\"a_pred_calibrated_deg\":" + String(e.a_pred_calibrated_cdeg / 100.0f, 3);
    if (e.prev_peak_abs_cdeg == LOG_NAN_I16) json += ",\"prev_peak_abs_deg\":null";
    else json += ",\"prev_peak_abs_deg\":" + String(e.prev_peak_abs_cdeg / 100.0f, 3);
    json += ",\"next_peak_side\":" + String(e.next_peak_side);
    json += ",\"q_effective_pred_mA_s\":" + String(e.q_effective_pred_mAms / 1000.0f, 4);
    json += ",\"calibrated_prediction_valid\":" + String(e.calibrated_prediction_valid ? "true" : "false");
    json += ",\"a_min_pred_deg\":" + String(e.a_min_cdeg / 100.0f, 3);
    json += ",\"a_max_pred_deg\":" + String(e.a_max_cdeg / 100.0f, 3);
    json += ",\"target_reachable\":" + String(e.target_reachable ? "true" : "false");
    json += ",\"bootstrap\":" + String(e.bootstrap ? "true" : "false");
    json += ",\"imu_peak_abs_deg\":" + String(fabsf(e.theta_peak_cdeg / 100.0f), 3);
    json += ",\"a_next_imu_abs_deg\":" + String(fabsf(e.theta_peak_cdeg / 100.0f), 3);
    json += ",\"pulse_suppressed\":" + String(e.pulse_suppressed ? "true" : "false");
    json += ",\"peak_detected\":" + String(e.peak_detected ? "true" : "false") + "}";
  }
  json += "],";
  json += "\"e2_shadow_peak_events\":[";
  for (uint16_t i = 0; i < e2_shadow_peak_event_count_; ++i) {
    const E2ShadowPeakEvent& e = e2_shadow_peak_events_[i];
    if (i) json += ",";
    json += "{\"event_index\":" + String(e.event_index);
    json += ",\"e2_armed\":" + String(e.e2_armed ? "true" : "false");
    json += ",\"release_detected_ms\":" + String(e.release_detected_ms);
    json += ",\"turn_index\":" + String(e.turn_index);    json += ",\"candidate_peak_ms\":" + String(e.candidate_peak_ms);
    json += ",\"confirmed_ms\":" + String(e.confirmed_ms);
    json += ",\"turn_side_from_rate\":" + String(e.turn_side_from_rate);
    json += ",\"alternating_side_ok\":" + String(e.alternating_side_ok ? "true" : "false");
    auto appendNullable = [&json](const char* name, float value, uint8_t decimals) {
      json += ",\"";
      json += name;
      json += "\":";
      if (isfinite(value)) json += String(value, static_cast<unsigned int>(decimals));
      else json += "null";
    };
    appendNullable("previous_peak", e.previous_peak_deg, 4);
    appendNullable("current_peak", e.current_peak_deg, 4);
    appendNullable("halfcycle_gyro_integral_deg", e.halfcycle_gyro_integral_deg, 4);
    appendNullable("H_prev_gyro_deg", e.h_prev_gyro_deg, 4);
    appendNullable("H_prev", e.h_prev_deg, 4);
    appendNullable("H_next_E2_deg", e.h_next_e2_explicit_deg, 4);
    appendNullable("H_next_E2", e.h_next_e2_deg, 4);
    appendNullable("static_anchor_abs_deg", e.static_anchor_abs_deg, 4);
    appendNullable("peak_gyro_integrated_abs_deg", e.peak_gyro_integrated_abs_deg, 4);
    appendNullable("peak_accel_abs_diag_deg", e.peak_accel_abs_diag_deg, 4);
    appendNullable("passive_next_peak", e.passive_next_peak_deg, 4);
    json += ",\"physical_next_peak_side\":" + String(e.physical_next_peak_side);

    appendNullable("target_peak", e.target_peak_deg, 4);
    appendNullable("delta_A_required", e.delta_a_required_deg, 4);
    appendNullable("shadow_gain", e.shadow_gain_deg_per_mA_s, 10);
    appendNullable("Q_req_shadow", e.q_req_shadow_mA_s, 5);
    json += ",\"shadow_valid\":" + String(e.shadow_valid ? "true" : "false");
    json += ",\"shadow_invalid_reason\":\"" +
        String(e2ShadowInvalidReasonName(e.shadow_invalid_reason)) + "\"";
    json += ",\"shadow_invalid_reason_code\":" + String(e.shadow_invalid_reason);
    json += "}";
  }
  json += "],";
  json += "\"q1_shadow_events\":[";
  for (uint16_t i = 0; i < q1_shadow_event_count_; ++i) {
    const Q1ShadowEvent& e = q1_shadow_events_[i];
    if (i) json += ",";
    json += "{\"q1_shadow_event_index\":" + String(e.q1_shadow_event_index);
    json += ",\"zero_cross_time_ms\":" + String(e.zero_cross_time_ms);
    auto appendQ1Nullable = [&json](const char* name, float value, uint8_t decimals) {
      json += ",\"";
      json += name;
      json += "\":";
      if (isfinite(value)) json += String(value, static_cast<unsigned int>(decimals));
      else json += "null";
    };
    appendQ1Nullable("zero_cross_rate_dps", e.zero_cross_rate_dps, 4);
    appendQ1Nullable("zero_cross_abs_rate_dps", e.zero_cross_abs_rate_dps, 4);
    json += ",\"physical_next_peak_side\":" + String(e.physical_next_peak_side);

    json += ",\"detector_crossing_direction\":" + String(e.detector_crossing_direction);
    appendQ1Nullable("detector_angle_before_deg", e.detector_angle_before_deg, 5);
    appendQ1Nullable("detector_angle_after_deg", e.detector_angle_after_deg, 5);
    appendQ1Nullable("crossing_interpolation_alpha", e.crossing_interpolation_alpha, 6);
    json += ",\"interpolated_zero_cross_time_ms\":" +
        String(e.interpolated_zero_cross_time_ms);
    appendQ1Nullable("physical_roll_rate_before_dps", e.physical_roll_rate_before_dps, 4);
    appendQ1Nullable("physical_roll_rate_after_dps", e.physical_roll_rate_after_dps, 4);
    appendQ1Nullable("interpolated_physical_roll_rate_dps",
                     e.interpolated_physical_roll_rate_dps, 4);
    json += ",\"sign_gate_passed\":" + String(e.sign_gate_passed ? "true" : "false");
    appendQ1Nullable("q1_intercept_deg", e.q1_intercept_deg, 5);
    appendQ1Nullable("q1_rate_term_deg", e.q1_rate_term_deg, 5);
    appendQ1Nullable("q1_side_term_deg", e.q1_side_term_deg, 5);
    appendQ1Nullable("q1_baseline_next_peak_abs_deg", e.q1_baseline_next_peak_abs_deg, 5);
    appendQ1Nullable("target_next_peak_abs_deg", e.target_next_peak_abs_deg, 5);
    appendQ1Nullable("delta_peak_required_deg", e.delta_peak_required_deg, 5);
    appendQ1Nullable("q1_gain_deg_per_mAs", e.q1_gain_deg_per_mA_s, 5);
    appendQ1Nullable("q_model_axis_mA_s", e.q_model_axis_mA_s, 5);
    appendQ1Nullable("q_req_shadow_mA_s", e.q_req_shadow_mA_s, 5);
    json += ",\"q1_shadow_valid\":" + String(e.q1_shadow_valid ? "true" : "false");
    json += ",\"q1_shadow_invalid_reason\":\"" +
        String(q1ShadowInvalidReasonName(e.q1_shadow_invalid_reason)) + "\"";
    json += ",\"q1_shadow_invalid_reason_code\":" + String(e.q1_shadow_invalid_reason);
    json += "}";
  }
  json += "],";
  json += "\"q_ident_events\":[";
  for (uint16_t i = 0; i < q_ident_event_count_; ++i) {
    const QIdentEvent& e = q_ident_events_[i];
    if (i) json += ",";
    json += "{\"q_ident_event_index\":" + String(e.q_ident_event_index);
    json += ",\"q_ident_run_schedule_id\":" + String(e.q_ident_run_schedule_id);
    json += ",\"zero_cross_time_ms\":" + String(e.zero_cross_time_ms);
    auto appendQIdentNullable = [&json](const char* name, float value, uint8_t decimals) {
      json += ",\"";
      json += name;
      json += "\":";
      if (isfinite(value)) json += String(value, static_cast<unsigned int>(decimals));
      else json += "null";
    };
    appendQIdentNullable("zero_cross_rate_dps", e.zero_cross_rate_dps, 4);
    appendQIdentNullable("zero_cross_abs_rate_dps", e.zero_cross_abs_rate_dps, 4);
    json += ",\"interpolated_zero_cross_time_ms\":" + String(e.interpolated_zero_cross_time_ms);
    appendQIdentNullable("interpolated_physical_roll_rate_dps", e.interpolated_physical_roll_rate_dps, 4);
    json += ",\"detector_crossing_direction\":" + String(e.detector_crossing_direction);
    appendQIdentNullable("detector_angle_before_deg", e.detector_angle_before_deg, 5);
    appendQIdentNullable("detector_angle_after_deg", e.detector_angle_after_deg, 5);
    json += ",\"sign_gate_passed\":" + String(e.sign_gate_passed ? "true" : "false");
    appendQIdentNullable("physical_roll_abs_diag_deg", e.physical_roll_abs_diag_deg, 4);
    appendQIdentNullable("current_roll_deg", e.current_roll_deg, 4);
    appendQIdentNullable("physical_roll_rate_dps", e.physical_roll_rate_dps, 4);
    json += ",\"physical_next_peak_side\":" + String(e.physical_next_peak_side);

    json += ",\"side_occurrence_index\":" + String(e.side_occurrence_index);
    json += ",\"q_ident_armed\":" + String(e.q_ident_armed ? "true" : "false");
    json += ",\"arm_consecutive_count\":" + String(e.arm_consecutive_count);
    appendQIdentNullable("q_schedule_target_mA_s", e.q_schedule_target_mA_s, 4);
    json += ",\"q_command_direction\":" + String(e.q_command_direction);
    json += ",\"command_current_mA\":" + String(e.command_current_mA);
    json += ",\"pulse_width_ms\":" + String(e.pulse_width_ms);
    appendQIdentNullable("q_target_mA_s", e.q_target_mA_s, 4);
    appendQIdentNullable("q_effective_pred_mA_s", e.q_effective_pred_mA_s, 4);
    json += ",\"vbat_mV\":" + String(e.vbat_mV);
    appendQIdentNullable("i0_estimated_mA", e.i0_estimated_mA, 4);
    appendQIdentNullable("solver_required_width_ms", e.solver_required_width_ms, 4);
    json += ",\"solver_selected_integer_width_ms\":" + String(e.solver_selected_integer_width_ms);
    json += ",\"pulse_width_guard_max_ms\":" + String(e.pulse_width_guard_max_ms);
    json += ",\"pulse_start_ms\":" + String(e.pulse_start_ms);
    json += ",\"pulse_end_ms\":" + String(e.pulse_end_ms);
    json += ",\"q_ident_valid\":" + String(e.q_ident_valid ? "true" : "false");
    json += ",\"q_ident_invalid_reason\":\"" + String(qIdentInvalidReasonName(e.q_ident_invalid_reason)) + "\"";
    json += ",\"q_ident_invalid_reason_code\":" + String(e.q_ident_invalid_reason);
    json += "}";
  }
  json += "],";
  json += "\"energy_control_v0_events\":[";
  for (uint16_t i = 0; i < energy_control_v0_event_count_; ++i) {
    const EnergyControlV0Event& e = energy_control_v0_events_[i];
    if (i) json += ",";
    json += "{\"event_index\":" + String(e.event_index);
    json += ",\"zero_cross_time_ms\":" + String(e.zero_cross_time_ms);
    auto appendEnergyNullable = [&json](const char* name, float value, uint8_t decimals) {
      json += ",\""; json += name; json += "\":";
      if (isfinite(value)) json += String(value, static_cast<unsigned int>(decimals));
      else json += "null";
    };
    appendEnergyNullable("zero_cross_rate_dps", e.zero_cross_rate_dps, 4);
    appendEnergyNullable("zero_cross_abs_rate_dps", e.zero_cross_abs_rate_dps, 4);
    json += ",\"physical_next_peak_side\":" + String(e.physical_next_peak_side);

    json += ",\"output_gate_state\":" + String(e.output_gate_state);
    json += ",\"previous_accepted_physical_next_peak_side\":" +
        String(e.previous_accepted_physical_next_peak_side);
    json += ",\"candidate_physical_next_peak_side\":" +
        String(e.candidate_physical_next_peak_side);
    json += ",\"rearm_excursion_seen\":" + String(e.rearm_excursion_seen ? "true" : "false");
    appendEnergyNullable("maximum_excursion_since_previous_cross_deg",
                         e.maximum_excursion_since_previous_cross_deg, 5);
    appendEnergyNullable("rearm_threshold_deg", e.rearm_threshold_deg, 5);
    json += ",\"side_alternation_passed\":" + String(e.side_alternation_passed ? "true" : "false");
    json += ",\"output_authorized\":" + String(e.output_authorized ? "true" : "false");
    json += ",\"output_blocked_reason\":\"" +
        String(energyControlV0ReasonName(e.output_blocked_reason)) + "\"";
    json += ",\"output_blocked_reason_code\":" + String(e.output_blocked_reason);
    json += ",\"q_command_direction\":" + String(e.q_command_direction);
    appendEnergyNullable("passive_next_peak_abs_deg", e.passive_next_peak_abs_deg, 5);
    appendEnergyNullable("passive_energy_j", e.passive_energy_j, 8);
    appendEnergyNullable("target_peak_abs_deg", e.target_peak_abs_deg, 5);
    appendEnergyNullable("target_energy_j", e.target_energy_j, 8);
    appendEnergyNullable("delta_energy_required_j", e.delta_energy_required_j, 8);
    appendEnergyNullable("q1_gain_deg_per_mA_s", e.q1_gain_deg_per_mA_s, 5);
    appendEnergyNullable("q_selected_mA_s", e.q_selected_mA_s, 5);
    appendEnergyNullable("q_effective_pred_mA_s", e.q_effective_pred_mA_s, 5);
    appendEnergyNullable("predicted_next_peak_abs_deg", e.predicted_next_peak_abs_deg, 5);
    appendEnergyNullable("predicted_next_energy_j", e.predicted_next_energy_j, 8);
    json += ",\"q_saturated_at_max\":" + String(e.q_saturated_at_max ? "true" : "false");
    json += ",\"rate_support_status\":" + String(e.rate_support_status);
    json += ",\"q_support_status\":" + String(e.q_support_status);
    json += ",\"vbat_mV\":" + String(e.vbat_mV);
    appendEnergyNullable("i0_estimated_mA", e.i0_estimated_mA, 4);
    appendEnergyNullable("solver_required_width_ms", e.solver_required_width_ms, 4);
    json += ",\"solver_selected_integer_width_ms\":" + String(e.solver_selected_integer_width_ms);
    json += ",\"command_current_mA\":" + String(e.command_current_mA);
    json += ",\"pulse_width_ms\":" + String(e.pulse_width_ms);
    json += ",\"pulse_start_ms\":" + String(e.pulse_start_ms);
    json += ",\"pulse_end_ms\":" + String(e.pulse_end_ms);
    json += ",\"output_executed\":" + String(e.output_executed ? "true" : "false");
    json += ",\"valid\":" + String(e.valid ? "true" : "false");
    json += ",\"reason\":\"" + String(energyControlV0ReasonName(e.reason)) + "\"";
    json += ",\"reason_code\":" + String(e.reason);
    json += "}";
  }
  json += "],";
  json += "\"energy_control_autonomous_peak_events\":[";
  bool metadata_event_detail_truncated = false;
  uint16_t autonomous_peak_event_rendered_count = 0;
  uint16_t autonomous_zero_cross_event_rendered_count = 0;
  auto appendAutonomousDetail = [&json, &metadata_event_detail_truncated](
      const String& detail, bool* has_rendered_detail) -> bool {
    const size_t separator_bytes = *has_rendered_detail ? 1U : 0U;
    if (json.length() + separator_bytes + detail.length() +
        kMetadataJsonTailReserveBytes > kMetadataJsonHardBudgetBytes) {
      metadata_event_detail_truncated = true;
      return false;
    }
    if (*has_rendered_detail) json += ",";
    json += detail;
    *has_rendered_detail = true;
    return true;
  };
  bool has_rendered_peak_detail = false;
  for (uint16_t i = 0; i < energy_control_autonomous_peak_event_count_; ++i) {
    const EnergyControlAutonomousPeakEvent& e = energy_control_autonomous_peak_events_[i];
    String detail = "{\"peak_index\":" + String(e.peak_index);
    detail += ",\"peak_time_ms\":" + String(e.peak_time_ms);
    detail += ",\"physical_peak_side\":" + String(e.physical_peak_side);
    detail += ",\"peak_amplitude_deg\":" + String(e.peak_amplitude_deg, 5);
    if (isfinite(e.detector_peak_angle_deg)) detail += ",\"detector_peak_angle_deg\":" + String(e.detector_peak_angle_deg, 5);
    else detail += ",\"detector_peak_angle_deg\":null";
    detail += ",\"target_peak_deg\":" + String(e.target_peak_deg, 5);
    detail += ",\"peak_error_deg\":" + String(e.peak_error_deg, 5);
    detail += ",\"phase\":" + String(e.phase);
    detail += ",\"integral_plus_mA_s\":" + String(e.integral_plus_mA_s, 5);
    detail += ",\"integral_minus_mA_s\":" + String(e.integral_minus_mA_s, 5);
    detail += ",\"first_peak\":" + String(e.first_peak ? "true" : "false");
    detail += ",\"pending_command_matched\":" + String(e.pending_command_matched ? "true" : "false");
    if (isfinite(e.pending_q_command_mA_s)) detail += ",\"pending_q_command_mA_s\":" + String(e.pending_q_command_mA_s, 5);
    else detail += ",\"pending_q_command_mA_s\":null";
    detail += ",\"antiwindup_upper_hold\":" + String(e.antiwindup_upper_hold ? "true" : "false");
    detail += ",\"antiwindup_lower_hold\":" + String(e.antiwindup_lower_hold ? "true" : "false") + "}";
    if (!appendAutonomousDetail(detail, &has_rendered_peak_detail)) break;
    ++autonomous_peak_event_rendered_count;
  }
  json += "],";
  json += "\"energy_control_autonomous_zero_cross_events\":[";
  bool has_rendered_zero_cross_detail = false;
  for (uint16_t i = 0; i < energy_control_autonomous_zero_cross_event_count_; ++i) {
    const EnergyControlAutonomousZeroCrossEvent& e = energy_control_autonomous_zero_cross_events_[i];
    String detail = "{\"event_index\":" + String(e.event_index);
    detail += ",\"event_kind\":\"" + String(e.event_kind == 1 ? "strong_start_kick" : "energy_control_zero_cross") + "\"";
    detail += ",\"zero_cross_time_ms\":" + String(e.zero_cross_time_ms);
    auto appendAutonomousNullable = [&detail](const char* name, float value, uint8_t decimals) {
      detail += ",\""; detail += name; detail += "\":";
      if (isfinite(value)) detail += String(value, static_cast<unsigned int>(decimals)); else detail += "null";
    };
    appendAutonomousNullable("zero_cross_rate_dps", e.zero_cross_rate_dps, 4);
    appendAutonomousNullable("zero_cross_abs_rate_dps", e.zero_cross_abs_rate_dps, 4);
    appendAutonomousNullable("detector_angle_before_deg", e.detector_angle_before_deg, 5);
    appendAutonomousNullable("detector_angle_after_deg", e.detector_angle_after_deg, 5);
    appendAutonomousNullable("detector_crossing_alpha", e.detector_crossing_alpha, 5);
    appendAutonomousNullable("interpolated_crossing_time_ms", e.interpolated_crossing_time_ms, 4);
    detail += ",\"rate_support_diagnostic\":" + String(e.rate_support_diagnostic ? "true" : "false");
    detail += ",\"previous_peak_time_ms\":" + String(e.previous_peak_time_ms);
    detail += ",\"previous_peak_side\":" + String(e.previous_peak_side);
    appendAutonomousNullable("previous_peak_amplitude_deg", e.previous_peak_amplitude_deg, 5);
    detail += ",\"physical_next_peak_side\":" + String(e.physical_next_peak_side);
    detail += ",\"side_mismatch_diagnostic\":" + String(e.side_mismatch_diagnostic ? "true" : "false");
    detail += ",\"phase\":" + String(e.phase);
    appendAutonomousNullable("free_next_peak_amplitude_deg", e.free_next_peak_amplitude_deg, 5);
    detail += ",\"free_model_revision\":\"" + String(Config::ENERGY_CONTROL_AUTONOMOUS_FREE_MODEL_REVISION) + "\"";
    appendAutonomousNullable("passive_energy_j", e.passive_energy_j, 8);
    appendAutonomousNullable("target_peak_deg", e.target_peak_deg, 5);
    appendAutonomousNullable("target_energy_j", e.target_energy_j, 8);
    appendAutonomousNullable("delta_energy_required_j", e.delta_energy_required_j, 8);
    appendAutonomousNullable("q1_gain_deg_per_mA_s", e.q1_gain_deg_per_mA_s, 5);
    appendAutonomousNullable("q_ff_energy_mA_s", e.q_ff_energy_mA_s, 5);
    appendAutonomousNullable("q_angle_diagnostic_mA_s", e.q_angle_diagnostic_mA_s, 5);
    appendAutonomousNullable("integral_side_mA_s", e.integral_side_mA_s, 5);
    appendAutonomousNullable("q_unclamped_mA_s", e.q_unclamped_mA_s, 5);
    appendAutonomousNullable("q_command_mA_s", e.q_command_mA_s, 5);
    appendAutonomousNullable("q_effective_pred_mA_s", e.q_effective_pred_mA_s, 5);
    appendAutonomousNullable("q_available_mA_s", e.q_available_mA_s, 5);
    detail += ",\"q_gain_extrapolated\":" + String(e.q_gain_extrapolated ? "true" : "false");
    appendAutonomousNullable("a_pred_base_deg", e.a_pred_base_deg, 5);
    appendAutonomousNullable("a_pred_corrected_deg", e.a_pred_corrected_deg, 5);
    appendAutonomousNullable("correction_deg", e.correction_deg, 5);
    appendAutonomousNullable("c_side_used_deg", e.c_side_used_deg, 5);
    appendAutonomousNullable("g_side_base_deg_per_mA_s", e.g_side_base_deg_per_mA_s, 5);
    appendAutonomousNullable("g_side_corrected_deg_per_mA_s", e.g_side_corrected_deg_per_mA_s, 5);
    appendAutonomousNullable("correction_blend_lambda", e.correction_blend_lambda, 5);
    appendAutonomousNullable("predicted_next_peak_amplitude_deg", e.predicted_next_peak_amplitude_deg, 5);
    appendAutonomousNullable("predicted_energy_j", e.predicted_energy_j, 8);
    detail += ",\"q_saturated_upper\":" + String(e.q_saturated_upper ? "true" : "false");
    detail += ",\"q_saturated_lower\":" + String(e.q_saturated_lower ? "true" : "false");
    detail += ",\"q_command_direction\":" + String(e.q_command_direction);
    detail += ",\"command_matches_zero_cross_motion\":" +
        String(e.command_matches_zero_cross_motion ? "true" : "false");
    detail += ",\"vbat_mV\":" + String(e.vbat_mV);
    appendAutonomousNullable("i0_estimated_mA", e.i0_estimated_mA, 4);
    appendAutonomousNullable("solver_required_width_ms", e.solver_required_width_ms, 4);
    detail += ",\"solver_selected_integer_width_ms\":" + String(e.solver_selected_integer_width_ms);
    detail += ",\"command_current_mA\":" + String(e.command_current_mA);
    detail += ",\"pulse_width_ms\":" + String(e.pulse_width_ms);
    detail += ",\"pulse_start_ms\":" + String(e.pulse_start_ms);
    detail += ",\"pulse_end_ms\":" + String(e.pulse_end_ms);
    detail += ",\"output_executed\":" + String(e.output_executed ? "true" : "false");
    detail += ",\"valid\":" + String(e.valid ? "true" : "false");
    detail += ",\"reason\":\"" + String(energyControlAutonomousReasonName(e.reason)) + "\"";
    detail += ",\"reason_code\":" + String(e.reason) + "}";
    if (!appendAutonomousDetail(detail, &has_rendered_zero_cross_detail)) break;
    ++autonomous_zero_cross_event_rendered_count;
  }
  json += "],";
  json += "\"metadata_event_detail_truncated\":" + String(metadata_event_detail_truncated ? "true" : "false") + ",";
  json += "\"energy_control_autonomous_peak_event_total_count\":" +
      String(energy_control_autonomous_peak_event_count_) + ",";
  json += "\"energy_control_autonomous_peak_event_rendered_count\":" +
      String(autonomous_peak_event_rendered_count) + ",";
  json += "\"energy_control_autonomous_zero_cross_event_total_count\":" +
      String(energy_control_autonomous_zero_cross_event_count_) + ",";
  json += "\"energy_control_autonomous_zero_cross_event_rendered_count\":" +
      String(autonomous_zero_cross_event_rendered_count) + ",";
  json += "\"metadata_json_budget_bytes\":" + String(kMetadataJsonHardBudgetBytes) + ",";
  json += "\"metadata_json_reserve_bytes\":" + String(kMetadataJsonReserveBytes) + ",";
    json += "\"states\":{";
  json += "\"0\":\"STARTUP_GYRO_CALIB\",\"1\":\"MADGWICK_SETTLING\",\"2\":\"READY_TO_MEASURE\",";
  json += "\"3\":\"RUNNING_BATCH_SWEEP\",\"4\":\"FINISHED\",\"5\":\"ESTOP\",";
  json += "\"6\":\"START_SYNC\",\"7\":\"END_SYNC\",\"8\":\"TRIAL_REST\"},";
  json += "\"beta_phase_states\":{\"0\":\"idle\",\"1\":\"outbound_wait_turn\",\"2\":\"return_to_zero\"},";
  json += "\"sync_event_ids\":{";
  json += "\"0\":\"none\",\"1\":\"start_sync_pattern_active\",";
  json += "\"2\":\"experiment_start_t0_after_start_sync_pattern\",";
  json += "\"3\":\"measurement_end_before_end_sync_pattern\",";
  json += "\"4\":\"end_sync_pattern_active\",\"5\":\"periodic_mid_sync_anchor_active\",";
  json += "\"6\":\"trial_start\",\"7\":\"trial_end\"},";
  json += "\"scales\":{";
  json += "\"pitch_cdeg\":\"degrees * 100\",";
  json += "\"gyro_cdps\":\"degrees_per_second * 100\",";
  json += "\"accel_mg\":\"g * 1000\",";
  json += "\"beta_x10000\":\"beta * 10000\",";
  json += "\"acc_norm_mg\":\"g * 1000\",\"roller_battery_mV\":\"mV\",";
  json += "\"beta_model_vbat_mV\":\"mV used after fallback or clamp\"},";
  json += "\"columns\":{\"timeseries\":[";
  json += "\"time_s\",\"log_time_s\",\"t_test_ms\",\"state_id\",";
  json += "\"pulse_id\",\"pulse_active\",\"pulse_direction\",\"motor_cmd_mA\",";
  json += "\"current_mA_setting\",\"pulse_width_ms_setting\",\"input_interval_ms\",";
  json += "\"trial_index\",\"trial_count\",\"trial_elapsed_ms\",\"trial_duration_ms\",";
  json += "\"trial_current_mA\",\"trial_pulse_width_ms\",\"trial_input_interval_ms\",";
  json += "\"trial_predicted_beta_min\",\"beta_recovery_tau_s\",";
  json += "\"beta_model_vbat_mV\",\"predicted_i_goal_mA\",\"predicted_peak_current_mA\",\"beta_model_vbat_status\",";
  json += "\"beta_ceiling_fixed_b100\",\"beta_ceiling_fixed_b000\",\"beta_ceiling_dynamic_hold073\",\"beta_ceiling_dynamic_hold120\",\"beta_ceiling_dynamic_hold170\",\"beta_ceiling_dynamic_turnfast\",";
  json += "\"pitch_madgwick_beta1_raw_deg\",\"pitch_madgwick_beta1_bias_deg\",";
  json += "\"pitch_fixed_b100_deg\",\"pitch_fixed_b000_deg\",\"pitch_dynamic_hold073_deg\",\"pitch_dynamic_hold120_deg\",\"pitch_dynamic_hold170_deg\",\"pitch_dynamic_turnfast_deg\",";
  json += "\"pitch_gyro_raw_deg\",\"pitch_gyro_bias_corrected_deg\",\"pitch_accel_only_deg\",";
  json += "\"gyro_bias_x_dps\",\"gyro_bias_y_dps\",\"gyro_bias_z_dps\",\"gyro_pitch_rate_dps\",";
  json += "\"beta_target_fixed_b100\",\"beta_target_fixed_b000\",\"beta_target_dynamic_hold073\",\"beta_target_dynamic_hold120\",\"beta_target_dynamic_hold170\",\"beta_target_dynamic_turnfast\",";
  json += "\"beta_applied_fixed_b100\",\"beta_applied_fixed_b000\",\"beta_applied_dynamic_hold073\",\"beta_applied_dynamic_hold120\",\"beta_applied_dynamic_hold170\",\"beta_applied_dynamic_turnfast\",";
  json += "\"ax_g\",\"ay_g\",\"az_g\",\"gx_dps\",\"gy_dps\",\"gz_dps\",";
  json += "\"acc_norm_g\",\"roller_actual_current_mA\",\"roller_battery_mV\",\"led_state\",\"sync_event_id\",\"log_active\",";
  json += "\"turn_fast_state\",\"turn_fast_recovery_progress\",\"turn_fast_peak_angle_deg\",\"turn_fast_integrated_angle_deg\",\"turn_fast_beta_target\",";
  json += "\"physical_roll_abs_deg\",\"current_roll_deg\",\"physical_roll_rate_dps\",\"static_confirmed\",\"target_roll_deg\",\"target_error_deg\",\"ready\"]}";
  if (current_timing_audit_) appendCurrentTimingJson(json, *current_timing_audit_);
  if (q_observer_) appendQObserverJson(json, *q_observer_);
  const String final_size_key = ",\"metadata_json_final_bytes\":";
  size_t final_size = json.length() + final_size_key.length() + 2U;
  for (uint8_t i = 0; i < 4; ++i) {
    const size_t candidate_size = json.length() + final_size_key.length() +
        String(final_size).length() + 1U;
    if (candidate_size == final_size) break;
    final_size = candidate_size;
  }
  json += final_size_key + String(final_size) + "}";
  return json;
}
RwLogFileHeader PsramLogger::buildHeader(uint32_t metadata_size) const {
  RwLogFileHeader header{};
  memcpy(header.magic, "RWLOG01", 8);
  header.format_version = RWLOG_FORMAT_VERSION;
  header.header_size = sizeof(RwLogFileHeader);
  header.run_id = current_run_id_;
  header.run_start_us = run_start_us_;
  header.metadata_json_size = metadata_size;
  header.sample_count = sample_count_;
  header.summary_count = 0;
  header.event_count = q_observer_ ? q_observer_->sample_count : 0;
  header.log_sample_size = sizeof(LogSample);
  header.summary_row_size = 0;
  header.event_row_size = sizeof(QObserver::Sample);
  header.log_period_ms = q_observer_ && q_observer_->v57.enabled ? 100 : Config::LOG_PERIOD_MS;
  header.imu_period_ms = Config::IMU_PERIOD_MS;
  header.roller_read_period_ms = Config::ROLLER_READ_PERIOD_MS;
  header.web_update_period_ms = Config::WEB_UPDATE_PERIOD_MS;
  header.total_trials = q_observer_ && q_observer_->v57.enabled ? q_observer_->v57.trial_count : q_observer_ && q_observer_->v55.enabled ? 1 : Config::BETA_SWEEP_TRIAL_COUNT;
  header.preset_id = 32;
  header.flags = RWLOG_FLAG_CRC32;
  header.samples_offset = sizeof(RwLogFileHeader) + metadata_size;
  header.summaries_offset = header.samples_offset + sample_count_ * sizeof(LogSample);
  header.events_offset = header.summaries_offset;
  header.crc_offset = header.events_offset + header.event_count * sizeof(QObserver::Sample);
  return header;
}

uint32_t PsramLogger::crc32Update(uint32_t crc, const uint8_t* data, size_t len) {
  crc = ~crc;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
    }
  }
  return ~crc;
}

uint32_t PsramLogger::calculateCrc(const RwLogFileHeader& header, const String& metadata) const {
  uint32_t crc = 0;
  crc = crc32Update(crc, reinterpret_cast<const uint8_t*>(&header), sizeof(header));
  crc = crc32Update(crc, reinterpret_cast<const uint8_t*>(metadata.c_str()), metadata.length());
  crc = crc32Update(crc, reinterpret_cast<const uint8_t*>(samples_), sample_count_ * sizeof(LogSample));
  if (q_observer_) crc = crc32Update(crc, reinterpret_cast<const uint8_t*>(q_observer_->samples),
      q_observer_->sample_count * sizeof(QObserver::Sample));
  return crc;
}

bool PsramLogger::writeBytes(WebServer& server, const uint8_t* data, size_t len) {
  WiFiClient client = server.client();
  while (len > 0) {
    const size_t n = len > STREAM_CHUNK_BYTES ? STREAM_CHUNK_BYTES : len;
    if (client.write(data, n) != n) return false;
    data += n;
    len -= n;
    delay(0);
  }
  return true;
}

bool PsramLogger::streamRwLog(WebServer& server) {
  if (!rwlogDownloadable()) {
    last_error_ = "rwlog_not_ready";
    server.send(409, "text/plain", last_error_);
    return false;
  }

  downloading_ = true;
  const String metadata = buildMetadataJson();
  const RwLogFileHeader header = buildHeader(metadata.length());
  const uint32_t crc = calculateCrc(header, metadata);
  char filename[72];
  downloadFilename(filename, sizeof(filename));

  server.sendHeader("Content-Disposition", String("attachment; filename=\"") + filename + "\"");
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.setContentLength(header.crc_offset + sizeof(crc));
  server.send(200, "application/octet-stream", "");

  bool ok = true;
  ok = ok && writeBytes(server, reinterpret_cast<const uint8_t*>(&header), sizeof(header));
  ok = ok && writeBytes(server, reinterpret_cast<const uint8_t*>(metadata.c_str()), metadata.length());
  ok = ok && writeBytes(server, reinterpret_cast<const uint8_t*>(samples_), sample_count_ * sizeof(LogSample));
  if (q_observer_) ok = ok && writeBytes(server, reinterpret_cast<const uint8_t*>(q_observer_->samples),
      q_observer_->sample_count * sizeof(QObserver::Sample));
  ok = ok && writeBytes(server, reinterpret_cast<const uint8_t*>(&crc), sizeof(crc));

  downloading_ = false;
  last_error_ = ok ? "" : "rwlog_stream_failed";
  return ok;
}






