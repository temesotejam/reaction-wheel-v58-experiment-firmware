#pragma once

#include <Arduino.h>
#include <limits.h>

#include "config.h"

static constexpr int16_t LOG_NAN_I16 = INT16_MIN;
static constexpr int32_t LOG_NAN_I32 = INT32_MIN;

enum class ExperimentState : uint8_t {
  STARTUP_GYRO_CALIB = 0,
  MADGWICK_SETTLING = 1,
  READY_TO_MEASURE = 2,
  RUNNING_BATCH_SWEEP = 3,
  FINISHED = 4,
  ESTOP = 5,
  START_SYNC = 6,
  END_SYNC = 7,
  TRIAL_REST = 8
};

#pragma pack(push, 1)
struct RwLogFileHeader {
  char magic[8];
  uint16_t format_version;
  uint16_t header_size;
  uint32_t run_id;
  uint64_t run_start_us;
  uint32_t metadata_json_size;
  uint32_t sample_count;
  uint32_t summary_count;
  uint32_t event_count;
  uint16_t log_sample_size;
  uint16_t summary_row_size;
  uint16_t event_row_size;
  uint16_t log_period_ms;
  uint16_t imu_period_ms;
  uint16_t roller_read_period_ms;
  uint16_t web_update_period_ms;
  uint16_t total_trials;
  uint16_t preset_id;
  uint32_t flags;
  uint32_t samples_offset;
  uint32_t summaries_offset;
  uint32_t events_offset;
  uint32_t crc_offset;
  uint32_t reserved[8];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct LogSample {
  uint32_t time_us;
  uint32_t t_test_ms;
  uint8_t state_id;
  uint32_t pulse_id;
  uint8_t pulse_active;
  int8_t pulse_direction;
  int16_t motor_cmd_mA;
  int16_t current_mA_setting;
  uint16_t pulse_width_ms_setting;
  uint16_t input_interval_ms;
  uint8_t trial_index;
  uint8_t trial_count;
  uint32_t trial_elapsed_ms;
  uint32_t trial_duration_ms;
  int16_t trial_current_mA;
  uint16_t trial_pulse_width_ms;
  uint16_t trial_input_interval_ms;
  int16_t trial_predicted_beta_min_x10000;
  uint16_t beta_recovery_tau_ms;
  uint16_t beta_model_vbat_mV;
  int16_t predicted_i_goal_mA;
  int16_t predicted_peak_current_mA;
  uint8_t beta_model_vbat_status;
  int16_t beta_ceiling_series_x10000[Config::DYNAMIC_BETA_COUNT];
  int16_t pitch_madgwick_beta1_raw_cdeg;
  int16_t pitch_madgwick_beta1_bias_cdeg;
  int16_t pitch_dynamic_series_cdeg[Config::DYNAMIC_BETA_COUNT];
  int16_t pitch_gyro_raw_cdeg;
  int16_t pitch_gyro_bias_corrected_cdeg;
  int16_t pitch_accel_only_cdeg;
  int16_t gyro_bias_x_cdps;
  int16_t gyro_bias_y_cdps;
  int16_t gyro_bias_z_cdps;
  int16_t gyro_pitch_rate_cdps;
  int16_t beta_target_series_x10000[Config::DYNAMIC_BETA_COUNT];
  int16_t beta_applied_series_x10000[Config::DYNAMIC_BETA_COUNT];
  int16_t ax_mg;
  int16_t ay_mg;
  int16_t az_mg;
  int16_t gx_cdps;
  int16_t gy_cdps;
  int16_t gz_cdps;
  int16_t acc_norm_mg;
  int16_t roller_actual_current_mA;
  uint16_t roller_battery_mV;
  uint8_t led_state;
  uint8_t sync_event_id;
  uint8_t log_active;
  uint8_t beta_phase_state;
  int16_t beta_phase_progress_x10000;
  int16_t beta_phase_peak_angle_cdeg;
  int16_t beta_phase_angle_cdeg;
  int16_t beta_phase_ceiling_x10000;
  // RWLOG v42: calibrated physical-roll UI state. These fields are appended
  // so v41 records remain byte-for-byte interpretable by their original layout.
  int16_t physical_roll_abs_cdeg;
  int16_t current_roll_cdeg;
  int16_t physical_roll_rate_cdps;
  int16_t target_roll_cdeg;
  int16_t target_error_cdeg;
  uint8_t static_confirmed;
  uint8_t ready;
  // RWLOG v45: actual-current freshness and observed-Q diagnostics. Values are
  // appended so v44 and older sample layouts remain unchanged.
  uint32_t roller_current_sample_time_us;
  uint32_t roller_current_sequence;
  uint32_t roller_current_age_us;
  uint32_t roller_current_read_failure_count;
  int32_t roller_q_meas_observed_mAms;
  int32_t pulse_q_target_mAms;
  int32_t pulse_q_pred_mAms;
  uint16_t roller_current_sample_count;
  uint8_t roller_current_valid;
  uint8_t roller_q_meas_observed_valid;
  // RWLOG v46: execution-time audit. The values are diagnostic only and are
  // captured from the previous fully completed main loop (see metadata).
  uint32_t timing_loop_sequence;
  uint32_t timing_loop_total_us;
  uint32_t timing_m5_update_us;
  uint32_t timing_service_fast_us;
  uint32_t timing_beta_context_us;
  uint32_t timing_imu_update_total_us;
  uint32_t timing_roller_update_total_us;
  uint32_t timing_runner_update_total_us;
  uint32_t timing_web_update_us;
  uint32_t timing_imu_hw_update_us;
  uint32_t timing_imu_get_data_us;
  uint32_t timing_imu_manager_filters_us;
  uint32_t timing_runner_beta1_filters_us;
  uint32_t timing_runner_dynamic_all_us;
  uint32_t timing_adopted_filter_only_us;
  uint32_t timing_beta_turn_fast_us;
  uint32_t timing_update_filter_series_us;
  uint32_t timing_update_displayed_angles_us;
  uint32_t timing_update_current_roll_us;
  uint32_t timing_q1_shadow_us;
  uint32_t timing_autonomous_motion_us;
  uint32_t timing_fast_current_read_us;
  uint32_t timing_roller_full_status_us;
  uint32_t timing_log_sample_us;
  uint32_t timing_imu_task_start_us;
  uint32_t timing_imu_task_sequence;
  uint32_t timing_roller_full_status_start_us;
  uint32_t timing_roller_full_status_sequence;
  uint32_t timing_runner_filter_start_us;
  uint32_t timing_runner_filter_sequence;
  uint32_t timing_log_task_start_us;
  uint32_t timing_log_task_sequence;
  uint32_t timing_web_task_start_us;
  uint32_t timing_web_task_sequence;
  // RWLOG v47: on-device successful-CURRENT_READBACK gap audit for the active
  // or most recently completed pulse.  It is metadata only, never control.
  uint32_t roller_current_audit_max_gap_us;
  uint16_t roller_current_audit_gt_3333_count;
  uint16_t roller_current_audit_interval_count;
  uint32_t roller_current_audit_tail_max_gap_us;
  uint16_t roller_current_audit_tail_gt_3333_count;
  uint16_t roller_current_audit_tail_interval_count;
  uint32_t roller_current_audit_pulse_end_current_age_us;
  uint16_t roller_current_audit_imu_deadline_guard_count;
  uint16_t roller_current_audit_post_imu_service_count;
  uint8_t roller_current_audit_finalized;
  uint8_t roller_current_audit_tail_covered;
};
#pragma pack(pop)

static_assert(sizeof(RwLogFileHeader) == 110, "RwLogFileHeader binary size changed");
static_assert(sizeof(LogSample) == 352, "LogSample binary size changed");
