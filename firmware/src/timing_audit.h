#pragma once

#include <Arduino.h>

// RWLOG v46 timing-audit snapshot.  Every field is diagnostic only.  A zero
// duration or sequence means that the corresponding task did not execute in
// the completed main-loop represented by this snapshot.
struct TimingAudit {
  uint32_t loop_sequence = 0;
  uint32_t loop_total_us = 0;
  uint32_t m5_update_us = 0;
  uint32_t service_fast_us = 0;
  uint32_t beta_context_us = 0;
  uint32_t imu_update_total_us = 0;
  uint32_t roller_update_total_us = 0;
  uint32_t runner_update_total_us = 0;
  uint32_t web_update_us = 0;

  uint32_t imu_hw_update_us = 0;
  uint32_t imu_get_data_us = 0;
  uint32_t imu_manager_filters_us = 0;

  uint32_t runner_beta1_filters_us = 0;
  uint32_t runner_dynamic_all_us = 0;
  uint32_t adopted_filter_only_us = 0;
  uint32_t beta_turn_fast_us = 0;

  uint32_t update_filter_series_us = 0;
  uint32_t update_displayed_angles_us = 0;
  uint32_t update_current_roll_us = 0;
  uint32_t q1_shadow_us = 0;
  uint32_t autonomous_motion_us = 0;

  uint32_t fast_current_read_us = 0;
  uint32_t roller_full_status_us = 0;
  uint32_t log_sample_us = 0;

  uint32_t imu_task_start_us = 0;
  uint32_t imu_task_sequence = 0;
  uint32_t roller_full_status_start_us = 0;
  uint32_t roller_full_status_sequence = 0;
  uint32_t runner_filter_start_us = 0;
  uint32_t runner_filter_sequence = 0;
  uint32_t log_task_start_us = 0;
  uint32_t log_task_sequence = 0;
  uint32_t web_task_start_us = 0;
  uint32_t web_task_sequence = 0;
};
