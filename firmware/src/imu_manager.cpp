#include "imu_manager.h"

#include <math.h>

#include <M5Unified.h>

#include "config.h"

bool ImuManager::begin() {
  imu_present_ = M5.Imu.begin();
  if (!imu_present_) {
    reading_.imu_ok = false;
    last_error_ = "imu_init_failed";
    return false;
  }
  constexpr float imu_hz = 1000.0f / Config::IMU_PERIOD_MS;
  filter_fixed_.begin(imu_hz);
  filter_fixed_.setBeta(Config::MADGWICK_BETA_NORMAL);
  filter_beta1_.begin(imu_hz);
  filter_beta1_.setBeta(Config::MADGWICK_BETA_ONE);
  filter_step_.begin(imu_hz);
  filter_step_.setBeta(Config::MADGWICK_BETA_NORMAL);
  filter_smooth_.begin(imu_hz);
  beta_smooth_ = Config::MADGWICK_BETA_NORMAL;
  filter_smooth_.setBeta(beta_smooth_);
  filter_started_ = true;
  reading_.imu_ok = true;
  reading_.beta_fixed = Config::MADGWICK_BETA_NORMAL;
  reading_.beta_step = Config::MADGWICK_BETA_NORMAL;
  reading_.beta_target = Config::MADGWICK_BETA_NORMAL;
  reading_.beta_smooth = beta_smooth_;
  reading_.beta_tau_s = Config::BETA_RECOVERY_TAU_S;
  reading_.beta_dynamic = Config::MADGWICK_BETA_NORMAL;
  reading_.beta_update_mode = 0;
  reading_.last_update_ms = millis();
  last_error_ = "";
  return true;
}

bool ImuManager::updateDue(uint32_t now_us) const {
  return last_due_us_ == 0 || static_cast<uint32_t>(now_us - last_due_us_) >=
      Config::IMU_PERIOD_MS * 1000UL;
}

void ImuManager::update() {
  const uint32_t now_us = micros();
  const uint32_t period_us = Config::IMU_PERIOD_MS * 1000UL;
  if (last_due_us_ != 0 && static_cast<uint32_t>(now_us - last_due_us_) < period_us) return;
  last_due_us_ = last_due_us_ == 0 ? now_us : last_due_us_ + period_us;

  const uint32_t task_start_us = micros();
  if (timing_audit_) {
    timing_audit_->imu_task_start_us = task_start_us;
    timing_audit_->imu_task_sequence = ++timing_task_sequence_;
  }
  if (!imu_present_ || !filter_started_) {
    reading_.imu_ok = false;
    reading_.error_count++;
    last_error_ = "imu_not_ready";
    return;
  }

  const uint32_t imu_hw_start_us = micros();
  const bool imu_updated = M5.Imu.update();
  if (timing_audit_) timing_audit_->imu_hw_update_us = micros() - imu_hw_start_us;
  if (!imu_updated) {
    reading_.error_count++;
    if (consecutive_errors_ < 255) consecutive_errors_++;
    if (consecutive_errors_ >= Config::IMU_ERROR_LIMIT) {
      reading_.imu_ok = false;
      last_error_ = "imu_update_false";
    }
    return;
  }

  consecutive_errors_ = 0;
  const uint32_t dt_us = prev_update_us_ == 0 ? period_us : static_cast<uint32_t>(now_us - prev_update_us_);
  prev_update_us_ = now_us;

  const uint32_t imu_data_start_us = micros();
  auto d = M5.Imu.getImuData();
  if (timing_audit_) timing_audit_->imu_get_data_us = micros() - imu_data_start_us;
  const float gx = d.gyro.x;
  const float gy = d.gyro.y;
  const float gz = d.gyro.z;
  const float ax = d.accel.x;
  const float ay = d.accel.y;
  const float az = d.accel.z;
  const float pitch_rate_dps = Config::GYRO_PITCH_RATE_SIGN * gy;
  const float beta_target = Config::MADGWICK_BETA_NORMAL;
  uint8_t beta_mode = 0;
  const float beta_tau_s = Config::BETA_RECOVERY_TAU_S;
  uint8_t beta_update_mode = 0;
  beta_smooth_ = Config::MADGWICK_BETA_NORMAL;

  const bool hold_comparison_filters = Config::AUTONOMOUS_SKIP_COMPARISON_MADGWICK &&
      autonomous_experiment_active_;
  if (!hold_comparison_filters) {
    const uint32_t filters_start_us = micros();
    filter_fixed_.setBeta(Config::MADGWICK_BETA_NORMAL);
    filter_beta1_.setBeta(Config::MADGWICK_BETA_ONE);
    filter_step_.setBeta(beta_target);
    filter_smooth_.setBeta(beta_smooth_);
    filter_fixed_.updateIMU(gx, gy, gz, ax, ay, az);
    filter_beta1_.updateIMU(gx, gy, gz, ax, ay, az);
    filter_step_.updateIMU(gx, gy, gz, ax, ay, az);
    filter_smooth_.updateIMU(gx, gy, gz, ax, ay, az);
    if (timing_audit_) timing_audit_->imu_manager_filters_us = micros() - filters_start_us;
  }

  raw_pitch_fixed_deg_ = filter_fixed_.getPitch();
  raw_pitch_beta1_deg_ = filter_beta1_.getPitch();
  raw_pitch_step_deg_ = filter_step_.getPitch();
  raw_pitch_smooth_deg_ = filter_smooth_.getPitch();
  reading_.pitch_fixed_beta_deg = Config::PITCH_SIGN * (raw_pitch_fixed_deg_ - pitch_fixed_offset_deg_);
  reading_.pitch_madgwick_beta1_deg = Config::PITCH_SIGN * (raw_pitch_beta1_deg_ - pitch_beta1_offset_deg_);
  reading_.pitch_step_beta_deg = Config::PITCH_SIGN * (raw_pitch_step_deg_ - pitch_step_offset_deg_);
  reading_.pitch_smooth_beta_deg = Config::PITCH_SIGN * (raw_pitch_smooth_deg_ - pitch_smooth_offset_deg_);
  reading_.pitch_dynamic_beta_deg = reading_.pitch_smooth_beta_deg;
  reading_.pitch_accel_only_deg = Config::PITCH_SIGN * atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2957795f;
  reading_.pitch_deg = reading_.pitch_smooth_beta_deg;
  reading_.pitch_rate_dps = pitch_rate_dps;
  reading_.ax_g = ax;
  reading_.ay_g = ay;
  reading_.az_g = az;
  reading_.gx_dps = gx;
  reading_.gy_dps = gy;
  reading_.gz_dps = gz;
  reading_.acc_norm_g = sqrtf(ax * ax + ay * ay + az * az);
  reading_.acc_norm_error_g = reading_.acc_norm_g - 1.0f;
  reading_.beta_fixed = Config::MADGWICK_BETA_NORMAL;
  reading_.beta_step = beta_target;
  reading_.beta_target = beta_target;
  reading_.beta_smooth = beta_smooth_;
  reading_.beta_tau_s = beta_tau_s;
  reading_.beta_dynamic = beta_target;
  reading_.beta_mode = beta_mode;
  reading_.beta_update_mode = beta_update_mode;
  reading_.time_since_last_pulse_ms = static_cast<uint16_t>(min<uint32_t>(65535, beta_context_time_since_last_pulse_ms_));
  reading_.update_dt_us = dt_us;
  reading_.last_update_us = now_us;
  reading_.last_update_ms = millis();
  reading_.imu_ok = true;
  last_error_ = "";
}

void ImuManager::zeroPitch() {
  pitch_fixed_offset_deg_ = raw_pitch_fixed_deg_;
  pitch_beta1_offset_deg_ = raw_pitch_beta1_deg_;
  pitch_step_offset_deg_ = raw_pitch_step_deg_;
  pitch_smooth_offset_deg_ = raw_pitch_smooth_deg_;
}

void ImuManager::setDynamicBetaContext(bool pulse_active, uint32_t time_since_last_pulse_ms, bool pre_start_stabilize) {
  beta_context_pulse_active_ = pulse_active;
  beta_context_time_since_last_pulse_ms_ = time_since_last_pulse_ms;
  beta_context_pre_start_stabilize_ = pre_start_stabilize;
}

void ImuManager::forceSmoothBeta(float beta, uint8_t update_mode) {
  beta_smooth_ = beta < 0.0f ? 0.0f : beta;
  filter_step_.setBeta(beta_smooth_);
  filter_smooth_.setBeta(beta_smooth_);
  reading_.beta_step = beta_smooth_;
  reading_.beta_target = beta_smooth_;
  reading_.beta_smooth = beta_smooth_;
  reading_.beta_tau_s = Config::BETA_RECOVERY_TAU_S;
  reading_.beta_dynamic = beta_smooth_;
  if (update_mode == 3) {
    reading_.beta_mode = 1;
  } else if (update_mode == 1) {
    reading_.beta_mode = 2;
  }
  reading_.beta_update_mode = update_mode;
}

bool ImuManager::stale(uint32_t now_ms) const {
  if (!reading_.imu_ok) return true;
  return static_cast<uint32_t>(now_ms - reading_.last_update_ms) > Config::IMU_STALE_LIMIT_MS;
}
