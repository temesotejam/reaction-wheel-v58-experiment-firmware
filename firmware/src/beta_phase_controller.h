#pragma once

#include <Arduino.h>
#include <math.h>

#include "config.h"

// Per-pulse phase gate for the adopted dynamic-beta series.
// The angle is a bias-corrected gyro integral relative to pulse start.  It is
// intentionally local: an uncalibrated global energy/geometry model is not
// needed for this first test implementation.
enum class BetaPhaseState : uint8_t {
  IDLE = 0,
  OUTBOUND_WAIT_TURN = 1,
  RETURN_TO_ZERO = 2,
};

class BetaPhaseController {
 public:
  void reset() {
    state_ = BetaPhaseState::IDLE;
    outbound_rate_sign_ = 0;
    phase_angle_deg_ = 0.0f;
    peak_abs_angle_deg_ = 0.0f;
    return_start_abs_angle_deg_ = 0.0f;
    return_progress_ = 0.0f;
  }

  void beginPulse() {
    state_ = BetaPhaseState::OUTBOUND_WAIT_TURN;
    outbound_rate_sign_ = 0;
    phase_angle_deg_ = 0.0f;
    peak_abs_angle_deg_ = 0.0f;
    return_start_abs_angle_deg_ = 0.0f;
    return_progress_ = 0.0f;
  }

  void update(float bias_corrected_rate_dps, float dt_s, bool input_protected) {
    if (state_ == BetaPhaseState::IDLE) return;
    if (dt_s > 0.0f && dt_s < 0.1f) phase_angle_deg_ += bias_corrected_rate_dps * dt_s;

    const float abs_rate_dps = fabsf(bias_corrected_rate_dps);
    if (state_ == BetaPhaseState::OUTBOUND_WAIT_TURN) {
      // Do not decide the outward direction or a turn while the known input
      // acceleration is still protected by the current/hold gate.
      if (input_protected) return;
      if (outbound_rate_sign_ == 0) {
        if (abs_rate_dps >= Config::BETA_PHASE_OUTBOUND_RATE_MIN_DPS) {
          outbound_rate_sign_ = bias_corrected_rate_dps >= 0.0f ? 1 : -1;
        }
        return;
      }
      if (bias_corrected_rate_dps * static_cast<float>(outbound_rate_sign_) <=
          -Config::BETA_PHASE_TURN_CONFIRM_RATE_DPS) {
        state_ = BetaPhaseState::RETURN_TO_ZERO;
        peak_abs_angle_deg_ = fmaxf(fabsf(phase_angle_deg_), Config::BETA_PHASE_MIN_PEAK_ANGLE_DEG);
        return_start_abs_angle_deg_ = peak_abs_angle_deg_;
        return_progress_ = 0.0f;
      }
      return;
    }

    const float denominator = fmaxf(return_start_abs_angle_deg_, Config::BETA_PHASE_MIN_PEAK_ANGLE_DEG);
    return_progress_ = constrain((denominator - fabsf(phase_angle_deg_)) / denominator, 0.0f, 1.0f);
  }

  float betaCeiling(float beta_floor, float beta_max) const {
    if (state_ != BetaPhaseState::RETURN_TO_ZERO) return beta_floor;
    const float k = Config::BETA_PHASE_EXPONENTIAL_K;
    const float numerator = expf(k * return_progress_) - 1.0f;
    const float denominator = expf(k) - 1.0f;
    const float normalized = denominator > 0.0f ? numerator / denominator : return_progress_;
    return beta_floor + (beta_max - beta_floor) * constrain(normalized, 0.0f, 1.0f);
  }

  BetaPhaseState state() const { return state_; }
  float phaseAngleDeg() const { return phase_angle_deg_; }
  float peakAbsAngleDeg() const { return peak_abs_angle_deg_; }
  float returnProgress() const { return return_progress_; }

 private:
  BetaPhaseState state_ = BetaPhaseState::IDLE;
  int8_t outbound_rate_sign_ = 0;
  float phase_angle_deg_ = 0.0f;
  float peak_abs_angle_deg_ = 0.0f;
  float return_start_abs_angle_deg_ = 0.0f;
  float return_progress_ = 0.0f;
};
