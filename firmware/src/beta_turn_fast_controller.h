#pragma once

#include <Arduino.h>
#include <math.h>

#include "config.h"

// Observational dynamic-beta strategy: keep beta at its current/Vbat-derived
// floor until the post-pulse motion has demonstrably turned, then restore beta
// to the normal dynamic ceiling on a short, deterministic time ramp.
enum class BetaTurnFastState : uint8_t {
  IDLE = 0,
  WAITING_TURN = 1,
  TURN_CONFIRMED_RECOVERY = 2,
  FALLBACK_RECOVERY = 3,
};

class BetaTurnFastController {
 public:
  void reset() {
    state_ = BetaTurnFastState::IDLE;
    outbound_rate_sign_ = 0;
    reverse_samples_ = 0;
    phase_angle_deg_ = 0.0f;
    peak_abs_angle_deg_ = 0.0f;
    post_input_elapsed_s_ = 0.0f;
    recovery_elapsed_s_ = 0.0f;
  }

  void beginPulse() {
    state_ = BetaTurnFastState::WAITING_TURN;
    outbound_rate_sign_ = 0;
    reverse_samples_ = 0;
    phase_angle_deg_ = 0.0f;
    peak_abs_angle_deg_ = 0.0f;
    post_input_elapsed_s_ = 0.0f;
    recovery_elapsed_s_ = 0.0f;
  }

  void update(float bias_corrected_rate_dps, float dt_s, bool input_protected) {
    if (state_ == BetaTurnFastState::IDLE || dt_s <= 0.0f || dt_s >= 0.1f) return;

    phase_angle_deg_ += bias_corrected_rate_dps * dt_s;
    peak_abs_angle_deg_ = fmaxf(peak_abs_angle_deg_, fabsf(phase_angle_deg_));

    if (state_ == BetaTurnFastState::TURN_CONFIRMED_RECOVERY ||
        state_ == BetaTurnFastState::FALLBACK_RECOVERY) {
      recovery_elapsed_s_ += dt_s;
      return;
    }

    // The normal 73 ms input guard prevents the commanded acceleration from
    // being mistaken for free-motion reversal. The integral remains continuous
    // across this guard so the minimum-angle check is still relative to pulse start.
    if (input_protected) return;
    post_input_elapsed_s_ += dt_s;

    const float abs_rate_dps = fabsf(bias_corrected_rate_dps);
    if (outbound_rate_sign_ == 0) {
      if (abs_rate_dps >= Config::BETA_TURN_FAST_OUTBOUND_RATE_MIN_DPS) {
        outbound_rate_sign_ = bias_corrected_rate_dps >= 0.0f ? 1 : -1;
      }
    } else if (peak_abs_angle_deg_ >= Config::BETA_TURN_FAST_MIN_PEAK_ANGLE_DEG &&
               bias_corrected_rate_dps * static_cast<float>(outbound_rate_sign_) <=
                   -Config::BETA_TURN_FAST_TURN_CONFIRM_RATE_DPS) {
      if (reverse_samples_ < 255) ++reverse_samples_;
      if (reverse_samples_ >= Config::BETA_TURN_FAST_CONFIRM_SAMPLES) {
        state_ = BetaTurnFastState::TURN_CONFIRMED_RECOVERY;
        recovery_elapsed_s_ = 0.0f;
        return;
      }
    } else {
      reverse_samples_ = 0;
    }

    if (post_input_elapsed_s_ >= static_cast<float>(Config::BETA_TURN_FAST_FALLBACK_MS) / 1000.0f) {
      state_ = BetaTurnFastState::FALLBACK_RECOVERY;
      recovery_elapsed_s_ = 0.0f;
    }
  }

  float betaTarget(float beta_floor, float beta_ceiling) const {
    if (state_ == BetaTurnFastState::IDLE) return beta_ceiling;
    if (state_ == BetaTurnFastState::WAITING_TURN) return beta_floor;
    const float duration_s = static_cast<float>(Config::BETA_TURN_FAST_RECOVERY_MS) / 1000.0f;
    const float u = duration_s > 0.0f ? constrain(recovery_elapsed_s_ / duration_s, 0.0f, 1.0f) : 1.0f;
    // Smoothstep reaches both endpoints without a beta step or a slope jump.
    const float smooth_u = u * u * (3.0f - 2.0f * u);
    return beta_floor + (beta_ceiling - beta_floor) * smooth_u;
  }

  BetaTurnFastState state() const { return state_; }
  float phaseAngleDeg() const { return phase_angle_deg_; }
  float peakAbsAngleDeg() const { return peak_abs_angle_deg_; }
  float recoveryProgress() const {
    if (state_ != BetaTurnFastState::TURN_CONFIRMED_RECOVERY && state_ != BetaTurnFastState::FALLBACK_RECOVERY) return 0.0f;
    const float duration_s = static_cast<float>(Config::BETA_TURN_FAST_RECOVERY_MS) / 1000.0f;
    return duration_s > 0.0f ? constrain(recovery_elapsed_s_ / duration_s, 0.0f, 1.0f) : 1.0f;
  }

 private:
  BetaTurnFastState state_ = BetaTurnFastState::IDLE;
  int8_t outbound_rate_sign_ = 0;
  uint8_t reverse_samples_ = 0;
  float phase_angle_deg_ = 0.0f;
  float peak_abs_angle_deg_ = 0.0f;
  float post_input_elapsed_s_ = 0.0f;
  float recovery_elapsed_s_ = 0.0f;
};