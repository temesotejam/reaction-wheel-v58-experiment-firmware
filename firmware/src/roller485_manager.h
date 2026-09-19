#pragma once

#include <Arduino.h>

#include "timing_audit.h"
#include "current_timing_audit.h"
#include "q_observer.h"

struct RollerTelemetry {
  bool roller_ok = false;
  int16_t actual_current_mA = 0;
  float current_raw_mA = NAN;
  uint32_t bus_voltage_sample_time_us = 0;
  uint16_t battery_mV = 0;
  uint32_t i2c_error_count = 0;
  uint8_t consecutive_errors = 0;
  uint8_t mode_raw = 0;
  uint8_t output_raw = 0;
  uint8_t status_raw = 0;
  uint8_t error_raw = 0;
  uint32_t status_sample_time_us = 0;

  // Freshness is about CURRENT_READBACK only. `current_valid` is true exactly
  // when the most recent current-read attempt succeeded; on failure the numeric
  // current remains the previous sample and must not be used as a new sample.
  uint32_t current_sample_time_us = 0;
  uint32_t current_sequence = 0;
  uint32_t current_read_failure_count = 0;
  uint16_t current_audit_sample_count = 0;
  float q_meas_observed_mA_s = NAN;
  bool current_valid = false;
  bool q_meas_observed_valid = false;
  // V48: values are measured from every successful CURRENT_READBACK in the
  // firmware, not reconstructed from decimated RWLOG rows.  They describe the
  // active pulse or the most recently completed pulse after output stops.
  uint32_t current_audit_max_gap_us = 0;
  uint16_t current_audit_gt_3333_count = 0;
  uint16_t current_audit_interval_count = 0;
  uint32_t current_audit_tail_max_gap_us = 0;
  uint16_t current_audit_tail_gt_3333_count = 0;
  uint16_t current_audit_tail_interval_count = 0;
  uint32_t current_audit_pulse_end_current_age_us = UINT32_MAX;
  uint16_t current_audit_imu_deadline_guard_count = 0;
  uint16_t current_audit_post_imu_service_count = 0;
  bool current_audit_finalized = false;
  bool current_audit_tail_covered = false;
};

class Roller485Manager {
public:
  bool begin();
  bool beginFixedProbe(bool full,uint16_t run_id);
  void updateFixedProbe();
  void abortFixedProbe(FixedProbeV57::Result reason);
  bool fixedProbeActive() const {return v57_acquisition_;}

  // Normal scheduler entry point. It retains V46 behavior when a caller does
  // not need to split the fresh-current and full-status work.
  void update();
  void updateWheelIdle();
  bool readWheelForV55();
  WheelProbeV55::Result v55StartResult() const { return v55_start_result_; }
  void updatePulseCurrent();
  // V48 scheduler hooks.  They are observation scheduling only and never
  // select, shorten, extend, or otherwise alter a motor command.
  void servicePulseCurrentBeforeImu();
  void servicePulseCurrentAfterImu();
  void servicePulseCurrentBeforeStatus();
  void servicePulseCurrentAfterStatus();
  bool fullStatusDue(uint32_t now_us) const;
  CurrentTimingAudit& currentTimingAudit() { return current_timing_audit_; }
  QObserver& qObserver() { return q_observer_; }
  bool currentObservationPriority() const { return v57_acquisition_ || currentCommandActive() || q_observer_.observingPost(); }
  void updateFullStatus();
  void setTimingAudit(TimingAudit* timing_audit) { timing_audit_ = timing_audit; }

  bool setCurrentMa(int16_t current_mA);
  bool stop(MeasuredQStop::Reason reason = MeasuredQStop::SAFETY_STOP);
  void serviceMeasuredQStop();
  bool deadlineCritical() { return measured_q_stop_.deadlineCritical(micros()); }
  bool measuredQStopEnabled() const { return measured_q_stop_.record.enabled; }
  MeasuredQStop::Reason measuredQStopReason() const { return measured_q_stop_.record.reason; }

  const RollerTelemetry& telemetry() const { return telemetry_; }
  bool currentCommandActive() const { return command_mA_ != 0; }
  uint32_t currentAgeUs(uint32_t now_us) const;
  bool ok() const { return !output_write_fault_ && telemetry_.roller_ok && telemetry_.consecutive_errors < 5 && telemetry_.error_raw == 0; }
  const char* lastError() const { return last_error_; }

private:
  bool v57_acquisition_=false;
  bool v57FreshCurrent();
  bool endFixedProbeOutput();
  bool failFixedProbeTrial(FixedProbeV57::Result reason);
  bool startFixedProbePreparation();
  bool switchFixedProbeToZero();
  void serviceFixedProbeSafety();
void updateFixedProbeStatus();
  void sampleV58Speed();
  bool beginFixedProbePulse();
  bool zeroFixedProbePulse(bool advance);
  bool readFixedProbeWheel(FixedProbeV57::Trial& trial, bool after_probe);
  bool writeU8(uint8_t reg, uint8_t value);
  bool writeI32(uint8_t reg, int32_t value);
  bool readBytes(uint8_t reg, uint8_t* buffer, size_t len);
  bool readI32(uint8_t reg, int32_t& value);
  bool readU8(uint8_t reg, uint8_t& value);
  bool readCurrentFresh(bool audit_sample);
  bool updatePulseCurrentInternal(bool force_now);
  void recordFreshCurrent(int32_t current_raw, uint32_t sample_time_us, bool audit_sample);
  void recordCurrentReadFailure(bool audit_sample);
  void beginCurrentAuditPulse();
  void endCurrentAuditPulse(uint32_t pulse_end_us);
  void recordCurrentAuditGap(uint32_t previous_sample_us, uint32_t sample_time_us,
                             uint32_t gap_us);
  void finalizeCurrentAuditTail(uint32_t pulse_end_us);
  void recordIo(bool ok);
  void captureWheelAfterV55();
  WheelProbeV55::Result v55_start_result_ = WheelProbeV55::PENDING;

  RollerTelemetry telemetry_;
  CurrentTimingAudit current_timing_audit_;
  QObserver q_observer_;
  MeasuredQStop measured_q_stop_;
  bool output_write_fault_ = false;
  bool v55_bus_timeout_active_ = false;
  void applyMeasuredQStop();
  uint32_t current_read_duration_us_ = 0;
  TimingAudit* timing_audit_ = nullptr;
  uint32_t timing_full_status_sequence_ = 0;
  uint32_t last_read_due_us_ = 0;
  uint32_t last_wheel_attempt_us_ = 0;
  bool has_wheel_attempt_ = false;
  uint32_t last_fast_current_due_us_ = 0;
  int16_t command_mA_ = 0;
  int16_t current_audit_previous_mA_ = 0;
  uint32_t current_audit_previous_sample_us_ = 0;
  bool current_audit_has_previous_sample_ = false;
  bool current_audit_active_ = false;
  bool current_audit_read_failed_ = false;
  struct CurrentAuditGap {
    uint32_t previous_sample_us;
    uint32_t sample_time_us;
    uint32_t gap_us;
  };
  static constexpr uint8_t CURRENT_AUDIT_GAP_HISTORY_CAPACITY = 16;
  CurrentAuditGap current_audit_gap_history_[CURRENT_AUDIT_GAP_HISTORY_CAPACITY]{};
  uint8_t current_audit_gap_history_head_ = 0;
  uint8_t current_audit_gap_history_count_ = 0;
  const char* last_error_ = "not_initialized";
};
