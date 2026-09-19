#include "q_observer_json.h"
#include <stdio.h>

namespace {
void number(String& j, const char* key, double value) {
  char b[160];
  if (isfinite(value)) snprintf(b, sizeof(b), "\"%s\":%.17g,", key, value);
  else snprintf(b, sizeof(b), "\"%s\":null,", key);
  j += b;
}
void value(String& j, double value) {
  char b[48];
  if (isfinite(value)) snprintf(b, sizeof(b), "%.17g,", value);
  else snprintf(b, sizeof(b), "null,");
  j += b;
}
void flag(String& j, const char* key, bool value) { j += "\""; j += key; j += "\":"; j += value ? "true," : "false,"; }
}

void appendFixedProbeV57Json(String& j, const QObserver& o) {
  const auto& r = o.v57;
  j += ",\"q_observer_v58\":{\"schema\":\"v58_probe_coast_current_v1\",\"control_connected\":false,";
  number(j, "raw_per_mA", o.raw_per_mA); number(j, "sample_count", o.sample_count);
  number(j, "sample_overflow", o.sample_overflow); number(j, "observation_count", o.pulse_count);
  number(j, "pulse_overflow", o.pulse_overflow); flag(j, "allocated", o.allocated());
  j += "\"sample_storage\":\"RWLOG_probe_pre_to_post_event_windows\",";
  j += "\"raw_current_storage_policy\":\"V58 stores fresh pre-event and active/post current; baseline/preparation/transfer retain trial aggregates\",";
  j += "\"sample_columns\":[\"end_us\",\"sequence\",\"current_raw\",\"read_duration_us\",\"valid\"],";
  j += "\"pulse_metadata_storage\":\"paired_probe_v58.trials\",";
  j += "\"raw_integral_semantics\":\"directed_trapezoid_between_successful_current_read_completion_times;edge_intervals_not_estimated\"}";

  j += ",\"paired_probe_v58\":{\"schema\":\"paired_probe_coast_v1\",";
#define R(f) number(j, #f, r.f)
  R(enabled); R(started); R(finished); R(aborted); R(full); R(start_us); R(end_us); R(index); R(trial_count); R(failed_trial_count); number(j,"valid_event_count",r.valid_probe_count); R(valid_goal_each); R(attempt_limit_each); R(order_seed);
#undef R
  number(j,"minimum_settle_us",FixedProbeV57::MIN_SETTLE_US);
  number(j,"baseline_window_us",FixedProbeBaseline::WINDOW_US);
  number(j,"baseline_max_MAD_mA",FixedProbeBaseline::MAX_MAD_MA);
  number(j,"baseline_max_spread_mA",FixedProbeBaseline::MAX_SPREAD_MA);
  number(j,"baseline_max_slope_mA_s",FixedProbeBaseline::MAX_SLOPE_MA_S);
  number(j,"maximum_preparation_rpm",FixedProbeV57::MAX_PREPARATION_RPM);
  number(j,"primary_Q_window_us",50000);
  number(j,"run_budget_ms",FixedProbeV57::RUN_LIMIT_MS);
  number(j,"timeseries_period_us",100000);
  j += "\"primary_Q_definition\":\"raw_directed_integral_write_end_to_plus_50000us_adjacent_interpolation_only\",";
  j += "\"baseline_statistic\":\"median_and_true_MAD_of_last_half_of_200ms_window\",";
  j += "\"valid_count_semantics\":\"hardware_gates_and_active_timing;offline_Q50_quality_rechecked\",";
  j += "\"event_semantics\":\"PROBE_CURRENT_300mA_or_COAST_CURRENT_0mA_symmetric_write_completion;coast_nonzero_write_fields_are_zero\",";
  j += "\"legacy_aliases\":\"before_probe_and_age_at_nonzero_end_fields_apply_to_both_event_roles;actual_probe_width_is_event_observation_width\",";
  number(j,"speed_50_late_limit_us",FixedProbeV57::SPEED_50_LATE_LIMIT_US);
  j += "\"speed_trace_columns\":[\"time_us\",\"rpm\"],";
  j += "\"valid_by_condition\":[";
  for(uint8_t c=0;c<FixedProbeV57::CONDITION_COUNT;++c) { if(c)j+=",";j+=String(r.valid_by_condition[c]); }
  j += "],";
  number(j, "probe_current_mA", FixedProbeV57::PROBE_CURRENT_MA);
  number(j, "planned_probe_us", FixedProbeV57::PROBE_US);
  number(j, "baseline_min_current_samples", FixedProbeBaseline::N);
  number(j, "zero_write_reservation_us", FixedProbeV57::ZERO_RESERVATION_US);
  number(j, "transfer_limit_us", FixedProbeV57::TRANSFER_LIMIT_US);
  number(j, "transfer_continuous_us", FixedProbeV57::TRANSFER_CONTINUOUS_US);
  number(j, "transfer_tolerance_mA", FixedProbeV57::TRANSFER_TOLERANCE_MA);
  number(j, "speed_age_target_us", FixedProbeV57::MAX_SPEED_AGE_US);
  number(j, "current_gap_limit_us", QObserver::GAP_US);
  number(j, "non_pulse_current_read_attempts", FixedProbeV57::NON_PULSE_CURRENT_READ_ATTEMPTS);
  j += "\"transfer_rule\":\"abs(current-baseline_median)<=max(3MAD,0.2mA)_continuous_30ms\",";
  j += "\"transfer_timeout_rule\":\"TRANSFER_NOT_REACHED_at_1500ms_without_threshold_relaxation\",";
  j += "\"pulse_stop_rule\":\"fixed_60ms_only;zero_write_started_1p5ms_before_deadline;no_measured_Q_early_stop\",";
  j += "\"current_read_transition_rule\":\"trial_or_phase_change_yields_without_read_failure\",";
  j += "\"current_read_failure_rule\":\"one_immediate_retry_outside_pulse_post_window;any_pulse_or_post_read_failure_stops_run\",";
  j += "\"q_probe_definition\":\"direction_normalized_integral_of_raw_current_from_event_start_to_zero_write_begin;offline_uses_explicit_boundary_interpolation\",";
  j += "\"timestamp_semantics\":\"I2C_read_or_write_completion_micros_modulo_2pow32_not_internal_sensor_time\",";
  j += "\"direction_alignment\":\"aligned_speed=d*rw_speed_rpm;d=probe_direction_or_coast_virtual_direction\",";
  j += "\"reason\":\""; j += FixedProbeV57::name(r.reason); j += "\",\"trial_columns\":[";
#define C(f) j += "\"" #f "\","
  C(id); C(condition_repeat); C(target_speed_rpm); C(probe_direction); C(start_us); C(end_us);
  C(baseline_begin_us); C(baseline_end_us); C(baseline_current_mA); C(baseline_current_mad_mA); C(baseline_current_samples);
  C(speed_mode_enter_us); C(speed_stable_begin_us); C(speed_target_reached_us); C(speed_mode_exit_begin_us); C(speed_mode_exit_end_us);
  C(transfer_begin_us); C(transfer_ready_begin_us); C(transfer_ready_us); C(target_reached); C(transition_ok); C(transfer_ready);
  C(rw_speed_before_probe_rpm); C(rw_speed_timestamp_us); C(rw_speed_age_at_nonzero_end_us); C(speed_fresh_at_probe);
  C(current_before_probe_mA); C(current_timestamp_us); C(bus_voltage_before_probe_mV); C(bus_voltage_timestamp_us);
  C(nonzero_write_begin_us); C(nonzero_write_end_us); C(planned_probe_us); C(probe_deadline_us);
  C(zero_write_begin_us); C(zero_write_end_us); C(actual_probe_width_us); C(zero_deadline_margin_us); C(zero_deadline_met);
  C(q_probe_60ms_on_device_mA_s); C(active_current_samples); C(active_current_failures); C(active_current_max_gap_us);
  C(rw_speed_after_probe_rpm); C(rw_speed_after_timestamp_us); C(aligned_speed_delta_rpm);
  C(condition_id); C(requested_aligned_speed_rpm); C(baseline_current_slope_mA_s); C(baseline_current_spread_mA); C(baseline_speed_median_rpm); C(baseline_valid); C(transfer_threshold_mA); C(transfer_last_current_mA); C(transfer_residual_min_mA); C(transfer_residual_max_mA); C(current_before_probe_aligned_mA); C(current_age_at_nonzero_end_us); C(fresh_current_valid); j += "\"valid_event\","; 
  C(is_coast); C(pair_id); C(event_start_us); C(coast_event_start_us); C(event_write_begin_us); C(event_write_end_us); C(speed_50ms_time_us); C(speed_observation_dt_us); C(speed_50ms_rpm); C(aligned_speed_delta_50_rpm); C(speed_50ms_valid); C(trace_count);
#undef C
  j += "\"result\",\"speed_trace\"],\"trials\":[";
  for (uint8_t i = 0; i < r.trial_count; ++i) {
    if (i) j += ","; const auto& t = r.trials[i]; j += "[";
#define T(f) value(j, t.f)
    T(id); T(condition_repeat); T(target_speed_rpm); T(probe_direction); T(start_us); T(end_us);
    T(baseline_begin_us); T(baseline_end_us); T(baseline_current_mA); T(baseline_current_mad_mA); T(baseline_current_samples);
    T(speed_mode_enter_us); T(speed_stable_begin_us); T(speed_target_reached_us); T(speed_mode_exit_begin_us); T(speed_mode_exit_end_us);
    T(transfer_begin_us); T(transfer_ready_begin_us); T(transfer_ready_us); T(target_reached); T(transition_ok); T(transfer_ready);
    T(rw_speed_before_probe_rpm); T(rw_speed_timestamp_us); T(rw_speed_age_at_nonzero_end_us); T(speed_fresh_at_probe);
    T(current_before_probe_mA); T(current_timestamp_us); T(bus_voltage_before_probe_mV); T(bus_voltage_timestamp_us);
    T(nonzero_write_begin_us); T(nonzero_write_end_us); T(planned_probe_us); T(probe_deadline_us);
    T(zero_write_begin_us); T(zero_write_end_us); T(actual_probe_width_us); T(zero_deadline_margin_us); T(zero_deadline_met);
    T(q_probe_60ms_on_device_mA_s); T(active_current_samples); T(active_current_failures); T(active_current_max_gap_us);
    T(rw_speed_after_probe_rpm); T(rw_speed_after_timestamp_us); T(aligned_speed_delta_rpm);
    T(condition_id); T(requested_aligned_speed_rpm); T(baseline_current_slope_mA_s); T(baseline_current_spread_mA); T(baseline_speed_median_rpm); T(baseline_valid); T(transfer_threshold_mA); T(transfer_last_current_mA); T(transfer_residual_min_mA); T(transfer_residual_max_mA); T(current_before_probe_aligned_mA); T(current_age_at_nonzero_end_us); T(fresh_current_valid); T(valid_probe); 
    T(is_coast); T(pair_id); T(event_start_us); T(coast_event_start_us); T(event_write_begin_us); T(event_write_end_us); T(speed_50ms_time_us); T(speed_observation_dt_us); T(speed_50ms_rpm); T(aligned_speed_delta_50_rpm); T(speed_50ms_valid); T(trace_count);
#undef T
    j += "\""; j += FixedProbeV57::name(t.result); j += "\",[";
    for(uint8_t k=0;k<t.trace_count && k<6;++k){if(k)j+=",";j+="[";value(j,t.speed_trace[k].time_us);char b[48];if(isfinite(t.speed_trace[k].rpm))snprintf(b,sizeof(b),"%.17g",double(t.speed_trace[k].rpm));else snprintf(b,sizeof(b),"null");j+=b;j+="]";}
    j+="]]";
  }
  j += "],"; number(j, "voltage_overflow", r.voltage_overflow);
  j += "\"voltage_sample_storage\":\"fresh_pre_probe_value_in_each_trial\"}";

  const auto& w = o.wheel;
  j += ",\"wheel_observer_v58\":{\"schema\":\"speed_preparation_and_probe_boundaries_v1\",\"raw_per_rpm\":100,";
  number(j, "count", w.count); number(j, "overflow", w.overflow); number(j, "failures", w.failures);
  j += "\"sample_storage\":\"per_trial_speed_trace_and_after_zero_boundary\",";
  j += "\"timestamp_semantics\":\"I2C_read_completion_micros_modulo_2pow32\"}";
}