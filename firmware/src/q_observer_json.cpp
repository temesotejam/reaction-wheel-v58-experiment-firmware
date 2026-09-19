#include "q_observer_json.h"
#include <stdio.h>
namespace {
void number(String& j, const char* key, double n) {
  char b[112];
  if (isfinite(n)) snprintf(b, sizeof(b), "\"%s\":%.17g,", key, n);
  else snprintf(b, sizeof(b), "\"%s\":null,", key);
  j += b;
}
void flag(String& j, const char* key, bool v) { number(j, key, v ? 1 : 0); }
void sample(String& j, const char* key, const QObserver::Sample& s, float scale) {
  char b[240];
  snprintf(b, sizeof(b), "\"%s\":{\"time_us\":%lu,\"sequence\":%lu,\"raw\":%ld,\"mA\":%.9g,\"valid\":%s},",
      key, static_cast<unsigned long>(s.time_us), static_cast<unsigned long>(s.sequence),
      static_cast<long>(s.raw), s.raw / static_cast<double>(scale), s.valid ? "true" : "false");
  j += b;
}
void reach(String& j, const char* key, const QObserver::Reach& r, bool valid) {
  j += "\""; j += key; j += "\":{";
  flag(j, "valid", valid); flag(j, "target_reached", r.reached);
  number(j, "reach_time_us", r.reached ? static_cast<double>(r.time_us) : NAN);
  j += "\"time_semantics\":\"first_causal_read_completion\"},";
}
}
void appendWheelProbeV55Json(String& j, const QObserver& o);
void appendFixedProbeV57Json(String&,const QObserver&);
void appendQObserverJson(String& j, const QObserver& o) {
  if(o.v57.enabled){appendFixedProbeV57Json(j,o);return;}
  if (o.v55.enabled) appendWheelProbeV55Json(j, o);
  // Retain the v50 key/raw layout for converter compatibility; schema is explicit.
  j += ",\"q_observer_v50\":{\"schema\":\"";
  j += o.v55.enabled ? "v55_fixed_probe_v1" : "v52_hard_deadline_v1";
  j += "\",\"control_connected\":true,";
  j += "\"stop_policy\":\"";
  j += o.v55.enabled ? "fixed_duration_probe_with_observation_and_hard_guards" : "signed_sample_only_normal_autonomous_pulses";
  j += "\",\"stop_deadline_origin\":\"nonzero_CURRENT_write_end\",";
  j += "\"wait_limit_rule\":\"";
  j += o.v55.enabled ? "fixed_duration_ms_with_98.5ms_hard_reservation" : "min(planned_ms+10,100)";
  j += "\",\"stop_boundary_is_physical\":false,";
  number(j, "hard_active_limit_us", MeasuredQStop::HARD_US);
  number(j, "zero_write_reservation_us", MeasuredQStop::RESERVE_US);
  number(j, "deadline_critical_window_us", MeasuredQStop::CRITICAL_US);
  j += "\"early_wait_stop_reason\":\"Q_TARGET_NOT_REACHED_only_before_forced_stop_deadline\",";
  j += "\"legacy_event_pulse_end_ms_semantics\":\"planned_only_use_observer_actual_timestamps\",";
  j += "\"command_boundary\":\"CURRENT_register_write_end_not_physical_edge\",";
  j += "\"raw_event_format\":\"<IIiIB3x\",\"sample_time_semantics\":\"read_completion_not_internal_sensor_time\",";
  j += "\"end_reason_legend\":\"0=open,1=100ms_window,2=next_nonzero_write_begin,3=run_end,4=write_failed\",";
  number(j, "raw_per_mA", o.raw_per_mA); number(j, "sample_count", o.sample_count);
  number(j, "pulse_count", o.pulse_count); number(j, "sample_overflow", o.sample_overflow);
  number(j, "pulse_overflow", o.pulse_overflow); flag(j, "allocated", o.allocated());
  flag(j, "recording_at_export", o.recording);
  j += "\"pulses\":[";
  for (uint32_t i = 0; i < o.pulse_count; ++i) {
    if (i) j += ",";
    j += "{";
    const auto& p = o.pulses[i];
#define N(field) number(j, #field, p.field)
    N(id); N(direction); N(command_start_us); N(nonzero_write_begin_us); N(nonzero_write_end_us);
    N(nonzero_output_end_us); N(stop_command_us); N(zero_write_begin_us); N(zero_write_end_us);
    N(zero_output_end_us); N(post_end_us); N(phase); N(end_reason);
    N(start_ok); N(stop_ok); N(has_first); N(active_bad); N(post_bad);
    N(active_covered_us); N(post_covered_us); N(active_samples); N(active_failures); N(post_failures);
    N(active_max_gap_us); N(post_max_gap_us);
    N(previous_direction); N(consecutive_same_direction);
#undef N
    sample(j, "current_before_pulse", p.before, o.raw_per_mA);
    const auto& w = p.wheel_before;
    j += "\"wheel_state_v54\":{";
    number(j, "rw_speed_raw", w.sequence ? static_cast<double>(w.raw) : NAN);
    number(j, "rw_speed_rpm", w.valid ? w.rpm() : NAN);
    number(j, "rw_speed_rad_s", w.valid ? w.radS() : NAN);
    number(j, "rw_speed_aligned_rpm", w.valid ? p.direction * w.rpm() : NAN);
    number(j, "read_begin_us", w.sequence ? static_cast<double>(w.begin_us) : NAN);
    number(j, "read_end_us", w.sequence ? static_cast<double>(w.time_us) : NAN);
    number(j, "sequence", w.sequence); flag(j, "read_valid", w.valid);
    number(j, "age_at_command_us", w.sequence ? static_cast<double>(p.command_start_us - w.time_us) : NAN);
    number(j, "age_at_nonzero_write_end_us", w.sequence ? static_cast<double>(p.nonzero_write_end_us - w.time_us) : NAN);
    number(j, "rw_speed_before_pulse", w.valid ? w.rpm() : NAN);
    number(j, "rw_speed_at_nonzero_write_end", NAN);
    number(j, "rw_speed_20ms", NAN); number(j, "rw_speed_50ms", NAN); number(j, "rw_speed_near_end", NAN);
    j += "\"missing_reason\":\"NO_ACTIVE_BUS_READ_TO_PRESERVE_CURRENT_TIMING\",\"before_semantics\":\"";
    j += p.context.fixed_duration_v55 ? "fresh_pre_CURRENT_read_with_explicit_age_not_internal_encoder_time" : "latest_idle_read_with_explicit_age_not_exact_start";
    j += "\"},";
    if (p.context.fixed_duration_v55) {
    j += "\"v55_identification\":{";
    flag(j, "fixed_duration", p.context.fixed_duration_v55);
    number(j, "role", p.context.v55_role); number(j, "trial_id", p.context.v55_trial_id);
    number(j, "probe_direction", p.context.v55_probe_direction);
    number(j, "target_aligned_rpm", p.context.v55_target_aligned_rpm);
    number(j, "status_sample_time_us", p.context.v55_status_time_us);
    number(j, "driver_mode", p.context.v55_driver_mode);
    number(j, "driver_status", p.context.v55_driver_status);
    number(j, "driver_error", p.context.v55_driver_error);
    const auto& wa = p.wheel_after;
    number(j, "speed_after_raw", wa.sequence ? static_cast<double>(wa.raw) : NAN);
    number(j, "speed_after_rpm", wa.valid ? wa.rpm() : NAN);
    number(j, "speed_after_begin_us", wa.sequence ? static_cast<double>(wa.begin_us) : NAN);
    number(j, "speed_after_end_us", wa.sequence ? static_cast<double>(wa.time_us) : NAN);
    number(j, "speed_after_sequence", wa.sequence); flag(j, "speed_after_valid", wa.valid);
    number(j, "speed_after_age_from_zero_us", wa.sequence ? static_cast<double>(wa.time_us - p.zero_write_end_us) : NAN);
    j += "\"speed_before_semantics\":\"fresh_I2C_read_before_CURRENT_not_internal_encoder_time\"},";
    }
    sample(j, "first_current_after_start", p.first, o.raw_per_mA);
    sample(j, "current_at_stop", p.at_stop, o.raw_per_mA);
    number(j, "start_to_first_current_us", p.has_first ? static_cast<double>(p.first.time_us - p.nonzero_write_end_us) : NAN);
    number(j, "Q_target", p.context.target); number(j, "Q_pred", p.context.pred);
    number(j, "goal_dir_mA", p.context.goal_dir); number(j, "tau_rise_s", p.context.tau_s);
    number(j, "initial_model_raw_mA", p.context.initial_model_raw_mA);
    number(j, "planned_width_ms", p.context.width_ms); number(j, "battery_mV", p.context.battery_mV);
    number(j, "physical_side", p.context.physical_side); flag(j, "saturated", p.context.saturated);
    number(j, "Q_active_measured", p.measured);
    const auto& stop = p.stopping;
    j += "\"measured_q_stop\":{";
    flag(j, "enabled", stop.enabled);
    if (stop.fixed_duration) {
      flag(j, "fixed_duration_v55", true); flag(j, "q_threshold_enabled", false);
    }
    j += "\"stop_reason\":\""; j += MeasuredQStop::name(stop.reason); j += "\",";
    number(j, "q_wait_limit_us", stop.enabled ? static_cast<double>(stop.wait_limit_us) : NAN);
    number(j, "hard_deadline_us", stop.enabled ? static_cast<double>(stop.hard_deadline_us) : NAN);
    number(j, "forced_stop_deadline_us", stop.enabled ? static_cast<double>(stop.forced_stop_deadline_us) : NAN);
    number(j, "planned_wait_deadline_us", stop.enabled ? static_cast<double>(stop.planned_wait_deadline_us) : NAN);
    flag(j, "deadline_critical_entered", stop.deadline_critical_entered);
    number(j, "deadline_critical_enter_us", stop.deadline_critical_entered ? static_cast<double>(stop.deadline_critical_enter_us) : NAN);
    number(j, "Q_final", stop.enabled ? stop.q : NAN);
    number(j, "Q_deficit", stop.enabled ? p.context.target - stop.q : NAN);
    number(j, "Q_final_before_forced_stop", stop.reason == MeasuredQStop::HARD_WIDTH_LIMIT ? stop.q : NAN);
    number(j, "decision_time_us", stop.reason != MeasuredQStop::NONE ? static_cast<double>(stop.decision_us) : NAN);
    const bool reached = stop.target_reached;
    flag(j, "target_reached_at_decision", reached);
    number(j, "previous_Q", reached ? stop.previous_q : NAN);
    number(j, "current_Q", stop.enabled ? stop.q : NAN);
    number(j, "Q_decision", stop.enabled ? stop.q : NAN);
    number(j, "Q_crossing_time_us", reached ? static_cast<double>(stop.reach_us) : NAN);
    number(j, "q_reach_time_us", reached ? static_cast<double>(stop.reach_us) : NAN);
    number(j, "crossing_current_mA", reached ? stop.crossing_current_mA : NAN);
    number(j, "reach_to_zero_begin_us", reached ? static_cast<double>(static_cast<uint32_t>(p.zero_write_begin_us - stop.reach_us)) : NAN);
    number(j, "reach_to_zero_end_us", reached ? static_cast<double>(static_cast<uint32_t>(p.zero_write_end_us - stop.reach_us)) : NAN);
    number(j, "Q_active_final_measured", p.measured);
    number(j, "Q_deficit_measured", p.context.target - p.measured);
    number(j, "decision_overshoot", reached ? stop.q - p.context.target : NAN);
    const bool stopped = p.phase >= 2 && p.zero_write_end_us != 0;
    const double margin = stopped && stop.enabled ?
        static_cast<double>(static_cast<int32_t>(stop.hard_deadline_us - p.zero_write_end_us)) : NAN;
    number(j, "hard_deadline_margin_us", margin);
    number(j, "zero_end_minus_hard_deadline_us", -margin);
    number(j, "actual_width_us", stopped ? static_cast<double>(static_cast<uint32_t>(p.zero_write_end_us - p.nonzero_write_end_us)) : NAN);
    number(j, "active_end_unobserved_us", stopped && p.at_stop.valid ? static_cast<double>(static_cast<uint32_t>(p.zero_write_end_us - p.at_stop.time_us)) : NAN);
    number(j, "deadline_to_zero_end_us", stopped && stop.enabled ? static_cast<double>(static_cast<int32_t>(p.zero_write_end_us - p.nonzero_write_end_us - stop.wait_limit_us)) : NAN);
    j += "\"write_delay_Q_is_directly_measured\":false,\"boundary_linear_Q\":\"offline_diagnostic_only\"},";
    number(j, "Q_edge_linear_start", p.start_linear); number(j, "Q_edge_model_start", p.start_model);
    number(j, "Q_active_start_linear_corrected", p.linear_available ? p.measured + p.start_linear : NAN);
    number(j, "Q_active_start_model_corrected", p.model_available ? p.measured + p.start_model : NAN);
    number(j, "Q_post_measured_window", p.post_measured);
    number(j, "Q_post_100ms_measured", p.end_reason == QObserver::WINDOW_100MS ? p.post_measured : NAN);
    number(j, "Q_post_truncated_measured", p.end_reason != QObserver::WINDOW_100MS ? p.post_measured : NAN);
    const bool valid = p.phase == 3 && p.start_ok && p.stop_ok && !p.active_bad && p.active_samples >= 2 &&
        isfinite(p.context.target) && p.context.target > 0 && !o.sample_overflow && !o.pulse_overflow;
    reach(j, "shadow_A", p.a, valid);
    reach(j, "shadow_B", p.b, valid && p.linear_available);
    reach(j, "shadow_C", p.c, valid && p.model_available);
    j += "\"model_correction_is_independent_validation\":false}";
  }
  j += "]}";
  const auto& wheel = o.wheel;
  j += ",\"wheel_observer_v54\":{\"schema\":\"";
  j += o.v55.enabled ? "idle_and_probe_boundary_v2" : "idle_speed_v1";
  j += "\",";
  flag(j, "control_connected", o.v55.enabled);
  flag(j, "normal_energy_control_connected", false);
  flag(j, "discrete_preparation_connected_v55", o.v55.enabled);
  flag(j, "post_speed_read_enabled_v55", o.v55.enabled);
  j += "\"register\":96,\"raw_per_rpm\":100,\"sign_definition\":\"positive_driver_speed_readback\",";
  j += "\"physical_sign_status\":\"NOT_YET_VERIFIED_ON_THIS_UNIT\",\"internal_update_period_verified\":false,";
  j += "\"sample_time_semantics\":\"I2C_read_completion_not_internal_sensor_time\",\"active_speed_reads_enabled\":false,";
  number(j, "period_us", WheelObserver::PERIOD_US); number(j, "count", wheel.count);
  number(j, "overflow", wheel.overflow); number(j, "failures", wheel.failures);
  flag(j, "allocated", wheel.samples != nullptr);
  j += "\"sample_columns\":[\"begin_us\",\"end_us\",\"sequence\",\"speed_raw\",\"valid\"],\"samples\":[";
  for (uint32_t i = 0; i < wheel.count; ++i) {
    const auto& w = wheel.samples[i]; char b[128];
    snprintf(b, sizeof(b), "%s[%lu,%lu,%lu,%ld,%u]", i ? "," : "",
        static_cast<unsigned long>(w.begin_us), static_cast<unsigned long>(w.time_us),
        static_cast<unsigned long>(w.sequence), static_cast<long>(w.raw), w.valid ? 1 : 0);
    j += b;
  }
  j += "]}";
}

void appendWheelProbeV55Json(String& j, const QObserver& o) {
  const auto& r = o.v55;
  j += ",\"wheel_probe_v55\":{\"schema\":\"fixed_probe_60ms_v1\",";
  flag(j, "enabled", r.enabled); flag(j, "started", r.started);
  flag(j, "finished", r.finished); flag(j, "aborted", r.aborted);
  number(j, "start_us", r.start_us); number(j, "end_us", r.end_us);
  number(j, "trial_index", r.index); number(j, "trial_count", WheelProbeV55::TRIALS);
  number(j, "probe_current_mA", WheelProbeV55::CURRENT_MA);
  number(j, "probe_duration_us", WheelProbeV55::PROBE_MS * 1000UL);
  number(j, "speed_age_limit_us", WheelProbeV55::MAX_AGE_US);
  number(j, "speed_band_rpm", WheelProbeV55::BAND_RPM);
  number(j, "initial_speed_limit_rpm", WheelProbeV55::MAX_INITIAL_RPM);
  number(j, "maximum_preparations_per_trial", WheelProbeV55::MAX_PREP);
  number(j, "inter_pulse_rest_us", WheelProbeV55::REST_US);
  number(j, "run_limit_ms", WheelProbeV55::RUN_LIMIT_MS);
  number(j, "active_bus_timeout_ms", 1);
  j += "\"measured_q_stop_for_probe\":false,\"normal_mode_q_stop_unchanged\":true,";
  j += "\"Q_name\":\"Q_probe_60ms\",\"Q_100ms_extrapolation\":false,";
  j += "\"active_speed_read\":false,\"post_speed_policy\":\"after_first_post_current_bracket\",";
  j += "\"sensor_internal_freshness_verified\":false,\"motor_temperature\":null,\"driver_temperature\":null,";
  j += "\"temperature_status\":\"NOT_ACQUIRED\",\"abort_reason\":\"";
  j += WheelProbeV55::name(r.abort_reason); j += "\",\"trials\":[";
  for (uint8_t i = 0; i < WheelProbeV55::TRIALS; ++i) {
    if (i) j += ",";
    const auto& t = r.trials[i]; j += "{";
    number(j, "trial_id", t.id); number(j, "direction", t.direction);
    number(j, "target_aligned_rpm", t.target_aligned_rpm);
    number(j, "preparation_count", t.preparation_count);
    number(j, "start_us", t.start_us); number(j, "end_us", t.end_us);
    number(j, "probe_pulse_id", t.probe_pulse_id);
    j += "\"result\":\""; j += WheelProbeV55::name(t.result); j += "\"}";
  }
  j += "]}";
}
