#include "current_timing_json.h"

static void number(String& json, const char* key, uint32_t value) {
  json += '"'; json += key; json += "\":"; json += String(value); json += ',';
}
static void appendBool(String& json, const char* key, bool value) {
  json += '"'; json += key; json += "\":"; json += value ? "true," : "false,";
}
static void stats(String& json, const CurrentTimingAudit::Stats& s) {
  json += '{';
  number(json, "count", s.count); number(json, "max_us", s.max_us);
  number(json, "gt_3333_count", s.gt3333); number(json, "gt_5000_count", s.gt5000);
  number(json, "gt_10000_count", s.gt10000);
  json += "\"histogram_100us\":[";
  for (size_t i = 0; i < CurrentTimingAudit::HISTOGRAM_BINS; ++i) {
    if (i) json += ',';
    json += String(s.bins[i]);
  }
  json += "]}";
}
void appendCurrentTimingJson(String& json, const CurrentTimingAudit& a) {
  json += ",\"current_timing_v49\":{\"revision\":\"v49_observation_only\",";
  number(json, "gap_limit_us", CurrentTimingAudit::LIMIT_US);
  number(json, "tail_window_us", CurrentTimingAudit::TAIL_US);
  number(json, "started_pulse_count", a.started_pulse_count);
  number(json, "pulse_count", a.pulse_count);
  number(json, "pulse_overflow", a.pulse_overflow);
  number(json, "violation_count", a.violation_count);
  number(json, "violation_overflow", a.violation_overflow);
  appendBool(json, "active_at_export", a.active);
  json += "\"time_semantics\":\"micros_mod32;read_completion;end=completed_zero_command_write;time_to_end=actual_end_minus_gap_end\",";
  json += "\"pulse_id_semantics\":\"run_local_successful_nonzero_command_episode_ordinal;check_event_count_before_pairing\",";
  json += "\"task_duration_semantics\":\"sum_of_executed_task_durations_since_previous_successful_read;temporal_attribution_not_proof_of_causation\",";
  json += "\"percentile_semantics\":\"100us_histogram_bounds;last_bin_is_ge10000;threshold_counts_and_max_are_exact\",";
  json += "\"whole\":"; stats(json, a.whole);
  json += ",\"tail\":"; stats(json, a.tail);
  json += ",\"pulses\":[";
  for (uint32_t i = 0; i < a.pulse_count; ++i) {
    if (i) json += ',';
    const auto& p = a.pulses[i];
    json += '{';
    number(json, "pulse_id", p.pulse_id); number(json, "start_us", p.start_us);
    number(json, "end_us", p.end_us); number(json, "current_read_count", p.current_read_count);
    number(json, "tail_current_read_count", p.tail_current_read_count);
    number(json, "interval_count", p.interval_count);
    number(json, "current_gap_max_us", p.current_gap_max_us);
    number(json, "current_gap_gt3333_count", p.current_gap_gt3333_count);
    number(json, "tail_interval_count", p.tail_interval_count);
    number(json, "tail_current_gap_max_us", p.tail_current_gap_max_us);
    number(json, "tail_current_gap_gt3333_count", p.tail_current_gap_gt3333_count);
    number(json, "pulse_end_current_age_us", p.pulse_end_current_age_us);
    number(json, "failed_reads", p.failed_reads);
    number(json, "pre_imu_read_attempts", p.pre_imu_reads);
    number(json, "post_imu_read_attempts", p.post_imu_reads);
    number(json, "pre_status_read_attempts", p.pre_status_reads);
    number(json, "post_status_read_attempts", p.post_status_reads);
    json += "\"tail_covered\":"; json += p.tail_covered ? "true}" : "false}";
  }
  json += "],\"violations\":[";
  for (uint32_t i = 0; i < a.violation_count; ++i) {
    if (i) json += ',';
    const auto& v = a.violations[i];
    json += '{';
    number(json, "gap_us", v.gap_us); number(json, "gap_start_time", v.gap_start_time);
    number(json, "gap_end_time", v.gap_end_time); number(json, "pulse_id", v.pulse_id);
    number(json, "time_to_pulse_end_us", v.time_to_pulse_end_us);
    appendBool(json, "imu_was_executed", v.tasks & CurrentTimingAudit::IMU);
    number(json, "imu_duration_us", v.imu_duration_us);
    appendBool(json, "full_status_was_executed", v.tasks & CurrentTimingAudit::FULL_STATUS);
    number(json, "full_status_duration_us", v.full_status_duration_us);
    appendBool(json, "web_was_executed", v.tasks & CurrentTimingAudit::WEB);
    number(json, "web_duration_us", v.web_duration_us);
    number(json, "previous_current_read_duration_us", v.previous_current_read_duration_us);
    number(json, "current_read_duration_us", v.current_read_duration_us);
    number(json, "failed_reads_since_previous_success", v.failed_reads);
    json += "\"end_age_only\":"; json += v.end_age_only ? "true}" : "false}";
  }
  json += "]}";
}
