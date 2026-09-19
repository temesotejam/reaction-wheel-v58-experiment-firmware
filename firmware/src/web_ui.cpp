#include "web_ui.h"

#include <WiFi.h>

#include "config.h"

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="ja"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>V58 Paired Probe / Coast</title><style>
body{font:16px system-ui;background:#f3f5f8;color:#17202a;margin:0}main{max-width:680px;margin:auto;padding:20px}section{padding:18px;margin:16px 0;background:white;border:1px solid #c7d1dc;border-radius:8px}button,a{display:block;width:100%;box-sizing:border-box;padding:14px;margin:12px 0;border:0;border-radius:6px;background:#175fc5;color:white;text-align:center;font-size:16px;text-decoration:none}.danger{background:#b51f2a}button:disabled,a.disabled{opacity:.4;pointer-events:none}small{display:block;line-height:1.6;color:#485365}pre{white-space:pre-wrap}</style></head><body><main><h1>V58 Paired Probe / Coast</h1><p id="state">接続中…</p><section><b>300 mA PROBE と 0 mA COAST の比較</b><p>同じ準備条件でPROBEとCOASTを順に測り、実際の初期速度・残留電流・電圧と50 ms後の速度を記録します。COASTは電流モード0 mAでの観測です。</p><button id="small" onclick="post('/start-fixed-probe?full=0')">Small Run：20有効イベント</button><button id="full" onclick="post('/start-fixed-probe?full=1')">Full Run：100有効イベント</button><small>速度5領域 × 正負2方向 × PROBE/COAST。Fullは各条件5有効イベントを目標にします。未達条件は再試行し、Smallは最大40試行、Fullは最大200試行・10分で終了します。条件不足の場合もログを保存してください。</small><pre id="progress"></pre></section><section><b>測定前の確認</b><p>機体を固定し、回転部から手を離してください。モータ電源と接続を確認してください。</p><small>測定中は電流記録を優先するため、画面とWeb停止の応答が遅れる場合があります。緊急時はAtom本体のボタンで停止するか、モータ電源を切ってください。</small></section><button id="stop" class="danger" onclick="post('/stop')">緊急停止</button><a id="download" href="/download/rwlog" onclick="downloading=true;setTimeout(()=>downloading=false,10000)">RWLOGを保存</a><button id="clear" onclick="if(confirm('現在のログを消去しますか？'))post('/clear')">ログを消去</button><p id="error"></p></main><script>
const ids=['small','full','stop','download','clear'];const el=id=>document.getElementById(id);let downloading=false;
function lock(id,b){const e=el(id);e.disabled=b;e.classList.toggle('disabled',b);}
async function post(path){try{const r=await fetch(path,{method:'POST'});if(!r.ok)el('error').textContent=await r.text();await refresh();}catch(e){el('error').textContent='接続を確認してください';}}
async function refresh(){if(downloading)return;try{const r=await fetch('/status.json',{cache:'no-store'}),j=await r.json();const run=!!j.running,busy=!!j.downloading,ready=['READY_TO_MEASURE','FINISHED'].includes(j.state);el('state').textContent=`${j.state} | 電流 ${j.roller_actual_current_mA||0} mA | 残り ${j.remaining_s||0} s`;el('progress').textContent=`結果: ${j.fixed_probe_result||'待機'}\n有効イベント: ${j.fixed_probe_valid_count||0} / ${j.fixed_probe_valid_goal||0}\n試行: ${j.fixed_probe_trial_index||0}\n条件別有効数: ${(j.fixed_probe_valid_by_condition||[]).join(', ')}`;el('error').textContent=j.last_error||'';lock('small',run||busy||!ready);lock('full',run||busy||!ready);lock('stop',!run);lock('clear',run||busy);lock('download',run||busy||j.rwlog_downloadable!=='yes');}catch(e){el('state').textContent='接続待ち（測定中は画面更新が待機します）';['small','full','download','clear'].forEach(id=>lock(id,true));}}
setInterval(refresh,1000);refresh();</script></body></html>
)HTML";

bool WebUi::begin(WebServer& server, ExperimentRunner& runner, ImuManager& imu, Roller485Manager& roller, PsramLogger& logger) {
  server_ = &server;
  runner_ = &runner;
  imu_ = &imu;
  roller_ = &roller;
  logger_ = &logger;

  // Start the AP before storage, IMU, and Roller485 initialization. This keeps
  // the SSID visible even when a later peripheral requires extra startup time.
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(50);
  WiFi.mode(WIFI_AP);

  bool ap_ok = false;
  constexpr uint8_t kApStartAttempts = 3;
  for (uint8_t attempt = 1; attempt <= kApStartAttempts; ++attempt) {
    ap_ok = WiFi.softAP(Config::AP_SSID, Config::AP_PASS, Config::AP_CHANNEL);
    const IPAddress ip = WiFi.softAPIP();
    Serial.printf("WiFi AP attempt %u/%u: %s ssid=%s channel=%u ip=%u.%u.%u.%u\n",
                  static_cast<unsigned>(attempt), static_cast<unsigned>(kApStartAttempts),
                  ap_ok ? "OK" : "FAILED", Config::AP_SSID,
                  static_cast<unsigned>(Config::AP_CHANNEL),
                  ip[0], ip[1], ip[2], ip[3]);
    if (ap_ok) break;

    WiFi.softAPdisconnect(true);
    delay(100);
    WiFi.mode(WIFI_OFF);
    delay(50);
    WiFi.mode(WIFI_AP);
  }
  server_->on("/", HTTP_GET, [this]() { handleRoot(); });
  server_->on("/status.json", HTTP_GET, [this]() { handleStatus(); });
  server_->on("/start-wheel-probe",HTTP_POST,[this](){
    server_->send(410,"text/plain","V57 has no Q capability probe");});
  server_->on("/start-fixed-probe",HTTP_POST,[this](){
    if(logger_->downloading()||runner_->running()){server_->send(409,"text/plain","busy");return;}
    const bool good=runner_->startFixedProbeCapture(server_->arg("full")=="1");
    server_->send(good?200:409,"text/plain",good?"fixed_probe_started":runner_->status().last_error);
  });
  server_->on("/start-passive", HTTP_POST, [this]() { server_->send(409,"text/plain","v58_paired_probe_coast_only"); });
  server_->on("/start-energy-control-v0", HTTP_POST, [this]() { server_->send(409,"text/plain","v58_paired_probe_coast_only"); });
  server_->on("/start-energy-control-autonomous", HTTP_POST, [this]() { server_->send(409,"text/plain","v58_paired_probe_coast_only"); });
  server_->on("/energy-control-autonomous/target", HTTP_POST, [this]() { handleSetEnergyControlAutonomousTarget(); });
  server_->on("/stop", HTTP_POST, [this]() { handleStop(); });
  server_->on("/clear", HTTP_POST, [this]() { handleClear(); });
  server_->on("/settings", HTTP_POST, [this]() { handleSettings(); });
  server_->on("/current-roll/zero", HTTP_POST, [this]() { handleCurrentRollZero(); });
  server_->on("/current-roll/target", HTTP_POST, [this]() { handleSetCurrentRollTarget(); });
  server_->on("/q1-shadow/target", HTTP_POST, [this]() { handleSetQ1ShadowTargetPeakAbs(); });
  server_->on("/download/rwlog", HTTP_GET, [this]() { handleRwLog(); });
  ap_ok_ = ap_ok;
  next_ap_health_report_ms_ = 0;
  server_->begin();
  return ap_ok_;
}

void WebUi::update() {
  if (server_) server_->handleClient();

  const uint32_t now_ms = millis();
  if (static_cast<int32_t>(now_ms - next_ap_health_report_ms_) < 0) return;
  next_ap_health_report_ms_ = now_ms + 5000;

  const wifi_mode_t mode = WiFi.getMode();
  if (!ap_ok_ || mode != WIFI_AP) {
    WiFi.mode(WIFI_AP);
    ap_ok_ = WiFi.softAP(Config::AP_SSID, Config::AP_PASS, Config::AP_CHANNEL);
  }

  const IPAddress ip = WiFi.softAPIP();
  Serial.printf("WiFi AP health: %s mode=%u ssid=%s channel=%u clients=%u ip=%u.%u.%u.%u\n",
                ap_ok_ ? "OK" : "FAILED", static_cast<unsigned>(WiFi.getMode()),
                Config::AP_SSID, static_cast<unsigned>(Config::AP_CHANNEL),
                static_cast<unsigned>(WiFi.softAPgetStationNum()), ip[0], ip[1], ip[2], ip[3]);
}

void WebUi::handleRoot() {
  server_->sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server_->sendHeader("Pragma", "no-cache");
  server_->send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void WebUi::handleStatus() {
  server_->send(200, "application/json", statusJson());
}

void WebUi::handleStartPassive() {
  if (logger_->downloading()) {
    server_->send(409, "text/plain", "download_in_progress");
    return;
  }
  const bool ok = runner_->startPassiveCapture();
  server_->send(ok ? 200 : 409, "text/plain", ok ? "passive_capture_started" : "start_failed");
}

void WebUi::handleStartEnergyControlV0() {
  if (logger_->downloading()) {
    server_->send(409, "text/plain", "download_in_progress");
    return;
  }
  const bool ok = runner_->startEnergyControlV0Capture();
  server_->send(ok ? 200 : 409, "text/plain",
                ok ? "energy_control_v0_started" : runner_->status().last_error);
}

void WebUi::handleStartEnergyControlAutonomous() {
  if (logger_->downloading()) { server_->send(409, "text/plain", "download_in_progress"); return; }
  const bool ok = runner_->startEnergyControlAutonomousCapture();
  server_->send(ok ? 200 : 409, "text/plain", ok ? "energy_control_autonomous_started" : runner_->status().last_error);
}

void WebUi::handleSetEnergyControlAutonomousTarget() {
  if (!server_->hasArg("deg")) { server_->send(400, "text/plain", "target_deg_required"); return; }
  if (runner_->running()) { server_->send(409, "text/plain", "running"); return; }
  const bool ok = runner_->setEnergyControlAutonomousTarget(server_->arg("deg").toFloat());
  server_->send(ok ? 200 : 400, "text/plain", ok ? "energy_target_set" : runner_->status().last_error);
}

void WebUi::handleStartQIdent() {
  server_->send(409, "text/plain", "q_ident_frozen_use_energy_control_v0");
}void WebUi::handleStart() {
  if (logger_->downloading()) {
    server_->send(409, "text/plain", "download_in_progress");
    return;
  }
  if (!server_->hasArg("trial")) {
    server_->send(400, "text/plain", "trial_required");
    return;
  }
  const uint8_t trial_number = static_cast<uint8_t>(server_->arg("trial").toInt());
  const bool ok = runner_->startSingleTrialTest(trial_number);
  server_->send(ok ? 200 : 409, "text/plain", ok ? "started" : "start_failed");
}

void WebUi::handleStartZeroCross() {
  if (logger_->downloading()) {
    server_->send(409, "text/plain", "download_in_progress");
    return;
  }
  if (!server_->hasArg("pulse_width_ms")) {
    server_->send(400, "text/plain", "pulse_width_ms_required");
    return;
  }
  const int16_t current_mA = Config::ZERO_CROSS_OPERATING_CURRENT_MA;
  const int pulse_width_ms = server_->arg("pulse_width_ms").toInt();
  const bool pulse_ok = pulse_width_ms >= Config::ZERO_CROSS_TIME_SWEEP_MIN_PULSE_MS &&
                        pulse_width_ms <= Config::ZERO_CROSS_TIME_SWEEP_MAX_PULSE_MS;
  if (!pulse_ok) {
    server_->send(400, "text/plain", "invalid_zero_cross_condition");
    return;
  }
  const bool ok = runner_->startZeroCrossTest(static_cast<int16_t>(current_mA),
                                              static_cast<uint16_t>(pulse_width_ms));
  server_->send(ok ? 200 : 409, "text/plain", ok ? "zero_cross_started" : "start_failed");
}
void WebUi::handleStartIdentification() {
  if (logger_->downloading()) { server_->send(409, "text/plain", "download_in_progress"); return; }
  const bool ok = runner_->startZeroCrossIdentificationTest();
  server_->send(ok ? 200 : 409, "text/plain", ok ? "validation_started" : "start_failed");
}

void WebUi::handleStartControl() {
  if (logger_->downloading()) { server_->send(409, "text/plain", "download_in_progress"); return; }
  if (!server_->hasArg("target_peak_deg")) { server_->send(400, "text/plain", "target_peak_deg_required"); return; }
  const float target_peak_deg = server_->arg("target_peak_deg").toFloat();
  const int schedule_arg = server_->hasArg("q_probe_schedule_id") ?
      server_->arg("q_probe_schedule_id").toInt() : Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A;
  if (schedule_arg < Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A ||
      schedule_arg >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_COUNT) {
    server_->send(400, "text/plain", "invalid_q_probe_schedule");
    return;
  }
  const bool ok = runner_->startZeroCrossControlTest(
      target_peak_deg, static_cast<uint8_t>(schedule_arg));
  server_->send(ok ? 200 : 409, "text/plain", ok ? "control_started" : "start_failed");
}
void WebUi::handleZero() {
  if (runner_->running()) {
    server_->send(409, "text/plain", "running");
    return;
  }
  runner_->zeroAngleNow();
  server_->send(200, "text/plain", "zeroed");
}

void WebUi::handleCurrentRollZero() {
  const bool ok = runner_->zeroCurrentRollDisplay();
  server_->send(ok ? 200 : 409, "text/plain", ok ? "current_roll_zeroed" : runner_->status().last_error);
}

void WebUi::handleSetCurrentRollTarget() {
  if (!server_->hasArg("deg")) {
    server_->send(400, "text/plain", "target_deg_required");
    return;
  }
  if (runner_->running()) {
    server_->send(409, "text/plain", "running");
    return;
  }
  const bool ok = runner_->setCurrentRollTarget(server_->arg("deg").toFloat());
  server_->send(ok ? 200 : 400, "text/plain", ok ? "current_roll_target_set" : runner_->status().last_error);
}

void WebUi::handleSetQ1ShadowTargetPeakAbs() {
  if (!server_->hasArg("deg")) {
    server_->send(400, "text/plain", "target_deg_required");
    return;
  }
  if (runner_->running()) {
    server_->send(409, "text/plain", "running");
    return;
  }
  const bool ok = runner_->setQ1ShadowTargetPeakAbs(server_->arg("deg").toFloat());
  server_->send(ok ? 200 : 400, "text/plain", ok ? "q1_shadow_target_set" : runner_->status().last_error);
}

void WebUi::handleStop() {
  runner_->requestEmergencyStop("web_estop");
  server_->send(200, "text/plain", "stopped");
}

void WebUi::handleClear() {
  if (runner_->running() || logger_->downloading()) {
    server_->send(409, "text/plain", "busy");
    return;
  }
  runner_->clearFinishedOrEstop();
  server_->send(200, "text/plain", "cleared");
}

void WebUi::handleSettings() {
  if (runner_->running()) {
    server_->send(409, "text/plain", "running");
    return;
  }
  const int16_t current_mA = server_->hasArg("current_mA") ? static_cast<int16_t>(server_->arg("current_mA").toInt())
                                                           : Config::DEFAULT_INPUT_CURRENT_MA;
  const uint16_t pulse_width_ms =
      server_->hasArg("pulse_width_ms") ? static_cast<uint16_t>(server_->arg("pulse_width_ms").toInt())
                                        : Config::DEFAULT_PULSE_WIDTH_MS;
  const uint16_t input_interval_ms =
      server_->hasArg("input_interval_ms") ? static_cast<uint16_t>(server_->arg("input_interval_ms").toInt())
                                           : Config::DEFAULT_INPUT_INTERVAL_MS;
  runner_->setInputSettings(current_mA, pulse_width_ms, input_interval_ms);
  server_->send(200, "text/plain", "settings_applied");
}

void WebUi::handleRwLog() {
  if (runner_->running()) {
    server_->send(409, "text/plain", "measurement_running");
    return;
  }
  logger_->streamRwLog(*server_);
}

void WebUi::appendJsonUint64(String& json, uint64_t value) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(value));
  json += buf;
}

String WebUi::statusJson() const {
  const auto& st = runner_->status();
  const auto& roller = roller_->telemetry();
  char filename[72];
  logger_->downloadFilename(filename, sizeof(filename));
  String json;
  json.reserve(3200);
  json += "{";
  json += "\"running\":" + String(runner_->running() ? "true" : "false");
  json += ",\"downloading\":" + String(logger_->downloading() ? "true" : "false");
  json += ",\"zero_cross_mode\":" + String(runner_->zeroCrossMode() ? "true" : "false");
  json += ",\"identification_mode\":" + String(runner_->identificationMode() ? "true" : "false");
  json += ",\"q_run_mode\":\"" + String(runner_->qRunModeName()) + "\"";
  json += ",\"passive_capture_mode\":" + String(runner_->passiveCaptureMode() ? "true" : "false");
  json += ",\"q_ident_mode\":" + String(runner_->qIdentMode() ? "true" : "false");
  json += ",\"energy_control_v0_mode\":" + String(runner_->energyControlV0Mode() ? "true" : "false");
  json += ",\"energy_control_autonomous_mode\":" + String(runner_->energyControlAutonomousMode() ? "true" : "false");
  json += ",\"energy_control_autonomous_target_peak_deg\":" + String(runner_->energyControlAutonomousTargetPeakDeg(), 2);
  json += ",\"energy_control_autonomous_phase\":\"" + String(runner_->energyControlAutonomousPhaseName()) + "\"";
  json += ",\"energy_control_v0_target_peak_deg\":" + String(Config::ENERGY_CONTROL_V0_TARGET_PEAK_DEG, 2);
  json += ",\"q_ident_armed\":" + String(runner_->qIdentArmed() ? "true" : "false");
  json += ",\"q_ident_run_schedule_id\":" + String(runner_->qIdentRunScheduleId());
  json += ",\"q_ident_plus_occurrences\":" + String(runner_->qIdentPlusOccurrenceCount());
  json += ",\"q_ident_minus_occurrences\":" + String(runner_->qIdentMinusOccurrenceCount());
  json += ",\"passive_static_window_s\":" + String(static_cast<float>(Config::PASSIVE_STATIC_WINDOW_MS) / 1000.0f, 1);
  json += ",\"q_probe_schedule_id\":" + String(runner_->qProbeScheduleId());
  json += ",\"q_probe_schedule_name\":\"" + String(runner_->qProbeScheduleName()) + "\"";
  json += ",\"q_control_target_peak_deg\":" + String(runner_->controlTargetPeakDeg(), 2);
  json += ",\"zero_cross_fixed_current_mA\":" + String(runner_->zeroCrossFixedCurrentMa());
  json += ",\"state\":\"" + String(runner_->stateName()) + "\"";
  json += ",\"state_id\":" + String(static_cast<uint8_t>(st.state));
  json += ",\"boot_elapsed_s\":" + String(static_cast<float>(st.boot_elapsed_ms) / 1000.0f, 1);
  json += ",\"measure_elapsed_s\":" + String(static_cast<float>(st.measure_elapsed_ms) / 1000.0f, 1);
  json += ",\"remaining_s\":" + String(static_cast<float>(st.remaining_ms) / 1000.0f, 1);
  json += ",\"trial_index\":" + String(st.trial_index);
  json += ",\"trial_count\":" + String(st.trial_count);
  json += ",\"trial_elapsed_s\":" + String(static_cast<float>(st.trial_elapsed_ms) / 1000.0f, 1);
  json += ",\"trial_duration_s\":" + String(static_cast<float>(st.trial_duration_ms) / 1000.0f, 1);
  json += ",\"pulse_id\":" + String(st.pulse_id);
  json += ",\"pulse_active\":" + String(st.pulse_active ? "true" : "false");
  json += ",\"pulse_direction\":" + String(st.pulse_direction);
  json += ",\"current_mA_setting\":" + String(st.current_mA_setting);
  json += ",\"pulse_width_ms_setting\":" + String(st.pulse_width_ms_setting);
  json += ",\"input_interval_ms\":" + String(st.input_interval_ms);
  json += ",\"predicted_beta_min\":" + String(st.predicted_beta_min, 5);
  json += ",\"beta_hold_after_input_ms\":" + String(st.beta_hold_after_input_ms_setting);
  json += ",\"beta_recovery_tau_s_setting\":" + String(st.beta_recovery_tau_s_setting, 3);
  json += ",\"beta_model_vbat_mV\":" + String(st.beta_model_vbat_mV);
  json += ",\"predicted_i_goal_mA\":" + String(st.predicted_i_goal_mA);
  json += ",\"predicted_peak_current_mA\":" + String(st.predicted_peak_current_mA);
  json += ",\"beta_model_vbat_status\":" + String(st.beta_model_vbat_status);
  json += ",\"led_state\":" + String(st.led_state ? "true" : "false");
  json += ",\"sync_event_id\":" + String(st.sync_event_id);
  json += ",\"led_sync_pattern_id\":\"" + String(Config::LED_SYNC_PATTERN_ID) + "\"";
  json += ",\"gyro_bias_x_dps\":" + String(st.gyro_bias_x_dps, 5);
  json += ",\"gyro_bias_y_dps\":" + String(st.gyro_bias_y_dps, 5);
  json += ",\"gyro_bias_z_dps\":" + String(st.gyro_bias_z_dps, 5);
  json += ",\"pitch_madgwick_beta1_raw_deg\":" + String(st.pitch_madgwick_beta1_raw_deg, 3);
  json += ",\"pitch_madgwick_dynamic_raw_deg\":" + String(st.pitch_madgwick_dynamic_raw_deg, 3);
  json += ",\"pitch_madgwick_beta1_bias_deg\":" + String(st.pitch_madgwick_beta1_bias_deg, 3);
  json += ",\"pitch_madgwick_dynamic_bias_deg\":" + String(st.pitch_madgwick_dynamic_bias_deg, 3);
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    json += ",\"pitch_beta_series_" + String(i) + "\":" + String(st.pitch_dynamic_beta_deg[i], 3);
    json += ",\"beta_applied_series_" + String(i) + "\":" + String(st.beta_smooth_series[i], 5);
  }
  json += ",\"pitch_gyro_raw_deg\":" + String(st.pitch_gyro_raw_deg, 3);
  json += ",\"pitch_gyro_bias_corrected_deg\":" + String(st.pitch_gyro_bias_corrected_deg, 3);
  json += ",\"pitch_accel_only_deg\":" + String(st.pitch_accel_only_deg, 3);
  json += ",\"gyro_pitch_rate_dps\":" + String(st.gyro_pitch_rate_dps, 4);
  json += ",\"beta_target\":" + String(st.beta_target, 5);
  json += ",\"beta_smooth\":" + String(st.beta_smooth, 5);
  json += ",\"ax_g\":" + String(st.ax_g, 4);
  json += ",\"ay_g\":" + String(st.ay_g, 4);
  json += ",\"az_g\":" + String(st.az_g, 4);
  json += ",\"gx_dps\":" + String(st.gx_dps, 4);
  json += ",\"gy_dps\":" + String(st.gy_dps, 4);
  json += ",\"gz_dps\":" + String(st.gz_dps, 4);
  json += ",\"acc_norm_g\":" + String(st.acc_norm_g, 4);
  json += ",\"physical_roll_candidate_deg\":" + String(st.physical_roll_candidate_deg, 3);
  json += ",\"physical_roll_abs_deg\":" + String(st.physical_roll_abs_deg, 3);
  json += ",\"current_roll_deg\":" + String(st.current_roll_deg, 3);
  json += ",\"physical_roll_rate_raw_dps\":" + String(st.physical_roll_rate_raw_dps, 4);
  json += ",\"physical_roll_rate_dps\":" + String(st.physical_roll_rate_dps, 4);
  json += ",\"display_zero_offset_deg\":" + String(st.display_zero_offset_deg, 3);
  json += ",\"target_roll_deg\":" + String(st.target_roll_deg, 3);
  json += ",\"q1_shadow_target_peak_abs_deg\":" + String(runner_->q1ShadowTargetPeakAbsDeg(), 3);
  json += ",\"q1_shadow_active_target_peak_abs_deg\":" + String(runner_->q1ShadowActiveTargetPeakAbsDeg(), 3);  json += ",\"target_error_deg\":" + String(st.target_error_deg, 3);
  json += ",\"static_confirmed\":" + String(st.static_confirmed ? "true" : "false");
  json += ",\"ready\":" + String(st.ready ? "true" : "false");
  json += ",\"static_rate_threshold_dps\":" + String(Config::STATIC_RATE_THRESHOLD_DPS, 3);
  json += ",\"static_hold_time_ms\":" + String(Config::STATIC_HOLD_TIME_MS);
  json += ",\"target_tolerance_deg\":" + String(Config::TARGET_TOLERANCE_DEG, 3);
  json += ",\"wheel_probe_mode\":" + String(runner_->wheelProbeMode() ? "true" : "false");
  const auto&v57=roller_->qObserver().v57;
  json += ",\"fixed_probe_mode\":"+String(runner_->fixedProbeMode()?"true":"false");
  json += ",\"fixed_probe_trial_index\":"+String(v57.index<v57.trial_count?v57.index+1:v57.trial_count);
  json += ",\"fixed_probe_trial_count\":"+String(v57.trial_count);
  json += ",\"fixed_probe_valid_count\":"+String(v57.valid_probe_count);
  json += ",\"fixed_probe_valid_goal\":"+String(v57.valid_goal_each*FixedProbeV57::CONDITION_COUNT);
  json += ",\"fixed_probe_valid_by_condition\":[";
  for(uint8_t c=0;c<FixedProbeV57::CONDITION_COUNT;++c){if(c)json+=",";json+=String(v57.valid_by_condition[c]);}json+="]";

  json += ",\"fixed_probe_result\":\""+String(FixedProbeV57::name(v57.reason))+"\"";
  const auto& v55 = roller_->qObserver().v55;
  json += ",\"wheel_probe_trial_index\":" + String(v55.index < WheelProbeV55::TRIALS ? v55.index + 1 : WheelProbeV55::TRIALS);
  json += ",\"wheel_probe_result\":\"" + String(v55.aborted ? WheelProbeV55::name(v55.abort_reason) : v55.finished ? "FINISHED" : "RUNNING") + "\"";
  json += ",\"motor_cmd_mA\":" + String(st.motor_cmd_mA);
  json += ",\"sample_count\":" + String(logger_->sampleCount());
  json += ",\"psram_usage_percent\":" + String(logger_->usagePercent());
  json += ",\"log_capacity\":" + String(logger_->sampleCapacity());
  json += ",\"rwlog_downloadable\":\"" + String(logger_->rwlogDownloadable() ? "yes" : "no") + "\"";
  json += ",\"download_filename\":\"" + String(filename) + "\"";
  json += ",\"run_id\":" + String(logger_->currentRunId());
  json += ",\"run_start_us\":";
  appendJsonUint64(json, logger_->runStartUs());
  json += ",\"last_measurement_done\":\"" + String(logger_->lastMeasurementDone() ? "yes" : "no") + "\"";
  json += ",\"calibration_sample_count\":" + String(st.calibration_sample_count);
  json += ",\"imu_ok\":" + String(imu_->ok() ? "true" : "false");
  json += ",\"roller_ok\":" + String(roller_->ok() ? "true" : "false");
  json += ",\"roller_actual_current_mA\":" + String(roller.actual_current_mA);
  json += ",\"battery_mV\":" + String(roller.battery_mV);
  json += ",\"loop_dt_us\":" + String(st.loop_dt_us);
  json += ",\"log_dt_us\":" + String(st.log_dt_us);
  json += ",\"imu_dt_us\":" + String(imu_->reading().update_dt_us);
  json += ",\"last_error\":\"" + String(st.last_error && st.last_error[0] ? st.last_error : logger_->lastError()) + "\"";
  json += "}";
  return json;
}

