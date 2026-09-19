#include <Arduino.h>
#include <M5Unified.h>
#include <WebServer.h>

#include "config.h"
#include "experiment_runner.h"
#include "imu_manager.h"
#include "psram_logger.h"
#include "roller485_manager.h"
#include "timing_audit.h"
#include "web_ui.h"

WebServer server(Config::HTTP_PORT);
PsramLogger logger;
ImuManager imu;
Roller485Manager roller;
ExperimentRunner runner;
WebUi web;
TimingAudit timing_audit;
static uint32_t timing_loop_sequence = 0;
static uint32_t timing_web_sequence = 0;

// One bounded stop-only scheduler slice; the Arduino loop immediately repeats.
// No busy wait inside a bus transaction, no filters/status/Web/log formatting.
static bool serviceDeadlineCritical() {
  if (!roller.deadlineCritical()) return false;
  runner.serviceFast(); // Existing authorization / emergency-stop state first.
  roller.updatePulseCurrent(); // Includes fault, observation and deadline checks.
  runner.serviceFast();
  return true;
}

static void displayLine(const char* line1, const char* line2 = "") {
  if (!M5.Display.width()) return;
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(0, 4);
  M5.Display.println(line1);
  if (line2 && line2[0]) M5.Display.println(line2);
}

void setup() {
  Serial.begin(Config::SERIAL_BAUD);
  delay(300);
  Serial.println();
  Serial.println("AtomS3CAM V58 confound-controlled fixed probe Q50");

  auto cfg = M5.config();
  cfg.serial_baudrate = 0;
  cfg.internal_imu = true;
  M5.begin(cfg);
  Serial.printf("V57 fixed-probe identity: board=%d imu_type=%d M5Unified=%s M5GFX=%s AHRS=%s base=%s\n",
                static_cast<int>(M5.getBoard()), static_cast<int>(M5.Imu.getType()),
                Config::RESOLVED_M5UNIFIED_VERSION, Config::RESOLVED_M5GFX_VERSION,
                Config::RESOLVED_ADAFRUIT_AHRS_VERSION, Config::V62_BASE_COMMIT);
  displayLine("WiFi starting", Config::AP_SSID);
  // Bring up the AP before any peripheral that can delay or fail during setup.
  // WebUi only stores the references here; requests are handled after setup.
  const bool ap_ok = web.begin(server, runner, imu, roller, logger);
  const bool psram_ok = logger.begin();
  Serial.printf("PSRAM: %s total=%u free=%u sample_capacity=%u\n", psram_ok ? "OK" : "FAILED",
                static_cast<unsigned>(logger.psramTotal()), static_cast<unsigned>(logger.psramFree()),
                static_cast<unsigned>(logger.sampleCapacity()));
  if (!psram_ok) Serial.printf("PSRAM error: %s\n", logger.lastError());

  const bool imu_ok = imu.begin();
  Serial.printf("IMU: %s\n", imu_ok ? "OK" : "FAILED");

  const bool roller_ok = roller.begin();
  roller.stop();
  Serial.printf("Roller485: %s\n", roller_ok ? "OK" : "FAILED");

  runner.begin(logger, imu, roller);
  logger.setCurrentTimingAudit(&roller.currentTimingAudit());
  logger.setQObserver(&roller.qObserver());
  Serial.printf("V57 raw-current observer storage: %s; wheel storage: %s\n",
                roller.qObserver().allocated() ? "OK" : "FAILED - normal Q pulse unavailable",
                roller.qObserver().wheel.samples ? "OK" : "FAILED - wheel log unavailable");
  runner.setTimingAudit(&timing_audit);
  imu.setTimingAudit(&timing_audit);
  roller.setTimingAudit(&timing_audit);

  if (ap_ok) {
    Serial.printf("WiFi AP ready: ssid=%s channel=%u url=http://192.168.4.1/\n",
                  Config::AP_SSID, static_cast<unsigned>(Config::AP_CHANNEL));
    displayLine("AP ready", Config::AP_SSID);
  } else {
    Serial.printf("WiFi AP FAILED after 3 attempts: ssid=%s channel=%u\n",
                  Config::AP_SSID, static_cast<unsigned>(Config::AP_CHANNEL));
    displayLine("WiFi AP FAILED", Config::AP_SSID);
  }
}

void loop() {
  if (serviceDeadlineCritical()) return;
  const uint32_t loop_start_us = micros();
  timing_audit = TimingAudit{};
  timing_audit.loop_sequence = ++timing_loop_sequence;

  M5.update();
  if(runner.fixedProbeMode() && runner.running() && M5.BtnA.wasPressed())
    runner.requestEmergencyStop("v57_physical_button_stop");
  const uint32_t t1 = micros();
  timing_audit.m5_update_us = t1 - loop_start_us;

  // Outside the stop-only slice, retain the existing M5 task ordering.
  // Once current is commanded, V47 prioritizes CURRENT_READBACK ahead of the
  // 5 ms IMU task and the 20 ms full Roller status snapshot.
  const bool pulse_current_priority = Config::MODE_A_PULSE_CURRENT_PRIORITY_ENABLED &&
      roller.currentObservationPriority();
  uint32_t roller_total_us = 0;
  uint32_t t_after_roller = t1;

  if (pulse_current_priority) {
    if (serviceDeadlineCritical()) return;
    const uint32_t fast_current_start_us = micros();
    roller.updatePulseCurrent();
    roller_total_us += micros() - fast_current_start_us;

    const uint32_t service_fast_start_us = micros();
    runner.serviceFast();
    const uint32_t t2 = micros();
    timing_audit.service_fast_us = t2 - service_fast_start_us;
    if (serviceDeadlineCritical()) return;
    runner.updateImuDynamicBetaContext();
    const uint32_t t3 = micros();
    timing_audit.beta_context_us = t3 - t2;
    if (imu.updateDue(micros())) {
      const uint32_t pre_start_us = micros();
      roller.servicePulseCurrentBeforeImu();
      roller_total_us += micros() - pre_start_us;
      const uint32_t safety_start_us = micros();
      runner.serviceFast();
      timing_audit.service_fast_us += micros() - safety_start_us;
      runner.updateImuDynamicBetaContext();
      if (serviceDeadlineCritical()) return;
      const uint32_t imu_start_us = micros();
      imu.update();
      const uint32_t imu_duration_us = micros() - imu_start_us;
      timing_audit.imu_update_total_us = imu_duration_us;
      roller.currentTimingAudit().noteTask(CurrentTimingAudit::IMU, imu_duration_us);
      const uint32_t post_start_us = micros();
      roller.servicePulseCurrentAfterImu();
      roller_total_us += micros() - post_start_us;
    }
    const uint32_t t4 = micros();
    if (serviceDeadlineCritical()) return;
    if (roller.fullStatusDue(t4)) {
      const uint32_t pre_start_us = micros();
      roller.servicePulseCurrentBeforeStatus();
      roller_total_us += micros() - pre_start_us;
      const uint32_t safety_start_us = micros();
      runner.serviceFast();
      timing_audit.service_fast_us += micros() - safety_start_us;
      if (serviceDeadlineCritical()) return;
      const uint32_t full_status_start_us = micros();
      roller.updateFullStatus();
      roller.servicePulseCurrentAfterStatus();
      roller_total_us += micros() - full_status_start_us;
    }
    t_after_roller = micros();
    timing_audit.roller_update_total_us = roller_total_us;
  } else {
    runner.serviceFast();
    const uint32_t t2 = micros();
    timing_audit.service_fast_us = t2 - t1;
    runner.updateImuDynamicBetaContext();
    const uint32_t t3 = micros();
    timing_audit.beta_context_us = t3 - t2;
    imu.update();
    const uint32_t t4 = micros();
    timing_audit.imu_update_total_us = t4 - t3;
    const uint32_t roller_start_us = micros();
    roller.update();
    t_after_roller = micros();
    timing_audit.roller_update_total_us = t_after_roller - roller_start_us;
  }

  if (serviceDeadlineCritical()) return;
  runner.update();
  const uint32_t t_after_runner = micros();
  timing_audit.runner_update_total_us = t_after_runner - t_after_roller;

  // This also covers a pulse that was started by runner.update in the current
  // loop, eliminating a post-register-write WebServer block at the pulse edge.
  if (!(Config::MODE_A_PULSE_CURRENT_PRIORITY_ENABLED && roller.currentObservationPriority())) {
    timing_audit.web_task_start_us = t_after_runner;
    timing_audit.web_task_sequence = ++timing_web_sequence;
    web.update();
    timing_audit.web_update_us = micros() - t_after_runner;
    roller.currentTimingAudit().noteTask(CurrentTimingAudit::WEB, timing_audit.web_update_us);
  }
  timing_audit.loop_total_us = micros() - loop_start_us;

  if (serviceDeadlineCritical()) return;
  runner.commitTimingAudit(timing_audit);
  runner.setLoopDt(static_cast<uint32_t>(micros() - loop_start_us));
}
