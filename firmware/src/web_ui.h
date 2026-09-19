#pragma once

#include <Arduino.h>
#include <WebServer.h>

#include "experiment_runner.h"

class WebUi {
public:
  bool begin(WebServer& server, ExperimentRunner& runner, ImuManager& imu, Roller485Manager& roller, PsramLogger& logger);
  void update();

private:
  void handleRoot();
  void handleStatus();
  void handleStart();
  void handleStartPassive();
  void handleStartQIdent();
  void handleStartEnergyControlV0();
  void handleStartEnergyControlAutonomous();
  void handleSetEnergyControlAutonomousTarget();
  void handleStartZeroCross();
  void handleStartIdentification();
  void handleStartControl();
  void handleZero();
  void handleCurrentRollZero();
  void handleSetCurrentRollTarget();
  void handleSetQ1ShadowTargetPeakAbs();
  void handleStop();
  void handleClear();
  void handleSettings();
  void handleRwLog();
  String statusJson() const;
  static void appendJsonUint64(String& json, uint64_t value);

  WebServer* server_ = nullptr;
  ExperimentRunner* runner_ = nullptr;
  ImuManager* imu_ = nullptr;
  Roller485Manager* roller_ = nullptr;
  PsramLogger* logger_ = nullptr;
  bool ap_ok_ = false;
  uint32_t next_ap_health_report_ms_ = 0;
};

