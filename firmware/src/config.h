#pragma once

#include <Arduino.h>

namespace Config {

static constexpr uint32_t SERIAL_BAUD = 115200;

static constexpr char AP_SSID[] = "AtomS3CAM_Q1_SHADOW";
static constexpr char AP_PASS[] = "12345678";
static constexpr uint8_t AP_CHANNEL = 1;
static constexpr uint16_t HTTP_PORT = 80;

static constexpr uint8_t SYNC_LED_PIN = 38;
static constexpr char LED_SYNC_PATTERN_ID[] = "LED_SYNC_PATTERN_V2_LOGGED_ANCHORS";
static constexpr uint16_t LED_SYNC_SHORT_ON_MS = 600;
static constexpr uint16_t LED_SYNC_LONG_ON_MS = 1200;
static constexpr uint16_t LED_SYNC_HALF_OFF_MS = 300;
static constexpr uint16_t LED_SYNC_BOUNDARY_OFF_MS = 1000;

// Logged video anchors: ON 300 ms, OFF 300 ms, ON 600 ms.
static constexpr uint32_t MID_SYNC_FIRST_MS = 2500UL;
static constexpr uint32_t MID_SYNC_INTERVAL_MS = 5000UL;
static constexpr uint16_t MID_SYNC_SHORT_ON_MS = 300;
static constexpr uint16_t MID_SYNC_GAP_MS = 300;
static constexpr uint16_t MID_SYNC_LONG_ON_MS = 600;
static constexpr uint16_t MID_SYNC_TOTAL_MS = MID_SYNC_SHORT_ON_MS + MID_SYNC_GAP_MS + MID_SYNC_LONG_ON_MS;

struct LedSyncStep {
  bool led_on;
  uint16_t duration_ms;
};

static constexpr LedSyncStep START_SYNC_PATTERN[] = {
    {false, LED_SYNC_BOUNDARY_OFF_MS},
    {true, LED_SYNC_SHORT_ON_MS},
    {false, LED_SYNC_HALF_OFF_MS},
    {true, LED_SYNC_SHORT_ON_MS},
    {false, LED_SYNC_HALF_OFF_MS},
    {true, LED_SYNC_LONG_ON_MS},
    {false, LED_SYNC_BOUNDARY_OFF_MS},
};

static constexpr LedSyncStep END_SYNC_PATTERN[] = {
    {false, LED_SYNC_BOUNDARY_OFF_MS},
    {true, LED_SYNC_LONG_ON_MS},
    {false, LED_SYNC_HALF_OFF_MS},
    {true, LED_SYNC_SHORT_ON_MS},
    {false, LED_SYNC_HALF_OFF_MS},
    {true, LED_SYNC_SHORT_ON_MS},
    {false, LED_SYNC_BOUNDARY_OFF_MS},
};

static constexpr uint8_t START_SYNC_PATTERN_STEP_COUNT = sizeof(START_SYNC_PATTERN) / sizeof(START_SYNC_PATTERN[0]);
static constexpr uint8_t END_SYNC_PATTERN_STEP_COUNT = sizeof(END_SYNC_PATTERN) / sizeof(END_SYNC_PATTERN[0]);
static constexpr uint32_t START_SYNC_PATTERN_TOTAL_MS = 5000UL;
static constexpr uint32_t END_SYNC_PATTERN_TOTAL_MS = 5000UL;

static constexpr uint8_t I2C_SDA_PIN = 2;
static constexpr uint8_t I2C_SCL_PIN = 1;
static constexpr uint32_t I2C_HZ = 400000;
static constexpr uint16_t I2C_TIMEOUT_MS = 20;
static constexpr uint8_t ROLLER_ADDR = 0x64;
static constexpr int32_t ROLLER_CURRENT_RAW_PER_MA = 100;
static constexpr uint8_t ROLLER_MODE_CURRENT = 3;

static constexpr int8_t PITCH_SIGN = 1;
static constexpr int8_t GYRO_PITCH_RATE_SIGN = -1;
// Static physical-roll calibration, fixed from the seven fixture points and
// independently checked at +/-15 and +/-18 deg against the fixed horizon.
// The candidate is atan2(ax, hypot(ay, az)) in degrees. It is an
// accelerometer/gravity estimate and therefore carries static-only confidence.
static constexpr float PHYSICAL_ROLL_AFFINE_SLOPE = 0.9278941864271074f;
static constexpr float PHYSICAL_ROLL_AFFINE_OFFSET_DEG = -0.49830848087090135f;
static constexpr float STATIC_RATE_THRESHOLD_DPS = 0.5f;
static constexpr uint32_t STATIC_HOLD_TIME_MS = 500UL;
static constexpr float TARGET_TOLERANCE_DEG = 0.20f;
static constexpr float CURRENT_ROLL_TARGET_MIN_DEG = -18.0f;
static constexpr float CURRENT_ROLL_TARGET_MAX_DEG = 18.0f;
static constexpr char CURRENT_ROLL_UI_REVISION[] = "current_roll_static_ui_v1_20260828";

static constexpr uint16_t IMU_PERIOD_MS = 5;
static constexpr uint16_t LOG_PERIOD_MS = 20;
static constexpr uint16_t ROLLER_READ_PERIOD_MS = 20;
// Actual-current audit only. While a motor command is active, CURRENT_READBACK
// is sampled on this independent schedule. These samples are observational and
// cannot select, shorten, extend, or otherwise alter a motor pulse.
static constexpr uint32_t CURRENT_AUDIT_FAST_READ_PERIOD_US = 2000UL;
static constexpr uint32_t CURRENT_AUDIT_LOG_PERIOD_US = 2000UL;
// V48 records gaps at each successful CURRENT_READBACK on-device.  The IMU
// guard is a scheduler deadline estimate only; it does not modify output.
static constexpr uint32_t CURRENT_AUDIT_MAX_GAP_US = 3333UL;
static constexpr uint32_t CURRENT_AUDIT_TAIL_WINDOW_US = 10000UL;
static constexpr uint32_t CURRENT_AUDIT_IMU_GUARD_BUDGET_US = 1300UL;
static constexpr uint16_t WEB_UPDATE_PERIOD_MS = 500;
// V47 Mode-A observation preparation. During a nonzero motor command the
// fresh-current read is scheduled before optional work, WebServer service is
// deferred, and only the adopted HOLD_073 Madgwick series advances.
static constexpr bool MODE_A_PULSE_CURRENT_PRIORITY_ENABLED = true;
static constexpr bool AUTONOMOUS_SKIP_COMPARISON_MADGWICK = true;
static constexpr char MODE_A_OBSERVATION_SCHEDULING_REVISION[] =
    "v49_forced_current_around_due_imu_full_status_strict_tail_end_age_diagnostics";
static constexpr uint16_t IMU_ERROR_LIMIT = 10;
static constexpr uint16_t IMU_STALE_LIMIT_MS = 500;

static constexpr uint32_t STARTUP_GYRO_CALIB_MS = 5000UL;
static constexpr uint32_t MADGWICK_SETTLING_MS = 5000UL;

// Dedicated manual-release capture. The first window is held static by the
// operator; it is metadata, not a per-run angle-zero operation.
static constexpr char PASSIVE_CAPTURE_FIRMWARE_REVISION[] = "paired_probe_coast_v58_50ms_20260913";
static constexpr uint32_t PASSIVE_CAPTURE_DURATION_MS = 60000UL;
static constexpr uint32_t PASSIVE_STATIC_WINDOW_MS = 3000UL;

// Q1 direct-video shadow remains a passive-observation build. The dedicated
// Q_IDENT path below is the sole exception, and it is isolated from Q1.
static constexpr bool Q1_SHADOW_MOTOR_OFF_ONLY = true;
static constexpr char Q1_SHADOW_MODEL_NAME[] = "direct_video_q1_20260829";
// Q1 uses exactly the `q_target_mA_s` axis from the Aug-17 canonical video
// reanalysis. `q_model_axis_type` is persisted as the shorter fixed label.
static constexpr char Q1_SHADOW_Q_MODEL_AXIS_TYPE[] = "q_target";
static constexpr char Q1_SHADOW_Q_MODEL_AXIS_FIELD[] = "q_target_mA_s";
static constexpr float Q1_SHADOW_INTERCEPT_DEG = 0.01304f;
static constexpr float Q1_SHADOW_RATE_GAIN_DEG_PER_DPS = 0.08812f;
static constexpr float Q1_SHADOW_SIDE_TERM_DEG = 0.11858f;
static constexpr float Q1_SHADOW_GAIN_PHYSICAL_PLUS_DEG_PER_MAS = 0.29032f;
static constexpr float Q1_SHADOW_GAIN_PHYSICAL_MINUS_DEG_PER_MAS = 0.25455f;
// Q=0 is the baseline class. Inverse-Q validity begins at the smallest
// nonzero accepted `q_target_mA_s` value and never clips outside this range.
static constexpr float Q1_SHADOW_Q_SUPPORT_MIN_MAS = 0.454f;
static constexpr float Q1_SHADOW_Q_SUPPORT_MAX_MAS = 1.197f;
static constexpr float Q1_SHADOW_RATE_SUPPORT_MIN_DPS = 1.68f;
static constexpr float Q1_SHADOW_RATE_SUPPORT_MAX_DPS = 27.80f;
static constexpr float Q1_SHADOW_TARGET_MIN_DEG = 0.0f;
static constexpr float Q1_SHADOW_TARGET_MAX_DEG = 18.0f;
static constexpr float Q1_SHADOW_ZERO_CROSS_REARM_ANGLE_DEG = 0.08f;
static constexpr float Q1_SHADOW_ZERO_CROSS_MIN_ABS_RATE_DPS = 0.25f;
static constexpr uint32_t Q1_SHADOW_ZERO_CROSS_MIN_INTERVAL_MS = 200UL;
static constexpr uint16_t Q1_SHADOW_MAX_EVENTS = 128;
static constexpr uint8_t Q1_SHADOW_INVALID_NONE = 0;
static constexpr uint8_t Q1_SHADOW_INVALID_NONFINITE_STATE = 1;
static constexpr uint8_t Q1_SHADOW_INVALID_RATE_BELOW_SUPPORT = 2;
static constexpr uint8_t Q1_SHADOW_INVALID_RATE_ABOVE_SUPPORT = 3;
static constexpr uint8_t Q1_SHADOW_INVALID_BRAKING_NOT_IDENTIFIED = 4;
static constexpr uint8_t Q1_SHADOW_INVALID_Q_BELOW_SUPPORT = 5;
static constexpr uint8_t Q1_SHADOW_INVALID_Q_ABOVE_SUPPORT = 6;

// Current-hardware fixed-Q identification. This is intentionally not a Q1,
// E2, calibration, rebuild, or inverse-Q controller. The four run schedules
// are selected before recording and cannot adapt to motion or target error.
static constexpr char Q_IDENT_MEASUREMENT_MODE[] = "q1_current_hardware_identification";
static constexpr bool Q_IDENT_FIXED_SCHEDULE = true;
static constexpr bool Q_IDENT_INVERSE_Q_ENABLED = false;
static constexpr int16_t Q_IDENT_CURRENT_MA = 300;
static constexpr uint32_t Q_IDENT_DURATION_MS = 60000UL;
static constexpr float Q_IDENT_ARM_MIN_ABS_RATE_DPS = 30.0f;
static constexpr uint8_t Q_IDENT_ARM_CONSECUTIVE_CROSSES = 2;
static constexpr float Q_IDENT_RATE_SUPPORT_MIN_DPS = 1.68f;
static constexpr float Q_IDENT_RATE_SUPPORT_MAX_DPS = 27.80f;
static constexpr uint16_t Q_IDENT_MIN_PULSE_MS = 5;
static constexpr uint16_t Q_IDENT_MAX_PULSE_MS = 25;
static constexpr uint16_t Q_IDENT_BATTERY_MIN_MV = 6180;
// The Q_IDENT guard is automatic: the observed full-charge range in the
// rejected 2026-09-02 Run 1 was 8,070--8,090 mV.  8,100 mV admits that
// range without removing either guard endpoint or changing pulse limits.
static constexpr uint16_t Q_IDENT_BATTERY_MAX_MV = 8100;
// Qhigh was selected offline at the battery guard minimum and I0=0 mA.  It
// needs 23 integer ms there, leaving 2 ms below the immutable 25 ms guard.
static constexpr float Q_IDENT_Q_HIGH_MAS = 0.900f;
static constexpr uint16_t Q_IDENT_QHIGH_SELECTION_VBAT_MV = 6180;
static constexpr float Q_IDENT_QHIGH_SELECTION_I0_MA = 0.0f;
// Diagnostic-only upper bound; never authorizes a physical pulse beyond 25 ms.
static constexpr uint16_t Q_IDENT_SOLVER_DIAGNOSTIC_MAX_WIDTH_MS = 100;
static constexpr uint8_t Q_IDENT_SCHEDULE_COUNT = 4;
static constexpr uint8_t Q_IDENT_SIDE_COUNT = 2;
static constexpr uint8_t Q_IDENT_MAIN_OCCURRENCES_PER_SIDE = 5;
static constexpr uint8_t Q_IDENT_OCCURRENCES_PER_SIDE = 6;  // sixth is optional.
static constexpr float Q_IDENT_Q_LEVELS_MAS[4] = {0.000f, 0.454f, 0.786f, 0.900f};
// First index: Run 1--4. Second index: physical + side (0), physical - side (1).
// Every support event consumes exactly one occurrence, including Q=0 or invalid.
static constexpr float Q_IDENT_SCHEDULE_Q_MAS[Q_IDENT_SCHEDULE_COUNT][Q_IDENT_SIDE_COUNT]
                                                  [Q_IDENT_OCCURRENCES_PER_SIDE] = {
    {{0.000f, 0.786f, 0.900f, 0.454f, 0.000f, 0.900f},
     {0.900f, 0.454f, 0.000f, 0.786f, 0.900f, 0.000f}},
    {{0.454f, 0.900f, 0.786f, 0.000f, 0.454f, 0.786f},
     {0.786f, 0.000f, 0.454f, 0.900f, 0.786f, 0.454f}},
    {{0.786f, 0.000f, 0.454f, 0.900f, 0.786f, 0.454f},
     {0.000f, 0.900f, 0.786f, 0.454f, 0.000f, 0.900f}},
    {{0.900f, 0.454f, 0.000f, 0.786f, 0.900f, 0.000f},
     {0.454f, 0.786f, 0.900f, 0.000f, 0.454f, 0.786f}},
};
static constexpr uint16_t Q_IDENT_MAX_EVENTS = 128;
static constexpr uint8_t Q_IDENT_INVALID_NONE = 0;
static constexpr uint8_t Q_IDENT_INVALID_ARM_WAITING = 1;
static constexpr uint8_t Q_IDENT_INVALID_ARMED_EVENT_NO_OUTPUT = 2;
static constexpr uint8_t Q_IDENT_INVALID_RATE_BELOW_SUPPORT = 3;
static constexpr uint8_t Q_IDENT_INVALID_RATE_ABOVE_SUPPORT = 4;
static constexpr uint8_t Q_IDENT_INVALID_SCHEDULE_EXHAUSTED = 5;
static constexpr uint8_t Q_IDENT_INVALID_Q_ZERO_NO_PULSE = 6;
static constexpr uint8_t Q_IDENT_INVALID_ROLLER_NOT_READY = 7;
static constexpr uint8_t Q_IDENT_INVALID_BATTERY_GUARD = 8;
static constexpr uint8_t Q_IDENT_INVALID_PULSE_WIDTH_GUARD = 9;
static constexpr uint8_t Q_IDENT_INVALID_SOLVER_NONFINITE = 10;
static constexpr uint8_t Q_IDENT_INVALID_CURRENT_WRITE_FAILED = 11;
static constexpr uint8_t Q_IDENT_INVALID_ESTOP_OR_STATE = 12;

// Integrated walking-theory prototype. This is the sole actual-output path.
// Q1 supplies passive next-peak and side-specific augmentation; the STEP
// geometry below supplies U(theta). All values are frozen for V0.
static constexpr char ENERGY_CONTROL_V0_MEASUREMENT_MODE[] = "energy_control_v0_p1_step_q1";
static constexpr uint32_t ENERGY_CONTROL_V0_DURATION_MS = 60000UL;
static constexpr float ENERGY_CONTROL_V0_TARGET_PEAK_DEG = 2.0f;
static constexpr float ENERGY_CONTROL_V0_Q_MAX_MAS = 0.900f;
static constexpr float ENERGY_CONTROL_V0_Q_SEARCH_STEP_MAS = 0.001f;
static constexpr float ENERGY_CONTROL_V0_RADIUS_M = 0.150f;
static constexpr float ENERGY_CONTROL_V0_INNER_EDGE_X_M = 0.005f;
static constexpr float ENERGY_CONTROL_V0_OUTER_EDGE_X_M = 0.045f;
static constexpr float ENERGY_CONTROL_V0_CG_HEIGHT_M = 0.120f;
static constexpr float ENERGY_CONTROL_V0_MASS_KG = 0.1997f;
static constexpr float ENERGY_CONTROL_V0_GRAVITY_M_S2 = 9.80665f;
static constexpr uint16_t ENERGY_CONTROL_V0_MAX_EVENTS = 128;
// V0.2 retains V0.1's output-only state machine around the unchanged Q1 detector.
// The continuous detector coordinate is used for the excursion measurement;
// the accelerometer-derived physical angle remains static-fixture-only.
static constexpr float ENERGY_CONTROL_V0_REARM_EXCURSION_DEG = 0.7f;
static constexpr uint8_t ENERGY_CONTROL_V0_INVALID_NONE = 0;
static constexpr uint8_t ENERGY_CONTROL_V0_VALID_PASSIVE_NO_OUTPUT = 1;
static constexpr uint8_t ENERGY_CONTROL_V0_INVALID_NONFINITE_STATE = 2;
static constexpr uint8_t ENERGY_CONTROL_V0_INVALID_POTENTIAL_DOMAIN = 3;
static constexpr uint8_t ENERGY_CONTROL_V0_INVALID_ROLLER_NOT_READY = 4;
static constexpr uint8_t ENERGY_CONTROL_V0_INVALID_BATTERY_GUARD = 5;
static constexpr uint8_t ENERGY_CONTROL_V0_INVALID_PULSE_WIDTH_GUARD = 6;
static constexpr uint8_t ENERGY_CONTROL_V0_INVALID_CURRENT_WRITE_FAILED = 7;
static constexpr uint8_t ENERGY_CONTROL_V0_INVALID_ESTOP_OR_STATE = 8;
static constexpr uint8_t ENERGY_CONTROL_V0_INVALID_WAIT_INITIAL_EXCURSION = 9;
static constexpr uint8_t ENERGY_CONTROL_V0_INVALID_WAIT_REARM_EXCURSION = 10;
static constexpr uint8_t ENERGY_CONTROL_V0_INVALID_NONALTERNATING_SIDE = 11;
static constexpr uint8_t ENERGY_CONTROL_V0_INVALID_EVENT_LOG_OVERFLOW = 12;

// Autonomous Energy Control V7: one startup-only strong kick is followed by
// direct normal P1/Q1 energy control on accepted physical half-cycles. The V5
// event policy separates raw detector candidates from accepted physical events
// so pulse transients cannot self-trigger the next control cycle.
static constexpr char ENERGY_CONTROL_AUTONOMOUS_MEASUREMENT_MODE[] =
    "energy_control_autonomous_v7_side_response_correction_rwlog30s";
static constexpr uint32_t ENERGY_CONTROL_AUTONOMOUS_DURATION_MS = 30000UL;
static constexpr float ENERGY_CONTROL_AUTONOMOUS_DEFAULT_TARGET_PEAK_DEG = 8.0f;
static constexpr float ENERGY_CONTROL_AUTONOMOUS_TARGET_MIN_DEG = 8.0f;
static constexpr float ENERGY_CONTROL_AUTONOMOUS_TARGET_MAX_DEG = 12.0f;
static constexpr float ENERGY_CONTROL_AUTONOMOUS_TARGET_CHOICES_DEG[3] = {8.0f, 10.0f, 12.0f};
// This strong kick is startup-only and independent of normal energy control.
static constexpr int16_t ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA = 300;
static constexpr uint16_t ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS = 100;
static constexpr int8_t ENERGY_CONTROL_AUTONOMOUS_START_KICK_DIRECTION = -1;
// Normal autonomous output deliberately does not use the Q_IDENT 5--25 ms
// experiment limits.  Width zero is a documented no-output decision.
static constexpr int16_t ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA = 300;
static constexpr uint16_t ENERGY_CONTROL_AUTONOMOUS_MIN_PULSE_MS = 0;
static constexpr uint16_t ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS = 100;

// V7: bounded side-response residual correction fitted in firmware peak coordinates
// from three V6 target-8 closed-loop runs. It augments, never replaces, P1/Q1.
static constexpr bool ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_CORRECTION_ENABLED = true;
static constexpr char ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_CORRECTION_SOURCE[] =
    "V6_three_run_closed_loop_20260904";
static constexpr float ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_BLEND_LAMBDA = 0.5f;
static constexpr float ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_MAX_ABS_DEG = 1.5f;
static constexpr float ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_C_PLUS_DEG = 0.4986094379f;
static constexpr float ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_G_PLUS_DEG_PER_MAS = 0.6583242379f;
static constexpr float ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_C_MINUS_DEG = -1.239921640f;
static constexpr float ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_G_MINUS_DEG_PER_MAS = 0.6580326445f;

// The start-referenced scaled +gy integral supplies the absolute energy peak
// amplitude.  It is not used to detect central passage.
static constexpr float ENERGY_CONTROL_AUTONOMOUS_GYRO_TO_VIDEO_PEAK_SCALE = 0.908911f;
static constexpr uint32_t ENERGY_CONTROL_AUTONOMOUS_GYRO_INTEGRATION_MAX_DT_US = 25000UL;
// A continuous adopted-angle extremum plus this many rate-confirming returning
// samples defines one physical peak.  No rate or amplitude acceptance minimum
// is introduced.
static constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_PEAK_CONFIRM_SAMPLES = 3;
// Physical-event timing debounce, not a Q/rate/model support gate. It rejects
// the 93--171 ms pulse-transient pseudo events observed in V4 while preserving
// the measured 0.317--0.490 s passive half-cycle range.
static constexpr uint32_t ENERGY_CONTROL_AUTONOMOUS_MIN_HALF_CYCLE_MS = 250UL;
static constexpr uint32_t ENERGY_CONTROL_AUTONOMOUS_MIN_ZERO_TO_PEAK_MS = 125UL;
// P1 free-decay model: F_s(A)=U_P1^-1(alpha*U_P1(A)-Ec), A>=0 deg.
// It is amplitude-side-independent (F_+=F_-); side is used for signed peak
// identity and the existing side-specific Q augmentation gain.
static constexpr char ENERGY_CONTROL_AUTONOMOUS_FREE_MODEL_REVISION[] =
    "P1_STEP_energy_alpha_0p870671664_Ec_0_20260828";
static constexpr float ENERGY_CONTROL_AUTONOMOUS_P1_FREE_DECAY_ALPHA = 0.8706716644111074f;
static constexpr float ENERGY_CONTROL_AUTONOMOUS_P1_FREE_DECAY_EC_J = 0.0f;
static constexpr float ENERGY_CONTROL_AUTONOMOUS_INTEGRAL_KI_MAS_PER_DEG = 0.10f;
static constexpr uint16_t ENERGY_CONTROL_AUTONOMOUS_MAX_EVENTS = 256;
static constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_NONE = 0;
static constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_VALID_NO_OUTPUT = 1;
static constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_STRONG_START_KICK_EXECUTED = 2;

static constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_EVENT_LOG_OVERFLOW = 6;
static constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_NONFINITE_STATE = 7;
static constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_POTENTIAL_DOMAIN = 8;
static constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_ROLLER_NOT_READY = 9;
static constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_BATTERY_GUARD = 10;
static constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_PULSE_WIDTH_GUARD = 11;
static constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_CURRENT_WRITE_FAILED = 12;
static constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_Q_BELOW_MIN_PULSE = 13;
static constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_ESTOP_OR_STATE = 14;

// E2 remains an offline diagnostic reference only. It is not called by the
// passive Q1 runtime, not used for Q1 validity, and has no Q gain constants.
static constexpr char E2_SHADOW_MODEL_VERSION[] = "E2_offline_diagnostic_only_20260829";
static constexpr float E2_SHADOW_E2_SLOPE = 0.9282969f;
static constexpr float E2_SHADOW_E2_OFFSET_DEG = -0.1267170f;// E2 was identified on the half-range H between adjacent opposite-side
// video peaks. The on-device input must use that same definition:
// H_prev = abs(integral gyro_rate dt between adjacent turns) / 2.
static constexpr float E2_SHADOW_H_PREV_MIN_DEG = 1.0f;
static constexpr float E2_SHADOW_H_PREV_MAX_DEG = 14.319f;
// E2 diagnostic implementation constants; no passive Q1 code calls it.
static constexpr float E2_SHADOW_RELEASE_ARM_RATE_THRESHOLD_DPS = 3.0f;
static constexpr float E2_SHADOW_PEAK_RATE_THRESHOLD_DPS = 0.75f;
static constexpr uint8_t E2_SHADOW_PEAK_CONFIRM_SAMPLES = 3;
static constexpr uint32_t E2_SHADOW_GYRO_INTEGRATION_MAX_DT_US = 25000UL;
static constexpr uint16_t E2_SHADOW_MAX_PEAK_EVENTS = 128;
static constexpr uint8_t E2_SHADOW_INVALID_NONE = 0;
static constexpr uint8_t E2_SHADOW_INVALID_H_OUT_OF_CALIBRATION_RANGE = 1;
static constexpr uint8_t E2_SHADOW_INVALID_BRAKING_NOT_IDENTIFIED = 2;
static constexpr uint8_t E2_SHADOW_INVALID_SIDE_MISMATCH = 3;
static constexpr uint8_t E2_SHADOW_INVALID_Q_OUT_OF_CALIBRATION_RANGE = 4;
static constexpr uint8_t E2_SHADOW_INVALID_NONFINITE_STATE = 5;
static constexpr uint8_t E2_SHADOW_INVALID_ABSOLUTE_PEAK_ESTIMATOR_UNVALIDATED = 6;
static constexpr uint8_t E2_SHADOW_INVALID_NONALTERNATING_TURN = 7;
static constexpr uint8_t E2_SHADOW_INVALID_INSUFFICIENT_TURNS = 8;
static constexpr char V62_BASE_COMMIT[] = "748347f3985e053684d979955dd70c7ee8daadff";
static constexpr char RESOLVED_M5UNIFIED_VERSION[] = "0.2.18";
static constexpr char RESOLVED_M5GFX_VERSION[] = "0.2.28";
static constexpr char RESOLVED_ADAFRUIT_AHRS_VERSION[] = "2.4.0";

static constexpr uint32_t BETA_SWEEP_TRIAL_DURATION_MS = 30000UL;
static constexpr uint32_t BETA_SWEEP_INTER_TRIAL_REST_MS = 5000UL;

// Zero-cross input comparison. The Web UI selects the current and pulse width
// before a run; the selected condition is used unchanged for the start kick
// and every later accepted zero-cross pulse.
static constexpr uint32_t ZERO_CROSS_TEST_DURATION_MS = 30000UL;
// V59 needs enough time for 12 accepted state-matched fixed-Q events. This
// extended duration is used only by the V59 Q-control measurement mode.
static constexpr uint32_t ZERO_CROSS_V59_TEST_DURATION_MS = 90000UL;
static constexpr int16_t ZERO_CROSS_CURRENT_MIN_MA = 100;
static constexpr int16_t ZERO_CROSS_CURRENT_MAX_MA = 300;
static constexpr int16_t ZERO_CROSS_CURRENT_100_MA = 100;
static constexpr int16_t ZERO_CROSS_CURRENT_150_MA = 150;
static constexpr int16_t ZERO_CROSS_CURRENT_200_MA = 200;
static constexpr int16_t ZERO_CROSS_CURRENT_250_MA = 250;
static constexpr int16_t ZERO_CROSS_CURRENT_300_MA = 300;
static constexpr int16_t ZERO_CROSS_OPERATING_CURRENT_MA = ZERO_CROSS_CURRENT_300_MA;
static constexpr int16_t ZERO_CROSS_DEFAULT_FIXED_CURRENT_MA = ZERO_CROSS_OPERATING_CURRENT_MA;
static constexpr uint16_t ZERO_CROSS_PULSE_5_MS = 5;
static constexpr uint16_t ZERO_CROSS_PULSE_30_MS = 30;
static constexpr uint16_t ZERO_CROSS_PULSE_40_MS = 40;
static constexpr uint16_t ZERO_CROSS_PULSE_50_MS = 50;
static constexpr uint16_t ZERO_CROSS_PULSE_55_MS = 55;
static constexpr uint16_t ZERO_CROSS_PULSE_60_MS = 60;
static constexpr uint16_t ZERO_CROSS_PULSE_65_MS = 65;
static constexpr uint16_t ZERO_CROSS_PULSE_70_MS = 70;
static constexpr uint16_t ZERO_CROSS_PULSE_80_MS = 80;
static constexpr uint16_t ZERO_CROSS_PULSE_90_MS = 90;
static constexpr uint16_t ZERO_CROSS_PULSE_100_MS = 100;
// Manual zero-cross condition range. The UI accepts integer values in this
// inclusive range; high energy combinations are boundary observations.
static constexpr uint16_t ZERO_CROSS_TIME_SWEEP_MIN_PULSE_MS = ZERO_CROSS_PULSE_5_MS;
static constexpr uint16_t ZERO_CROSS_TIME_SWEEP_MAX_PULSE_MS = ZERO_CROSS_PULSE_100_MS;
static constexpr uint16_t ZERO_CROSS_DEFAULT_FIXED_PULSE_MS = ZERO_CROSS_PULSE_5_MS;
static constexpr int8_t ZERO_CROSS_BOOTSTRAP_DIRECTION = 1;
static constexpr uint16_t ZERO_CROSS_START_REFRACTORY_MS = 250;
static constexpr float ZERO_CROSS_REARM_ANGLE_DEG = 0.08f;
static constexpr float ZERO_CROSS_MIN_RATE_DPS = 0.25f;
static constexpr uint16_t ZERO_CROSS_MIN_PULSE_INTERVAL_MS = 200;

// Identification capture targets are signed-current-integral magnitudes in mA*s.
static constexpr uint8_t ZERO_CROSS_IDENTIFICATION_TARGET_COUNT = 4;
static constexpr float ZERO_CROSS_IDENTIFICATION_TARGET_Q_MAS[ZERO_CROSS_IDENTIFICATION_TARGET_COUNT] = {
    0.000f, 0.454f, 0.786f, 1.197f,
};
static constexpr float ZERO_CROSS_IDENTIFICATION_BOOTSTRAP_Q_MAS = 1.197f;
static constexpr uint16_t ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS = 5;
static constexpr uint16_t ZERO_CROSS_IDENTIFICATION_MAX_PULSE_MS = 25;
static constexpr uint8_t ZERO_CROSS_IDENTIFICATION_PEAK_CONFIRM_SAMPLES = 3;

// Empirical next-peak model identified from three independently synchronized
// 300 mA zero-cross Q runs. Q is the discrete command target in mA*s.
static constexpr float ZERO_CROSS_Q_MODEL_INTERCEPT_DEG = 0.0226223f;
static constexpr float ZERO_CROSS_Q_MODEL_RATE_GAIN_DEG_PER_DPS = 0.0881459f;
static constexpr float ZERO_CROSS_Q_MODEL_ANGLE_GAIN_DEG_PER_DEG = -0.0737659f;
static constexpr float ZERO_CROSS_Q_MODEL_GAIN_DEG_PER_MAS = 0.2697985f;
static constexpr float ZERO_CROSS_Q_MODEL_DIRECTION_OFFSET_DEG = -0.1244166f;
static constexpr float ZERO_CROSS_Q_MODEL_DIRECTION_GAIN_DEG_PER_MAS = -0.0221271f;
// Log-only validation candidate. Never use this offset for Q selection or pulse control.
static constexpr float ZERO_CROSS_Q_MODEL_SHADOW_OFFSET_DEG = 0.15f;
// v52 is a strictly log-only predictor for the video-derived half-range
// increment. These coefficients use only state available before a Q pulse:
// Delta-H_video = b0 + bH*Hprev + bw*abs(rate) + bd*next_peak_side + bQ*Q.
// Do not add the old event-center C here: it contains the post-pulse peak and
// would leak future information into an online prediction.
static constexpr bool ZERO_CROSS_V52_SHADOW_ENABLED = true;
static constexpr const char* ZERO_CROSS_V52_SHADOW_MODEL_VERSION = "v52_online_54pt_20260821";
static constexpr float ZERO_CROSS_V52_SHADOW_INTERCEPT_DEG = -1.1393659f;
static constexpr float ZERO_CROSS_V52_SHADOW_HPREV_GAIN = -0.4714266f;
static constexpr float ZERO_CROSS_V52_SHADOW_ABS_RATE_GAIN_DEG_PER_DPS = 0.0823895f;
static constexpr float ZERO_CROSS_V52_SHADOW_NEXT_PEAK_SIDE_OFFSET_DEG = -0.0961605f;
static constexpr float ZERO_CROSS_V52_SHADOW_Q_GAIN_DEG_PER_MAS = 0.1919561f;
// The model was identified only from the actual 0.503--1.582 mA*s Q values.
static constexpr float ZERO_CROSS_V52_SHADOW_Q_SUPPORT_MIN_MAS = 0.50f;
static constexpr float ZERO_CROSS_V52_SHADOW_Q_SUPPORT_MAX_MAS = 1.60f;
// v53 keeps the v52 forward predictor fixed and adds an inverse-shadow audit.
// It is metadata-only: no value in this block may select Q, alter width, or
// command the motor. H_ref is expressed in the video half-range coordinate.
static constexpr bool ZERO_CROSS_V53_INVERSE_SHADOW_ENABLED = true;
static constexpr const char* ZERO_CROSS_V53_INVERSE_SHADOW_MODEL_VERSION =
    "v53_inverse_shadow_v51_video_hfree_20260821";
static constexpr bool ZERO_CROSS_V53_H_REF_CONFIGURED = true;
// V59 records the inverse-Q diagnostic at this reference only. It is never a
// pulse or motor-command source.
static constexpr float ZERO_CROSS_V53_H_REF_VIDEO_DEG = 6.10f;
// The fixed v51-only external mapping, applied to v52/v53 held-out states:
// H_free_video = -0.1960 + 0.94286 * H_free_imu.
static constexpr float ZERO_CROSS_V53_H_FREE_VIDEO_INTERCEPT_DEG = -0.1960f;
static constexpr float ZERO_CROSS_V53_H_FREE_VIDEO_FROM_IMU_GAIN = 0.94286f;
// These are the v51 forward-model pre-pulse state limits. They are separate
// from the per-run free-decay support used for H_free_imu prediction.
static constexpr float ZERO_CROSS_V53_STATE_HPREV_MIN_DEG = 5.98f;
static constexpr float ZERO_CROSS_V53_STATE_HPREV_MAX_DEG = 8.67f;
static constexpr float ZERO_CROSS_V53_STATE_ABS_RATE_MIN_DPS = 51.97f;
static constexpr float ZERO_CROSS_V53_STATE_ABS_RATE_MAX_DPS = 73.15f;
static constexpr uint8_t ZERO_CROSS_V53_Q_CANDIDATE_COUNT = 4;
static constexpr float ZERO_CROSS_V53_Q_CANDIDATE_MAS[ZERO_CROSS_V53_Q_CANDIDATE_COUNT] = {
    0.0f, 0.5f, 1.0f, 1.5f,
};
// v53 evaluates inverse-Q against the current highest discrete candidate, not
// against the wider 1.60 mA*s forward-model audit boundary.
static constexpr float ZERO_CROSS_V53_Q_REQUEST_MAX_MAS = 1.5f;
// Region describes Q_req_raw itself, even outside the state support. Q=0 is
// an explicitly flagged diagnostic extrapolation; it is never a model-valid
// candidate. `shadow_q_req_valid` additionally requires the state support.
static constexpr uint8_t ZERO_CROSS_V53_Q_REQ_REGION_UNAVAILABLE = 0;
static constexpr uint8_t ZERO_CROSS_V53_Q_REQ_REGION_BELOW_ZERO = 1;
static constexpr uint8_t ZERO_CROSS_V53_Q_REQ_REGION_WITHIN_ZERO_TO_QMAX = 2;
static constexpr uint8_t ZERO_CROSS_V53_Q_REQ_REGION_ABOVE_QMAX = 3;
// Reason adds why the raw inverse cannot be a future control candidate.
static constexpr uint8_t ZERO_CROSS_V53_Q_REQ_WITHIN_RANGE = 0;
static constexpr uint8_t ZERO_CROSS_V53_Q_REQ_REFERENCE_UNCONFIGURED = 1;
static constexpr uint8_t ZERO_CROSS_V53_Q_REQ_STATE_INVALID = 2;
static constexpr uint8_t ZERO_CROSS_V53_Q_REQ_BELOW_ZERO = 3;
static constexpr uint8_t ZERO_CROSS_V53_Q_REQ_ABOVE_QMAX = 4;
static constexpr uint8_t ZERO_CROSS_V53_Q_REQ_OUTSIDE_MODEL_SUPPORT = 5;
static constexpr uint8_t ZERO_CROSS_V53_Q_REQ_STATE_OUT_OF_SUPPORT = 6;
static constexpr uint8_t ZERO_CROSS_V53_Q_REQ_H_FREE_INVALID = 7;
// v54 adds a reachability audit on top of the unchanged v53 inverse shadow.
// It records the recommended policy only. No value in this block may select
// Q, alter pulse width, alter zero-cross timing, or command the motor.
static constexpr bool ZERO_CROSS_V54_REACHABILITY_AUDIT_ENABLED = true;
static constexpr const char* ZERO_CROSS_V54_REACHABILITY_AUDIT_VERSION =
    "v54_reachability_log_only_20260821";
// Keep the current physical candidate range distinct from the wider range in
// which the forward Delta-H model was measured.
static constexpr float ZERO_CROSS_V54_CONTROL_CANDIDATE_Q_MIN_MAS = 0.50f;
static constexpr float ZERO_CROSS_V54_CONTROL_CANDIDATE_Q_MAX_MAS = 1.50f;
// Why the target reachability decision has the reported value.
static constexpr uint8_t ZERO_CROSS_V54_REACHABILITY_UNAVAILABLE = 0;
static constexpr uint8_t ZERO_CROSS_V54_REACHABILITY_REFERENCE_UNCONFIGURED = 1;
static constexpr uint8_t ZERO_CROSS_V54_REACHABILITY_STATE_INVALID = 2;
static constexpr uint8_t ZERO_CROSS_V54_REACHABILITY_STATE_OUT_OF_SUPPORT = 3;
static constexpr uint8_t ZERO_CROSS_V54_REACHABILITY_H_FREE_INVALID = 4;
static constexpr uint8_t ZERO_CROSS_V54_REACHABILITY_CANDIDATES_INVALID = 5;
static constexpr uint8_t ZERO_CROSS_V54_REACHABILITY_TARGET_BELOW_RANGE = 6;
static constexpr uint8_t ZERO_CROSS_V54_REACHABILITY_TARGET_WITHIN_RANGE = 7;
static constexpr uint8_t ZERO_CROSS_V54_REACHABILITY_TARGET_ABOVE_RANGE = 8;
// The policy is a shadow recommendation only: 0=unavailable, 1=no pulse,
// 2=continuous inverse-Q inside range, 3=saturate at the current Q maximum.
static constexpr uint8_t ZERO_CROSS_V54_SHADOW_ACTION_UNAVAILABLE = 0;
static constexpr uint8_t ZERO_CROSS_V54_SHADOW_ACTION_NO_PULSE = 1;
static constexpr uint8_t ZERO_CROSS_V54_SHADOW_ACTION_INVERSE_Q = 2;
static constexpr uint8_t ZERO_CROSS_V54_SHADOW_ACTION_SATURATE_QMAX = 3;
// Q support is reported independently of the Hprev/rate state support.
static constexpr uint8_t ZERO_CROSS_V54_Q_SUPPORT_UNAVAILABLE = 0;
static constexpr uint8_t ZERO_CROSS_V54_Q_SUPPORT_NEGATIVE = 1;
static constexpr uint8_t ZERO_CROSS_V54_Q_SUPPORT_BELOW_FORWARD_MODEL = 2;
static constexpr uint8_t ZERO_CROSS_V54_Q_SUPPORT_WITHIN_CURRENT_CANDIDATES = 3;
static constexpr uint8_t ZERO_CROSS_V54_Q_SUPPORT_ABOVE_CANDIDATES_WITHIN_FORWARD_MODEL = 4;
static constexpr uint8_t ZERO_CROSS_V54_Q_SUPPORT_ABOVE_FORWARD_MODEL = 5;
// v57 preserves the v52--v54 coefficients and physical pulse path, but makes
// the inverse-shadow decision auditable one condition at a time. It remains
// strictly log-only: none of these values can select Q, alter pulse width, or
// command the motor.
static constexpr bool ZERO_CROSS_V57_SUPPORT_AWARE_INVERSE_SHADOW_ENABLED = true;
static constexpr const char* ZERO_CROSS_V57_FORWARD_MODEL_VERSION = "v51_forward_20260821";
static constexpr const char* ZERO_CROSS_V57_INVERSE_SHADOW_VERSION =
    "v57_support_aware_inverse_shadow_20260822";
// This wider interval is descriptive only: it is the observed state envelope
// of the 36 v57 events. The narrower v53 limits above remain the only online
// model-support gate.
static constexpr float ZERO_CROSS_V57_OBSERVED_HPREV_MIN_DEG = 5.69f;
static constexpr float ZERO_CROSS_V57_OBSERVED_HPREV_MAX_DEG = 9.01f;
static constexpr float ZERO_CROSS_V57_OBSERVED_ABS_RATE_MIN_DPS = 55.01f;
static constexpr float ZERO_CROSS_V57_OBSERVED_ABS_RATE_MAX_DPS = 74.38f;
static constexpr uint8_t ZERO_CROSS_V57_Q_REQ_REASON_UNAVAILABLE = 0;
static constexpr uint8_t ZERO_CROSS_V57_Q_REQ_REASON_VALID = 1;
static constexpr uint8_t ZERO_CROSS_V57_Q_REQ_REASON_REFERENCE_UNCONFIGURED = 2;
static constexpr uint8_t ZERO_CROSS_V57_Q_REQ_REASON_STATE_INVALID = 3;
static constexpr uint8_t ZERO_CROSS_V57_Q_REQ_REASON_STATE_OUT_OF_SUPPORT = 4;
static constexpr uint8_t ZERO_CROSS_V57_Q_REQ_REASON_H_FREE_INVALID = 5;
static constexpr uint8_t ZERO_CROSS_V57_Q_REQ_REASON_Q_BELOW_CANDIDATE_RANGE = 6;
static constexpr uint8_t ZERO_CROSS_V57_Q_REQ_REASON_Q_ABOVE_CANDIDATE_RANGE = 7;
static constexpr uint8_t ZERO_CROSS_V57_Q_REQ_REASON_REFERENCE_UNREACHABLE = 8;
// v58 keeps all v57 diagnostics and physical behavior unchanged. It marks the
// repeatability-study subset only; the gate is never a Q, pulse, or motor gate.
static constexpr bool ZERO_CROSS_V58_REPEATABILITY_STATE_GATE_ENABLED = true;
static constexpr const char* ZERO_CROSS_V58_REPEATABILITY_GATE_VERSION =
    "v58_inverse_q_repeatability_state_gate_20260822";
static constexpr float ZERO_CROSS_V58_GATE_HPREV_MIN_DEG = 6.40f;
static constexpr float ZERO_CROSS_V58_GATE_HPREV_MAX_DEG = 6.80f;
static constexpr float ZERO_CROSS_V58_GATE_ABS_RATE_MIN_DPS = 57.50f;
static constexpr float ZERO_CROSS_V58_GATE_ABS_RATE_MAX_DPS = 60.50f;
static constexpr int8_t ZERO_CROSS_V58_GATE_NEXT_PEAK_SIDE = 1;
static constexpr uint8_t ZERO_CROSS_V58_GATE_REASON_UNAVAILABLE = 0;
static constexpr uint8_t ZERO_CROSS_V58_GATE_REASON_PASSED = 1;
static constexpr uint8_t ZERO_CROSS_V58_GATE_REASON_DISABLED = 2;
static constexpr uint8_t ZERO_CROSS_V58_GATE_REASON_STATE_INVALID = 3;
static constexpr uint8_t ZERO_CROSS_V58_GATE_REASON_HPREV_BELOW = 4;
static constexpr uint8_t ZERO_CROSS_V58_GATE_REASON_HPREV_ABOVE = 5;
static constexpr uint8_t ZERO_CROSS_V58_GATE_REASON_RATE_BELOW = 6;
static constexpr uint8_t ZERO_CROSS_V58_GATE_REASON_RATE_ABOVE = 7;
static constexpr uint8_t ZERO_CROSS_V58_GATE_REASON_DIRECTION_MISMATCH = 8;
// V59 is a physical fixed-Q response repeatability measurement. It waits for
// a measured pre-pulse state and then executes only the already planned Q;
// neither Hfree_pred nor Qreq_pred can select Q, width, or motor direction.
static constexpr bool ZERO_CROSS_V59_STATE_WAIT_FIXED_Q_ENABLED = true;
static constexpr const char* ZERO_CROSS_V59_STATE_WAIT_FIXED_Q_VERSION =
    "v59_state_wait_fixed_q_20260822";
// The first Cprev interval comes from the usable low-H V58 neighbourhood
// (0.72--0.98 deg) with a small measurement margin. It will be re-estimated
// from the accepted V59 response set, not treated as a universal machine bias.
static constexpr float ZERO_CROSS_V59_GATE_HPREV_MIN_DEG = 6.50f;
static constexpr float ZERO_CROSS_V59_GATE_HPREV_MAX_DEG = 6.75f;
static constexpr float ZERO_CROSS_V59_GATE_CPREV_MIN_DEG = 0.70f;
static constexpr float ZERO_CROSS_V59_GATE_CPREV_MAX_DEG = 1.30f;
static constexpr float ZERO_CROSS_V59_GATE_ABS_RATE_MIN_DPS = 58.00f;
static constexpr float ZERO_CROSS_V59_GATE_ABS_RATE_MAX_DPS = 60.50f;
static constexpr int8_t ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE = 1;
static constexpr uint8_t ZERO_CROSS_V59_MAX_CONSECUTIVE_GATE_SKIPS = 36;
static constexpr uint8_t ZERO_CROSS_V59_GATE_REASON_UNAVAILABLE = 0;
static constexpr uint8_t ZERO_CROSS_V59_GATE_REASON_PASSED = 1;
static constexpr uint8_t ZERO_CROSS_V59_GATE_REASON_DISABLED = 2;
static constexpr uint8_t ZERO_CROSS_V59_GATE_REASON_STATE_INVALID = 3;
static constexpr uint8_t ZERO_CROSS_V59_GATE_REASON_DYNAMIC_H_OUT_OF_SUPPORT = 4;
static constexpr uint8_t ZERO_CROSS_V59_GATE_REASON_HPREV_BELOW = 5;
static constexpr uint8_t ZERO_CROSS_V59_GATE_REASON_HPREV_ABOVE = 6;
static constexpr uint8_t ZERO_CROSS_V59_GATE_REASON_CPREV_BELOW = 7;
static constexpr uint8_t ZERO_CROSS_V59_GATE_REASON_CPREV_ABOVE = 8;
static constexpr uint8_t ZERO_CROSS_V59_GATE_REASON_RATE_BELOW = 9;
static constexpr uint8_t ZERO_CROSS_V59_GATE_REASON_RATE_ABOVE = 10;
static constexpr uint8_t ZERO_CROSS_V59_GATE_REASON_DIRECTION_MISMATCH = 11;
// This is a V61 control action rather than a rejection of the V59 fixed-Q gate.
static constexpr uint8_t ZERO_CROSS_V59_GATE_REASON_V61_STATE_FEEDBACK = 12;
// V61 retains the V59 fixed-Q values, order, and narrow admission gate. It
// changes only the rebuild transition: a controlled opposite-side peak is
// chosen so the following unforced pair predicts the unchanged H/C gate.
static constexpr const char* ZERO_CROSS_V60_REBUILD_PHASE_VERSION =
    "v61_state_feedback_rebuild_fixed_q_20260822";
// V62 keeps the fixed-Q plan and gate, but selects a rebuild target only from
// the session-specific H/C/rate intersection observed during free decay.
static constexpr const char* ZERO_CROSS_V62_STATE_FEASIBILITY_VERSION = "v62_joint_h_c_rate_feasibility_fixed_q_20260824";
static constexpr uint8_t ZERO_CROSS_V62_RATE_MODEL_MIN_SAMPLES = 4;
static constexpr float ZERO_CROSS_V62_RATE_MODEL_RIDGE = 0.01f;
static constexpr float ZERO_CROSS_V62_RATE_MODEL_MIN_R2 = 0.80f;
static constexpr float ZERO_CROSS_V62_TARGET_SCAN_STEP_DEG = 0.01f;
static constexpr uint8_t ZERO_CROSS_CAL_FAILURE_V62_NO_FEASIBLE_HC_TARGET = 11;
static constexpr uint8_t ZERO_CROSS_CAL_FAILURE_V62_NO_FEASIBLE_HC_RATE_TARGET = 12;

// H is updated for every consecutive peak pair, i.e. once per physical
// half-cycle. Two updates return to the same desired (+) arrival side.
static constexpr uint8_t ZERO_CROSS_V60_REBUILD_COOLDOWN_HALF_CYCLES = 2;
// The user has declared the experimental surroundings safe. V60 rebuilds may
// therefore request the full model-inverted Q and width without a Q cap or
// predicted-peak guard. This applies only before fixed-Q measurement begins.
static constexpr bool ZERO_CROSS_V60_REBUILD_UNRESTRICTED = true;

// v40 per-run calibration is an explicitly log-only shadow model. The
// established Q selector continues to use the fixed response model above.
static constexpr bool ZERO_CROSS_CALIBRATION_SHADOW_ENABLED = true;
// Calibration-only amplitude range, derived from synchronized video.  The
// mechanism begins an unrecovered outward fall at about 22 deg, so identify
// the model in the representative 14-16 deg range rather than at small sway.
static constexpr float ZERO_CROSS_CALIBRATION_INITIAL_MIN_DEG = 14.0f;
static constexpr float ZERO_CROSS_CALIBRATION_INITIAL_MAX_DEG = 16.0f;
// A small overshoot above the nominal range is accepted without another kick.
// At this boundary calibration stops and normal Q control takes over.
static constexpr float ZERO_CROSS_CALIBRATION_INITIAL_ABORT_DEG = 19.0f;
static constexpr uint8_t ZERO_CROSS_CALIBRATION_INITIAL_MAX_KICKS = 32;
// Initial build-up uses the response-model inverse, not the normal 5--25 ms
// identification selector. The Q cap is the measured 200 mA / 60 ms
// near-limit input (about 3.80 mA*s); at 300 mA this is approximately 48 ms.
static constexpr float ZERO_CROSS_CALIBRATION_INITIAL_TARGET_PEAK_DEG = 15.0f;
static constexpr float ZERO_CROSS_CALIBRATION_INITIAL_PREDICT_GUARD_DEG = 18.0f;
static constexpr float ZERO_CROSS_CALIBRATION_INITIAL_Q_MAX_MAS = 3.80f;
static constexpr uint16_t ZERO_CROSS_CALIBRATION_INITIAL_MAX_PULSE_MS = 60;
// The normal controller uses the same experimentally bounded energy envelope,
// but selects a continuous Q from the target-peak inverse rather than one of
// the former four discrete identification targets.
static constexpr float ZERO_CROSS_CONTROL_Q_MAX_MAS = ZERO_CROSS_CALIBRATION_INITIAL_Q_MAX_MAS;
static constexpr uint16_t ZERO_CROSS_CONTROL_MAX_PULSE_MS = ZERO_CROSS_CALIBRATION_INITIAL_MAX_PULSE_MS;
static constexpr float ZERO_CROSS_CALIBRATION_MIN_PEAK_DEG = 0.45f;
// At least six transitions per arrival side provide the free-decay fit. V60
// does not leave FREE_DECAY at that count until the unchanged H-gate center is
// inside the measured dynamic-H input support.
static constexpr uint8_t ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_PER_SIDE = 6;
// Additional free-decay observations are retained only while waiting for that
// support condition. This is a bounded safety/diagnostic limit, not a gate
// widening or a change to the fixed-Q plan.
static constexpr uint8_t ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_MAX_PER_SIDE = 8;
static constexpr float ZERO_CROSS_CALIBRATION_MIN_X_VARIANCE_DEG2 = 0.0004f;
// v46 uses a deterministic signed-Q calibration sequence. The physical
// arrival side is planned independently from motor direction so every
// requested side and Q level can be audited after a run.
static constexpr uint8_t ZERO_CROSS_CALIBRATION_Q_PROBE_Q_LEVEL_COUNT = 3;
static constexpr float ZERO_CROSS_CALIBRATION_Q_PROBE_Q_LEVEL_MAS[
    ZERO_CROSS_CALIBRATION_Q_PROBE_Q_LEVEL_COUNT] = {0.5f, 1.0f, 1.5f};
// V59 has 12 accepted fixed-Q responses: four 3-level blocks, each level
// occurring four times. The planned index advances only after an accepted
// state-gated Q pulse; waiting and rebuild pulses never consume a plan entry.
static constexpr uint8_t ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_COUNT = 12;
static constexpr uint8_t ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_COUNT = 3;
static constexpr uint8_t ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A = 0;
static constexpr uint8_t ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_B = 1;
static constexpr uint8_t ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_C = 2;
static constexpr uint8_t ZERO_CROSS_CALIBRATION_Q_PROBE_Q_LEVEL_INDEX_BY_SCHEDULE[
    ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_COUNT][ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_COUNT] = {
        // A blocks: 0.5/1.0/1.5, 1.0/1.5/0.5, 1.5/0.5/1.0, 1.5/1.0/0.5.
        {0, 1, 2, 1, 2, 0, 2, 0, 1, 2, 1, 0},
        // B and C retain four observations per Q level with distinct order.
        {0, 2, 1, 2, 1, 0, 1, 0, 2, 1, 2, 0},
        {1, 0, 2, 0, 2, 1, 2, 1, 0, 0, 1, 2},
};
// v47 widens only calibration Q probes so their 1.5 mA*s level is reachable.
static constexpr uint16_t ZERO_CROSS_CALIBRATION_Q_PROBE_MAX_PULSE_MS = 30;
static constexpr int8_t ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_DESIRED_SIDE[
    ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_COUNT] = {+1, +1, +1, +1, +1, +1,
                                                    +1, +1, +1, +1, +1, +1};
// Historical v50--v59 fixed rebuild threshold. V60 does not use this value to
// command a rebuild: it derives a per-run H target from measured free decay,
// then converts it to a desired-side peak using the V59 C-gate center. Retained
// solely for backward-compatible audit decoding of older RWLOGs.
static constexpr float ZERO_CROSS_CALIBRATION_Q_REBUILD_TARGET_PEAK_DEG = 10.0f;
// Zero disables the per-episode V60 rebuild-attempt limit. The user has
// explicitly requested unrestricted rebuilds in the current safe test setup;
// the measurement timeout remains the finite terminal condition.
static constexpr uint8_t ZERO_CROSS_CALIBRATION_Q_REBUILD_MAX_ATTEMPTS = 0;
// The first v46 validation run records all three Q levels on each arrival
// side. Positive-gain acceptance is retained only for the log-only summary;
// it never changes normal control.
static constexpr uint8_t ZERO_CROSS_CALIBRATION_Q_PROBE_SAMPLES_PER_SIDE = 12;
static constexpr float ZERO_CROSS_CALIBRATION_Q_PROBE_SUPPORT_MARGIN_DEG = 0.25f;
static constexpr float ZERO_CROSS_CALIBRATION_Q_PROBE_MIN_POSITIVE_GAIN_DEG_PER_MAS = 0.01f;
static constexpr uint8_t ZERO_CROSS_CALIBRATION_Q_PROBE_MAX_ATTEMPTS =
    ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_COUNT;
// A free-decay pair must be consecutive physical half-cycles. Do not bridge a
// missed turn with a later candidate.
static constexpr uint32_t ZERO_CROSS_CALIBRATION_MAX_CONSECUTIVE_PEAK_GAP_MS = 900;
static constexpr uint32_t ZERO_CROSS_CALIBRATION_TIMEOUT_MS = 85000;
static constexpr float ZERO_CROSS_CONTROL_TARGET_MIN_DEG = 0.0f;
static constexpr float ZERO_CROSS_CONTROL_TARGET_MAX_DEG = 10.0f;

static constexpr int16_t DEFAULT_INPUT_CURRENT_MA = 150;
static constexpr uint16_t DEFAULT_PULSE_WIDTH_MS = 70;
static constexpr uint16_t DEFAULT_INPUT_INTERVAL_MS = 1000;
static constexpr uint16_t MIN_PULSE_WIDTH_MS = 5;
static constexpr uint16_t MAX_PULSE_WIDTH_MS = 1000;
static constexpr uint16_t MIN_INPUT_INTERVAL_MS = 20;
static constexpr uint16_t MAX_INPUT_INTERVAL_MS = 10000;
static constexpr int16_t MAX_ABS_INPUT_CURRENT_MA = 1000;

static constexpr float MADGWICK_BETA_ONE = 1.0f;
// Fixed reference and this first zero-cross test both use the established
// soft50-linear profile that recovers to beta=0.050.
static constexpr float MADGWICK_BETA_NORMAL = 0.10f;
static constexpr float MADGWICK_BETA_DYNAMIC_MAX = 0.050f;
// Reference profile used to preserve the legacy time-to-maximum beta for each floor.
static constexpr uint16_t BETA_HOLD_AFTER_INPUT_MS = 73;
static constexpr uint16_t BETA_SOFT_START_MS = 50;
static constexpr float BETA_TIME_MATCH_REFERENCE_MAX = 0.100f;
static constexpr float BETA_TIME_MATCH_REFERENCE_RECOVERY_PER_MS = 0.000450f;
// Retained for the fixed, non-pulse diagnostic path and legacy log field only.
static constexpr float BETA_RECOVERY_TAU_S = 0.0f;
// Test-only adopted-series cap: after a gyro-confirmed turn, recover beta as
// a normalized increasing exponential of the measured return phase.
static constexpr bool BETA_PHASE_RETURN_TEST_ENABLED = false;
static constexpr float BETA_PHASE_EXPONENTIAL_K = 1.50f;
static constexpr float BETA_PHASE_OUTBOUND_RATE_MIN_DPS = 0.25f;
static constexpr float BETA_PHASE_TURN_CONFIRM_RATE_DPS = 0.25f;
static constexpr float BETA_PHASE_MIN_PEAK_ANGLE_DEG = 0.25f;

static constexpr float MODEL_VBAT_REFERENCE_V = 7.50f;
static constexpr float MODEL_VBAT_MIN_V = 6.18f;
static constexpr float MODEL_VBAT_MAX_V = 8.02f;
static constexpr float MODEL_I_SAT_AT_REFERENCE_MA = 329.547119f;
static constexpr float MODEL_I_SAT_SLOPE_MA_PER_V = 46.253815f;
static constexpr float MODEL_I_SAT_EXPONENT = 3.86f;
static constexpr float MODEL_TAU_RISE_MIN_MS = 31.4f;
static constexpr float MODEL_TAU_RISE_MAX_MS = 72.8f;
static constexpr float MODEL_TAU_RISE_U_MA = 372.0f;
static constexpr float MODEL_TAU_RISE_EXPONENT = 4.35f;
static constexpr float BETA_MIN_AT_ZERO_CURRENT = 0.005f;
static constexpr float BETA_MIN_AT_REFERENCE_CURRENT = 0.010f;
static constexpr float BETA_MIN_REFERENCE_CURRENT_MA = 300.0f;

// One fixed reference plus three dynamic hold-time candidates. All dynamic
// series use the same current/Vbat-dependent floor and beta ceiling (0.025),
// so this capture isolates how long the post-input disturbance protection is
// held before the existing 50 ms soft start and time-matched ramp begin.
static constexpr uint8_t DYNAMIC_BETA_COUNT = 6;
static constexpr uint8_t FILTER_FIXED_B100_INDEX = 0;
static constexpr uint8_t FILTER_FIXED_B000_INDEX = 1;
static constexpr uint8_t FILTER_DYNAMIC_HOLD_073_INDEX = 2;
static constexpr uint8_t FILTER_DYNAMIC_HOLD_120_INDEX = 3;
static constexpr uint8_t FILTER_DYNAMIC_HOLD_170_INDEX = 4;
static constexpr uint8_t FILTER_DYNAMIC_TURN_FAST_INDEX = 5;
// Preserve the established 73 ms profile as the timing / zero-cross detector
// reference. The other two filters are observational comparison series.
static constexpr uint8_t FILTER_ADOPTED_INDEX = FILTER_DYNAMIC_HOLD_073_INDEX;
static constexpr float DYNAMIC_BETA_CEILINGS[DYNAMIC_BETA_COUNT] = {
    MADGWICK_BETA_NORMAL, 0.000f, 0.025f, 0.025f, 0.025f, 0.025f,
};
static constexpr uint16_t DYNAMIC_BETA_HOLD_AFTER_INPUT_MS[DYNAMIC_BETA_COUNT] = { 0, 0, 73, 120, 170, 0, };
static constexpr const char* DYNAMIC_BETA_LABELS[DYNAMIC_BETA_COUNT] = {
    "fixed_b100", "fixed_b000", "dynamic_vbat_hold073_soft50_timematch_max025",
    "dynamic_vbat_hold120_soft50_timematch_max025", "dynamic_vbat_hold170_soft50_timematch_max025",
    "dynamic_vbat_turn_confirm_fast25_max025",
};

// This observational series starts checking only after the adopted 73 ms input
// protection ends. Three consecutive reverse-rate samples confirm the turn;
// then beta returns from its floor to 0.025 in 25 ms. A fallback bounds it.
static constexpr float BETA_TURN_FAST_OUTBOUND_RATE_MIN_DPS = 0.25f;
static constexpr float BETA_TURN_FAST_TURN_CONFIRM_RATE_DPS = 0.25f;
static constexpr float BETA_TURN_FAST_MIN_PEAK_ANGLE_DEG = 0.25f;
static constexpr uint8_t BETA_TURN_FAST_CONFIRM_SAMPLES = 3;
static constexpr uint16_t BETA_TURN_FAST_RECOVERY_MS = 25;
static constexpr uint16_t BETA_TURN_FAST_FALLBACK_MS = 400;

struct BetaSweepTrial {
  const char* name;
  int16_t current_mA;
  uint16_t pulse_width_ms;
  uint16_t input_interval_ms;
  uint32_t duration_ms;
};

// Legacy reference conditions for the non-UI batch path. The Web UI accepts
// every integer current from 100 to 300 mA and width from 30 to 100 ms.
// It runs one selected condition per 30 s capture; it does not batch-run this table.
static constexpr BetaSweepTrial BETA_SWEEP_TRIALS[] = {
    {"200mA_30ms", 200, ZERO_CROSS_PULSE_30_MS, 1000, BETA_SWEEP_TRIAL_DURATION_MS},
    {"200mA_40ms", 200, ZERO_CROSS_PULSE_40_MS, 1000, BETA_SWEEP_TRIAL_DURATION_MS},
    {"200mA_50ms", 200, ZERO_CROSS_PULSE_50_MS, 1000, BETA_SWEEP_TRIAL_DURATION_MS},
    {"200mA_55ms", 200, ZERO_CROSS_PULSE_55_MS, 1000, BETA_SWEEP_TRIAL_DURATION_MS},
    {"200mA_60ms", 200, ZERO_CROSS_PULSE_60_MS, 1000, BETA_SWEEP_TRIAL_DURATION_MS},
    {"200mA_65ms", 200, ZERO_CROSS_PULSE_65_MS, 1000, BETA_SWEEP_TRIAL_DURATION_MS},
    {"200mA_70ms_boundary", 200, ZERO_CROSS_PULSE_70_MS, 1000, BETA_SWEEP_TRIAL_DURATION_MS},
};

static constexpr uint8_t BETA_SWEEP_TRIAL_COUNT = sizeof(BETA_SWEEP_TRIALS) / sizeof(BETA_SWEEP_TRIALS[0]);
static constexpr uint32_t BETA_SWEEP_TOTAL_DURATION_MS =
    BETA_SWEEP_TRIAL_COUNT * BETA_SWEEP_TRIAL_DURATION_MS +
    (BETA_SWEEP_TRIAL_COUNT - 1) * BETA_SWEEP_INTER_TRIAL_REST_MS;

// V50 storage only: reserve PSRAM for full-rate current attempts and export JSON.
static constexpr size_t LOG_BUFFER_BYTES = 4UL * 1024UL * 1024UL;
static constexpr uint8_t BUFFER_WARNING_PERCENT = 90;

}  // namespace Config






