// 4-motor closed-loop RPM controller, RP2040 (arduino-pico / Earle Philhower core)
// + RC PWM input -> mecanum mixing -> per-wheel rpm setpoints
//
//   core 0: setup()/loop()   -> serial test input + RC read (ISRs) -> setVelocities() + telemetry print
//   core 1: setup1()/loop1() -> PIO encoders + mecanum velocity layer (setVelocities) + PID + PWM/direction pins
//                               + GY-87 AHRS + 4x VL53L0X (I2C0, GP0/GP1), run in the idle time between PID ticks
//
// Easy API (callable from core 0, or anywhere):
//   motorSetRPM(m, rpm)         m = 0..3  (FL, FR, BL, BR), rpm signed, +CW / -CCW
//   motorSetAll(a, b, c, d)
//   motorGetRPM(m)             filtered measured rpm
//   setVelocities(vx, vy, vr)  body velocity target: vx strafe right (m/s), vy forward (m/s), vr rotation clockwise (deg/s).
//                              core 1 turns it into wheel rpm setpoints and closes the yaw rate on the gyro
//   setYawHold(deg)            -1 = no hold, 0..359 = hold this heading in the earth frame (clockwise from north).
//                              angle PID -> yaw-rate target (overrides vr) -> gyro rate loop
//   velocityGet(i)             i = 0..2 -> measured vx, vy (m/s), vr (deg/s) from wheel encoders
//   rcRead(ch)                 RC pulse width in us (0 = no signal)
//   lidarGet(i)                i = 0..3 -> dist1..dist4 in mm, -1 = no valid reading
//   IMU: g_roll g_pitch g_yaw (deg), g_acc[3] g_lin[3] (m/s2, x right / y up / z fwd), g_alt g_vz, g_tempC g_pressPa
//
// Serial (115200):  "rpm 100,200,300,400" -> manual wheel rpm (RC control off)
//                   "rc"                  -> back to RC control
//                   "vel 0.2,0,0"         -> vx,vy (m/s), vr (deg/s) via setVelocities (RC control off)
//                   "hold 90"             -> hold heading 90 deg ("hold -1" = off, "hold" = print)
//                   "auto 1" / "auto 0"   -> side-lidar auto strafe on / off ("auto" = print)
//                   "c"                   -> start/stop magnetometer calibration (rotate in all directions)
//                   "a"                   -> start/abort 6-position accelerometer calibration ("n" = capture each position)
//                   "n"                   -> capture current position during accel calibration
//                   "l"                   -> level calibration (robot on a flat surface) -> roll/pitch trim
//                   "z"                   -> re-zero baro altitude
//
// MAZE (cell moves, ToF only, see the MAZE section): the robot never rotates, N = forward at the BOOTSEL press.
//   RC: flick + release the pitch stick = 1 cell N/S, the roll stick = 1 cell E/W (full deflection = 2 cells). Ignored while solving.
//   BOOTSEL: first press after power-up = start (heading captured, map cleared, position = start cell) + SOLVE (flood fill to the goal).
//            press while solving = stop at the next cell centre (press again before it stops = stop immediately). short press
//            while stopped = solve again from where it is (map kept). hold 1.5 s while stopped = new start (heading captured,
//            map cleared, position = start cell). LED on while solving.
//                   "solve"               -> same as a short BOOTSEL press (start / stop the flood-fill run)
//                   "goal x y"            -> set the goal cell ("goal" = print)
//                   "go N 2"              -> move 2 cells N (N/E/S/W, n = 1..3), same as an RC flick
//                   "stop"                -> abort the current move now (and the solve run)
//                   "map"                 -> print the map (values + drawing)
//                   "win"                 -> take a standstill window, print readings / walls / free cells / offsets
//                   "reset"               -> new start: heading captured, map cleared, position = start cell (no solve)
//                   "ccal"                -> robot centred in a closed 4-wall cell: measure C_MM[] (print, paste into the code)
//                   "tcal i mm"           -> add a ruler point for sensor i (0 F, 1 B, 2 L, 3 R); "tcal i fit" -> gain/offset + residuals; "tcal i clr"
//                   "mv kp,vmax,vcreep"   -> drive tuning (no args = print)
//                   "tel 0/1"             -> 200 ms telemetry off/on (to read the MOVE lines)
//                   "mzdbg 0/1"           -> drive debug lines (progress, target, speed) every 50 ms

#include <Arduino.h>
#include <strings.h>
#include "hardware/pio.h"
#include <Wire.h>
#include <math.h>
#include <VL53L0X.h>               // "VL53L0X" by Pololu

// must live above all functions, or the IDE's auto-prototypes hit "'Rot' does not name a type"
struct Rot { float r11, r21, r31, r32, r33; };

// ===================== CONFIG =====================

#define N_MOTORS 4
#define LIDAR_LEFT  2
#define LIDAR_RIGHT 3
#define LIDAR_FRONT 0
#define LIDAR_BACK  1
#define TELEMETRY_MS 200    // 0 = off
#define RC_PIN_CH1 18
#define RC_PIN_CH2 19
#define RC_PIN_CH3 20
#define RC_PIN_CH4 21
#define RC_CENTER_US     1485
#define RC_DEADBAND_US   5         // center +- this = zero
#define RC_MIN_US        995       // full stick one way
#define RC_MAX_US        1970      // full stick other way
#define RC_REJECT_MIN_US 800       // pulses outside this window are treated as noise
#define RC_REJECT_MAX_US 2200
#define RC_TIMEOUT_MS    100       // rcRead() returns 0 after this long with no valid pulse
#define RC_CH_STRAFE 1             // sideways   (stick right = +)
#define RC_CH_FWD    2             // forward    (stick up    = +)
#define RC_CH_ROT    4             // rotation   (stick right = +, clockwise)
#define SIGN_STRAFE  1.0f          // flip to -1.0f if a channel moves the wrong way
#define SIGN_FWD     1.0f
#define SIGN_ROT     1.0f
#define VEL_PERIOD_US 10000                   // velocity layer update period (100 Hz)
#define RC_UPDATE_MS 20            // RC -> setpoint update rate (50 Hz)
#define SENS_SDA_PIN 0
#define SENS_SCL_PIN 1
#define SENS_I2C_HZ  400000        // TCA9548A max is 400 kHz
#define SENSOR_GUARD_US 900        // a sensor job only starts if at least this much time is left before the next PID tick

// order: a=FL, b=FR, c=BL, d=BR.  encB is always encA+1
struct MotorPins { uint8_t cw, ccw, en, encA; };
static const MotorPins MOT[N_MOTORS] = {
  //  CW  CCW  EN  ENC_A
  {   26,  28, 16,  8 },   // FL  (ENC_B = 9)
  {   11,  10, 15,  6 },   // FR  (ENC_B = 7)
  {   22,  27, 17,  2 },   // BL  (ENC_B = 3)
  {   13,  12, 14,  4 },   // BR  (ENC_B = 5)
};

static const int8_t ENC_DIR[N_MOTORS] = { 1, -1, 1, -1 };  // flip to -1 for a motor if measured rpm is negative while driving CW
static const int8_t WHEEL_DIR[N_MOTORS] = { 1, 1, 1, 1 };  // 1 -1 1 -1 = right wrong way

static const uint8_t LIDAR_INVALID_MAX = 5;   // this many invalid samples in a row -> filtered value = -1

static const int TIME_WAIT_BOOT = 5000;   // time in milliseconds to wait before calibrating the IMU
static const int   WALL_SEEN_MM    = 140;     // a side lidar reading above this (or -1) = no wall on that side
static const int COLLISION_STOP_MM = 50;

static const float COUNTS_PER_REV   = 672.0f;     // x4 decoding -> counts per OUTPUT-shaft rev = encoder PPR * 4 * gear ratio = 3 * 4 * 24
static const float RPM_MAX          = 520.0f;     // motor limits
static const float MIN_DUTY         = 45.0f;      // duty (of 255) where the motor starts to move
static const float RPM_FILT_ALPHA   = 0.3f;       // measured-rpm EMA (1 = no filter)
static const float INTEG_LIM        = 80.0f;      // max duty contribution of I term
static const float DRIVE_MAX_RPM    = 450.0f;     // max duty to use when driving
static const float BRAKE_DUTY       = 255.0f;     // brake strength when setpoint = 0 (0 = coast)
static const float BRAKE_MARGIN_RPM = 20.0f;      // brake when measured speed exceeds setpoint by this much (same direction)
static const float WHEEL_RADIUS_M   = 0.030f;     // FILL IN: wheel radius (m)
static const float ROBOT_LX_M       = 0.0375f;    // FILL IN: HALF the distance between front and rear axles (m)
static const float ROBOT_LY_M       = 0.05f;      // FILL IN: HALF the distance between left and right wheel centres (m)
static const float RC_MAX_VX_MS     = 1.20f;      // FILL IN: strafe speed at full stick (m/s)
static const float RC_MAX_VY_MS     = 1.20f;      // FILL IN: forward speed at full stick (m/s)
static const float RC_MAX_VR_DPS    = 780.0f;     // FILL IN: rotation rate at full stick (deg/s)
static const float YAW_I_LIM        = 135.0f;     // max deg/s contribution of the I term
static const float WALL_HOLD_MM     = 92.0f;      // distance kept from the wall when only one side wall is seen
static const float STRAFE_AUTO_MAX  = 0.40f;      // m/s max auto strafe
static const float STRAFE_DB_MM     = 3.0f;       // mm: error deadband (lidar noise), no strafe command inside it
static const float WALL_HOLD_FB_MM  = 70.0f;      // same as WALL_HOLD_MM, for the front/back pair (used while strafing)
static const float LIDAR_EMA_ALPHA  = 0.4f;       // EMA on each valid ToF sample (1 = no filter)
static const float LIDAR_F_CAP_MM   = 1000.0f;     // samples clipped to this before the EMA (out-of-range values like 8190 would drag it)
static const float ANG_DB_IN        = 0.4f;       // deg: heading error below this (and nothing commanded) -> brake, stop driving the wheels
static const float ANG_DB_OUT       = 1.0f;       // deg: heading error above this -> angle loop takes over again

static const uint32_t PWM_FREQ       = 20000;      // Hz
static const uint32_t PID_PERIOD_US  = 2000;      // 1000 Hz loop

static volatile float K_SLIP_STRAFE = 0.75f;
static volatile float K_SLIP_FWD    = 0.95f;

static const bool  YAW_PID_EN = true;

static float YAW_KP       = 2.50f;
static float YAW_KI       = 8.00f;
static float STRAFE_KP    = 0.006f;  // m/s of strafe per mm of error
static float ANG_KP       = 4.0f;     // deg/s per deg of heading error
static float ANG_KI       = 0.0f;
static float ANG_KD       = 0.0f;     // acts on measured yaw rate
static float ANG_I_LIM    = 60.0f;    // deg/s max from I term
static float ANG_RATE_MAX = 360.0f;   // deg/s max rate commanded to the rate loop
static float STRAFE_YAW_FF = -80.0f;
static float KP = 0.40f;
static float KI = 4.00f;
static float KD = 0.00f;

// ===================== SHARED STATE =====================
// 32-bit aligned float accesses are atomic on RP2040, so plain volatiles are fine.
static volatile float g_setpoint[N_MOTORS];
static volatile float g_measured[N_MOTORS];
static volatile bool  g_rcMode = true;             // true = RC drives the wheels, false = serial "rpm" command

uint32_t loopHz=0;

// ---- sensor data: written by core 1, read from either core (aligned 32-bit/16-bit accesses are atomic) ----
// IMU (GY-87). frame: x = right, y = up, z = forward
static volatile float    g_roll, g_pitch, g_yaw;          // deg. roll: right side down +, pitch: nose up +, yaw: clockwise from north 0..360
static volatile float    g_acc[3];                        // m/s^2, raw specific force (x right, y up, z fwd)
static volatile float    g_lin[3];                        // m/s^2, gravity removed
static volatile float    g_alt, g_vz;                     // m, m/s  (baro + accel fused)
static volatile float    g_tempC, g_pressPa;              // BMP180
static volatile bool     g_magValid;
static volatile uint32_t g_imuSeq;                        // +1 on every IMU publish (detect fresh data)
// lidar: 4x VL53L0X, mm, -1 = no valid reading. index 0..3 = dist1..dist4 (TCA ch 7,6,5,4)
static volatile int16_t  g_lidar[4] = {-1, -1, -1, -1};
static volatile float    g_lidarF[4] = {-1, -1, -1, -1};  // EMA-filtered g_lidar (mm), used by the auto centering
static volatile uint32_t g_lidarSeq;                      // +1 on every new lidar sample (any sensor)
static volatile uint32_t g_lidarSeqI[4];                  // +1 on every new sample of sensor i (maze layer: fresh-sample windows)
// status / cross-core commands
static volatile bool     g_sensorsReady = false;
static volatile bool     g_mpuOk, g_qmcOk, g_bmpOk;
static volatile uint8_t  g_mpuWho, g_qmcId, g_tofMask;    // g_tofMask bit i = dist(i+1) found
static volatile bool     g_cmdZero = false;               // core 0 -> core 1: re-zero altitude
static volatile bool     g_cmdMagCal = false;             // core 0 -> core 1: toggle mag calibration
static volatile uint8_t  g_calEvent = 0;                  // core 1 -> core 0: 1 = cal started, 2 = cal finished (core 0 prints, then clears)
static volatile bool g_magUse = true;
// accel 6-position calibration + level calibration
static volatile uint8_t  g_accCmd = 0;                    // core 0 -> core 1: 1 = start/abort, 2 = capture current position
static volatile uint8_t  g_accEvent = 0;                  // core 1 -> core 0: 1 started, 2 captured, 3 rejected (retry), 4 done, 5 aborted (core 0 prints, then clears)
static volatile uint8_t  g_accStep = 0;                   // current position index 0..5
static volatile bool     g_accActive = false;             // accel cal in progress
static volatile bool     g_cmdLevel = false;              // core 0 -> core 1: start level calibration
static volatile uint8_t  g_lvlEvent = 0;                  // core 1 -> core 0: 1 = level cal finished
// velocity layer (runs on core 1)
static volatile float    g_velCmd[3];                     // target: vx (m/s, right +), vy (m/s, fwd +), vr (deg/s, clockwise +)
static volatile float    g_velMeas[3];                    // measured from wheel encoders, same units
static volatile bool     g_velActive = false;             // true = velocity layer drives the wheel setpoints; any motorSetRPM/motorSetAll takes over
static volatile float    g_yawHold = -1.0f;               // -1 = off, 0..359 = hold this heading (earth frame, clockwise from north)
static volatile bool     g_autoStrafe = true;             // true = vx comes from the side lidars (overrides the commanded strafe), false = commanded vx
static volatile bool     g_yawHoldSet = false;            // true once any hold command was given; until then the boot heading is held
static bool g_started = false;

static volatile float g_posX = 0.0f;
static volatile float g_posY = 0.0f;

void motorSetRPM(uint8_t m, float rpm) {
  if (m < N_MOTORS) { g_velActive = false; g_setpoint[m] = constrain(rpm, -RPM_MAX, RPM_MAX); }
}
void motorSetAll(float a, float b, float c, float d) {
  motorSetRPM(0, a); motorSetRPM(1, b); motorSetRPM(2, c); motorSetRPM(3, d);
}
float motorGetRPM(uint8_t m) { return (m < N_MOTORS) ? g_measured[m] : 0.0f; }
int16_t lidarGet(uint8_t i) { return (i < 4) ? g_lidar[i] : (int16_t)-1; }
// vx = strafe right (m/s), vy = forward (m/s), vr = rotate clockwise (deg/s). Callable from either core.
void setVelocities(float vx, float vy, float vr) {
  g_velCmd[0] = vx; g_velCmd[1] = vy; g_velCmd[2] = vr;
  g_velActive = true;
}
// deg < 0 = hold off, 0..359 = hold this heading (earth frame, clockwise from north). Callable from either core.
void setYawHold(float deg) { g_yawHoldSet = true; g_yawHold = (deg < 0.0f) ? -1.0f : fmodf(deg, 360.0f); }
float velocityGet(uint8_t i) { return (i < 3) ? g_velMeas[i] : 0.0f; }

// ===================== ENCODERS (PIO, x4 quadrature) =====================
// One PIO state machine per encoder (pio0 has exactly 4), all running the same program.
// Counting is done in hardware, so no GPIO interrupts and no missed edges.
// Program is pre-assembled with pioasm (no .pio file needed). It reads pins encA, encA+1
// as state = (A<<1)|B, same direction convention as the old software decoder.
static const uint16_t ENC_PIO_INSTR[] = {
  0x0015, 0x0010, 0x0014, 0x0015, 0x0014, 0x0015, 0x0015, 0x0010,
  0x0010, 0x0015, 0x0015, 0x0014, 0x0015, 0x0014, 0x0010, 0x0015,
  0xa04a, 0x0092, 0xa04a, 0x0015, 0x0095, 0xa0c2, 0x8000, 0x60c2,
  0x4002, 0xa0e6, 0xa0a6
};
static PIO  enc_pio = pio0;
static uint enc_sm[N_MOTORS];

// drain the RX FIFO, plus one blocking read that is guaranteed fresh
static int32_t encRaw(int i) {
  int n = pio_sm_get_rx_fifo_level(enc_pio, enc_sm[i]) + 1;
  uint32_t v = 0;
  while (n-- > 0) v = pio_sm_get_blocking(enc_pio, enc_sm[i]);
  return (int32_t)v;
}

static void encInit() {
  static pio_program prog = {};
  prog.instructions = ENC_PIO_INSTR;
  prog.length = sizeof(ENC_PIO_INSTR) / sizeof(ENC_PIO_INSTR[0]);
  prog.origin = 0;
  const uint offset = 0;                                  // program needs offset 0 (jump table)
  pio_add_program_at_offset(enc_pio, &prog, offset);

  for (int i = 0; i < N_MOTORS; i++) {
    uint sm = pio_claim_unused_sm(enc_pio, true);
    enc_sm[i] = sm;
    for (uint p = MOT[i].encA; p < MOT[i].encA + 2u; p++) {
      pio_gpio_init(enc_pio, p);
      gpio_pull_up(p);
    }
    pio_sm_set_consecutive_pindirs(enc_pio, sm, MOT[i].encA, 2, false);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + prog.length - 1);
    sm_config_set_in_pins(&c, MOT[i].encA);
    sm_config_set_in_shift(&c, false, false, 32);         // shift left, no autopush
    sm_config_set_out_shift(&c, true, false, 32);         // shift right, no autopull
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&c, 1.0f);
    pio_sm_init(enc_pio, sm, offset, &c);
  }
  for (int i = 0; i < N_MOTORS; i++) pio_sm_set_enabled(enc_pio, enc_sm[i], true);
}

static int32_t encCount(int i) { return encRaw(i) * ENC_DIR[i]; }

// ===================== RC PWM INPUT =====================
#define RC_N 4
static const uint8_t RC_PINS[RC_N] = { RC_PIN_CH1, RC_PIN_CH2, RC_PIN_CH3, RC_PIN_CH4 };
static volatile uint32_t rcRise[RC_N];
static volatile uint16_t rcWidth[RC_N];
static volatile uint32_t rcLastMs[RC_N];

static void rcISR(void *arg) {
  int i = (int)(intptr_t)arg;
  uint32_t now = micros();
  if (gpio_get(RC_PINS[i])) {
    rcRise[i] = now;
  } else {
    uint32_t w = now - rcRise[i];
    if (w >= RC_REJECT_MIN_US && w <= RC_REJECT_MAX_US) {
      rcWidth[i]  = (uint16_t)w;
      rcLastMs[i] = millis();
    }
  }
}

static void rcInit() {
  for (int i = 0; i < RC_N; i++) {
    pinMode(RC_PINS[i], INPUT_PULLDOWN);
    attachInterruptParam(RC_PINS[i], rcISR, CHANGE, (void *)(intptr_t)i);
  }
}

// ch = 1..4 (RC channel number). returns pulse width in us, 0 = no signal
uint16_t rcRead(uint8_t ch) {
  if (ch < 1 || ch > RC_N) return 0;
  uint8_t i = ch - 1;
  if (millis() - rcLastMs[i] > RC_TIMEOUT_MS) return 0;
  return rcWidth[i];
}

// pulse width -> -1..+1, zero inside the center deadband, separate scaling each side
static float rcNorm(uint16_t us) {
  const int hi = RC_CENTER_US + RC_DEADBAND_US;
  const int lo = RC_CENTER_US - RC_DEADBAND_US;
  int u = (int)us;
  float v;
  if (u > hi)      v =  (float)(u - hi) / (float)(RC_MAX_US - hi);
  else if (u < lo) v = -(float)(lo - u) / (float)(lo - RC_MIN_US);
  else             v = 0.0f;
  return constrain(v, -1.0f, 1.0f);
}

// ===================== MOTOR DRIVE =====================
static void driveMotor(int i, float duty, bool brake = false) {
  if (brake) {                       // both inputs LOW + EN high = motor terminals shorted
    digitalWrite(MOT[i].cw,  LOW);
    digitalWrite(MOT[i].ccw, LOW);
    analogWrite(MOT[i].en, (int)BRAKE_DUTY);
    return;
  }
  int d = (int)lroundf(duty);
  if (d > 0)      { digitalWrite(MOT[i].ccw, LOW);  digitalWrite(MOT[i].cw, HIGH); }
  else if (d < 0) { digitalWrite(MOT[i].cw,  LOW);  digitalWrite(MOT[i].ccw, HIGH); }
  else            { digitalWrite(MOT[i].cw,  LOW);  digitalWrite(MOT[i].ccw, LOW); }
  analogWrite(MOT[i].en, abs(d));
}
// ===================== SENSORS (run on core 1, inside the PID core) =====================
// GY-87 (MPU6050 + QMC5883P via MPU bypass + BMP180) and 4x VL53L0X behind a TCA9548A, all on I2C0.
// ONLY core 1 touches Wire. sensorsStep() does ONE small job per call and loop1() only calls it when there
// is enough idle time before the next PID tick (SENSOR_GUARD_US), so PID timing is not disturbed.
// Results are published into the g_* variables in SHARED STATE -> readable from either core.
// Core 1 never prints; status/events go through g_* flags and core 0 prints them.

// ---- GY-87 config ----
static const uint8_t MPU_ADDR = 0x68;
static const uint8_t QMC_ADDR = 0x2C;
static const uint8_t BMP_ADDR = 0x77;

static const float DECLINATION_DEG = 1.0f;  // magnetic declination, Faridabad ~ +1 deg E
static const float BETA_START = 1.5f;       // Madgwick gain during initial convergence
static const float BETA_RUN   = 0.005f;      // Madgwick gain in steady state
static const uint32_t CONVERGE_MS = 2500;
static const float G0 = 9.80665f;
static const uint8_t BMP_OSS = 3;
// Mag hard/soft-iron calibration (chip frame, uT). Run 'c' and paste the printed values here.
static float magOff[3]   = {-9.40f, 2.13f, 4.21f};
static float magScale[3] = {1.029f, 0.984f, 0.988f};
// Accel calibration (chip frame, in g): cal = (raw - accOff) * accScale. Run 'a' (6 positions) and paste the printed values here.
static float accOff[3]   = {0.0219f, 0.0f, 0.0364f};
static float accScale[3] = {1.0057f, 0.9977f, 0.9868f};
// Level trim (deg), subtracted from published roll/pitch. Run 'l' with the robot flat and paste the printed values here.
static float rollTrim = -1.59f, pitchTrim = 3.10f;

static const float DEG2RAD = 0.017453292519943295f;
static const float RAD2DEG = 57.29577951308232f;

// ---- VL53L0X / TCA9548A config ----
static const uint8_t  TCA_ADDR = 0x70;
static const uint8_t  TOF_N    = 4;
static const uint8_t  TOF_CH[TOF_N] = {7, 6, 5, 4};          // dist1=ch7, dist2=ch6, dist3=ch5, dist4=ch4
static const uint32_t TOF_TIMING_BUDGET_US = 20000;          // minimum the lib allows -> ~50 Hz per sensor

// ----------------------------- axis mapping -----------------------------
// Internal body frame F = (fwd, left, up), right-handed (Madgwick convention).
// Default mount assumed: chip +X = right, chip +Y = up, chip +Z = BACK
// (your "x right, y up, z fwd" is left-handed, a real right-handed chip can't have all three).
// Must stay a proper rotation (det = +1). Edit if your board is mounted differently.
static inline void imuToBody(float cx, float cy, float cz, float &fx, float &fy, float &fz) {
  fx = -cz; fy = -cx; fz = cy;
}
// Mag chip axes may differ from the MPU. If heading is mirrored/jumpy, fix it here.
static inline void magToBody(float cx, float cy, float cz, float &fx, float &fy, float &fz) {
  fx = cz; fy = cy; fz = -cx;
}
// Body F -> your frame (x right, y up, z fwd), used for printing only.
static inline void bodyToUser(float fx, float fy, float fz, float &ux, float &uy, float &uz) {
  ux = -fy; uy = fz; uz = fx;
}

// ------------------------------ I2C helpers ------------------------------
static bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (size_t)n) != (size_t)n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// ------------------------------ state ------------------------------
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   // body->earth (earth: x=N, y=W, z=up)
static float beta = BETA_START;

static float gyroBias[3] = {0, 0, 0};          // chip frame, rad/s
static float gyroYawRate = 0;                  // deg/s, clockwise +, bias removed (updated at 200 Hz, used by the velocity layer)
static float aF[3] = {0, 0, G0};               // accel, body F, m/s^2 (specific force)
static float mF[3] = {1, 0, 0};                // mag, body F, uT (calibrated)
static bool  magValid = false;
static float magRawChip[3] = {0, 0, 0};

static float fAlt = 0, fVz = 0;                // fused altitude / vertical speed
static float bTempC = 0, bPa = 0, baroAlt = 0, p0 = 0, pSum = 0;
static int   nRef = 0;
static uint32_t tLastBaro = 0;

static bool  calibrating = false;
static float calMin[3], calMax[3];
static uint32_t calStart = 0;

static uint32_t tImu = 0, tMag = 0, tStart = 0;
static uint32_t i2cErr = 0;

// ------------------------------ MPU6050 ------------------------------
static bool mpuInit() {
  writeReg(MPU_ADDR, 0x6B, 0x80);   // device reset
  delay(100);
  writeReg(MPU_ADDR, 0x6B, 0x01);   // wake, clock = gyro X PLL
  delay(10);
  writeReg(MPU_ADDR, 0x6A, 0x00);   // I2C_MST_EN = 0
  writeReg(MPU_ADDR, 0x37, 0x02);   // INT_PIN_CFG: I2C_BYPASS_EN = 1 -> mag/baro visible on main bus
  writeReg(MPU_ADDR, 0x1A, 0x03);   // DLPF ~44 Hz
  writeReg(MPU_ADDR, 0x19, 0x04);   // sample rate 1 kHz / 5 = 200 Hz
  writeReg(MPU_ADDR, 0x1B, 0x08);   // gyro  +-500 dps  (65.5 LSB/dps)
  writeReg(MPU_ADDR, 0x1C, 0x08);   // accel +-4 g      (8192 LSB/g)
  uint8_t who = 0;
  if (!readRegs(MPU_ADDR, 0x75, &who, 1)) return false;
  g_mpuWho = who;
  return true;
}

static void mpuCalibGyro() {
  double s[3] = {0, 0, 0};
  int n = 0;
  for (int i = 0; i < 400; i++) {
    uint8_t b[6];
    if (readRegs(MPU_ADDR, 0x43, b, 6)) {
      s[0] += (int16_t)((b[0] << 8) | b[1]);
      s[1] += (int16_t)((b[2] << 8) | b[3]);
      s[2] += (int16_t)((b[4] << 8) | b[5]);
      n++;
    }
    delay(3);
  }
  if (n > 0) for (int i = 0; i < 3; i++) gyroBias[i] = (float)(s[i] / n) / 65.5f * DEG2RAD;
}

// ------------------------------ QMC5883P (0x2C) ------------------------------
static bool qmcInit() {
  uint8_t id = 0;
  if (!readRegs(QMC_ADDR, 0x00, &id, 1)) return false;
  g_qmcId = id;
  writeReg(QMC_ADDR, 0x0B, 0x80);   // soft reset
  delay(10);
  writeReg(QMC_ADDR, 0x0B, 0x08);   // range +-8 G, set/reset on
  writeReg(QMC_ADDR, 0x0A, 0x0B);   // continuous, ODR 100 Hz, OSR 8, downsample 1
  return true;
}

static bool qmcRead(float uT[3]) {
  uint8_t st = 0;
  if (!readRegs(QMC_ADDR, 0x09, &st, 1)) return false;
  if (!(st & 0x01)) return false;   // DRDY
  uint8_t b[6];
  if (!readRegs(QMC_ADDR, 0x01, b, 6)) return false;
  int16_t x = (int16_t)((b[1] << 8) | b[0]);
  int16_t y = (int16_t)((b[3] << 8) | b[2]);
  int16_t z = (int16_t)((b[5] << 8) | b[4]);
  const float k = 100.0f / 3750.0f; // 3750 LSB/G at +-8G, 1 G = 100 uT
  uT[0] = x * k; uT[1] = y * k; uT[2] = z * k;
  return true;
}

static void magCalStart() {
  for (int i = 0; i < 3; i++) { calMin[i] = 1e9f; calMax[i] = -1e9f; }
  calStart = millis();
  calibrating = true;
  g_calEvent = 1;
}

static void magCalFinish() {
  calibrating = false;
  float half[3], avg = 0;
  for (int i = 0; i < 3; i++) {
    magOff[i] = 0.5f * (calMax[i] + calMin[i]);
    half[i] = 0.5f * (calMax[i] - calMin[i]);
    if (half[i] < 1e-3f) half[i] = 1e-3f;
    avg += half[i];
  }
  avg /= 3.0f;
  for (int i = 0; i < 3; i++) magScale[i] = avg / half[i];
  g_calEvent = 2;
}

static void stepMag() {
  uint32_t ms = millis();
  if (ms - tMag < 10) return;
  tMag = ms;
  float m[3];
  if (!qmcRead(m)) return;
  for (int i = 0; i < 3; i++) magRawChip[i] = m[i];
  if (calibrating) {
    for (int i = 0; i < 3; i++) {
      if (m[i] < calMin[i]) calMin[i] = m[i];
      if (m[i] > calMax[i]) calMax[i] = m[i];
    }
    if (ms - calStart > 25000) magCalFinish();
  }
  float c[3];
  for (int i = 0; i < 3; i++) c[i] = (m[i] - magOff[i]) * magScale[i];
  magToBody(c[0], c[1], c[2], mF[0], mF[1], mF[2]);
  magValid = true;
}

// ------------------------------ BMP180 ------------------------------
static int16_t  bAC1, bAC2, bAC3, bB1, bB2, bMB, bMC, bMD;
static uint16_t bAC4, bAC5, bAC6;
static int bState = 0;
static uint32_t tBmp = 0;
static int32_t bUT = 0, bUP = 0;

static bool bmpInit() {
  uint8_t id = 0;
  if (!readRegs(BMP_ADDR, 0xD0, &id, 1) || id != 0x55) return false;
  uint8_t d[22];
  if (!readRegs(BMP_ADDR, 0xAA, d, 22)) return false;
  auto s16 = [&](int i) { return (int16_t)((d[i] << 8) | d[i + 1]); };
  auto u16 = [&](int i) { return (uint16_t)((d[i] << 8) | d[i + 1]); };
  bAC1 = s16(0); bAC2 = s16(2); bAC3 = s16(4);
  bAC4 = u16(6); bAC5 = u16(8); bAC6 = u16(10);
  bB1 = s16(12); bB2 = s16(14); bMB = s16(16); bMC = s16(18); bMD = s16(20);
  return true;
}

static void bmpCompute(int32_t ut, int32_t up) {
  int32_t x1 = ((ut - (int32_t)bAC6) * (int32_t)bAC5) >> 15;
  int32_t x2 = ((int32_t)bMC << 11) / (x1 + bMD);
  int32_t b5 = x1 + x2;
  bTempC = ((b5 + 8) >> 4) / 10.0f;
  int32_t b6 = b5 - 4000;
  x1 = ((int32_t)bB2 * ((b6 * b6) >> 12)) >> 11;
  x2 = ((int32_t)bAC2 * b6) >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = ((((int32_t)bAC1 * 4 + x3) << BMP_OSS) + 2) >> 2;
  x1 = ((int32_t)bAC3 * b6) >> 13;
  x2 = ((int32_t)bB1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  uint32_t b4 = ((uint32_t)bAC4 * (uint32_t)(x3 + 32768)) >> 15;
  uint32_t b7 = ((uint32_t)up - (uint32_t)b3) * (uint32_t)(50000 >> BMP_OSS);
  int32_t p = (b7 < 0x80000000UL) ? (int32_t)((b7 << 1) / b4) : (int32_t)((b7 / b4) << 1);
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p += (x1 + x2 + 3791) >> 4;
  bPa = (float)p;
}

// complementary (alpha-beta) fusion of baro altitude into the accel-integrated state
static void baroFuse() {
  uint32_t ms = millis();
  if (nRef < 16) {                       // average first samples as ground reference
    pSum += bPa;
    if (++nRef == 16) { p0 = pSum / 16.0f; fAlt = 0; fVz = 0; }
    tLastBaro = ms;
    return;
  }
  baroAlt = 44330.0f * (1.0f - powf(bPa / p0, 0.190295f));
  float dt = (ms - tLastBaro) * 1e-3f;
  tLastBaro = ms;
  if (dt < 0.005f || dt > 0.5f) return;
  const float alpha = 0.15f;
  const float betaG = alpha * alpha / (2.0f - alpha);
  float err = baroAlt - fAlt;
  fAlt += alpha * err;
  fVz  += (betaG / dt) * err;
}

static bool stepBaro() {
  uint32_t ms = millis();
  switch (bState) {
    case 0:
      if (writeReg(BMP_ADDR, 0xF4, 0x2E)) { tBmp = ms; bState = 1; return true; }
      break;
    case 1:
      if (ms - tBmp >= 5) {
        uint8_t d[2];
        if (readRegs(BMP_ADDR, 0xF6, d, 2)) {
          bUT = (d[0] << 8) | d[1];
          writeReg(BMP_ADDR, 0xF4, 0x34 | (BMP_OSS << 6));
          tBmp = ms; bState = 2;
        } else bState = 0;
        return true;
      }
      break;
    case 2:
      if (ms - tBmp >= 26) {
        uint8_t d[3];
        if (readRegs(BMP_ADDR, 0xF6, d, 3)) {
          bUP = (((int32_t)d[0] << 16) | ((int32_t)d[1] << 8) | d[2]) >> (8 - BMP_OSS);
          bmpCompute(bUT, bUP);
          baroFuse();
        }
        bState = 0;
        return true;
      }
      break;
  }
  return false;
}

// ------------------------------ Madgwick MARG ------------------------------
// gyro rad/s, accel any unit, mag any unit; body F = (fwd, left, up)
static void madgwick(float gx, float gy, float gz, float ax, float ay, float az,
                     float mx, float my, float mz, float dt) {
  float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  float an = sqrtf(ax * ax + ay * ay + az * az);
  float mn = sqrtf(mx * mx + my * my + mz * mz);
  if (an > 1e-6f && mn > 1e-6f) {
    ax /= an; ay /= an; az /= an;
    mx /= mn; my /= mn; mz /= mn;

    float _2q0mx = 2.0f * q0 * mx, _2q0my = 2.0f * q0 * my, _2q0mz = 2.0f * q0 * mz, _2q1mx = 2.0f * q1 * mx;
    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _2q0q2 = 2.0f * q0 * q2, _2q2q3 = 2.0f * q2 * q3;
    float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
    float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
    float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

    float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    float _2bx = sqrtf(hx * hx + hy * hy);
    float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mx * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    float _4bx = 2.0f * _2bx, _4bz = 2.0f * _2bz;

    float eA1 = 2.0f * q1q3 - _2q0q2 - ax;
    float eA2 = 2.0f * q0q1 + _2q2q3 - ay;
    float eA3 = 1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az;
    float eM1 = _2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx;
    float eM2 = _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my;
    float eM3 = _2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz;

    float s0 = -_2q2 * eA1 + _2q1 * eA2 - _2bz * q2 * eM1 + (-_2bx * q3 + _2bz * q1) * eM2 + _2bx * q2 * eM3;
    float s1 =  _2q3 * eA1 + _2q0 * eA2 - 4.0f * q1 * eA3 + _2bz * q3 * eM1 + (_2bx * q2 + _2bz * q0) * eM2 + (_2bx * q3 - _4bz * q1) * eM3;
    float s2 = -_2q0 * eA1 + _2q3 * eA2 - 4.0f * q2 * eA3 + (-_4bx * q2 - _2bz * q0) * eM1 + (_2bx * q1 + _2bz * q3) * eM2 + (_2bx * q0 - _4bz * q2) * eM3;
    float s3 =  _2q1 * eA1 + _2q2 * eA2 + (-_4bx * q3 + _2bz * q1) * eM1 + (-_2bx * q0 + _2bz * q2) * eM2 + _2bx * q1 * eM3;

    float sn = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (sn > 1e-9f) {
      s0 /= sn; s1 /= sn; s2 /= sn; s3 /= sn;
      qDot1 -= beta * s0; qDot2 -= beta * s1; qDot3 -= beta * s2; qDot4 -= beta * s3;
    }
  }

  q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;
  float qn = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= qn; q1 /= qn; q2 /= qn; q3 /= qn;
}

// body->earth rotation row 3 (gravity direction in body) and others, from q
static Rot getRot() {
  Rot r;
  r.r11 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  r.r21 = 2.0f * (q1 * q2 + q0 * q3);
  r.r31 = 2.0f * (q1 * q3 - q0 * q2);
  r.r32 = 2.0f * (q2 * q3 + q0 * q1);
  r.r33 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  return r;
}

// ------------------------------ accel 6-position calibration ------------------------------
// Works on RAW chip-frame accel (before accOff/accScale and before imuToBody), so it is independent of the axis mapping.
// 6 positions, each one chip axis pointing straight up (+) or straight down (-). ~1 s of samples per position.
// Per axis: off = (hi + lo) / 2, scale = 2 / (hi - lo)  -> 1 g reads exactly 1 g on both sides.
static const uint8_t ACC_AXIS[6] = { 1, 1, 0, 0, 2, 2 };            // chip axis that points up/down in each position (0=X,1=Y,2=Z)
static const int8_t  ACC_SIGN[6] = { 1, -1, 1, -1, 1, -1 };         // +1 = that axis points up, -1 = down
static const uint16_t ACC_N = 200;                                  // samples per position (200 Hz -> 1 s)

static bool     accCal = false, accSampling = false;
static uint8_t  accStep = 0;
static uint16_t accCnt = 0;
static float    accSum[3], accDMin, accDMax;
static float    accPos[6][3];                                       // averaged raw accel per position, in g
static volatile float accLast[3];                                   // last captured average (core 0 prints it)

static void accSampleStart() {
  accSum[0] = accSum[1] = accSum[2] = 0;
  accDMin = 1e9f; accDMax = -1e9f;
  accCnt = 0;
  accSampling = true;
}

// x,y,z = raw chip-frame accel in g. Called from stepImu() at 200 Hz while accSampling.
static void accSample(float x, float y, float z) {
  accSum[0] += x; accSum[1] += y; accSum[2] += z;
  uint8_t ax = ACC_AXIS[accStep];
  float d = (ax == 0) ? x : (ax == 1) ? y : z;
  if (d < accDMin) accDMin = d;
  if (d > accDMax) accDMax = d;
  if (++accCnt < ACC_N) return;

  accSampling = false;
  float av[3] = { accSum[0] / ACC_N, accSum[1] / ACC_N, accSum[2] / ACC_N };
  for (int i = 0; i < 3; i++) accLast[i] = av[i];

  // accept only if the expected axis is ~+-1 g, the other two are small, and it stayed still
  bool ok = (av[ax] * ACC_SIGN[accStep] > 0.8f) && ((accDMax - accDMin) < 0.08f);
  for (int i = 0; i < 3; i++) if (i != ax && fabsf(av[i]) > 0.3f) ok = false;
  if (!ok) { g_accEvent = 3; return; }                 // same position again

  for (int i = 0; i < 3; i++) accPos[accStep][i] = av[i];
  accStep++;
  g_accStep = accStep;
  if (accStep < 6) { g_accEvent = 2; return; }

  float hi[3] = {0, 0, 0}, lo[3] = {0, 0, 0};
  for (int s = 0; s < 6; s++) {
    uint8_t a = ACC_AXIS[s];
    if (ACC_SIGN[s] > 0) hi[a] = accPos[s][a]; else lo[a] = accPos[s][a];
  }
  for (int a = 0; a < 3; a++) {
    float span = hi[a] - lo[a];
    accOff[a] = 0.5f * (hi[a] + lo[a]);
    accScale[a] = (span > 0.5f) ? (2.0f / span) : 1.0f;
  }
  accCal = false;
  g_accActive = false;
  g_accEvent = 4;
}

// ------------------------------ level calibration (roll/pitch trim) ------------------------------
// Averages published roll/pitch (before trim) for ~1 s while the robot sits on a flat surface.
static bool     levelling = false;
static uint16_t lvN = 0;
static float    lvR = 0, lvP = 0;

// ------------------------------ IMU step ------------------------------
static void stepImu(float dt) {
  uint8_t b[14];
  if (!readRegs(MPU_ADDR, 0x3B, b, 14)) { i2cErr++; return; }
  int16_t rax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ray = (int16_t)((b[2] << 8) | b[3]);
  int16_t raz = (int16_t)((b[4] << 8) | b[5]);
  int16_t rgx = (int16_t)((b[8] << 8) | b[9]);
  int16_t rgy = (int16_t)((b[10] << 8) | b[11]);
  int16_t rgz = (int16_t)((b[12] << 8) | b[13]);

  if (accSampling) accSample(rax / 8192.0f, ray / 8192.0f, raz / 8192.0f);

  const float ak = G0 / 8192.0f;
  const float gk = DEG2RAD / 65.5f;
  float cax = (rax * ak - accOff[0] * G0) * accScale[0];
  float cay = (ray * ak - accOff[1] * G0) * accScale[1];
  float caz = (raz * ak - accOff[2] * G0) * accScale[2];
  float cgx = rgx * gk - gyroBias[0], cgy = rgy * gk - gyroBias[1], cgz = rgz * gk - gyroBias[2];

  float gF[3];
  imuToBody(cax, cay, caz, aF[0], aF[1], aF[2]);
  imuToBody(cgx, cgy, cgz, gF[0], gF[1], gF[2]);
  gyroYawRate = -gF[2] * RAD2DEG;

  beta = (millis() - tStart < CONVERGE_MS) ? BETA_START : BETA_RUN;
  if (magValid) {
    float mk = g_magUse ? 1.0f : 0.0f;
    madgwick(gF[0], gF[1], gF[2], aF[0], aF[1], aF[2], mF[0] * mk, mF[1] * mk, mF[2] * mk, dt);
  }

  // vertical channel: predict with gravity-removed earth-up accel
  Rot r = getRot();
  float aUp = r.r31 * aF[0] + r.r32 * aF[1] + r.r33 * aF[2] - G0;
  fAlt += fVz * dt + 0.5f * aUp * dt * dt;
  fVz  += aUp * dt;
}

// ------------------------------ publish IMU -> shared vars ------------------------------
static void publishImu() {
  Rot r = getRot();

  float roll  = atan2f(r.r32, r.r33) * RAD2DEG;                       // right side down +
  float s     = r.r31; if (s > 1) s = 1; if (s < -1) s = -1;
  float pitch = asinf(s) * RAD2DEG;                                   // nose up +
  float yaw   = -atan2f(r.r21, r.r11) * RAD2DEG + DECLINATION_DEG;    // clockwise from north
  while (yaw < 0) yaw += 360.0f;
  while (yaw >= 360.0f) yaw -= 360.0f;

  if (levelling) {
    lvR += roll; lvP += pitch;
    if (++lvN >= 200) {
      rollTrim = lvR / lvN; pitchTrim = lvP / lvN;
      levelling = false;
      g_lvlEvent = 1;
    }
  }
  roll -= rollTrim; pitch -= pitchTrim;
  roll=-roll;

  // accel in your frame (x right, y up, z fwd): raw specific force and gravity-removed
  float ux, uy, uz;
  bodyToUser(aF[0], aF[1], aF[2], ux, uy, uz);
  float lF[3] = { aF[0] - G0 * r.r31, aF[1] - G0 * r.r32, aF[2] - G0 * r.r33 };
  float lx, ly, lz;
  bodyToUser(lF[0], lF[1], lF[2], lx, ly, lz);

  g_roll = roll; g_pitch = pitch; g_yaw = yaw;
  g_acc[0] = ux; g_acc[1] = uy; g_acc[2] = uz;
  g_lin[0] = lx; g_lin[1] = ly; g_lin[2] = lz;
  g_alt = fAlt;  g_vz = fVz;
  g_tempC = bTempC; g_pressPa = bPa;
  g_magValid = magValid;
  g_imuSeq = g_imuSeq + 1;
}

// ------------------------------ VL53L0X via TCA9548A ------------------------------
static VL53L0X tof[TOF_N];
static bool    tofAlive[TOF_N];
static int8_t  curCh = -1;

static bool tcaSelect(uint8_t ch) {
  if ((int8_t)ch == curCh) return true;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write((uint8_t)(1u << ch));
  if (Wire.endTransmission() != 0) return false;
  curCh = ch;
  return true;
}

// polls ONE sensor per call (round-robin over the alive ones), never waits on a measurement
static void stepTof() {
  static uint8_t k = 0;
  for (uint8_t n = 0; n < TOF_N; n++) {
    uint8_t i = (uint8_t)((k + n) % TOF_N);
    if (!tofAlive[i]) continue;
    k = (uint8_t)((i + 1) % TOF_N);
    if (!tcaSelect(TOF_CH[i])) return;
    // new sample ready? (bits 2:0 of RESULT_INTERRUPT_STATUS)
    if (tof[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) {
      uint16_t d = tof[i].readRangeContinuousMillimeters();   // data is ready, returns immediately
      g_lidar[i] = tof[i].timeoutOccurred() ? -1 : (int16_t)d;
      static float   emaV[TOF_N];
      static uint8_t emaBad[TOF_N] = {0};
      static bool    emaInit[TOF_N] = {false};
      if (g_lidar[i] < 0) {
        if (emaBad[i] < 255) emaBad[i]++;
        if (emaBad[i] >= LIDAR_INVALID_MAX) { emaInit[i] = false; g_lidarF[i] = -1.0f; }
      } else {
        float s = (float)g_lidar[i];
        if (s > LIDAR_F_CAP_MM) s = LIDAR_F_CAP_MM;
        emaV[i] = emaInit[i] ? emaV[i] + LIDAR_EMA_ALPHA * (s - emaV[i]) : s;
        emaInit[i] = true; emaBad[i] = 0;
        g_lidarF[i] = emaV[i];
      }
      g_lidarSeq = g_lidarSeq + 1;
      g_lidarSeqI[i] = g_lidarSeqI[i] + 1;
    }
    return;
  }
}

// ------------------------------ init / scheduler ------------------------------
static void sensorsInit() {
  Wire.setSDA(SENS_SDA_PIN);
  Wire.setSCL(SENS_SCL_PIN);
  Wire.setClock(SENS_I2C_HZ);
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  g_mpuOk = mpuInit();
  delay(20);
  g_qmcOk = qmcInit();
  g_bmpOk = bmpInit();
  delay(TIME_WAIT_BOOT);
  digitalWrite(LED_BUILTIN, HIGH);
  mpuCalibGyro();                              // ~1.2 s, keep the robot still at power-up
  digitalWrite(LED_BUILTIN, LOW);

  uint8_t mask = 0;
  for (uint8_t i = 0; i < TOF_N; i++) {
    tofAlive[i] = false;
    if (!tcaSelect(TOF_CH[i])) continue;
    tof[i].setTimeout(100);
    if (!tof[i].init()) continue;
    tof[i].setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
    tof[i].startContinuous();                  // back-to-back continuous ranging
    tofAlive[i] = true;
    mask |= (uint8_t)(1u << i);
  }
  g_tofMask = mask;

  tStart = millis();
  tImu = micros();
  g_sensorsReady = true;
}

// one small job per call, priority: IMU (200 Hz) > mag (100 Hz) > baro state machine > one ToF poll
static void sensorsStep() {
  // default: once the heading has converged after boot, hold it (unless a hold command was already given)
  static bool bootHoldDone = false;
  if (!bootHoldDone && magValid && g_imuSeq > 0 && (uint32_t)(millis() - tStart) > CONVERGE_MS + 500) {
    bootHoldDone = true;
    if (!g_yawHoldSet) g_yawHold = g_yaw;
  }
  if (g_cmdZero)   { g_cmdZero = false; nRef = 0; pSum = 0; fAlt = 0; fVz = 0; }
  if (g_cmdMagCal) { g_cmdMagCal = false; if (calibrating) magCalFinish(); else magCalStart(); }
  if (g_cmdLevel)  { g_cmdLevel = false; lvR = 0; lvP = 0; lvN = 0; levelling = true; }
  uint8_t ac = g_accCmd;
  if (ac) {
    g_accCmd = 0;
    if (ac == 1) {                                       // start / abort
      if (accCal) { accCal = false; accSampling = false; g_accActive = false; g_accEvent = 5; }
      else { accCal = true; accSampling = false; accStep = 0; g_accStep = 0; g_accActive = true; g_accEvent = 1; }
    } else if (ac == 2 && accCal && !accSampling) {      // capture current position
      accSampleStart();
    }
  }

  uint32_t nowU = micros();
  if ((uint32_t)(nowU - tImu) >= 5000) {
    float dt = (nowU - tImu) * 1e-6f;
    tImu = nowU;
    stepImu(dt);
    publishImu();
    return;
  }
  stepTof();
  if (millis() - tMag >= 10) { stepMag(); return; }
  if (g_bmpOk && stepBaro()) return;
  
}

// ===================== VELOCITY LAYER (core 1) =====================
// setVelocities(vx, vy, vr) -> mecanum inverse kinematics -> wheel rpm setpoints (g_setpoint) for the wheel PIDs.
// Rotation is closed on the gyro yaw rate, so it holds straight when vr = 0 and tracks vr when turning.
// Also computes the measured body velocities from the wheel encoders (g_velMeas).
// RC is read on core 0 and just calls setVelocities().
static const float TWO_PI_F = 6.283185307f;
static float yawInteg = 0.0f;
static float angInteg = 0.0f;
static bool  angSettled = false;

static void velocityStep(float dt) {
  const float ksx = K_SLIP_STRAFE, ksy = K_SLIP_FWD;
  float vx = g_velCmd[0];                                        // m/s (replaced by the auto strafe while it is on)
  float vy = g_velCmd[1];                                        // m/s
  float vr = g_velCmd[2];                                        // deg/s (replaced by the angle loop while yaw hold is on)
  const float k = ROBOT_LX_M + ROBOT_LY_M;                       // m
  const float rpmPerMs = 60.0f / (TWO_PI_F * WHEEL_RADIUS_M);    // wheel rpm per m/s of rim speed

  // measured body velocities from the wheel encoders (forward kinematics)
  float wv[N_MOTORS];
  for (int i = 0; i < N_MOTORS; i++) wv[i] = g_measured[i] * WHEEL_DIR[i] / rpmPerMs;
  g_velMeas[0] = ksx * 0.25f * (wv[0] - wv[1] - wv[2] + wv[3]);            // vx  m/s
  g_velMeas[1] = ksy * 0.25f * (wv[0] + wv[1] + wv[2] + wv[3]);            // vy  m/s
  g_velMeas[2] = 0.25f * (wv[0] - wv[1] + wv[2] - wv[3]) / k * RAD2DEG;    // vr  deg/s

  g_posX += g_velMeas[0] * dt;   // m from start (+ = right)
  g_posY += g_velMeas[1] * dt;   // m from start (+ = forward)

  // auto strafe from the side lidars (P only), overrides vx. no wall -> 0, one wall -> hold WALL_HOLD_MM, both -> centre
  const float cmdMag = hypotf(g_velCmd[0], g_velCmd[1]);
  if (g_autoStrafe && cmdMag > 1e-3f) {
    // axis from the COMMANDED velocity: strafing -> correct vy with front/back, otherwise correct vx with left/right
    const bool  travelX = fabsf(g_velCmd[0]) > fabsf(g_velCmd[1]);
    const int   sNeg = travelX ? LIDAR_BACK  : LIDAR_LEFT;
    const int   sPos = travelX ? LIDAR_FRONT : LIDAR_RIGHT;
    const float holdMm = travelX ? WALL_HOLD_FB_MM : WALL_HOLD_MM;
    const float dn = g_lidarF[sNeg], dp = g_lidarF[sPos];
    const bool wn = (dn > 0.0f && dn < WALL_SEEN_MM);
    const bool wp = (dp > 0.0f && dp < WALL_SEEN_MM);
    float e = 0.0f;                                              // mm, + = move right (y travel) / forward (x travel)
    if (wn && wp)    e = 0.5f * (dp - dn);
    else if (wn)     e = holdMm - dn;
    else if (wp)     e = dp - holdMm;
    if (e > STRAFE_DB_MM)       e -= STRAFE_DB_MM;
    else if (e < -STRAFE_DB_MM) e += STRAFE_DB_MM;
    else                        e = 0.0f;
    const float corr = constrain(STRAFE_KP * e, -STRAFE_AUTO_MAX, STRAFE_AUTO_MAX);
    if (travelX) vy = corr; else vx = corr;
  }
  // collision guard: if a wall on the travel axis is closer than 50 mm in that direction, zero that axis
  const float df = (float)g_lidar[LIDAR_FRONT];
  const float db = (float)g_lidar[LIDAR_BACK];
  const float dl = (float)g_lidar[LIDAR_LEFT];
  const float dr = (float)g_lidar[LIDAR_RIGHT];

  if ((vy > 0.0f && df > 0.0f && df < COLLISION_STOP_MM) ||
      (vy < 0.0f && db > 0.0f && db < COLLISION_STOP_MM)) {
    vy = 0.0f;
  }
  if ((vx > 0.0f && dr > 0.0f && dr < (COLLISION_STOP_MM+10)) ||
      (vx < 0.0f && dl > 0.0f && dl < (COLLISION_STOP_MM+10))) {
    vx = 0.0f;
  }

  // angle hold: outer loop -> yaw-rate target (overrides vr)
  const float hold = g_yawHold;
  const bool holdOn = (hold >= 0.0f) && g_mpuOk && g_magValid;
  if (holdOn) {
    float e = hold - g_yaw;
    while (e > 180.0f)  e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    if (fabsf(e) < ANG_DB_IN) angSettled = true;
    else if (fabsf(e) > ANG_DB_OUT) angSettled = false;
    angInteg = constrain(angInteg + ANG_KI * e * dt, -ANG_I_LIM, ANG_I_LIM);
    vr = constrain(ANG_KP * e + angInteg - ANG_KD * gyroYawRate, -ANG_RATE_MAX, ANG_RATE_MAX);
  } else {
    angInteg = 0.0f;
    angSettled = false;
  }

  // nothing commanded: exactly zero so the wheel controllers brake
  if (fabsf(vx) < 1e-4f && fabsf(vy) < 1e-4f && (holdOn ? angSettled : fabsf(vr) < 0.05f)) {
    yawInteg = 0.0f;
    angInteg = 0.0f;
    for (int i = 0; i < N_MOTORS; i++) g_setpoint[i] = 0.0f;
    return;
  }

  // yaw-rate PI on the gyro (deg/s, adds to the rotation command)
  float wr = vr + STRAFE_YAW_FF * vx;
  if (YAW_PID_EN && g_mpuOk) {
    float err = vr - gyroYawRate;
    yawInteg = constrain(yawInteg + YAW_KI * err * dt, -YAW_I_LIM, YAW_I_LIM);
    wr = vr + STRAFE_YAW_FF * vx + YAW_KP * err + yawInteg;
  }
  const float wrRad = wr * DEG2RAD;
  const float vxw = vx / ksx;   // wheel-side rim speed needed for the desired actual speed
  const float vyw = vy / ksy;

  float rpm[N_MOTORS] = {
    (vyw + vxw + k * wrRad) * rpmPerMs,   // FL
    (vyw - vxw - k * wrRad) * rpmPerMs,   // FR
    (vyw - vxw + k * wrRad) * rpmPerMs,   // BL
    (vyw + vxw - k * wrRad) * rpmPerMs,   // BR
  };
  // inverse kinematics: wheel rim speed (m/s) -> rpm, + = robot forward before WHEEL_DIR

  // scale everything down together if one wheel would exceed the limit (keeps the motion direction)
  const float lim = (DRIVE_MAX_RPM < RPM_MAX) ? DRIVE_MAX_RPM : RPM_MAX;
  float mx = 0.0f;
  for (int i = 0; i < N_MOTORS; i++) mx = max(mx, fabsf(rpm[i]));
  float sc = (mx > lim) ? (lim / mx) : 1.0f;

  for (int i = 0; i < N_MOTORS; i++) g_setpoint[i] = constrain(rpm[i] * sc * WHEEL_DIR[i], -RPM_MAX, RPM_MAX);
}

// called from loop1() on every PID tick
static void velocityTask(uint32_t nowU) {
  static uint32_t tVel = 0;
  if (g_velActive) {
    if ((uint32_t)(nowU - tVel) >= VEL_PERIOD_US) {
      float dt = (nowU - tVel) * 1e-6f;
      if (dt > 0.05f) dt = 0.05f;                    // first run after being idle
      tVel = nowU;
      velocityStep(dt);
    }
  } else {
    yawInteg = 0.0f;
    angInteg = 0.0f;
    angSettled = false;
  }
}

// ===================== CORE 1: PID =====================
static int32_t lastCount[N_MOTORS];
static float   rpmFilt[N_MOTORS], integ[N_MOTORS], prevMeas[N_MOTORS], prevSp[N_MOTORS];

void setup1() {
  analogWriteFreq(PWM_FREQ);
  analogWriteRange(255);
  for (int i = 0; i < N_MOTORS; i++) {
    pinMode(MOT[i].cw, OUTPUT);  digitalWrite(MOT[i].cw, LOW);
    pinMode(MOT[i].ccw, OUTPUT); digitalWrite(MOT[i].ccw, LOW);
    pinMode(MOT[i].en, OUTPUT);  analogWrite(MOT[i].en, 0);
  }
  encInit();
  sensorsInit();
  for (int i = 0; i < N_MOTORS; i++) lastCount[i] = encCount(i);
}

void loop1() {
  static uint32_t tLast = micros();
  uint32_t now = micros();
  if ((uint32_t)(now - tLast) < PID_PERIOD_US) {
    // idle time before the next PID tick: run one small sensor job, but only if it fits
    if (PID_PERIOD_US - (uint32_t)(now - tLast) > SENSOR_GUARD_US) sensorsStep();
    return;
  }
  loopHz = 1000000UL / (now - tLast);
  float dt = (now - tLast) * 1e-6f;
  tLast = now;

  velocityTask(now);

  for (int i = 0; i < N_MOTORS; i++) {
    // --- measure ---
    int32_t c = encCount(i);
    int32_t d = (int32_t)((uint32_t)c - (uint32_t)lastCount[i]);
    lastCount[i] = c;
    float rpmRaw = ((float)d / COUNTS_PER_REV) / dt * 60.0f;
    rpmFilt[i] += RPM_FILT_ALPHA * (rpmRaw - rpmFilt[i]);
    g_measured[i] = rpmFilt[i];

    // --- control ---
    float sp = constrain(g_setpoint[i], -RPM_MAX, RPM_MAX);
    float duty = 0.0f;
    bool brake = false;

    if (fabsf(sp) < 0.5f) {
      integ[i] = 0.0f;
      brake = true;
    } else {
      if (sp * prevSp[i] <= 0.0f) integ[i] = 0.0f;       // setpoint just started / reversed

      float err = sp - rpmFilt[i];

      // overshoot: moving the same direction as sp, but faster than sp -> active brake
      if (rpmFilt[i] * sp > 0.0f && fabsf(rpmFilt[i]) > fabsf(sp) + BRAKE_MARGIN_RPM) brake = true;

      // feedforward: linear map |rpm| 0..RPM_MAX -> MIN_DUTY..255
      float ff = copysignf(MIN_DUTY + (fabsf(sp) / RPM_MAX) * (255.0f - MIN_DUTY), sp);

      integ[i] += KI * err * dt;
      integ[i]  = constrain(integ[i], -INTEG_LIM, INTEG_LIM);
      float dTerm = -KD * (rpmFilt[i] - prevMeas[i]) / dt;  // on measurement, no kick

      duty = ff + KP * err + integ[i] + dTerm;
      // never drive against the setpoint direction
      duty = (sp > 0) ? constrain(duty, 0.0f, 255.0f) : constrain(duty, -255.0f, 0.0f);

      if (brake) integ[i] = 0.0f;                        // don't wind up while braking
    }
    prevSp[i]   = sp;
    prevMeas[i] = rpmFilt[i];
    driveMotor(i, duty, brake);
  }
}

// ===================== CORE 0: serial test =====================
static const char *ACC_PROMPT[6] = {
  "+Y up  (robot level, upright)",
  "-Y up  (robot upside down)",
  "+X up  (robot on its LEFT side, right side facing up)",
  "-X up  (robot on its RIGHT side, left side facing up)",
  "+Z up  (nose down, back facing up)",
  "-Z up  (nose up, back facing down)",
};

// RC sticks -> velocity targets (runs on core 0). Core 1 does the kinematics.
// NOT CALLED any more: the RC now commands cell moves (rcFlick() in the MAZE section). Kept for reference.
static void rcControl() {
  uint16_t a = rcRead(RC_CH_STRAFE);
  uint16_t b = rcRead(RC_CH_FWD);
  if (!a || !b) { setVelocities(0, 0, 0); return; }   // failsafe: signal lost -> stop
  setVelocities(SIGN_STRAFE * rcNorm(a) * RC_MAX_VX_MS,
                SIGN_FWD    * rcNorm(b) * RC_MAX_VY_MS,
                0);
}

// ===================== MAZE: ToF-only cell moves + wall map (core 0) =====================
// MAZE_SECTION_BEGIN
// The robot never rotates: the heading is held from the BOOTSEL press, so N = forward at start, E = right, S = back, W = left.
// Position and map come ONLY from the ToFs (no odometry). The velocity layer on core 1 (lidar centering, yaw hold,
// collision guard) is NOT changed: a move only commands the speed along the direction of travel via setVelocities().
//
//   IDLE -> SENSE (standstill window) -> GATE -> DRIVE -> SETTLE (window, up to 2 creep corrections) -> MAP -> IDLE
//   A refused gate or an abort goes back to IDLE; position and map stay unchanged.
//
// RC: flick the pitch stick (CH2) up/down = N/S, the roll stick (CH1) right/left = E/W. The move fires when the stick
//     comes back to centre: peak deflection >= RC_FLICK_MIN -> 1 cell, >= RC_FLICK_FULL -> 2 cells.
// Map: maze[y][x], x grows E, y grows N, start = (START_X, START_Y).
//     bits: 1 = visited, 2 = N wall (front), 4 = E (right), 8 = S (back), 16 = W (left). 0 = unvisited.
//     A wall is written to both neighbours, so an unvisited cell can already hold a wall seen from its neighbour.
// One CSV line per move (header: see logHeader()).

#define MAZE_N 13                                   // grid is MAZE_N x MAZE_N cells
static const int8_t START_X = 0;                    // start cell. (0,0) = SW corner: moves N / E go into the grid
static const int8_t START_Y = 0;

// ---- geometry / calibration: measure, then paste the values printed by "ccal" / "tcal" ----
static float CELL_P_MM   = 225.0f;                          // cell pitch, wall centre to wall centre
static float C_MM[4]     = { 61.8f, 57.9f, 96.2f, 97.0f };  // reading when centred in a closed cell: front, back, left, right
static float TOF_GAIN[4] = { 1.0f, 1.0f, 1.0f, 1.0f };      // per-sensor ToF calibration (maze layer only): mm = gain * raw + off
static float TOF_OFF[4]  = { 0.0f, 0.0f, 0.0f, 0.0f };

// ---- sample window / classification ----
#define WIN_N 8                                     // fresh samples per sensor in a standstill window
#define WIN_N_FAR 16                                // settle window when the nearest wall along the move axis is far
static const uint8_t  WIN_MIN        = 4;           // fewer kept samples -> sensor invalid (counts as a wall / blocked)
static const float    WIN_OUTLIER_MM = 15.0f;       // samples further than this from the median are dropped ...
static const float    WIN_OUTLIER_FRAC = 0.03f;     // ... or 3 % of the median if larger (ToF noise grows with range)
static const float    SETTLE_FAR_MM  = 400.0f;      // nearest along-axis reading above this -> settle window of WIN_N_FAR
static const float    SCATTER_MARGIN_MM = 20.0f;    // window too scattered to be valid, but every sample beyond the cell boundary (C + P/2 + this)
                                                    // -> no wall in this cell on that side: open / far, not a wall
static int            OFFSET_K_MAX   = 2;           // start offset (e0) and settle check only use walls up to this many cells away. 0 = only the
                                                    // walls of the current cell (your ToFs read 30-50 mm short / unsteady at 1 cell). Raise to 1..3
                                                    // once "win" shows walls 1 cell away reading C + 225 within a few mm
static const uint32_t WIN_TIMEOUT_MS = 600;
static const int16_t  TOF_FAR_RAW    = 1400;        // raw reading >= this (incl. 8190 "no target") = far: clear, no obstacle
static const int16_t  TOF_MIN_RAW    = 20;          // raw reading below this (the 0s the VL53L0X gives with no target) = invalid
static const uint8_t  FAR_MIN_N      = 2;           // window with at least this many 8190s and no steady group of readings = clear
static const float    TOF_FAR_MM     = 9999.0f;     // value stored for a far sample
static float          TRUST_MAX_MM   = 700.0f;      // reliable ToF range: a reading is used for target / progress only below this, and a
                                                    // move needs a reference that stays below it. Raise it if "win" shows steady readings
                                                    // further out (a wall 3 cells away reads ~735 front/back, ~772 left/right)
static float          TRUST_FAR_MM   = 1200.0f;     // no reference below TRUST_MAX_MM (walls 3+ cells away both ways): a reading up to this
                                                    // is still used if the gate window was steady (no 8190, at most 2 samples off) and the
                                                    // sensor has already been seen reading steadily that far (learned per sensor). 0 = off
static const int      FREE_MAX       = 3;           // max cells per move, and max k used for the start offset

// ---- drive ----
static float MV_KP     = 3.0f;                      // m/s per m of remaining distance (3 -> full speed beyond ~80 mm)
static float MV_VMAX   = 0.4f;                     // m/s
static float MV_VCREEP = 0.05f;                     // m/s, minimum speed outside the stop band
static float MV_ACC    = 1.6f;                      // m/s^2, limits speeding up only (stopping is never limited)
static float MV_VMAX_EW   = 0.25f;                  // m/s max for E/W (strafe) moves: at 0.4 the robot drifted ~30 mm N/S and clipped wall ends
static float MV_VCREEP_EW = 0.07f;                  // m/s creep for E/W moves (strafing stalls at very low wheel speed)
static const float    MV_VCORR        = 0.10f;      // m/s max during a settle correction
static const float    MV_TOL_MM       = 3.0f;       // stop band: enter within this (speed 0) ...
static const float    MV_TOL_EXIT_MM  = 10.0f;      // ... leave only beyond this (2 updates in a row); settle does the fine check
static const uint32_t MV_DONE_MS      = 100;        // in the band this long -> stopped
static const uint32_t MV_NOSENS_MS    = 300;        // no valid progress sensor this long -> abort
static const uint32_t MV_TICK_MS      = 10;         // speed command rate
static const uint32_t MV_TIMEOUT_MS   = 5000;       // + MV_TIMEOUT_CELL_MS per cell
static const uint32_t MV_TIMEOUT_CELL_MS = 1500;
static const uint32_t CORR_TIMEOUT_MS = 1500;
static const float    FUSE_R0_MM      = 300.0f;     // progress fusion weight 1 / (1 + (reading / R0)^2)
static const float    SUM_DRIFT_MM    = 12.0f;      // (A + B) drifting more than this from A0 + B0 -> drop one sensor
static const float    JUMP_BASE_MM    = 20.0f;      // a progress sample jumping more than this (+ 1.5 * vmax * dt) is ignored
static const float    E0_AGREE_MM     = 8.0f;       // start offsets from ahead / behind averaged if they agree this well
static const float    CLOSE_MM        = 3.0f;       // ahead reading below C + this -> stop (arrived if a wall was expected there, else abort)
static const uint32_t SAMPLE_STALE_MS = 150;
static const float    SPIKE_MM        = 25.0f;      // a sample this far (+ 3 %) from the median of its last 3 counts as scatter
static const float    AB_ALPHA        = 0.30f;      // alpha-beta filter on the fused progress (ToF only, runs every MV_TICK_MS): position gain
static const float    AB_BETA         = 0.03f;      // ... speed gain

// ---- settle ----
static const uint32_t SETTLE_MS   = 400;            // wait after stopping before the verify window
static const float    CORR_MIN_MM = 4.0f;           // |e| above this (and below CORR_MAX_MM) -> creep correction
static const float    CORR_MAX_MM = 40.0f;          // |e| at/above this -> no correction, logged SUSPECT
static const uint8_t  CORR_MAX_N  = 2;              // corrections per move

// ---- side walls of intermediate cells (multi-cell runs) ----
static const float    SIDE_WIN_MM  = 40.0f;         // +- window around the cell centre crossing
static const float    SIDE_LEAD_MM = 20.0f;         // side sensors sit this far ahead (N) of the robot centre
static const float    SIDE_FRAC    = 0.6f;          // wall if at least this fraction of samples is below the threshold
static const uint16_t SIDE_MIN_N   = 3;             // fewer samples -> that edge is left as it was

// ---- flood-fill solver ----
static int8_t   GOAL_X = MAZE_N - 1;                // goal cell (default: corner opposite the start). "goal x y" changes it
static int8_t   GOAL_Y = MAZE_N - 1;
static const int      RAY_K_MAX      = 2;           // map what each ToF sees along its line up to this many cells (walls 2 cells away: 510 / 547 mm)
static const uint8_t  RUN_MAX        = 3;           // longest straight run per command (cells known open ahead)
static const uint16_t SOLVE_MAX_CMDS = 600;         // give up after this many commands
static const uint8_t  SOLVE_MAX_REFUSE = 16;        // refused moves in a row (no successful move between) before giving up
static const uint32_t WIN_REUSE_MS   = 1500;        // next move within this time of the last standstill window (no motion since) reuses it
static const uint32_t LONG_PRESS_MS  = 1500;        // BOOTSEL held this long while stopped = new start

// ---- RC flicks ----
static const float RC_FLICK_CENTER = 0.15f;         // |stick| below this = centred
static const float RC_FLICK_MIN    = 0.50f;         // peak deflection for a 1-cell move
static const float RC_FLICK_FULL   = 0.90f;         // peak deflection for a 2-cell move
static const bool  RC_FLICK_N2     = true;          // false = every flick is 1 cell

// ---- directions / sensors (robot never rotates: front = N, right = E, back = S, left = W) ----
#define D_N 0
#define D_E 1
#define D_S 2
#define D_W 3
#define W_VISITED 1
static const char    DIR_CH[4]      = { 'N', 'E', 'S', 'W' };
static const int8_t  DIR_DX[4]      = { 0, 1, 0, -1 };
static const int8_t  DIR_DY[4]      = { 1, 0, -1, 0 };
static const uint8_t DIR_BIT[4]     = { 2, 4, 8, 16 };
static const uint8_t DIR_OPP[4]     = { D_S, D_W, D_N, D_E };
static const uint8_t DIR_AHEAD[4]   = { LIDAR_FRONT, LIDAR_RIGHT, LIDAR_BACK,  LIDAR_LEFT  };
static const uint8_t DIR_BEHIND[4]  = { LIDAR_BACK,  LIDAR_LEFT,  LIDAR_FRONT, LIDAR_RIGHT };
static const uint8_t DIR_SIDE1[4]   = { LIDAR_LEFT,  LIDAR_FRONT, LIDAR_LEFT,  LIDAR_FRONT };
static const uint8_t DIR_SIDE2[4]   = { LIDAR_RIGHT, LIDAR_BACK,  LIDAR_RIGHT, LIDAR_BACK  };
static const float   DIR_SIDE_SHIFT[4] = { -SIDE_LEAD_MM, 0.0f, SIDE_LEAD_MM, 0.0f };  // N: sensors lead -> sample earlier
static const uint8_t SENSOR_DIR[4]  = { D_N, D_S, D_W, D_E };                         // wall seen by: front, back, left, right
static const char    SENSOR_CH[4]   = { 'F', 'B', 'L', 'R' };

// ---- map ----
static uint8_t maze[MAZE_N][MAZE_N];                // [y][x]
static uint8_t mzOpen[MAZE_N][MAZE_N];              // edges the robot has driven through (same bits): never turned into a wall
static uint8_t mzSeen[MAZE_N][MAZE_N];              // edges observed by any sensor (same bits): known, wall or open
static int8_t  mzX = START_X, mzY = START_Y;        // current cell, updated only after a successful settle

static bool inGrid(int x, int y) { return x >= 0 && y >= 0 && x < MAZE_N && y < MAZE_N; }
static bool edgeKnown(int x, int y, uint8_t d) {
  if (inGrid(x, y)) return (mzSeen[y][x] & DIR_BIT[d]) != 0;
  const int nx = x + DIR_DX[d], ny = y + DIR_DY[d];
  return inGrid(nx, ny) && (mzSeen[ny][nx] & DIR_BIT[DIR_OPP[d]]);
}
static void setEdge(int x, int y, uint8_t d, bool wall) {   // writes the edge into both neighbours
  const int nx = x + DIR_DX[d], ny = y + DIR_DY[d];
  const uint8_t b = DIR_BIT[d], ob = DIR_BIT[DIR_OPP[d]];
  if (wall && ((inGrid(x, y) && (mzOpen[y][x] & b)) || (inGrid(nx, ny) && (mzOpen[ny][nx] & ob)))) return;   // driven through = open
  if (inGrid(x, y))   { if (wall) maze[y][x]   |= b;  else maze[y][x]   &= (uint8_t)~b;  mzSeen[y][x]   |= b;  }
  if (inGrid(nx, ny)) { if (wall) maze[ny][nx] |= ob; else maze[ny][nx] &= (uint8_t)~ob; mzSeen[ny][nx] |= ob; }
}
static bool edgeOpenKnown(int x, int y, uint8_t d) { return edgeKnown(x, y, d) && inGrid(x, y) && !(maze[y][x] & DIR_BIT[d]); }
static void lockOpen(int x, int y, uint8_t d) {     // the robot drove through this edge: open for good
  setEdge(x, y, d, false);
  const int nx = x + DIR_DX[d], ny = y + DIR_DY[d];
  if (inGrid(x, y))   mzOpen[y][x]   |= DIR_BIT[d];
  if (inGrid(nx, ny)) mzOpen[ny][nx] |= DIR_BIT[DIR_OPP[d]];
}

// ---- ToF samples ----
// newest sample of sensor i if it is newer than *lastSeq. raw: mm, TOF_FAR_MM = far, -1 = invalid
static bool tofFresh(uint8_t i, uint32_t *lastSeq, float *raw) {
  uint32_t s1 = g_lidarSeqI[i];
  if (s1 == *lastSeq) return false;
  int16_t r = -1;
  for (uint8_t t = 0; t < 3; t++) {                 // re-read if core 1 published a newer sample meanwhile
    r = g_lidar[i];
    const uint32_t s2 = g_lidarSeqI[i];
    if (s2 == s1) break;
    s1 = s2;
  }
  *lastSeq = s1;
  *raw = (r < TOF_MIN_RAW) ? -1.0f : (r >= TOF_FAR_RAW) ? TOF_FAR_MM : (float)r;
  return true;
}
static float tofCal(uint8_t i, float raw) {         // keeps -1 (invalid) and TOF_FAR_MM (far) unchanged
  if (raw < 0.0f || raw >= TOF_FAR_MM) return raw;
  return TOF_GAIN[i] * raw + TOF_OFF[i];
}

// ---- standstill sample window: WIN_N fresh samples per sensor, median outlier rejection ----
static float    wnRaw[4][WIN_N_FAR];
static uint8_t  wnGot[4];
static uint8_t  wnN = WIN_N;                        // samples per sensor for the current window
static uint32_t wnSeq[4];
static uint32_t wnT0;
static uint32_t wnDoneT = 0;                        // when the last window completed
static bool     wnDone = false;                     // a completed standstill window is in the buffers
static bool     mzReuse = false;                    // SENSE_WIN: gate on the last window instead of a new one
static float    tofSteadyMax[4] = { 0, 0, 0, 0 };   // farthest distance each sensor has read steadily in a window (since power-up)
static float    wnRawAvg[4], wnAvg[4];              // raw / calibrated mean of the kept samples (TOF_FAR_MM = far, -1 = none)
static uint8_t  wnCnt[4];
static bool     wnOk[4];
static float    wnNear[4];                          // scattered far window: nearest sample (caps the free cells), else TOF_FAR_MM

static void winStart(uint8_t n = WIN_N) {
  wnN = (n > WIN_N_FAR) ? WIN_N_FAR : n;
  wnDone = false;
  for (uint8_t i = 0; i < 4; i++) { wnGot[i] = 0; wnSeq[i] = g_lidarSeqI[i]; }
  wnT0 = millis();
}
static void winReduce(uint8_t i) {
  float a[WIN_N_FAR];
  uint8_t n = 0;
  for (uint8_t k = 0; k < wnGot[i]; k++) if (wnRaw[i][k] >= 0.0f) a[n++] = wnRaw[i][k];
  wnRawAvg[i] = -1.0f; wnAvg[i] = -1.0f; wnCnt[i] = 0; wnOk[i] = false; wnNear[i] = TOF_FAR_MM;
  if (n == 0) return;
  for (uint8_t p = 1; p < n; p++) {                 // insertion sort
    const float v = a[p];
    int q = p - 1;
    while (q >= 0 && a[q] > v) { a[q + 1] = a[q]; q--; }
    a[q + 1] = v;
  }
  const float med = a[(n - 1) / 2];                 // lower median = a real sample
  const float band = fmaxf(WIN_OUTLIER_MM, WIN_OUTLIER_FRAC * med);
  float sum = 0.0f;
  uint8_t m = 0;
  for (uint8_t k = 0; k < n; k++) if (fabsf(a[k] - med) <= band) { sum += a[k]; m++; }
  wnCnt[i] = m;
  wnRawAvg[i] = sum / m;                            // m >= 1 (the median itself)
  wnAvg[i] = tofCal(i, wnRawAvg[i]);
  wnOk[i] = (m >= WIN_MIN);
  // 8190 = nothing in range. Between its 8190s a VL53L0X with no target also returns random values (200..800, 0), so
  // no steady group forms. A window with 8190s and no steady group of readings = clear (far), no cap on the free cells.
  // A real wall reads steadily (WIN_MIN samples agreeing) and still wins, 8190s or not.
  if (!wnOk[i]) {
    uint8_t nFar = 0;
    for (uint8_t k = 0; k < n; k++) if (a[k] >= TOF_FAR_MM) nFar++;
    if (nFar >= FAR_MIN_N) {
      wnRawAvg[i] = TOF_FAR_MM; wnAvg[i] = TOF_FAR_MM; wnCnt[i] = nFar; wnOk[i] = true;
      return;
    }
  }
  // VL53L0X near its range limit (or its cone catching side walls far down a corridor): readings scatter, so no WIN_MIN
  // of them agree. A wall inside the cell reads close and steady, so "enough samples, all far" = open / far, not a wall.
  // Stored as far (never used for target / progress); the nearest sample still caps the free cells (winFree).
  if (!wnOk[i] && n >= WIN_MIN && tofCal(i, a[0]) > C_MM[i] + 0.5f * CELL_P_MM + SCATTER_MARGIN_MM) {
    wnNear[i] = tofCal(i, a[0]);
    wnRawAvg[i] = TOF_FAR_MM; wnAvg[i] = TOF_FAR_MM; wnCnt[i] = n; wnOk[i] = true;
  }
}
// steady window: a real reading (no 8190 / far sample, at most 2 samples off the median group)
static bool winSteady(uint8_t i) {
  if (!wnOk[i] || !(wnAvg[i] < TOF_FAR_MM) || wnCnt[i] + 2 < wnGot[i]) return false;
  for (uint8_t k = 0; k < wnGot[i]; k++) if (wnRaw[i][k] >= TOF_FAR_MM) return false;
  return true;
}
static bool winPoll() {                             // true when the window is complete (or timed out)
  bool full = true;
  for (uint8_t i = 0; i < 4; i++) {
    float r;
    if (wnGot[i] < wnN && tofFresh(i, &wnSeq[i], &r)) wnRaw[i][wnGot[i]++] = r;
    if (wnGot[i] < wnN) full = false;
  }
  if (!full && (uint32_t)(millis() - wnT0) < WIN_TIMEOUT_MS * wnN / WIN_N) return false;
  for (uint8_t i = 0; i < 4; i++) {
    winReduce(i);
    if (winSteady(i) && wnAvg[i] < TRUST_FAR_MM && wnAvg[i] > tofSteadyMax[i]) tofSteadyMax[i] = wnAvg[i];
  }
  wnDone = true; wnDoneT = millis();
  return true;
}
static bool winWall(uint8_t i) { return !wnOk[i] || wnAvg[i] < C_MM[i] + 0.5f * CELL_P_MM; }   // invalid counts as a wall
static int  winK(uint8_t i) {                       // free cells before the wall seen by sensor i (round)
  const int k = (int)lroundf((wnAvg[i] - C_MM[i]) / CELL_P_MM);
  return (k < 0) ? 0 : k;
}
static int  winFree(uint8_t i) {
  if (!wnOk[i]) return 0;
  int k = winK(i);
  if (wnNear[i] < TOF_FAR_MM) {                     // scattered far window: be conservative, the nearest sample may be a wall
    int kn = (int)lroundf((wnNear[i] - C_MM[i]) / CELL_P_MM);
    if (kn < 0) kn = 0;
    if (kn < k) k = kn;
  }
  return (k > FREE_MAX) ? FREE_MAX : k;
}

// offset of the robot from its cell centre along dir (+ = already ahead of centre), from the nearest wall
// within FREE_MAX cells ahead or behind. Both usable and agreeing within E0_AGREE_MM -> averaged.
static char  offSrc = '-';                          // 'A' ahead, 'B' behind, '2' both, '-' none
static bool winOffset(uint8_t dir, float *e) {
  const uint8_t a = DIR_AHEAD[dir], b = DIR_BEHIND[dir];
  const bool okA = wnOk[a] && wnAvg[a] < TRUST_MAX_MM && winK(a) <= OFFSET_K_MAX;
  const bool okB = wnOk[b] && wnAvg[b] < TRUST_MAX_MM && winK(b) <= OFFSET_K_MAX;
  const int  kA = okA ? winK(a) : 99, kB = okB ? winK(b) : 99;
  const float eA = okA ? C_MM[a] + kA * CELL_P_MM - wnAvg[a] : 0.0f;
  const float eB = okB ? wnAvg[b] - C_MM[b] - kB * CELL_P_MM : 0.0f;
  if (okA && okB && fabsf(eA - eB) <= E0_AGREE_MM) { *e = 0.5f * (eA + eB); offSrc = '2'; return true; }
  if (okA && (!okB || kA < kB || (kA == kB && wnAvg[a] <= wnAvg[b]))) { *e = eA; offSrc = 'A'; return true; }
  if (okB) { *e = eB; offSrc = 'B'; return true; }
  *e = 0.0f; offSrc = '-';
  return false;
}

// ---- FSM ----
#define M_OFF         0
#define M_IDLE        1
#define M_START_WAIT  2
#define M_START_WIN   3
#define M_CAL_WIN     4
#define M_SENSE_WAIT  5
#define M_SENSE_WIN   6
#define M_DRIVE       7
#define M_SETTLE_WAIT 8
#define M_SETTLE_WIN  9
static uint8_t  mzState = M_OFF;
static uint32_t mzT = 0;                            // state timer
static uint32_t mzLastMotion = 0;                   // last time a non-zero speed was commanded (standstill check)
static uint8_t  mzWinReq = 0, mzWinArgI = 0;        // serial window request: 1 print, 2 ccal, 3 tcal point
static float    mzWinArgMm = 0.0f;
static bool     g_telOn = true;                     // 200 ms telemetry line on/off ("tel 0/1")
static bool     g_mzDbg = false;                    // drive debug lines ("mzdbg 0/1")

struct MazeMove {
  uint8_t  dir, nReq, nTgt, nCorr, closeA, closeB, outCnt, aHistN, used, settleN;
  uint8_t  badA, badB;                              // last samples of the ahead / behind reference: bit = out of range / invalid / scatter
  bool     sickA, sickB;                            // reference currently scattering -> not used until it is clean again
  bool     corr, useA, useB, inBand, wallAtDest, suspect;
  float    trust;                                   // reference range limit for this move: TRUST_MAX_MM, or TRUST_FAR_MM (gate found only a far one)
  int8_t   sx, sy;
  int      gateFree, kA, kB;
  float    A0, B0, e0, T, sFinal, eSettle;          // logged values (gate window / main drive)
  float    dA0, dB0;                                // baseline of the current drive (main or correction)
  float    tgt, s, sv, Anow, Bnow, vCmd, vmax;      // target / filtered progress (mm) / its rate (mm/ms) of the current drive, live readings, speed
  float    zSum; uint8_t zN;                        // fused progress measurements since the last tick
  float    aHist[3], bHist[3];                      // last 3 valid ahead / behind samples (progress uses their median)
  uint8_t  bHistN;
  uint32_t tReq, tDrive, tS, tA, tB, tBand, tTick, tDbg, timeoutMs;
  uint32_t seq[4];
  uint16_t sideTot[2][2], sideHit[2][2];            // [intermediate cell 1..2][side sensor 1..2]
  char     e0src;
  const char *gate;
};
static MazeMove mv;

static void mvSetSpeed(float v) {                   // v along the direction of travel (m/s); centering / yaw hold untouched
  float vx = 0.0f, vy = 0.0f;
  if (mv.dir == D_N) vy = v; else if (mv.dir == D_S) vy = -v; else if (mv.dir == D_E) vx = v; else vx = -v;
  setVelocities(vx, vy, 0.0f);
  if (v != 0.0f) mzLastMotion = millis();
}

static void logHeader() {
  Serial.println("MOVEHDR,sx,sy,dir,n,gate,free,A0,B0,kA,kB,e0,T,s,F,B,L,R,err,corr,ms,result,nDone,x,y");
}
static const char *mzLastResult = "";              // result of the last move request (for the solver)
static uint32_t     mzResultSeq = 0;
static void logMove(const char *result, bool finalWin, uint8_t nDone) {
  mzLastResult = result;
  mzResultSeq++;
  Serial.printf("MOVE,%d,%d,%c,%u,%s,%d,%.1f,%.1f,%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%lu,%s,%u,%d,%d\n",
                (int)mv.sx, (int)mv.sy, DIR_CH[mv.dir], (unsigned)mv.nReq, mv.gate, mv.gateFree,
                mv.A0, mv.B0, mv.kA, mv.kB, mv.e0, mv.T, mv.sFinal,
                finalWin ? wnAvg[0] : NAN, finalWin ? wnAvg[1] : NAN, finalWin ? wnAvg[2] : NAN, finalWin ? wnAvg[3] : NAN,
                mv.eSettle, (unsigned)mv.nCorr, (unsigned long)(millis() - mv.tReq), result, (unsigned)nDone, (int)mzX, (int)mzY);
}
static void warnInvalid() {
  for (uint8_t i = 0; i < 4; i++)
    if (!wnOk[i]) Serial.printf("WARN sensor %c invalid (%u good samples) -> blocked for moves, map edge not written\n", SENSOR_CH[i], (unsigned)wnCnt[i]);
}

// write the edges of cell (x,y) from the current (standstill) window; skipDir = edge known open (came from there), -1 = none.
// An invalid sensor writes nothing (that edge stays as it was / unknown); it only counts as blocked for the gate.
static void mapStandstill(int x, int y, int skipDir) {
  for (uint8_t i = 0; i < 4; i++) {
    const uint8_t d = SENSOR_DIR[i];
    if ((int)d == skipDir || !wnOk[i]) continue;
    setEdge(x, y, d, winWall(i));
  }
  if (inGrid(x, y)) maze[y][x] |= W_VISITED;
}

// what each ToF sees along its line from cell (x,y), beyond the cell's own edges (those are written by mapStandstill):
// open edges up to the first wall, and that wall, up to RAY_K_MAX cells away. Steady readings only; 8190 / far = open
// up to RAY_K_MAX. Only fills edges not known yet (a closer look always wins; driven-through edges stay open).
static void mapRays(int x, int y) {
  for (uint8_t i = 0; i < 4; i++) {
    if (!wnOk[i]) continue;
    const uint8_t d = SENSOR_DIR[i];
    const bool steady = wnAvg[i] < TRUST_MAX_MM;    // a real reading within the trusted range
    const int free = steady ? winK(i) : winFree(i); // cells free before the wall (far / scattered: capped, no wall)
    int cx = x + DIR_DX[d], cy = y + DIR_DY[d];
    for (int j = 1; j <= RAY_K_MAX && inGrid(cx, cy); j++) {
      if (j < free) { if (!edgeKnown(cx, cy, d)) setEdge(cx, cy, d, false); }
      else { if (steady && j == free && !edgeKnown(cx, cy, d)) setEdge(cx, cy, d, true); break; }
      cx += DIR_DX[d]; cy += DIR_DY[d];
    }
  }
}

// new start: clear the map, position = start cell, record the start cell walls (robot must be still)
static void mazeReset() {
  memset(maze, 0, sizeof(maze));
  memset(mzOpen, 0, sizeof(mzOpen));
  memset(mzSeen, 0, sizeof(mzSeen));
  mzX = START_X; mzY = START_Y;
  mzWinReq = 0;
  mzT = millis();
  mzState = M_START_WAIT;
}
static bool mazeIdle() { return mzState == M_IDLE; }

// request a move of n cells in direction dir (D_N..D_W). Ignored unless idle.
static void mazeRequest(uint8_t dir, uint8_t n, const char *src) {
  if (mzState == M_OFF) return;
  if (mzState != M_IDLE) { Serial.printf("MOVE %c%u from %s ignored (busy)\n", DIR_CH[dir & 3], (unsigned)n, src); return; }
  if (n < 1) n = 1;
  if (n > FREE_MAX) n = FREE_MAX;
  memset(&mv, 0, sizeof(mv));
  mv.dir = dir & 3; mv.nReq = n; mv.nTgt = n;
  mv.sx = mzX; mv.sy = mzY;
  mv.tReq = millis();
  mv.gate = "-"; mv.gateFree = -1; mv.kA = -1; mv.kB = -1; mv.e0src = '-';
  mv.A0 = mv.B0 = mv.e0 = mv.T = mv.sFinal = mv.eSettle = NAN;
  mzState = M_SENSE_WAIT;
}

static void mvAbort(const char *why, bool stopMotors) {   // move failed: position and map unchanged
  if (stopMotors) { mvSetSpeed(0.0f); mv.vCmd = 0.0f; }
  const bool moved = (mzState == M_DRIVE || mzState == M_SETTLE_WAIT || mzState == M_SETTLE_WIN) && fabsf(mv.s) > 0.3f * CELL_P_MM;
  if (mzState == M_DRIVE && !mv.corr) mv.sFinal = mv.s;
  logMove(why, false, 0);
  if (moved) Serial.printf("WARN aborted after moving ~%.0f mm: the map position may now be wrong ('reset' / BOOTSEL to restart)\n", mv.s);
  mzState = M_IDLE;
}
static void mazeStop() {                            // serial "stop"
  if (mzState >= M_SENSE_WAIT) mvAbort("ABORT_USER", true);
  else if (mzState == M_START_WAIT || mzState == M_START_WIN || mzState == M_CAL_WIN) mzState = M_IDLE;
}

// the move has a ToF reference below lim for its whole length: the ahead one (only gets closer) or the behind one at the
// end (B0 + n*P). steadyOnly: that sensor's gate window must be steady (at most 2 samples off, none 8190)
// Far tier: the ahead one is fine if steady now (it only gets closer); the behind one must end within the range where this
// sensor has already read steadily (tofSteadyMax): a sensor that turns to junk past ~700 mm never gets the far tier.
static bool gateRef(uint8_t a, uint8_t b, float lim, bool steadyOnly) {
  const bool okA = wnOk[a] && wnAvg[a] < lim && (!steadyOnly || winSteady(a));
  const float bEnd = wnAvg[b] + mv.nReq * CELL_P_MM;
  const bool okB = wnOk[b] && bEnd < lim && (!steadyOnly || (winSteady(b) && bEnd <= tofSteadyMax[b] + 15.0f));
  return okA || okB;
}

// start a drive with the current window as baseline. T = target progress (mm, + = forward along mv.dir)
static void startDrive(bool corr, float T, uint32_t timeoutMs, float vmax) {
  const uint8_t a = DIR_AHEAD[mv.dir], b = DIR_BEHIND[mv.dir];
  const uint32_t now = millis();
  mv.corr = corr;
  mv.dA0 = wnAvg[a]; mv.dB0 = wnAvg[b];
  if (!(mv.trust > 0.0f)) mv.trust = TRUST_MAX_MM;
  mv.useA = wnOk[a] && wnAvg[a] < mv.trust;
  mv.useB = wnOk[b] && wnAvg[b] < mv.trust;
  mv.Anow = mv.dA0; mv.Bnow = mv.dB0;
  if (!corr) mv.T = T;                              // mv.T = main target (logged), mv.tgt = target of this drive
  mv.tgt = T;
  mv.s = 0.0f; mv.sv = 0.0f; mv.used = 0; mv.zSum = 0.0f; mv.zN = 0;
  mv.tS = mv.tA = mv.tB = mv.tDrive = mv.tTick = mv.tDbg = now;
  mv.timeoutMs = timeoutMs; mv.vmax = vmax; mv.vCmd = 0.0f;
  mv.inBand = false; mv.outCnt = 0; mv.closeA = 0; mv.closeB = 0; mv.badA = 0; mv.badB = 0; mv.sickA = mv.sickB = false;
  mv.aHist[0] = mv.aHist[1] = mv.dA0; mv.aHistN = (mv.dA0 >= 0.0f) ? 2 : 0;   // start the medians at the standstill values
  mv.bHist[0] = mv.bHist[1] = mv.dB0; mv.bHistN = (mv.dB0 >= 0.0f) ? 2 : 0;
  for (uint8_t i = 0; i < 4; i++) mv.seq[i] = g_lidarSeqI[i];
  if (!corr) { memset(mv.sideTot, 0, sizeof(mv.sideTot)); memset(mv.sideHit, 0, sizeof(mv.sideHit)); }
  mzState = M_DRIVE;
}

static float med3(const float *v, uint8_t n) {
  if (n == 1) return v[0];
  if (n == 2) return 0.5f * (v[0] + v[1]);
  return fmaxf(fminf(v[0], v[1]), fminf(fmaxf(v[0], v[1]), v[2]));
}

// progress of the current drive from the ahead / behind ToFs (on every new ahead / behind sample):
//   sA = A0 - A_now, sB = B_now - B0, fused with weights 1 / (1 + (reading / 300)^2).
// The fused value is filtered in mvFilter() (alpha-beta, every MV_TICK_MS): smooths the ToF noise without lag and
// uses only the ToF progress itself (no wheel odometry).
static void mvMeasure(uint32_t now) {
  bool okA = mv.useA && !mv.sickA && mv.Anow >= 0.0f && mv.Anow < mv.trust && (uint32_t)(now - mv.tA) < SAMPLE_STALE_MS;
  bool okB = mv.useB && !mv.sickB && mv.Bnow >= 0.0f && mv.Bnow < mv.trust && (uint32_t)(now - mv.tB) < SAMPLE_STALE_MS;
  const float sPred = mv.s + mv.sv * (float)(uint32_t)(now - mv.tTick);   // filter estimate, extrapolated to now
  const float sA = mv.dA0 - mv.Anow, sB = mv.Bnow - mv.dB0;
  if (okA && okB && fabsf((mv.Anow + mv.Bnow) - (mv.dA0 + mv.dB0)) > SUM_DRIFT_MM) {
    // the two disagree: drop the one further from the estimate (can't tell -> drop the farther one)
    const float dA = fabsf(sA - sPred), dB = fabsf(sB - sPred);
    if (fabsf(dA - dB) < 5.0f) { if (mv.Anow > mv.Bnow) okA = false; else okB = false; }
    else if (dA > dB) okA = false;
    else okB = false;
  }
  const float lim = JUMP_BASE_MM + 1.5f * mv.vmax * (float)(uint32_t)(now - mv.tS);   // m/s == mm/ms
  if (okA && fabsf(sA - sPred) > lim) okA = false;
  if (okB && fabsf(sB - sPred) > lim) okB = false;
  if (!okA && !okB) return;                         // nothing usable -> the MV_NOSENS_MS timer keeps running
  const float wA = okA ? 1.0f / (1.0f + (mv.Anow / FUSE_R0_MM) * (mv.Anow / FUSE_R0_MM)) : 0.0f;
  const float wB = okB ? 1.0f / (1.0f + (mv.Bnow / FUSE_R0_MM) * (mv.Bnow / FUSE_R0_MM)) : 0.0f;
  mv.zSum += (wA * sA + wB * sB) / (wA + wB);
  mv.zN++;
  mv.tS = now;
  mv.used = (okA ? 1 : 0) | (okB ? 2 : 0);
}

// alpha-beta step (every MV_TICK_MS) + stop band: enter within MV_TOL_MM (speed -> 0), leave only beyond
// MV_TOL_EXIT_MM (2 updates in a row)
static void mvFilter(uint32_t now, float dtMs) {
  mv.s += mv.sv * dtMs;
  if (mv.zN == 0) return;
  const float r = mv.zSum / mv.zN - mv.s;
  mv.zSum = 0.0f; mv.zN = 0;
  mv.s += AB_ALPHA * r;
  mv.sv += AB_BETA * r / dtMs;
  const float svMax = 1.5f * mv.vmax;
  mv.sv = constrain(mv.sv, -svMax, svMax);
  const float err = fabsf(mv.tgt - mv.s);
  if (!mv.inBand) {
    if (err <= MV_TOL_MM) { mv.inBand = true; mv.tBand = now; mv.outCnt = 0; }
  } else if (err > MV_TOL_EXIT_MM) {
    if (++mv.outCnt >= 2) mv.inBand = false;
  } else {
    mv.outCnt = 0;
  }
}

// multi-cell runs: no stop at intermediate centres. Re-gate on the live ahead reading (median of the last 3 samples)
// on every new ahead sample: the wall it sees is at cell boundary j = round((A - C_ahead + p) / P), p = position from
// the start-cell centre. A wall before the planned end -> the target becomes the cell in front of it. Checking on every
// sample (not only at each centre crossing) leaves a whole cell to brake. Never re-targets the start cell (obstacles
// that close are handled by the CLOSE_MM abort).
static bool histHasFar(const float *h, uint8_t n) {  // an 8190 among the last samples = that sensor sees nothing: clear
  for (uint8_t k = 0; k < n; k++) if (h[k] >= TOF_FAR_MM) return true;
  return false;
}

static void mvRegate() {
  if (mv.corr || mv.aHistN < 3 || histHasFar(mv.aHist, mv.aHistN)) return;
  const uint8_t a = DIR_AHEAD[mv.dir];
  const float am = med3(mv.aHist, mv.aHistN);
  if (am > C_MM[a] + 2.0f * CELL_P_MM) return;      // only walls up to 2 cells ahead (reliable ToF range) can re-target
  const float p = mv.s + mv.e0;
  const int j = (int)lroundf((am - C_MM[a] + p) / CELL_P_MM);
  if (j == (int)mv.nTgt && !mv.wallAtDest) {        // wall at the far side of the destination (gate could not see it)
    mv.wallAtDest = true;
    if (g_mzDbg) Serial.printf("DRV wall seen at the destination (ahead %.0f)\n", am);
  }
  if (j >= 1 && j < (int)mv.nTgt) {
    mv.nTgt = (uint8_t)j;
    mv.tgt = mv.T = mv.nTgt * CELL_P_MM - mv.e0;
    mv.wallAtDest = true;
    mv.inBand = false;
    if (g_mzDbg) Serial.printf("DRV re-gate: ahead %.0f at p=%.0f -> stop after %u cell(s)\n", am, p, (unsigned)mv.nTgt);
  }
}

// stop request during a multi-cell run: end at the first cell centre the speed law can still stop at smoothly
// (remaining distance >= vCmd / Kp), so the robot stops on a cell and its position stays known
static void mvShorten() {
  if (mzState != M_DRIVE || mv.corr || mv.nTgt <= 1) return;
  const float p = mv.s + mv.e0;                     // mm from the start cell centre
  const float need = fabsf(mv.vCmd) * 1000.0f / MV_KP + 10.0f;
  int j = (int)ceilf((p + need) / CELL_P_MM);
  if (j < 1) j = 1;
  if (j >= (int)mv.nTgt) return;
  mv.nTgt = (uint8_t)j;
  mv.tgt = mv.T = mv.nTgt * CELL_P_MM - mv.e0;
  mv.inBand = false;
  Serial.printf("MOVE shortened to %u cell(s) (stop request)\n", (unsigned)mv.nTgt);
}

// side walls of the intermediate cells: +-SIDE_WIN_MM window around each cell-centre crossing,
// shifted for the side-sensor lead (N: -20 mm, S: +20 mm, E/W: 0)
static void mvSide(uint8_t i, float mm) {
  if (mv.corr || mm < 0.0f) return;
  const uint8_t sd = (i == DIR_SIDE1[mv.dir]) ? 0 : 1;
  const float p = mv.s + mv.e0;
  for (uint8_t c = 1; c < mv.nReq && c <= 2; c++) {
    if (fabsf(p - (c * CELL_P_MM + DIR_SIDE_SHIFT[mv.dir])) > SIDE_WIN_MM) continue;
    if (mv.sideTot[c - 1][sd] >= 60000) continue;
    mv.sideTot[c - 1][sd]++;
    if (mm < C_MM[i] + 0.5f * CELL_P_MM) mv.sideHit[c - 1][sd]++;
  }
}

static void mvEndDrive(uint32_t now) {              // drive finished (or correction ended): stop, then settle + verify
  mvSetSpeed(0.0f);
  mv.vCmd = 0.0f;
  // nearest wall along the move axis far away -> twice the samples in the verify window (ToF noise grows with range)
  float nearMm = TOF_FAR_MM;
  if (mv.Anow >= 0.0f) nearMm = fminf(nearMm, mv.Anow);
  if (mv.Bnow >= 0.0f) nearMm = fminf(nearMm, mv.Bnow);
  mv.settleN = (nearMm > SETTLE_FAR_MM) ? WIN_N_FAR : WIN_N;
  if (!mv.corr) mv.sFinal = mv.s;
  mzT = now;
  mzState = M_SETTLE_WAIT;
}

// a reference sensor returning out-of-range / invalid / scattered samples (3 of its last 5) is unreliable right now
// (VL53L0X near its range limit, or the cone catching side walls): paused until at most 1 of its last 5 is bad
static void mvHealth(uint8_t *bits, bool *sick, bool bad, char which) {
  *bits = (uint8_t)((*bits << 1) | (bad ? 1 : 0));
  const uint8_t b5 = *bits & 0x1F;
  const uint8_t nBad = (uint8_t)((b5 & 1) + ((b5 >> 1) & 1) + ((b5 >> 2) & 1) + ((b5 >> 3) & 1) + ((b5 >> 4) & 1));
  if (!*sick && nBad >= 3)     { *sick = true;  if (g_mzDbg) Serial.printf("DRV sensor %c scattering -> paused\n", which); }
  else if (*sick && nBad <= 1) { *sick = false; if (g_mzDbg) Serial.printf("DRV sensor %c clean again\n", which); }
}

// push a valid sample into a 3-deep history, return the median (single scattered samples never reach the progress)
static float histMed(float *h, uint8_t *n, float x) {
  if (*n < 3) h[(*n)++] = x;
  else { h[0] = h[1]; h[1] = h[2]; h[2] = x; }
  return med3(h, *n);
}

static void mvDriveStep(uint32_t now) {
  const uint8_t a = DIR_AHEAD[mv.dir], b = DIR_BEHIND[mv.dir];
  bool upd = false;
  for (uint8_t i = 0; i < 4; i++) {
    float r;
    if (!tofFresh(i, &mv.seq[i], &r)) continue;
    const float mm = tofCal(i, r);
    if (i == a) {
      if (mm < 0.0f) { mvHealth(&mv.badA, &mv.sickA, true, 'A'); continue; }   // invalid sample: no update
      const float med = histMed(mv.aHist, &mv.aHistN, mm);
      mvHealth(&mv.badA, &mv.sickA, mm >= mv.trust || fabsf(mm - med) > SPIKE_MM + 0.03f * med, 'A');
      mv.Anow = med; mv.tA = now; upd = true;
      if (mm < C_MM[a] + CLOSE_MM && mv.vCmd >= 0.0f && !histHasFar(mv.aHist, mv.aHistN)) mv.closeA++; else mv.closeA = 0;   // at/inside centred distance (2 in a row, no 8190 in the last 3)
    } else if (i == b) {
      if (mm < 0.0f) { mvHealth(&mv.badB, &mv.sickB, true, 'B'); continue; }
      const float med = histMed(mv.bHist, &mv.bHistN, mm);
      mvHealth(&mv.badB, &mv.sickB, mm >= mv.trust || fabsf(mm - med) > SPIKE_MM + 0.03f * med, 'B');
      mv.Bnow = med; mv.tB = now; upd = true;
      if (mm < C_MM[b] + CLOSE_MM && mv.vCmd < 0.0f && !histHasFar(mv.bHist, mv.bHistN)) mv.closeB++; else mv.closeB = 0;
    } else {
      mvSide(i, mm);
    }
  }
  if (upd) { mvMeasure(now); mvRegate(); }

  // ahead reading below C + 3 mm (2 samples in a row): arrived if that wall was expected there, otherwise abort
  if (mv.closeA >= 2 || mv.closeB >= 2) {
    const bool nearEnd = fabsf(mv.tgt - mv.s) < 0.3f * CELL_P_MM;   // a wall at centred distance this close to the target is the destination wall
    if (mv.corr || (mv.closeA >= 2 && (mv.wallAtDest || nearEnd))) mvEndDrive(now);
    else mvAbort(mv.closeA >= 2 ? "ABORT_CLOSE" : "ABORT_CLOSE_B", true);
    return;
  }
  // no valid sensor for 300 ms / timeout: abort. Exception: already within CORR_MAX_MM of the target -> stop and let
  // SETTLE verify / correct (aborting there would leave the map one cell behind the robot)
  const bool nearTgt = fabsf(mv.tgt - mv.s) < CORR_MAX_MM;
  if ((uint32_t)(now - mv.tS) > MV_NOSENS_MS)     { if (mv.corr || nearTgt) mvEndDrive(now); else mvAbort("ABORT_NOSENS", true);  return; }
  if ((uint32_t)(now - mv.tDrive) > mv.timeoutMs) { if (mv.corr || nearTgt) mvEndDrive(now); else mvAbort("ABORT_TIMEOUT", true); return; }

  if ((uint32_t)(now - mv.tTick) < MV_TICK_MS) return;
  const float dtMs = (float)(uint32_t)(now - mv.tTick);
  const float dt = (dtMs > 50.0f) ? 0.05f : dtMs * 1e-3f;
  mv.tTick = now;
  mvFilter(now, dtMs);
  if (mv.inBand && (uint32_t)(now - mv.tBand) >= MV_DONE_MS) { mvEndDrive(now); return; }   // within 3 mm for 100 ms

  const float err = mv.tgt - mv.s;                  // mm
  float v = 0.0f;
  if (!mv.inBand) {
    v = constrain(MV_KP * err * 0.001f, -mv.vmax, mv.vmax);
    const float creep = (mv.dir == D_E || mv.dir == D_W) ? MV_VCREEP_EW : MV_VCREEP;
    if (fabsf(v) < creep) v = copysignf(creep, err);
  }
  if (fabsf(v) > fabsf(mv.vCmd) && v * mv.vCmd >= 0.0f) {   // limit speeding up only
    const float vUp = fabsf(mv.vCmd) + MV_ACC * dt;
    if (fabsf(v) > vUp) v = copysignf(vUp, v);
  }
  mv.vCmd = v;
  mvSetSpeed(v);
  if (g_mzDbg && (uint32_t)(now - mv.tDbg) >= 50) {
    mv.tDbg = now;
    Serial.printf("DRV%s s=%.1f tgt=%.1f v=%.3f A=%.0f B=%.0f use=%u band=%d n=%u\n", mv.corr ? "(corr)" : "",
                  mv.s, mv.tgt, v, mv.Anow, mv.Bnow, (unsigned)mv.used, (int)mv.inBand, (unsigned)mv.nTgt);
  }
}

// successful move: map (intermediate cells + destination), position, log line, back to IDLE
static void mvCommit() {
  const uint8_t d = mv.dir, back = DIR_OPP[d];
  for (uint8_t c = 1; c < mv.nTgt; c++) {
    const int x = mv.sx + DIR_DX[d] * c, y = mv.sy + DIR_DY[d] * c;
    lockOpen(x, y, back);                           // drove through: travel-axis edges are open for good
    lockOpen(x, y, d);
    for (uint8_t sd = 0; sd < 2; sd++) {            // side walls from the in-motion window, only into unknown edges
      const uint8_t i = sd ? DIR_SIDE2[d] : DIR_SIDE1[d];
      const uint16_t tot = mv.sideTot[c - 1][sd], hit = mv.sideHit[c - 1][sd];
      if (tot >= SIDE_MIN_N && !edgeKnown(x, y, SENSOR_DIR[i])) setEdge(x, y, SENSOR_DIR[i], hit >= SIDE_FRAC * tot);
    }
    if (inGrid(x, y)) maze[y][x] |= W_VISITED;
  }
  const int x = mv.sx + DIR_DX[d] * mv.nTgt, y = mv.sy + DIR_DY[d] * mv.nTgt;
  lockOpen(x, y, back);                             // came in through this edge
  mapStandstill(x, y, back);                        // the other 3 edges from the final standstill window
  mapRays(x, y);
  mzX = (int8_t)x; mzY = (int8_t)y;
  warnInvalid();
  logMove(mv.suspect ? "SUSPECT" : (mv.nTgt < mv.nReq ? "OK_SHORT" : "OK"), true, mv.nTgt);
  mzState = M_IDLE;
}

// ---- serial helpers: window print, C calibration, ToF calibration, map print ----
static void printWindow() {
  Serial.println("WIN  sensor: avg mm (kept/got)  wall  free");
  for (uint8_t i = 0; i < 4; i++)
    Serial.printf("  %c: %7.1f (%u/%u)  %s  free %d   raw %.1f\n", SENSOR_CH[i], wnAvg[i], (unsigned)wnCnt[i], (unsigned)wnGot[i],
                  winWall(i) ? (wnOk[i] ? "WALL" : "WALL(invalid)") : (wnAvg[i] >= TOF_FAR_MM ? "clear (8190 / far)" : (wnNear[i] < TOF_FAR_MM ? "open(far, scattered)" : "open")), winFree(i), wnRawAvg[i]);
  float e;
  bool ok = winOffset(D_N, &e);
  Serial.printf("  N/S offset: %s%+.1f mm (src %c)", ok ? "" : "n/a ", e, offSrc);
  ok = winOffset(D_E, &e);
  Serial.printf("   E/W offset: %s%+.1f mm (src %c)\n", ok ? "" : "n/a ", e, offSrc);
  uint8_t v = W_VISITED;
  for (uint8_t i = 0; i < 4; i++) if (winWall(i)) v |= DIR_BIT[SENSOR_DIR[i]];
  Serial.printf("  cell value here: %u\n", (unsigned)v);
  Serial.printf("  steady range seen so far: F %.0f  B %.0f  L %.0f  R %.0f mm\n", tofSteadyMax[0], tofSteadyMax[1], tofSteadyMax[2], tofSteadyMax[3]);
}

static void ccalDone() {                            // robot centred in a closed 4-wall cell
  for (uint8_t i = 0; i < 4; i++) {
    if (wnOk[i] && wnAvg[i] > 10.0f && wnAvg[i] < 0.5f * CELL_P_MM + 40.0f) C_MM[i] = wnAvg[i];
    else Serial.printf("CCAL: sensor %c invalid or no close wall (%.1f), C not changed\n", SENSOR_CH[i], wnAvg[i]);
  }
  Serial.printf("CCAL done (active now). static float C_MM[4] = { %.1ff, %.1ff, %.1ff, %.1ff };  // front, back, left, right\n",
                C_MM[0], C_MM[1], C_MM[2], C_MM[3]);
}

#define TCAL_MAXP 8
static float   tcRaw[4][TCAL_MAXP], tcMm[4][TCAL_MAXP];
static uint8_t tcN[4] = { 0, 0, 0, 0 };
static void tcalPrint() {
  Serial.printf("static float TOF_GAIN[4] = { %.4ff, %.4ff, %.4ff, %.4ff };\n", TOF_GAIN[0], TOF_GAIN[1], TOF_GAIN[2], TOF_GAIN[3]);
  Serial.printf("static float TOF_OFF[4]  = { %.2ff, %.2ff, %.2ff, %.2ff };\n", TOF_OFF[0], TOF_OFF[1], TOF_OFF[2], TOF_OFF[3]);
}
static void tcalPoint() {
  const uint8_t i = mzWinArgI;
  if (!wnOk[i] || wnRawAvg[i] >= TOF_FAR_MM) { Serial.printf("TCAL %c: no valid reading, point not added\n", SENSOR_CH[i]); return; }
  if (tcN[i] >= TCAL_MAXP) { Serial.printf("TCAL %c: %u points already, send 'tcal %u clr' first\n", SENSOR_CH[i], (unsigned)tcN[i], (unsigned)i); return; }
  tcRaw[i][tcN[i]] = wnRawAvg[i];
  tcMm[i][tcN[i]] = mzWinArgMm;
  tcN[i]++;
  Serial.printf("TCAL %c point %u: raw %.1f -> true %.1f mm (%u samples kept)\n", SENSOR_CH[i], (unsigned)tcN[i], wnRawAvg[i], mzWinArgMm, (unsigned)wnCnt[i]);
}
static void tcalFit(uint8_t i) {
  const uint8_t n = tcN[i];
  if (n < 2) { Serial.printf("TCAL %c: need at least 2 points (have %u)\n", SENSOR_CH[i], (unsigned)n); return; }
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (uint8_t k = 0; k < n; k++) {
    sx += tcRaw[i][k]; sy += tcMm[i][k];
    sxx += (double)tcRaw[i][k] * tcRaw[i][k]; sxy += (double)tcRaw[i][k] * tcMm[i][k];
  }
  float rmin = tcRaw[i][0], rmax = tcRaw[i][0];
  for (uint8_t k = 1; k < n; k++) { rmin = fminf(rmin, tcRaw[i][k]); rmax = fmaxf(rmax, tcRaw[i][k]); }
  if (rmax - rmin < 50.0f) { Serial.printf("TCAL %c: points only %.0f mm apart, spread them out (e.g. 60 .. 700 mm). Not applied.\n", SENSOR_CH[i], rmax - rmin); return; }
  const double den = n * sxx - sx * sx;
  const float g = (float)((n * sxy - sx * sy) / den);
  const float o = (float)((sy - g * sx) / n);
  if (g < 0.7f || g > 1.45f || fabsf(o) > 80.0f) {
    Serial.printf("TCAL %c: fit gain %.3f off %.1f is implausible (check sensor index / ruler values). Not applied.\n", SENSOR_CH[i], g, o);
    return;
  }
  TOF_GAIN[i] = g; TOF_OFF[i] = o;
  Serial.printf("TCAL %c fit (active now): gain %.4f  off %.2f\n", SENSOR_CH[i], g, o);
  float worst = 0.0f;
  for (uint8_t k = 0; k < n; k++) {
    const float r = g * tcRaw[i][k] + o - tcMm[i][k];
    worst = fmaxf(worst, fabsf(r));
    Serial.printf("  raw %.1f  true %.1f  -> %.1f  residual %+.1f\n", tcRaw[i][k], tcMm[i][k], g * tcRaw[i][k] + o, r);
  }
  Serial.printf("  max residual %.1f mm%s\n", worst, (worst <= 2.0f) ? "" : "  (> 2 mm: re-check the points)");
  tcalPrint();
}

// serial window request (only when idle): 1 = print, 2 = ccal, 3 = tcal point (sensor i, true mm)
static void mazeWinRequest(uint8_t req, uint8_t i, float mm) {
  if (mzState != M_IDLE) { Serial.println("busy (move or window running), try again when idle"); return; }
  mzWinReq = req; mzWinArgI = i; mzWinArgMm = mm;
}

static void mazeEdgeLine(int y, uint8_t d) {        // horizontal edges (d = D_N or D_S) of row y
  char ln[MAZE_N * 4 + 4];
  int k = 0;
  for (int x = 0; x < MAZE_N; x++) {
    ln[k++] = '+';
    const char *seg = !edgeKnown(x, y, d) ? " . " : (maze[y][x] & DIR_BIT[d]) ? "---" : "   ";
    memcpy(ln + k, seg, 3); k += 3;
  }
  ln[k++] = '+'; ln[k] = 0;
  Serial.printf("   %s\n", ln);
}
static void mazeCellLine(int y) {                   // vertical edges + cells of row y
  char ln[MAZE_N * 4 + 4];
  int k = 0;
  for (int x = 0; x < MAZE_N; x++) {
    ln[k++] = !edgeKnown(x, y, D_W) ? ':' : (maze[y][x] & DIR_BIT[D_W]) ? '|' : ' ';
    const char *in = (x == mzX && y == mzY) ? " R " : (maze[y][x] & W_VISITED) ? "   " : " ? ";
    memcpy(ln + k, in, 3); k += 3;
  }
  ln[k++] = !edgeKnown(MAZE_N - 1, y, D_E) ? ':' : (maze[y][MAZE_N - 1] & DIR_BIT[D_E]) ? '|' : ' ';
  ln[k] = 0;
  Serial.printf("%2d %s\n", y, ln);
}
static void mazePrint() {
  Serial.printf("MAP %dx%d, robot (%d,%d), start (%d,%d). Values: 1 visited, 2 N, 4 E, 8 S, 16 W. Row = y (N at top), column = x.\n",
                MAZE_N, MAZE_N, (int)mzX, (int)mzY, (int)START_X, (int)START_Y);
  for (int y = MAZE_N - 1; y >= 0; y--) {
    Serial.printf("%2d:", y);
    for (int x = 0; x < MAZE_N; x++) Serial.printf("%3u", (unsigned)maze[y][x]);
    Serial.println();
  }
  Serial.print("   ");
  for (int x = 0; x < MAZE_N; x++) Serial.printf("%3d", x);
  Serial.println();
  Serial.println("--- = wall, blank = open, . / : = unknown, R = robot, ? = unvisited");
  for (int y = MAZE_N - 1; y >= 0; y--) { mazeEdgeLine(y, D_N); mazeCellLine(y); }
  mazeEdgeLine(0, D_S);
}

// ---- main maze task (core 0, called every loop) ----
static void mazeTask() {
  const uint32_t now = millis();
  if (mzState == M_OFF) return;
  if (!g_rcMode && mzState >= M_SENSE_WAIT) mvAbort("ABORT_MANUAL", false);   // serial rpm / vel took over: leave the motors to it
  switch (mzState) {
    case M_IDLE:
      if (mzWinReq) { winStart(); mzState = M_CAL_WIN; }
      break;

    case M_START_WAIT:                              // let the robot come to rest after the button press
      if ((uint32_t)(now - mzT) >= 500) { winStart(); mzState = M_START_WIN; }
      break;

    case M_START_WIN:
      if (!winPoll()) break;
      mapStandstill(mzX, mzY, -1);
      mapRays(mzX, mzY);
      Serial.printf("START cell (%d,%d) = %u   F %.0f  B %.0f  L %.0f  R %.0f\n", (int)mzX, (int)mzY,
                    (unsigned)maze[mzY][mzX], wnAvg[0], wnAvg[1], wnAvg[2], wnAvg[3]);
      warnInvalid();
      logHeader();
      mzState = M_IDLE;
      break;

    case M_CAL_WIN:
      if (!winPoll()) break;
      if (mzWinReq == 1) printWindow();
      else if (mzWinReq == 2) ccalDone();
      else if (mzWinReq == 3) tcalPoint();
      mzWinReq = 0;
      mzState = M_IDLE;
      break;

    case M_SENSE_WAIT:                              // the gate is only evaluated on a standstill window
      // the verify window of the last move is still fresh and nothing moved since it started: use it (saves a window)
      if (wnDone && (int32_t)(wnT0 - mzLastMotion) > 0 && (uint32_t)(now - wnDoneT) < WIN_REUSE_MS) { mzReuse = true; mzState = M_SENSE_WIN; }
      else if ((uint32_t)(now - mzLastMotion) >= SETTLE_MS) { winStart(); mzState = M_SENSE_WIN; }
      break;

    case M_SENSE_WIN: {                             // GATE + TARGET
      if (!mzReuse && !winPoll()) break;
      mzReuse = false;
      mapStandstill(mv.sx, mv.sy, -1);              // standstill look at the current cell: own walls + what lies further along each line
      mapRays(mv.sx, mv.sy);
      const uint8_t a = DIR_AHEAD[mv.dir], b = DIR_BEHIND[mv.dir];
      mv.A0 = wnAvg[a]; mv.B0 = wnAvg[b];
      mv.kA = wnOk[a] ? winK(a) : -1;
      mv.kB = wnOk[b] ? winK(b) : -1;
      mv.gateFree = winFree(a);                     // round((A - C_ahead) / P), clamped to 3
      const int dx = mv.sx + DIR_DX[mv.dir] * mv.nReq, dy = mv.sy + DIR_DY[mv.dir] * mv.nReq;
      const char *refuse = NULL;
      if (!wnOk[a])                   refuse = "NOSENSOR";   // invalid ahead sensor counts as blocked
      else if (mv.gateFree < mv.nReq) refuse = "BLOCKED";
      else if (!inGrid(dx, dy))       refuse = "OUTGRID";
      // progress needs a ToF reference for the whole move: the ahead one only gets closer; the behind one must still be
      // in range at the end (B0 + n*P), otherwise the move would lose its reference halfway and abort
      else {
        mv.trust = TRUST_MAX_MM;
        if (!gateRef(a, b, TRUST_MAX_MM, false)) {
          if (gateRef(a, b, TRUST_FAR_MM, true)) mv.trust = TRUST_FAR_MM;   // only a far one: use it if it read steadily
          else refuse = "NOREF";
        }
      }
      if (refuse) {
        wnDone = false;                             // a retry takes a fresh window
        mv.gate = refuse;
        warnInvalid();
        logMove(refuse, true, 0);
        mzState = M_IDLE;
        break;
      }
      mv.gate = (mv.trust > TRUST_MAX_MM) ? "OKFAR" : "OK";   // OKFAR: driven on a far reference (walls 3+ cells away)
      float e0;
      winOffset(mv.dir, &e0);                       // no wall within 3 cells either way -> e0 = 0
      mv.e0 = e0; mv.e0src = offSrc;
      mv.wallAtDest = (mv.kA == (int)mv.nReq);      // the wall ahead is at the far side of the destination cell
      const bool ew = (mv.dir == D_E || mv.dir == D_W);
      startDrive(false, mv.nReq * CELL_P_MM - e0, MV_TIMEOUT_MS + MV_TIMEOUT_CELL_MS * mv.nReq, ew ? fminf(MV_VMAX, MV_VMAX_EW) : MV_VMAX);
      break;
    }

    case M_DRIVE:
      mvDriveStep(now);
      break;

    case M_SETTLE_WAIT:
      if ((uint32_t)(now - mzT) >= SETTLE_MS) { winStart(mv.settleN); mzState = M_SETTLE_WIN; }
      break;

    case M_SETTLE_WIN: {                            // VERIFY: offset from the nearest wall, up to 2 creep corrections
      if (!winPoll()) break;
      float e;
      const bool ok = winOffset(mv.dir, &e);
      mv.eSettle = ok ? e : NAN;
      if (ok && fabsf(e) > CORR_MIN_MM && fabsf(e) < CORR_MAX_MM && mv.nCorr < CORR_MAX_N) {
        mv.nCorr++;
        startDrive(true, -e, CORR_TIMEOUT_MS, MV_VCORR);
        break;
      }
      mv.suspect = ok && fabsf(e) >= CORR_MAX_MM;
      mvCommit();
      break;
    }
  }
}

// ---- flood-fill solver (optimistic: unknown edges = open), re-planned after every move ----
#define FF_UNREACH 0xFFFF
static uint16_t ffDist[MAZE_N][MAZE_N];
static uint8_t  slVisits[MAZE_N][MAZE_N];
static bool     slActive = false;                   // solving
static bool     slStopReq = false;                  // stop after the current move
static bool     slWait = false;                     // a solver move is running, result pending
static uint32_t slSeq = 0;                          // mzResultSeq when the solver move was requested
static uint8_t  slPendDir = 0, slPendN = 1;
static uint8_t  slLastDir = 255;                    // direction of the last move (prefer going straight)
static uint8_t  slCap[MAZE_N][MAZE_N];              // per cell, 2 bits per direction: longest run the gate accepted / may accept
                                                    // from here (3 = untested). Lowered when a run is refused; 0 = no move that way
static bool     slRetried = false;                  // the last refused request was already retried once
static uint16_t slRefuse = 0;
static uint16_t slCmds = 0;
static uint32_t slT0 = 0;

static uint8_t slCapGet(int x, int y, uint8_t d) { return (slCap[y][x] >> (2 * d)) & 3; }
static void slCapSet(int x, int y, uint8_t d, uint8_t c) { slCap[y][x] = (uint8_t)((slCap[y][x] & ~(3 << (2 * d))) | ((c & 3) << (2 * d))); }
static bool ffBlocked(int x, int y, uint8_t d) {    // planning: can the robot move from (x,y) towards d? (wall / grid / refused)
  if (!inGrid(x + DIR_DX[d], y + DIR_DY[d])) return true;
  if (maze[y][x] & DIR_BIT[d]) return true;
  return slCapGet(x, y, d) == 0;
}
static void ffFlood() {                             // BFS distance (moves) to the goal over all edges not known to be walls
  static uint8_t qx[MAZE_N * MAZE_N], qy[MAZE_N * MAZE_N];
  for (int y = 0; y < MAZE_N; y++) for (int x = 0; x < MAZE_N; x++) ffDist[y][x] = FF_UNREACH;
  int h = 0, t = 0;
  ffDist[GOAL_Y][GOAL_X] = 0; qx[t] = GOAL_X; qy[t] = GOAL_Y; t++;
  while (h < t) {
    const int x = qx[h], y = qy[h]; h++;
    for (uint8_t d = 0; d < 4; d++) {               // neighbour (nx,ny) reaches (x,y) by moving opposite to d
      const int nx = x + DIR_DX[d], ny = y + DIR_DY[d];
      if (!inGrid(nx, ny) || ffBlocked(nx, ny, DIR_OPP[d])) continue;
      if (ffDist[ny][nx] != FF_UNREACH) continue;
      ffDist[ny][nx] = ffDist[y][x] + 1;
      qx[t] = (uint8_t)nx; qy[t] = (uint8_t)ny; t++;
    }
  }
}

static void solveLed(bool on) { digitalWrite(LED_BUILTIN, on ? HIGH : LOW); }

static void solveStart() {
  if (slActive) return;
  if (!g_rcMode) { setVelocities(0, 0, 0); mzLastMotion = millis(); g_rcMode = true; }   // leave manual mode
  slActive = true; slStopReq = false; slWait = false;
  memset(slCap, 0xFF, sizeof(slCap)); slRetried = false; slRefuse = 0;
  slCmds = 0; slT0 = millis(); slLastDir = 255;
  memset(slVisits, 0, sizeof(slVisits));
  solveLed(true);
  Serial.printf("SOLVE start at (%d,%d) -> goal (%d,%d)\n", (int)mzX, (int)mzY, (int)GOAL_X, (int)GOAL_Y);
}
static void solveEnd(const char *why) {
  slActive = false; slStopReq = false; slWait = false;
  solveLed(false);
  Serial.printf("SOLVE %s at (%d,%d): %u commands, %.1f s\n", why, (int)mzX, (int)mzY, (unsigned)slCmds, (millis() - slT0) * 1e-3f);
}
// button / "solve" while solving: stop at the next cell centre (a run is shortened), position stays valid.
// A second press before it has stopped = stop immediately (emergency; the position may then be off).
static void solveStopRequest() {
  if (!slActive) return;
  if (mazeIdle()) { solveEnd("stopped"); return; }
  if (mzState == M_SENSE_WAIT || mzState == M_SENSE_WIN) {   // not moving yet: just cancel the request
    mazeStop();
    solveEnd("stopped");
    return;
  }
  if (slStopReq) {
    mazeStop();
    solveEnd("stopped immediately (position may be off: hold BOOTSEL 1.5 s at the start cell to restart)");
    return;
  }
  slStopReq = true;
  mvShorten();
  Serial.println("SOLVE stopping at the next cell (press again = stop immediately)");
}

// one solver step: called every loop; acts only when the move layer is idle
static void solveStep() {
  if (!slActive || !mazeIdle()) return;
  if (slWait) {                                     // result of our last move
    if (mzResultSeq == slSeq) return;               // not finished yet
    slWait = false;
    const char *r = mzLastResult;
    if (!strncmp(r, "ABORT", 5)) {                  // the move layer gave up: stop and say whether the position still holds
      solveEnd(fabsf(mv.s) > 0.3f * CELL_P_MM ? "aborted mid-move: position may be wrong (put it on the start cell, hold BOOTSEL 1.5 s)"
                                               : "aborted before moving: position still valid (short press = continue)");
      return;
    }
    if (!strncmp(r, "OK", 2) || !strcmp(r, "SUSPECT")) {
      slCmds++; slRetried = false; slRefuse = 0;
      if (inGrid(mzX, mzY) && slVisits[mzY][mzX] < 255) slVisits[mzY][mzX]++;
    } else {                                        // refused (BLOCKED / NOREF / NOSENSOR / OUTGRID): plan around it
      if (++slRefuse > SOLVE_MAX_REFUSE) { solveEnd("stuck (too many refused moves)"); return; }
      const bool retry = !strcmp(r, "NOREF") || !strcmp(r, "NOSENSOR");   // may be one bad window: try once more
      if (retry && !slRetried && !slStopReq) {
        slRetried = true;
        slSeq = mzResultSeq; slWait = true;
        mazeRequest(slPendDir, slPendN, "solve");
        return;
      }
      slRetried = false;
      if (slCapGet(mzX, mzY, slPendDir) >= slPendN) slCapSet(mzX, mzY, slPendDir, slPendN - 1);   // 0 = never that way from here
    }
  }
  if (slStopReq) { solveEnd("stopped"); return; }
  if (mzX == GOAL_X && mzY == GOAL_Y) { solveEnd("GOAL reached"); mazePrint(); return; }
  if (slCmds >= SOLVE_MAX_CMDS) { solveEnd("gave up (command limit)"); return; }
  ffFlood();
  const uint16_t here = ffDist[mzY][mzX];
  if (here == FF_UNREACH) { solveEnd("no path to the goal (walls seen + moves the ToF gate refused)"); return; }
  int best = -1; long bestKey = 0;
  for (uint8_t d = 0; d < 4; d++) {                 // downhill neighbour; ties: straight on, closer to the goal, fewer visits
    if (ffBlocked(mzX, mzY, d)) continue;
    const int nx = mzX + DIR_DX[d], ny = mzY + DIR_DY[d];
    if (ffDist[ny][nx] + 1 != here) continue;
    const long key = (d == slLastDir ? 0L : 1000000L) + (long)(abs(nx - GOAL_X) + abs(ny - GOAL_Y)) * 1000L + (long)slVisits[ny][nx] * 4L + d;
    if (best < 0 || key < bestKey) { best = d; bestKey = key; }
  }
  if (best < 0) { solveEnd("no downhill move (map inconsistent)"); return; }   // cannot happen: BFS distances are consistent
  const uint8_t d = (uint8_t)best;
  // run length: keep going straight while the next edge is KNOWN open and the flood keeps going downhill
  uint8_t n = 1;
  const uint8_t cap = slCapGet(mzX, mzY, d);
  int cx = mzX + DIR_DX[d], cy = mzY + DIR_DY[d];
  while (n < cap) {
    if (cx == GOAL_X && cy == GOAL_Y) break;
    if (!edgeOpenKnown(cx, cy, d)) break;
    const int nx = cx + DIR_DX[d], ny = cy + DIR_DY[d];
    if (!inGrid(nx, ny) || ffDist[ny][nx] + 1 != ffDist[cy][cx]) break;
    n++; cx = nx; cy = ny;
  }
  if (g_mzDbg) Serial.printf("PLAN at (%d,%d) dist %u -> %c%u\n", (int)mzX, (int)mzY, (unsigned)here, DIR_CH[d], (unsigned)n);
  slPendDir = d; slPendN = n; slLastDir = d;
  slSeq = mzResultSeq; slWait = true;
  mazeRequest(d, n, "solve");
}

// ---- RC sticks -> cell moves: flick and release. pitch (fwd stick, CH2) = N/S, roll (strafe stick, CH1) = E/W ----
static float fkPeakX = 0.0f, fkPeakY = 0.0f;
static bool  fkArmed = false;                       // sticks seen centred (with signal) since start / signal loss
static void rcFlick() {
  const uint16_t a = rcRead(RC_CH_STRAFE), b = rcRead(RC_CH_FWD);
  if (!a || !b) { fkArmed = false; fkPeakX = fkPeakY = 0.0f; return; }   // no signal: nothing fires
  if (slActive) { fkArmed = false; fkPeakX = fkPeakY = 0.0f; return; }   // solving: sticks ignored
  const float x = SIGN_STRAFE * rcNorm(a), y = SIGN_FWD * rcNorm(b);
  if (fabsf(x) < RC_FLICK_CENTER && fabsf(y) < RC_FLICK_CENTER) {
    const float px = fabsf(fkPeakX), py = fabsf(fkPeakY), pk = fmaxf(px, py);
    if (fkArmed && pk >= RC_FLICK_MIN) {
      const uint8_t d = (py >= px) ? (fkPeakY > 0.0f ? D_N : D_S) : (fkPeakX > 0.0f ? D_E : D_W);
      mazeRequest(d, (RC_FLICK_N2 && pk >= RC_FLICK_FULL) ? 2 : 1, "rc");
    }
    fkArmed = true;
    fkPeakX = fkPeakY = 0.0f;
    return;
  }
  if (fabsf(x) > fabsf(fkPeakX)) fkPeakX = x;
  if (fabsf(y) > fabsf(fkPeakY)) fkPeakY = y;
}
// MAZE_SECTION_END

static void handleLine(char *s) {
  while (*s == ' ') s++;
  if (!strcasecmp(s, "c")) { g_cmdMagCal = true; Serial.println("OK mag cal toggle"); return; }
  if (!strcasecmp(s, "z")) { g_cmdZero = true;   Serial.println("OK zero altitude"); return; }
  if (!strcasecmp(s, "m")) { g_magUse = !g_magUse; Serial.println(g_magUse ? "OK mag on" : "OK mag off (gyro only)"); return; }
  if (!strcasecmp(s, "a")) { g_accCmd = 1; return; }
  if (!strcasecmp(s, "n")) {
    if (!g_accActive) { Serial.println("no accel cal running, send 'a' first"); return; }
    g_accCmd = 2; Serial.println("  sampling ~1 s, hold still...");
    return;
  }
  if (!strcasecmp(s, "l")) { g_cmdLevel = true; Serial.println("OK level cal, keep the robot still on a flat surface (~1 s)"); return; }
  if (!strncasecmp(s, "rc", 2)) {
    if (!g_rcMode) { setVelocities(0, 0, 0); mzLastMotion = millis(); }   // stop the manual motion first
    g_rcMode = true;
    Serial.println("OK RC cell moves (position may be stale after manual driving: BOOTSEL or \"reset\" to restart the map)");
    return;
  }
  if (!strncasecmp(s, "vel", 3)) {
    float vv[3]; int nv = 0;
    char *pv = s + 3;
    while (nv < 3) {
      char *end;
      float f = strtof(pv, &end);
      if (end == pv) break;
      vv[nv++] = f;
      pv = end;
      while (*pv == ',' || *pv == ' ') pv++;
    }
    if (nv == 3) {
      g_rcMode = false;
      setVelocities(vv[0], vv[1], vv[2]);
      Serial.printf("OK vel vx=%.2f vy=%.2f vr=%.2f (RC off, send \"rc\" to resume)\n", vv[0], vv[1], vv[2]);
    } else {
      Serial.println("usage: vel vx,vy,vr   (m/s right+, m/s fwd+, deg/s clockwise+)");
    }
    return;
  }
  if (!strncasecmp(s, "hold", 4)) {
    char *end;
    float f = strtof(s + 4, &end);
    if (end != s + 4) setYawHold(f);
    float h = g_yawHold;
    if (h < 0.0f) Serial.println("hold off");
    else Serial.printf("hold %.1f deg\n", h);
    return;
  }
  if (!strncasecmp(s, "auto", 4)) {
    char *end;
    float f = strtof(s + 4, &end);
    if (end != s + 4) g_autoStrafe = (f != 0.0f);
    Serial.println(g_autoStrafe ? "auto strafe on" : "auto strafe off");
    return;
  }
  if (!strncasecmp(s, "ks", 2)) {
    float kv[2]; int nk = 0;
    char *pk = s + 2;
    while (nk < 2) {
      char *end;
      float f = strtof(pk, &end);
      if (end == pk) break;
      kv[nk++] = f;
      pk = end;
      while (*pk == ',' || *pk == ' ') pk++;
    }
    if (nk == 2) {
      K_SLIP_STRAFE = constrain(kv[0], 0.2f, 1.5f);
      K_SLIP_FWD    = constrain(kv[1], 0.2f, 1.5f);
    } else if (nk != 0) {
      Serial.println("usage: ks strafe,fwd   (no args = print)");
      return;
    }
    Serial.printf("ks strafe=%.3f fwd=%.3f\n", (float)K_SLIP_STRAFE, (float)K_SLIP_FWD);
    return;
  }
  // ---- maze ----
  if (!strcasecmp(s, "map"))   { mazePrint(); return; }
  if (!strcasecmp(s, "win"))   { mazeWinRequest(1, 0, 0.0f); return; }
  if (!strcasecmp(s, "ccal"))  { mazeWinRequest(2, 0, 0.0f); Serial.println("CCAL: robot centred in a closed 4-wall cell, hold still..."); return; }
  if (!strcasecmp(s, "stop"))  { if (slActive) solveEnd("stopped"); mazeStop(); Serial.println("OK stop"); return; }
  if (!strcasecmp(s, "solve")) { if (slActive) solveStopRequest(); else solveStart(); return; }
  if (!strncasecmp(s, "goal", 4)) {
    char *e1, *e2;
    long gx = strtol(s + 4, &e1, 10), gy = strtol(e1, &e2, 10);
    if (e1 != s + 4 && e2 != e1) {
      if (inGrid((int)gx, (int)gy)) { GOAL_X = (int8_t)gx; GOAL_Y = (int8_t)gy; }
      else Serial.printf("goal must be inside the %dx%d grid\n", MAZE_N, MAZE_N);
    }
    Serial.printf("goal (%d,%d)\n", (int)GOAL_X, (int)GOAL_Y);
    return;
  }
  if (!strcasecmp(s, "reset")) {
    if (slActive) solveEnd("stopped");
    mazeStop();
    setYawHold(g_yaw);                                    // new start: N = current forward
    mazeReset();
    Serial.println("OK map cleared, position = start, heading captured");
    return;
  }
  if (!strncasecmp(s, "go", 2)) {                         // go N 2  -> same as an RC flick (n = 1..3) ("goal" is handled above)
    if (slActive) { Serial.println("solving: press BOOTSEL or send \"solve\" / \"stop\" first"); return; }
    char *p = s + 2;
    while (*p == ' ') p++;
    const char *dirs = "NESW";
    const char *hit = (*p) ? strchr(dirs, toupper((unsigned char)*p)) : NULL;
    if (!hit) { Serial.println("usage: go N|E|S|W [n]   (n = 1..3)"); return; }
    long n = strtol(p + 1, NULL, 10);
    if (n < 1) n = 1;
    if (!g_rcMode) { setVelocities(0, 0, 0); mzLastMotion = millis(); g_rcMode = true; }
    mazeRequest((uint8_t)(hit - dirs), (uint8_t)constrain(n, 1L, (long)FREE_MAX), "serial");
    return;
  }
  if (!strncasecmp(s, "tcal", 4)) {                       // tcal | tcal i mm | tcal i fit | tcal i clr   (i: 0 F, 1 B, 2 L, 3 R)
    char *p = s + 4, *end;
    while (*p == ' ') p++;
    if (!*p) { tcalPrint(); return; }
    long i = strtol(p, &end, 10);
    if (end == p || i < 0 || i > 3) { Serial.println("usage: tcal i mm | tcal i fit | tcal i clr   (i: 0 front, 1 back, 2 left, 3 right)"); return; }
    p = end;
    while (*p == ' ' || *p == ',') p++;
    if (!strncasecmp(p, "fit", 3)) { tcalFit((uint8_t)i); return; }
    if (!strncasecmp(p, "clr", 3)) { tcN[i] = 0; Serial.printf("TCAL %c points cleared\n", SENSOR_CH[i]); return; }
    float mm = strtof(p, &end);
    if (end == p || mm <= 0.0f) { Serial.println("usage: tcal i mm   (true distance sensor face -> wall, from the ruler)"); return; }
    mazeWinRequest(3, (uint8_t)i, mm);
    return;
  }
  if (!strncasecmp(s, "tel", 3))   { char *e; float f = strtof(s + 3, &e); if (e != s + 3) g_telOn = (f != 0.0f); Serial.println(g_telOn ? "telemetry on" : "telemetry off"); return; }
  if (!strncasecmp(s, "mzdbg", 5)) { char *e; float f = strtof(s + 5, &e); if (e != s + 5) g_mzDbg = (f != 0.0f); Serial.println(g_mzDbg ? "drive debug on" : "drive debug off"); return; }
  if (!strncasecmp(s, "mv", 2)) {                         // mv kp,vmax,vcreep[,acc[,vmaxEW]]
    float kv[5]; int nk = 0;
    char *pk = s + 2;
    while (nk < 5) {
      char *end;
      float f = strtof(pk, &end);
      if (end == pk) break;
      kv[nk++] = f;
      pk = end;
      while (*pk == ',' || *pk == ' ') pk++;
    }
    if (nk >= 3) {
      MV_KP = constrain(kv[0], 0.5f, 20.0f);
      MV_VMAX = constrain(kv[1], 0.05f, 1.0f);
      MV_VCREEP = constrain(kv[2], 0.0f, MV_VMAX);
      if (nk >= 4) MV_ACC = constrain(kv[3], 0.2f, 10.0f);
      if (nk >= 5) MV_VMAX_EW = constrain(kv[4], 0.05f, 1.0f);
    } else if (nk != 0) {
      Serial.println("usage: mv kp,vmax,vcreep[,acc[,vmaxEW]]   (no args = print)");
      return;
    }
    Serial.printf("mv kp=%.2f vmax=%.3f vcreep=%.3f acc=%.2f vmaxEW=%.3f\n", MV_KP, MV_VMAX, MV_VCREEP, MV_ACC, MV_VMAX_EW);
    return;
  }
  if (!strncasecmp(s, "rpm", 3)) s += 3;
  float v[4]; int n = 0;
  char *p = s;
  while (n < 4) {
    char *end;
    float f = strtof(p, &end);
    if (end == p) break;
    v[n++] = f;
    p = end;
    while (*p == ',' || *p == ' ') p++;
  }
  if (n == 4) {
    g_rcMode = false;
    motorSetAll(v[0], v[1], v[2], v[3]);
    Serial.printf("OK set %.0f %.0f %.0f %.0f (RC off, send \"rc\" to resume)\n", v[0], v[1], v[2], v[3]);
  } else {
    Serial.println("usage: rpm a,b,c,d   (FL,FR,BL,BR; signed, +-500)  |  rc  |  vel vx,vy,vr  |  hold deg (-1 = off)  |  auto 0/1  |  c (mag cal)  |  a/n (accel 6-pos cal)  |  l (level)  |  z (zero alt)");
    Serial.println("maze: solve  |  goal x y  |  go N|E|S|W [n]  |  stop  |  map  |  win  |  reset  |  ccal  |  tcal i mm / tcal i fit / tcal i clr  |  mv kp,vmax,vcreep[,acc]  |  tel 0/1  |  mzdbg 0/1");
  }
}

void setup() {
  Serial.begin(115200);
  rcInit();                    // RC pin interrupts live on core 0 only
}

void loop() {
  // Wait until IMU/sensors are ready, do nothing until button press
  static bool btnPrev = false;
  static uint32_t tBtn = 0;
  static uint32_t tDown = 0;                      // when the current press began
  static bool btnUsed = false;                    // the current press already acted (start / stop / long press)
  if (!g_started) {
    if (g_sensorsReady && g_imuSeq > 0 && (millis() - tBtn >= 50)) {
      tBtn = millis();
      bool b = BOOTSEL;   // or digitalRead(YOUR_BTN_PIN) == LOW
      if (b && !btnPrev) {
        setYawHold(g_yaw);      // capture heading right at press moment
        g_started = true;
        g_posX=0.0f;
        g_posY=0.0f;
        setVelocities(0.0f, 0.0f, 0.0f);
        mazeReset();            // map cleared, position = start cell, start cell walls recorded (keep the robot still)
        solveStart();           // flood fill to the goal (starts once the start cell is recorded)
        btnUsed = true;         // its release does nothing
      }
      btnPrev = b;
    }
    return;   // stay idle/limp while waiting for the button press
  }
  static char buf[64];
  static uint8_t len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len) { buf[len] = 0; handleLine(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = ch;
    }
  }

  // sensor status / events coming from core 1 (core 1 never prints)
  static bool sensorsReported = false;
  if (!sensorsReported && g_sensorsReady) {
    sensorsReported = true;
    Serial.printf("SENSORS: MPU %s (WHO_AM_I 0x%02X) | QMC %s (ID 0x%02X) | BMP180 %s | ToF found 0x%X (bit0=dist1 .. bit3=dist4)\n",
                  g_mpuOk ? "ok" : "MISSING", (unsigned)g_mpuWho, g_qmcOk ? "ok" : "MISSING", (unsigned)g_qmcId,
                  g_bmpOk ? "ok" : "MISSING", (unsigned)g_tofMask);
  }
  uint8_t calEv = g_calEvent;
  if (calEv) {
    g_calEvent = 0;
    if (calEv == 1) Serial.println("MAG CAL: rotate through all orientations for ~25 s ('c' again to stop)");
    else Serial.printf("MAG CAL done. magOff = {%.2f, %.2f, %.2f}; magScale = {%.3f, %.3f, %.3f};\n",
                       magOff[0], magOff[1], magOff[2], magScale[0], magScale[1], magScale[2]);
  }

  uint8_t accEv = g_accEvent;
  if (accEv) {
    g_accEvent = 0;
    uint8_t st = g_accStep;
    if (accEv == 1) Serial.println("ACC CAL: 6 positions, motors off, keep still. 'n' = capture position, 'a' = abort");
    if (accEv == 2 || accEv == 4) Serial.printf("  captured, avg g = {%.3f, %.3f, %.3f}\n", (float)accLast[0], (float)accLast[1], (float)accLast[2]);
    if (accEv == 3) Serial.printf("  REJECTED (moved, or wrong orientation), avg g = {%.3f, %.3f, %.3f} -> repeat this position\n",
                                  (float)accLast[0], (float)accLast[1], (float)accLast[2]);
    if (accEv == 5) Serial.println("ACC CAL aborted");
    if (accEv <= 3) Serial.printf("[%u/6] place robot: %s, then send 'n'\n", (unsigned)(st + 1), ACC_PROMPT[st]);
    if (accEv == 4) Serial.printf("ACC CAL done (active now). accOff = {%.4f, %.4f, %.4f}; accScale = {%.4f, %.4f, %.4f};\n",
                                  accOff[0], accOff[1], accOff[2], accScale[0], accScale[1], accScale[2]);
  }
  if (g_lvlEvent) {
    g_lvlEvent = 0;
    Serial.printf("LEVEL done (active now). rollTrim = %.2f; pitchTrim = %.2f;\n", rollTrim, pitchTrim);
  }

  // RC flicks -> cell moves (pitch = N/S, roll = E/W)
  static uint32_t tRc = 0;
  if (g_rcMode && millis() - tRc >= RC_UPDATE_MS) {
    tRc = millis();
    rcFlick();
  }

  mazeTask();
  solveStep();

  // BOOTSEL: while solving, a press stops (after the current move; again = now). While stopped: short press = solve from
  // here (map kept), hold LONG_PRESS_MS = new start (heading captured, map cleared, position = start cell)
  if (millis() - tBtn >= 50) {
    tBtn = millis();
    bool b = BOOTSEL;
    if (b && !btnPrev) {
      tDown = millis(); btnUsed = false;
      if (slActive) { solveStopRequest(); btnUsed = true; }
    }
    if (b && btnPrev && !btnUsed && !slActive && mazeIdle() && millis() - tDown >= LONG_PRESS_MS) {
      btnUsed = true;
      setYawHold(g_yaw);
      mazeReset();
      Serial.println("BOOTSEL (long): new start, map cleared, heading captured. Short press = solve");
    }
    if (!b && btnPrev && !btnUsed) {              // short press released while stopped: solve from here (map kept)
      if (mazeIdle()) solveStart();
      else Serial.println("BOOTSEL: busy (start cell still being recorded), press again");
    }
    btnPrev = b;
  }

#if TELEMETRY_MS
  static uint32_t tPrint = 0;
  static uint32_t lastLidarSeq = 0;
  static bool     lidarRateInit = false;
  static float    lidarRateHz = 0.0f;

  if (g_telOn && millis() - tPrint >= TELEMETRY_MS) {
    uint32_t now = millis();
    uint32_t dtMs = now - tPrint;
    uint32_t seqNow = g_lidarSeq;

    if (!lidarRateInit) {
      lidarRateInit = true;
      lidarRateHz = 0.0f;
    } else if (dtMs > 0) {
      uint32_t dSeq = seqNow - lastLidarSeq;
      lidarRateHz = (dSeq * 1000.0f) / (float)dtMs;
    }

    lastLidarSeq = seqNow;
    tPrint = now;

    Serial.printf("sp1:%.0f,sp2:%.0f,sp3:%.0f,sp4:%.0f,rpm1:%.1f,rpm2:%.1f,rpm3:%.1f,rpm4:%.1f,hz:%lu,"
                  "roll:%.1f,pitch:%.1f,yaw:%.1f,ax:%.2f,ay:%.2f,az:%.2f,lx:%.2f,ly:%.2f,lz:%.2f,"
                  "alt:%.2f,vz:%.2f,temp:%.1f,pa:%.0f,d1:%d,d2:%d,d3:%d,d4:%d,tof_hz:%.1f\n",
                  (float)g_setpoint[0], (float)g_setpoint[1], (float)g_setpoint[2], (float)g_setpoint[3],
                  (float)g_measured[0], (float)g_measured[1], (float)g_measured[2], (float)g_measured[3], (unsigned long)loopHz,
                  (float)g_roll, (float)g_pitch, (float)g_yaw,
                  (float)g_acc[0], (float)g_acc[1], (float)g_acc[2],
                  (float)g_lin[0], (float)g_lin[1], (float)g_lin[2],
                  (float)g_alt, (float)g_vz, (float)g_tempC, (float)g_pressPa,
                  (int)g_lidar[0], (int)g_lidar[1], (int)g_lidar[2], (int)g_lidar[3],
                  lidarRateHz);
  }
#endif
}