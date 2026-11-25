// 4-motor closed-loop RPM controller, RP2040 (arduino-pico / Earle Philhower core)
// + RC PWM input -> mecanum mixing -> per-wheel rpm setpoints
//
//   core 0: setup()/loop()   -> serial test input + RC read / mecanum mix, calls motorSetRPM()
//   core 1: setup1()/loop1() -> PIO encoders + RC read + PID + PWM/direction pins
//                               + GY-87 AHRS + 4x VL53L0X (I2C0, GP0/GP1), run in the idle time between PID ticks
//
// Easy API (callable from core 0, or anywhere):
//   motorSetRPM(m, rpm)         m = 0..3  (FL, FR, BL, BR), rpm signed, +CW / -CCW
//   motorSetAll(a, b, c, d)
//   motorGetRPM(m)             filtered measured rpm
//   rcRead(ch)                 RC pulse width in us (0 = no signal)
//   lidarGet(i)                i = 0..3 -> dist1..dist4 in mm, -1 = no valid reading
//   IMU: g_roll g_pitch g_yaw (deg), g_acc[3] g_lin[3] (m/s2, x right / y up / z fwd), g_alt g_vz, g_tempC g_pressPa
//
// Serial (115200):  "rpm 100,200,300,400" -> manual wheel rpm (RC control off)
//                   "rc"                  -> back to RC control
//                   "c"                   -> start/stop magnetometer calibration (rotate in all directions)
//                   "a"                   -> start/abort 6-position accelerometer calibration ("n" = capture each position)
//                   "n"                   -> capture current position during accel calibration
//                   "l"                   -> level calibration (robot on a flat surface) -> roll/pitch trim
//                   "z"                   -> re-zero baro altitude

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

// order: a=FL, b=FR, c=BL, d=BR.  encB is always encA+1
struct MotorPins { uint8_t cw, ccw, en, encA; };
static const MotorPins MOT[N_MOTORS] = {
  //  CW  CCW  EN  ENC_A
  {   26,  28, 16,  8 },   // FL  (ENC_B = 9)
  {   11,  10, 15,  6 },   // FR  (ENC_B = 7)
  {   22,  27, 17,  2 },   // BL  (ENC_B = 3)
  {   13,  12, 14,  4 },   // BR  (ENC_B = 5)
};

// x4 decoding -> counts per OUTPUT-shaft rev = encoder PPR * 4 * gear ratio = 3 * 4 * 24
static const float  COUNTS_PER_REV = 672.0f;
// flip to -1 for a motor if measured rpm is negative while driving CW
static const int8_t ENC_DIR[N_MOTORS] = { 1, -1, 1, -1 };

static const float    RPM_MAX        = 520.0f;
static const float    MIN_DUTY       = 45.0f;      // duty (of 255) where the motor starts to move
static const uint32_t PWM_FREQ       = 20000;      // Hz
static const uint32_t PID_PERIOD_US  = 2000;      // 1000 Hz loop
static const float    RPM_FILT_ALPHA = 0.3f;       // measured-rpm EMA (1 = no filter)

// PID gains in duty(0-255) per rpm. Starting values, tune.
static float KP = 0.40f;
static float KI = 4.00f;
static float KD = 0.00f;
static const float INTEG_LIM = 80.0f;              // max duty contribution of I term

#define TELEMETRY_MS 200                           // 0 = off

// ---------- RC input ----------
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

// ---------- mecanum mapping ----------
#define RC_CH_STRAFE 1             // sideways   (stick right = +)
#define RC_CH_FWD    2             // forward    (stick up    = +)
#define RC_CH_ROT    4             // rotation   (stick right = +, clockwise)
#define SIGN_STRAFE  1.0f          // flip to -1.0f if a channel moves the wrong way
#define SIGN_FWD     1.0f
#define SIGN_ROT     1.0f

// wheel rpm at full stick (all axes at 100% is scaled down so no wheel exceeds this).
// keep it below what the slowest motor can actually reach, or the wheels saturate and the motion skews.
static const float DRIVE_MAX_RPM = 720.0f;
static const float    BRAKE_DUTY     = 255.0f;     // brake strength when setpoint = 0 (0 = coast)
static const float    BRAKE_MARGIN_RPM = 20.0f;    // brake when measured speed exceeds setpoint by this much (same direction)
// sign so that a positive wheel value = robot moves forward. flip a wheel if it spins the wrong way.
static const int8_t WHEEL_DIR[N_MOTORS] = { 1, 1, 1, 1 };  // 1 -1 1 -1 = right wrong way

uint32_t loopHz=0;

#define RC_UPDATE_MS 20            // RC -> setpoint update rate (50 Hz)

// ---------- sensors (I2C0) ----------
#define SENS_SDA_PIN 0
#define SENS_SCL_PIN 1
#define SENS_I2C_HZ  400000        // TCA9548A max is 400 kHz
#define SENSOR_GUARD_US 900        // a sensor job only starts if at least this much time is left before the next PID tick

// ===================== SHARED STATE =====================
// 32-bit aligned float accesses are atomic on RP2040, so plain volatiles are fine.
static volatile float g_setpoint[N_MOTORS];
static volatile float g_measured[N_MOTORS];
static volatile bool  g_rcMode = true;             // true = RC drives the wheels, false = serial "rpm" command

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
static volatile uint32_t g_lidarSeq;                      // +1 on every new lidar sample (any sensor)
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

void motorSetRPM(uint8_t m, float rpm) {
  if (m < N_MOTORS) g_setpoint[m] = constrain(rpm, -RPM_MAX, RPM_MAX);
}
void motorSetAll(float a, float b, float c, float d) {
  motorSetRPM(0, a); motorSetRPM(1, b); motorSetRPM(2, c); motorSetRPM(3, d);
}
float motorGetRPM(uint8_t m) { return (m < N_MOTORS) ? g_measured[m] : 0.0f; }
int16_t lidarGet(uint8_t i) { return (i < 4) ? g_lidar[i] : (int16_t)-1; }

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

// ===================== MECANUM MIX =====================
// vx = strafe right, vy = forward, w = rotate clockwise, each -1..1
static void mecanumDrive(float vx, float vy, float w) {
  float m[N_MOTORS] = {
    vy + vx + w,   // FL
    vy - vx - w,   // FR
    vy - vx + w,   // BL
    vy + vx - w,   // BR
  };
  float mx = 1.0f;
  for (int i = 0; i < N_MOTORS; i++) mx = max(mx, fabsf(m[i]));
  for (int i = 0; i < N_MOTORS; i++)
    motorSetRPM(i, (m[i] / mx) * WHEEL_DIR[i] * DRIVE_MAX_RPM);
}

static void rcControl() {
  uint16_t a = rcRead(RC_CH_STRAFE);
  uint16_t b = rcRead(RC_CH_FWD);
  uint16_t r = rcRead(RC_CH_ROT);
  if (!a || !b || !r) { motorSetAll(0, 0, 0, 0); return; }   // failsafe: signal lost -> stop
  mecanumDrive(SIGN_STRAFE * rcNorm(a), SIGN_FWD * rcNorm(b), SIGN_ROT * rcNorm(r));
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
static const float BETA_RUN   = 0.05f;      // Madgwick gain in steady state
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
      g_lidarSeq = g_lidarSeq + 1;
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

  g_mpuOk = mpuInit();
  delay(20);
  g_qmcOk = qmcInit();
  g_bmpOk = bmpInit();
  mpuCalibGyro();                              // ~1.2 s, keep the robot still at power-up

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
  if (millis() - tMag >= 10) { stepMag(); return; }
  if (g_bmpOk && stepBaro()) return;
  stepTof();
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
  rcInit();
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
    g_rcMode = true;
    Serial.println("OK RC control");
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
    Serial.println("usage: rpm a,b,c,d   (FL,FR,BL,BR; signed, +-500)  |  rc  |  c (mag cal)  |  a/n (accel 6-pos cal)  |  l (level)  |  z (zero alt)");
  }
}

void setup() {
  Serial.begin(115200);
}

void loop() {
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

  static uint32_t tRc = 0;
  if (g_rcMode && millis() - tRc >= RC_UPDATE_MS) {
    tRc = millis();
    rcControl();
  }

#if TELEMETRY_MS
  static uint32_t tPrint = 0;
  if (millis() - tPrint >= TELEMETRY_MS) {
    tPrint = millis();
    Serial.printf("sp1:%.0f,sp2:%.0f,sp3:%.0f,sp4:%.0f,rpm1:%.1f,rpm2:%.1f,rpm3:%.1f,rpm4:%.1f,hz:%lu,"
                  "roll:%.1f,pitch:%.1f,yaw:%.1f,ax:%.2f,ay:%.2f,az:%.2f,lx:%.2f,ly:%.2f,lz:%.2f,"
                  "alt:%.2f,vz:%.2f,temp:%.1f,pa:%.0f,d1:%d,d2:%d,d3:%d,d4:%d\n",
                  (float)g_setpoint[0], (float)g_setpoint[1], (float)g_setpoint[2], (float)g_setpoint[3],
                  (float)g_measured[0], (float)g_measured[1], (float)g_measured[2], (float)g_measured[3], (unsigned long)loopHz,
                  (float)g_roll, (float)g_pitch, (float)g_yaw,
                  (float)g_acc[0], (float)g_acc[1], (float)g_acc[2],
                  (float)g_lin[0], (float)g_lin[1], (float)g_lin[2],
                  (float)g_alt, (float)g_vz, (float)g_tempC, (float)g_pressPa,
                  (int)g_lidar[0], (int)g_lidar[1], (int)g_lidar[2], (int)g_lidar[3]);
  }
#endif
}