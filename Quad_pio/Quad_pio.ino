// 4-motor closed-loop RPM controller, RP2040 (arduino-pico / Earle Philhower core)
// + RC PWM input -> mecanum mixing -> per-wheel rpm setpoints
//
//   core 0: setup()/loop()   -> serial test input + RC read / mecanum mix, calls motorSetRPM()
//   core 1: setup1()/loop1() -> PIO encoders + RC read + PID + PWM/direction pins
//
// Easy API (callable from core 0, or anywhere):
//   motorSetRPM(m, rpm)         m = 0..3  (FL, FR, BL, BR), rpm signed, +CW / -CCW
//   motorSetAll(a, b, c, d)
//   motorGetRPM(m)             filtered measured rpm
//   rcRead(ch)                 RC pulse width in us (0 = no signal)
//
// Serial (115200):  "rpm 100,200,300,400" -> manual wheel rpm (RC control off)
//                   "rc"                  -> back to RC control

#include <Arduino.h>
#include <strings.h>
#include "hardware/pio.h"

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

// ===================== SHARED STATE =====================
// 32-bit aligned float accesses are atomic on RP2040, so plain volatiles are fine.
static volatile float g_setpoint[N_MOTORS];
static volatile float g_measured[N_MOTORS];
static volatile bool  g_rcMode = true;             // true = RC drives the wheels, false = serial "rpm" command

void motorSetRPM(uint8_t m, float rpm) {
  if (m < N_MOTORS) g_setpoint[m] = constrain(rpm, -RPM_MAX, RPM_MAX);
}
void motorSetAll(float a, float b, float c, float d) {
  motorSetRPM(0, a); motorSetRPM(1, b); motorSetRPM(2, c); motorSetRPM(3, d);
}
float motorGetRPM(uint8_t m) { return (m < N_MOTORS) ? g_measured[m] : 0.0f; }

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
  for (int i = 0; i < N_MOTORS; i++) lastCount[i] = encCount(i);
}

void loop1() {
  static uint32_t tLast = micros();
  uint32_t now = micros();
  if ((uint32_t)(now - tLast) < PID_PERIOD_US) return;
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
static void handleLine(char *s) {
  while (*s == ' ') s++;
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
    Serial.println("usage: rpm a,b,c,d   (FL,FR,BL,BR; signed, +-500)  |  rc");
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

  static uint32_t tRc = 0;
  if (g_rcMode && millis() - tRc >= RC_UPDATE_MS) {
    tRc = millis();
    rcControl();
  }

#if TELEMETRY_MS
  static uint32_t tPrint = 0;
  if (millis() - tPrint >= TELEMETRY_MS) {
    tPrint = millis();
    Serial.printf("SP %.0f %.0f %.0f %.0f | RPM %.1f %.1f %.1f %.1f | LOOP HZ %.1f\n",
                  (float)g_setpoint[0], (float)g_setpoint[1], (float)g_setpoint[2], (float)g_setpoint[3],
                  (float)g_measured[0], (float)g_measured[1], (float)g_measured[2], (float)g_measured[3], (float)loopHz);
  }
#endif
}