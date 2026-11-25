// Magnetometer axis-mapping test. STANDALONE: flash this INSTEAD of the robot firmware, reflash the robot after.
// GY-87 (MPU6050 + QMC5883P via MPU bypass) on RP2040, I2C0 GP0/GP1. Serial 115200.
//
//   boot  : keep the board still ~1.5 s (gyro bias), wait ~3 s for the filter to settle
//   's'   : start a 25 s capture. Rotate the board through ALL orientations: flat spins AND tilts/flips
//   'x'   : abort capture
//
// Attitude is gyro+accel only (no mag). Each stored mag sample is rotated into the earth frame
// for all 24 proper axis mappings. The correct mapping gives a CONSTANT earth-frame field (lowest variance).
// Mag calibration (offset/scale) is computed from the same capture, so no existing cal values are needed.
// Body frame F = (fwd, left, up), same as the robot firmware; accel/gyro use imuToBody() below.

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

#define SDA_PIN  0
#define SCL_PIN  1
#define I2C_HZ   400000
#define CAP_MS   25000
#define MAXS     1300        // 50 Hz * 25 s = 1250
#define NMAP     24

static const uint8_t MPU_ADDR = 0x68;
static const uint8_t QMC_ADDR = 0x2C;
static const float G0 = 9.80665f;
static const float DEG2RAD = 0.017453292519943295f;
static const float RAD2DEG = 57.29577951308232f;

// same as robot firmware imuToBody: chip +X = right, +Y = up, +Z = back
static inline void imuToBody(float cx, float cy, float cz, float &fx, float &fy, float &fz) {
  fx = -cz; fy = -cx; fz = cy;
}

static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   // body->earth
static float beta = 1.5f;
static float gyroBias[3] = {0, 0, 0};          // chip frame, rad/s
static uint32_t tImu = 0, tMag = 0, tBoot = 0, tCap = 0;
static bool capturing = false;

static float sm[MAXS][3];                      // raw mag, chip frame, uT
static float sq[MAXS][4];                      // quaternion at that sample
static int   nS = 0;

static int8_t kPerm[NMAP][3], kSgn[NMAP][3];   // fx = kSgn[0]*c[kPerm[0]], fy = ..., fz = ...

// ------------------------------ I2C ------------------------------
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

// ------------------------------ MPU6050 ------------------------------
static bool mpuInit() {
  writeReg(MPU_ADDR, 0x6B, 0x80);   // reset
  delay(100);
  writeReg(MPU_ADDR, 0x6B, 0x01);   // wake, PLL
  delay(10);
  writeReg(MPU_ADDR, 0x6A, 0x00);   // I2C_MST_EN = 0
  writeReg(MPU_ADDR, 0x37, 0x02);   // bypass on -> mag visible on main bus
  writeReg(MPU_ADDR, 0x1A, 0x03);   // DLPF ~44 Hz
  writeReg(MPU_ADDR, 0x19, 0x04);   // 200 Hz
  writeReg(MPU_ADDR, 0x1B, 0x08);   // gyro +-500 dps
  writeReg(MPU_ADDR, 0x1C, 0x08);   // accel +-4 g
  uint8_t who = 0;
  return readRegs(MPU_ADDR, 0x75, &who, 1);
}

static void calibGyro() {
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

// ------------------------------ QMC5883P ------------------------------
static bool qmcInit() {
  uint8_t id = 0;
  if (!readRegs(QMC_ADDR, 0x00, &id, 1)) return false;
  writeReg(QMC_ADDR, 0x0B, 0x80);   // soft reset
  delay(10);
  writeReg(QMC_ADDR, 0x0B, 0x08);   // +-8 G, set/reset on
  writeReg(QMC_ADDR, 0x0A, 0x0B);   // continuous, 100 Hz, OSR 8
  return true;
}

static bool qmcRead(float uT[3]) {
  uint8_t st = 0;
  if (!readRegs(QMC_ADDR, 0x09, &st, 1)) return false;
  if (!(st & 0x01)) return false;
  uint8_t b[6];
  if (!readRegs(QMC_ADDR, 0x01, b, 6)) return false;
  const float k = 100.0f / 3750.0f;
  uT[0] = (int16_t)((b[1] << 8) | b[0]) * k;
  uT[1] = (int16_t)((b[3] << 8) | b[2]) * k;
  uT[2] = (int16_t)((b[5] << 8) | b[4]) * k;
  return true;
}

// ------------------------------ Madgwick IMU (no mag) ------------------------------
static void madgwickImu(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float qd1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qd2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  float qd3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  float qd4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  float an = sqrtf(ax * ax + ay * ay + az * az);
  if (an > 1e-6f) {
    ax /= an; ay /= an; az /= an;
    float _2q0 = 2 * q0, _2q1 = 2 * q1, _2q2 = 2 * q2, _2q3 = 2 * q3;
    float _4q0 = 4 * q0, _4q1 = 4 * q1, _4q2 = 4 * q2, _8q1 = 8 * q1, _8q2 = 8 * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4 * q1q1 * q3 - _2q1 * ax + 4 * q2q2 * q3 - _2q2 * ay;
    float sn = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (sn > 1e-9f) {
      s0 /= sn; s1 /= sn; s2 /= sn; s3 /= sn;
      qd1 -= beta * s0; qd2 -= beta * s1; qd3 -= beta * s2; qd4 -= beta * s3;
    }
  }
  q0 += qd1 * dt; q1 += qd2 * dt; q2 += qd3 * dt; q3 += qd4 * dt;
  float qn = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= qn; q1 /= qn; q2 /= qn; q3 /= qn;
}

static void stepImu(float dt) {
  uint8_t b[14];
  if (!readRegs(MPU_ADDR, 0x3B, b, 14)) return;
  const float ak = G0 / 8192.0f;
  const float gk = DEG2RAD / 65.5f;
  float cax = (int16_t)((b[0] << 8) | b[1]) * ak;
  float cay = (int16_t)((b[2] << 8) | b[3]) * ak;
  float caz = (int16_t)((b[4] << 8) | b[5]) * ak;
  float cgx = (int16_t)((b[8]  << 8) | b[9])  * gk - gyroBias[0];
  float cgy = (int16_t)((b[10] << 8) | b[11]) * gk - gyroBias[1];
  float cgz = (int16_t)((b[12] << 8) | b[13]) * gk - gyroBias[2];
  float a[3], g[3];
  imuToBody(cax, cay, caz, a[0], a[1], a[2]);
  imuToBody(cgx, cgy, cgz, g[0], g[1], g[2]);
  beta = (millis() - tBoot < 3000) ? 1.5f : 0.05f;
  madgwickImu(g[0], g[1], g[2], a[0], a[1], a[2], dt);
}

// ------------------------------ mapping candidates ------------------------------
static void kmapInit() {
  static const int8_t P[6][3] = {{0,1,2},{1,2,0},{2,0,1},{0,2,1},{2,1,0},{1,0,2}};  // first 3 even, last 3 odd
  int n = 0;
  for (int p = 0; p < 6; p++)
    for (int s = 0; s < 8; s++) {
      int s0 = (s & 1) ? -1 : 1, s1 = (s & 2) ? -1 : 1, s2 = (s & 4) ? -1 : 1;
      if (((p < 3) ? 1 : -1) * s0 * s1 * s2 != 1) continue;   // det = +1 only
      for (int i = 0; i < 3; i++) kPerm[n][i] = P[p][i];
      kSgn[n][0] = s0; kSgn[n][1] = s1; kSgn[n][2] = s2;
      n++;
    }
}

static void printMap(int k) {
  Serial.printf("fx = %cc%c; fy = %cc%c; fz = %cc%c;",
                kSgn[k][0] < 0 ? '-' : '+', "xyz"[kPerm[k][0]],
                kSgn[k][1] < 0 ? '-' : '+', "xyz"[kPerm[k][1]],
                kSgn[k][2] < 0 ? '-' : '+', "xyz"[kPerm[k][2]]);
}

// ------------------------------ analysis ------------------------------
static void analyse() {
  capturing = false;
  Serial.printf("\nCapture done: %d samples\n", nS);
  if (nS < 200) { Serial.println("Too few samples (mag not reading?). Check wiring / bypass."); return; }

  // calibration from this capture (min/max, same method as robot firmware)
  float mn[3] = {1e9f, 1e9f, 1e9f}, mx[3] = {-1e9f, -1e9f, -1e9f};
  for (int s = 0; s < nS; s++)
    for (int i = 0; i < 3; i++) {
      if (sm[s][i] < mn[i]) mn[i] = sm[s][i];
      if (sm[s][i] > mx[i]) mx[i] = sm[s][i];
    }
  float off[3], sc[3], half[3], avg = 0;
  for (int i = 0; i < 3; i++) {
    off[i] = 0.5f * (mx[i] + mn[i]);
    half[i] = 0.5f * (mx[i] - mn[i]);
    if (half[i] < 1e-3f) half[i] = 1e-3f;
    avg += half[i];
  }
  avg /= 3.0f;
  for (int i = 0; i < 3; i++) sc[i] = avg / half[i];
  Serial.printf("chip-axis range uT: x %.1f  y %.1f  z %.1f   (each should be ~60-100; < 30 = not rotated enough)\n",
                mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]);
  Serial.printf("cal from this run: magOff = {%.2f, %.2f, %.2f}; magScale = {%.3f, %.3f, %.3f};\n",
                off[0], off[1], off[2], sc[0], sc[1], sc[2]);

  // earth-frame field for every candidate mapping
  double sum[NMAP][3], sqs[NMAP][3];
  memset(sum, 0, sizeof(sum));
  memset(sqs, 0, sizeof(sqs));
  for (int s = 0; s < nS; s++) {
    float c[3];
    for (int i = 0; i < 3; i++) c[i] = (sm[s][i] - off[i]) * sc[i];
    float a = sq[s][0], b = sq[s][1], cc = sq[s][2], d = sq[s][3];
    float R[9] = {
      1 - 2 * (cc * cc + d * d), 2 * (b * cc - a * d),     2 * (b * d + a * cc),
      2 * (b * cc + a * d),      1 - 2 * (b * b + d * d),  2 * (cc * d - a * b),
      2 * (b * d - a * cc),      2 * (cc * d + a * b),     1 - 2 * (b * b + cc * cc) };
    for (int k = 0; k < NMAP; k++) {
      float b0 = kSgn[k][0] * c[kPerm[k][0]];
      float b1 = kSgn[k][1] * c[kPerm[k][1]];
      float b2 = kSgn[k][2] * c[kPerm[k][2]];
      for (int i = 0; i < 3; i++) {
        float e = R[3 * i] * b0 + R[3 * i + 1] * b1 + R[3 * i + 2] * b2;
        sum[k][i] += e;
        sqs[k][i] += (double)e * e;
      }
    }
  }
  float var[NMAP];
  for (int k = 0; k < NMAP; k++) {
    double v = 0;
    for (int i = 0; i < 3; i++) { double m = sum[k][i] / nS; v += sqs[k][i] / nS - m * m; }
    var[k] = (float)v;
  }

  // top 3 + current
  int idx[NMAP];
  for (int k = 0; k < NMAP; k++) idx[k] = k;
  for (int i = 0; i < 3; i++)
    for (int j = i + 1; j < NMAP; j++)
      if (var[idx[j]] < var[idx[i]]) { int t = idx[i]; idx[i] = idx[j]; idx[j] = t; }
  int cur = -1;
  for (int k = 0; k < NMAP; k++)
    if (kPerm[k][0] == 2 && kPerm[k][1] == 0 && kPerm[k][2] == 1 &&
        kSgn[k][0] == -1 && kSgn[k][1] == -1 && kSgn[k][2] == 1) cur = k;   // robot's current magToBody

  Serial.println("\nvariance of earth-frame field (uT^2), lower = more constant = better:");
  for (int i = 0; i < 3; i++) {
    Serial.printf("  #%d  %8.1f   ", i + 1, var[idx[i]]);
    printMap(idx[i]);
    Serial.println(idx[i] == cur ? "   <-- your current mapping" : "");
  }
  if (cur >= 0) {
    Serial.printf("  cur %8.1f   ", var[cur]);
    printMap(cur);
    Serial.println();
  }

  int best = idx[0];
  float ratio = var[idx[1]] / (var[best] > 1e-6f ? var[best] : 1e-6f);
  float ex = (float)(sum[best][0] / nS), ey = (float)(sum[best][1] / nS), ez = (float)(sum[best][2] / nS);
  float hz = sqrtf(ex * ex + ey * ey);
  Serial.printf("\nbest mean earth field: x %.1f  y %.1f  z(up) %.1f uT, |B| %.1f, dip %.0f deg\n",
                ex, ey, ez, sqrtf(ex * ex + ey * ey + ez * ez), atan2f(-ez, hz) * RAD2DEG);
  Serial.println("(northern hemisphere: z(up) should be NEGATIVE, field points down; dip ~ +40 in Delhi region)");
  Serial.printf("runner-up / best variance = %.1f  ->  %s\n", ratio,
                ratio > 3.0f ? "CLEAR winner" : "AMBIGUOUS: rotate again, include more tilts/flips");
  if (best == cur) Serial.println("Your current magToBody is already the best -> mapping is NOT the problem (suspect calibration / interference).");
  else { Serial.print("Use in magToBody():  "); printMap(best); Serial.println(); }
  Serial.println("\nSend 's' to run again.");
}

static void startCapture() {
  if (millis() - tBoot < 3000) { Serial.println("wait ~3 s after boot for the filter to settle"); return; }
  nS = 0;
  capturing = true;
  tCap = millis();
  tMag = millis();
  Serial.println("CAPTURE 25 s: rotate through ALL orientations (spins + tilts + flips)...");
}

// ------------------------------ main ------------------------------
void setup() {
  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && millis() - t < 3000) {}
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.setClock(I2C_HZ);
  Wire.begin();
  bool ok1 = mpuInit();
  delay(20);
  bool ok2 = qmcInit();
  Serial.printf("MPU %s | QMC %s\n", ok1 ? "ok" : "MISSING", ok2 ? "ok" : "MISSING");
  Serial.println("keep still, calibrating gyro...");
  calibGyro();
  kmapInit();
  tBoot = millis();
  tImu = micros();
  Serial.println("ready. send 's' to start capture");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S') { if (!capturing) startCapture(); }
    else if (c == 'x' || c == 'X') { if (capturing) { capturing = false; Serial.println("aborted"); } }
  }

  uint32_t nu = micros();
  if ((uint32_t)(nu - tImu) >= 5000) {
    float dt = (nu - tImu) * 1e-6f;
    tImu = nu;
    stepImu(dt);
  }

  if (capturing && millis() - tMag >= 20) {
    tMag = millis();
    float m[3];
    if (nS < MAXS && qmcRead(m)) {
      for (int i = 0; i < 3; i++) sm[nS][i] = m[i];
      sq[nS][0] = q0; sq[nS][1] = q1; sq[nS][2] = q2; sq[nS][3] = q3;
      nS++;
    }
    static uint32_t tProg = 0;
    if (millis() - tProg >= 5000) {
      tProg = millis();
      Serial.printf("  %lu s, %d samples\n", (unsigned long)((millis() - tCap) / 1000), nS);
    }
    if (millis() - tCap > CAP_MS || nS >= MAXS) analyse();
  }
}