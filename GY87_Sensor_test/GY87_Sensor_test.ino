// Pico 2 (RP2350) AHRS: MPU6050 + QMC5883P (0x2C, via MPU aux bypass) + BMP180
// Core: arduino-pico (Earle Philhower). I2C0 on GP0 (SDA) / GP1 (SCL).
//
// Output frame (yours): x = right, y = up, z = forward.
// Angles: roll  = right side down positive (about fwd axis)
//         pitch = nose up positive        (about right axis)
//         yaw   = compass heading, clockwise from north, 0..360
//
// Serial commands: 'c' = start/stop magnetometer calibration (rotate in all directions)
//                  'z' = re-zero baro altitude

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// must live above all functions, or the IDE's auto-prototypes hit "'Rot' does not name a type"
struct Rot { float r11, r21, r31, r32, r33; };

// ------------------------------ config ------------------------------
#define PIN_SDA 0
#define PIN_SCL 1
#define I2C_HZ  400000

static const uint8_t MPU_ADDR = 0x68;
static const uint8_t QMC_ADDR = 0x2C;
static const uint8_t BMP_ADDR = 0x77;

static const float DECLINATION_DEG = 1.0f;  // magnetic declination, Faridabad ~ +1 deg E
static const float BETA_START = 1.5f;       // Madgwick gain during initial convergence
static const float BETA_RUN   = 0.05f;      // Madgwick gain in steady state
static const uint32_t CONVERGE_MS = 2500;
static const float G0 = 9.80665f;
static const uint8_t BMP_OSS = 3;
static const uint32_t PRINT_MS = 50;

// Mag hard/soft-iron calibration (chip frame, uT). Run 'c' and paste the printed values here.
static float magOff[3]   = {0.0f, 0.0f, 0.0f};
static float magScale[3] = {1.0f, 1.0f, 1.0f};

static const float DEG2RAD = 0.017453292519943295f;
static const float RAD2DEG = 57.29577951308232f;

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
  fx = -cz; fy = -cx; fz = cy;
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

static uint32_t tImu = 0, tMag = 0, tPrint = 0, tStart = 0;
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
  Serial.printf("MPU WHO_AM_I = 0x%02X\n", who);
  return true;
}

static void mpuCalibGyro() {
  Serial.println("Gyro bias calibration, keep still...");
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
  Serial.printf("QMC chip ID = 0x%02X (QMC5883P expects 0x80)\n", id);
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
  Serial.println("MAG CAL: rotate through all orientations for ~25 s ('c' again to stop)");
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
  Serial.printf("MAG CAL done. magOff = {%.2f, %.2f, %.2f}; magScale = {%.3f, %.3f, %.3f};\n",
                magOff[0], magOff[1], magOff[2], magScale[0], magScale[1], magScale[2]);
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

static void stepBaro() {
  uint32_t ms = millis();
  switch (bState) {
    case 0:
      if (writeReg(BMP_ADDR, 0xF4, 0x2E)) { tBmp = ms; bState = 1; }
      break;
    case 1:
      if (ms - tBmp >= 5) {
        uint8_t d[2];
        if (readRegs(BMP_ADDR, 0xF6, d, 2)) {
          bUT = (d[0] << 8) | d[1];
          writeReg(BMP_ADDR, 0xF4, 0x34 | (BMP_OSS << 6));
          tBmp = ms; bState = 2;
        } else bState = 0;
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
      }
      break;
  }
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

  const float ak = G0 / 8192.0f;
  const float gk = DEG2RAD / 65.5f;
  float cax = rax * ak, cay = ray * ak, caz = raz * ak;
  float cgx = rgx * gk - gyroBias[0], cgy = rgy * gk - gyroBias[1], cgz = rgz * gk - gyroBias[2];

  float gF[3];
  imuToBody(cax, cay, caz, aF[0], aF[1], aF[2]);
  imuToBody(cgx, cgy, cgz, gF[0], gF[1], gF[2]);

  beta = (millis() - tStart < CONVERGE_MS) ? BETA_START : BETA_RUN;
  if (magValid) madgwick(gF[0], gF[1], gF[2], aF[0], aF[1], aF[2], mF[0], mF[1], mF[2], dt);

  // vertical channel: predict with gravity-removed earth-up accel
  Rot r = getRot();
  float aUp = r.r31 * aF[0] + r.r32 * aF[1] + r.r33 * aF[2] - G0;
  fAlt += fVz * dt + 0.5f * aUp * dt * dt;
  fVz  += aUp * dt;
}

// ------------------------------ setup / loop ------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.setClock(I2C_HZ);
  Wire.begin();

  if (!mpuInit())  Serial.println("MPU6050 not found!");
  delay(20);
  if (!qmcInit())  Serial.println("QMC mag @0x2C not found (is bypass on? check wiring)");
  if (!bmpInit())  Serial.println("BMP180 not found!");

  mpuCalibGyro();
  tStart = millis();
  tImu = micros();
  Serial.println("Running. 'c' = mag cal, 'z' = zero altitude");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'c') { if (calibrating) magCalFinish(); else magCalStart(); }
    if (c == 'z') { nRef = 0; pSum = 0; fAlt = 0; fVz = 0; }
  }

  uint32_t nowU = micros();
  if (nowU - tImu >= 5000) {            // 200 Hz
    float dt = (nowU - tImu) * 1e-6f;
    tImu = nowU;
    stepImu(dt);
  }
  stepMag();
  stepBaro();

  uint32_t ms = millis();
  if (ms - tPrint >= PRINT_MS) {
    tPrint = ms;
    Rot r = getRot();

    float roll  = atan2f(r.r32, r.r33) * RAD2DEG;                       // right side down +
    float s     = r.r31; if (s > 1) s = 1; if (s < -1) s = -1;
    float pitch = asinf(s) * RAD2DEG;                                   // nose up +
    float yaw   = -atan2f(r.r21, r.r11) * RAD2DEG + DECLINATION_DEG;    // clockwise from north
    while (yaw < 0) yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;

    // accel in your frame (x right, y up, z fwd): raw specific force and gravity-removed
    float ux, uy, uz;
    bodyToUser(aF[0], aF[1], aF[2], ux, uy, uz);
    float lF[3] = { aF[0] - G0 * r.r31, aF[1] - G0 * r.r32, aF[2] - G0 * r.r33 };
    float lx, ly, lz;
    bodyToUser(lF[0], lF[1], lF[2], lx, ly, lz);

    Serial.printf("RPY[deg] R=%7.2f P=%7.2f Y=%6.1f | acc[m/s2] x=%6.2f y=%6.2f z=%6.2f | lin x=%6.2f y=%6.2f z=%6.2f | alt=%6.2fm vz=%5.2f | %.1fC %.0fPa%s\n",
                  roll, pitch, yaw, ux, uy, uz, lx, ly, lz, fAlt, fVz, bTempC, bPa,
                  calibrating ? " [MAG CAL]" : "");
  }
}