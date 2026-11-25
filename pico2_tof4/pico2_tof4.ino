// Pico 2 (arduino-pico): 4x VL53L0X behind a TCA9548A (0x70), channels 7,6,5,4
// Lib: "VL53L0X" by Pololu (Library Manager)
// CSV out: loop_rate,dist1,dist2,dist3,dist4   (dist in mm, -1 = no valid reading)
//   dist1=ch7, dist2=ch6, dist3=ch5, dist4=ch4
// loop_rate = full sweeps (all 4 channels polled) per second.
// Non-blocking: sensors run continuous back-to-back; we only read one when its
// "data ready" flag is set, so nothing ever waits on a measurement.

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#define PIN_SDA 0
#define PIN_SCL 1
#define I2C_HZ  400000            // TCA9548A max is 400 kHz

static const uint8_t  TCA_ADDR = 0x70;
static const uint8_t  NUM = 4;
static const uint8_t  CH[NUM] = {7, 6, 5, 4};
static const uint32_t TIMING_BUDGET_US = 20000;   // minimum the lib allows -> ~50 Hz per sensor
static const uint32_t PRINT_MS = 10;

static VL53L0X tof[NUM];
static bool    alive[NUM];
static int16_t dist[NUM] = {-1, -1, -1, -1};
static int8_t  curCh = -1;

static bool tcaSelect(uint8_t ch) {
  if ((int8_t)ch == curCh) return true;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write((uint8_t)(1u << ch));
  if (Wire.endTransmission() != 0) return false;
  curCh = ch;
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.setClock(I2C_HZ);
  Wire.begin();

  for (uint8_t i = 0; i < NUM; i++) {
    alive[i] = false;
    if (!tcaSelect(CH[i])) { Serial.printf("# TCA ch%u select failed\n", CH[i]); continue; }
    tof[i].setTimeout(100);
    if (!tof[i].init()) { Serial.printf("# VL53L0X on ch%u not found\n", CH[i]); continue; }
    tof[i].setMeasurementTimingBudget(TIMING_BUDGET_US);
    tof[i].startContinuous();         // back-to-back continuous ranging
    alive[i] = true;
  }
  Serial.println("loop_rate,dist1,dist2,dist3,dist4");
}

void loop() {
  static uint32_t loops = 0, tLast = millis();

  for (uint8_t i = 0; i < NUM; i++) {
    if (!alive[i] || !tcaSelect(CH[i])) continue;
    // new sample ready? (bits 2:0 of RESULT_INTERRUPT_STATUS)
    if (tof[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) {
      uint16_t d = tof[i].readRangeContinuousMillimeters();   // data is ready, returns immediately
      dist[i] = tof[i].timeoutOccurred() ? -1 : (int16_t)d;
    }
  }
  loops++;

  uint32_t now = millis();
  if (now - tLast >= PRINT_MS) {
    uint32_t rate = (uint32_t)((uint64_t)loops * 1000 / (now - tLast));
    Serial.printf("%lu,%d,%d,%d,%d\n", (unsigned long)rate, dist[0], dist[1], dist[2], dist[3]);
    loops = 0;
    tLast = now;
  }
}
