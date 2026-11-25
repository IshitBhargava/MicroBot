#include <Arduino.h>

// BOOTSEL button test - Pico 2 (RP2350), arduino-pico core
// Press BOOTSEL -> LED on + serial message.

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  static bool last = false;
  bool now = BOOTSEL;  // core-provided global, reads QSPI CS pin

  if (now != last) {
    last = now;
    digitalWrite(LED_BUILTIN, now);
    Serial.println(now ? "BOOTSEL pressed" : "BOOTSEL released");
  }
  delay(10);
}