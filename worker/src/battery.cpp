#include "Battery.h"
#include <Arduino.h>

static const int BATT_PIN = 4;

void battBegin() {
  pinMode(BATT_PIN, INPUT);
}

int readBattLevel() {
  // battery measurement: assume simple resistor divider to keep ADC input <= Vref.
  int raw = analogRead(BATT_PIN);
  const float ADC_REF = 3.3f;
  const int ADC_MAX = 4095;
  const float VDIV = 2.0f; // assumed divider ratio
  float bat_v = (raw * (ADC_REF / ADC_MAX)) * VDIV;
  // Map typical Li-ion range 3.0V - 4.2V to 0-100%
  uint8_t gBattLevel = (uint8_t)constrain((int)(((bat_v - 3.0f) / (4.2f - 3.0f)) * 100.0f), 0, 100);

  return gBattLevel;
}
