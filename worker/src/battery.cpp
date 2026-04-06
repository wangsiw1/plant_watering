#include "Battery.h"
#include "Utility.h"
#include <Arduino.h>

static const int BATT_PIN = 4;
static const uint8_t BATT_SAMPLE_COUNT = 10;
static const uint8_t BATT_TRIM_COUNT = 1;
volatile uint8_t gBattLevel = 0;

void battBegin() {
  pinMode(BATT_PIN, INPUT);
  readBattLevel();
}

void readBattLevel() {
  uint16_t samples[BATT_SAMPLE_COUNT];
  for (uint8_t i = 0; i < BATT_SAMPLE_COUNT; i++) {
    samples[i] = analogRead(BATT_PIN);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  uint16_t raw = trimmedMean(samples, BATT_SAMPLE_COUNT, BATT_TRIM_COUNT);
  const float ADC_REF = 3.3f;
  const int ADC_MAX = 4095;
  const float VDIV = 2.0f; // assumed divider ratio
  float bat_v = (raw * (ADC_REF / ADC_MAX)) * VDIV;
  // Map typical Li-ion range 3.0V - 4.2V to 0-100%
  gBattLevel = (uint8_t)constrain((int)(((bat_v - 3.0f) / (4.2f - 3.0f)) * 100.0f), 0, 100);
  // LOG("Read battery: %d", gBattLevel);
}

uint8_t getBattLevel() {
  return gBattLevel;
}
