#include "Battery.h"
#include "Utility.h"
#include <Arduino.h>
#include "HardwareConfig.h"

static const uint8_t BATT_SAMPLE_COUNT = 10;
static const uint8_t BATT_TRIM_COUNT = 1;
volatile uint8_t gBattLevel = 0;

void battBegin() {
#if WORKER_POT_COUNT == 1
  // Read before for possible bug: https://community.simplefoc.com/t/pin-is-not-configured-as-analog-channel/6751/5
  analogRead(BATT_PIN);
	analogSetPinAttenuation(BATT_PIN, ADC_11db);
  pinMode(BATT_PIN, INPUT);
#else
  // Read before for possible bug: https://community.simplefoc.com/t/pin-is-not-configured-as-analog-channel/6751/5
  analogRead(BATT_ADC_PIN);
	analogSetPinAttenuation(BATT_ADC_PIN, ADC_11db);
  pinMode(BATT_EN_PIN, OUTPUT);
  pinMode(BATT_ADC_PIN, INPUT);
#endif
  readBattLevel();
}

void readBattLevel() {
  uint16_t samples[BATT_SAMPLE_COUNT];
#if WORKER_POT_COUNT == 1
  for (uint8_t i = 0; i < BATT_SAMPLE_COUNT; i++) {
    samples[i] = analogReadMilliVolts(BATT_PIN);
    vTaskDelay(pdMS_TO_TICKS(20));
#else
    digitalWrite(BATT_EN_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(200)); // settle time
    for (uint8_t i = 0; i < BATT_SAMPLE_COUNT; i++) {
      samples[i] = analogReadMilliVolts(BATT_ADC_PIN);
      vTaskDelay(pdMS_TO_TICKS(20));
    digitalWrite(BATT_EN_PIN, LOW);
#endif
  }
  uint16_t raw = trimmedMean(samples, BATT_SAMPLE_COUNT, BATT_TRIM_COUNT);
  const float ADC_REF = 3.3f;
  const int ADC_MAX = 4095;
  const float VDIV = 2.0f; // assumed divider ratio
  float bat_v = (raw * (ADC_REF / ADC_MAX)) * VDIV;
  // Map typical Li-ion range 3.0V - 4.2V to 0-100%
  gBattLevel = (uint8_t)constrain((int)(((bat_v - 3.0f) / (4.2f - 3.0f)) * 100.0f), 0, 100);
  LOG("Read battery: %d", gBattLevel);
}

uint8_t getBattLevel() {
  return gBattLevel;
}
