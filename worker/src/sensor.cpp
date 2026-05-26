#include "Sensor.h"
#include "Utility.h"
#include <Arduino.h>
#include "HardwareConfig.h"

static const uint8_t SOIL_SAMPLE_COUNT = 10;
static const uint8_t SOIL_TRIM_COUNT = 1;
volatile uint16_t gSoilMoisture[WORKER_POT_COUNT];

void sensorBegin() {
#if WORKER_POT_COUNT == 1
  // Read before for possible bug: https://community.simplefoc.com/t/pin-is-not-configured-as-analog-channel/6751/5
  analogRead(SOIL_PIN);
	analogSetPinAttenuation(SOIL_PIN, ADC_11db);
  pinMode(SOIL_PIN, INPUT);
#else
  // Read before for possible bug: https://community.simplefoc.com/t/pin-is-not-configured-as-analog-channel/6751/5
  analogRead(MUX_ADC_PIN);
	analogSetPinAttenuation(MUX_ADC_PIN, ADC_11db);
  pinMode(SOIL_EN_PIN, OUTPUT);
  pinMode(MUX_ADC_PIN, INPUT);
  pinMode(MUX_SEL_PIN0, OUTPUT);
  pinMode(MUX_SEL_PIN1, OUTPUT);
  pinMode(MUX_SEL_PIN2, OUTPUT);
#endif
  readSoilMoisture();
}

void readSoilMoisture() {
  for (size_t ch = 0; ch < WORKER_POT_COUNT; ++ch) {
    uint16_t samples[SOIL_SAMPLE_COUNT];
#if WORKER_POT_COUNT == 1
    for (uint8_t i = 0; i < SOIL_SAMPLE_COUNT; i++) {
      samples[i] = analogReadMilliVolts(SOIL_PIN);
      vTaskDelay(pdMS_TO_TICKS(20));
    }
#else
    digitalWrite(SOIL_EN_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(10)); // settle time
    // select multiplexer channel
    digitalWrite(MUX_SEL_PIN0, (ch >> 0) & 0x1);
    digitalWrite(MUX_SEL_PIN1, (ch >> 1) & 0x1);
    digitalWrite(MUX_SEL_PIN2, (ch >> 2) & 0x1);
    vTaskDelay(pdMS_TO_TICKS(10)); // settle time
    for (uint8_t i = 0; i < SOIL_SAMPLE_COUNT; i++) {
      samples[i] = analogReadMilliVolts(MUX_ADC_PIN);
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    digitalWrite(SOIL_EN_PIN, LOW);
#endif
    gSoilMoisture[ch] = trimmedMean(samples, SOIL_SAMPLE_COUNT, SOIL_TRIM_COUNT);
  }
}

uint16_t getSoilMoisture(size_t idx) {
  if (idx >= WORKER_POT_COUNT) return 0;
  return gSoilMoisture[idx];
}
