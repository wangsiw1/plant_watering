#include "Sensor.h"
#include "Utility.h"
#include <Arduino.h>

static const int SOIL_PIN = 3;
static const uint8_t SOIL_SAMPLE_COUNT = 10;
static const uint8_t SOIL_TRIM_COUNT = 1;
volatile uint16_t gSoilMoisture = 0;

void sensorBegin() {
  pinMode(SOIL_PIN, INPUT);
  readSoilMoisture();
}

void readSoilMoisture() {
  uint16_t samples[SOIL_SAMPLE_COUNT];
  for (uint8_t i = 0; i < SOIL_SAMPLE_COUNT; i++) {
    samples[i] = analogRead(SOIL_PIN);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  gSoilMoisture = trimmedMean(samples, SOIL_SAMPLE_COUNT, SOIL_TRIM_COUNT);
  // LOG("Read Soil Moisture: %d", gSoilMoisture);
}

uint16_t getSoilMoisture() {
  return gSoilMoisture;
}
