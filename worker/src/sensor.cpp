#include "Sensor.h"
#include <Arduino.h>

static const int SOIL_PIN = 3;

void sensorBegin() {
  pinMode(SOIL_PIN, INPUT);
}

int readSoilMoisture() {
  int gSoilMoisture = analogRead(SOIL_PIN);
  return gSoilMoisture;
}
