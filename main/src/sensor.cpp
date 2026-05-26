#include "Sensor.h"
#include <Arduino.h>
#include "HardwareConfig.h"

volatile int gTankLevel = 0;

void sensorBegin() {
  analogSetPinAttenuation(TANK_SENSOR_PIN, ADC_11db);
  pinMode(TANK_SENSOR_PIN, INPUT);
  readTankLevel();
}

void readTankLevel() {
  gTankLevel = analogReadMilliVolts(TANK_SENSOR_PIN);
}

int getTankLevel() {
  return gTankLevel;
}
