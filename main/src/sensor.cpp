#include "Sensor.h"
#include <Arduino.h>
#include "HardwareConfig.h"

volatile int gTankLevel = 0;
volatile bool gTankSensorReady = false;

void sensorBegin() {
  analogSetPinAttenuation(TANK_SENSOR_PIN, ADC_11db);
  pinMode(TANK_SENSOR_PIN, INPUT);
  readTankLevel();
  gTankSensorReady = true;
}

void readTankLevel() {
  gTankLevel = analogReadMilliVolts(TANK_SENSOR_PIN);
}

int getTankLevel() {
  return gTankLevel;
}

bool tankSensorIsReady() {
  return gTankSensorReady;
}

bool isTankLow() {
  return tankSensorIsReady() && getTankLevel() <= TANK_LOW_THRESHOLD_MV;
}

bool isTankSafe() {
  return tankSensorIsReady() && getTankLevel() > TANK_LOW_THRESHOLD_MV;
}
