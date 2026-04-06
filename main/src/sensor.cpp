#include "Sensor.h"
#include <Arduino.h>

static const int TANK_PIN = 3;
volatile int gTankLevel = 0;

void sensorBegin() {
  pinMode(TANK_PIN, INPUT);
  gTankLevel = analogRead(TANK_PIN);
}

void readTankLevel() {
  gTankLevel = analogRead(TANK_PIN);
}

int getTankLevel() {
  return gTankLevel;
}
