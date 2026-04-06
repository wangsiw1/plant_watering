#include "Valve.h"
#include "Utility.h"
#include <Arduino.h>

static const int VALVE_PIN = 1;

void valveBegin() {
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(VALVE_PIN, LOW);
}

void valveOn() {
  LOG("Valve on");
  digitalWrite(VALVE_PIN, HIGH);
}

void valveOff() {
  LOG("Valve off");
  digitalWrite(VALVE_PIN, LOW);
}
