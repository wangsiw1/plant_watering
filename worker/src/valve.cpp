#include "Valve.h"
#include <Arduino.h>

static const int VALVE_PIN = 1;

void valveBegin() {
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(VALVE_PIN, LOW);
}

void valveOn() {
  digitalWrite(VALVE_PIN, HIGH);
}

void valveOff() {
  digitalWrite(VALVE_PIN, LOW);
}
