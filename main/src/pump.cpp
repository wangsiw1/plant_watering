#include "Pump.h"
#include <Arduino.h>

static const int PUMP_PIN = 1;

void pumpBegin() {
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
}

void pumpOn() {
  digitalWrite(PUMP_PIN, HIGH);
}

void pumpOff() {
  digitalWrite(PUMP_PIN, LOW);
}
