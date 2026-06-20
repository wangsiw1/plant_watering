#include "Pump.h"
#include <Arduino.h>
#include "HardwareConfig.h"
#include "Utility.h"

void pumpBegin() {
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
}

void pumpOn() {
  LOG("Pump on");
  digitalWrite(PUMP_PIN, HIGH);
}

void pumpOff() {
  LOG("Pump off");
  digitalWrite(PUMP_PIN, LOW);
}
