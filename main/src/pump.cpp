#include "Pump.h"
#include <Arduino.h>
#include "HardwareConfig.h"
#include "Utility.h"

namespace {
volatile bool gPumpActive = false;
}

void pumpBegin() {
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  gPumpActive = false;
}

void pumpOn() {
  LOG("Pump on");
  digitalWrite(PUMP_PIN, HIGH);
  gPumpActive = true;
}

void pumpOff() {
  LOG("Pump off");
  digitalWrite(PUMP_PIN, LOW);
  gPumpActive = false;
}

bool pumpIsOn() {
  return gPumpActive;
}
