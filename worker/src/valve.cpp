#include "Valve.h"
#include "Utility.h"
#include <Arduino.h>
#include "HardwareConfig.h"

static volatile uint16_t gValveState = 0;

#if WORKER_POT_COUNT == 1

void valveBegin() {
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(VALVE_PIN, LOW);
  gValveState = 0;
}

void valveOn(uint8_t idx) {
  (void)idx;
  LOG("Valve on");
  digitalWrite(VALVE_PIN, HIGH);
  gValveState = 1;
}

void valveOff(uint8_t idx) {
  (void)idx;
  LOG("Valve off");
  digitalWrite(VALVE_PIN, LOW);
  gValveState = 0;
}

void valveSetMask(uint16_t mask) {
  if (mask & 0x1) valveOn(0); else valveOff(0);
}

#else

static const uint16_t VALVE_MASK_ALL = (WORKER_POT_COUNT >= 16) ? 0xFFFF : ((1u << WORKER_POT_COUNT) - 1);

static void writeShiftRegisters(uint16_t mask) {
  uint16_t m = mask & VALVE_MASK_ALL;
  digitalWrite(SHIFT_LATCH_PIN, LOW);
  // send bytes MSB-first so chip ordering is consistent
  for (int chip = SHIFT_REG_CHIPS - 1; chip >= 0; --chip) {
    uint8_t b = (m >> (chip * 8)) & 0xFF;
    shiftOut(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, MSBFIRST, b);
  }
  digitalWrite(SHIFT_LATCH_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(1));
  digitalWrite(SHIFT_LATCH_PIN, LOW);
}

void valveBegin() {
  pinMode(SHIFT_DATA_PIN, OUTPUT);
  pinMode(SHIFT_CLOCK_PIN, OUTPUT);
  pinMode(SHIFT_LATCH_PIN, OUTPUT);
  digitalWrite(SHIFT_LATCH_PIN, LOW);
  gValveState = 0;
  writeShiftRegisters(0);
}

void valveOn(uint8_t idx) {
  if (idx >= WORKER_POT_COUNT) return;
  gValveState |= (1u << idx);
  writeShiftRegisters(gValveState);
}

void valveOff(uint8_t idx) {
  if (idx >= WORKER_POT_COUNT) return;
  gValveState &= ~(1u << idx);
  writeShiftRegisters(gValveState);
}

void valveSetMask(uint16_t mask) {
  gValveState = mask & VALVE_MASK_ALL;
  writeShiftRegisters(gValveState);
}

#endif
