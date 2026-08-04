#include "Valve.h"
#include "Utility.h"
#include <Arduino.h>
#include "HardwareConfig.h"
#include "esp_timer.h"
#include "freertos/semphr.h"

static volatile uint16_t gValveState = 0;
static int64_t gBatteryBlockedUntilUs = 0;
static SemaphoreHandle_t gValveBatteryMutex = nullptr;
static constexpr int64_t BATTERY_RECOVERY_US = 5000000LL;

static void ensureValveBatteryMutex() {
  if (!gValveBatteryMutex) gValveBatteryMutex = xSemaphoreCreateMutex();
}

static uint16_t setValveState(uint16_t nextState) {
  ensureValveBatteryMutex();
  int64_t nowUs = esp_timer_get_time();
  xSemaphoreTake(gValveBatteryMutex, portMAX_DELAY);
  uint16_t previousState = gValveState;
  gValveState = nextState;
  if (previousState != 0 && nextState == 0) {
    gBatteryBlockedUntilUs = nowUs + BATTERY_RECOVERY_US;
  }
  xSemaphoreGive(gValveBatteryMutex);
  return nextState;
}

static uint16_t updateValveBit(uint8_t idx, bool enabled) {
  ensureValveBatteryMutex();
  int64_t nowUs = esp_timer_get_time();
  xSemaphoreTake(gValveBatteryMutex, portMAX_DELAY);
  uint16_t previousState = gValveState;
  uint16_t nextState = enabled
                           ? static_cast<uint16_t>(previousState | (1u << idx))
                           : static_cast<uint16_t>(previousState & ~(1u << idx));
  gValveState = nextState;
  if (previousState != 0 && nextState == 0) {
    gBatteryBlockedUntilUs = nowUs + BATTERY_RECOVERY_US;
  }
  xSemaphoreGive(gValveBatteryMutex);
  return nextState;
}

static void resetValveState() {
  ensureValveBatteryMutex();
  xSemaphoreTake(gValveBatteryMutex, portMAX_DELAY);
  gValveState = 0;
  gBatteryBlockedUntilUs = 0;
  xSemaphoreGive(gValveBatteryMutex);
}

bool valveBeginBatterySampling() {
  ensureValveBatteryMutex();
  int64_t nowUs = esp_timer_get_time();
  xSemaphoreTake(gValveBatteryMutex, portMAX_DELAY);
  bool allowed = gValveState == 0 && nowUs >= gBatteryBlockedUntilUs;
  if (!allowed) xSemaphoreGive(gValveBatteryMutex);
  return allowed;
}

void valveEndBatterySampling() {
  if (gValveBatteryMutex) xSemaphoreGive(gValveBatteryMutex);
}

#if WORKER_POT_COUNT == 1

void valveBegin() {
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(VALVE_PIN, LOW);
  resetValveState();
}

void valveOn(uint8_t idx) {
  (void)idx;
  LOG("Valve on");
  setValveState(1);
  digitalWrite(VALVE_PIN, HIGH);
}

void valveOff(uint8_t idx) {
  (void)idx;
  LOG("Valve off");
  setValveState(0);
  digitalWrite(VALVE_PIN, LOW);
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
  resetValveState();
  writeShiftRegisters(0);
}

void valveOn(uint8_t idx) {
  if (idx >= WORKER_POT_COUNT) return;
  writeShiftRegisters(updateValveBit(idx, true));
}

void valveOff(uint8_t idx) {
  if (idx >= WORKER_POT_COUNT) return;
  writeShiftRegisters(updateValveBit(idx, false));
}

void valveSetMask(uint16_t mask) {
  writeShiftRegisters(setValveState(mask & VALVE_MASK_ALL));
}

#endif
