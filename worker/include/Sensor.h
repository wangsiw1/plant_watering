#pragma once
#include <Arduino.h>
#include "Utility.h"
#include "HardwareConfig.h"

struct WorkerSensorSnapshot {
  uint16_t batteryMv;
  uint16_t soils[WORKER_POT_COUNT];
};

void sensorBegin();
void readSoilMoisture();
void readWorkerSensors();
bool getWorkerSensorSnapshot(WorkerSensorSnapshot &snapshot);
// Get soil for channel index (default 0 for single-pot compatibility)
uint16_t getSoilMoisture(size_t idx = 0);
uint16_t getCorrectedSoilMoisture(uint16_t batteryMv, uint16_t sensorMv);
