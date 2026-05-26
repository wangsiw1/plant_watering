#pragma once
#include <Arduino.h>
#include "Utility.h"

void sensorBegin();
void readSoilMoisture();
// Get soil for channel index (default 0 for single-pot compatibility)
uint16_t getSoilMoisture(size_t idx = 0);
