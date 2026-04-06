#pragma once
#include <Arduino.h>
#include "Utility.h"

void sensorBegin();
void readSoilMoisture();
uint16_t getSoilMoisture();
