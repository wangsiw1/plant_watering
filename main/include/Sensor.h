#pragma once
#include <Arduino.h>

void sensorBegin();
int readTankLevel();

extern volatile int gTankLevel;
