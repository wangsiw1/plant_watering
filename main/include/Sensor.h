#pragma once
#include <Arduino.h>

constexpr int TANK_LOW_THRESHOLD_MV = 2800;

void sensorBegin();
void readTankLevel();
int getTankLevel();
bool tankSensorIsReady();
bool isTankLow();
bool isTankSafe();
