#pragma once
#include <Arduino.h>

void valveBegin();
// Turn on indexed valve (default 0 for single-valve compatibility)
void valveOn(uint8_t idx = 0);
// Turn off indexed valve (default 0)
void valveOff(uint8_t idx = 0);
// Set valves by bitmask: bit0 -> pot 0
void valveSetMask(uint16_t mask);
// Acquire permission to sample the battery. A successful call holds the
// valve/battery mutex until valveEndBatterySampling() is called.
bool valveBeginBatterySampling();
void valveEndBatterySampling();
