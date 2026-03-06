#pragma once
#include <stdint.h>
#include <stddef.h>

extern uint8_t mainMac[6];

extern bool mainMacIsSet();
void btWorkerBegin();
// Worker advertises its status (soil 0-4095, battery 0-100)
void btWorkerAdvertiseStatus(uint16_t soil, uint8_t batt);
