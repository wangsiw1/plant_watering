#pragma once
#include <stdint.h>
#include <stddef.h>

extern uint8_t mainMac[6];

bool mainMacIsSet();
void mainMacReset();
bool btLastCommOverdue();
void btWorkerBegin();
// Worker advertises its status (soil 0-4095 per pot, battery 0-100)
void btWorkerAdvertiseStatus();
