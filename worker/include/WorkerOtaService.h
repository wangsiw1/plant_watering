#pragma once

#include <stdint.h>

void workerOtaServiceBegin();
void workerOtaOpenWindow(const uint8_t allowedMainMac[6], uint32_t windowMs);
bool workerOtaIsActive();
void workerOtaServiceLoop();

