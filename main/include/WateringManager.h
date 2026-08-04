#pragma once

#include <stdint.h>

#include "Utility.h"

void wateringManagerInit();
bool wateringQueueManual(const uint8_t mac[6], uint16_t potMask,
                         const uint16_t* durations, size_t durationCount);
bool wateringProcessOneManual();
bool wateringExecuteWorker(const uint8_t mac[6], uint16_t potMask,
                           const uint16_t* durations, size_t durationCount,
                           bool abortOnLowTank);
void wateringNotifyCompleted(const uint8_t mac[6], uint16_t potMask);
