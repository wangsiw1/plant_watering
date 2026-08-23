#pragma once

#include <stdint.h>

enum class ClockSource : uint8_t {
  UNINITIALIZED,
  FALLBACK,
  RESTORED,
  MANUAL,
  NTP
};

struct ClockStatusSnapshot {
  bool valid;
  ClockSource source;
  bool ntpSyncPending;
  char ntpSyncError[32];
};

void clockManagerInit();
void clockManagerLoop();

bool clockRequestNtpSync();
void clockResetToDefault();
void clockCheckpointNow();

bool clockSetUserTimeOfDaySec(uint32_t secOfDay);
uint32_t clockGetCurrentTimeOfDaySec();
uint64_t clockGetCurrentEpochSec();
bool clockIsValid();

void clockGetStatus(ClockStatusSnapshot& out);
const char* clockSourceName(ClockSource source);
