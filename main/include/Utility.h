#pragma once
#include <Arduino.h>

constexpr int MAX_WORKER_COUNT = 16;

const String& getWifiMacLast6();
bool macFromHexString(const String &macHex, uint8_t out[6]);

// Time helpers: set user-provided time (seconds since midnight) and query
// current time of day computed using millis() and saved reference.
void setUserTimeOfDaySec(uint32_t secOfDay);
uint32_t getCurrentTimeOfDaySec();
