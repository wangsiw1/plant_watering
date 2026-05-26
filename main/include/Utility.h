#pragma once
#include <Arduino.h>

#ifdef DEBUG_LOG
  #define LOG(fmt, ...) Serial.printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
  #define LOG(fmt, ...) do {} while(0)
#endif

constexpr int MAX_WORKER_COUNT = 16;
constexpr int MAX_POTS_PER_DEVICE = 16;

const String& getWifiMacLast6();
const String& getBtMac();
bool macFromHexString(const String &macHex, uint8_t out[6]);

// Time helpers: set user-provided time (seconds since midnight) and query
// current time of day computed using millis() and saved reference.
void setUserTimeOfDaySec(uint32_t secOfDay);
uint32_t getCurrentTimeOfDaySec();
// Epoch-based time helpers
void setUserEpoch(uint64_t epochSec);
uint64_t getCurrentEpochSec();
// Attempt NTP sync; returns true on success
bool trySyncNTP(unsigned long timeoutMs = 10000);
uint32_t calculateSleepSec(unsigned long now_s);
