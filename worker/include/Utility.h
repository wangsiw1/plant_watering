#pragma once
#include <Arduino.h>

#ifdef DEBUG_LOG
  #define LOG(fmt, ...) Serial.printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
  #define LOG(fmt, ...) do {} while(0)
#endif

constexpr int MAX_WORKER_COUNT = 16;

const String& getBtMac();
bool macFromHexString(const String &macHex, uint8_t out[6]);

// Trimmed mean: sort values and remove min/max before averaging
uint16_t trimmedMean(uint16_t *values, uint16_t count, uint8_t trimCount);
