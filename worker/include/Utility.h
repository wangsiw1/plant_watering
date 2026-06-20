#pragma once
#include <Arduino.h>

#ifdef DEBUG_LOG
  #define LOG(fmt, ...) Serial.printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
  #define LOG(fmt, ...) do {} while(0)
#endif

constexpr int MAX_WORKER_COUNT = 16;

// Parse a 12-character hex MAC string (no separators) into 6 bytes.
// Accepts only exactly 12 hex characters (0-9 A-F a-f).
bool macFromHexString(const char *macHex, uint8_t out[6]);

// Format helpers: produce lower-case hex strings. Buffers must include
// space for the terminating NUL (`out[13]` for full MAC).
void macToHexLower(const uint8_t mac[6], char out[13]);
void getBtMacHex(char out[13]);

// Trimmed mean: sort values and remove min/max before averaging
uint16_t trimmedMean(uint16_t *values, uint16_t count, uint8_t trimCount);
