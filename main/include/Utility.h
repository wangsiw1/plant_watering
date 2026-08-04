#pragma once
#include <Arduino.h>

#ifdef DEBUG_LOG
  #define LOG(fmt, ...) Serial.printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
  #define LOG(fmt, ...) do {} while(0)
#endif

constexpr int MAX_WORKER_COUNT = 8;
constexpr int MAX_POTS_PER_DEVICE = 16;
constexpr size_t UTF8_NAME_MAX_BYTES = 64;
constexpr size_t UTF8_NAME_STORAGE_BYTES = UTF8_NAME_MAX_BYTES + 1;

// Parse a 12-character hex MAC string (no separators) into 6 bytes.
// Accepts only exactly 12 hex characters (0-9 A-F a-f).
bool macFromHexString(const char *macHex, uint8_t out[6]);
size_t copyUtf8Truncated(const char *src, char *dst, size_t dstSize);

// Format helpers: produce lower-case hex strings. Buffers must include
// space for the terminating NUL (`out[13]` for full MAC, `out[7]` for last6`).
void macToHexLower(const uint8_t mac[6], char out[13]);
void getBtMacHex(char out[13]);
void getWifiMacLast6Hex(char out[7]);

uint8_t calculateBatteryPercent(uint16_t batteryMv);
uint16_t getCorrectedSoilMoisture(uint16_t batteryMv, uint16_t sensorMv);
