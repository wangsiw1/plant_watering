#include "Utility.h"
#include <esp_mac.h>
#include <cstring>
#include <cctype>
#include <cstdio>

static inline int hexchar_to_nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool macFromHexString(const char *macHex, uint8_t out[6]) {
  if (!macHex) return false;
  size_t len = strlen(macHex);
  if (len != 12) return false;
  for (int i = 0; i < 6; ++i) {
    int hi = hexchar_to_nibble(macHex[i*2]);
    int lo = hexchar_to_nibble(macHex[i*2 + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i] = (uint8_t)((hi << 4) | lo);
  }
  return true;
}

void macToHexLower(const uint8_t mac[6], char out[13]) {
  if (!out) return;
  snprintf(out, 13, "%02x%02x%02x%02x%02x%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void getBtMacHex(char out[13]) {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  macToHexLower(mac, out);
}

// Trimmed mean: sort values and remove min/max before averaging
uint16_t trimmedMean(uint16_t *values, uint16_t count, uint8_t trimCount) {
  if (count <= 2 * trimCount) return 0; // Not enough samples after trimming
  
  // Simple bubble sort for small arrays
  for (uint16_t i = 0; i < count - 1; i++) {
    for (uint16_t j = 0; j < count - i - 1; j++) {
      if (values[j] > values[j + 1]) {
        uint16_t temp = values[j];
        values[j] = values[j + 1];
        values[j + 1] = temp;
      }
    }
  }
  
  // Sum middle values after trimming min and max
  uint32_t sum = 0;
  uint16_t validCount = count - 2 * trimCount;
  for (uint8_t i = trimCount; i < count - trimCount; i++) {
    sum += values[i];
  }
  
  return (uint16_t)(sum / validCount);
}
