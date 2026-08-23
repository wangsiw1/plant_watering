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

static inline bool is_utf8_continuation(uint8_t c) {
  return (c & 0xC0u) == 0x80u;
}

static size_t utf8_sequence_length(const uint8_t *src, size_t remaining) {
  if (!src || remaining == 0) return 0;
  uint8_t c0 = src[0];
  if (c0 <= 0x7Fu) return 1;
  if (c0 >= 0xC2u && c0 <= 0xDFu) {
    return (remaining >= 2 && is_utf8_continuation(src[1])) ? 2 : 0;
  }
  if (c0 == 0xE0u) {
    return (remaining >= 3 && src[1] >= 0xA0u && src[1] <= 0xBFu && is_utf8_continuation(src[2])) ? 3 : 0;
  }
  if ((c0 >= 0xE1u && c0 <= 0xECu) || (c0 >= 0xEEu && c0 <= 0xEFu)) {
    return (remaining >= 3 && is_utf8_continuation(src[1]) && is_utf8_continuation(src[2])) ? 3 : 0;
  }
  if (c0 == 0xEDu) {
    return (remaining >= 3 && src[1] >= 0x80u && src[1] <= 0x9Fu && is_utf8_continuation(src[2])) ? 3 : 0;
  }
  if (c0 == 0xF0u) {
    return (remaining >= 4 && src[1] >= 0x90u && src[1] <= 0xBFu &&
            is_utf8_continuation(src[2]) && is_utf8_continuation(src[3])) ? 4 : 0;
  }
  if (c0 >= 0xF1u && c0 <= 0xF3u) {
    return (remaining >= 4 && is_utf8_continuation(src[1]) &&
            is_utf8_continuation(src[2]) && is_utf8_continuation(src[3])) ? 4 : 0;
  }
  if (c0 == 0xF4u) {
    return (remaining >= 4 && src[1] >= 0x80u && src[1] <= 0x8Fu &&
            is_utf8_continuation(src[2]) && is_utf8_continuation(src[3])) ? 4 : 0;
  }
  return 0;
}

// Strict parser: accept exactly 12 hex chars (no separators)
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

size_t copyUtf8Truncated(const char *src, char *dst, size_t dstSize) {
  if (!dst || dstSize == 0) return 0;
  dst[0] = '\0';
  if (!src) return 0;

  const uint8_t *cur = reinterpret_cast<const uint8_t *>(src);
  size_t remaining = strlen(src);
  size_t written = 0;
  const size_t maxBytes = dstSize - 1;

  while (remaining > 0) {
    size_t seqLen = utf8_sequence_length(cur, remaining);
    if (seqLen == 0) {
      ++cur;
      --remaining;
      continue;
    }
    if (written + seqLen > maxBytes) break;
    memcpy(dst + written, cur, seqLen);
    written += seqLen;
    cur += seqLen;
    remaining -= seqLen;
  }

  dst[written] = '\0';
  return written;
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

void getWifiMacLast6Hex(char out[7]) {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  // last 3 bytes -> 6 hex chars
  snprintf(out, 7, "%02x%02x%02x", mac[3], mac[4], mac[5]);
}

uint8_t calculateBatteryPercent(uint16_t batteryMv) {
  return (uint8_t)constrain(
      (int)((((float)batteryMv - 3300.0f) / (4200.0f - 3300.0f)) * 100.0f),
      0, 100);
}

uint16_t getCorrectedSoilMoisture(uint16_t batteryMv, uint16_t sensorMv) {
  // Return raw reading after hardware modification to
  // directly feed power to sensor from battery
  // Need correction at low battery(around 3.4V and below)
  
  // return sensorMv;

  if (batteryMv >= 3700) {
    return sensorMv;
  }

  // Apply low voltage compensation:
  int16_t estimatedRailMv = 3700 - batteryMv;
  uint16_t correctedSensorMv = (uint16_t)(
    0.075f * (float)estimatedRailMv + (float)sensorMv
  );

  return correctedSensorMv;
}
