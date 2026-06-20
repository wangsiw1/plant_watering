#include "Utility.h"
#include "config.h"
#include <time.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <cstring>
#include <cctype>
#include <cstdio>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {
SemaphoreHandle_t gClockMutex = nullptr;
uint64_t gClockBaseUtc = 0;
int64_t gClockBaseUs = 0;
bool gClockValid = false;

void ensureClockMutex() {
  if (!gClockMutex) gClockMutex = xSemaphoreCreateMutex();
}

uint64_t currentClockBaseValue() {
  ensureClockMutex();
  xSemaphoreTake(gClockMutex, portMAX_DELAY);
  uint64_t value = gClockBaseUtc;
  xSemaphoreGive(gClockMutex);
  return value;
}
}

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

void initializeClockFromSettings() {
  Settings settings{};
  getSettingsSnapshot(settings);
  ensureClockMutex();
  xSemaphoreTake(gClockMutex, portMAX_DELAY);
  gClockBaseUtc = settings.savedUtcSec;
  gClockBaseUs = esp_timer_get_time();
  gClockValid = false;
  xSemaphoreGive(gClockMutex);
}

bool isClockValid() {
  ensureClockMutex();
  xSemaphoreTake(gClockMutex, portMAX_DELAY);
  bool valid = gClockValid;
  xSemaphoreGive(gClockMutex);
  return valid;
}

bool setUserTimeOfDaySec(uint32_t secOfDay) {
  if (secOfDay >= 86400) return false;
  Settings settings{};
  getSettingsSnapshot(settings);
  uint64_t referenceUtc = getCurrentEpochSec();
  if (referenceUtc == 0) referenceUtc = currentClockBaseValue();
  if (referenceUtc == 0) return false;

  int64_t localReference =
      static_cast<int64_t>(referenceUtc) +
      static_cast<int64_t>(settings.tzOffsetMinutes) * 60;
  int64_t localDay = localReference - ((localReference % 86400 + 86400) % 86400);
  int64_t newUtc = localDay + secOfDay -
                   static_cast<int64_t>(settings.tzOffsetMinutes) * 60;
  if (newUtc <= 0) return false;
  setUserEpoch(static_cast<uint64_t>(newUtc));
  return true;
}

uint32_t getCurrentTimeOfDaySec() {
  uint64_t utc = getCurrentEpochSec();
  if (utc == 0) return 0;
  Settings settings{};
  getSettingsSnapshot(settings);
  int64_t local = static_cast<int64_t>(utc) +
                  static_cast<int64_t>(settings.tzOffsetMinutes) * 60;
  return static_cast<uint32_t>((local % 86400 + 86400) % 86400);
}

void setUserEpoch(uint64_t epochSec) {
  ensureClockMutex();
  xSemaphoreTake(gClockMutex, portMAX_DELAY);
  gClockBaseUtc = epochSec;
  gClockBaseUs = esp_timer_get_time();
  gClockValid = epochSec != 0;
  xSemaphoreGive(gClockMutex);
  setSavedUtc(epochSec, false);
}

uint64_t getCurrentEpochSec() {
  ensureClockMutex();
  xSemaphoreTake(gClockMutex, portMAX_DELAY);
  bool valid = gClockValid;
  uint64_t base = gClockBaseUtc;
  int64_t baseUs = gClockBaseUs;
  xSemaphoreGive(gClockMutex);
  if (!valid || base == 0) return 0;
  int64_t elapsedUs = esp_timer_get_time() - baseUs;
  if (elapsedUs < 0) elapsedUs = 0;
  return base + static_cast<uint64_t>(elapsedUs / 1000000LL);
}

bool trySyncNTP(unsigned long timeoutMs) {
  if (WiFi.status() != WL_CONNECTED) return false;
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    time_t utc = time(nullptr);
    if (utc > 1600000000) {
      setUserEpoch(static_cast<uint64_t>(utc));
      saveSettingsNow();
      return true;
    }
    delay(200);
  }
  return false;
}

uint8_t calculateBatteryPercent(uint16_t batteryMv) {
  return (uint8_t)constrain(
      (int)((((float)batteryMv - 3300.0f) / (4200.0f - 3300.0f)) * 100.0f),
      0, 100);
}

uint16_t getCorrectedSoilMoisture(uint16_t batteryMv, uint16_t sensorMv) {
  if (batteryMv >= 3750) {
    return sensorMv;
  }

  int16_t estimatedRailMv = 3750 - batteryMv;

  // Apply compensation:
  // Linear = 937*(3750-battery)/100-25.8+sensor
  // uint16_t correctedSensorMv = (uint16_t)(
  //   0.937f * (float)estimatedRailMv - 25.8f + (float)sensorMv
  // );

  // Polynoimial = -8.57 + 0.682 * sensor + 7.08E-04 * sensor^2
  uint16_t correctedSensorMv = (uint16_t)(
      -8.57f + 0.682f * (float)estimatedRailMv +
      7.08e-4f * (float)estimatedRailMv * (float)estimatedRailMv +
      (float)sensorMv);

  return correctedSensorMv;
}
