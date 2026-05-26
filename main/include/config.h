#pragma once
#include <Arduino.h>
#include "Utility.h"

typedef enum {
  READY, 
  SYNCING,
  WATERING,
  SLEEPING
} State;

struct Settings {
  String name;
  uint32_t waterInterval;
  uint32_t dataSyncInterval;
  uint32_t activeStart;
  uint32_t activeEnd;
  // Time maintenance: store user-provided time of day (seconds since midnight)
  // and the millis() value when it was saved. Use these to compute current
  // time as: savedTimeOfDay + (millis() - savedMillis)/1000 (wrap-safe).
  uint32_t savedTimeOfDaySec;
  uint64_t savedMillis;
  // Absolute time storage (seconds since Unix epoch, local = UTC + tz offset)
  uint64_t savedEpochSec;      // 64-bit unsigned epoch seconds
  uint64_t savedEpochMillis; // millis() when epoch was captured
  int16_t tzOffsetMinutes;     // minutes east of UTC (e.g. +60 = +1h)
};

// Configured worker node entry persisted in NVS. Main maintains a list of these workers
// which is edited via the web UI (add/remove). Each entry contains MAC, moisture threshold
// and watering duration (seconds).
struct WorkerConfig {
  uint8_t mac[6];
  uint16_t threshold; // soil ADC threshold (lower means drier)
  uint16_t duration;  // watering duration in seconds
  uint8_t potIndex;   // which pot on the device this config applies to (flattened per-pot list)
  char name[32];
};

extern WorkerConfig workerList[MAX_WORKER_COUNT];
extern int workerListCount;
extern State state;

// Worker list management
bool addWorkerByHex(const String &macHex, uint16_t threshold, uint16_t duration, const String &name = String());
// Remove all per-pot configs for a worker (by MAC)
bool removeWorkerByHex(const String &macHex);
bool updateWorkerByHex(const String &macHex, uint16_t threshold, uint16_t duration, const String &name = String(), int potIndex = -1);

// Ensure per-pot WorkerConfig entries exist for a given MAC and reported pot count
void ensureWorkerConfigsForMac(const uint8_t mac[6], uint8_t potCount);
// Clear all persisted settings and reset in-memory defaults
bool clearAllSettings();

extern Settings settings;
extern volatile bool autoEnabled;
extern volatile unsigned long lastWateringEnd;
extern volatile unsigned long lastDataSync;

void saveSettings();
void loadSettings();
bool connectToWiFi();
void saveWifiCred(const char *ssid, const char *password);
bool clearWifiCredentials();
// Debounced save helpers
void markSettingsDirty();
void maybeSaveSettings();
