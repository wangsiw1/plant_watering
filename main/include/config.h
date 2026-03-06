#pragma once
#include <Arduino.h>
#include "Utility.h"

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
  unsigned long savedMillis;
};

// Configured worker node entry persisted in NVS. Main maintains a list of these workers
// which is edited via the web UI (add/remove). Each entry contains MAC, moisture threshold
// and watering duration (seconds).
struct WorkerConfig {
  uint8_t mac[6];
  uint16_t threshold; // soil ADC threshold (lower means drier)
  uint16_t duration;  // watering duration in seconds
  char name[32];
};

extern WorkerConfig workerList[MAX_WORKER_COUNT];
extern int workerListCount;

// Worker list management
bool addWorkerByHex(const String &macHex, uint16_t threshold, uint16_t duration, const String &name = String());
bool removeWorkerByHex(const String &macHex);
bool updateWorkerByHex(const String &macHex, uint16_t threshold, uint16_t duration, const String &name = String());

extern Settings settings;
extern volatile unsigned long autoEnabled;
extern volatile unsigned long lastWateringEnd;
extern volatile unsigned long lastDataSync;

void saveSettings();
void loadSettings();
