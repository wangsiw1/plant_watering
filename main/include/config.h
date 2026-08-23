#pragma once

#include <Arduino.h>
#include "Utility.h"

enum State : uint8_t {
  READY,
  SYNCING,
  WATERING,
  SLEEPING,
  UPDATING
};

enum class UiLanguage : uint8_t {
  ENGLISH = 0,
  SIMPLIFIED_CHINESE = 1
};

struct Settings {
  char name[UTF8_NAME_STORAGE_BYTES];
  uint32_t dataSyncInterval;
  uint8_t pumpDelaySeconds;
  uint8_t manualPumpTimeoutSeconds;
  uint32_t activeStart;
  uint32_t activeEnd;
  uint64_t savedUtcSec;
  int16_t tzOffsetMinutes;
  UiLanguage language;
};

struct WorkerConfig {
  uint8_t mac[6];
  char workerName[UTF8_NAME_STORAGE_BYTES];
  uint8_t potCount;
  uint16_t thresholds[MAX_POTS_PER_DEVICE];
  uint16_t durations[MAX_POTS_PER_DEVICE];
  uint32_t waterIntervals[MAX_POTS_PER_DEVICE];
  uint32_t lastAutoWateringUtcSec[MAX_POTS_PER_DEVICE];
  char potName[MAX_POTS_PER_DEVICE][UTF8_NAME_STORAGE_BYTES];
};

struct CompletedAutoWatering {
  uint8_t mac[6];
  uint16_t potMask;
};

struct RuntimeSnapshot {
  State state;
  bool autoEnabled;
  int64_t lastDataSyncUs;
  int64_t nextDataSyncUs;
};

enum class WiFiStartupResult : uint8_t {
  CONNECTED,
  NO_SAVED_CREDENTIALS,
  CONNECTION_FAILED
};

void loadSettings();
void getSettingsSnapshot(Settings& out);
bool applySettingsSnapshot(const Settings& next);
void setSavedUtc(uint64_t utcSec, bool saveImmediately);
void recordCompletedAutoWateringBatch(
    const CompletedAutoWatering* completions, size_t completionCount,
    uint32_t completedUtcSec);
void rebaseLastAutoWateringTimestamps(uint64_t oldNowUtcSec,
                                      uint64_t newNowUtcSec);

int getWorkerConfigCount();
bool getWorkerConfigAt(int index, WorkerConfig& out);
bool findWorkerConfigByMac(const uint8_t mac[6], WorkerConfig& out);
bool isWorkerConfigured(const uint8_t mac[6]);

bool addWorkerByHex(const char* macHex, uint16_t threshold, uint16_t duration,
                    const char* name = nullptr);
bool removeWorkerByHex(const char* macHex);
bool updateWorkerByHex(const char* macHex, uint16_t threshold, uint16_t duration,
                       uint32_t waterInterval, const char* name = nullptr,
                       int potIndex = -1);
void ensureWorkerConfigsForMac(const uint8_t mac[6], uint8_t potCount);

void getRuntimeSnapshot(RuntimeSnapshot& out);
bool getAutoEnabled();
void setAutoEnabled(bool enabled);
void setRuntimeState(State state);
void setDataSyncRuntime(int64_t lastSyncUs, int64_t nextSyncUs);

WiFiStartupResult connectToWiFi();
void initializeWiFiMaintenance();
// Returns true only when a disconnected station has newly regained Wi-Fi.
bool serviceWiFiMaintenance();
void saveWifiCred(const char* ssid, const char* password);
bool clearWifiCredentials();
bool clearAllSettings();

void markSettingsDirty();
void maybeSaveSettings();
void saveSettingsNow();
