#include "config.h"
#include "BluetoothMain.h"

#include <Preferences.h>
#include <WiFi.h>
#include <cstring>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {
constexpr uint32_t WATER_INTERVAL_MIN = 60;
constexpr uint32_t WATER_INTERVAL_MAX = 2419200;
constexpr uint32_t DATA_SYNC_INTERVAL_MIN = 60;
constexpr uint32_t DATA_SYNC_INTERVAL_MAX = 2419200;
constexpr uint16_t DEFAULT_THRESHOLD = 2000;
constexpr uint16_t DEFAULT_DURATION = 5;
constexpr int64_t SAVE_DEBOUNCE_US = 5000000LL;

Settings gSettings{};
WorkerConfig gWorkers[MAX_WORKER_COUNT] = {};
Settings gPersistenceSettings{};
WorkerConfig gPersistenceWorkers[MAX_WORKER_COUNT] = {};
int gWorkerCount = 0;
RuntimeSnapshot gRuntime{READY, false, 0, 0};
SemaphoreHandle_t gConfigMutex = nullptr;
SemaphoreHandle_t gPersistenceMutex = nullptr;
uint32_t gDirtyGeneration = 0;
uint32_t gSavedGeneration = 0;
int64_t gDirtyAtUs = 0;

void ensureMutexes() {
  if (!gConfigMutex) gConfigMutex = xSemaphoreCreateMutex();
  if (!gPersistenceMutex) gPersistenceMutex = xSemaphoreCreateMutex();
}

void lockConfig() {
  ensureMutexes();
  xSemaphoreTake(gConfigMutex, portMAX_DELAY);
}

void unlockConfig() {
  xSemaphoreGive(gConfigMutex);
}

void markDirtyLocked() {
  ++gDirtyGeneration;
  if (gDirtyGeneration == 0) ++gDirtyGeneration;
  gDirtyAtUs = esp_timer_get_time();
}

void initializeWorker(WorkerConfig& worker, const uint8_t mac[6],
                      const char* name, uint8_t potCount) {
  memset(&worker, 0, sizeof(worker));
  memcpy(worker.mac, mac, 6);
  worker.potCount = static_cast<uint8_t>(constrain(potCount, 1, MAX_POTS_PER_DEVICE));
  copyUtf8Truncated(name, worker.workerName, sizeof(worker.workerName));
  for (int pot = 0; pot < worker.potCount; ++pot) {
    worker.thresholds[pot] = DEFAULT_THRESHOLD;
    worker.durations[pot] = DEFAULT_DURATION;
  }
}

int findWorkerLocked(const uint8_t mac[6]) {
  for (int i = 0; i < gWorkerCount; ++i) {
    if (memcmp(gWorkers[i].mac, mac, 6) == 0) return i;
  }
  return -1;
}

bool validSettings(const Settings& value) {
  return value.waterInterval >= WATER_INTERVAL_MIN &&
         value.waterInterval <= WATER_INTERVAL_MAX &&
         value.dataSyncInterval >= DATA_SYNC_INTERVAL_MIN &&
         value.dataSyncInterval <= DATA_SYNC_INTERVAL_MAX &&
         value.activeStart < 86400 && value.activeEnd < 86400 &&
         value.tzOffsetMinutes >= -14 * 60 &&
         value.tzOffsetMinutes <= 14 * 60;
}

bool writeSnapshot(const Settings& settings,
                   const WorkerConfig workers[MAX_WORKER_COUNT],
                   int workerCount) {
  Preferences prefs;
  if (!prefs.begin("plant", false)) return false;
  if (prefs.getString("name", "") != settings.name)
    prefs.putString("name", settings.name);
  if (prefs.getUInt("wint", 0) != settings.waterInterval)
    prefs.putUInt("wint", settings.waterInterval);
  if (prefs.getUInt("dsint", 0) != settings.dataSyncInterval)
    prefs.putUInt("dsint", settings.dataSyncInterval);
  if (prefs.getUInt("start", UINT32_MAX) != settings.activeStart)
    prefs.putUInt("start", settings.activeStart);
  if (prefs.getUInt("end", UINT32_MAX) != settings.activeEnd)
    prefs.putUInt("end", settings.activeEnd);
  if (prefs.getInt("tz", INT32_MAX) != settings.tzOffsetMinutes)
    prefs.putInt("tz", settings.tzOffsetMinutes);

  uint64_t previous64 = 0;
  if (prefs.getBytes("utc64", &previous64, sizeof(previous64)) != sizeof(previous64) ||
      previous64 != settings.savedUtcSec) {
    prefs.putBytes("utc64", &settings.savedUtcSec, sizeof(settings.savedUtcSec));
  }
  previous64 = 0;
  if (prefs.getBytes("lastw64", &previous64, sizeof(previous64)) != sizeof(previous64) ||
      previous64 != settings.lastWateringUtcSec) {
    prefs.putBytes("lastw64", &settings.lastWateringUtcSec,
                   sizeof(settings.lastWateringUtcSec));
  }
  if (prefs.getUInt("wcount", UINT32_MAX) != static_cast<uint32_t>(workerCount))
    prefs.putUInt("wcount", workerCount);

  for (int i = 0; i < workerCount; ++i) {
    char key[24];
    uint8_t previousMac[6] = {};
    snprintf(key, sizeof(key), "wmac%d", i);
    if (prefs.getBytes(key, previousMac, sizeof(previousMac)) != sizeof(previousMac) ||
        memcmp(previousMac, workers[i].mac, sizeof(previousMac)) != 0) {
      prefs.putBytes(key, workers[i].mac, 6);
    }
    snprintf(key, sizeof(key), "wname%d", i);
    if (prefs.getString(key, "") != workers[i].workerName)
      prefs.putString(key, workers[i].workerName);
    snprintf(key, sizeof(key), "wpc%d", i);
    if (prefs.getUChar(key, 0) != workers[i].potCount)
      prefs.putUChar(key, workers[i].potCount);
    uint16_t previousValues[MAX_POTS_PER_DEVICE] = {};
    snprintf(key, sizeof(key), "wtharr%d", i);
    if (prefs.getBytes(key, previousValues, sizeof(previousValues)) !=
            sizeof(previousValues) ||
        memcmp(previousValues, workers[i].thresholds, sizeof(previousValues)) != 0) {
      prefs.putBytes(key, workers[i].thresholds, sizeof(workers[i].thresholds));
    }
    memset(previousValues, 0, sizeof(previousValues));
    snprintf(key, sizeof(key), "wdurarr%d", i);
    if (prefs.getBytes(key, previousValues, sizeof(previousValues)) !=
            sizeof(previousValues) ||
        memcmp(previousValues, workers[i].durations, sizeof(previousValues)) != 0) {
      prefs.putBytes(key, workers[i].durations, sizeof(workers[i].durations));
    }
    for (int pot = 0; pot < workers[i].potCount; ++pot) {
      snprintf(key, sizeof(key), "wpnm%d_%d", i, pot);
      if (prefs.getString(key, "") != workers[i].potName[pot])
        prefs.putString(key, workers[i].potName[pot]);
    }
  }
  prefs.end();
  return true;
}
}

void loadSettings() {
  ensureMutexes();
  Settings loaded{};
  char defaultName[7];
  getWifiMacLast6Hex(defaultName);
  copyUtf8Truncated(defaultName, loaded.name, sizeof(loaded.name));
  loaded.waterInterval = 3600;
  loaded.dataSyncInterval = 3600;
  loaded.activeStart = 0;
  loaded.activeEnd = 86399;

  memset(gPersistenceWorkers, 0, sizeof(gPersistenceWorkers));
  int workerCount = 0;
  Preferences prefs;
  if (prefs.begin("plant", true)) {
    String name = prefs.getString("name", defaultName);
    copyUtf8Truncated(name.c_str(), loaded.name, sizeof(loaded.name));
    loaded.waterInterval =
        constrain(prefs.getUInt("wint", 3600), WATER_INTERVAL_MIN, WATER_INTERVAL_MAX);
    loaded.dataSyncInterval =
        constrain(prefs.getUInt("dsint", 3600), DATA_SYNC_INTERVAL_MIN, DATA_SYNC_INTERVAL_MAX);
    loaded.activeStart =
        min(prefs.getUInt("start", 0), static_cast<uint32_t>(86399));
    loaded.activeEnd =
        min(prefs.getUInt("end", 86399), static_cast<uint32_t>(86399));
    loaded.tzOffsetMinutes =
        static_cast<int16_t>(constrain(prefs.getInt("tz", 0), -14 * 60, 14 * 60));
    if (prefs.getBytes("utc64", &loaded.savedUtcSec, sizeof(loaded.savedUtcSec)) !=
        sizeof(loaded.savedUtcSec)) {
      uint64_t legacyLocalEpoch = 0;
      if (prefs.getBytes("epoch64", &legacyLocalEpoch, sizeof(legacyLocalEpoch)) ==
          sizeof(legacyLocalEpoch)) {
        int64_t migrated =
            static_cast<int64_t>(legacyLocalEpoch) -
            static_cast<int64_t>(loaded.tzOffsetMinutes) * 60;
        loaded.savedUtcSec = migrated > 0 ? static_cast<uint64_t>(migrated) : 0;
      }
    }
    prefs.getBytes("lastw64", &loaded.lastWateringUtcSec,
                   sizeof(loaded.lastWateringUtcSec));

    int storedCount = min(static_cast<int>(prefs.getUInt("wcount", 0)),
                          MAX_WORKER_COUNT);
    for (int i = 0; i < storedCount; ++i) {
      WorkerConfig worker{};
      char key[24];
      snprintf(key, sizeof(key), "wmac%d", i);
      if (prefs.getBytes(key, worker.mac, 6) != 6) continue;
      bool duplicate = false;
      for (int existing = 0; existing < workerCount; ++existing) {
        if (memcmp(gPersistenceWorkers[existing].mac, worker.mac, 6) == 0)
          duplicate = true;
      }
      if (duplicate) continue;
      snprintf(key, sizeof(key), "wname%d", i);
      String workerName = prefs.getString(key, "");
      copyUtf8Truncated(workerName.c_str(), worker.workerName,
                        sizeof(worker.workerName));
      snprintf(key, sizeof(key), "wpc%d", i);
      worker.potCount =
          static_cast<uint8_t>(constrain(prefs.getUChar(key, 1), 1, MAX_POTS_PER_DEVICE));
      for (int pot = 0; pot < MAX_POTS_PER_DEVICE; ++pot) {
        worker.thresholds[pot] = DEFAULT_THRESHOLD;
        worker.durations[pot] = DEFAULT_DURATION;
      }
      snprintf(key, sizeof(key), "wtharr%d", i);
      prefs.getBytes(key, worker.thresholds, sizeof(worker.thresholds));
      snprintf(key, sizeof(key), "wdurarr%d", i);
      prefs.getBytes(key, worker.durations, sizeof(worker.durations));
      for (int pot = 0; pot < worker.potCount; ++pot) {
        worker.thresholds[pot] = min(worker.thresholds[pot], static_cast<uint16_t>(4095));
        worker.durations[pot] =
            static_cast<uint16_t>(constrain(worker.durations[pot], 1, 60));
        snprintf(key, sizeof(key), "wpnm%d_%d", i, pot);
        String potName = prefs.getString(key, "");
        copyUtf8Truncated(potName.c_str(), worker.potName[pot],
                          sizeof(worker.potName[pot]));
      }
      gPersistenceWorkers[workerCount++] = worker;
    }
    prefs.end();
  }

  lockConfig();
  gSettings = loaded;
  memcpy(gWorkers, gPersistenceWorkers, sizeof(gWorkers));
  gWorkerCount = workerCount;
  gDirtyGeneration = 1;
  gSavedGeneration = 1;
  unlockConfig();
}

void getSettingsSnapshot(Settings& out) {
  lockConfig();
  out = gSettings;
  unlockConfig();
}

bool applySettingsSnapshot(const Settings& next) {
  if (!validSettings(next)) return false;
  lockConfig();
  gSettings = next;
  gSettings.name[UTF8_NAME_STORAGE_BYTES - 1] = '\0';
  markDirtyLocked();
  unlockConfig();
  return true;
}

void setSavedUtc(uint64_t utcSec, bool saveImmediately) {
  lockConfig();
  gSettings.savedUtcSec = utcSec;
  markDirtyLocked();
  unlockConfig();
  if (saveImmediately) saveSettingsNow();
}

void setLastWateringUtc(uint64_t utcSec) {
  lockConfig();
  gSettings.lastWateringUtcSec = utcSec;
  markDirtyLocked();
  unlockConfig();
}

int getWorkerConfigCount() {
  lockConfig();
  int count = gWorkerCount;
  unlockConfig();
  return count;
}

bool getWorkerConfigAt(int index, WorkerConfig& out) {
  lockConfig();
  bool valid = index >= 0 && index < gWorkerCount;
  if (valid) out = gWorkers[index];
  unlockConfig();
  return valid;
}

bool findWorkerConfigByMac(const uint8_t mac[6], WorkerConfig& out) {
  if (!mac) return false;
  lockConfig();
  int index = findWorkerLocked(mac);
  if (index >= 0) out = gWorkers[index];
  unlockConfig();
  return index >= 0;
}

bool isWorkerConfigured(const uint8_t mac[6]) {
  WorkerConfig ignored{};
  return findWorkerConfigByMac(mac, ignored);
}

bool addWorkerByHex(const char* macHex, uint16_t threshold, uint16_t duration,
                    const char* name) {
  uint8_t mac[6];
  if (!macFromHexString(macHex, mac) || threshold > 4095 ||
      duration == 0 || duration > 60) {
    return false;
  }
  lockConfig();
  int index = findWorkerLocked(mac);
  if (index < 0) {
    if (gWorkerCount >= MAX_WORKER_COUNT) {
      unlockConfig();
      return false;
    }
    index = gWorkerCount++;
    initializeWorker(gWorkers[index], mac, name, 1);
    gWorkers[index].thresholds[0] = threshold;
    gWorkers[index].durations[0] = duration;
  } else if (name) {
    copyUtf8Truncated(name, gWorkers[index].workerName,
                      sizeof(gWorkers[index].workerName));
  }
  markDirtyLocked();
  unlockConfig();
  btMainEnsureNodeExists(mac);
  return true;
}

bool removeWorkerByHex(const char* macHex) {
  uint8_t mac[6];
  if (!macFromHexString(macHex, mac)) return false;
  lockConfig();
  int index = findWorkerLocked(mac);
  if (index < 0) {
    unlockConfig();
    return false;
  }
  for (int i = index; i < gWorkerCount - 1; ++i) gWorkers[i] = gWorkers[i + 1];
  --gWorkerCount;
  markDirtyLocked();
  unlockConfig();
  btMainRemoveNodeByMac(mac);
  return true;
}

bool updateWorkerByHex(const char* macHex, uint16_t threshold, uint16_t duration,
                       const char* name, int potIndex) {
  uint8_t mac[6];
  if (!macFromHexString(macHex, mac) || threshold > 4095 ||
      duration == 0 || duration > 60) {
    return false;
  }
  lockConfig();
  int index = findWorkerLocked(mac);
  if (index < 0 || potIndex >= MAX_POTS_PER_DEVICE) {
    unlockConfig();
    return false;
  }
  WorkerConfig& worker = gWorkers[index];
  if (potIndex >= 0) {
    for (int pot = worker.potCount; pot <= potIndex; ++pot) {
      worker.thresholds[pot] = DEFAULT_THRESHOLD;
      worker.durations[pot] = DEFAULT_DURATION;
      worker.potName[pot][0] = '\0';
    }
    if (potIndex >= worker.potCount) worker.potCount = potIndex + 1;
    worker.thresholds[potIndex] = threshold;
    worker.durations[potIndex] = duration;
    if (name) copyUtf8Truncated(name, worker.potName[potIndex],
                                sizeof(worker.potName[potIndex]));
  } else if (name) {
    copyUtf8Truncated(name, worker.workerName, sizeof(worker.workerName));
  }
  markDirtyLocked();
  unlockConfig();
  return true;
}

void ensureWorkerConfigsForMac(const uint8_t mac[6], uint8_t potCount) {
  if (!mac || potCount == 0) return;
  potCount = min(potCount, static_cast<uint8_t>(MAX_POTS_PER_DEVICE));
  lockConfig();
  int index = findWorkerLocked(mac);
  if (index < 0 || gWorkers[index].potCount >= potCount) {
    unlockConfig();
    return;
  }
  WorkerConfig& worker = gWorkers[index];
  for (int pot = worker.potCount; pot < potCount; ++pot) {
    worker.thresholds[pot] = DEFAULT_THRESHOLD;
    worker.durations[pot] = DEFAULT_DURATION;
    worker.potName[pot][0] = '\0';
  }
  worker.potCount = potCount;
  markDirtyLocked();
  unlockConfig();
}

void getRuntimeSnapshot(RuntimeSnapshot& out) {
  lockConfig();
  out = gRuntime;
  unlockConfig();
}

bool getAutoEnabled() {
  RuntimeSnapshot snapshot{};
  getRuntimeSnapshot(snapshot);
  return snapshot.autoEnabled;
}

void setAutoEnabled(bool enabled) {
  lockConfig();
  gRuntime.autoEnabled = enabled;
  unlockConfig();
}

void setRuntimeState(State state) {
  lockConfig();
  gRuntime.state = state;
  unlockConfig();
}

void setDataSyncRuntime(int64_t lastSyncUs, int64_t nextSyncUs) {
  lockConfig();
  gRuntime.lastDataSyncUs = lastSyncUs;
  gRuntime.nextDataSyncUs = nextSyncUs;
  unlockConfig();
}

bool connectToWiFi() {
  ensureMutexes();
  xSemaphoreTake(gPersistenceMutex, portMAX_DELAY);
  Preferences prefs;
  if (!prefs.begin("plant", true)) {
    xSemaphoreGive(gPersistenceMutex);
    return false;
  }
  String ssid = prefs.getString("ssid", "");
  String password = prefs.getString("password", "");
  prefs.end();
  xSemaphoreGive(gPersistenceMutex);
  if (ssid.isEmpty()) return false;
  if (password.isEmpty()) WiFi.begin(ssid.c_str());
  else WiFi.begin(ssid.c_str(), password.c_str());
  int64_t deadline = esp_timer_get_time() + 10000000LL;
  while (WiFi.status() != WL_CONNECTED && esp_timer_get_time() < deadline) {
    delay(250);
  }
  return WiFi.status() == WL_CONNECTED;
}

void saveWifiCred(const char* ssid, const char* password) {
  ensureMutexes();
  xSemaphoreTake(gPersistenceMutex, portMAX_DELAY);
  Preferences prefs;
  if (!prefs.begin("plant", false)) {
    xSemaphoreGive(gPersistenceMutex);
    return;
  }
  prefs.putString("ssid", ssid ? ssid : "");
  prefs.putString("password", password ? password : "");
  prefs.end();
  xSemaphoreGive(gPersistenceMutex);
}

bool clearWifiCredentials() {
  ensureMutexes();
  xSemaphoreTake(gPersistenceMutex, portMAX_DELAY);
  Preferences prefs;
  if (!prefs.begin("plant", false)) {
    xSemaphoreGive(gPersistenceMutex);
    return false;
  }
  prefs.remove("ssid");
  prefs.remove("password");
  prefs.end();
  xSemaphoreGive(gPersistenceMutex);
  return true;
}

bool clearAllSettings() {
  ensureMutexes();
  xSemaphoreTake(gPersistenceMutex, portMAX_DELAY);
  Preferences prefs;
  bool opened = prefs.begin("plant", false);
  if (opened) {
    prefs.clear();
    prefs.end();
  }
  xSemaphoreGive(gPersistenceMutex);
  if (!opened) return false;

  char defaultName[7];
  getWifiMacLast6Hex(defaultName);
  lockConfig();
  memset(&gSettings, 0, sizeof(gSettings));
  copyUtf8Truncated(defaultName, gSettings.name, sizeof(gSettings.name));
  gSettings.waterInterval = 3600;
  gSettings.dataSyncInterval = 3600;
  gSettings.activeEnd = 86399;
  memset(gWorkers, 0, sizeof(gWorkers));
  gWorkerCount = 0;
  gRuntime = {READY, false, 0, 0};
  markDirtyLocked();
  unlockConfig();
  initializeClockFromSettings();
  WorkerNode node{};
  while (btMainGetNodeAt(0, node)) btMainRemoveNodeByMac(node.mac);
  saveSettingsNow();
  return true;
}

void markSettingsDirty() {
  lockConfig();
  markDirtyLocked();
  unlockConfig();
}

void saveSettingsNow() {
  ensureMutexes();
  int workerCount = 0;
  uint32_t generation = 0;
  xSemaphoreTake(gPersistenceMutex, portMAX_DELAY);
  lockConfig();
  gPersistenceSettings = gSettings;
  memcpy(gPersistenceWorkers, gWorkers, sizeof(gPersistenceWorkers));
  workerCount = gWorkerCount;
  generation = gDirtyGeneration;
  unlockConfig();

  bool saved =
      writeSnapshot(gPersistenceSettings, gPersistenceWorkers, workerCount);
  xSemaphoreGive(gPersistenceMutex);

  lockConfig();
  if (saved && gDirtyGeneration == generation) gSavedGeneration = generation;
  unlockConfig();
}

void maybeSaveSettings() {
  lockConfig();
  bool due = gDirtyGeneration != gSavedGeneration &&
             esp_timer_get_time() - gDirtyAtUs >= SAVE_DEBOUNCE_US;
  unlockConfig();
  if (due) saveSettingsNow();
}
