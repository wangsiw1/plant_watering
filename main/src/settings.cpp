#include "config.h"
#include "AutoWateringPolicy.h"
#include "BluetoothMain.h"
#include "ClockMath.h"
#include "ClockManager.h"
#include "AutoRecovery.h"

#include <Preferences.h>
#include <WiFi.h>
#include <cstring>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {
constexpr uint32_t DATA_SYNC_INTERVAL_MIN = 60;
constexpr uint32_t DATA_SYNC_INTERVAL_MAX = 2419200;
constexpr uint8_t PUMP_DELAY_SECONDS_DEFAULT = 1;
constexpr uint8_t PUMP_DELAY_SECONDS_MAX = 30;
constexpr uint8_t MANUAL_PUMP_TIMEOUT_SECONDS_DEFAULT = 10;
constexpr uint8_t MANUAL_PUMP_TIMEOUT_SECONDS_MIN = 1;
constexpr uint8_t MANUAL_PUMP_TIMEOUT_SECONDS_MAX = 60;
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
bool gWifiCredentialsAvailable = false;
bool gWifiWasConnected = false;
uint8_t gWifiReconnectAttempt = 0;
int64_t gNextWifiReconnectUs = 0;

constexpr uint32_t WIFI_RECONNECT_DELAYS_MS[] = {5000, 15000, 30000, 60000};

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
    worker.waterIntervals[pot] = DEFAULT_WATER_INTERVAL;
  }
}

int findWorkerLocked(const uint8_t mac[6]) {
  for (int i = 0; i < gWorkerCount; ++i) {
    if (memcmp(gWorkers[i].mac, mac, 6) == 0) return i;
  }
  return -1;
}

bool validSettings(const Settings& value) {
  return value.dataSyncInterval >= DATA_SYNC_INTERVAL_MIN &&
         value.dataSyncInterval <= DATA_SYNC_INTERVAL_MAX &&
         value.pumpDelaySeconds <= PUMP_DELAY_SECONDS_MAX &&
         value.manualPumpTimeoutSeconds >= MANUAL_PUMP_TIMEOUT_SECONDS_MIN &&
         value.manualPumpTimeoutSeconds <= MANUAL_PUMP_TIMEOUT_SECONDS_MAX &&
         value.activeStart < 86400 && value.activeEnd < 86400 &&
         value.tzOffsetMinutes >= -14 * 60 &&
         value.tzOffsetMinutes <= 14 * 60 &&
         (value.language == UiLanguage::ENGLISH ||
          value.language == UiLanguage::SIMPLIFIED_CHINESE);
}

bool writeSnapshot(const Settings& settings,
                   const WorkerConfig workers[MAX_WORKER_COUNT],
                   int workerCount) {
  Preferences prefs;
  if (!prefs.begin("plant", false)) return false;
  if (prefs.getString("name", "") != settings.name)
    prefs.putString("name", settings.name);
  if (prefs.getUInt("dsint", 0) != settings.dataSyncInterval)
    prefs.putUInt("dsint", settings.dataSyncInterval);
  if (prefs.getUChar("pdelay", UINT8_MAX) != settings.pumpDelaySeconds)
    prefs.putUChar("pdelay", settings.pumpDelaySeconds);
  if (prefs.getUChar("mptime", UINT8_MAX) !=
      settings.manualPumpTimeoutSeconds)
    prefs.putUChar("mptime", settings.manualPumpTimeoutSeconds);
  if (prefs.getUInt("start", UINT32_MAX) != settings.activeStart)
    prefs.putUInt("start", settings.activeStart);
  if (prefs.getUInt("end", UINT32_MAX) != settings.activeEnd)
    prefs.putUInt("end", settings.activeEnd);
  if (prefs.getInt("tz", INT32_MAX) != settings.tzOffsetMinutes)
    prefs.putInt("tz", settings.tzOffsetMinutes);
  if (prefs.getUChar("lang", UINT8_MAX) !=
      static_cast<uint8_t>(settings.language)) {
    prefs.putUChar("lang", static_cast<uint8_t>(settings.language));
  }

  uint64_t previous64 = 0;
  if (prefs.getBytes("utc64", &previous64, sizeof(previous64)) != sizeof(previous64) ||
      previous64 != settings.savedUtcSec) {
    prefs.putBytes("utc64", &settings.savedUtcSec, sizeof(settings.savedUtcSec));
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
    uint32_t previousIntervals[MAX_POTS_PER_DEVICE] = {};
    snprintf(key, sizeof(key), "wintarr%d", i);
    if (prefs.getBytes(key, previousIntervals, sizeof(previousIntervals)) !=
            sizeof(previousIntervals) ||
        memcmp(previousIntervals, workers[i].waterIntervals,
               sizeof(previousIntervals)) != 0) {
      prefs.putBytes(key, workers[i].waterIntervals,
                     sizeof(workers[i].waterIntervals));
    }
    uint32_t previousWatering[MAX_POTS_PER_DEVICE] = {};
    snprintf(key, sizeof(key), "wlastarr%d", i);
    if (prefs.getBytes(key, previousWatering, sizeof(previousWatering)) !=
            sizeof(previousWatering) ||
        memcmp(previousWatering, workers[i].lastAutoWateringUtcSec,
               sizeof(previousWatering)) != 0) {
      prefs.putBytes(key, workers[i].lastAutoWateringUtcSec,
                     sizeof(workers[i].lastAutoWateringUtcSec));
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
  loaded.dataSyncInterval = 3600;
  loaded.pumpDelaySeconds = PUMP_DELAY_SECONDS_DEFAULT;
  loaded.manualPumpTimeoutSeconds = MANUAL_PUMP_TIMEOUT_SECONDS_DEFAULT;
  loaded.activeStart = 0;
  loaded.activeEnd = 86399;
  loaded.language = UiLanguage::ENGLISH;

  memset(gPersistenceWorkers, 0, sizeof(gPersistenceWorkers));
  int workerCount = 0;
  Preferences prefs;
  if (prefs.begin("plant", true)) {
    String name = prefs.getString("name", defaultName);
    copyUtf8Truncated(name.c_str(), loaded.name, sizeof(loaded.name));
    loaded.dataSyncInterval =
        constrain(prefs.getUInt("dsint", 3600), DATA_SYNC_INTERVAL_MIN, DATA_SYNC_INTERVAL_MAX);
    loaded.pumpDelaySeconds = static_cast<uint8_t>(constrain(
        prefs.getUChar("pdelay", PUMP_DELAY_SECONDS_DEFAULT), 0,
        PUMP_DELAY_SECONDS_MAX));
    loaded.manualPumpTimeoutSeconds = static_cast<uint8_t>(constrain(
        prefs.getUChar("mptime", MANUAL_PUMP_TIMEOUT_SECONDS_DEFAULT),
        MANUAL_PUMP_TIMEOUT_SECONDS_MIN, MANUAL_PUMP_TIMEOUT_SECONDS_MAX));
    loaded.activeStart =
        min(prefs.getUInt("start", 0), static_cast<uint32_t>(86399));
    loaded.activeEnd =
        min(prefs.getUInt("end", 86399), static_cast<uint32_t>(86399));
    loaded.tzOffsetMinutes =
        static_cast<int16_t>(constrain(prefs.getInt("tz", 0), -14 * 60, 14 * 60));
    uint8_t storedLanguage = prefs.getUChar(
        "lang", static_cast<uint8_t>(UiLanguage::ENGLISH));
    loaded.language =
        storedLanguage == static_cast<uint8_t>(UiLanguage::SIMPLIFIED_CHINESE)
            ? UiLanguage::SIMPLIFIED_CHINESE
            : UiLanguage::ENGLISH;
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
        worker.waterIntervals[pot] = DEFAULT_WATER_INTERVAL;
        worker.lastAutoWateringUtcSec[pot] = 0;
      }
      snprintf(key, sizeof(key), "wtharr%d", i);
      prefs.getBytes(key, worker.thresholds, sizeof(worker.thresholds));
      snprintf(key, sizeof(key), "wdurarr%d", i);
      prefs.getBytes(key, worker.durations, sizeof(worker.durations));
      snprintf(key, sizeof(key), "wintarr%d", i);
      if (prefs.getBytesLength(key) == sizeof(worker.waterIntervals)) {
        prefs.getBytes(key, worker.waterIntervals,
                       sizeof(worker.waterIntervals));
      }
      snprintf(key, sizeof(key), "wlastarr%d", i);
      if (prefs.getBytesLength(key) ==
          sizeof(worker.lastAutoWateringUtcSec)) {
        prefs.getBytes(key, worker.lastAutoWateringUtcSec,
                       sizeof(worker.lastAutoWateringUtcSec));
      }
      for (int pot = 0; pot < worker.potCount; ++pot) {
        worker.thresholds[pot] = min(worker.thresholds[pot], static_cast<uint16_t>(4095));
        worker.durations[pot] =
            static_cast<uint16_t>(constrain(worker.durations[pot], 1, 60));
        worker.waterIntervals[pot] = constrain(
            worker.waterIntervals[pot], WATER_INTERVAL_MIN,
            WATER_INTERVAL_MAX);
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

void recordCompletedAutoWateringBatch(
    const CompletedAutoWatering* completions, size_t completionCount,
    uint32_t completedUtcSec) {
  if (!completions || completionCount == 0 || completedUtcSec == 0) return;
  lockConfig();
  bool changed = false;
  for (size_t completion = 0; completion < completionCount; ++completion) {
    int workerIndex = findWorkerLocked(completions[completion].mac);
    if (workerIndex < 0) continue;
    WorkerConfig& worker = gWorkers[workerIndex];
    changed |= applyCompletedAutoWatering(
        worker.lastAutoWateringUtcSec, worker.potCount,
        completions[completion].potMask, completedUtcSec);
  }
  if (changed) markDirtyLocked();
  unlockConfig();
}

void rebaseLastAutoWateringTimestamps(uint64_t oldNowUtcSec,
                                      uint64_t newNowUtcSec) {
  if (newNowUtcSec == 0 || newNowUtcSec > UINT32_MAX) return;
  lockConfig();
  bool changed = false;
  for (int workerIndex = 0; workerIndex < gWorkerCount; ++workerIndex) {
    WorkerConfig& worker = gWorkers[workerIndex];
    for (int pot = 0; pot < worker.potCount; ++pot) {
      uint32_t previous = worker.lastAutoWateringUtcSec[pot];
      uint64_t rebased = clockRebaseTimestampPreservingAge(
          oldNowUtcSec, newNowUtcSec, previous);
      uint32_t next = static_cast<uint32_t>(rebased);
      if (next != previous) {
        worker.lastAutoWateringUtcSec[pot] = next;
        changed = true;
      }
    }
  }
  if (changed) markDirtyLocked();
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
                       uint32_t waterInterval, const char* name, int potIndex) {
  uint8_t mac[6];
  if (!macFromHexString(macHex, mac) || threshold > 4095 ||
      duration == 0 || duration > 60 || !validWaterInterval(waterInterval)) {
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
      worker.waterIntervals[pot] = DEFAULT_WATER_INTERVAL;
      worker.lastAutoWateringUtcSec[pot] = 0;
      worker.potName[pot][0] = '\0';
    }
    if (potIndex >= worker.potCount) worker.potCount = potIndex + 1;
    worker.thresholds[potIndex] = threshold;
    worker.durations[potIndex] = duration;
    worker.waterIntervals[potIndex] = waterInterval;
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
    worker.waterIntervals[pot] = DEFAULT_WATER_INTERVAL;
    worker.lastAutoWateringUtcSec[pot] = 0;
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
  // Disarming the retained intent first ensures a reset cannot undo a user's
  // request to turn automation off.
  if (!enabled) autoRecoverySetArmed(false);
  lockConfig();
  gRuntime.autoEnabled = enabled;
  unlockConfig();
  if (enabled) autoRecoverySetArmed(true);
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

WiFiStartupResult connectToWiFi() {
  ensureMutexes();
  xSemaphoreTake(gPersistenceMutex, portMAX_DELAY);
  Preferences prefs;
  if (!prefs.begin("plant", true)) {
    xSemaphoreGive(gPersistenceMutex);
    lockConfig();
    gWifiCredentialsAvailable = false;
    unlockConfig();
    return WiFiStartupResult::NO_SAVED_CREDENTIALS;
  }
  String ssid = prefs.getString("ssid", "");
  String password = prefs.getString("password", "");
  prefs.end();
  xSemaphoreGive(gPersistenceMutex);
  lockConfig();
  gWifiCredentialsAvailable = !ssid.isEmpty();
  unlockConfig();
  if (ssid.isEmpty()) return WiFiStartupResult::NO_SAVED_CREDENTIALS;
  WiFi.setAutoReconnect(true);
  if (password.isEmpty()) WiFi.begin(ssid.c_str());
  else WiFi.begin(ssid.c_str(), password.c_str());
  int64_t deadline = esp_timer_get_time() + 10000000LL;
  while (WiFi.status() != WL_CONNECTED && esp_timer_get_time() < deadline) {
    delay(250);
  }
  return WiFi.status() == WL_CONNECTED ? WiFiStartupResult::CONNECTED
                                       : WiFiStartupResult::CONNECTION_FAILED;
}

void initializeWiFiMaintenance() {
  WiFi.setAutoReconnect(true);
  lockConfig();
  gWifiWasConnected = WiFi.status() == WL_CONNECTED;
  gWifiReconnectAttempt = 0;
  gNextWifiReconnectUs = 0;
  unlockConfig();
}

bool serviceWiFiMaintenance() {
  bool connected = WiFi.status() == WL_CONNECTED;
  bool requestNtp = false;
  bool reconnect = false;
  int64_t nowUs = esp_timer_get_time();

  lockConfig();
  if (connected) {
    requestNtp = !gWifiWasConnected;
    gWifiWasConnected = true;
    gWifiReconnectAttempt = 0;
    gNextWifiReconnectUs = 0;
  } else {
    if (gWifiWasConnected || gNextWifiReconnectUs == 0) {
      gWifiWasConnected = false;
      gWifiReconnectAttempt = 0;
      gNextWifiReconnectUs =
          nowUs + static_cast<int64_t>(WIFI_RECONNECT_DELAYS_MS[0]) * 1000;
    } else if (gWifiCredentialsAvailable && nowUs >= gNextWifiReconnectUs) {
      reconnect = true;
      size_t delayIndex = min(
          static_cast<size_t>(gWifiReconnectAttempt + 1),
          sizeof(WIFI_RECONNECT_DELAYS_MS) / sizeof(WIFI_RECONNECT_DELAYS_MS[0]) - 1);
      gNextWifiReconnectUs =
          nowUs + static_cast<int64_t>(WIFI_RECONNECT_DELAYS_MS[delayIndex]) * 1000;
      if (gWifiReconnectAttempt <
          sizeof(WIFI_RECONNECT_DELAYS_MS) / sizeof(WIFI_RECONNECT_DELAYS_MS[0]) - 1) {
        ++gWifiReconnectAttempt;
      }
    }
  }
  unlockConfig();

  if (reconnect) {
    LOG("WiFi reconnect attempt=%u", static_cast<unsigned>(gWifiReconnectAttempt));
    WiFi.reconnect();
  }
  if (requestNtp) {
    LOG("WiFi reconnected; requesting NTP sync");
    clockRequestNtpSync();
  }
  return requestNtp;
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
  lockConfig();
  gWifiCredentialsAvailable = ssid && ssid[0] != '\0';
  unlockConfig();
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
  lockConfig();
  gWifiCredentialsAvailable = false;
  unlockConfig();
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
  autoRecoverySetArmed(false);
  lockConfig();
  memset(&gSettings, 0, sizeof(gSettings));
  copyUtf8Truncated(defaultName, gSettings.name, sizeof(gSettings.name));
  gSettings.dataSyncInterval = 3600;
  gSettings.pumpDelaySeconds = PUMP_DELAY_SECONDS_DEFAULT;
  gSettings.manualPumpTimeoutSeconds =
      MANUAL_PUMP_TIMEOUT_SECONDS_DEFAULT;
  gSettings.activeEnd = 86399;
  gSettings.language = UiLanguage::ENGLISH;
  memset(gWorkers, 0, sizeof(gWorkers));
  gWorkerCount = 0;
  gRuntime = {READY, false, 0, 0};
  gWifiCredentialsAvailable = false;
  markDirtyLocked();
  unlockConfig();
  clockResetToDefault();
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
