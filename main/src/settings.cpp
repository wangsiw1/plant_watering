#include "Config.h"
#include "Utility.h"
#include "BluetoothMain.h"
#include <WiFi.h>
#include <Preferences.h>

static Preferences prefs;

Settings settings;
volatile bool autoEnabled = false;
volatile unsigned long lastWateringEnd = 0;
volatile unsigned long lastDataSync = 0;

WorkerConfig workerList[MAX_WORKER_COUNT];
int workerListCount = 0;
bool workerListDirty = false;
State state = READY;

// Debounce state for settings persistence
static volatile bool settingsDirty = false;
static unsigned long settingsDirtyAt = 0;
const unsigned long SAVE_DEBOUNCE_MS = 5000;

static const uint32_t WATER_INTERVAL_MIN = 60;
static const uint32_t WATER_INTERVAL_MAX = 2419200;
static const uint32_t DATA_SYNC_INTERVAL_MIN = 60;
static const uint32_t DATA_SYNC_INTERVAL_MAX = 2419200;

void saveSettings() {
  // Only write changed values to reduce wear on NVS flash.
  prefs.begin("plant", false);
  String prevName = prefs.getString("name", "");
  if (!prevName.equals(settings.name)) prefs.putString("name", settings.name);
  uint32_t prev = prefs.getUInt("wint", 0);
  if (prev != settings.waterInterval) prefs.putUInt("wint", settings.waterInterval);
  prev = prefs.getUInt("dsint", 0);
  if (prev != settings.dataSyncInterval) prefs.putUInt("dsint", settings.dataSyncInterval);
  prev = prefs.getUInt("start", 0);
  if (prev != settings.activeStart) prefs.putUInt("start", settings.activeStart);
  prev = prefs.getUInt("end", 0);
  if (prev != settings.activeEnd) prefs.putUInt("end", settings.activeEnd);
  prev = prefs.getUInt("tsec", 0);
  if (prev != settings.savedTimeOfDaySec) prefs.putUInt("tsec", settings.savedTimeOfDaySec);
  // persisted savedMillis (64-bit)
  uint64_t prevMillis = 0;
  size_t got = prefs.getBytes("tmill64", &prevMillis, sizeof(prevMillis));
  if (got != sizeof(prevMillis)) prevMillis = 0;
  if (prevMillis != settings.savedMillis) prefs.putBytes("tmill64", &settings.savedMillis, sizeof(settings.savedMillis));
  // epoch and timezone (64-bit epoch)
  uint64_t prevEpoch = 0;
  got = prefs.getBytes("epoch64", &prevEpoch, sizeof(prevEpoch));
  if (got != sizeof(prevEpoch)) prevEpoch = 0;
  if (prevEpoch != settings.savedEpochSec) prefs.putBytes("epoch64", &settings.savedEpochSec, sizeof(settings.savedEpochSec));
  uint64_t prevEpochMillis = 0;
  got = prefs.getBytes("epochmill64", &prevEpochMillis, sizeof(prevEpochMillis));
  if (got != sizeof(prevEpochMillis)) prevEpochMillis = 0;
  if (prevEpochMillis != settings.savedEpochMillis) prefs.putBytes("epochmill64", &settings.savedEpochMillis, sizeof(settings.savedEpochMillis));
  int prevTz = prefs.getInt("tz", 0);
  if (prevTz != settings.tzOffsetMinutes) prefs.putInt("tz", settings.tzOffsetMinutes);

  // Write worker list only when it has been modified to avoid unnecessary writes
  if (workerListDirty) {
    prefs.putUInt("wcount", workerListCount);
    for (int i=0;i<workerListCount;i++) {
      char key[16];
      sprintf(key, "wmac%d", i);
      prefs.putBytes(key, workerList[i].mac, 6);
      sprintf(key, "wth%d", i);
      prefs.putUInt(key, workerList[i].threshold);
      sprintf(key, "wdur%d", i);
      prefs.putUInt(key, workerList[i].duration);
      sprintf(key, "wpi%d", i);
      prefs.putUChar(key, workerList[i].potIndex);
      sprintf(key, "wnm%d", i);
      prefs.putString(key, String(workerList[i].name));
    }
    workerListDirty = false;
  }
}

void loadSettings() {
  prefs.begin("plant", false);
  settings.name = prefs.getString("name", getWifiMacLast6());
  settings.waterInterval = prefs.getUInt("wint", 3600);
  settings.dataSyncInterval = prefs.getUInt("dsint", 3600);
  settings.activeStart = prefs.getUInt("start", 0);
  settings.activeEnd = prefs.getUInt("end", 24*3600-1);
  settings.savedTimeOfDaySec = prefs.getUInt("tsec", 0);
  // load 64-bit persisted values (fall back to 0 on missing)
  size_t got = prefs.getBytes("tmill64", &settings.savedMillis, sizeof(settings.savedMillis));
  if (got != sizeof(settings.savedMillis)) settings.savedMillis = 0;
  got = prefs.getBytes("epoch64", &settings.savedEpochSec, sizeof(settings.savedEpochSec));
  if (got != sizeof(settings.savedEpochSec)) settings.savedEpochSec = 0;
  got = prefs.getBytes("epochmill64", &settings.savedEpochMillis, sizeof(settings.savedEpochMillis));
  if (got != sizeof(settings.savedEpochMillis)) settings.savedEpochMillis = 0;
  settings.tzOffsetMinutes = (int16_t)prefs.getInt("tz", 0);
  // If the persisted millis appears to be from a previous boot (larger than
  // the current millis()), it will cause huge deltas when computing current
  // time. Adjust to current millis() to keep time calculations sane after a
  // reboot. Persist the corrected value back to NVS.
  uint64_t nowMillis = (uint64_t)millis();
  if (settings.savedMillis > nowMillis) {
    settings.savedMillis = nowMillis;
    prefs.putBytes("tmill64", &settings.savedMillis, sizeof(settings.savedMillis));
  }
  // same sanity check for persisted epoch millis
  if (settings.savedEpochMillis > nowMillis) {
    settings.savedEpochMillis = nowMillis;
    prefs.putBytes("epochmill64", &settings.savedEpochMillis, sizeof(settings.savedEpochMillis));
  }
  if (settings.waterInterval < WATER_INTERVAL_MIN) settings.waterInterval = WATER_INTERVAL_MIN;
  if (settings.waterInterval > WATER_INTERVAL_MAX) settings.waterInterval = WATER_INTERVAL_MAX;
  if (settings.dataSyncInterval < DATA_SYNC_INTERVAL_MIN) settings.dataSyncInterval = DATA_SYNC_INTERVAL_MIN;
  if (settings.dataSyncInterval > DATA_SYNC_INTERVAL_MAX) settings.dataSyncInterval = DATA_SYNC_INTERVAL_MAX;
  // load worker list
  workerListCount = prefs.getUInt("wcount", 0);
  if (workerListCount > MAX_WORKER_COUNT) workerListCount = MAX_WORKER_COUNT;
  for (int i=0;i<workerListCount;i++) {
    char key[16];
    sprintf(key, "wmac%d", i);
    prefs.getBytes(key, workerList[i].mac, 6);
    sprintf(key, "wth%d", i);
    workerList[i].threshold = prefs.getUInt(key, 2000);
    sprintf(key, "wdur%d", i);
    workerList[i].duration = prefs.getUInt(key, 5);
    sprintf(key, "wpi%d", i);
    workerList[i].potIndex = prefs.getUChar(key, 0);
    sprintf(key, "wnm%d", i);
    String nm = prefs.getString(key, "");
    if (nm.length()) {
      strncpy(workerList[i].name, nm.c_str(), sizeof(workerList[i].name)-1);
      workerList[i].name[sizeof(workerList[i].name)-1] = '\0';
    } else workerList[i].name[0] = '\0';
  }
}

// Function to connect to Wi-Fi using stored credentials
bool connectToWiFi() {
  prefs.begin("plant", true);
  String savedSSID = prefs.getString("ssid", "");
  String savedPassword = prefs.getString("password", "");

  if (savedSSID.isEmpty()) {
    LOG("No saved Wi-Fi credentials found.");
    return false;
  }

  LOG("Connecting to saved Wi-Fi: %s", savedSSID.c_str());
  if (savedPassword.isEmpty()) {
    WiFi.begin(savedSSID.c_str());
  } else {
    WiFi.begin(savedSSID.c_str(), savedPassword.c_str());
  }

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startTime > 10000) {
      LOG("Failed to connect to saved Wi-Fi.");
      return false;
    }
    delay(500);
  }

  LOG("Successfully connected to %s", savedSSID.c_str());
  return true;
}

void saveWifiCred(const char *ssid, const char *password) {
  LOG("Provisioning successful! SSID: %s", ssid);
  prefs.begin("plant", false);
  // Store the credentials and API key in preferences
  prefs.putString("ssid", String(ssid));
  if (password) {
    prefs.putString("password", String(password));
  }
  
  LOG("Credentials saved.");
}

bool addWorkerByHex(const String &macHex, uint16_t threshold, uint16_t duration, const String &name) {
  if (workerListCount >= MAX_WORKER_COUNT) return false;
  uint8_t mac[6];
  if (!macFromHexString(macHex, mac)) return false;
  // auto-assign potIndex: pick next available index for this MAC
  uint8_t newPotIndex = 0;
  for (int i=0;i<workerListCount;i++) if (memcmp(workerList[i].mac, mac, 6)==0) if (workerList[i].potIndex >= newPotIndex) newPotIndex = workerList[i].potIndex + 1;
  if (newPotIndex >= MAX_POTS_PER_DEVICE) return false;
  memcpy(workerList[workerListCount].mac, mac, 6);
  workerList[workerListCount].threshold = threshold;
  workerList[workerListCount].duration = duration;
  workerList[workerListCount].potIndex = newPotIndex;
  if (name.length()) {
    strncpy(workerList[workerListCount].name, name.c_str(), sizeof(workerList[workerListCount].name)-1);
    workerList[workerListCount].name[sizeof(workerList[workerListCount].name)-1] = '\0';
  } else workerList[workerListCount].name[0] = '\0';
  workerListCount++;
  // ensure a placeholder exists in discovery cache so updates can be applied
  btMainEnsureNodeExists(mac);
  workerListDirty = true;
  saveSettings();
  return true;
}

bool removeWorkerByHex(const String &macHex) {
  uint8_t mac[6];
  if (!macFromHexString(macHex, mac)) return false;
  int write = 0;
  bool removed = false;
  for (int i = 0; i < workerListCount; ++i) {
    if (memcmp(workerList[i].mac, mac, 6) == 0) {
      removed = true;
      continue;
    }
    if (write != i) workerList[write] = workerList[i];
    ++write;
  }
  if (removed) {
    workerListCount = write;
    btMainRemoveNodeByMac(mac);
    workerListDirty = true;
    saveSettings();
    return true;
  }
  return false;
}

// Ensure per-pot WorkerConfig entries exist for a given worker MAC
void ensureWorkerConfigsForMac(const uint8_t mac[6], uint8_t potCount) {
  if (potCount == 0) return;
  if (potCount > MAX_POTS_PER_DEVICE) potCount = MAX_POTS_PER_DEVICE;
  // Count existing entries and find a template entry if present
  int existing = 0;
  int firstIndex = -1;
  for (int i = 0; i < workerListCount; ++i) {
    if (memcmp(workerList[i].mac, mac, 6) == 0) {
      ++existing;
      if (firstIndex == -1) firstIndex = i;
    }
  }
  if (existing >= potCount) return; // already have enough configs

  uint16_t defThreshold = 2000;
  uint16_t defDuration = 5;
  char defName[32] = {0};
  if (firstIndex != -1) {
    defThreshold = workerList[firstIndex].threshold;
    defDuration = workerList[firstIndex].duration;
    strncpy(defName, workerList[firstIndex].name, sizeof(defName)-1);
    defName[sizeof(defName)-1] = '\0';
  }

  for (int pi = 0; pi < potCount; ++pi) {
    bool found = false;
    for (int i = 0; i < workerListCount; ++i) {
      if (memcmp(workerList[i].mac, mac, 6) == 0 && workerList[i].potIndex == (uint8_t)pi) { found = true; break; }
    }
    if (found) continue;
    if (workerListCount >= MAX_WORKER_COUNT) break;
    memcpy(workerList[workerListCount].mac, mac, 6);
    workerList[workerListCount].threshold = defThreshold;
    workerList[workerListCount].duration = defDuration;
    workerList[workerListCount].potIndex = (uint8_t)pi;
    strncpy(workerList[workerListCount].name, defName, sizeof(workerList[workerListCount].name)-1);
    workerList[workerListCount].name[sizeof(workerList[workerListCount].name)-1] = '\0';
    workerListCount++;
  }
  workerListDirty = true;
  saveSettings();
}

// Clear all persisted settings and reset in-memory defaults
bool clearAllSettings() {
  prefs.begin("plant", false);
  prefs.clear();
  prefs.end();
  // reset in-memory state to defaults
  workerListCount = 0;
  workerListDirty = true;
  settings.name = getWifiMacLast6();
  settings.waterInterval = 3600;
  settings.dataSyncInterval = 3600;
  settings.activeStart = 0;
  settings.activeEnd = 24*3600-1;
  settings.savedTimeOfDaySec = 0;
  settings.savedMillis = (uint64_t)millis();
  settings.savedEpochSec = 0;
  settings.savedEpochMillis = (uint64_t)millis();
  settings.tzOffsetMinutes = 0;
  saveSettings();
  return true;
}

// Update existing worker identified by MAC. Returns true if updated.
bool updateWorkerByHex(const String &macHex, uint16_t threshold, uint16_t duration, const String &name, int potIndex) {
  uint8_t mac[6];
  if (!macFromHexString(macHex, mac)) return false;
  for (int i=0;i<workerListCount;i++) {
    if (memcmp(workerList[i].mac, mac, 6)==0 && (potIndex < 0 || workerList[i].potIndex == (uint8_t)potIndex)) {
      workerList[i].threshold = threshold;
      workerList[i].duration = duration;
      if (name.length()) {
        strncpy(workerList[i].name, name.c_str(), sizeof(workerList[i].name)-1);
        workerList[i].name[sizeof(workerList[i].name)-1] = '\0';
      } else workerList[i].name[0] = '\0';
      workerListDirty = true;
      saveSettings();
      return true;
    }
  }
  return false;
}

// Function to clear saved WiFi credentials
bool clearWifiCredentials() {
  prefs.begin("plant", false);
  prefs.remove("ssid");
  prefs.remove("password");
  
  LOG("WiFi credentials cleared.");
  return true;
}

// Mark settings dirty for debounced persistence
void markSettingsDirty() {
  settingsDirty = true;
  settingsDirtyAt = millis();
}

// Called periodically (e.g., from main loop) to persist settings when debounce window elapses
void maybeSaveSettings() {
  if (settingsDirty && (millis() - settingsDirtyAt > SAVE_DEBOUNCE_MS)) {
    saveSettings();
    settingsDirty = false;
  }
}
