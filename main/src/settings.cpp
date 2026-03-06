#include "Config.h"
#include "Utility.h"
#include <Preferences.h>

static Preferences prefs;

Settings settings;
volatile unsigned long autoEnabled = false;
volatile unsigned long lastWateringEnd = 0;
volatile unsigned long lastDataSync = 0;

WorkerConfig workerList[MAX_WORKER_COUNT];
int workerListCount = 0;

static const uint32_t WATER_INTERVAL_MIN = 60;
static const uint32_t WATER_INTERVAL_MAX = 2419200;
static const uint32_t DATA_SYNC_INTERVAL_MIN = 60;
static const uint32_t DATA_SYNC_INTERVAL_MAX = 2419200;

void saveSettings() {
  prefs.putString("name", settings.name);
  prefs.putUInt("wint", settings.waterInterval);
  prefs.putUInt("dsint", settings.dataSyncInterval);
  prefs.putUInt("start", settings.activeStart);
  prefs.putUInt("end", settings.activeEnd);
  prefs.putUInt("tsec", settings.savedTimeOfDaySec);
  prefs.putULong("tmill", settings.savedMillis);
  // save worker list
  prefs.putUInt("wcount", workerListCount);
  for (int i=0;i<workerListCount;i++) {
    char key[16];
    sprintf(key, "wmac%d", i);
    prefs.putBytes(key, workerList[i].mac, 6);
    sprintf(key, "wth%d", i);
    prefs.putUInt(key, workerList[i].threshold);
    sprintf(key, "wdur%d", i);
    prefs.putUInt(key, workerList[i].duration);
    sprintf(key, "wnm%d", i);
    prefs.putString(key, String(workerList[i].name));
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
  settings.savedMillis = prefs.getULong("tmill", 0);
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
    sprintf(key, "wnm%d", i);
    String nm = prefs.getString(key, "");
    if (nm.length()) {
      strncpy(workerList[i].name, nm.c_str(), sizeof(workerList[i].name)-1);
      workerList[i].name[sizeof(workerList[i].name)-1] = '\0';
    } else workerList[i].name[0] = '\0';
  }
}

bool addWorkerByHex(const String &macHex, uint16_t threshold, uint16_t duration, const String &name) {
  if (workerListCount >= MAX_WORKER_COUNT) return false;
  uint8_t mac[6];
  if (!macFromHexString(macHex, mac)) return false;
  // check duplicate
  for (int i=0;i<workerListCount;i++) if (memcmp(workerList[i].mac, mac, 6)==0) return false;
  memcpy(workerList[workerListCount].mac, mac, 6);
  workerList[workerListCount].threshold = threshold;
  workerList[workerListCount].duration = duration;
  if (name.length()) {
    strncpy(workerList[workerListCount].name, name.c_str(), sizeof(workerList[workerListCount].name)-1);
    workerList[workerListCount].name[sizeof(workerList[workerListCount].name)-1] = '\0';
  } else workerList[workerListCount].name[0] = '\0';
  workerListCount++;
  saveSettings();
  return true;
}

bool removeWorkerByHex(const String &macHex) {
  uint8_t mac[6];
  if (!macFromHexString(macHex, mac)) return false;
  for (int i=0;i<workerListCount;i++) {
    if (memcmp(workerList[i].mac, mac, 6)==0) {
      // shift
      for (int j=i;j<workerListCount-1;j++) workerList[j]=workerList[j+1];
      workerListCount--;
      saveSettings();
      return true;
    }
  }
  return false;
}

// Update existing worker identified by MAC. Returns true if updated.
bool updateWorkerByHex(const String &macHex, uint16_t threshold, uint16_t duration, const String &name) {
  uint8_t mac[6];
  if (!macFromHexString(macHex, mac)) return false;
  for (int i=0;i<workerListCount;i++) {
    if (memcmp(workerList[i].mac, mac, 6)==0) {
      workerList[i].threshold = threshold;
      workerList[i].duration = duration;
      if (name.length()) {
        strncpy(workerList[i].name, name.c_str(), sizeof(workerList[i].name)-1);
        workerList[i].name[sizeof(workerList[i].name)-1] = '\0';
      } else workerList[i].name[0] = '\0';
      saveSettings();
      return true;
    }
  }
  return false;
}
