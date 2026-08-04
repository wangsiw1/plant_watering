#include "WebUI.h"
#include "config.h"
#include "ClockManager.h"
#include "Utility.h"
#include "Sensor.h"
#include "Pump.h"
#include <WebServer.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>
#include <cctype>
#include <cstring>
#include "BluetoothMain.h"
// extra system headers for diagnostics
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_system.h>
#include <WiFi.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>
#include "esp_timer.h"
#include "WateringManager.h"
#include "OtaManager.h"
#include "WorkerOtaManager.h"
#include "generated_web_assets.h"

static WebServer server(80);

static TaskHandle_t sLoopTask = nullptr;
static TaskHandle_t sWebTask = nullptr;
static TaskHandle_t sSensorTask = nullptr;
static TaskHandle_t sWateringTask = nullptr;
static bool sOtaSawFile = false;
static bool sOtaAccepted = false;
static bool sOtaFinished = false;
static char sOtaRequestError[48] = {};
static bool sWorkerOtaSawFile = false;
static bool sWorkerOtaAccepted = false;
static bool sWorkerOtaFinished = false;
static char sWorkerOtaRequestError[80] = {};
static String sJsonResponse;

constexpr size_t JSON_STREAM_BUFFER_SIZE = 512;

static void sendJsonResponse(int statusCode, JsonDocument& doc) {
  size_t length = measureJson(doc);
  sJsonResponse.clear();
  if (!sJsonResponse.reserve(length)) {
    server.send(500, "text/plain", "JSON_ALLOC_FAILED");
    return;
  }
  serializeJson(doc, sJsonResponse);
  server.send(statusCode, "application/json", sJsonResponse);
}

static void sendJsonResponseBuffered(JsonDocument& doc) {
  server.setContentLength(measureJson(doc));
  server.send(200, "application/json", "");
  WiFiClient client = server.client();
  WriteBufferingStream bufferedClient(client, JSON_STREAM_BUFFER_SIZE);
  serializeJson(doc, bufferedClient);
  bufferedClient.flush();
}

void webSetDiagnosticsTaskHandles(TaskHandle_t loopTask,
                                  TaskHandle_t webTask,
                                  TaskHandle_t sensorTask,
                                  TaskHandle_t wateringTask) {
  sLoopTask = loopTask;
  sWebTask = webTask;
  sSensorTask = sensorTask;
  sWateringTask = wateringTask;
}

// One-shot timer used to turn the pump off without allocating a task.
static TimerHandle_t sPumpOffTimer = nullptr;
static SemaphoreHandle_t sManualPumpMutex = nullptr;
static bool sManualPumpActive = false;
static int64_t sManualPumpDeadlineUs = 0;

static void pumpOffTimerCallback(TimerHandle_t xTimer) {
  if (!sManualPumpMutex) return;
  xSemaphoreTake(sManualPumpMutex, portMAX_DELAY);
  if (!sManualPumpActive) {
    xSemaphoreGive(sManualPumpMutex);
    return;
  }

  int64_t nowUs = esp_timer_get_time();
  if (nowUs < sManualPumpDeadlineUs) {
    uint32_t remainingMs = static_cast<uint32_t>(
        (sManualPumpDeadlineUs - nowUs + 999) / 1000);
    TickType_t remainingTicks = pdMS_TO_TICKS(remainingMs);
    if (remainingTicks == 0) remainingTicks = 1;
    if (xTimerChangePeriod(xTimer, remainingTicks, 0) == pdPASS) {
      xSemaphoreGive(sManualPumpMutex);
      return;
    }
  }

  sManualPumpActive = false;
  sManualPumpDeadlineUs = 0;
  pumpOff();
  xSemaphoreGive(sManualPumpMutex);
}

static bool ensureManualPumpResources() {
  if (!sManualPumpMutex) sManualPumpMutex = xSemaphoreCreateMutex();
  if (!sManualPumpMutex) return false;
  if (!sPumpOffTimer) {
    sPumpOffTimer = xTimerCreate("pumpOff", pdMS_TO_TICKS(10000), pdFALSE,
                                nullptr, pumpOffTimerCallback);
  }
  return sPumpOffTimer != nullptr;
}

static bool manualPumpIsActive() {
  if (!sManualPumpMutex) return false;
  xSemaphoreTake(sManualPumpMutex, portMAX_DELAY);
  bool active = sManualPumpActive;
  xSemaphoreGive(sManualPumpMutex);
  return active;
}

static void stopManualPump() {
  if (!sManualPumpMutex) return;
  xSemaphoreTake(sManualPumpMutex, portMAX_DELAY);
  bool wasActive = sManualPumpActive;
  sManualPumpActive = false;
  sManualPumpDeadlineUs = 0;
  if (wasActive) pumpOff();
  xSemaphoreGive(sManualPumpMutex);
  if (sPumpOffTimer) xTimerStop(sPumpOffTimer, 0);
}

static bool startManualPump(uint8_t timeoutSeconds) {
  if (!ensureManualPumpResources()) return false;
  TickType_t timeoutTicks =
      pdMS_TO_TICKS(static_cast<uint32_t>(timeoutSeconds) * 1000u);
  if (timeoutTicks == 0) timeoutTicks = 1;

  xSemaphoreTake(sManualPumpMutex, portMAX_DELAY);
  sManualPumpDeadlineUs =
      esp_timer_get_time() + static_cast<int64_t>(timeoutSeconds) * 1000000LL;
  if (xTimerChangePeriod(sPumpOffTimer, timeoutTicks, 0) != pdPASS) {
    sManualPumpDeadlineUs = 0;
    sManualPumpActive = false;
    pumpOff();
    xSemaphoreGive(sManualPumpMutex);
    return false;
  }
  sManualPumpActive = true;
  pumpOn();
  xSemaphoreGive(sManualPumpMutex);
  return true;
}

static const char *stateToString(State currentState) {
  switch (currentState) {
    case READY: return "READY";
    case SYNCING: return "SYNCING";
    case WATERING: return "WATERING";
    case SLEEPING: return "SLEEPING";
    case UPDATING: return "UPDATING";
    default: return "UNKNOWN";
  }
}

static bool isValidWebName(const char* value) {
  if (!value) return false;
  char checked[UTF8_NAME_STORAGE_BYTES];
  copyUtf8Truncated(value, checked, sizeof(checked));
  if (strcmp(value, checked) != 0) return false;
  for (const unsigned char* p = reinterpret_cast<const unsigned char*>(value);
       *p; ++p) {
    if (*p < 0x20 || *p == 0x7f) return false;
    if (*p == 0xc2 && p[1] >= 0x80 && p[1] <= 0x9f) return false;
  }
  return true;
}

static void setOtaRequestError(const char* error) {
  if (!error) error = "update_failed";
  strncpy(sOtaRequestError, error, sizeof(sOtaRequestError) - 1);
  sOtaRequestError[sizeof(sOtaRequestError) - 1] = '\0';
}

static void setWorkerOtaRequestError(const char* error) {
  if (!error) error = "worker_update_failed";
  strncpy(sWorkerOtaRequestError, error, sizeof(sWorkerOtaRequestError) - 1);
  sWorkerOtaRequestError[sizeof(sWorkerOtaRequestError) - 1] = '\0';
}

static void sendHtmlDocument(const uint8_t* html, size_t length) {
  server.sendHeader("Content-Encoding", "gzip");
  server.sendHeader("Vary", "Accept-Encoding");
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(length);
  server.send(200, "text/html; charset=utf-8", "");
  WiFiClient client = server.client();
  while (length > 0) {
    size_t chunk = min(length, static_cast<size_t>(1460));
    size_t written = client.write(html, chunk);
    if (written == 0) break;
    html += written;
    length -= written;
  }
}

void handleRoot() {
  sendHtmlDocument(ROOT_HTML_GZ, ROOT_HTML_GZ_LEN);
}

void handleOtaPage() {
  sendHtmlDocument(OTA_HTML_GZ, OTA_HTML_GZ_LEN);
}

void handleStatus() {
  int tank = getTankLevel();
  unsigned long now = millis()/1000;
  uint32_t tod = clockGetCurrentTimeOfDaySec();
  Settings settings{};
  RuntimeSnapshot runtime{};
  ClockStatusSnapshot clock{};
  getSettingsSnapshot(settings);
  getRuntimeSnapshot(runtime);
  clockGetStatus(clock);
  JsonDocument doc;
  doc["name"] = settings.name;
  doc["auto"] = runtime.autoEnabled;
  doc["tank"] = tank;
  doc["now"] = now;
  doc["timeOfDaySec"] = tod;
  doc["waterInterval"] = settings.waterInterval;
  doc["dataSyncInterval"] = settings.dataSyncInterval;
  doc["pumpDelay"] = settings.pumpDelaySeconds;
  doc["manualPumpTimeout"] = settings.manualPumpTimeoutSeconds;
  doc["manualPumpActive"] = manualPumpIsActive();
  doc["tzOffsetMinutes"] = settings.tzOffsetMinutes;
  doc["activeStart"] = settings.activeStart;
  doc["activeEnd"] = settings.activeEnd;
  doc["workerCount"] = getWorkerConfigCount();
  doc["maxWorkerCount"] = MAX_WORKER_COUNT;
  doc["state"] = stateToString(runtime.state);
  doc["clockValid"] = clock.valid;
  doc["clockSource"] = clockSourceName(clock.source);
  doc["clockSyncPending"] = clock.ntpSyncPending;
  if (clock.ntpSyncError[0]) doc["clockSyncError"] = clock.ntpSyncError;
  else doc["clockSyncError"] = nullptr;
  int64_t nowUs = esp_timer_get_time();
  if (runtime.autoEnabled && runtime.lastDataSyncUs != 0 &&
      nowUs >= runtime.lastDataSyncUs) {
    doc["lastDataSyncAgo"] =
        static_cast<uint64_t>((nowUs - runtime.lastDataSyncUs) / 1000000LL);
  } else {
    doc["lastDataSyncAgo"] = nullptr;
  }
  if (runtime.autoEnabled && runtime.nextDataSyncUs != 0 &&
      runtime.nextDataSyncUs >= nowUs) {
    doc["nextDataSyncIn"] =
        static_cast<uint64_t>((runtime.nextDataSyncUs - nowUs) / 1000000LL);
  } else {
    doc["nextDataSyncIn"] = nullptr;
  }
  uint64_t curEpoch = clockGetCurrentEpochSec();
  if (curEpoch) doc["epoch"] = curEpoch;
  if (settings.lastWateringUtcSec != 0 &&
      curEpoch >= settings.lastWateringUtcSec) {
    doc["lastAutoWateringAgo"] = curEpoch - settings.lastWateringUtcSec;
  } else {
    doc["lastAutoWateringAgo"] = nullptr;
  }

  sendJsonResponse(200, doc);
}

void handleDiagnostics() {
  JsonDocument doc;
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t minFreeHeap = ESP.getMinFreeHeap();
  uint32_t largestFreeBlock = ESP.getMaxAllocHeap();
  uint32_t heapSize = ESP.getHeapSize();
  uint32_t freeSketch = ESP.getFreeSketchSpace();
  uint32_t sketchSize = ESP.getSketchSize();
  uint32_t flashChipSize = ESP.getFlashChipSize();
  uint32_t chipRev = ESP.getChipRevision();
  doc["freeHeap"] = freeHeap;
  doc["minFreeHeap"] = minFreeHeap;
  doc["largestFreeBlock"] = largestFreeBlock;
  doc["heapSize"] = heapSize;
  doc["freeSketchSpace"] = freeSketch;
  doc["sketchSize"] = sketchSize;
  doc["flashChipSize"] = flashChipSize;
  doc["chipRevision"] = chipRev;

  auto addStackHighWaterMark = [&doc](const char* name, TaskHandle_t task) {
    if (task) doc[name] = uxTaskGetStackHighWaterMark(task);
    else doc[name] = nullptr;
  };
  addStackHighWaterMark("loopStackMinFree", sLoopTask);
  addStackHighWaterMark("webStackMinFree", sWebTask);
  addStackHighWaterMark("sensorStackMinFree", sSensorTask);
  addStackHighWaterMark("wateringStackMinFree", sWateringTask);
  UBaseType_t btSenderStackMinFree = 0;
  if (BT_TLV::btCommonGetSenderStackHighWaterMark(btSenderStackMinFree)) {
    doc["btSenderStackMinFree"] = btSenderStackMinFree;
  } else {
    doc["btSenderStackMinFree"] = nullptr;
  }
  // NVS stats (may fail if NVS not initialized)
  nvs_stats_t nvs_stats; // from nvs_flash.h
  if (nvs_get_stats(NULL, &nvs_stats) == ESP_OK) {
    doc["nvs_entries"] = (uint32_t)nvs_stats.used_entries;
    doc["nvs_total_entries"] = (uint32_t)nvs_stats.total_entries;
  } else {
    doc["nvs_entries"] = nullptr;
    doc["nvs_total_entries"] = nullptr;
  }
  // WiFi RSSI if connected
  if (WiFi.status() == WL_CONNECTED) doc["rssi"] = WiFi.RSSI();
  else doc["rssi"] = nullptr;

  sendJsonResponse(200, doc);
}

void handleTimeSync() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  JsonDocument doc;
  if (WiFi.status() != WL_CONNECTED) {
    doc["ok"] = false;
    doc["error"] = "WIFI_DISCONNECTED";
    sendJsonResponse(503, doc);
    return;
  }
  bool accepted = clockRequestNtpSync();
  doc["ok"] = accepted;
  doc["pending"] = accepted;
  if (!accepted) doc["error"] = "WIFI_DISCONNECTED";
  sendJsonResponse(accepted ? 202 : 503, doc);
}

void handlePumpToggle() {
  if (manualPumpIsActive()) {
    stopManualPump();
    server.send(200, "text/plain", "OK");
    return;
  }
  if (otaIsActive() || workerOtaIsActive()) {
    server.send(409, "text/plain", "OTA_ACTIVE");
    return;
  }
  if (getAutoEnabled()) {
    server.send(409, "text/plain", "AUTO_MODE");
    return;
  }
  Settings settings{};
  getSettingsSnapshot(settings);
  if (!startManualPump(settings.manualPumpTimeoutSeconds)) {
    server.send(503, "text/plain", "PUMP_TIMER_FAILED");
    return;
  }
  server.send(200, "text/plain", "OK");
}

void handleSettings() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  String body = server.arg("plain");
  if (body.isEmpty()) { server.send(400, "text/plain", "EMPTY"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body) || !doc.is<JsonObject>()) {
    server.send(400, "text/plain", "BAD_JSON");
    return;
  }

  Settings next{};
  getSettingsSnapshot(next);
  bool oldAuto = getAutoEnabled();
  bool newAuto = oldAuto;
  bool hasManualTime = false;
  uint32_t manualTimeSec = 0;
  if (!doc["name"].isNull()) {
    if (!doc["name"].is<const char*>() ||
        !isValidWebName(doc["name"].as<const char*>())) {
      server.send(400, "text/plain", "BAD_NAME");
      return;
    }
    copyUtf8Truncated(doc["name"].as<const char*>(), next.name, sizeof(next.name));
  }
  if (!doc["auto"].isNull()) {
    if (!doc["auto"].is<bool>()) { server.send(400, "text/plain", "BAD_AUTO"); return; }
    newAuto = doc["auto"].as<bool>();
  }
  if (!doc["wint"].isNull()) {
    if (!doc["wint"].is<uint32_t>()) { server.send(400, "text/plain", "BAD_WATER_INTERVAL"); return; }
    next.waterInterval = doc["wint"].as<uint32_t>();
  }
  if (!doc["dsint"].isNull()) {
    if (!doc["dsint"].is<uint32_t>()) { server.send(400, "text/plain", "BAD_SYNC_INTERVAL"); return; }
    next.dataSyncInterval = doc["dsint"].as<uint32_t>();
  }
  if (!doc["pumpDelay"].isNull()) {
    if (!doc["pumpDelay"].is<uint32_t>()) { server.send(400, "text/plain", "BAD_PUMP_DELAY"); return; }
    uint32_t value = doc["pumpDelay"].as<uint32_t>();
    if (value > 30) { server.send(400, "text/plain", "BAD_PUMP_DELAY"); return; }
    next.pumpDelaySeconds = static_cast<uint8_t>(value);
  }
  if (!doc["manualPumpTimeout"].isNull()) {
    if (!doc["manualPumpTimeout"].is<uint32_t>()) { server.send(400, "text/plain", "BAD_MANUAL_PUMP_TIMEOUT"); return; }
    uint32_t value = doc["manualPumpTimeout"].as<uint32_t>();
    if (value < 1 || value > 60) { server.send(400, "text/plain", "BAD_MANUAL_PUMP_TIMEOUT"); return; }
    next.manualPumpTimeoutSeconds = static_cast<uint8_t>(value);
  }
  if (!doc["activeStart"].isNull()) {
    if (!doc["activeStart"].is<uint32_t>()) { server.send(400, "text/plain", "BAD_ACTIVE_START"); return; }
    next.activeStart = doc["activeStart"].as<uint32_t>();
  }
  if (!doc["activeEnd"].isNull()) {
    if (!doc["activeEnd"].is<uint32_t>()) { server.send(400, "text/plain", "BAD_ACTIVE_END"); return; }
    next.activeEnd = doc["activeEnd"].as<uint32_t>();
  }
  if (!doc["newTime"].isNull()) {
    if (!doc["newTime"].is<const char*>()) {
      server.send(400, "text/plain", "BAD_TIME");
      return;
    }
    const char* value = doc["newTime"];
    int hours = 0;
    int minutes = 0;
    char trailing = '\0';
    if (!value || sscanf(value, "%d:%d%c", &hours, &minutes, &trailing) != 2 ||
        hours < 0 || hours > 23 || minutes < 0 || minutes > 59) {
      server.send(400, "text/plain", "BAD_TIME");
      return;
    }
    hasManualTime = true;
    manualTimeSec = hours * 3600u + minutes * 60u;
  }

  if (doc["tzOffsetMinutes"].is<int>()) {
    int value = doc["tzOffsetMinutes"].as<int>();
    if (value < -840 || value > 840) {
      server.send(400, "text/plain", "BAD_TIMEZONE");
      return;
    }
    next.tzOffsetMinutes = value;
  } else if (doc["tzOffset"].is<const char*>()) {
    const char* value = doc["tzOffset"];
    int hours = 0;
    int minutes = 0;
    char trailing = '\0';
    if (!value || (value[0] != '+' && value[0] != '-') ||
        sscanf(value + 1, "%d:%d%c", &hours, &minutes, &trailing) != 2 ||
        hours > 14 || minutes > 59 || (hours == 14 && minutes != 0)) {
      server.send(400, "text/plain", "BAD_TIMEZONE");
      return;
    }
    int sign = value[0] == '-' ? -1 : 1;
    next.tzOffsetMinutes = sign * (hours * 60 + minutes);
  } else if (!doc["tzOffsetMinutes"].isNull() || !doc["tzOffset"].isNull()) {
    server.send(400, "text/plain", "BAD_TIMEZONE");
    return;
  }

  if (!applySettingsSnapshot(next)) {
    server.send(400, "text/plain", "BAD_SETTINGS");
    return;
  }
  if (newAuto && !oldAuto) {
    stopManualPump();
    pumpOff();
  }
  setAutoEnabled(newAuto);

  if (hasManualTime && !clockSetUserTimeOfDaySec(manualTimeSec)) {
      server.send(400, "text/plain", "BAD_TIME");
      return;
  }
  server.send(200, "text/plain", "OK");
}

void handleClearWifiCred() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  bool ok = clearWifiCredentials();
  server.send(ok?200:400, "text/plain", ok?"OK":"ERR");
}

void handleClearAllSettings() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  stopManualPump();
  pumpOff();
  bool ok = clearAllSettings();
  if (ok && WiFi.status() == WL_CONNECTED) clockRequestNtpSync();
  server.send(ok?200:400, "text/plain", ok?"OK":"ERR");
}

void handleNodes() {
  // Return configured worker devices with latest discovered readings when available
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  int64_t nowUs = esp_timer_get_time();
  int workerCount = getWorkerConfigCount();
  for (int i = 0; i < workerCount; ++i) {
    WorkerConfig wc{};
    if (!getWorkerConfigAt(i, wc)) continue;
    char macs[32];
    sprintf(macs, "%02X%02X%02X%02X%02X%02X", wc.mac[0],wc.mac[1],wc.mac[2],wc.mac[3],wc.mac[4],wc.mac[5]);
    JsonObject o = arr.add<JsonObject>();
    o["mac"] = macs;
    o["workerName"] = wc.workerName;
    o["potCount"] = wc.potCount;
    JsonArray pots = o["pots"].to<JsonArray>();
    WorkerNode node{};
    bool hasNode = btMainGetNodeByMac(wc.mac, node);
    if (hasNode) {
      o["battery"] = calculateBatteryPercent(node.batteryMv);
      o["batteryMv"] = node.batteryMv;
      if (node.lastRssi != 0x7FFF) o["rssi"] = (int)node.lastRssi;
      else o["rssi"] = nullptr;
      if (node.lastSyncUs != 0 && nowUs >= node.lastSyncUs) {
        o["lastSyncAgo"] =
            static_cast<uint64_t>((nowUs - node.lastSyncUs) / 1000000LL);
      } else {
        o["lastSyncAgo"] = nullptr;
      }
      o["nodePotCount"] = node.potCount;
    } else {
      o["battery"] = nullptr;
      o["batteryMv"] = nullptr;
      o["rssi"] = nullptr;
      o["lastSyncAgo"] = nullptr;
      o["nodePotCount"] = nullptr;
    }
    int potCount = wc.potCount;
    if (potCount > MAX_POTS_PER_DEVICE) potCount = MAX_POTS_PER_DEVICE;
    for (int p = 0; p < potCount; ++p) {
      JsonObject po = pots.add<JsonObject>();
      po["index"] = p;
      po["name"] = wc.potName[p];
      po["threshold"] = wc.thresholds[p];
      po["duration"] = wc.durations[p];
      if (hasNode && p < node.potCount) {
        po["soil"] =
            (int)getCorrectedSoilMoisture(node.batteryMv, node.soils[p]);
        po["soilRaw"] = (int)node.soils[p];
      } else {
        po["soil"] = nullptr;
        po["soilRaw"] = nullptr;
      }
      int64_t lastWaterUs = hasNode ? node.lastWaterUs[p] : 0;
      if (lastWaterUs != 0 && nowUs >= lastWaterUs) {
        po["lastWaterAgo"] =
            static_cast<uint64_t>((nowUs - lastWaterUs) / 1000000LL);
      } else {
        po["lastWaterAgo"] = nullptr;
      }
    }
  }
  sendJsonResponseBuffered(doc);
}

void handleWorkerAdd() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  String body = server.arg("plain");
  String mac=""; uint16_t th=2000; uint16_t dur=5; String name="";
  JsonDocument doc;
  if (body.isEmpty() || deserializeJson(doc, body) || !doc.is<JsonObject>()) {
    server.send(400, "text/plain", "BAD_JSON");
    return;
  }
  if (!doc["mac"].is<const char*>()) { server.send(400, "text/plain", "BAD_MAC"); return; }
  mac = String(doc["mac"].as<const char*>());
  if ((!doc["threshold"].isNull() && !doc["threshold"].is<uint32_t>()) ||
      (!doc["duration"].isNull() && !doc["duration"].is<uint32_t>())) {
    server.send(400, "text/plain", "BAD_VALUES");
    return;
  }
  uint32_t thresholdValue = doc["threshold"].isNull() ? 2000 : doc["threshold"].as<uint32_t>();
  uint32_t durationValue = doc["duration"].isNull() ? 5 : doc["duration"].as<uint32_t>();
  if (thresholdValue > 4095 || durationValue == 0 || durationValue > 60) {
    server.send(400, "text/plain", "BAD_VALUES");
    return;
  }
  th = thresholdValue;
  dur = durationValue;
  const char* nameValue = nullptr;
  if (!doc["workerName"].isNull()) {
    if (!doc["workerName"].is<const char*>()) { server.send(400, "text/plain", "BAD_NAME"); return; }
    nameValue = doc["workerName"].as<const char*>();
  } else if (!doc["name"].isNull()) {
    if (!doc["name"].is<const char*>()) { server.send(400, "text/plain", "BAD_NAME"); return; }
    nameValue = doc["name"].as<const char*>();
  }
  if (nameValue && !isValidWebName(nameValue)) { server.send(400, "text/plain", "BAD_NAME"); return; }
  if (nameValue) name = String(nameValue);
  bool ok = addWorkerByHex(mac.c_str(), th, dur, name.length()?name.c_str():nullptr);
  server.send(ok?200:400, "text/plain", ok?"OK":"ERR");
}

void handleWorkerRemove() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  String body = server.arg("plain");
  String mac="";
  JsonDocument doc;
  if (body.isEmpty() || deserializeJson(doc, body) || !doc.is<JsonObject>() ||
      !doc["mac"].is<const char*>()) {
    server.send(400, "text/plain", "BAD_JSON");
    return;
  }
  mac = String(doc["mac"].as<const char*>());
  bool ok = removeWorkerByHex(mac.c_str());
  server.send(ok?200:400, "text/plain", ok?"OK":"ERR");
}

void handleWorkerUpdate() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  String body = server.arg("plain");
  String mac=""; uint16_t th=2000; uint16_t dur=5; String name="";
  int potIndex = -1;
  bool hasNameField = false;
  if (body.length()) {
    JsonDocument doc;
    auto err = deserializeJson(doc, body);
    if (err || !doc.is<JsonObject>()) {
      server.send(400, "text/plain", "BAD_JSON");
      return;
    }
    if (!doc["mac"].is<const char*>()) { server.send(400, "text/plain", "BAD_MAC"); return; }
    mac = String(doc["mac"].as<const char*>());
    if ((!doc["threshold"].isNull() && !doc["threshold"].is<uint32_t>()) ||
        (!doc["duration"].isNull() && !doc["duration"].is<uint32_t>())) {
      server.send(400, "text/plain", "BAD_VALUES");
      return;
    }
    uint32_t thresholdValue = doc["threshold"].isNull() ? 2000 : doc["threshold"].as<uint32_t>();
    uint32_t durationValue = doc["duration"].isNull() ? 5 : doc["duration"].as<uint32_t>();
    if (thresholdValue > 4095 || durationValue == 0 || durationValue > 60) {
      server.send(400, "text/plain", "BAD_VALUES");
      return;
    }
    th = thresholdValue;
    dur = durationValue;
    if (!doc["name"].isNull()) {
      if (!doc["name"].is<const char*>() || !isValidWebName(doc["name"].as<const char*>())) { server.send(400, "text/plain", "BAD_NAME"); return; }
      name = String(doc["name"].as<const char*>()); hasNameField = true;
    }
    if (!doc["workerName"].isNull()) {
      if (!doc["workerName"].is<const char*>() || !isValidWebName(doc["workerName"].as<const char*>())) { server.send(400, "text/plain", "BAD_NAME"); return; }
      name = String(doc["workerName"].as<const char*>()); hasNameField = true;
    }
    if (!doc["potIndex"].isNull()) {
      if (!doc["potIndex"].is<int>()) { server.send(400, "text/plain", "BAD_POT_INDEX"); return; }
      potIndex = doc["potIndex"].as<int>();
      if (potIndex < 0 || potIndex >= MAX_POTS_PER_DEVICE) { server.send(400, "text/plain", "BAD_POT_INDEX"); return; }
    }
  } else {
    server.send(400, "text/plain", "EMPTY");
    return;
  }
  bool ok = updateWorkerByHex(mac.c_str(), th, dur, hasNameField ? name.c_str() : nullptr, potIndex);
  server.send(ok?200:400, "text/plain", ok?"OK":"ERR");
}

void handleWorkerWater() {
  if (otaIsActive() || workerOtaIsActive()) {
    server.send(409, "text/plain", "OTA_ACTIVE");
    return;
  }
  if (server.method() != HTTP_POST) { server.send(405); return; }
  if (getAutoEnabled()) { server.send(409, "text/plain", "AUTO_MODE"); return; }
  String body = server.arg("plain");
  String mac=""; int dur = -1;
  int potIndex = -1;
  JsonDocument doc;
  if (body.isEmpty() || deserializeJson(doc, body) || !doc.is<JsonObject>()) {
    server.send(400, "text/plain", "BAD_JSON");
    return;
  }
  if (!doc["mac"].is<const char*>()) { server.send(400, "text/plain", "BAD_MAC"); return; }
  mac = String(doc["mac"].as<const char*>());
  if (!doc["duration"].isNull()) {
    if (!doc["duration"].is<int>()) { server.send(400, "text/plain", "BAD_DURATION"); return; }
    dur = doc["duration"].as<int>();
    if (dur <= 0 || dur > 60) { server.send(400, "text/plain", "BAD_DURATION"); return; }
  }
  if (!doc["potIndex"].isNull()) {
    if (!doc["potIndex"].is<int>()) { server.send(400, "text/plain", "BAD_POT_INDEX"); return; }
    potIndex = doc["potIndex"].as<int>();
    if (potIndex < 0 || potIndex >= MAX_POTS_PER_DEVICE) { server.send(400, "text/plain", "BAD_POT_INDEX"); return; }
  }
  uint8_t macb[6];
  if (!macFromHexString(mac.c_str(), macb)) { server.send(400, "text/plain", "BAD_MAC"); return; }
  WorkerConfig worker{};
  if (!findWorkerConfigByMac(macb, worker)) {
    server.send(400, "text/plain", "NO_WORKER");
    return;
  }
  uint16_t potMask = 0;
  uint16_t durations[MAX_POTS_PER_DEVICE] = {};
  int durCount = 0;
  if (potIndex >= 0) {
    if (potIndex >= worker.potCount) {
      server.send(400, "text/plain", "BAD_POT_INDEX");
      return;
    }
    if (dur <= 0) dur = worker.durations[potIndex];
    potMask = (uint16_t)(1u << potIndex);
    durations[durCount++] = (uint16_t)dur;
  } else {
    int potCount = min(static_cast<int>(worker.potCount), MAX_POTS_PER_DEVICE);
    for (int p = 0; p < potCount; ++p) {
      potMask |= (uint16_t)(1u << p);
      durations[durCount++] =
          dur > 0 ? static_cast<uint16_t>(dur) : worker.durations[p];
    }
  }
  bool queued = wateringQueueManual(macb, potMask, durations, durCount);
  server.send(queued ? 202 : 503, "text/plain",
              queued ? "QUEUED" : "BUSY");
}

void handleOtaStatus() {
  OtaStatusSnapshot status{};
  otaGetStatus(status);
  JsonDocument doc;
  doc["state"] = otaStateName(status.state);
  doc["active"] = status.active;
  doc["packageBytesReceived"] = status.packageBytesReceived;
  doc["imageBytesReceived"] = status.imageBytesReceived;
  doc["expectedImageBytes"] = status.expectedImageBytes;
  doc["packageVersion"] = status.firmwareVersion;
  doc["packageHardware"] = status.hardwareTarget;
  doc["currentVersion"] = otaCurrentFirmwareVersion();
  doc["currentHardware"] = otaCurrentHardwareTarget();
  doc["freeHeapAtStart"] = status.freeHeapAtStart;
  doc["largestBlockAtStart"] = status.largestBlockAtStart;
  if (status.error[0]) doc["error"] = status.error;
  else doc["error"] = nullptr;

  sendJsonResponse(200, doc);
}

void handleOtaUploadChunk() {
  HTTPUpload& upload = server.upload();
  switch (upload.status) {
    case UPLOAD_FILE_START: {
      if (sOtaSawFile) {
        setOtaRequestError("multiple_files_not_allowed");
        otaUploadAbort(sOtaRequestError);
        sOtaAccepted = false;
        return;
      }
      sOtaSawFile = true;
      RuntimeSnapshot runtime{};
      getRuntimeSnapshot(runtime);
      if (runtime.state == WATERING || runtime.state == SYNCING ||
          otaIsActive() || workerOtaIsActive()) {
        setOtaRequestError("system_busy");
        return;
      }
      stopManualPump();
      pumpOff();
      if (!otaUploadStart()) {
        OtaStatusSnapshot status{};
        otaGetStatus(status);
        setOtaRequestError(status.error[0] ? status.error : "ota_start_failed");
        return;
      }
      sOtaAccepted = true;
      setRuntimeState(UPDATING);
      break;
    }

    case UPLOAD_FILE_WRITE:
      if (sOtaAccepted &&
          !otaUploadConsume(upload.buf, upload.currentSize)) {
        OtaStatusSnapshot status{};
        otaGetStatus(status);
        setOtaRequestError(status.error);
        sOtaAccepted = false;
      }
      break;

    case UPLOAD_FILE_END:
      sOtaFinished = true;
      if (sOtaAccepted && !otaUploadFinish()) {
        OtaStatusSnapshot status{};
        otaGetStatus(status);
        setOtaRequestError(status.error);
        sOtaAccepted = false;
      }
      break;

    case UPLOAD_FILE_ABORTED:
      setOtaRequestError("upload_aborted");
      otaUploadAbort(sOtaRequestError);
      otaReleaseFailedUpload();
      sOtaAccepted = false;
      sOtaFinished = true;
      break;

    default:
      break;
  }
}

void handleWorkerOtaStatus() {
  WorkerOtaStatusSnapshot status{};
  workerOtaGetStatus(status);
  JsonDocument doc;
  doc["state"] = workerOtaStateName(status.state);
  doc["active"] = status.active;
  if (status.targetMac[0] || status.targetMac[1] || status.targetMac[2] ||
      status.targetMac[3] || status.targetMac[4] || status.targetMac[5]) {
    char mac[13];
    macToHexLower(status.targetMac, mac);
    for (char* p = mac; *p; ++p) *p = toupper(*p);
    doc["targetMac"] = mac;
  } else {
    doc["targetMac"] = nullptr;
  }
  doc["packageBytesReceived"] = status.packageBytesReceived;
  doc["imageBytesSent"] = status.imageBytesSent;
  doc["expectedImageBytes"] = status.expectedImageBytes;
  doc["packageVersion"] = status.packageVersion;
  doc["packageHardware"] = status.packageHardware;
  doc["message"] = status.message;
  sendJsonResponse(200, doc);
}

void handleWorkerOtaUploadChunk() {
  HTTPUpload& upload = server.upload();
  switch (upload.status) {
    case UPLOAD_FILE_START: {
      if (sWorkerOtaSawFile) {
        setWorkerOtaRequestError("multiple_files_not_allowed");
        workerOtaPackageUploadAbort(sWorkerOtaRequestError);
        sWorkerOtaAccepted = false;
        return;
      }
      sWorkerOtaSawFile = true;
      RuntimeSnapshot runtime{};
      getRuntimeSnapshot(runtime);
      if (runtime.state == WATERING || runtime.state == SYNCING ||
          otaIsActive() || workerOtaIsActive()) {
        setWorkerOtaRequestError("system_busy");
        return;
      }
      stopManualPump();
      pumpOff();
      if (!workerOtaPackageUploadStart()) {
        WorkerOtaStatusSnapshot status{};
        workerOtaGetStatus(status);
        setWorkerOtaRequestError(status.message[0] ? status.message
                                                    : "worker_ota_start_failed");
        return;
      }
      sWorkerOtaAccepted = true;
      break;
    }

    case UPLOAD_FILE_WRITE:
      if (sWorkerOtaAccepted &&
          !workerOtaPackageUploadConsume(upload.buf, upload.currentSize)) {
        WorkerOtaStatusSnapshot status{};
        workerOtaGetStatus(status);
        setWorkerOtaRequestError(status.message);
        workerOtaPackageUploadAbort(sWorkerOtaRequestError);
        sWorkerOtaAccepted = false;
      }
      break;

    case UPLOAD_FILE_END:
      sWorkerOtaFinished = true;
      if (sWorkerOtaAccepted && !workerOtaPackageUploadFinish()) {
        WorkerOtaStatusSnapshot status{};
        workerOtaGetStatus(status);
        setWorkerOtaRequestError(status.message);
        sWorkerOtaAccepted = false;
      }
      break;

    case UPLOAD_FILE_ABORTED:
      setWorkerOtaRequestError("upload_aborted");
      workerOtaPackageUploadAbort(sWorkerOtaRequestError);
      sWorkerOtaAccepted = false;
      sWorkerOtaFinished = true;
      break;

    default:
      break;
  }
}

void handleWorkerOtaRequestComplete() {
  bool succeeded = sWorkerOtaSawFile && sWorkerOtaFinished &&
                   !sWorkerOtaRequestError[0];
  if (!sWorkerOtaSawFile && !sWorkerOtaRequestError[0]) {
    setWorkerOtaRequestError("missing_firmware_file");
  } else if (!succeeded && !sWorkerOtaRequestError[0]) {
    WorkerOtaStatusSnapshot status{};
    workerOtaGetStatus(status);
    setWorkerOtaRequestError(status.message[0] ? status.message
                                                : "worker_update_failed");
  }

  JsonDocument doc;
  doc["ok"] = succeeded;
  if (succeeded) {
    WorkerOtaStatusSnapshot status{};
    workerOtaGetStatus(status);
    doc["version"] = status.packageVersion;
    doc["imageSize"] = status.expectedImageBytes;
  } else {
    doc["error"] = sWorkerOtaRequestError;
  }
  String response;
  serializeJson(doc, response);
  server.send(succeeded ? 200 : 400, "application/json", response);

  if (!succeeded) workerOtaPackageUploadAbort(sWorkerOtaRequestError);
  sWorkerOtaSawFile = false;
  sWorkerOtaAccepted = false;
  sWorkerOtaFinished = false;
  sWorkerOtaRequestError[0] = '\0';
}

void handleWorkerOtaStart() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  if (otaIsActive() || workerOtaIsActive()) {
    server.send(409, "application/json", "{\"error\":\"OTA_ACTIVE\"}");
    return;
  }
  RuntimeSnapshot runtime{};
  getRuntimeSnapshot(runtime);
  if (runtime.state == WATERING || runtime.state == SYNCING) {
    server.send(409, "application/json", "{\"error\":\"system_busy\"}");
    return;
  }
  String body = server.arg("plain");
  JsonDocument doc;
  if (body.isEmpty() || deserializeJson(doc, body) || !doc.is<JsonObject>() ||
      !doc["mac"].is<const char*>()) {
    server.send(400, "application/json", "{\"error\":\"BAD_JSON\"}");
    return;
  }
  uint8_t mac[6];
  if (!macFromHexString(doc["mac"].as<const char*>(), mac)) {
    server.send(400, "application/json", "{\"error\":\"BAD_MAC\"}");
    return;
  }
  stopManualPump();
  pumpOff();
  bool started = workerOtaStartForWorker(mac);
  WorkerOtaStatusSnapshot status{};
  workerOtaGetStatus(status);
  JsonDocument out;
  out["ok"] = started;
  out["state"] = workerOtaStateName(status.state);
  out["message"] = status.message;
  if (!started) out["error"] = status.message[0] ? status.message : "start_failed";
  sendJsonResponse(started ? 202 : 400, out);
}

void handleOtaRequestComplete() {
  bool succeeded = sOtaSawFile && sOtaFinished && otaUploadSucceeded();
  if (!sOtaSawFile && !sOtaRequestError[0]) {
    setOtaRequestError("missing_firmware_file");
  } else if (!succeeded && !sOtaRequestError[0]) {
    OtaStatusSnapshot status{};
    otaGetStatus(status);
    setOtaRequestError(status.error[0] ? status.error : "update_failed");
  }

  JsonDocument doc;
  doc["ok"] = succeeded;
  if (succeeded) {
    OtaStatusSnapshot status{};
    otaGetStatus(status);
    doc["version"] = status.firmwareVersion;
    doc["imageSize"] = status.expectedImageBytes;
    doc["rebooting"] = true;
  } else {
    doc["error"] = sOtaRequestError;
  }
  String response;
  serializeJson(doc, response);
  server.send(succeeded ? 200 : 400, "application/json", response);

  if (!succeeded) {
    otaReleaseFailedUpload();
    setRuntimeState(getAutoEnabled() ? SLEEPING : READY);
  }
  sOtaSawFile = false;
  sOtaAccepted = false;
  sOtaFinished = false;
  sOtaRequestError[0] = '\0';
}

void webBegin() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/diagnostics", HTTP_GET, handleDiagnostics);
  server.on("/pump/toggle", HTTP_POST, handlePumpToggle);
  server.on("/settings", HTTP_ANY, handleSettings);
  server.on("/time/sync", HTTP_POST, handleTimeSync);
  server.on("/nodes", HTTP_GET, handleNodes);
  server.on("/worker/add", HTTP_POST, handleWorkerAdd);
  server.on("/worker/remove", HTTP_POST, handleWorkerRemove);
  server.on("/worker/update", HTTP_POST, handleWorkerUpdate);
  server.on("/worker/water", HTTP_POST, handleWorkerWater);
  server.on("/wifi/clear", HTTP_POST, handleClearWifiCred);
  server.on("/settings/clear_all", HTTP_POST, handleClearAllSettings);
  server.on("/ota", HTTP_GET, handleOtaPage);
  server.on("/ota/status", HTTP_GET, handleOtaStatus);
  server.on("/ota/main", HTTP_POST, handleOtaRequestComplete,
            handleOtaUploadChunk);
  server.on("/ota/worker/status", HTTP_GET, handleWorkerOtaStatus);
  server.on("/ota/worker", HTTP_POST, handleWorkerOtaRequestComplete,
            handleWorkerOtaUploadChunk);
  server.on("/ota/worker/start", HTTP_POST, handleWorkerOtaStart);
  server.begin();
  // Create the manual pump mutex and one-shot safety timer up front.
  ensureManualPumpResources();
}

void webHandleClientLoop() {
  server.handleClient();
}
