#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiProvisioner.h>

#include "config.h"
#include "Pump.h"
#include "Sensor.h"
#include "WebUI.h"
#include "BluetoothMain.h"
#include "Utility.h"
#include "ClockManager.h"
#include "WateringManager.h"
#include "OtaManager.h"
#include "WorkerOtaManager.h"
#include "AutoRecovery.h"
#include "AutoWateringPolicy.h"
#include "StatusLed.h"
#include "esp_timer.h"

namespace {
constexpr int64_t MANUAL_SYNC_INTERVAL_US = 30000000LL;
constexpr uint32_t STATUS_WAIT_MS = 3000;
constexpr uint32_t AUTO_WAKE_PADDING_MS = 10000;

String gMdnsName;
bool gMdnsStarted = false;

void startMdnsIfNeeded() {
  if (gMdnsStarted || gMdnsName.isEmpty()) return;
  if (!MDNS.begin(gMdnsName.c_str())) {
    LOG("Error starting mDNS");
    return;
  }
  gMdnsStarted = true;
  LOG("mDNS started: %s.local", gMdnsName.c_str());
}

struct AutoWaterRequest {
  uint8_t mac[6];
  uint16_t potMask;
  uint16_t durations[MAX_POTS_PER_DEVICE];
  uint8_t durationCount;
};

const char* btSendStatusName(BtSendStatus status) {
  switch (status) {
    case BtSendStatus::INVALID: return "INVALID";
    case BtSendStatus::QUEUE_FULL: return "QUEUE_FULL";
    case BtSendStatus::TRANSMIT_FAILED: return "TRANSMIT_FAILED";
    case BtSendStatus::SENT: return "SENT";
    case BtSendStatus::ACKED: return "ACKED";
    default: return "UNKNOWN";
  }
}

bool synchronizeWorker(const WorkerConfig& worker, int64_t cycleStartUs) {
  if (workerOtaIsActive()) return false;
  char workerMac[13];
  macToHexLower(worker.mac, workerMac);
  WorkerNode before{};
  uint32_t generation = 0;
  if (btMainGetNodeByMac(worker.mac, before)) generation = before.statusGeneration;
  LOG("WaterTask sync probe start worker=%s previous_generation=%lu",
      workerMac, static_cast<unsigned long>(generation));

  BT_TLV::BtBodyBuilder probe;
  BT_TLV::btBodyBegin(probe, BT_TLV::TYPE_CMD_PROBE);
  BtSendResult sent =
      btMainSendCommand(worker.mac, probe.data, probe.len, 2, 700);
  char ack[13];
  macToHexLower(sent.ackMac, ack);
  LOG("WaterTask sync probe result worker=%s status=%s started=%u ack=%s id=%08lx%08lx:%lu",
      workerMac, btSendStatusName(sent.status), sent.transmissionStarted ? 1u : 0u,
      ack, static_cast<unsigned long>(sent.messageId.sessionId >> 32),
      static_cast<unsigned long>(sent.messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(sent.messageId.sequence));
  if (!sent.transmissionStarted) return false;

  WorkerNode refreshed{};
  bool gotStatus = btMainWaitForStatus(worker.mac, generation,
                                       STATUS_WAIT_MS, refreshed);
  bool fresh = gotStatus && refreshed.lastSyncUs >= cycleStartUs;
  LOG("WaterTask sync status worker=%s got_status=%u fresh=%u new_generation=%lu",
      workerMac, gotStatus ? 1u : 0u, fresh ? 1u : 0u,
      static_cast<unsigned long>(gotStatus ? refreshed.statusGeneration : generation));
  return fresh;
}

int synchronizeConfiguredWorkers(int64_t cycleStartUs) {
  int refreshed = 0;
  int count = getWorkerConfigCount();
  LOG("WaterTask sync cycle start workers=%d", count);
  for (int i = 0; i < count; ++i) {
    WorkerConfig worker{};
    if (getWorkerConfigAt(i, worker) &&
        synchronizeWorker(worker, cycleStartUs)) {
      ++refreshed;
    }
  }
  LOG("WaterTask sync cycle end refreshed=%d/%d", refreshed, count);
  return refreshed;
}

bool wateringWindowOpen(const Settings& settings) {
  uint32_t secondOfDay = clockGetCurrentTimeOfDaySec();
  if (settings.activeStart <= settings.activeEnd) {
    return secondOfDay >= settings.activeStart &&
           secondOfDay <= settings.activeEnd;
  }
  return secondOfDay >= settings.activeStart ||
         secondOfDay <= settings.activeEnd;
}

void waterFreshWorkers(int64_t cycleStartUs, uint32_t nowUtc) {
  int count = getWorkerConfigCount();
  LOG("WaterTask watering fresh workers start count=%d", count);
  AutoWaterRequest requests[MAX_WORKER_COUNT] = {};
  int requestCount = 0;
  for (int i = 0; i < count; ++i) {
    WorkerConfig worker{};
    WorkerNode node{};
    if (!getWorkerConfigAt(i, worker) ||
        !btMainGetNodeByMac(worker.mac, node) ||
        node.lastSyncUs < cycleStartUs) {
      if (getWorkerConfigAt(i, worker)) {
        char workerMac[13];
        macToHexLower(worker.mac, workerMac);
        LOG("WaterTask watering skip worker=%s reason=not_fresh", workerMac);
      }
      continue;
    }

    char workerMac[13];
    macToHexLower(worker.mac, workerMac);
    int potCount = min(static_cast<int>(worker.potCount),
                       static_cast<int>(node.potCount));
    AutoWaterRequest request{};
    memcpy(request.mac, worker.mac, sizeof(request.mac));
    for (int pot = 0; pot < potCount; ++pot) {
      uint16_t correctedSoil =
          getCorrectedSoilMoisture(node.batteryMv, node.soils[pot]);
      if (autoWateringPotEligible(
              nowUtc, worker.lastAutoWateringUtcSec[pot],
              worker.waterIntervals[pot], node.soils[pot], correctedSoil,
              worker.thresholds[pot])) {
        request.potMask |= static_cast<uint16_t>(1u << pot);
        request.durations[request.durationCount++] = worker.durations[pot];
      }
    }
    LOG("WaterTask watering decision worker=%s pot_count=%d mask=0x%04x duration_count=%u",
        workerMac, potCount, static_cast<unsigned>(request.potMask),
        static_cast<unsigned>(request.durationCount));
    if (request.potMask != 0 && requestCount < MAX_WORKER_COUNT) {
      requests[requestCount++] = request;
    }
  }

  if (requestCount == 0) {
    LOG("WaterTask watering fresh workers end reason=no_candidates");
    return;
  }
  if (isTankLow()) {
    LOG("WaterTask watering batch rejected reason=tank_low tank_mv=%u",
        static_cast<unsigned>(getTankLevel()));
    return;
  }

  Settings settings{};
  getSettingsSnapshot(settings);
  LOG("WaterTask watering pump start candidates=%d prime_seconds=%u",
      requestCount, static_cast<unsigned>(settings.pumpDelaySeconds));
  pumpOn();

  int64_t primeDeadlineUs =
      esp_timer_get_time() +
      static_cast<int64_t>(settings.pumpDelaySeconds) * 1000000LL;
  bool tankLow = false;
  while (esp_timer_get_time() < primeDeadlineUs) {
    if (isTankLow()) {
      tankLow = true;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  if (isTankLow()) tankLow = true;

  CompletedAutoWatering completions[MAX_WORKER_COUNT] = {};
  size_t completionCount = 0;
  if (!tankLow) {
    for (int i = 0; i < requestCount; ++i) {
      if (isTankLow()) {
        tankLow = true;
        LOG("WaterTask watering batch abort reason=tank_low index=%d tank_mv=%u",
            i, static_cast<unsigned>(getTankLevel()));
        break;
      }
      AutoWaterRequest& request = requests[i];
      char workerMac[13];
      macToHexLower(request.mac, workerMac);
      bool completed = wateringExecuteWorker(
          request.mac, request.potMask, request.durations,
          request.durationCount, true);
      if (completed) {
        if (completionCount < MAX_WORKER_COUNT) {
          memcpy(completions[completionCount].mac, request.mac,
                 sizeof(completions[completionCount].mac));
          completions[completionCount].potMask = request.potMask;
          ++completionCount;
        }
      }
      LOG("WaterTask watering worker=%s completed=%u",
          workerMac, completed ? 1u : 0u);
      if (isTankLow()) {
        tankLow = true;
        LOG("WaterTask watering batch abort reason=tank_low_after_worker worker=%s tank_mv=%u",
            workerMac, static_cast<unsigned>(getTankLevel()));
        break;
      }
    }
  } else {
    LOG("WaterTask watering batch abort reason=tank_low_during_priming tank_mv=%u",
        static_cast<unsigned>(getTankLevel()));
  }

  LOG("WaterTask watering pump stop");
  pumpOff();
  if (completionCount != 0) {
    uint64_t completedUtc = clockGetCurrentEpochSec();
    if (completedUtc != 0 && completedUtc <= UINT32_MAX) {
      recordCompletedAutoWateringBatch(
          completions, completionCount, static_cast<uint32_t>(completedUtc));
    }
  }
  LOG("WaterTask watering fresh workers end completed=%u tank_low=%u",
      completionCount != 0 ? 1u : 0u, tankLow ? 1u : 0u);
}

void sendWorkerToSleepAt(const uint8_t mac[6], int64_t wakeAtUs) {
  int64_t remainingUs = wakeAtUs - esp_timer_get_time();
  uint32_t seconds = remainingUs > 0
                         ? static_cast<uint32_t>((remainingUs + 999999) / 1000000)
                         : 60;
  seconds = max(seconds, static_cast<uint32_t>(60));
  BT_TLV::BtBodyBuilder sleep;
  BT_TLV::btBodyBegin(sleep, BT_TLV::TYPE_CMD_SLEEP);
  if (!BT_TLV::btTlvAppendU32(sleep, BT_TLV::FIELD_SLEEP_SEC, seconds)) return;
  char workerMac[13];
  macToHexLower(mac, workerMac);
  LOG("WaterTask sleep send worker=%s seconds=%lu wake_at_us=%lld",
      workerMac, static_cast<unsigned long>(seconds),
      static_cast<long long>(wakeAtUs));
  BtSendResult result = btMainSendCommand(mac, sleep.data, sleep.len, 2, 700);
  LOG("WaterTask sleep result worker=%s status=%s started=%u id=%08lx%08lx:%lu",
      workerMac, btSendStatusName(result.status),
      result.transmissionStarted ? 1u : 0u,
      static_cast<unsigned long>(result.messageId.sessionId >> 32),
      static_cast<unsigned long>(result.messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(result.messageId.sequence));
}

void sendWorkersToSleepAt(int64_t wakeAtUs) {
  int count = getWorkerConfigCount();
  LOG("WaterTask sleep cycle start workers=%d wake_at_us=%lld",
      count, static_cast<long long>(wakeAtUs));
  for (int i = 0; i < count; ++i) {
    WorkerConfig worker{};
    if (getWorkerConfigAt(i, worker)) sendWorkerToSleepAt(worker.mac, wakeAtUs);
  }
  LOG("WaterTask sleep cycle end");
}

void handleWorkerSleepRequests(int64_t wakeAtUs) {
  uint8_t mac[6];
  while (btMainTakeSleepRequest(mac)) {
    char workerMac[13];
    macToHexLower(mac, workerMac);
    LOG("WaterTask sleep request handle worker=%s", workerMac);
    sendWorkerToSleepAt(mac, wakeAtUs);
  }
}
}

void TaskWeb(void*) {
  for (;;) {
    webHandleClientLoop();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void TaskSensor(void*) {
  for (;;) {
    readTankLevel();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void TaskWatering(void*) {
  int64_t lastSyncUs = 0;
  int64_t lastAutoWaitLogUs = 0;
  bool previousAuto = false;
  bool loggedManualMode = false;

  for (;;) {
    if (otaIsActive() || workerOtaIsActive()) {
      setRuntimeState(UPDATING);
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }
    bool automatic = getAutoEnabled();
    if (!automatic) {
      if (!loggedManualMode || previousAuto) {
        LOG("WaterTask mode=manual");
        loggedManualMode = true;
      }
      previousAuto = false;
      setRuntimeState(READY);
      setDataSyncRuntime(0, 0);
      if (wateringProcessOneManual()) continue;
      int64_t nowUs = esp_timer_get_time();
      if (lastSyncUs == 0 || nowUs - lastSyncUs >= MANUAL_SYNC_INTERVAL_US) {
        if (otaIsActive() || workerOtaIsActive()) continue;
        LOG("WaterTask manual sync due now_us=%lld last_sync_us=%lld",
            static_cast<long long>(nowUs), static_cast<long long>(lastSyncUs));
        setRuntimeState(SYNCING);
        synchronizeConfiguredWorkers(nowUs);
        lastSyncUs = esp_timer_get_time();
        setRuntimeState(READY);
        LOG("WaterTask manual sync complete last_sync_us=%lld",
            static_cast<long long>(lastSyncUs));
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (!previousAuto) {
      LOG("WaterTask mode=auto");
      lastSyncUs = 0;
      setDataSyncRuntime(0, 0);
      previousAuto = true;
      loggedManualMode = false;
    }

    Settings settings{};
    getSettingsSnapshot(settings);
    int64_t nowUs = esp_timer_get_time();
    int64_t syncIntervalUs =
        static_cast<int64_t>(settings.dataSyncInterval) * 1000000LL;
    if (lastSyncUs != 0 && nowUs - lastSyncUs < syncIntervalUs) {
      if (lastAutoWaitLogUs == 0 || nowUs - lastAutoWaitLogUs >= MANUAL_SYNC_INTERVAL_US) {
        LOG("WaterTask auto sleep-window remaining_us=%lld",
            static_cast<long long>(syncIntervalUs - (nowUs - lastSyncUs)));
        lastAutoWaitLogUs = nowUs;
      }
      handleWorkerSleepRequests(lastSyncUs + syncIntervalUs);
      setDataSyncRuntime(lastSyncUs, lastSyncUs + syncIntervalUs);
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }
    lastAutoWaitLogUs = 0;

    if (otaIsActive() || workerOtaIsActive()) continue;
    setRuntimeState(SYNCING);
    LOG("WaterTask auto sync wake padding_ms=%lu",
        static_cast<unsigned long>(AUTO_WAKE_PADDING_MS));
    vTaskDelay(pdMS_TO_TICKS(AUTO_WAKE_PADDING_MS));
    if (otaIsActive() || workerOtaIsActive()) continue;
    int64_t cycleStartUs = esp_timer_get_time();
    synchronizeConfiguredWorkers(cycleStartUs);
    lastSyncUs = esp_timer_get_time();
    int64_t completedSyncUs = lastSyncUs;

    uint64_t nowUtc = clockGetCurrentEpochSec();
    bool clockOk = clockIsValid() && nowUtc != 0;
    bool windowOpen = clockOk && wateringWindowOpen(settings);
    LOG("WaterTask auto watering gate utc=%lu window_open=%u",
        static_cast<unsigned long>(nowUtc), windowOpen ? 1u : 0u);
    if (clockOk && windowOpen && nowUtc <= UINT32_MAX) {
      if (otaIsActive() || workerOtaIsActive()) continue;
      setRuntimeState(WATERING);
      waterFreshWorkers(cycleStartUs, static_cast<uint32_t>(nowUtc));
    } else {
      LOG("WaterTask auto watering skipped");
    }

    setRuntimeState(SLEEPING);
    lastSyncUs = esp_timer_get_time();
    int64_t wakeAtUs =
        lastSyncUs + static_cast<int64_t>(settings.dataSyncInterval) * 1000000LL;
    setDataSyncRuntime(completedSyncUs, wakeAtUs);
    sendWorkersToSleepAt(wakeAtUs);
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

void setup() {
  Serial.begin(115200);
  bool resumeAutoAfterCrash = autoRecoveryInitialize();
  otaManagerInit();
  workerOtaManagerInit();
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  statusLedBegin();
  vTaskDelay(pdMS_TO_TICKS(3000));
  char btmac[13];
  getBtMacHex(btmac);
  LOG("Bluetooth MAC address: %s", btmac);

  char wifiLast6[7];
  getWifiMacLast6Hex(wifiLast6);
  gMdnsName = String("plant-watering-") + wifiLast6;

  loadSettings();
  clockManagerInit();

  WiFiProvisioner provisioner;
  WiFiProvisioner::Config &config = provisioner.getConfig();
  config.PROJECT_TITLE = "Plant Watering";
  config.AP_NAME = gMdnsName.c_str();
  provisioner.onSuccess([](const char* ssid, const char* password, const char*) {
    saveWifiCred(ssid, password);
  });
  provisioner.getConfig().SHOW_INPUT_FIELD = false;
  provisioner.getConfig().SHOW_RESET_FIELD = false;

  WiFiStartupResult wifiResult = connectToWiFi();
  bool bootOfflineForRecovery =
      resumeAutoAfterCrash &&
      wifiResult == WiFiStartupResult::CONNECTION_FAILED;
  if (wifiResult != WiFiStartupResult::CONNECTED &&
      !bootOfflineForRecovery) {
    provisioner.startProvisioning();
    while (WiFi.status() != WL_CONNECTED) delay(500);
  } else if (bootOfflineForRecovery) {
    LOG("Crash recovery continuing offline; WiFi will retry in background");
  }
  initializeWiFiMaintenance();
  clockRequestNtpSync();

  pumpBegin();
  sensorBegin();
  wateringManagerInit();
  btMainBegin();
  startMdnsIfNeeded();

  webBegin();
  TaskHandle_t webTask = nullptr;
  TaskHandle_t sensorTask = nullptr;
  TaskHandle_t wateringTask = nullptr;
  xTaskCreate(TaskSensor, "sensorTask", 2048, nullptr, 2, &sensorTask);
  xTaskCreate(TaskWatering, "waterTask", 8192, nullptr, 2, &wateringTask);
  if (!otaMarkRunningAppValid()) {
    LOG("OTA running app validation did not complete");
  }
  xTaskCreate(TaskWeb, "webTask", 6144, nullptr, 1, &webTask);
  webSetDiagnosticsTaskHandles(xTaskGetCurrentTaskHandle(), webTask,
                               sensorTask, wateringTask);
  if (resumeAutoAfterCrash) {
    LOG("Restoring auto mode after eligible crash/watchdog reset");
    setAutoEnabled(true);
  }
}

void loop() {
  bool wifiReconnected = serviceWiFiMaintenance();
  if (wifiReconnected && !gMdnsStarted) startMdnsIfNeeded();
  clockManagerLoop();
  otaManagerLoop();
  workerOtaManagerLoop();
  maybeSaveSettings();
  vTaskDelay(pdMS_TO_TICKS(1000));
}
