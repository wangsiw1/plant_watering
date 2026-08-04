#include "BluetoothMain.h"
#include "BluetoothCrypto.h"
#include "WateringManager.h"
#include "config.h"

#include <NimBLEDevice.h>
#include <cstring>
#include <esp_mac.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

using namespace BT_TLV;

namespace {
uint8_t gThisMac[6];
WorkerNode gNodes[MAX_WORKER_COUNT] = {};
int gNodeCount = 0;
SemaphoreHandle_t gNodeMutex = nullptr;
QueueHandle_t gSleepRequestQueue = nullptr;

const char* btTypeName(uint8_t type) {
  switch (type) {
    case TYPE_ACK: return "ACK";
    case TYPE_CMD_PROBE: return "CMD_PROBE";
    case TYPE_CMD_SLEEP: return "CMD_SLEEP";
    case TYPE_CMD_WATER: return "CMD_WATER";
    case TYPE_CMD_OTA_PREPARE: return "CMD_OTA_PREPARE";
    case TYPE_STATUS: return "STATUS";
    case TYPE_CONFIG: return "CONFIG";
    case TYPE_EVENT_WATER_DONE: return "EVENT_WATER_DONE";
    default: return "UNKNOWN";
  }
}

void lockNodes() {
  xSemaphoreTake(gNodeMutex, portMAX_DELAY);
}

void unlockNodes() {
  xSemaphoreGive(gNodeMutex);
}

bool sameMessageId(const BtMessageId& a, const BtMessageId& b) {
  return a.sessionId == b.sessionId && a.sequence == b.sequence;
}

int findNodeLocked(const uint8_t mac[6]) {
  for (int i = 0; i < gNodeCount; ++i) {
    if (memcmp(gNodes[i].mac, mac, 6) == 0) return i;
  }
  return -1;
}

bool acceptMessageId(const uint8_t mac[6], const BtMessageId& id) {
  lockNodes();
  int index = findNodeLocked(mac);
  bool accepted = false;
  if (index >= 0 && !sameMessageId(gNodes[index].lastMessageId, id)) {
    gNodes[index].lastMessageId = id;
    accepted = true;
  }
  unlockNodes();
  char source[13];
  macToHexLower(mac, source);
  LOG("BT RX duplicate check source=%s id=%08lx%08lx:%lu accepted=%u",
      source, static_cast<unsigned long>(id.sessionId >> 32),
      static_cast<unsigned long>(id.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(id.sequence), accepted ? 1u : 0u);
  return accepted;
}

void updateStatus(const uint8_t mac[6], const uint16_t* soils,
                  uint8_t potCount, uint16_t batteryMv, int rssi) {
  lockNodes();
  int index = findNodeLocked(mac);
  if (index >= 0) {
    WorkerNode& node = gNodes[index];
    node.batteryMv = batteryMv;
    node.potCount = min(potCount, static_cast<uint8_t>(MAX_POTS_PER_DEVICE));
    memset(node.soils, 0, sizeof(node.soils));
    memcpy(node.soils, soils, node.potCount * sizeof(uint16_t));
    node.lastSyncUs = esp_timer_get_time();
    node.lastRssi = static_cast<int16_t>(rssi);
    ++node.statusGeneration;
    if (node.statusGeneration == 0) ++node.statusGeneration;
  }
  unlockNodes();
  char source[13];
  macToHexLower(mac, source);
  LOG("BT RX status update source=%s pots=%u batt_mv=%u rssi=%d",
      source, static_cast<unsigned>(potCount),
      static_cast<unsigned>(batteryMv), rssi);
  // Detailed value log for future debugging:
  // for (uint8_t i = 0; i < potCount; ++i) LOG("BT RX status soil[%u]=%u", i, soils[i]);
  ensureWorkerConfigsForMac(mac, potCount);
}

void queueSleepRequestIfNeeded(const uint8_t mac[6]) {
  if (!gSleepRequestQueue) return;
  RuntimeSnapshot runtime{};
  getRuntimeSnapshot(runtime);
  if (!runtime.autoEnabled || runtime.state != SLEEPING) return;

  uint8_t queuedMac[6];
  memcpy(queuedMac, mac, sizeof(queuedMac));
  bool queued = xQueueSendToBack(gSleepRequestQueue, queuedMac, 0) == pdTRUE;
  char source[13];
  macToHexLower(mac, source);
  LOG("BT RX sleep request source=%s queued=%u", source, queued ? 1u : 0u);
}

bool parseStatusBody(const uint8_t* body, size_t bodyLen,
                     uint16_t soils[MAX_POTS_PER_DEVICE],
                     uint8_t& potCount, uint16_t& batteryMv) {
  if (!body || bodyLen < 1 || body[0] != TYPE_STATUS) return false;
  const uint8_t* tlvs = body + 1;
  size_t tlvsLen = bodyLen - 1;
  const uint8_t* soilBytes = nullptr;
  uint8_t soilBytesLen = 0;
  if (!btTlvReadRequiredU16(tlvs, tlvsLen, FIELD_BATT, batteryMv) ||
      !btTlvReadRequiredU8(tlvs, tlvsLen, FIELD_POT_COUNT, potCount) ||
      !btTlvReadRequiredBytes(tlvs, tlvsLen, FIELD_SOIL_LIST,
                              soilBytes, soilBytesLen) ||
      potCount == 0 || potCount > MAX_POTS_PER_DEVICE ||
      soilBytesLen != static_cast<uint8_t>(potCount * 2u)) {
    return false;
  }
  for (size_t i = 0; i < potCount; ++i) {
    soils[i] = (static_cast<uint16_t>(soilBytes[i * 2]) << 8) |
               soilBytes[i * 2 + 1];
  }
  return true;
}

void parseAdvert(const uint8_t* data, size_t len, const NimBLEAdvertisedDevice* adv) {
  uint8_t targetMac[6];
  BtMessageId messageId{};
  const uint8_t* body = nullptr;
  size_t bodyLen = 0;
  uint8_t sourceMac[6];
  extract_src_mac(adv, sourceMac);
  char source[13];
  macToHexLower(sourceMac, source);
  if (!parse_bt_packet_header(data, len, targetMac, messageId, body, bodyLen)) {
    LOG("BT RX drop reason=decode_failed source=%s rssi=%d len=%u",
        source, adv->getRSSI(), static_cast<unsigned>(len));
    return;
  }
  char target[13];
  macToHexLower(targetMac, target);
  uint8_t type = bodyLen > 0 ? body[0] : 0;
  if (memcmp(targetMac, gThisMac, 6) != 0 &&
      memcmp(targetMac, BROADCAST_MAC, 6) != 0) {
    LOG("BT RX drop reason=target_mismatch source=%s target=%s type=%s(0x%02x) id=%08lx%08lx:%lu rssi=%d",
        source, target, btTypeName(type), type,
        static_cast<unsigned long>(messageId.sessionId >> 32),
        static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
        static_cast<unsigned long>(messageId.sequence), adv->getRSSI());
    return;
  }

  LOG("BT RX packet source=%s target=%s type=%s(0x%02x) id=%08lx%08lx:%lu rssi=%d len=%u",
      source, target, btTypeName(type), type,
      static_cast<unsigned long>(messageId.sessionId >> 32),
      static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(messageId.sequence), adv->getRSSI(),
      static_cast<unsigned>(bodyLen));
  if (!isWorkerConfigured(sourceMac)) {
    LOG("BT RX drop reason=unconfigured_worker source=%s target=%s type=%s(0x%02x) id=%08lx%08lx:%lu",
        source, target, btTypeName(type), type,
        static_cast<unsigned long>(messageId.sessionId >> 32),
        static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
        static_cast<unsigned long>(messageId.sequence));
    return;
  }

  if (body[0] == TYPE_ACK && bodyLen == 1) {
    btCommonMarkAck(sourceMac, messageId);
    return;
  }

  if (body[0] == TYPE_STATUS) {
    uint16_t soils[MAX_POTS_PER_DEVICE] = {};
    uint8_t potCount = 0;
    uint16_t batteryMv = 0;
    if (!parseStatusBody(body, bodyLen, soils, potCount, batteryMv)) {
      LOG("BT RX drop reason=invalid_status source=%s id=%08lx%08lx:%lu",
          source, static_cast<unsigned long>(messageId.sessionId >> 32),
          static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
          static_cast<unsigned long>(messageId.sequence));
      return;
    }
    btCommonQueueAck(sourceMac, messageId);
    if (!acceptMessageId(sourceMac, messageId)) {
      LOG("BT RX drop reason=duplicate_status source=%s id=%08lx%08lx:%lu",
          source, static_cast<unsigned long>(messageId.sessionId >> 32),
          static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
          static_cast<unsigned long>(messageId.sequence));
      return;
    }
    updateStatus(sourceMac, soils, potCount, batteryMv, adv->getRSSI());
    queueSleepRequestIfNeeded(sourceMac);
    return;
  }

  if (body[0] == TYPE_EVENT_WATER_DONE) {
    uint16_t potMask = 0;
    if (!btTlvReadRequiredU16(body + 1, bodyLen - 1,
                              FIELD_POT_MASK, potMask)) {
      LOG("BT RX drop reason=bad_water_done_tlv source=%s id=%08lx%08lx:%lu",
          source, static_cast<unsigned long>(messageId.sessionId >> 32),
          static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
          static_cast<unsigned long>(messageId.sequence));
      return;
    }
    btCommonQueueAck(sourceMac, messageId);
    if (!acceptMessageId(sourceMac, messageId)) {
      LOG("BT RX drop reason=duplicate_water_done source=%s id=%08lx%08lx:%lu",
          source, static_cast<unsigned long>(messageId.sessionId >> 32),
          static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
          static_cast<unsigned long>(messageId.sequence));
      return;
    }
    LOG("BT RX water done source=%s mask=0x%04x id=%08lx%08lx:%lu",
        source, static_cast<unsigned>(potMask),
        static_cast<unsigned long>(messageId.sessionId >> 32),
        static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
        static_cast<unsigned long>(messageId.sequence));
    // Detailed value log for future debugging:
    // LOG("BT RX water done pot_mask=0x%04x", potMask);
    wateringNotifyCompleted(sourceMac, potMask);
    return;
  }

  LOG("BT RX drop reason=unknown_type source=%s target=%s type=%s(0x%02x) id=%08lx%08lx:%lu",
      source, target, btTypeName(type), type,
      static_cast<unsigned long>(messageId.sessionId >> 32),
      static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(messageId.sequence));
}
}

void btMainBegin() {
  if (!gNodeMutex) gNodeMutex = xSemaphoreCreateMutex();
  if (!gSleepRequestQueue) {
    gSleepRequestQueue = xQueueCreate(MAX_WORKER_COUNT, 6);
  }
  int workerCount = getWorkerConfigCount();
  for (int i = 0; i < workerCount; ++i) {
    WorkerConfig worker{};
    if (getWorkerConfigAt(i, worker)) btMainEnsureNodeExists(worker.mac);
  }

  esp_read_mac(gThisMac, ESP_MAC_BT);
  char self[13];
  macToHexLower(gThisMac, self);
  LOG("BT main init start mac=%s workers=%d", self, workerCount);
  NimBLEDevice::init("plant-main");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  if (!btCryptoInit()) {
    LOG("BT main init failed reason=crypto");
    return;
  }
  if (!btCommonInitSender()) {
    LOG("BT main init failed reason=sender");
    return;
  }

  btCommonSetAdvertHandler(parseAdvert);
  btCommonInstallScanCallbacks();
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setInterval(50);
  scan->setWindow(50);
  scan->setMaxResults(0);
  scan->start(0);
  LOG("BT main scan started interval=50 window=50");
}

BtSendResult btMainSendCommand(const uint8_t targetMac[6],
                               const uint8_t* payload, size_t len,
                               int retries, uint32_t timeoutMs,
                               BtBeforeTransmitCb beforeTransmit,
                               void* context) {
  return btCommonSendCommand(targetMac, payload, len, retries, timeoutMs,
                             beforeTransmit, context);
}

int btMainNodeCount() {
  if (!gNodeMutex) return 0;
  lockNodes();
  int count = gNodeCount;
  unlockNodes();
  return count;
}

bool btMainGetNodeAt(int index, WorkerNode& out) {
  if (!gNodeMutex) return false;
  lockNodes();
  bool valid = index >= 0 && index < gNodeCount;
  if (valid) out = gNodes[index];
  unlockNodes();
  return valid;
}

bool btMainGetNodeByMac(const uint8_t mac[6], WorkerNode& out) {
  if (!gNodeMutex || !mac) return false;
  lockNodes();
  int index = findNodeLocked(mac);
  if (index >= 0) out = gNodes[index];
  unlockNodes();
  return index >= 0;
}

bool btMainWaitForStatus(const uint8_t mac[6], uint32_t previousGeneration,
                         uint32_t timeoutMs, WorkerNode& out) {
  int64_t deadline =
      esp_timer_get_time() + static_cast<int64_t>(timeoutMs) * 1000;
  while (esp_timer_get_time() < deadline) {
    if (btMainGetNodeByMac(mac, out) &&
        out.statusGeneration != previousGeneration) {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  return false;
}

void btMainMarkNodeWatered(const uint8_t mac[6], uint16_t potMask) {
  if (!gNodeMutex) return;
  int64_t nowUs = esp_timer_get_time();
  lockNodes();
  int index = findNodeLocked(mac);
  if (index >= 0) {
    for (int pot = 0; pot < MAX_POTS_PER_DEVICE; ++pot) {
      if (potMask & (1u << pot)) gNodes[index].lastWaterUs[pot] = nowUs;
    }
  }
  unlockNodes();
}

bool btMainTakeSleepRequest(uint8_t mac[6]) {
  if (!gSleepRequestQueue || !mac) return false;
  return xQueueReceive(gSleepRequestQueue, mac, 0) == pdTRUE;
}

void btMainEnsureNodeExists(const uint8_t mac[6]) {
  if (!mac) return;
  if (!gNodeMutex) gNodeMutex = xSemaphoreCreateMutex();
  lockNodes();
  if (findNodeLocked(mac) < 0 && gNodeCount < MAX_WORKER_COUNT) {
    WorkerNode& node = gNodes[gNodeCount++];
    memset(&node, 0, sizeof(node));
    memcpy(node.mac, mac, 6);
    node.lastRssi = 0x7FFF;
  }
  unlockNodes();
}

void btMainRemoveNodeByMac(const uint8_t mac[6]) {
  if (!gNodeMutex || !mac) return;
  lockNodes();
  int index = findNodeLocked(mac);
  if (index >= 0) {
    for (int i = index; i < gNodeCount - 1; ++i) gNodes[i] = gNodes[i + 1];
    --gNodeCount;
  }
  unlockNodes();
}
