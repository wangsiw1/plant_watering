#include "BluetoothCommon.h"
#include "BluetoothCrypto.h"
#include "Utility.h"

#include <cstring>
#include <NimBLEExtAdvertising.h>
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#if !defined(CONFIG_BT_NIMBLE_EXT_ADV) || !CONFIG_BT_NIMBLE_EXT_ADV
#error "Bluetooth TLV protocol requires CONFIG_BT_NIMBLE_EXT_ADV=1"
#endif

const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

namespace {
constexpr size_t PACKET_HEADER_SIZE = 18;
constexpr size_t GCM_TAG_SIZE = 16;
constexpr size_t MAX_PACKET_SIZE = 2 + PACKET_HEADER_SIZE + BT_TLV::MAX_BODY_SIZE + GCM_TAG_SIZE;
constexpr UBaseType_t TX_QUEUE_LEN = 6;
constexpr uint32_t ADV_DURATION_MS = 500;
constexpr uint16_t ADV_INTERVAL_MIN_UNITS = 48;  // 30 ms at 0.625 ms/unit
constexpr uint16_t ADV_INTERVAL_MAX_UNITS = 80;  // 50 ms at 0.625 ms/unit

enum class TxKind : uint8_t { COMMAND, ACK };

struct TxJob {
  TxKind kind;
  uint8_t mac[6];
  BtMessageId messageId;
  uint8_t payload[BT_TLV::MAX_BODY_SIZE];
  uint8_t payloadLen;
  uint8_t retries;
  uint32_t ackTimeoutMs;
  BtBeforeTransmitCb beforeTransmit;
  void* context;
  TaskHandle_t waiter;
  BtSendResult* result;
};

struct PendingAck {
  bool active;
  bool acked;
  uint8_t targetMac[6];
  BtMessageId messageId;
  uint8_t ackMac[6];
};

QueueHandle_t gTxQueue = nullptr;
TaskHandle_t gSenderTask = nullptr;
BT_TLV::AdvertHandler gAdvertHandler = nullptr;
uint64_t gSessionId = 0;
uint32_t gNextSequence = 1;
portMUX_TYPE gIdMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE gPendingMux = portMUX_INITIALIZER_UNLOCKED;
PendingAck gPending = {};

const char* btTypeName(uint8_t type) {
  switch (type) {
    case BT_TLV::TYPE_ACK: return "ACK";
    case BT_TLV::TYPE_CMD_PROBE: return "CMD_PROBE";
    case BT_TLV::TYPE_CMD_SLEEP: return "CMD_SLEEP";
    case BT_TLV::TYPE_CMD_WATER: return "CMD_WATER";
    case BT_TLV::TYPE_CMD_OTA_PREPARE: return "CMD_OTA_PREPARE";
    case BT_TLV::TYPE_STATUS: return "STATUS";
    case BT_TLV::TYPE_CONFIG: return "CONFIG";
    case BT_TLV::TYPE_EVENT_WATER_DONE: return "EVENT_WATER_DONE";
    default: return "UNKNOWN";
  }
}

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

bool sameMessageId(const BtMessageId& a, const BtMessageId& b) {
  return a.sessionId == b.sessionId && a.sequence == b.sequence;
}

bool isBroadcast(const uint8_t mac[6]) {
  return memcmp(mac, BROADCAST_MAC, 6) == 0;
}

BtMessageId allocateMessageId() {
  portENTER_CRITICAL(&gIdMux);
  BtMessageId id{gSessionId, gNextSequence++};
  if (gNextSequence == 0) {
    ++gSessionId;
    if (gSessionId == 0) gSessionId = 1;
    gNextSequence = 1;
  }
  portEXIT_CRITICAL(&gIdMux);
  return id;
}

void completeJob(const TxJob& job, const BtSendResult& result) {
  if (!job.waiter || !job.result) return;
  *job.result = result;
  xTaskNotifyGive(job.waiter);
}

bool broadcastPacket(const uint8_t* packet, size_t len, uint32_t durationMs,
                     BtBeforeTransmitCb beforeTransmit, void* context,
                     int64_t& startedUs) {
  NimBLEExtAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->stop(0);
  advertising->removeInstance(0);

  NimBLEExtAdvertisement advert;
  advert.setLegacyAdvertising(false);
  advert.setConnectable(false);
  advert.setScannable(false);
  advert.setMinInterval(ADV_INTERVAL_MIN_UNITS);
  advert.setMaxInterval(ADV_INTERVAL_MAX_UNITS);
  advert.setManufacturerData(packet, len);

  bool configured = false;
  for (int attempt = 0; attempt < 3 && !configured; ++attempt) {
    configured = advertising->setInstanceData(0, advert);
    if (!configured) vTaskDelay(pdMS_TO_TICKS(10));
  }
  if (!configured || !advertising->start(0, durationMs)) {
    advertising->stop(0);
    advertising->removeInstance(0);
    return false;
  }

  startedUs = esp_timer_get_time();
  if (beforeTransmit) beforeTransmit(context);

  const int64_t deadlineUs = startedUs + (static_cast<int64_t>(durationMs) + 500) * 1000;
  while (advertising->isActive(0) && esp_timer_get_time() < deadlineUs) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  advertising->stop(0);
  advertising->removeInstance(0);
  return true;
}

BtSendResult emptyResult(BtSendStatus status) {
  BtSendResult result{};
  result.status = status;
  return result;
}

void processAckJob(const TxJob& job) {
  BtSendResult result = emptyResult(BtSendStatus::TRANSMIT_FAILED);
  result.messageId = job.messageId;
  char target[13];
  macToHexLower(job.mac, target);
  LOG("BT TX ack start target=%s id=%08lx%08lx:%lu",
      target, static_cast<unsigned long>(job.messageId.sessionId >> 32),
      static_cast<unsigned long>(job.messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(job.messageId.sequence));

  uint8_t body[1] = {BT_TLV::TYPE_ACK};
  uint8_t packet[MAX_PACKET_SIZE];
  size_t packetLen = make_bt_packet(job.mac, job.messageId, body, sizeof(body), packet);
  if (packetLen != 0) {
    int64_t startedUs = 0;
    if (broadcastPacket(packet, packetLen, ADV_DURATION_MS, nullptr, nullptr, startedUs)) {
      result.status = BtSendStatus::SENT;
      result.transmissionStarted = true;
      result.startedUs = startedUs;
    }
  }
  LOG("BT TX ack result target=%s id=%08lx%08lx:%lu status=%s started=%u",
      target, static_cast<unsigned long>(job.messageId.sessionId >> 32),
      static_cast<unsigned long>(job.messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(job.messageId.sequence),
      btSendStatusName(result.status), result.transmissionStarted ? 1u : 0u);
  completeJob(job, result);
}

void processUrgentAcks() {
  TxJob queued{};
  while (xQueuePeek(gTxQueue, &queued, 0) == pdTRUE && queued.kind == TxKind::ACK) {
    if (xQueueReceive(gTxQueue, &queued, 0) == pdTRUE) processAckJob(queued);
  }
}

void processCommandJob(const TxJob& job) {
  BtSendResult result = emptyResult(BtSendStatus::TRANSMIT_FAILED);
  result.messageId = job.messageId;
  char target[13];
  macToHexLower(job.mac, target);
  const uint8_t type = job.payloadLen > 0 ? job.payload[0] : 0;

  uint8_t packet[MAX_PACKET_SIZE];
  size_t packetLen =
      make_bt_packet(job.mac, job.messageId, job.payload, job.payloadLen, packet);
  if (packetLen == 0) {
    LOG("BT TX encode failed target=%s type=%s(0x%02x) id=%08lx%08lx:%lu",
        target, btTypeName(type), type,
        static_cast<unsigned long>(job.messageId.sessionId >> 32),
        static_cast<unsigned long>(job.messageId.sessionId & 0xFFFFFFFFULL),
        static_cast<unsigned long>(job.messageId.sequence));
    completeJob(job, result);
    return;
  }

  portENTER_CRITICAL(&gPendingMux);
  gPending.active = true;
  gPending.acked = false;
  memcpy(gPending.targetMac, job.mac, 6);
  gPending.messageId = job.messageId;
  memset(gPending.ackMac, 0, sizeof(gPending.ackMac));
  portEXIT_CRITICAL(&gPendingMux);

  bool callbackUsed = false;
  for (int attempt = 0; attempt <= job.retries; ++attempt) {
    int64_t startedUs = 0;
    BtBeforeTransmitCb callback = callbackUsed ? nullptr : job.beforeTransmit;
    LOG("BT TX command attempt=%d/%u target=%s type=%s(0x%02x) id=%08lx%08lx:%lu len=%u",
        attempt + 1, static_cast<unsigned>(job.retries) + 1u, target,
        btTypeName(type), type,
        static_cast<unsigned long>(job.messageId.sessionId >> 32),
        static_cast<unsigned long>(job.messageId.sessionId & 0xFFFFFFFFULL),
        static_cast<unsigned long>(job.messageId.sequence),
        static_cast<unsigned>(job.payloadLen));
    if (!broadcastPacket(packet, packetLen, ADV_DURATION_MS, callback, job.context, startedUs)) {
      LOG("BT TX command broadcast failed target=%s type=%s(0x%02x) id=%08lx%08lx:%lu",
          target, btTypeName(type), type,
          static_cast<unsigned long>(job.messageId.sessionId >> 32),
          static_cast<unsigned long>(job.messageId.sessionId & 0xFFFFFFFFULL),
          static_cast<unsigned long>(job.messageId.sequence));
      continue;
    }
    callbackUsed = true;
    if (!result.transmissionStarted) {
      result.transmissionStarted = true;
      result.startedUs = startedUs;
      result.status = BtSendStatus::SENT;
    }

    const int64_t deadlineUs =
        esp_timer_get_time() + static_cast<int64_t>(job.ackTimeoutMs) * 1000;
    while (esp_timer_get_time() < deadlineUs) {
      processUrgentAcks();
      portENTER_CRITICAL(&gPendingMux);
      bool acked = gPending.acked;
      if (acked) memcpy(result.ackMac, gPending.ackMac, 6);
      portEXIT_CRITICAL(&gPendingMux);
      if (acked) {
        result.status = BtSendStatus::ACKED;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (result.status == BtSendStatus::ACKED) break;
  }

  portENTER_CRITICAL(&gPendingMux);
  gPending.active = false;
  portEXIT_CRITICAL(&gPendingMux);
  char ack[13];
  macToHexLower(result.ackMac, ack);
  LOG("BT TX command result target=%s type=%s(0x%02x) id=%08lx%08lx:%lu status=%s started=%u ack=%s",
      target, btTypeName(type), type,
      static_cast<unsigned long>(job.messageId.sessionId >> 32),
      static_cast<unsigned long>(job.messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(job.messageId.sequence),
      btSendStatusName(result.status), result.transmissionStarted ? 1u : 0u, ack);
  completeJob(job, result);
}

void senderTask(void*) {
  TxJob job;
  for (;;) {
    if (xQueueReceive(gTxQueue, &job, portMAX_DELAY) != pdTRUE) continue;
    if (job.kind == TxKind::ACK) processAckJob(job);
    else processCommandJob(job);
  }
}

bool findFirstField(const uint8_t* tlvs, size_t tlvsLen, uint8_t type,
                    BT_TLV::TlvFieldView& match, bool& found) {
  found = false;
  size_t offset = 0;
  BT_TLV::TlvFieldView field{};
  while (offset < tlvsLen) {
    if (!BT_TLV::btTlvNext(tlvs, tlvsLen, offset, field)) return false;
    if (!found && field.type == type) {
      match = field;
      found = true;
    }
  }
  return true;
}
}

namespace BT_TLV {
bool btCommonInitSender() {
  if (!btCryptoInit()) return false;
  if (gSessionId == 0) {
    gSessionId = (static_cast<uint64_t>(esp_random()) << 32) | esp_random();
    if (gSessionId == 0) gSessionId = 1;
  }
  if (!gTxQueue) gTxQueue = xQueueCreate(TX_QUEUE_LEN, sizeof(TxJob));
  if (!gTxQueue) return false;
  if (!gSenderTask) {
    if (xTaskCreate(senderTask, "btSender", 4096, nullptr, 3, &gSenderTask) != pdPASS) {
      gSenderTask = nullptr;
      return false;
    }
  }
  return true;
}

bool btCommonGetSenderStackHighWaterMark(UBaseType_t& outBytes) {
  if (!gSenderTask) return false;
  outBytes = uxTaskGetStackHighWaterMark(gSenderTask);
  return true;
}

BtSendResult btCommonSendCommand(const uint8_t targetMac[6],
                                 const uint8_t* payload, size_t len,
                                 int retries, uint32_t ackTimeoutMs,
                                 BtBeforeTransmitCb beforeTransmit,
                                 void* context) {
  if (!targetMac || !payload || len == 0 || len > MAX_BODY_SIZE || !gTxQueue) {
    LOG("BT TX command invalid target=%p payload=%p len=%u queue=%p",
        targetMac, payload, static_cast<unsigned>(len), gTxQueue);
    return emptyResult(BtSendStatus::INVALID);
  }

  BtSendResult result = emptyResult(BtSendStatus::QUEUE_FULL);
  TxJob job;
  job.kind = TxKind::COMMAND;
  memcpy(job.mac, targetMac, 6);
  job.messageId = allocateMessageId();
  memcpy(job.payload, payload, len);
  job.payloadLen = static_cast<uint8_t>(len);
  job.retries = static_cast<uint8_t>(constrain(retries, 0, 10));
  job.ackTimeoutMs = ackTimeoutMs;
  job.beforeTransmit = beforeTransmit;
  job.context = context;
  job.waiter = xTaskGetCurrentTaskHandle();
  job.result = &result;
  result.messageId = job.messageId;
  char target[13];
  macToHexLower(targetMac, target);
  LOG("BT TX command queued target=%s type=%s(0x%02x) id=%08lx%08lx:%lu len=%u retries=%d timeout_ms=%lu",
      target, btTypeName(payload[0]), payload[0],
      static_cast<unsigned long>(job.messageId.sessionId >> 32),
      static_cast<unsigned long>(job.messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(job.messageId.sequence),
      static_cast<unsigned>(len), retries, static_cast<unsigned long>(ackTimeoutMs));

  if (xQueueSendToBack(gTxQueue, &job, 0) != pdTRUE) {
    LOG("BT TX command queue full target=%s type=%s(0x%02x) id=%08lx%08lx:%lu",
        target, btTypeName(payload[0]), payload[0],
        static_cast<unsigned long>(job.messageId.sessionId >> 32),
        static_cast<unsigned long>(job.messageId.sessionId & 0xFFFFFFFFULL),
        static_cast<unsigned long>(job.messageId.sequence));
    return result;
  }
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  LOG("BT TX command completed target=%s type=%s(0x%02x) id=%08lx%08lx:%lu status=%s",
      target, btTypeName(payload[0]), payload[0],
      static_cast<unsigned long>(result.messageId.sessionId >> 32),
      static_cast<unsigned long>(result.messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(result.messageId.sequence),
      btSendStatusName(result.status));
  return result;
}

bool btCommonQueueAck(const uint8_t targetMac[6], const BtMessageId& messageId) {
  if (!targetMac || !gTxQueue) {
    LOG("BT TX ack invalid target=%p queue=%p", targetMac, gTxQueue);
    return false;
  }
  TxJob job;
  job.kind = TxKind::ACK;
  memcpy(job.mac, targetMac, 6);
  job.messageId = messageId;
  job.waiter = nullptr;
  job.result = nullptr;
  char target[13];
  macToHexLower(targetMac, target);
  bool queued = xQueueSendToFront(gTxQueue, &job, 0) == pdTRUE;
  LOG("BT TX ack queue target=%s id=%08lx%08lx:%lu queued=%u",
      target, static_cast<unsigned long>(messageId.sessionId >> 32),
      static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(messageId.sequence), queued ? 1u : 0u);
  return queued;
}

BtSendResult btCommonSendAckAndWait(const uint8_t targetMac[6],
                                   const BtMessageId& messageId) {
  if (!targetMac || !gTxQueue) {
    LOG("BT TX ack invalid target=%p queue=%p", targetMac, gTxQueue);
    return emptyResult(BtSendStatus::INVALID);
  }
  BtSendResult result = emptyResult(BtSendStatus::QUEUE_FULL);
  TxJob job;
  job.kind = TxKind::ACK;
  memcpy(job.mac, targetMac, 6);
  job.messageId = messageId;
  job.waiter = xTaskGetCurrentTaskHandle();
  job.result = &result;
  result.messageId = messageId;
  char target[13];
  macToHexLower(targetMac, target);
  if (xQueueSendToFront(gTxQueue, &job, 0) != pdTRUE) {
    LOG("BT TX ack queue full target=%s id=%08lx%08lx:%lu",
        target, static_cast<unsigned long>(messageId.sessionId >> 32),
        static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
        static_cast<unsigned long>(messageId.sequence));
    return result;
  }
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  LOG("BT TX ack completed target=%s id=%08lx%08lx:%lu status=%s",
      target, static_cast<unsigned long>(messageId.sessionId >> 32),
      static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(messageId.sequence),
      btSendStatusName(result.status));
  return result;
}

void btCommonMarkAck(const uint8_t sourceMac[6], const BtMessageId& messageId) {
  if (!sourceMac) return;
  bool matched = false;
  portENTER_CRITICAL(&gPendingMux);
  if (gPending.active && sameMessageId(gPending.messageId, messageId) &&
      (isBroadcast(gPending.targetMac) ||
       memcmp(gPending.targetMac, sourceMac, 6) == 0)) {
    gPending.acked = true;
    memcpy(gPending.ackMac, sourceMac, 6);
    matched = true;
  }
  portEXIT_CRITICAL(&gPendingMux);
  char source[13];
  macToHexLower(sourceMac, source);
  LOG("BT RX ack source=%s id=%08lx%08lx:%lu matched=%u",
      source, static_cast<unsigned long>(messageId.sessionId >> 32),
      static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(messageId.sequence), matched ? 1u : 0u);
}

void btCommonSetAdvertHandler(AdvertHandler handler) {
  gAdvertHandler = handler;
}

void btBodyBegin(BtBodyBuilder& body, uint8_t msgType) {
  body.len = 1;
  body.data[0] = msgType;
}

bool btTlvAppend(BtBodyBuilder& body, uint8_t type, const uint8_t* value, uint8_t len) {
  if (body.len + 2u + len > MAX_BODY_SIZE) return false;
  body.data[body.len++] = type;
  body.data[body.len++] = len;
  if (len) {
    memcpy(body.data + body.len, value, len);
    body.len += len;
  }
  return true;
}

bool btTlvAppendU8(BtBodyBuilder& body, uint8_t type, uint8_t value) {
  return btTlvAppend(body, type, &value, 1);
}

bool btTlvAppendU16(BtBodyBuilder& body, uint8_t type, uint16_t value) {
  uint8_t raw[2] = {static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
  return btTlvAppend(body, type, raw, sizeof(raw));
}

bool btTlvAppendU32(BtBodyBuilder& body, uint8_t type, uint32_t value) {
  uint8_t raw[4] = {
    static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
    static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)
  };
  return btTlvAppend(body, type, raw, sizeof(raw));
}

bool btTlvNext(const uint8_t* tlvs, size_t tlvsLen, size_t& offset,
               TlvFieldView& field) {
  if (!tlvs || offset + 2 > tlvsLen) return false;
  uint8_t len = tlvs[offset + 1];
  if (offset + 2u + len > tlvsLen) return false;
  field.type = tlvs[offset];
  field.len = len;
  field.value = tlvs + offset + 2;
  offset += 2u + len;
  return true;
}

bool btTlvReadRequiredU8(const uint8_t* tlvs, size_t len, uint8_t type, uint8_t& out) {
  TlvFieldView field{};
  bool found = false;
  if (!findFirstField(tlvs, len, type, field, found) || !found || field.len != 1) return false;
  out = field.value[0];
  return true;
}

bool btTlvReadRequiredU16(const uint8_t* tlvs, size_t len, uint8_t type, uint16_t& out) {
  TlvFieldView field{};
  bool found = false;
  if (!findFirstField(tlvs, len, type, field, found) || !found || field.len != 2) return false;
  out = (static_cast<uint16_t>(field.value[0]) << 8) | field.value[1];
  return true;
}

bool btTlvReadRequiredU32(const uint8_t* tlvs, size_t len, uint8_t type, uint32_t& out) {
  TlvFieldView field{};
  bool found = false;
  if (!findFirstField(tlvs, len, type, field, found) || !found || field.len != 4) return false;
  out = (static_cast<uint32_t>(field.value[0]) << 24) |
        (static_cast<uint32_t>(field.value[1]) << 16) |
        (static_cast<uint32_t>(field.value[2]) << 8) | field.value[3];
  return true;
}

bool btTlvReadRequiredBytes(const uint8_t* tlvs, size_t len, uint8_t type,
                            const uint8_t*& out, uint8_t& outLen) {
  TlvFieldView field{};
  bool found = false;
  if (!findFirstField(tlvs, len, type, field, found) || !found) return false;
  out = field.value;
  outLen = field.len;
  return true;
}

bool btTlvReadRequiredU16Array(const uint8_t* tlvs, size_t len, uint8_t type,
                               uint16_t* out, size_t maxCount, size_t& outCount) {
  const uint8_t* bytes = nullptr;
  uint8_t bytesLen = 0;
  outCount = 0;
  if (!btTlvReadRequiredBytes(tlvs, len, type, bytes, bytesLen) || bytesLen % 2 != 0) {
    return false;
  }
  outCount = bytesLen / 2;
  if (outCount > maxCount) return false;
  for (size_t i = 0; i < outCount; ++i) {
    out[i] = (static_cast<uint16_t>(bytes[i * 2]) << 8) | bytes[i * 2 + 1];
  }
  return true;
}

class CommonScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* adv) override {
    if (!gAdvertHandler) return;
    const std::vector<uint8_t>& bytes = adv->getPayload();
    size_t offset = 0;
    while (offset < bytes.size()) {
      uint8_t adLen = bytes[offset];
      if (adLen == 0 || offset + adLen >= bytes.size()) break;
      if (bytes[offset + 1] == 0xFF && adLen >= 3 &&
          bytes[offset + 2] == 0xFF && bytes[offset + 3] == 0xFF) {
        gAdvertHandler(bytes.data() + offset + 4, adLen - 3, adv);
        break;
      }
      offset += adLen + 1;
    }
  }
};

void btCommonInstallScanCallbacks() {
  NimBLEDevice::getScan()->setScanCallbacks(new CommonScanCallbacks(), false);
}
}

void extract_src_mac(const NimBLEAdvertisedDevice* adv, uint8_t outMac[6]) {
  NimBLEAddress address = adv->getAddress();
  address.reverseByteOrder();
  memcpy(outMac, address.getVal(), 6);
}

size_t make_bt_packet(const uint8_t targetMac[6], const BtMessageId& messageId,
                      const uint8_t* payload, size_t payloadLen, uint8_t* out) {
  if (!targetMac || !payload || !out || payloadLen == 0 ||
      payloadLen > BT_TLV::MAX_BODY_SIZE) {
    return 0;
  }
  size_t pos = 0;
  out[pos++] = 0xFF;
  out[pos++] = 0xFF;
  memcpy(out + pos, targetMac, 6);
  pos += 6;
  for (int i = 0; i < 8; ++i) {
    out[pos++] = static_cast<uint8_t>(messageId.sessionId >> (56 - i * 8));
  }
  out[pos++] = static_cast<uint8_t>(messageId.sequence >> 24);
  out[pos++] = static_cast<uint8_t>(messageId.sequence >> 16);
  out[pos++] = static_cast<uint8_t>(messageId.sequence >> 8);
  out[pos++] = static_cast<uint8_t>(messageId.sequence);

#if USE_BT_CRYPTO
  size_t encryptedLen = 0;
  if (!btEncryptPayload(payload, payloadLen, targetMac, messageId,
                        out + pos, encryptedLen)) {
    return 0;
  }
  pos += encryptedLen;
#else
  memcpy(out + pos, payload, payloadLen);
  pos += payloadLen;
#endif
  return pos;
}

bool parse_bt_packet_header(const uint8_t* data, size_t len, uint8_t targetMac[6],
                            BtMessageId& messageId, const uint8_t*& payload,
                            size_t& payloadLen) {
  if (!data || len < PACKET_HEADER_SIZE) return false;
  memcpy(targetMac, data, 6);
  messageId.sessionId = 0;
  for (int i = 0; i < 8; ++i) {
    messageId.sessionId = (messageId.sessionId << 8) | data[6 + i];
  }
  messageId.sequence = (static_cast<uint32_t>(data[14]) << 24) |
                       (static_cast<uint32_t>(data[15]) << 16) |
                       (static_cast<uint32_t>(data[16]) << 8) | data[17];

  const uint8_t* encoded = data + PACKET_HEADER_SIZE;
  size_t encodedLen = len - PACKET_HEADER_SIZE;
#if USE_BT_CRYPTO
  static uint8_t decrypted[BT_TLV::MAX_BODY_SIZE];
  size_t decryptedLen = 0;
  if (!btDecryptPayload(encoded, encodedLen, targetMac, messageId,
                        decrypted, decryptedLen)) {
    return false;
  }
  payload = decrypted;
  payloadLen = decryptedLen;
#else
  payload = encoded;
  payloadLen = encodedLen;
#endif
  return payloadLen > 0;
}
