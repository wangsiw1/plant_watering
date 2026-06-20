#include "BluetoothWorker.h"
#include "BluetoothCommon.h"
#include "BluetoothCrypto.h"
#include "Sensor.h"
#include "Utility.h"
#include "Valve.h"
#include "HardwareConfig.h"

#include <NimBLEDevice.h>
#include <cstring>
#include <esp_mac.h>
#include <esp_sleep.h>
#include <inttypes.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

using namespace BT_TLV;

namespace {
constexpr UBaseType_t COMMAND_QUEUE_LEN = 4;
constexpr uint32_t MAX_SLEEP_SECONDS = 2419200;
constexpr uint16_t MAX_WATER_SECONDS = 60;

struct WorkerCommand {
  uint8_t sourceMac[6];
  BtMessageId messageId;
  uint8_t body[BT_TLV::MAX_BODY_SIZE];
  uint8_t bodyLen;
};

uint8_t gThisMac[6];
RTC_DATA_ATTR uint8_t gMainMac[6] = {};
RTC_DATA_ATTR BtMessageId gLastCommandId = {};
int64_t gLastCommunicationUs = 0;
QueueHandle_t gCommandQueue = nullptr;
portMUX_TYPE gWorkerStateMux = portMUX_INITIALIZER_UNLOCKED;

const char* btTypeName(uint8_t type) {
  switch (type) {
    case TYPE_ACK: return "ACK";
    case TYPE_CMD_PROBE: return "CMD_PROBE";
    case TYPE_CMD_SLEEP: return "CMD_SLEEP";
    case TYPE_CMD_WATER: return "CMD_WATER";
    case TYPE_STATUS: return "STATUS";
    case TYPE_CONFIG: return "CONFIG";
    case TYPE_EVENT_WATER_DONE: return "EVENT_WATER_DONE";
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

bool messageIdEqual(const BtMessageId& a, const BtMessageId& b) {
  return a.sessionId == b.sessionId && a.sequence == b.sequence;
}

bool copyMainMac(uint8_t out[6]) {
  portENTER_CRITICAL(&gWorkerStateMux);
  memcpy(out, gMainMac, 6);
  portEXIT_CRITICAL(&gWorkerStateMux);
  for (int i = 0; i < 6; ++i) {
    if (out[i] != 0) return true;
  }
  return false;
}

void setMainMac(const uint8_t mac[6]) {
  portENTER_CRITICAL(&gWorkerStateMux);
  memcpy(gMainMac, mac, 6);
  gLastCommunicationUs = esp_timer_get_time();
  portEXIT_CRITICAL(&gWorkerStateMux);
  char mainMac[13];
  macToHexLower(mac, mainMac);
  LOG("BT worker paired main=%s", mainMac);
}

void markCommunication() {
  portENTER_CRITICAL(&gWorkerStateMux);
  gLastCommunicationUs = esp_timer_get_time();
  portEXIT_CRITICAL(&gWorkerStateMux);
}

bool parseWaterCommand(const uint8_t* body, size_t bodyLen,
                       uint16_t& potMask, uint16_t durations[WORKER_POT_COUNT],
                       size_t& durationCount) {
  if (!body || bodyLen < 1 || body[0] != TYPE_CMD_WATER) return false;
  const uint8_t* tlvs = body + 1;
  size_t tlvsLen = bodyLen - 1;
  if (!btTlvReadRequiredU16(tlvs, tlvsLen, FIELD_POT_MASK, potMask)) return false;
  const uint16_t allowedMask =
      WORKER_POT_COUNT >= 16 ? 0xFFFFu :
      static_cast<uint16_t>((1u << WORKER_POT_COUNT) - 1u);
  if (potMask == 0 || (potMask & ~allowedMask) != 0) return false;
  if (!btTlvReadRequiredU16Array(tlvs, tlvsLen, FIELD_DURATION_LIST,
                                 durations, WORKER_POT_COUNT, durationCount)) {
    return false;
  }
  if (durationCount != static_cast<size_t>(__builtin_popcount(potMask))) return false;
  for (size_t i = 0; i < durationCount; ++i) {
    if (durations[i] == 0 || durations[i] > MAX_WATER_SECONDS) return false;
  }
  return true;
}

bool validateCommand(const uint8_t* body, size_t bodyLen) {
  if (!body || bodyLen == 0) return false;
  if (body[0] == TYPE_CMD_PROBE) return bodyLen == 1;
  if (body[0] == TYPE_CMD_SLEEP) {
    uint32_t seconds = 0;
    return btTlvReadRequiredU32(body + 1, bodyLen - 1,
                                FIELD_SLEEP_SEC, seconds) &&
           seconds > 0 && seconds <= MAX_SLEEP_SECONDS;
  }
  if (body[0] == TYPE_CMD_WATER) {
    uint16_t mask = 0;
    uint16_t durations[WORKER_POT_COUNT] = {};
    size_t count = 0;
    return parseWaterCommand(body, bodyLen, mask, durations, count);
  }
  return false;
}

bool isDuplicate(const BtMessageId& id) {
  portENTER_CRITICAL(&gWorkerStateMux);
  bool duplicate = messageIdEqual(gLastCommandId, id);
  if (!duplicate) gLastCommandId = id;
  portEXIT_CRITICAL(&gWorkerStateMux);
  LOG("BT RX duplicate check id=%08lx%08lx:%lu duplicate=%u",
      static_cast<unsigned long>(id.sessionId >> 32),
      static_cast<unsigned long>(id.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(id.sequence), duplicate ? 1u : 0u);
  return duplicate;
}

void executeWaterCommand(const WorkerCommand& command) {
  uint16_t potMask = 0;
  uint16_t durations[WORKER_POT_COUNT] = {};
  size_t durationCount = 0;
  if (!parseWaterCommand(command.body, command.bodyLen,
                         potMask, durations, durationCount)) {
    LOG("BT worker water parse failed id=%08lx%08lx:%lu",
        static_cast<unsigned long>(command.messageId.sessionId >> 32),
        static_cast<unsigned long>(command.messageId.sessionId & 0xFFFFFFFFULL),
        static_cast<unsigned long>(command.messageId.sequence));
    return;
  }

  char source[13];
  macToHexLower(command.sourceMac, source);
  LOG("BT worker water start source=%s mask=0x%04x duration_count=%u id=%08lx%08lx:%lu",
      source, static_cast<unsigned>(potMask), static_cast<unsigned>(durationCount),
      static_cast<unsigned long>(command.messageId.sessionId >> 32),
      static_cast<unsigned long>(command.messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(command.messageId.sequence));
  // Detailed value log for future debugging:
  // for (size_t i = 0; i < durationCount; ++i) LOG("BT worker water duration[%u]=%u", i, durations[i]);
  size_t durationIndex = 0;
  for (int pot = 0; pot < WORKER_POT_COUNT; ++pot) {
    if ((potMask & (1u << pot)) == 0) continue;
    uint16_t seconds = durations[durationIndex++];
    LOG("BT worker valve on pot=%d seconds=%u", pot, static_cast<unsigned>(seconds));
    valveSetMask(static_cast<uint16_t>(1u << pot));
    vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(seconds) * 1000u));
    valveSetMask(0);
    LOG("BT worker valve off pot=%d", pot);
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  BtBodyBuilder done;
  btBodyBegin(done, TYPE_EVENT_WATER_DONE);
  if (!btTlvAppendU16(done, FIELD_POT_MASK, potMask)) return;
  BtSendResult result = btCommonSendCommand(command.sourceMac, done.data, done.len, 2, 700);
  LOG("BT worker water done sent source=%s mask=0x%04x status=%s id=%08lx%08lx:%lu",
      source, static_cast<unsigned>(potMask), btSendStatusName(result.status),
      static_cast<unsigned long>(result.messageId.sessionId >> 32),
      static_cast<unsigned long>(result.messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(result.messageId.sequence));
}

void workerControlTask(void*) {
  WorkerCommand command{};
  for (;;) {
    if (xQueueReceive(gCommandQueue, &command, portMAX_DELAY) != pdTRUE) continue;

    BtSendResult ack = btCommonSendAckAndWait(command.sourceMac, command.messageId);
    char source[13];
    macToHexLower(command.sourceMac, source);
    LOG("BT worker command ack source=%s type=%s(0x%02x) id=%08lx%08lx:%lu status=%s started=%u",
        source, btTypeName(command.body[0]), command.body[0],
        static_cast<unsigned long>(command.messageId.sessionId >> 32),
        static_cast<unsigned long>(command.messageId.sessionId & 0xFFFFFFFFULL),
        static_cast<unsigned long>(command.messageId.sequence),
        btSendStatusName(ack.status), ack.transmissionStarted ? 1u : 0u);
    if (!ack.transmissionStarted) continue;
    markCommunication();
    if (isDuplicate(command.messageId)) {
      LOG("BT worker command drop reason=duplicate source=%s type=%s(0x%02x) id=%08lx%08lx:%lu",
          source, btTypeName(command.body[0]), command.body[0],
          static_cast<unsigned long>(command.messageId.sessionId >> 32),
          static_cast<unsigned long>(command.messageId.sessionId & 0xFFFFFFFFULL),
          static_cast<unsigned long>(command.messageId.sequence));
      continue;
    }

    if (command.body[0] == TYPE_CMD_PROBE) {
      LOG("BT worker command execute type=CMD_PROBE source=%s id=%08lx%08lx:%lu",
          source, static_cast<unsigned long>(command.messageId.sessionId >> 32),
          static_cast<unsigned long>(command.messageId.sessionId & 0xFFFFFFFFULL),
          static_cast<unsigned long>(command.messageId.sequence));
      btWorkerAdvertiseStatus();
      continue;
    }
    if (command.body[0] == TYPE_CMD_WATER) {
      executeWaterCommand(command);
      continue;
    }
    if (command.body[0] == TYPE_CMD_SLEEP) {
      uint32_t seconds = 0;
      if (!btTlvReadRequiredU32(command.body + 1, command.bodyLen - 1,
                                FIELD_SLEEP_SEC, seconds)) {
        LOG("BT worker sleep drop reason=bad_tlv source=%s id=%08lx%08lx:%lu",
            source, static_cast<unsigned long>(command.messageId.sessionId >> 32),
            static_cast<unsigned long>(command.messageId.sessionId & 0xFFFFFFFFULL),
            static_cast<unsigned long>(command.messageId.sequence));
        continue;
      }
      LOG("BT worker command execute type=CMD_SLEEP source=%s seconds=%lu id=%08lx%08lx:%lu",
          source, static_cast<unsigned long>(seconds),
          static_cast<unsigned long>(command.messageId.sessionId >> 32),
          static_cast<unsigned long>(command.messageId.sessionId & 0xFFFFFFFFULL),
          static_cast<unsigned long>(command.messageId.sequence));
      valveSetMask(0);
      esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(seconds) * 1000000ULL);
      esp_deep_sleep_start();
    }
  }
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
  if (memcmp(targetMac, gThisMac, 6) != 0) {
    LOG("BT RX drop reason=target_mismatch source=%s target=%s type=%s(0x%02x) id=%08lx%08lx:%lu rssi=%d",
        source, target, btTypeName(type), type,
        static_cast<unsigned long>(messageId.sessionId >> 32),
        static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
        static_cast<unsigned long>(messageId.sequence), adv->getRSSI());
    return;
  }

  uint8_t pairedMac[6];
  bool paired = copyMainMac(pairedMac);
  char pairedText[13];
  macToHexLower(paired ? pairedMac : BROADCAST_MAC, pairedText);

  LOG("BT RX packet source=%s target=%s type=%s(0x%02x) id=%08lx%08lx:%lu rssi=%d len=%u paired=%u paired_main=%s",
      source, target, btTypeName(type), type,
      static_cast<unsigned long>(messageId.sessionId >> 32),
      static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(messageId.sequence), adv->getRSSI(),
      static_cast<unsigned>(bodyLen), paired ? 1u : 0u, pairedText);

  if (body[0] == TYPE_ACK && bodyLen == 1) {
    if (paired && memcmp(sourceMac, pairedMac, 6) != 0) {
      LOG("BT RX drop reason=paired_main_mismatch source=%s paired=%s type=ACK id=%08lx%08lx:%lu",
          source, pairedText,
          static_cast<unsigned long>(messageId.sessionId >> 32),
          static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
          static_cast<unsigned long>(messageId.sequence));
      return;
    }
    btCommonMarkAck(sourceMac, messageId);
    return;
  }

  if (!validateCommand(body, bodyLen)) {
    LOG("BT RX drop reason=invalid_command source=%s target=%s type=%s(0x%02x) id=%08lx%08lx:%lu",
        source, target, btTypeName(type), type,
        static_cast<unsigned long>(messageId.sessionId >> 32),
        static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
        static_cast<unsigned long>(messageId.sequence));
    return;
  }
  if (paired && memcmp(sourceMac, pairedMac, 6) != 0) {
    LOG("BT RX drop reason=paired_main_mismatch source=%s paired=%s type=%s(0x%02x) id=%08lx%08lx:%lu",
        source, pairedText, btTypeName(type), type,
        static_cast<unsigned long>(messageId.sessionId >> 32),
        static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
        static_cast<unsigned long>(messageId.sequence));
    return;
  }

  WorkerCommand command{};
  memcpy(command.sourceMac, sourceMac, 6);
  command.messageId = messageId;
  memcpy(command.body, body, bodyLen);
  command.bodyLen = static_cast<uint8_t>(bodyLen);
  if (xQueueSend(gCommandQueue, &command, 0) != pdTRUE) {
    LOG("BT RX drop reason=command_queue_full source=%s type=%s(0x%02x) id=%08lx%08lx:%lu",
        source, btTypeName(type), type,
        static_cast<unsigned long>(messageId.sessionId >> 32),
        static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
        static_cast<unsigned long>(messageId.sequence));
    return;
  }
  LOG("BT RX command queued source=%s type=%s(0x%02x) id=%08lx%08lx:%lu",
      source, btTypeName(type), type,
      static_cast<unsigned long>(messageId.sessionId >> 32),
      static_cast<unsigned long>(messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(messageId.sequence));
  if (!paired) setMainMac(sourceMac);
}
}

bool btLastCommOverdue() {
  portENTER_CRITICAL(&gWorkerStateMux);
  int64_t last = gLastCommunicationUs;
  portEXIT_CRITICAL(&gWorkerStateMux);
  return last != 0 && esp_timer_get_time() - last >= 300000000LL;
}

bool mainMacIsSet() {
  uint8_t mac[6];
  return copyMainMac(mac);
}

void mainMacReset() {
  uint8_t previous[6];
  portENTER_CRITICAL(&gWorkerStateMux);
  memcpy(previous, gMainMac, 6);
  memset(gMainMac, 0, sizeof(gMainMac));
  gLastCommandId = {};
  gLastCommunicationUs = 0;
  portEXIT_CRITICAL(&gWorkerStateMux);
  char mainMac[13];
  macToHexLower(previous, mainMac);
  LOG("BT worker main pairing reset previous=%s", mainMac);
}

void btWorkerAdvertiseStatus() {
  WorkerSensorSnapshot snapshot{};
  if (!getWorkerSensorSnapshot(snapshot)) {
    LOG("BT worker status drop reason=no_sensor_snapshot");
    return;
  }

  BtBodyBuilder body;
  btBodyBegin(body, TYPE_STATUS);
  if (!btTlvAppendU16(body, FIELD_BATT, snapshot.batteryMv) ||
      !btTlvAppendU8(body, FIELD_POT_COUNT, WORKER_POT_COUNT)) {
    LOG("BT worker status build failed reason=header");
    return;
  }
  uint8_t soilBytes[WORKER_POT_COUNT * 2];
  for (size_t i = 0; i < WORKER_POT_COUNT; ++i) {
    soilBytes[i * 2] = static_cast<uint8_t>(snapshot.soils[i] >> 8);
    soilBytes[i * 2 + 1] = static_cast<uint8_t>(snapshot.soils[i]);
  }
  if (!btTlvAppend(body, FIELD_SOIL_LIST, soilBytes, sizeof(soilBytes))) {
    LOG("BT worker status build failed reason=soil_list");
    return;
  }

  uint8_t target[6];
  if (!copyMainMac(target)) memcpy(target, BROADCAST_MAC, 6);
  char targetText[13];
  macToHexLower(target, targetText);
  LOG("BT worker status send target=%s pots=%u batt_mv=%u",
      targetText, static_cast<unsigned>(WORKER_POT_COUNT),
      static_cast<unsigned>(snapshot.batteryMv));
  // Detailed value log for future debugging:
  // for (size_t i = 0; i < WORKER_POT_COUNT; ++i) LOG("BT worker status soil[%u]=%u", i, snapshot.soils[i]);
  BtSendResult result = btCommonSendCommand(target, body.data, body.len, 2, 700);
  LOG("BT worker status result target=%s status=%s id=%08lx%08lx:%lu",
      targetText, btSendStatusName(result.status),
      static_cast<unsigned long>(result.messageId.sessionId >> 32),
      static_cast<unsigned long>(result.messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(result.messageId.sequence));
}

void btWorkerBegin() {
  esp_read_mac(gThisMac, ESP_MAC_BT);
  char self[13];
  macToHexLower(gThisMac, self);
  uint8_t pairedMac[6];
  bool paired = copyMainMac(pairedMac);
  char pairedText[13];
  macToHexLower(paired ? pairedMac : BROADCAST_MAC, pairedText);
  LOG("BT worker init start mac=%s paired=%u paired_main=%s",
      self, paired ? 1u : 0u, pairedText);
  if (paired) markCommunication();
  NimBLEDevice::init("plant-worker");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  if (!btCryptoInit()) {
    LOG("BT worker init failed reason=crypto");
    return;
  }
  if (!btCommonInitSender()) {
    LOG("BT worker init failed reason=sender");
    return;
  }
  gCommandQueue = xQueueCreate(COMMAND_QUEUE_LEN, sizeof(WorkerCommand));
  if (!gCommandQueue) {
    LOG("BT worker init failed reason=command_queue");
    return;
  }
  xTaskCreate(workerControlTask, "workerControl", 4096, nullptr, 3, nullptr);

  btCommonSetAdvertHandler(parseAdvert);
  btCommonInstallScanCallbacks();
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setInterval(150);
  scan->setWindow(50);
  scan->setMaxResults(0);
  scan->start(0);
  LOG("BT worker scan started interval=150 window=50");
}
