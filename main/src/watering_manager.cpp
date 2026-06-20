#include "WateringManager.h"
#include "BluetoothMain.h"
#include "BluetoothCommon.h"
#include "Pump.h"
#include "Sensor.h"
#include "Utility.h"
#include "config.h"

#include <cstring>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace {
constexpr UBaseType_t MANUAL_QUEUE_LEN = 4;
constexpr uint32_t WATER_ACK_TIMEOUT_MS = 700;
constexpr uint32_t WATER_COMPLETION_MARGIN_SECONDS = 5;

struct ManualWaterRequest {
  uint8_t mac[6];
  uint16_t potMask;
  uint16_t durations[MAX_POTS_PER_DEVICE];
  uint8_t durationCount;
};

QueueHandle_t gManualQueue = nullptr;
portMUX_TYPE gCompletionMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t gActiveMac[6] = {};
uint16_t gActiveMask = 0;
bool gCompleted = false;

struct PumpStartContext {
  bool started;
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

void startPump(void* rawContext) {
  PumpStartContext* context = static_cast<PumpStartContext*>(rawContext);
  LOG("Watering pump start");
  pumpOn();
  context->started = true;
}

void beginCompletionWait(const uint8_t mac[6], uint16_t mask) {
  portENTER_CRITICAL(&gCompletionMux);
  memcpy(gActiveMac, mac, 6);
  gActiveMask = mask;
  gCompleted = false;
  portEXIT_CRITICAL(&gCompletionMux);
  char target[13];
  macToHexLower(mac, target);
  LOG("Watering completion wait begin worker=%s mask=0x%04x",
      target, static_cast<unsigned>(mask));
}

bool completionReceived() {
  portENTER_CRITICAL(&gCompletionMux);
  bool completed = gCompleted;
  portEXIT_CRITICAL(&gCompletionMux);
  return completed;
}

void endCompletionWait() {
  portENTER_CRITICAL(&gCompletionMux);
  memset(gActiveMac, 0, sizeof(gActiveMac));
  gActiveMask = 0;
  gCompleted = false;
  portEXIT_CRITICAL(&gCompletionMux);
  LOG("Watering completion wait end");
}
}

void wateringManagerInit() {
  if (!gManualQueue) {
    gManualQueue = xQueueCreate(MANUAL_QUEUE_LEN, sizeof(ManualWaterRequest));
  }
}

bool wateringQueueManual(const uint8_t mac[6], uint16_t potMask,
                         const uint16_t* durations, size_t durationCount) {
  if (!gManualQueue || !mac || !durations || potMask == 0 ||
      durationCount == 0 || durationCount > MAX_POTS_PER_DEVICE ||
      durationCount != static_cast<size_t>(__builtin_popcount(potMask))) {
    LOG("Watering manual queue rejected reason=invalid_input queue=%p mac=%p mask=0x%04x duration_count=%u",
        gManualQueue, mac, static_cast<unsigned>(potMask),
        static_cast<unsigned>(durationCount));
    return false;
  }
  ManualWaterRequest request{};
  memcpy(request.mac, mac, 6);
  request.potMask = potMask;
  request.durationCount = durationCount;
  for (size_t i = 0; i < durationCount; ++i) {
    if (durations[i] == 0 || durations[i] > 60) {
      char target[13];
      macToHexLower(mac, target);
      LOG("Watering manual queue rejected reason=bad_duration worker=%s index=%u value=%u",
          target, static_cast<unsigned>(i), static_cast<unsigned>(durations[i]));
      return false;
    }
    request.durations[i] = durations[i];
  }
  bool queued = xQueueSend(gManualQueue, &request, 0) == pdTRUE;
  char target[13];
  macToHexLower(mac, target);
  LOG("Watering manual queue worker=%s mask=0x%04x duration_count=%u queued=%u",
      target, static_cast<unsigned>(potMask),
      static_cast<unsigned>(durationCount), queued ? 1u : 0u);
  return queued;
}

bool wateringProcessOneManual() {
  if (!gManualQueue) return false;
  ManualWaterRequest request{};
  if (xQueueReceive(gManualQueue, &request, 0) != pdTRUE) return false;
  char target[13];
  macToHexLower(request.mac, target);
  LOG("Watering manual dequeue worker=%s mask=0x%04x duration_count=%u",
      target, static_cast<unsigned>(request.potMask),
      static_cast<unsigned>(request.durationCount));
  setRuntimeState(WATERING);
  bool completed = wateringExecuteWorker(request.mac, request.potMask,
                                         request.durations,
                                         request.durationCount, false);
  LOG("Watering manual complete worker=%s mask=0x%04x completed=%u",
      target, static_cast<unsigned>(request.potMask), completed ? 1u : 0u);
  setRuntimeState(READY);
  return true;
}

bool wateringExecuteWorker(const uint8_t mac[6], uint16_t potMask,
                           const uint16_t* durations, size_t durationCount,
                           bool usePump) {
  if (!mac || !durations || potMask == 0 || durationCount == 0 ||
      durationCount > MAX_POTS_PER_DEVICE ||
      durationCount != static_cast<size_t>(__builtin_popcount(potMask))) {
    LOG("Watering execute rejected reason=invalid_input mac=%p mask=0x%04x duration_count=%u use_pump=%u",
        mac, static_cast<unsigned>(potMask), static_cast<unsigned>(durationCount),
        usePump ? 1u : 0u);
    return false;
  }

  char target[13];
  macToHexLower(mac, target);
  unsigned long totalSeconds = 0;
  uint8_t durationBytes[MAX_POTS_PER_DEVICE * 2];
  for (size_t i = 0; i < durationCount; ++i) {
    if (durations[i] == 0 || durations[i] > 60) {
      LOG("Watering execute rejected reason=bad_duration worker=%s index=%u value=%u",
          target, static_cast<unsigned>(i), static_cast<unsigned>(durations[i]));
      return false;
    }
    totalSeconds += durations[i];
    durationBytes[i * 2] = static_cast<uint8_t>(durations[i] >> 8);
    durationBytes[i * 2 + 1] = static_cast<uint8_t>(durations[i]);
  }
  LOG("Watering execute start worker=%s mask=0x%04x duration_count=%u total_seconds=%lu use_pump=%u",
      target, static_cast<unsigned>(potMask), static_cast<unsigned>(durationCount),
      totalSeconds, usePump ? 1u : 0u);
  // Detailed value log for future debugging:
  // for (size_t i = 0; i < durationCount; ++i) LOG("Watering duration[%u]=%u", i, durations[i]);

  BT_TLV::BtBodyBuilder body;
  BT_TLV::btBodyBegin(body, BT_TLV::TYPE_CMD_WATER);
  if (!BT_TLV::btTlvAppendU16(body, BT_TLV::FIELD_POT_MASK, potMask) ||
      !BT_TLV::btTlvAppend(body, BT_TLV::FIELD_DURATION_LIST,
                           durationBytes, durationCount * 2)) {
    LOG("Watering execute rejected reason=build_failed worker=%s mask=0x%04x",
        target, static_cast<unsigned>(potMask));
    return false;
  }

  if (usePump && getTankLevel() <= 2800) {
    LOG("Watering execute rejected reason=tank_low worker=%s tank_mv=%u",
        target, static_cast<unsigned>(getTankLevel()));
    return false;
  }
  beginCompletionWait(mac, potMask);
  PumpStartContext pumpContext{};
  BtSendResult sent = btMainSendCommand(
      mac, body.data, body.len, 0, WATER_ACK_TIMEOUT_MS,
      usePump ? startPump : nullptr, usePump ? &pumpContext : nullptr);
  char ack[13];
  macToHexLower(sent.ackMac, ack);
  LOG("Watering command result worker=%s mask=0x%04x status=%s started=%u ack=%s id=%08lx%08lx:%lu",
      target, static_cast<unsigned>(potMask), btSendStatusName(sent.status),
      sent.transmissionStarted ? 1u : 0u, ack,
      static_cast<unsigned long>(sent.messageId.sessionId >> 32),
      static_cast<unsigned long>(sent.messageId.sessionId & 0xFFFFFFFFULL),
      static_cast<unsigned long>(sent.messageId.sequence));
  if (!sent.transmissionStarted) {
    endCompletionWait();
    return false;
  }

  int64_t deadlineUs =
      sent.startedUs +
      static_cast<int64_t>(totalSeconds + WATER_COMPLETION_MARGIN_SECONDS) *
          1000000LL;
  while (!completionReceived() && esp_timer_get_time() < deadlineUs) {
    if (usePump && getTankLevel() <= 2800) {
      LOG("Watering wait abort reason=tank_low worker=%s tank_mv=%u",
          target, static_cast<unsigned>(getTankLevel()));
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  bool completed = completionReceived();
  if (!completed) {
    LOG("Watering wait ended worker=%s mask=0x%04x completed=0 deadline_reached=%u",
        target, static_cast<unsigned>(potMask),
        esp_timer_get_time() >= deadlineUs ? 1u : 0u);
  }
  if (pumpContext.started) {
    LOG("Watering pump stop");
    pumpOff();
  }
  endCompletionWait();

  if (completed) {
    LOG("Watering completed worker=%s mask=0x%04x",
        target, static_cast<unsigned>(potMask));
    btMainMarkNodeWatered(mac, potMask);
    uint64_t utc = getCurrentEpochSec();
    if (usePump && utc != 0) setLastWateringUtc(utc);
  }
  LOG("Watering execute result worker=%s mask=0x%04x completed=%u",
      target, static_cast<unsigned>(potMask), completed ? 1u : 0u);
  return completed;
}

void wateringNotifyCompleted(const uint8_t mac[6], uint16_t potMask) {
  if (!mac) return;
  char target[13];
  macToHexLower(mac, target);
  bool matched = false;
  portENTER_CRITICAL(&gCompletionMux);
  if (memcmp(gActiveMac, mac, 6) == 0 && gActiveMask == potMask) {
    gCompleted = true;
    matched = true;
  }
  portEXIT_CRITICAL(&gCompletionMux);
  LOG("Watering completion notify worker=%s mask=0x%04x matched=%u",
      target, static_cast<unsigned>(potMask), matched ? 1u : 0u);
}
