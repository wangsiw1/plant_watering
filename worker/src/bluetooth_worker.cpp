#include "BluetoothCommon.h"
#include "Sensor.h"
#include "Valve.h"
#include "Battery.h"
#include "HardwareConfig.h"
#include <NimBLEDevice.h>
#include <cstring>
#include <esp_mac.h>
#include <inttypes.h>

using namespace BT_TLV;

// Worker-side behavior:
// - Periodically advertise status TLV (mac, soil, batt)
// - Listen for commands (advertisements) that contain TYPE_TARGET matching own MAC
// - When command received and targeted, extract nonce and payload, send ACK TLV with nonce,
//   then (placeholder) perform action (user should replace with actual valve control)

static uint8_t thisMac[6];
static unsigned long btLastComm = millis();
// remember the one main controller MAC we will accept commands from, store in RTC memory to maintain after deep sleep
static RTC_DATA_ATTR uint8_t mainMac[6] = {0,0,0,0,0,0};
static uint16_t mainLastNonce = 0;

// Check if no bluetooth communication over 5 minutes
bool btLastCommOverdue() { if (millis() - btLastComm >= 300000) return true; return false; }
bool mainMacIsSet() { for (int i=0;i<6;i++) if (mainMac[i]!=0) return true; return false; }
void mainMacReset() { memset(mainMac, 0, sizeof(mainMac)); mainLastNonce = 0; }

void btWorkerAdvertiseStatus() {
  // Build compact STATUS payload: [TYPE_STATUS][Battery(1)][PotCount(1)][Soil1(2), ...]
  const size_t payload_len = 3 + 2 * WORKER_POT_COUNT;
  std::vector<uint8_t> payload(payload_len);
  payload[0] = TYPE_STATUS;
  uint8_t batt = getBattLevel();
  payload[1] = batt;
  payload[2] = (uint8_t)WORKER_POT_COUNT;
  for (size_t i = 0; i < WORKER_POT_COUNT; ++i) {
    uint16_t soil = getSoilMoisture(i);
    payload[3 + i * 2] = (uint8_t)((soil >> 8) & 0xFF);
    payload[3 + i * 2 + 1] = (uint8_t)(soil & 0xFF);
  }
  LOG("[->] Advertising compact STATUS: pots=%d batt=%d", WORKER_POT_COUNT, batt);
  if (mainMacIsSet()) BT_TLV::btCommonQueueCommand(mainMac, payload.data(), payload.size(), 2, 700);
  else BT_TLV::btCommonQueueCommand(BROADCAST_MAC, payload.data(), payload.size(), 2, 700);
}

static void parseAdvert(const uint8_t* data, size_t len, const NimBLEAdvertisedDevice* adv) {
  if (!data) return;

  // Parse compact header (CompanyID already stripped by common callback)
  uint8_t compactTarget[6]; uint16_t nonce; const uint8_t* payload_ptr; size_t payload_len;
  if (!parse_compact_packet_header(data, len, compactTarget, nonce, payload_ptr, payload_len)) return;
  if (memcmp(compactTarget, thisMac, 6) != 0) return;

  uint8_t srcMac[6]; extract_src_mac(adv, srcMac);
  LOG("[<-] (compact) Received advert from MAC: %02X:%02X:%02X:%02X:%02X:%02X", srcMac[0], srcMac[1], srcMac[2], srcMac[3], srcMac[4], srcMac[5]);

  // learn main MAC on first command
  if (!mainMacIsSet()) { LOG("Learning main MAC from advertiser"); memcpy(mainMac, srcMac, 6); }
  if (mainMacIsSet() && memcmp(srcMac, mainMac, 6) != 0) return;

  // ACK handling (mark pending if present)
  if (payload_len >= 1 && payload_ptr[0] == BT_TLV::TYPE_ACK) {
    LOG("[v] [%d] Received ACK (compact)", nonce);
    BT_TLV::btCommonMarkAck(nonce);
    return;
  }

  // send ACK back
  BT_TLV::btCommonQueueAck(srcMac, nonce);
  LOG("[->] [%d] Sending ACK (compact)", nonce);
  btLastComm = millis();

  if (nonce == mainLastNonce) { LOG("[x] [%d] Skip duplication", nonce); return; }
  mainLastNonce = nonce;
  LOG("[<-] [%d] Received compact command for this node", nonce);

  // Handle compact commands
  if (payload_len >= 1 && payload_ptr[0] == BT_TLV::TYPE_CMD_PROBE) {
    LOG("[<-] [%d] Received CMD_PROBE (compact)", nonce);
    btWorkerAdvertiseStatus();
  }
  else if (payload_len >= 1 && payload_ptr[0] == BT_TLV::TYPE_CMD_WATER) {
    // New compact format (required):
    // [TYPE_CMD_WATER][potMask(2)][dur1(2), dur2(2), ...]
    // Durations correspond to set bits in potMask in increasing pot index order (0..N-1).
    // Example: if potMask has bits for pots 0,2,3 then durations = [dur_for_0, dur_for_2, dur_for_3].
    if (payload_len < 3) {
      LOG("[<-] [%d] CMD_WATER missing mask/durations", nonce);
      return;
    }
    uint16_t potMask = (uint16_t(payload_ptr[1])<<8) | uint16_t(payload_ptr[2]);
    size_t remaining = payload_len - 3;
    size_t durationsCount = remaining / 2;
    uint16_t durations[WORKER_POT_COUNT];
    for (size_t di = 0; di < durationsCount; ++di) {
      durations[di] = (uint16_t(payload_ptr[3 + di*2])<<8) | uint16_t(payload_ptr[3 + di*2 + 1]);
    }
    LOG("[<-] [%d] Received CMD_WATER (compact) mask=%04X durations=%d", nonce, potMask, (int)durationsCount);
    size_t di = 0;
    for (int pi = 0; pi < WORKER_POT_COUNT; ++pi) {
      if (potMask & (1u << pi)) {
        uint16_t duration = (di < durationsCount) ? durations[di++] : 0;
        valveSetMask((uint16_t)(1u << pi));
        vTaskDelay(pdMS_TO_TICKS(duration * 1000));
        valveSetMask(0);
        vTaskDelay(pdMS_TO_TICKS(50));
      }
    }
    // Send simple completion notice back to main: [TYPE_CMD_WATER][potMask(2)]
    uint8_t buf[3];
    buf[0] = BT_TLV::TYPE_CMD_WATER;
    buf[1] = (uint8_t)((potMask >> 8) & 0xFF);
    buf[2] = (uint8_t)(potMask & 0xFF);
    BT_TLV::btCommonQueueCommand(mainMac, buf, 3, 2, 700);
  }
  else if (payload_len >= 1 && payload_ptr[0] == BT_TLV::TYPE_CMD_SLEEP) {
    uint32_t delay_s = 0;
    if (payload_len >= 5) {
      delay_s = (uint32_t(payload_ptr[1])<<24) | (uint32_t(payload_ptr[2])<<16) | (uint32_t(payload_ptr[3])<<8) | uint32_t(payload_ptr[4]);
    }
    LOG("[<-] [%d] Received CMD_SLEEP (compact) for %" PRIu32 " seconds", nonce, delay_s);
    esp_sleep_enable_timer_wakeup((uint64_t)delay_s * 1000000ULL);
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_deep_sleep_start();
  }
}

void btWorkerBegin() {
  // capture this MAC
  esp_read_mac(thisMac, ESP_MAC_BT);
  NimBLEDevice::init("plant-worker");
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);
  NimBLEScan* scan = NimBLEDevice::getScan();
  // set our advert handler and install common scan callbacks
  BT_TLV::btCommonSetAdvertHandler([](const uint8_t* d, size_t l, const NimBLEAdvertisedDevice* adv){ parseAdvert(d,l,adv); });
  BT_TLV::btCommonInstallScanCallbacks();
  scan->setInterval(150);  // 150 * 0.625ms = 93.75ms scan interval
  scan->setWindow(50);    // 15 * 0.625ms = 31.25 ms actual listen time
  scan->start(0);
  // start shared command sender so worker can queue non-ACK packets with retry/timeout
  BT_TLV::btCommonInitSender();
}
