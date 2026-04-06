#include "BluetoothCommon.h"
#include "Sensor.h"
#include "Valve.h"
#include "Battery.h"
#include <NimBLEDevice.h>
#include <vector>
#include <cstring>

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

void btWorkerAdvertiseStatus(uint16_t soil, uint8_t batt) {
  uint8_t payload[7];
  payload[0] = TYPE_SOIL;
  payload[1] = 2;
  payload[2] = (uint8_t)((soil >> 8) & 0xFF);
  payload[3] = (uint8_t)(soil & 0xFF);
  payload[4] = TYPE_BATT;
  payload[5] = 1;
  payload[6] = (uint8_t)(batt & 0xFF);
  LOG("[->] Advertising status: soil=%d batt=%d", soil, batt);
  if (mainMacIsSet()) {
    BT_TLV::btCommonQueueCommand(mainMac, payload, sizeof(payload), 2, 700);
  } else {
    BT_TLV::btCommonQueueCommand(BROADCAST_MAC, payload, sizeof(payload), 2, 700);
  }
}

static void parseAdvert(const uint8_t* data, size_t len, const NimBLEAdvertisedDevice* adv) {
  if (!data) return;
  
  uint8_t tgtMac[6];
  if (tlv_extract_tgt_mac(data,len,tgtMac) && (memcmp(tgtMac, thisMac, 6)==0)) {
    uint8_t srcMac[6];
    extract_src_mac(adv, srcMac);
    LOG("[<-] Received advert from MAC: %02X:%02X:%02X:%02X:%02X:%02X", srcMac[0], srcMac[1], srcMac[2], srcMac[3], srcMac[4], srcMac[5]);
      
    // learn main MAC on first command (use advertiser address from adv)
    if (!mainMacIsSet()) {
      LOG("Learning main MAC from advertiser");
      memcpy(mainMac, srcMac, 6);
    }

    // if mainMac is set, only accept commands from that advertiser address
    if (mainMacIsSet() && memcmp(srcMac, mainMac, 6) != 0) {
      return;
    }
 
    // If this advert is an ACK from a worker, mark pending command as acknowledged
    uint16_t ackNonce;
    if (tlv_extract_ack(data,len,ackNonce)) {
      LOG("[v] [%d] Received ACK", ackNonce);
      BT_TLV::btCommonMarkAck(ackNonce);
      return;
    }

    uint16_t nonce;
    tlv_extract_nonce(data,len,nonce);
    // send ACK (include our MAC to scope the ACK to this system)
    BT_TLV::btCommonQueueAck(srcMac, nonce);
    LOG("[->] [%d] Sending ACK", nonce);
    btLastComm = millis();

    if (nonce == mainLastNonce) { LOG("[x] [%d] Skip duplication", nonce); return; } 
    mainLastNonce = nonce;
    LOG("[<-] [%d] Received command for this node", nonce);

    // Use helper extractors for command TLVs
    uint16_t duration;
    uint32_t delay_s;
    if (tlv_extract_cmd_probe(data, len)) {
      LOG("[<-] [%d] Received CMD_PROBE command", nonce);
      uint16_t soil = getSoilMoisture();
      uint8_t batt = getBattLevel();
      btWorkerAdvertiseStatus(soil, batt);
    }
    else if (tlv_extract_cmd_water(data, len, duration)) {
      LOG("[<-] [%d] Received CMD_WATER command for %d seconds", nonce, duration);
      valveOn();
      vTaskDelay(pdMS_TO_TICKS(duration * 1000));
      valveOff();
      // enqueue the CMD_WATER TLV (type+len+value)
      uint8_t buf[4]; 
      buf[0] = TYPE_CMD_WATER; 
      buf[1] = 2; 
      buf[2] = (uint8_t)(duration>>8); 
      buf[3] = (uint8_t)(duration & 0xFF);
      LOG("[->] Queuing water confirmation to main");
      BT_TLV::btCommonQueueCommand(mainMac, buf, sizeof(buf), 2, 700);
    }
    else if (tlv_extract_cmd_sleep(data, len, delay_s)) {
      LOG("[<-] [%d] Received CMD_SLEEP command for %d seconds", nonce, delay_s);
      esp_sleep_enable_timer_wakeup((uint64_t)delay_s * 1000000ULL);
      vTaskDelay(pdMS_TO_TICKS(50));
      esp_deep_sleep_start();
    }
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
