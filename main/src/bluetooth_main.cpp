#include "BluetoothMain.h"
#include "BluetoothCommon.h"
#include <NimBLEDevice.h>
#include <vector>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "Utility.h"
#include "config.h"
#include "WateringManager.h"

using namespace BT_TLV;

static uint8_t thisMac[6];
static WorkerNode nodes[MAX_WORKER_COUNT];
static int nodeCount = 0;

static void updateNode(const uint8_t mac[6], uint16_t soil, uint8_t batt) {
  // Check if node is added
  bool worker_added = false;
  for (int i=0;i<workerListCount;i++) {
    if (memcmp(workerList[i].mac, mac, 6)==0) {
      worker_added = true;
      break;
    }
  }
  if (!worker_added) return;
  for (int i=0;i<nodeCount;i++) {
    if (memcmp(nodes[i].mac, mac, 6)==0) {
      nodes[i].soil = soil; nodes[i].battery = batt; nodes[i].lastSeen = millis()/1000; return;
    }
  }
  if (nodeCount < MAX_WORKER_COUNT) {
    memcpy(nodes[nodeCount].mac, mac, 6);
    nodes[nodeCount].soil = soil; nodes[nodeCount].battery = batt; nodes[nodeCount].lastSeen = millis()/1000;
    nodes[nodeCount].lastWater = 0;
    nodeCount++;
  }
}

// Pending command tracking
struct PendingCmd { uint16_t nonce; bool inUse; bool acked; };
static const int MAX_PENDING = 8;
static PendingCmd pending[MAX_PENDING];
static uint16_t next_nonce = 1;
static uint16_t allocNonce() { uint16_t n = next_nonce++; if (next_nonce==0) next_nonce=1; return n; }
static void markAck(uint16_t nonce) { for (int i=0;i<MAX_PENDING;i++) if (pending[i].inUse && pending[i].nonce==nonce) pending[i].acked = true; }

static void parseAdvert(const uint8_t* data, size_t len, const NimBLEAdvertisedDevice* adv) {
  if (!data) return;
  uint8_t srcMac[6];
  extract_src_mac(adv, srcMac);

  // If this advert is an ACK from a worker, mark pending command as acknowledged
  uint16_t ackNonce;
  if (tlv_extract_ack(data,len,ackNonce)) { markAck(ackNonce); onCommandAcked(srcMac, ackNonce); return; }

  // Otherwise treat as worker status update (TYPE_MAC in TLV is target MAC on commands; the advertiser address
  // identifies the sender). Extract worker-reported fields and update node list.
  uint8_t tgtMac[6]; 
  if (tlv_extract_tgt_mac(data,len,tgtMac) && (memcmp(tgtMac, thisMac, 6)==0 || memcmp(tgtMac, BROADCAST_MAC, 6)==0)) {
    uint16_t soil; uint8_t batt;
    if (tlv_extract_soil(data,len,soil) && tlv_extract_batt(data,len,batt)) {
      updateNode(srcMac, soil, batt);

      if (autoEnabled && memcmp(tgtMac, BROADCAST_MAC, 6)==0) {
        unsigned long now_s = millis() / 1000;
        uint32_t sleepSec = calculateSleepSec(now_s);

        uint8_t payload[1 + 4];
        payload[0] = 0x02; // CMD_SLEEP
        // encode big-endian uint32 seconds
        payload[1] = (uint8_t)((sleepSec >> 24) & 0xFF);
        payload[2] = (uint8_t)((sleepSec >> 16) & 0xFF);
        payload[3] = (uint8_t)((sleepSec >> 8) & 0xFF);
        payload[4] = (uint8_t)(sleepSec & 0xFF);
        btMainQueueCommand(srcMac, payload, sizeof(payload), 2, 700);
        return;
      }
    }
    // If advertiser sent a payload and it's a water command, treat as watering completion
    const uint8_t* payload; size_t len;
    if (tlv_extract_payload(data,len,&payload,len) && len>0 && payload[0]==BT_CMD::CMD_WATER) {
      onWorkerCompleted(srcMac);
    }
    // send a scoped ACK back to the advertiser to confirm receipt (nonce=0)
    auto ack = tlv_make_ack((const uint8_t*)tgtMac, 0);
    BT_TLV::btCommonBroadcast(ack.data(), ack.size(), 100);
  }
}

// Command queue definitions
static const int CMD_QUEUE_LEN = 16;
static const size_t MAX_CMD_PAYLOAD = 24;
struct CmdItem { uint8_t mac[6]; uint8_t payload[MAX_CMD_PAYLOAD]; size_t len; int retries; unsigned timeoutMs; };
static QueueHandle_t cmdQueue = nullptr;

static void btMainSenderTask(void* pv) {
  (void)pv;
  CmdItem item;
  for (;;) {
    if (xQueueReceive(cmdQueue, &item, portMAX_DELAY) == pdTRUE) {
      uint16_t nonce = allocNonce();
      auto cmd = tlv_make_command(item.mac, nonce, item.payload, item.len);
      int slot=-1; for (int i=0;i<MAX_PENDING;i++) if (!pending[i].inUse) { slot=i; break; }
      if (slot<0) continue; // drop if no pending slot
      pending[slot].inUse = true; pending[slot].acked = false; pending[slot].nonce = nonce;
      // notify watering manager of the actual nonce used for this sent command
      if (item.payload[0] == 0x03) onCommandSent(item.mac, nonce);
      for (int attempt=0; attempt<=item.retries; ++attempt) {
        BT_TLV::btCommonBroadcast(cmd.data(), cmd.size(), 100);
        unsigned long start = millis();
        while (millis()-start < item.timeoutMs) {
          if (pending[slot].acked) break;
          vTaskDelay(pdMS_TO_TICKS(20));
        }
        if (pending[slot].acked) break;
      }
      pending[slot].inUse = false;
    }
  }
}

void btMainBegin() {
  // capture this MAC
  esp_read_mac(thisMac, ESP_MAC_BT);
  NimBLEDevice::init("plant-main");
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);
  NimBLEScan* scan = NimBLEDevice::getScan();
  // use common scan callback and set our advert handler
  BT_TLV::btCommonSetAdvertHandler([](const uint8_t* d, size_t l, const NimBLEAdvertisedDevice* adv){ parseAdvert(d,l,adv); });
  BT_TLV::btCommonInstallScanCallbacks();
  scan->setInterval(45); 
  scan->setWindow(15); 
  scan->setActiveScan(true);
  scan->start(0);
  // start command queue and sender task
  if (!cmdQueue) cmdQueue = xQueueCreate(CMD_QUEUE_LEN, sizeof(CmdItem));
  if (cmdQueue) xTaskCreate(btMainSenderTask, "btSender", 3072, NULL, 3, NULL);
}

bool btMainQueueCommand(const uint8_t target_mac[6], const uint8_t* payload, size_t len, int retries, unsigned timeoutMs) {
  if (!cmdQueue) return false;
  if (len > MAX_CMD_PAYLOAD) return false;
  CmdItem it;
  memcpy(it.mac, target_mac, 6);
  memset(it.payload, 0, MAX_CMD_PAYLOAD);
  memcpy(it.payload, payload, len);
  it.len = len; it.retries = retries; it.timeoutMs = timeoutMs;
  if (xQueueSend(cmdQueue, &it, 0) == pdTRUE) {
    return true; // queued
  }
  return false; // queue full
}

int btMainNodeCount() { return nodeCount; }
const WorkerNode* btMainNodeAt(int idx) { if (idx<0||idx>=nodeCount) return nullptr; return &nodes[idx]; }

// Find a discovered node by MAC. Returns nullptr if not found.
const WorkerNode* btMainFindNodeByMac(const uint8_t mac[6]) {
  for (int i=0;i<nodeCount;i++) if (memcmp(nodes[i].mac, mac, 6)==0) return &nodes[i];
  return nullptr;
}

void btMainSetNodeLastWater(const uint8_t mac[6], unsigned long epochSeconds) {
  for (int i=0;i<nodeCount;i++) {
    if (memcmp(nodes[i].mac, mac, 6)==0) { nodes[i].lastWater = epochSeconds; break; }
  }
}
