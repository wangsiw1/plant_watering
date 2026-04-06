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
      nodes[i].soil = soil; 
      nodes[i].battery = batt; 
      nodes[i].lastSync = millis()/1000; 
      return;
    }
  }
  // Do not create a new node here. Node placeholders are created when a worker
  // is added via addWorkerByHex (btMainEnsureNodeExists). This avoids keeping
  // discovery cache entries for non-configured workers and centralizes init.
  return;
}

// Ensure a placeholder node exists in the discovery cache for the given MAC.
// This is called when a worker is added to the configured list.
void btMainEnsureNodeExists(const uint8_t mac[6]) {
  for (int i = 0; i < nodeCount; ++i) {
    if (memcmp(nodes[i].mac, mac, 6) == 0) return;
  }
  if (nodeCount >= MAX_WORKER_COUNT) return;
  memcpy(nodes[nodeCount].mac, mac, 6);
  nodes[nodeCount].soil = 0;
  nodes[nodeCount].battery = 0;
  nodes[nodeCount].lastSync = 0;
  nodes[nodeCount].lastWater = 0;
  nodes[nodeCount].lastNonce = 0;
  nodeCount++;
}

// Remove a discovered node by MAC from the cache.
void btMainRemoveNodeByMac(const uint8_t mac[6]) {
  for (int i = 0; i < nodeCount; ++i) {
    if (memcmp(nodes[i].mac, mac, 6) == 0) {
      for (int k = i; k < nodeCount - 1; ++k) nodes[k] = nodes[k+1];
      nodeCount--;
      return;
    }
  }
}

// Remove nodes that are not present in the configured workerList.
void btMainCleanupOrphanedNodes() {
  int write = 0;
  for (int i = 0; i < nodeCount; ++i) {
    bool found = false;
    for (int j = 0; j < workerListCount; ++j) {
      if (memcmp(nodes[i].mac, workerList[j].mac, 6) == 0) { found = true; break; }
    }
    if (found) {
      if (write != i) nodes[write] = nodes[i];
      write++;
    }
  }
  nodeCount = write;
}

// Background task to periodically clean up orphaned nodes.
static void btMainCleanupTask(void* param) {
  (void)param;
  for (;;) {
    // Sleep for 60 seconds
    vTaskDelay(pdMS_TO_TICKS(60000));
    btMainCleanupOrphanedNodes();
  }
}

// Pending command tracking and sender moved to BluetoothCommon

static void parseAdvert(const uint8_t* data, size_t len, const NimBLEAdvertisedDevice* adv) {
  if (!data) return;
  
  // Otherwise treat as worker status update (TYPE_MAC in TLV is target MAC on commands; the advertiser address
  // identifies the sender). Extract worker-reported fields and update node list.
  uint8_t tgtMac[6];
  if (tlv_extract_tgt_mac(data,len,tgtMac) && (memcmp(tgtMac, thisMac, 6)==0 || memcmp(tgtMac, BROADCAST_MAC, 6)==0)) {
    uint8_t srcMac[6];
    extract_src_mac(adv, srcMac);
    LOG("[<-] Received advert from worker MAC: %02X:%02X:%02X:%02X:%02X:%02X", srcMac[0], srcMac[1], srcMac[2], srcMac[3], srcMac[4], srcMac[5]);
  
    // If this advert is an ACK from a worker, mark pending command as acknowledged
    uint16_t ackNonce;
    if (tlv_extract_ack(data,len,ackNonce)) {
      LOG("[v] [%d] Received ACK from worker", ackNonce);
      BT_TLV::btCommonMarkAck(ackNonce);
      onCommandAcked(srcMac, ackNonce);
      return;
    }

    uint16_t nonce;
    tlv_extract_nonce(data,len,nonce);
    BT_TLV::btCommonQueueAck(srcMac, nonce);
    LOG("[->] [%d] Sent ACK to worker", nonce);

    uint8_t nonceUpdateResult = btMainSetNodeLastNonce(srcMac, nonce);
    if (nonceUpdateResult == 0) {
      LOG("[x] [%d] Skip not added node", nonce); return;
    } else if (nonceUpdateResult == 2) {
      LOG("[x] [%d] Skip duplication", nonce); return;
    }
    LOG("[<-] [%d] Received status update from worker", nonce);

    uint16_t soil; uint8_t batt;
    if (tlv_extract_soil(data,len,soil) && tlv_extract_batt(data,len,batt)) {
      LOG("[<-] [%d] Received soil=%d batt=%d from worker", nonce, soil, batt);
      updateNode(srcMac, soil, batt);

      if (state == SLEEPING) {
        unsigned long now_s = millis() / 1000;
        uint32_t sleepSec = calculateSleepSec(now_s);
        LOG("Entering SLEEPING state, calculating sleep duration: %d seconds", sleepSec);

        uint8_t payload[6];
        payload[0] = BT_TLV::TYPE_CMD_SLEEP; // TLV type
        payload[1] = 4; // length
        // encode big-endian uint32 seconds
        payload[2] = (uint8_t)((sleepSec >> 24) & 0xFF);
        payload[3] = (uint8_t)((sleepSec >> 16) & 0xFF);
        payload[4] = (uint8_t)((sleepSec >> 8) & 0xFF);
        payload[5] = (uint8_t)(sleepSec & 0xFF);
        LOG("[->] Queuing CMD_SLEEP command to worker %02X:%02X:%02X:%02X:%02X:%02X", srcMac[0], srcMac[1], srcMac[2], srcMac[3], srcMac[4], srcMac[5]);
        btMainQueueCommand(srcMac, payload, sizeof(payload), 2, 700);
        return;
      }
    }
    // If advertiser included a command TLV (e.g. BT_TLV::TYPE_CMD_WATER), treat as watering completion
    uint16_t dur;
    if (tlv_extract_cmd_water(data, len, dur)) {
      LOG("[<-] [%d] Received CMD_WATER completion from worker %02X:%02X:%02X:%02X:%02X:%02X", nonce, srcMac[0], srcMac[1], srcMac[2], srcMac[3], srcMac[4], srcMac[5]);
      onWorkerCompleted(srcMac);
    }
  }
}

// Command queue and sender live in BluetoothCommon; main will register callbacks and use the shared queue.

void btMainBegin() {
  // capture this MAC
  esp_read_mac(thisMac, ESP_MAC_BT);
  NimBLEDevice::init("plant-main");
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);
  NimBLEScan* scan = NimBLEDevice::getScan();
  // Pre-populate discovery cache with configured workers so node list
  // matches configuration at startup.
  for (int i = 0; i < workerListCount; ++i) {
    btMainEnsureNodeExists(workerList[i].mac);
  }
  // use common scan callback and set our advert handler
  BT_TLV::btCommonSetAdvertHandler([](const uint8_t* d, size_t l, const NimBLEAdvertisedDevice* adv){ parseAdvert(d,l,adv); });
  BT_TLV::btCommonInstallScanCallbacks();
  scan->setInterval(50);  // 31.25 ms
  scan->setWindow(50);    // 31.25 ms
  scan->start(0);
  // start command queue and sender task
  BT_TLV::btCommonRegisterOnCommandSent(onCommandSent);
  BT_TLV::btCommonInitSender();
  // start periodic cleanup task to remove orphaned nodes
  xTaskCreate(btMainCleanupTask, "btCleanup", 2048, NULL, 1, NULL);
}

bool btMainQueueCommand(const uint8_t target_mac[6], const uint8_t* payload, size_t len, int retries, unsigned timeoutMs) {
  return BT_TLV::btCommonQueueCommand(target_mac, payload, len, retries, timeoutMs);
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

uint8_t btMainSetNodeLastNonce(const uint8_t mac[6], unsigned long nonce) {
  // Return 
  // 0: MAC not found
  // 1: Updated
  // 2: Nonce is the same
  for (int i=0;i<nodeCount;i++) {
    if (memcmp(nodes[i].mac, mac, 6)==0) {
      if (nodes[i].lastNonce != nonce) {
        nodes[i].lastNonce = nonce;
        return 1;
      }
      return 2;
    }
  }
  return 0;
}
