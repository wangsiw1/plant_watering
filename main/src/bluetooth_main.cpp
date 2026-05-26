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

static void updateNode(const uint8_t mac[6], const uint16_t *soils, uint8_t potCount, uint8_t batt) {
  // Only update nodes for configured workers
  bool worker_added = false;
  for (int i = 0; i < workerListCount; ++i) {
    if (memcmp(workerList[i].mac, mac, 6) == 0) { worker_added = true; break; }
  }
  if (!worker_added) return;

  // Find existing discovery cache entry and update soils/battery/lastSync
  for (int i = 0; i < nodeCount; ++i) {
    if (memcmp(nodes[i].mac, mac, 6) == 0) {
      nodes[i].battery = batt;
      nodes[i].potCount = (uint8_t)min((size_t)potCount, (size_t)MAX_POTS_PER_DEVICE);
      for (size_t s = 0; s < nodes[i].potCount; ++s) {
        if (soils && s < potCount) nodes[i].soils[s] = soils[s];
        else nodes[i].soils[s] = 0;
      }
      nodes[i].lastSync = millis() / 1000;
      // Ensure persisted per-pot configs exist for this worker now that we know potCount
      ensureWorkerConfigsForMac(nodes[i].mac, nodes[i].potCount);
      return;
    }
  }
  // Do not create a new node here; placeholders are created via btMainEnsureNodeExists
}

// Ensure a placeholder node exists in the discovery cache for the given MAC.
// This is called when a worker is added to the configured list.
void btMainEnsureNodeExists(const uint8_t mac[6]) {
  for (int i = 0; i < nodeCount; ++i) {
    if (memcmp(nodes[i].mac, mac, 6) == 0) return;
  }
  if (nodeCount >= MAX_WORKER_COUNT) return;
  memcpy(nodes[nodeCount].mac, mac, 6);
  // initialize soils array and basic metadata (unknown potCount until worker syncs)
  nodes[nodeCount].potCount = 0;
  for (int si = 0; si < MAX_POTS_PER_DEVICE; ++si) nodes[nodeCount].soils[si] = 0;
  nodes[nodeCount].battery = 0;
  nodes[nodeCount].lastSync = 0;
  for (int si = 0; si < MAX_POTS_PER_DEVICE; ++si) nodes[nodeCount].lastWater[si] = 0;
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

  // Parse compact packet header (CompanyID already stripped by common scan cb)
  uint8_t compactTarget[6]; uint16_t nonce; const uint8_t* payload_ptr; size_t payload_len;
  if (!parse_compact_packet_header(data, len, compactTarget, nonce, payload_ptr, payload_len)) return;
  if (memcmp(compactTarget, thisMac, 6) != 0 && memcmp(compactTarget, BROADCAST_MAC, 6) != 0) return;

  uint8_t srcMac[6]; extract_src_mac(adv, srcMac);
  LOG("[<-] (compact) Received advert from worker MAC: %02X:%02X:%02X:%02X:%02X:%02X", srcMac[0], srcMac[1], srcMac[2], srcMac[3], srcMac[4], srcMac[5]);

  // ACK handling: payload begins with MsgType
  if (payload_len >= 1 && payload_ptr[0] == BT_TLV::TYPE_ACK) {
    LOG("[v] [%d] Received ACK (compact) from worker", nonce);
    BT_TLV::btCommonMarkAck(nonce);
    onCommandAcked(srcMac, nonce);
    return;
  }

  // Queue ACK back to sender for any non-ACK compact messages
  BT_TLV::btCommonQueueAck(srcMac, nonce);
  LOG("[->] [%d] Sent ACK to worker (compact)", nonce);

  uint8_t updateResult = btMainSetNodeLastNonce(srcMac, nonce);
  if (updateResult == 0) { LOG("[x] [%d] Skip not added node", nonce); return; }
  else if (updateResult == 2) { LOG("[x] [%d] Skip duplication", nonce); return; }
  LOG("[<-] [%d] Received compact status/command from worker", nonce);

  // Handle STATUS payload (TYPE_STATUS)
  if (payload_len >= 1 && payload_ptr[0] == BT_TLV::TYPE_STATUS) {
    // layout: [TYPE_STATUS][Battery(1)][PotCount(1)][Soil1(2), Soil2(2), ...]
    if (payload_len >= 3) {
      uint8_t batt = payload_ptr[1];
      uint8_t potCount = payload_ptr[2];
      size_t soilsBytes = (payload_len - 3);
      size_t soilsCount = soilsBytes / 2;
      size_t toCopy = (potCount < soilsCount) ? potCount : soilsCount;
      LOG("[<-] [%d] Compact STATUS batt=%d pots=%d soils=%d", nonce, batt, potCount, toCopy);
      // Update node entry with potCount, battery, and soils (up to MAX_POTS_PER_DEVICE)
      WorkerNode* node = nullptr;
      for (int i=0;i<nodeCount;i++) if (memcmp(nodes[i].mac, srcMac, 6)==0) { node = &nodes[i]; break; }
      // Build soils array and delegate update to updateNode
      uint16_t tmpSoils[MAX_POTS_PER_DEVICE];
      for (size_t s = 0; s < (size_t)potCount && s < MAX_POTS_PER_DEVICE; ++s) {
        if (s < toCopy) tmpSoils[s] = (uint16_t(payload_ptr[3 + s*2]) << 8) | uint16_t(payload_ptr[3 + s*2 + 1]);
        else tmpSoils[s] = 0;
      }
      uint8_t useCount = (uint8_t)min((size_t)potCount, (size_t)MAX_POTS_PER_DEVICE);
      updateNode(srcMac, tmpSoils, useCount, batt);

      if (state == SLEEPING) {
        unsigned long now_s = millis() / 1000;
        uint32_t sleepSec = calculateSleepSec(now_s);
        LOG("Entering SLEEPING state, calculating sleep duration: %d seconds", sleepSec);

        uint8_t payload[5];
        payload[0] = BT_TLV::TYPE_CMD_SLEEP; // MsgType
        // encode big-endian uint32 seconds
        payload[1] = (uint8_t)((sleepSec >> 24) & 0xFF);
        payload[2] = (uint8_t)((sleepSec >> 16) & 0xFF);
        payload[3] = (uint8_t)((sleepSec >> 8) & 0xFF);
        payload[4] = (uint8_t)(sleepSec & 0xFF);
        LOG("[->] Queuing CMD_SLEEP command to worker %02X:%02X:%02X:%02X:%02X:%02X", srcMac[0], srcMac[1], srcMac[2], srcMac[3], srcMac[4], srcMac[5]);
        btMainQueueCommand(srcMac, payload, sizeof(payload), 2, 700);
        return;
      }
    }
    return;
  }

  // Handle command-completion notices (worker may echo CMD_WATER as completion)
  if (payload_len >= 1 && payload_ptr[0] == BT_TLV::TYPE_CMD_WATER) {
    LOG("[<-] [%d] Received CMD_WATER completion (compact) from worker", nonce);
    onWorkerCompleted(srcMac);
    return;
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

void btMainSetNodeLastWater(const uint8_t mac[6], uint16_t potMask, unsigned long bootSeconds) {
  for (int i=0;i<nodeCount;i++) {
    if (memcmp(nodes[i].mac, mac, 6)==0) {
      for (int p = 0; p < MAX_POTS_PER_DEVICE; ++p) {
        if (potMask & (1u << p)) nodes[i].lastWater[p] = bootSeconds;
      }
      break;
    }
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
