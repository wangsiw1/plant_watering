#pragma once
#include "BluetoothCommon.h"
#include "Utility.h"
#include <Arduino.h>

struct WorkerNode {
  uint8_t mac[6];
  uint8_t potCount;                    // number of pots reported by this device
  uint16_t soils[MAX_POTS_PER_DEVICE]; // per-pot soil ADC readings (big-endian on the wire)
  uint8_t battery;
  unsigned long lastSync;
  // Per-pot last-water timestamps stored as boot-relative seconds (millis()/1000)
  unsigned long lastWater[MAX_POTS_PER_DEVICE];
  uint16_t lastNonce;
};

void btMainBegin();
// Non-blocking: enqueue a command for the sender task. Returns true if queued.
bool btMainQueueCommand(const uint8_t target_mac[6], const uint8_t* payload, size_t len, int retries, unsigned timeoutMs);
int btMainNodeCount();
const WorkerNode* btMainNodeAt(int idx);
const WorkerNode* btMainFindNodeByMac(const uint8_t mac[6]);

// Update per-pot lastWater boot-seconds for a discovered node
void btMainSetNodeLastWater(const uint8_t mac[6], uint16_t potMask, unsigned long bootSeconds);
uint8_t btMainSetNodeLastNonce(const uint8_t mac[6], unsigned long nonce);
// Ensure node entry exists in discovery cache (adds placeholder if missing)
void btMainEnsureNodeExists(const uint8_t mac[6]);
// Remove a discovered node from the cache by MAC
void btMainRemoveNodeByMac(const uint8_t mac[6]);
// Remove nodes that are not present in configured workerList (periodic cleanup)
void btMainCleanupOrphanedNodes();
