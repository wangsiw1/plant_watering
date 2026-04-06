#pragma once
#include "BluetoothCommon.h"
#include <Arduino.h>

struct WorkerNode {
  uint8_t mac[6];
  uint16_t soil;
  uint8_t battery;
  unsigned long lastSync;
  unsigned long lastWater; // epoch seconds when main commanded this worker to water
  uint16_t lastNonce;
};

void btMainBegin();
// Non-blocking: enqueue a command for the sender task. Returns true if queued.
bool btMainQueueCommand(const uint8_t target_mac[6], const uint8_t* payload, size_t len, int retries, unsigned timeoutMs);
int btMainNodeCount();
const WorkerNode* btMainNodeAt(int idx);
const WorkerNode* btMainFindNodeByMac(const uint8_t mac[6]);

// Update lastWater epoch for a discovered node (seconds since epoch)
void btMainSetNodeLastWater(const uint8_t mac[6], unsigned long epochSeconds);
uint8_t btMainSetNodeLastNonce(const uint8_t mac[6], unsigned long nonce);
// Ensure node entry exists in discovery cache (adds placeholder if missing)
void btMainEnsureNodeExists(const uint8_t mac[6]);
// Remove a discovered node from the cache by MAC
void btMainRemoveNodeByMac(const uint8_t mac[6]);
// Remove nodes that are not present in configured workerList (periodic cleanup)
void btMainCleanupOrphanedNodes();
