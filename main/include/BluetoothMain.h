#pragma once

#include "BluetoothCommon.h"
#include "Utility.h"
#include <Arduino.h>

struct WorkerNode {
  uint8_t mac[6];
  uint8_t potCount;
  uint16_t soils[MAX_POTS_PER_DEVICE];
  uint16_t batteryMv;
  int16_t lastRssi;
  int64_t lastSyncUs;
  int64_t lastWaterUs[MAX_POTS_PER_DEVICE];
  uint32_t statusGeneration;
  BtMessageId lastMessageId;
};

void btMainBegin();
BtSendResult btMainSendCommand(const uint8_t targetMac[6],
                               const uint8_t* payload, size_t len,
                               int retries, uint32_t timeoutMs,
                               BtBeforeTransmitCb beforeTransmit = nullptr,
                               void* context = nullptr);

int btMainNodeCount();
bool btMainGetNodeAt(int index, WorkerNode& out);
bool btMainGetNodeByMac(const uint8_t mac[6], WorkerNode& out);
bool btMainWaitForStatus(const uint8_t mac[6], uint32_t previousGeneration,
                         uint32_t timeoutMs, WorkerNode& out);
void btMainMarkNodeWatered(const uint8_t mac[6], uint16_t potMask);
bool btMainTakeSleepRequest(uint8_t mac[6]);
void btMainEnsureNodeExists(const uint8_t mac[6]);
void btMainRemoveNodeByMac(const uint8_t mac[6]);
