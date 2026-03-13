#pragma once
#include <vector>
#include <Arduino.h>
#include "config.h"

void startWatering(const std::vector<const WorkerConfig*>& targets);

// Callbacks invoked by bluetooth main when commands are sent/acked/completed
void onCommandSent(const uint8_t mac[6], uint16_t nonce);
void onCommandAcked(const uint8_t mac[6], uint16_t nonce);
void onWorkerCompleted(const uint8_t mac[6]);

