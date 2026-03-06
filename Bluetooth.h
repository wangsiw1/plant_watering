#pragma once
#include "BluetoothMain.h"

// Backwards-compatible wrappers (call main-node APIs)
inline void btBegin() { btMainBegin(); }
inline void btBroadcastCommand(const uint8_t mac[6], const uint8_t* payload, size_t len) { btMainBroadcast(payload, len); (void)mac; }
inline int btNodeCount() { return btMainNodeCount(); }
inline const WorkerNode* btNodeAt(int idx) { return btMainNodeAt(idx); }
