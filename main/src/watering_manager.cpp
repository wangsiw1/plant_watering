#include "WateringManager.h"
#include "Sensor.h"
#include "BluetoothMain.h"
#include "config.h"


static uint16_t currentNonce = 0;
static uint8_t currentMac[6] = {0,0,0,0,0,0};
static unsigned long now_s = millis() / 1000;
static uint16_t currentPotMask = 0;
const uint8_t zeroMac[6] = {0,0,0,0,0,0};

void startWatering(const std::vector<const WorkerConfig*>& toWater) {
  // Group targets by device MAC and send per-pot durations (sequential watering on worker)
  struct MacGroup { uint8_t mac[6]; WorkerConfig entries[MAX_WORKER_COUNT]; int entryCount; };
  struct MacGroup groups[MAX_WORKER_COUNT];
  int groupCount = 0;
  for (const WorkerConfig* wc : toWater) {
    bool found = false;
    for (int gi = 0; gi < groupCount; ++gi) {
      if (memcmp(groups[gi].mac, wc->mac, 6) == 0) {
        groups[gi].entries[groups[gi].entryCount++] = *wc;
        found = true;
        break;
      }
    }
    if (!found) {
      if (groupCount < MAX_WORKER_COUNT) {
        MacGroup g;
        memcpy(g.mac, wc->mac, 6);
        g.entries[0] = *wc;
        g.entryCount = 1;
        groups[groupCount++] = g;
      }
    }
  }

  for (int gi = 0; gi < groupCount; ++gi) {
    const MacGroup& g = groups[gi];
    uint16_t potMask = 0;
    // Get potCount from WorkerNode for this MAC
    const WorkerNode* node = btMainFindNodeByMac(g.mac);
    // build durations in increasing pot-index order for bits set in potMask
    uint16_t durations[node->potCount];
    int durCount = 0;
    for (int pi = 0; pi < node->potCount; ++pi) {
      for (int ei = 0; ei < g.entryCount; ++ei) {
        if ((int)g.entries[ei].potIndex == pi) {
          potMask |= (uint16_t)(1u << pi);
          durations[durCount++] = g.entries[ei].duration;
          break;
        }
      }
    }

    int tank = getTankLevel();
    // Only allow watering when tank sensor indicates HIGH (analog > 2800)
    if (tank <= 2800) break;

    // build compact CMD_WATER payload: [TYPE][potMask(2)][duration1(2), duration2(2), ...]
    size_t payload_len = 1 + 2 + durCount * 2;
    uint8_t payload[5 + durCount * 2];
    payload[0] = BT_TLV::TYPE_CMD_WATER;
    payload[1] = (uint8_t)((potMask >> 8) & 0xFF);
    payload[2] = (uint8_t)(potMask & 0xFF);
    for (int di = 0; di < durCount; ++di) {
      uint16_t dv = durations[di];
      payload[3 + di*2] = (uint8_t)((dv >> 8) & 0xFF);
      payload[3 + di*2 + 1] = (uint8_t)(dv & 0xFF);
    }

    bool queued = btMainQueueCommand(g.mac, payload, payload_len, 2, 700);
    if (queued) {
      uint32_t bootSec = millis() / 1000;
      btMainSetNodeLastWater(g.mac, potMask, bootSec);
      currentPotMask = potMask;
    } else currentPotMask = 0;

    currentNonce = 0;
    now_s = millis() / 1000;
    memcpy(currentMac, g.mac, 6);

    // Wait until worker completed or timeout (sum of durations + small margin)
    unsigned long totalDur = 0;
    for (int di = 0; di < durCount; ++di) totalDur += durations[di];
    const unsigned long margin = 5;
    while ((((millis()/1000) - now_s) <= (totalDur + margin)) && memcmp(currentMac, zeroMac, 6) != 0) {
      tank = getTankLevel();
      if (tank <= 2800) break;
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

void onCommandSent(const uint8_t mac[6], uint16_t nonce) {
  // record nonce for active entries matching mac
  if (memcmp(currentMac, mac, 6) == 0) {
    currentNonce = nonce;
    now_s = millis() / 1000;
  }
}

void onCommandAcked(const uint8_t mac[6], uint16_t nonce) {
  if (memcmp(currentMac, mac, 6) == 0 && currentNonce == nonce) {
    currentNonce = 0;
    now_s = millis() / 1000;
  }
}

void onWorkerCompleted(const uint8_t mac[6]) {
  if (memcmp(currentMac, mac, 6) == 0) {
    memcpy(currentMac, zeroMac, 6);
    btMainSetNodeLastWater(mac, currentPotMask, now_s);
    currentPotMask = 0;
  }
}
