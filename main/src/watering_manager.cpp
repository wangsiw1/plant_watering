#include "WateringManager.h"
#include "Sensor.h"
#include "BluetoothMain.h"
#include "config.h"


static uint16_t currentNonce = 0;
static uint8_t currentMac[6] = {0,0,0,0,0,0};
static unsigned long now_s = millis() / 1000;
const uint8_t zeroMac[6] = {0,0,0,0,0,0};

void startWatering(const std::vector<const WorkerConfig*>& toWater) {
  for (auto w : toWater) {
    int tank = getTankLevel();
    if (tank < 500) {
      break;
    }

    uint16_t dur = w->duration;
    uint8_t payload[4];
    payload[0] = BT_TLV::TYPE_CMD_WATER; payload[1] = 2; // TLV: type + len
    payload[2] = (uint8_t)(dur >> 8);
    payload[3] = (uint8_t)(dur & 0xFF);
    btMainQueueCommand(w->mac, payload, sizeof(payload), 2, 700);

    currentNonce = 0;
    now_s = millis() / 1000;
    memcpy(currentMac, w->mac, 6);

    // Wait until timeout or current node done
    while (
      ((millis()/1000)-now_s) <= (dur+1) || 
      memcmp(currentMac, zeroMac, 6) == 0
    ) {
      tank = getTankLevel();
      if (tank < 500) {
        break;
      }
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
    btMainSetNodeLastWater(mac, now_s);
  }
}
