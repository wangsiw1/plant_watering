#include "BluetoothCommon.h"
#include <NimBLEDevice.h>
#include <vector>
#include <cstring>

using namespace BT_TLV;

// Worker-side behavior:
// - Periodically advertise status TLV (mac, soil, batt)
// - Listen for commands (advertisements) that contain TYPE_TARGET matching own MAC
// - When command received and targeted, extract nonce and payload, send ACK TLV with nonce,
//   then (placeholder) perform action (user should replace with actual valve control)

static uint8_t ourMac[6];

class WorkerScanCb : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* adv) override {
    std::string m = adv->getManufacturerData();
    const uint8_t* ptr = nullptr; size_t plen = 0;
    if (!m.empty()) { ptr = (const uint8_t*)m.data(); plen = m.size(); }
    else { std::string s = adv->getServiceData(); if (!s.empty()) { ptr = (const uint8_t*)s.data(); plen = s.size(); } }
    if (!ptr) return;
    // Check TARGET
    uint8_t tgt[6]; bool hasTgt=false;
    // scan for TYPE_TARGET
    size_t off=0; while (off+2 <= plen) {
      uint8_t t = ptr[off]; uint8_t l = ptr[off+1]; if (off+2+l>plen) break; const uint8_t* p = &ptr[off+2];
      if (t == TYPE_TARGET && l>=6) { memcpy(tgt,p,6); hasTgt=true; }
      if (t == TYPE_NONCE && l>=2) { uint16_t n = (uint16_t(p[0])<<8)|uint16_t(p[1]);
        if (hasTgt && memcmp(tgt, ourMac, 6)==0) {
          // send ACK
          auto ack = tlv_make_ack(n);
          NimBLEAdvertising* advb = NimBLEDevice::getAdvertising(); NimBLEAdvertisementData ad; std::string ms((const char*)ack.data(), ack.size()); ad.setManufacturerData(ms); advb->setAdvertisementData(ad); advb->start(); delay(80); advb->stop();
          // payload handling could be added here (extract TYPE_PAYLOAD)
        }
      }
      off += 2 + l;
    }
  }
};

void btWorkerBegin() {
  // capture our MAC
  esp_read_mac(ourMac, ESP_MAC_BT);
  NimBLEDevice::init("plant-worker");
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new WorkerScanCb());
  scan->setInterval(45); scan->setWindow(15); scan->setActiveScan(true);
  scan->start(0);
}

void btWorkerAdvertiseStatus(uint16_t soil, uint8_t batt) {
  auto pkt = tlv_make_status(ourMac, soil, batt);
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising(); NimBLEAdvertisementData ad; std::string m((const char*)pkt.data(), pkt.size()); ad.setManufacturerData(m); adv->setAdvertisementData(ad); adv->start(); delay(80); adv->stop();
}
