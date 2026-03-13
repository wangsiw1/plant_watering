#include "BluetoothCommon.h"
// #include "Utility.h"
#include <NimBLEDevice.h>
#include <vector>
#include <cstring>

using namespace BT_TLV;

// Worker-side behavior:
// - Periodically advertise status TLV (mac, soil, batt)
// - Listen for commands (advertisements) that contain TYPE_TARGET matching own MAC
// - When command received and targeted, extract nonce and payload, send ACK TLV with nonce,
//   then (placeholder) perform action (user should replace with actual valve control)

static uint8_t thisMac[6];
// remember the one main controller MAC we will accept commands from, store in RTC memory to maintain after deep sleep
static RTC_DATA_ATTR uint8_t mainMac[6] = {0,0,0,0,0,0};

bool mainMacIsSet() { for (int i=0;i<6;i++) if (mainMac[i]!=0) return true; return false; }

// Worker advert parsing (moved into a dedicated parseAdvert)
static void parseAdvert(const uint8_t* data, size_t len, const NimBLEAdvertisedDevice* adv) {
  if (!data) return;

  uint8_t srcMac[6];
  extract_src_mac(adv, srcMac);
  const uint8_t* payptr=nullptr; size_t paylen=0; bool hasPayload = tlv_extract_payload(data, len, &payptr, paylen);

  // Check TARGET (TYPE_TARGET in TLV is always the intended target MAC)
  uint8_t tgtMac[6]; bool hasTgt=false;
  size_t off=0; while (off+2 <= len) {
    uint8_t t = data[off]; uint8_t l = data[off+1]; if (off+2+l>len) break; const uint8_t* p = &data[off+2];
    if (t == TYPE_TARGET && l>=6) { memcpy(tgtMac,p,6); hasTgt=true; }
    if (t == TYPE_NONCE && l>=2) {
      uint16_t n = (uint16_t(p[0])<<8)|uint16_t(p[1]);
      if (hasTgt && memcmp(tgtMac, thisMac, 6)==0) {
        // learn main MAC on first PROBE command (use advertiser address from adv)
        if (!mainMacIsSet() && hasPayload && paylen>=1 && payptr[0]==BT_CMD::CMD_PROBE) {
          memcpy(mainMac, srcMac, 6);
        }
        // if mainMac is set, only accept commands from that advertiser address
        if (mainMacIsSet()) {
          if (memcmp(srcMac, mainMac, 6) != 0) { off += 2 + l; continue; }
        }
        // send ACK (include our MAC to scope the ACK to this system)
        auto ack = tlv_make_ack(mainMac, n);
        BT_TLV::btCommonBroadcast(ack.data(), ack.size(), 80);
        // payload handling could be added here (extract TYPE_PAYLOAD)
      }
    }
    off += 2 + l;
  }
}

void btWorkerBegin() {
  // capture this MAC
  esp_read_mac(thisMac, ESP_MAC_BT);
  NimBLEDevice::init("plant-worker");
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);
  NimBLEScan* scan = NimBLEDevice::getScan();
  // set our advert handler and install common scan callbacks
  BT_TLV::btCommonSetAdvertHandler([](const uint8_t* d, size_t l, const NimBLEAdvertisedDevice* adv){ parseAdvert(d,l,adv); });
  BT_TLV::btCommonInstallScanCallbacks();
  scan->setInterval(45);
  scan->setWindow(15);
  scan->setActiveScan(true);
  scan->start(0);
}

void btWorkerAdvertiseStatus(uint16_t soil, uint8_t batt) {
  auto pkt = tlv_make_status(mainMac, soil, batt);
  BT_TLV::btCommonBroadcast(pkt.data(), pkt.size(), 80);
}
