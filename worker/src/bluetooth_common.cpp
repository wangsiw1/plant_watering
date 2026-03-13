#include "BluetoothCommon.h"
#include "Utility.h"
#include <cstring>
#include <NimBLEDevice.h>

using namespace BT_TLV;

/*
  Bluetooth TLV payload specification (shared common helpers)

  Envelope:
  - Use BLE Advertisement Manufacturer Specific Data (AD type 0xFF).
  - Start the AD payload with 2 bytes Company Identifier (little-endian). Use 0xFFFF for local/test.
  - Follow with TLV bytes described below.

  TLV format:
  - Each field is: 1 byte Type, 1 byte Length, <Length> bytes Value.
  - Multi-byte integers are transmitted big-endian (network order) inside TLV values.
  - Keep (CompanyID + TLV bytes) <= 31 bytes to fit in a single adv packet.

  Important TLV types (see BluetoothCommon.h for constants):
  - TYPE_MAC (0x01): 6 bytes, worker MAC.
  - TYPE_SOIL (0x02): 2 bytes, soil ADC value (uint16).
  - TYPE_BATT (0x03): 1 byte battery percentage.
  - TYPE_TARGET (0x12): 6 bytes target MAC in main->worker commands.
  - TYPE_NONCE (0x11): 2 bytes nonce to correlate requests and ACKs.
  - TYPE_PAYLOAD (0x14): n bytes of command-specific payload.
  - TYPE_ACK (0x20): 2 bytes nonce — worker acknowledges command by advertising this TLV.

  Example command (water for 10s):
    CompanyID(2) +
    TYPE_TARGET(1) LEN(1) MAC(6) +
    TYPE_NONCE(1) LEN(1) NONCE(2) +
    TYPE_PAYLOAD(1) LEN(1) [CMD_WATER + duration(2)]

  ACK semantics:
  - Worker should immediately advertise TYPE_ACK with the same nonce when it accepts a command.
  - Main retransmits the command (same nonce) on timeout. Worker should tolerate duplicates and
    ignore repeated commands already handled.

*/

static bool _parse_tlv_once(const uint8_t* data, size_t len, uint8_t &type, const uint8_t*& ptr, uint8_t &l, size_t &offset) {
  if (offset + 2 > len) return false;
  type = data[offset];
  l = data[offset+1];
  if (offset + 2 + l > len) return false;
  ptr = &data[offset+2];
  offset += 2 + l;
  return true;
}

void extract_src_mac(const NimBLEAdvertisedDevice* adv, uint8_t src_mac[6]) {
  // read advertiser address (sender) from the advertisement metadata
  std::string advAddr = adv->getAddress().toString();
  uint8_t advMacBuf[6];
  macFromHexString(advAddr.c_str(), src_mac);
}

bool tlv_extract_tgt_mac(const uint8_t* data, size_t len, uint8_t tgt_mac[6]) {
  size_t off = 0; uint8_t type,l; const uint8_t* ptr;
  while (_parse_tlv_once(data,len,type,ptr,l,off)) {
    if (type == TYPE_MAC && l>=6) { memcpy(tgt_mac, ptr, 6); return true; }
  }
  return false;
}

bool tlv_extract_soil(const uint8_t* data, size_t len, uint16_t &soil) {
  size_t off = 0; uint8_t type,l; const uint8_t* ptr;
  while (_parse_tlv_once(data,len,type,ptr,l,off)) {
    if (type == TYPE_SOIL && l>=2) { soil = (uint16_t(ptr[0])<<8) | uint16_t(ptr[1]); return true; }
  }
  return false;
}

bool tlv_extract_batt(const uint8_t* data, size_t len, uint8_t &batt) {
  size_t off = 0; uint8_t type,l; const uint8_t* ptr;
  while (_parse_tlv_once(data,len,type,ptr,l,off)) {
    if (type == TYPE_BATT && l>=1) { batt = ptr[0]; return true; }
  }
  return false;
}

bool tlv_extract_nonce(const uint8_t* data, size_t len, uint16_t &nonce) {
  size_t off = 0; uint8_t type,l; const uint8_t* ptr;
  while (_parse_tlv_once(data,len,type,ptr,l,off)) {
    if (type == TYPE_NONCE && l>=2) { nonce = (uint16_t(ptr[0])<<8) | uint16_t(ptr[1]); return true; }
  }
  return false;
}

bool tlv_extract_payload(const uint8_t* data, size_t len, const uint8_t** out_ptr, size_t &out_len) {
  size_t off = 0; uint8_t type,l; const uint8_t* ptr;
  while (_parse_tlv_once(data,len,type,ptr,l,off)) {
    if (type == TYPE_PAYLOAD) { *out_ptr = ptr; out_len = l; return true; }
  }
  return false;
}

bool tlv_extract_ack(const uint8_t* data, size_t len, uint16_t &nonce) {
  size_t off = 0; uint8_t type,l; const uint8_t* ptr;
  while (_parse_tlv_once(data,len,type,ptr,l,off)) {
    if (type == TYPE_ACK && l>=2) { nonce = (uint16_t(ptr[0])<<8) | uint16_t(ptr[1]); return true; }
  }
  return false;
}

// Build an ACK TLV that includes a MAC (TYPE_MAC) followed by TYPE_ACK (nonce)
std::vector<uint8_t> tlv_make_ack(const uint8_t tgt_mac[6], uint16_t nonce) {
  std::vector<uint8_t> v;
  v.push_back(TYPE_MAC); v.push_back(6);
  for (int i=0;i<6;i++) v.push_back(tgt_mac[i]);
  v.push_back(TYPE_ACK); v.push_back(2);
  v.push_back((uint8_t)(nonce>>8));
  v.push_back((uint8_t)(nonce & 0xFF));
  return v;
}

std::vector<uint8_t> tlv_make_status(const uint8_t tgt_mac[6], uint16_t soil, uint8_t batt) {
  std::vector<uint8_t> v;
  v.push_back(TYPE_MAC); v.push_back(6);
  for (int i=0;i<6;i++) v.push_back(tgt_mac[i]);
  v.push_back(TYPE_SOIL); v.push_back(2);
  v.push_back((uint8_t)(soil>>8)); v.push_back((uint8_t)(soil&0xFF));
  v.push_back(TYPE_BATT); v.push_back(1);
  v.push_back(batt);
  return v;
}

std::vector<uint8_t> tlv_make_command(const uint8_t target_mac[6], uint16_t nonce, const uint8_t* payload, size_t payload_len) {
  std::vector<uint8_t> v;
  v.push_back(TYPE_TARGET); v.push_back(6);
  for (int i=0;i<6;i++) v.push_back(target_mac[i]);
  v.push_back(TYPE_NONCE); v.push_back(2);
  v.push_back((uint8_t)(nonce>>8)); v.push_back((uint8_t)(nonce&0xFF));
  v.push_back(TYPE_PAYLOAD); v.push_back((uint8_t)payload_len);
  for (size_t i=0;i<payload_len;i++) v.push_back(payload[i]);
  return v;
}

// Shared scan callback implementation and wiring
namespace BT_TLV {
  AdvertHandler gAdvertHandler = nullptr;
  void btCommonSetAdvertHandler(AdvertHandler h) { gAdvertHandler = h; }

  void btCommonBroadcast(const uint8_t* payload, size_t len, unsigned msDelay) {
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    NimBLEAdvertisementData ad;
    std::string m((const char*)payload, len);
    ad.setManufacturerData(m);
    adv->setAdvertisementData(ad);
    adv->start();
    delay(msDelay);
    adv->stop();
  }

  class CommonScanCb : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* adv) override {
      if (!gAdvertHandler) return;
      std::string m = adv->getManufacturerData();
      if (!m.empty()) { gAdvertHandler((const uint8_t*)m.data(), m.size(), adv); return; }
      std::string s = adv->getServiceData(); 
      if (!s.empty()) { gAdvertHandler((const uint8_t*)s.data(), s.size(), adv); }
    }
  };

  // helper to attach common scan cb to the global scanner
  void btCommonInstallScanCallbacks() {
    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(new CommonScanCb());
  }

}
