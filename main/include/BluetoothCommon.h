#pragma once
#include <stdint.h>
#include <stddef.h>
#include <vector>
#include <NimBLEDevice.h>

// TLV types used across main and worker
namespace BT_TLV {
  constexpr uint8_t TYPE_MAC = 0x01;      // 6 bytes
  constexpr uint8_t TYPE_SOIL = 0x02;     // 2 bytes (big-endian)
  constexpr uint8_t TYPE_BATT = 0x03;     // 1 byte
  constexpr uint8_t TYPE_TARGET = 0x12;   // 6 bytes - target MAC in a command
  constexpr uint8_t TYPE_NONCE = 0x11;    // 2 bytes nonce
  constexpr uint8_t TYPE_PAYLOAD = 0x14;  // n bytes blob
  constexpr uint8_t TYPE_ACK = 0x20;      // 2 bytes nonce ack
}

// Shared scan callback wiring
namespace BT_TLV {
  using AdvertHandler = void(*)(const uint8_t* data, size_t len, const NimBLEAdvertisedDevice* adv);
  extern AdvertHandler gAdvertHandler;
  void btCommonSetAdvertHandler(AdvertHandler h);

  // Shared broadcast helper (used by main and worker)
  void btCommonBroadcast(const uint8_t* payload, size_t len, unsigned msDelay=100);
  // Install common scan callbacks on the global NimBLE scanner
  void btCommonInstallScanCallbacks();
}

// Command IDs for TYPE_CMD (1-byte payload when present)
namespace BT_CMD {
  constexpr uint8_t CMD_PROBE = 0x01; // ask worker to broadcast status immediately
  constexpr uint8_t CMD_SYNC  = 0x02; // set next sync/wake time or delay
  constexpr uint8_t CMD_WATER = 0x03; // instruct worker to open valve
}

/*
 TLV envelope and usage notes (see DESIGN.md for a fuller explanation):

 - Advertisement envelope: use BLE Advertisement "Manufacturer Specific Data" (AD type 0xFF).
   Prepend a 2-byte Company Identifier (little-endian). You may use 0xFFFF for local/test usage.

 - TLV format: each field is [type:1][len:1][value:len]. Multi-byte integers are big-endian on the wire.

 - Size: advertisement payload (CompanyID + TLV bytes) must be <= 31 bytes. Use scan-response or multiple adverts
   only when necessary.

 - Recommended TLV fields (already defined above):
   TYPE_MAC (0x01)  : 6 bytes, worker MAC (used in worker -> main status adverts)
   TYPE_SOIL (0x02) : 2 bytes, soil ADC (uint16, 0..4095)
   TYPE_BATT (0x03) : 1 byte, battery %
   TYPE_TARGET (0x12): 6 bytes, target MAC in a command
   TYPE_NONCE (0x11): 2 bytes nonce (uint16) to correlate requests and ACKs
   TYPE_PAYLOAD (0x14): variable command payload
   TYPE_ACK (0x20)  : 2 bytes nonce used by worker to ACK commands

 - Command envelope (main -> worker): always include TYPE_TARGET and TYPE_NONCE. Then include either
   TYPE_CMD (1 byte via TYPE_PAYLOAD or a dedicated byte inside TYPE_PAYLOAD) or a structured payload.

 - ACK flow: worker replies by broadcasting an advertisement containing TYPE_ACK with the nonce.
   The main marks the command acknowledged on receipt of matching nonce.

*/

void extract_src_mac(const NimBLEAdvertisedDevice* adv, uint8_t out_mac[6]);
// Helpers for parsing TLV blobs (non-owning). Return true if field found.
bool tlv_extract_tgt_mac(const uint8_t* data, size_t len, uint8_t out_mac[6]);
bool tlv_extract_soil(const uint8_t* data, size_t len, uint16_t &soil);
bool tlv_extract_batt(const uint8_t* data, size_t len, uint8_t &batt);
bool tlv_extract_nonce(const uint8_t* data, size_t len, uint16_t &nonce);
// Extract TYPE_ACK nonce
bool tlv_extract_ack(const uint8_t* data, size_t len, uint16_t &nonce);
// Get payload blob pointer and length if present
bool tlv_extract_payload(const uint8_t* data, size_t len, const uint8_t** out_ptr, size_t &out_len);

// Helpers to build common TLV packets (return vector owning bytes)
std::vector<uint8_t> tlv_make_ack(const uint8_t mac[6], uint16_t nonce);
std::vector<uint8_t> tlv_make_status(const uint8_t mac[6], uint16_t soil, uint8_t batt);
std::vector<uint8_t> tlv_make_command(const uint8_t target_mac[6], uint16_t nonce, const uint8_t* payload, size_t payload_len);
