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
    // TYPE_TARGET removed: use TYPE_MAC (0x01) to carry target MAC in command TLVs
  constexpr uint8_t TYPE_NONCE = 0x11;    // 2 bytes nonce
  // TYPE_PAYLOAD removed: command fields are now TLV fields themselves
  constexpr uint8_t TYPE_ACK = 0x20;      // 2 bytes nonce ack

  // Command TLV types (merged from previous BT_CMD namespace) - choose values
  // outside the sensor/status namespace to avoid collisions.
  constexpr uint8_t TYPE_CMD_PROBE = 0x30; // probe request (len=0)
  constexpr uint8_t TYPE_CMD_SLEEP = 0x31; // sleep request (len=4: seconds)
  constexpr uint8_t TYPE_CMD_WATER = 0x32; // water request (len=2: duration seconds)
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

  // Command sender/queue shared between main and worker
  using OnCommandSentCb = void(*)(const uint8_t mac[6], uint16_t nonce);

  // Initialize the common sender (creates queue and sender task). Safe to call multiple times.
  bool btCommonInitSender();

  // Queue a command for sending. Returns true if queued.
  bool btCommonQueueCommand(const uint8_t target_mac[6], const uint8_t* payload, size_t len, int retries, unsigned timeoutMs);
  
  // Queue a ACK for sending. Returns true if queued.
  bool btCommonQueueAck(const uint8_t target_mac[6], uint16_t nonce);

  // Mark a nonce as acknowledged (called when an ACK TLV observed)
  void btCommonMarkAck(uint16_t nonce);

  // Register callback invoked when a command is actually sent (nonce allocated). Optional.
  void btCommonRegisterOnCommandSent(OnCommandSentCb cb);
}

// (BT_CMD removed; commands are TLV fields under BT_TLV)

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
  // - TARGET is carried using TYPE_MAC (0x01): 6 bytes, target MAC in a command
  TYPE_NONCE (0x11): 2 bytes nonce (uint16) to correlate requests and ACKs
  (legacy TYPE_PAYLOAD removed) — commands are TLV fields themselves
  TYPE_ACK (0x20)  : 2 bytes nonce used by worker to ACK commands

 - Command envelope (main -> worker): include TYPE_MAC (target) and TYPE_NONCE. Then include either
  TYPE_CMD (1 byte via TYPE_PAYLOAD or a dedicated byte inside TYPE_PAYLOAD) or a structured payload.

 - ACK flow: worker replies by broadcasting an advertisement containing TYPE_ACK with the nonce.
   The main marks the command acknowledged on receipt of matching nonce.

*/

const uint8_t BROADCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

void extract_src_mac(const NimBLEAdvertisedDevice* adv, uint8_t out_mac[6]);
// Helpers for parsing TLV blobs (non-owning). Return true if field found.
bool tlv_extract_tgt_mac(const uint8_t* data, size_t len, uint8_t out_mac[6]);
bool tlv_extract_soil(const uint8_t* data, size_t len, uint16_t &soil);
bool tlv_extract_batt(const uint8_t* data, size_t len, uint8_t &batt);
bool tlv_extract_nonce(const uint8_t* data, size_t len, uint16_t &nonce);
// Extract TYPE_ACK nonce
bool tlv_extract_ack(const uint8_t* data, size_t len, uint16_t &nonce);
// Get payload blob pointer and length if present
// (tlv_extract_payload removed) — command TLVs should be parsed by scanning TLVs.

// Helpers to build common TLV packets
size_t tlv_make_command(const uint8_t target_mac[6], uint16_t nonce, const uint8_t* payload, size_t payload_len, uint8_t* out, bool is_ack);

// Helpers to extract command TLV fields (return true if found)
bool tlv_extract_cmd_probe(const uint8_t* data, size_t len);
bool tlv_extract_cmd_water(const uint8_t* data, size_t len, uint16_t &duration);
bool tlv_extract_cmd_sleep(const uint8_t* data, size_t len, uint32_t &delay_s);
