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
  constexpr uint8_t TYPE_STATUS = 0x40;   // compact STATUS msg (worker -> main)
  constexpr uint8_t TYPE_CONFIG = 0x41;   // unpaired worker announces potCount
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

// Legacy TLV parsing helpers were removed. The codebase now uses the
// compact packet format (CompanyID + TargetMAC(6) + Nonce(2) + payload...).

size_t make_compact_packet(const uint8_t target_mac[6], uint16_t nonce, const uint8_t* payload, size_t payload_len, uint8_t* out, bool encrypt);

// Compact packet parsing helpers (new format: CompanyID + TargetMAC(6) + Nonce(2) + Payload...)
// Parse a compact packet (data starts AFTER CompanyID). On success returns true and fills
// `out_target_mac`, `nonce` and returns pointer/len to payload (payload begins with msgType byte).
bool parse_compact_packet_header(const uint8_t* data, size_t len, uint8_t out_target_mac[6], uint16_t &nonce, const uint8_t*& payload_ptr, size_t &payload_len);

// Placeholder encryption API: encrypt/decrypt the bytes AFTER the CompanyID prefix.
// These are no-ops by default; implement real crypto later and call here.
// New signature accepts target MAC and nonce so AEAD IV can be derived deterministically.
// Returns true on success and fills out_len with the output length (ciphertext+tag or plaintext).
bool btEncryptPayload(const uint8_t* in, size_t in_len, const uint8_t target_mac[6], uint16_t nonce, uint8_t* out, size_t &out_len);
bool btDecryptPayload(const uint8_t* in, size_t in_len, const uint8_t target_mac[6], uint16_t nonce, uint8_t* out, size_t &out_len);

// Compile-time flag to enable BT encryption. Defaults to enabled (AES-GCM).
#ifndef USE_BT_CRYPTO
#define USE_BT_CRYPTO 1
#endif

  // Feature flag: enable extended adverts (requires host/stack support). Default: 0 (legacy)
#ifndef USE_EXT_ADV
#define USE_EXT_ADV 0
#endif

// Command helpers migrated to compact payload interpretation.
