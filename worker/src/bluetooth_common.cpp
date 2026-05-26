#include "BluetoothCommon.h"
#include "Utility.h"
#include <cstring>
#include <NimBLEDevice.h>
#include <NimBLEExtAdvertising.h>
#include <NimBLEAdvertising.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "BluetoothCrypto.h"
#include "HardwareConfig.h"

#if defined(WORKER_POT_COUNT)
#if WORKER_POT_COUNT > 4 && !USE_EXT_ADV
#error "WORKER_POT_COUNT > 4 requires USE_EXT_ADV=1 in build flags to avoid legacy advert truncation"
#endif
#endif

using namespace BT_TLV;

/*
  Bluetooth packet formats and crypto (shared common helpers)

  Overview
  - BLE Manufacturer Specific Data (AD type 0xFF) is used. The AD payload begins
    with a 2-byte Company Identifier (little-endian). Use 0xFFFF for local/test.

  Supported payload encodings
  1) Compact packet (preferred)
     CompanyID (2) + TargetMAC (6) + Nonce (2, big-endian) + MsgPayload...
     - MsgPayload starts with a single MsgType byte (reuse TYPE_CMD_* /TYPE_STATUS values).
     - Examples:
       - TYPE_STATUS (0x40): [TYPE_STATUS][Battery(1)][PotCount(1)][Soil1(2), ...]
       - TYPE_CMD_WATER (0x32): compact form: [TYPE_CMD_WATER][PotMask(2)][Duration(2)]
       - TYPE_ACK (0x20): payload may be empty or include additional fields as needed.

  2) Legacy TLV (backwards compatibility)
     CompanyID (2) + sequence of TLV fields: [Type(1), Length(1), Value(Length)]
     - The code continues to provide TLV helpers (tlv_*). Sender code will convert
       queued TLV payloads to compact packets when appropriate (see below).

  Encryption (AEAD)
  - When `USE_BT_CRYPTO` is enabled, MsgPayload (the bytes after TargetMAC+Nonce) is
    encrypted with AES-GCM and the 16-byte authentication tag is appended to the
    ciphertext.
  - IV derivation: IV = first 12 bytes of HMAC-SHA256(network_key, target_mac || nonce || "btiv").
    This provides a deterministic per-packet IV derived from the 2-byte nonce and target MAC.
  - Implementations: `btEncryptPayload` / `btDecryptPayload` in `bluetooth_crypto.cpp` use
    mbedTLS GCM APIs. The network key is currently a placeholder and must be provisioned
    securely in production (NVS / provisioning flow).
  - On decryption failure, `parse_compact_packet_header()` currently falls back to returning
    the raw, unencrypted payload so the system can interoperate with unencrypted workers.
    A small compatibility fallback attempts decryption with a reversed-MAC byte order when
    IV derivation ordering mismatches occur.

  Extended Advertising
  - If `USE_EXT_ADV` is enabled and the build/stack supports extended adverts, the code
    will attempt to publish using `NimBLEExtAdvertising` to allow payloads larger than
    the legacy 31-byte limit. The implementation falls back to `NimBLEAdvertising` when
    extended advertising is unavailable.

  Parsing and compatibility notes
  - The common scan callback strips the CompanyID and hands the remainder to the
    registered advert handler. `parse_compact_packet_header()` expects data starting at
    TargetMAC (i.e., the first 6 bytes are the target MAC) and returns a pointer/len
    to the payload (decrypted if AEAD is enabled).
  - For queued send payloads, the sender performs a TLV->compact conversion only when
    the queued buffer appears to be a TLV payload robustly: the length byte must
    exactly equal the remaining bytes (payload[1] == len - 2). This avoids
    mis-detecting compact messages whose second byte (e.g. battery) looks like a TLV length.

  Security note
  - Do not hardcode network keys in production. Ensure keys are provisioned and rotated
    securely. Consider ChaCha20-Poly1305 as an alternative AEAD on platforms lacking AES
    hardware acceleration.
*/

// Legacy TLV parsing removed as repository uses compact packet format.
// _parse_tlv_once and per-field TLV extractors were intentionally removed.

// --- Command queue / sender shared implementation ---
namespace {
  struct PendingCmd { uint16_t nonce; bool inUse; volatile bool acked; };
  static const int MAX_PENDING = 8;
  static PendingCmd pending[MAX_PENDING];
  static uint16_t next_nonce = 1;
  static uint16_t allocNonce() { uint16_t n = next_nonce++; if (next_nonce==0) next_nonce=1; return n; }

  static const int CMD_QUEUE_LEN = 8;
  static const size_t MAX_CMD_PAYLOAD = 64; // allow STATUS payloads for multi-pot workers
  struct CmdItem { uint8_t mac[6]; uint8_t payload[MAX_CMD_PAYLOAD]; size_t len; int retries; unsigned timeoutMs; };
  static QueueHandle_t cmdQueue = nullptr;
  
  static const int ACK_QUEUE_LEN = 4;
  struct AckItem { uint8_t mac[6]; uint16_t nonce; };
  static QueueHandle_t ackQueue = nullptr;

  static SemaphoreHandle_t sBroadcastMutex = nullptr;
  static OnCommandSentCb gOnSent = nullptr;
}

namespace BT_TLV {
  void btCommonRegisterOnCommandSent(OnCommandSentCb cb) { gOnSent = cb; }

  void btCommonMarkAck(uint16_t nonce) {
    for (int i=0;i<MAX_PENDING;i++) if (pending[i].inUse && pending[i].nonce==nonce) pending[i].acked = true;
  }

  static void btCommonSenderTask(void* pv) {
    (void)pv;
    CmdItem cmdItem;
    for (;;) {
      if (xQueueReceive(cmdQueue, &cmdItem, portMAX_DELAY) == pdTRUE) {
        int slot=-1; for (int i=0;i<MAX_PENDING;i++) if (!pending[i].inUse) { slot=i; break; }
        if (slot<0) continue; // drop if no pending slot
        uint8_t cmdData[256];
        uint16_t nonce = allocNonce();
        // Queued payloads are compact: [msgType][...]
        size_t compactLen = cmdItem.len;
        const uint8_t* compactPayload = cmdItem.payload;
        size_t cmdLen = make_compact_packet(cmdItem.mac, nonce, compactPayload, compactLen, cmdData, USE_BT_CRYPTO);
        pending[slot].inUse = true; pending[slot].acked = false; pending[slot].nonce = nonce;
        // Notify on-sent for water commands (compact payload starts with msgType)
        if (cmdItem.len >= 1 && cmdItem.payload[0] == TYPE_CMD_WATER) {
          if (gOnSent) gOnSent(cmdItem.mac, nonce);
        }
        LOG("[->] [%d] Send to: %02X:%02X:%02X:%02X:%02X:%02X", nonce, cmdItem.mac[0], cmdItem.mac[1], cmdItem.mac[2], cmdItem.mac[3], cmdItem.mac[4], cmdItem.mac[5]);
        LOG("[->] [%d] Payload type: %02X", nonce, cmdItem.payload[0]);
        for (int attempt=0; attempt<=cmdItem.retries; ++attempt) {
          LOG("[->] [%d] Attempt %d/%d", nonce, attempt, cmdItem.retries);
          BT_TLV::btCommonBroadcast(cmdData, cmdLen, 300);
          unsigned long start = millis();
          while (millis()-start < cmdItem.timeoutMs) {
            if (pending[slot].acked) break;
            vTaskDelay(pdMS_TO_TICKS(20));
          }
          if (pending[slot].acked) break;
        }
        pending[slot].inUse = false;
      }
    }
  }

  static void btAckSenderTask(void* pv) {
    (void)pv;
    AckItem ackItem;
    for (;;) {
      if (xQueueReceive(ackQueue, &ackItem, portMAX_DELAY) == pdTRUE) {
        uint8_t ackData[64];
        uint8_t ackPayload[1]; ackPayload[0] = TYPE_ACK; // MsgType==ACK
        size_t ackLen = make_compact_packet(ackItem.mac, ackItem.nonce, ackPayload, sizeof(ackPayload), ackData, USE_BT_CRYPTO);
        btCommonBroadcast(ackData, ackLen, 300);
      }
    }
  }

  bool btCommonInitSender() {
    sBroadcastMutex = xSemaphoreCreateMutex();
    if (!cmdQueue) cmdQueue = xQueueCreate(CMD_QUEUE_LEN, sizeof(CmdItem));
    if (!ackQueue) ackQueue = xQueueCreate(ACK_QUEUE_LEN, sizeof(AckItem));
    if (cmdQueue && ackQueue) {
      static bool started = false;
      if (!started) {
        xTaskCreate(btCommonSenderTask, "btCommonSender", 3072, NULL, 3, NULL);
        xTaskCreate(btAckSenderTask, "btAckSender", 3072, NULL, 2, NULL);
        started = true;
      }
      return true;
    }
    return false;
  }

  bool btCommonQueueCommand(const uint8_t target_mac[6], const uint8_t* payload, size_t len, int retries, unsigned timeoutMs) {
    if (!cmdQueue) return false;
    if (len > MAX_CMD_PAYLOAD) return false;
    CmdItem it;
    memcpy(it.mac, target_mac, 6);
    memset(it.payload, 0, MAX_CMD_PAYLOAD);
    memcpy(it.payload, payload, len);
    it.len = len; it.retries = retries; it.timeoutMs = timeoutMs;
    return xQueueSend(cmdQueue, &it, 0) == pdTRUE;
  }

  bool btCommonQueueAck(const uint8_t target_mac[6], uint16_t nonce) {
    if (!ackQueue) return false;
    AckItem it;
    memcpy(it.mac, target_mac, 6);
    it.nonce = nonce;
    return xQueueSend(ackQueue, &it, 0) == pdTRUE;
  }

}

void extract_src_mac(const NimBLEAdvertisedDevice* adv, uint8_t src_mac[6]) {
  // read advertiser address (sender) from the advertisement metadata
  NimBLEAddress advAddr = adv->getAddress();
  advAddr.reverseByteOrder();
  for (int i = 0; i < 6; i++) src_mac[i] = advAddr.getVal()[i];
}
// Legacy TLV helpers removed — repository uses compact packet format only.

// New compact packet builder: CompanyID(2) + TargetMAC(6) + Nonce(2 BE) + payload (msgType + ...)
size_t make_compact_packet(const uint8_t target_mac[6], uint16_t nonce, const uint8_t* payload, size_t payload_len, uint8_t* out, bool encrypt) {
  size_t pos = 0;
  out[pos++] = 0xFF; out[pos++] = 0xFF; // CompanyID
  memcpy(&out[pos], target_mac, 6); pos += 6;
  out[pos++] = (uint8_t)(nonce >> 8);
  out[pos++] = (uint8_t)(nonce & 0xFF);
  if (payload_len > 0) {
    if (encrypt) {
      size_t out_len = payload_len + 16; // AES-GCM tag 16 bytes
      if (!btEncryptPayload(payload, payload_len, target_mac, nonce, &out[pos], out_len)) {
        // encryption failed — fall back to plaintext
        memcpy(&out[pos], payload, payload_len); pos += payload_len;
      } else {
        pos += out_len;
      }
    } else {
      memcpy(&out[pos], payload, payload_len); pos += payload_len;
    }
  }
  return pos;
}

// Parse a compact packet header (data starts AFTER CompanyID). On success
// returns true and fills `out_target_mac`, `nonce` and returns pointer/len
// to payload (payload begins with msgType byte). If crypto is enabled this
// will attempt to decrypt the payload into an internal buffer and return
// a pointer to the decrypted bytes.
bool parse_compact_packet_header(const uint8_t* data, size_t len, uint8_t out_target_mac[6], uint16_t &nonce, const uint8_t*& payload_ptr, size_t &payload_len) {
  if (!data) return false;
  // Need at least TargetMAC(6) + Nonce(2)
  if (len < 8) return false;
  memcpy(out_target_mac, data, 6);
  nonce = (uint16_t(data[6]) << 8) | uint16_t(data[7]);
  const uint8_t* raw_payload = data + 8;
  size_t raw_len = len - 8;
#if USE_BT_CRYPTO
  static uint8_t s_decrypt_buf[256];
  size_t dec_len = 0;
  // Try to decrypt; if decryption fails we'll fall back to raw payload
  if (raw_len > 0 && btDecryptPayload(raw_payload, raw_len, out_target_mac, nonce, s_decrypt_buf, dec_len)) {
    payload_ptr = s_decrypt_buf;
    payload_len = dec_len;
  } else {
    payload_ptr = raw_payload;
    payload_len = raw_len;
  }
#else
  payload_ptr = raw_payload;
  payload_len = raw_len;
#endif
  return true;
}

// Crypto and extended advertising helpers are now in BluetoothCrypto.h
// Shared scan callback implementation and wiring
namespace BT_TLV {
  AdvertHandler gAdvertHandler = nullptr;
  void btCommonSetAdvertHandler(AdvertHandler h) { gAdvertHandler = h; }

  void btCommonBroadcast(const uint8_t* payload, size_t len, unsigned msDelay) {
    if (!sBroadcastMutex) return;
    xSemaphoreTake(sBroadcastMutex, portMAX_DELAY);

#if CONFIG_BT_NIMBLE_EXT_ADV
    NimBLEExtAdvertising* adv = NimBLEDevice::getAdvertising();
    NimBLEExtAdvertisement ad;
    
    // Set your intervals (Units are 0.625ms)
    ad.setMinInterval(80);  // 80 * 0.625 = 50ms
    ad.setMaxInterval(120); // 120 * 0.625 = 75ms
    
    // Set the Data
    ad.setManufacturerData(payload, len);

    if (!adv->setInstanceData(0, ad)) {
      LOG("EXT_ADV: setInstanceData failed, falling back");
    } else {
      adv->start(0, msDelay);
    }
#else
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->setMinInterval(80); // 50ms
    adv->setMaxInterval(120); // 75ms
    NimBLEAdvertisementData ad;
    std::string m((const char*)payload, len);
    ad.setManufacturerData(m);
    adv->setAdvertisementData(ad);
    adv->start(msDelay); // Try later so no need to manual stop
    // adv->start();
    // vTaskDelay(pdMS_TO_TICKS(msDelay));
    // adv->stop();
#endif

    xSemaphoreGive(sBroadcastMutex);
  }

  class CommonScanCb : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* adv) override {
      if (!gAdvertHandler) return;
      std::string m = adv->getManufacturerData();
      if (m.size() < 2) return;
      if ((uint8_t)m[0] != 0xFF || (uint8_t)m[1] != 0xFF) return;

      gAdvertHandler((const uint8_t*)m.data() + 2, m.size() - 2, adv);
    }
  };

  // helper to attach common scan cb to the global scanner
  void btCommonInstallScanCallbacks() {
    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(new CommonScanCb(), true);
  }

}
