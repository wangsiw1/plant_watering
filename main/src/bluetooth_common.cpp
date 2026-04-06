#include "BluetoothCommon.h"
#include "Utility.h"
#include <cstring>
#include <NimBLEDevice.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

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
  - TARGET is carried using TYPE_MAC (0x01): 6 bytes target MAC in main->worker commands.
  - TYPE_NONCE (0x11): 2 bytes nonce to correlate requests and ACKs.
  - (legacy TYPE_PAYLOAD removed): command fields are TLV fields themselves.
  - TYPE_ACK (0x20): 2 bytes nonce — worker acknowledges command by advertising this TLV.

  Example command (water for 10s):
    CompanyID(2) +
    TYPE_MAC(1) LEN(1) MAC(6) +  // target MAC carried in TYPE_MAC
    TYPE_NONCE(1) LEN(1) NONCE(2) +
    Commands are encoded as TLV fields (TYPE_CMD_*), see header for types.

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

// --- Command queue / sender shared implementation ---
namespace {
  struct PendingCmd { uint16_t nonce; bool inUse; volatile bool acked; };
  static const int MAX_PENDING = 8;
  static PendingCmd pending[MAX_PENDING];
  static uint16_t next_nonce = 1;
  static uint16_t allocNonce() { uint16_t n = next_nonce++; if (next_nonce==0) next_nonce=1; return n; }

  static const int CMD_QUEUE_LEN = 8;
  static const size_t MAX_CMD_PAYLOAD = 17; // 31 - 2(CompnayID) - 8(MAC TLV) - 4(NONCE TLV)
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
        uint8_t cmdData[31];
        uint16_t nonce = allocNonce();
        size_t cmdLen = tlv_make_command(cmdItem.mac, nonce, cmdItem.payload, cmdItem.len, cmdData, false);
        pending[slot].inUse = true; pending[slot].acked = false; pending[slot].nonce = nonce;
        // Determine if the queued payload contains a water command TLV; payload
        // is now a sequence of TLV fields. Scan for TYPE_CMD_WATER and invoke
        // the on-sent callback when present.
        {
          uint16_t dur;
          if (tlv_extract_cmd_water(cmdItem.payload, cmdItem.len, dur) && gOnSent) gOnSent(cmdItem.mac, nonce);
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
        uint8_t ackData[31];
        size_t ackLen = tlv_make_command(ackItem.mac, ackItem.nonce, 0, 0, ackData, true);
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

// tlv_extract_payload removed — commands are TLV fields and should be parsed
// by scanning with _parse_tlv_once.

bool tlv_extract_ack(const uint8_t* data, size_t len, uint16_t &nonce) {
  size_t off = 0; uint8_t type,l; const uint8_t* ptr;
  while (_parse_tlv_once(data,len,type,ptr,l,off)) {
    if (type == TYPE_ACK && l>=2) { nonce = (uint16_t(ptr[0])<<8) | uint16_t(ptr[1]); return true; }
  }
  return false;
}

bool tlv_extract_cmd_probe(const uint8_t* data, size_t len) {
  size_t off = 0; uint8_t type,l; const uint8_t* ptr;
  while (_parse_tlv_once(data,len,type,ptr,l,off)) {
    if (type == TYPE_CMD_PROBE) return true;
  }
  return false;
}

bool tlv_extract_cmd_water(const uint8_t* data, size_t len, uint16_t &duration) {
  size_t off = 0; uint8_t type,l; const uint8_t* ptr;
  while (_parse_tlv_once(data,len,type,ptr,l,off)) {
    if (type == TYPE_CMD_WATER && l>=2) { duration = (uint16_t(ptr[0])<<8) | uint16_t(ptr[1]); return true; }
  }
  return false;
}

bool tlv_extract_cmd_sleep(const uint8_t* data, size_t len, uint32_t &delay_s) {
  size_t off = 0; uint8_t type,l; const uint8_t* ptr;
  while (_parse_tlv_once(data,len,type,ptr,l,off)) {
    if (type == TYPE_CMD_SLEEP && l>=4) { delay_s = (uint32_t(ptr[0])<<24) | (uint32_t(ptr[1])<<16) | (uint32_t(ptr[2])<<8) | uint32_t(ptr[3]); return true; }
  }
  return false;
}

// tlv_make_status removed: status payloads are constructed by the worker and queued via btCommonQueueBroadcast

size_t tlv_make_command(const uint8_t target_mac[6], uint16_t nonce, const uint8_t* payload, size_t payload_len, uint8_t* out, bool is_ack) {
  size_t pos = 0;
  // company ID
  out[pos++] = 0xFF; out[pos++] = 0xFF;
  // target MAC TLV
  out[pos++] = TYPE_MAC; out[pos++] = 6;
  memcpy(&out[pos], target_mac, 6); pos += 6;
  // nonce TLV
  out[pos++] = is_ack ? TYPE_ACK : TYPE_NONCE;
  out[pos++] = 2; out[pos++] = nonce >> 8; out[pos++] = nonce & 0xFF;
  // payload TLVs
  if (!is_ack) { memcpy(&out[pos], payload, payload_len); pos += payload_len; }
  return pos;
}

// Shared scan callback implementation and wiring
namespace BT_TLV {
  AdvertHandler gAdvertHandler = nullptr;
  void btCommonSetAdvertHandler(AdvertHandler h) { gAdvertHandler = h; }

  void btCommonBroadcast(const uint8_t* payload, size_t len, unsigned msDelay) {
    if (!sBroadcastMutex) return;
    xSemaphoreTake(sBroadcastMutex, portMAX_DELAY);

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
