#pragma once

#include <NimBLEDevice.h>
#include <stddef.h>
#include <stdint.h>

#ifndef USE_BT_CRYPTO
#define USE_BT_CRYPTO 1
#endif

#ifndef USE_EXT_ADV
#define USE_EXT_ADV 0
#endif

#if !USE_EXT_ADV
#error "Bluetooth TLV protocol requires USE_EXT_ADV=1"
#endif

struct BtMessageId {
  uint64_t sessionId;
  uint32_t sequence;
};

enum class BtSendStatus : uint8_t {
  INVALID,
  QUEUE_FULL,
  TRANSMIT_FAILED,
  SENT,
  ACKED
};

struct BtSendResult {
  BtSendStatus status;
  BtMessageId messageId;
  bool transmissionStarted;
  int64_t startedUs;
  uint8_t ackMac[6];
};

using BtBeforeTransmitCb = void(*)(void* context);

namespace BT_TLV {
  constexpr size_t MAX_BODY_SIZE = 64;
  constexpr uint8_t TYPE_ACK = 0x20;
  constexpr uint8_t TYPE_CMD_PROBE = 0x30;
  constexpr uint8_t TYPE_CMD_SLEEP = 0x31;
  constexpr uint8_t TYPE_CMD_WATER = 0x32;
  constexpr uint8_t TYPE_CMD_OTA_PREPARE = 0x33;
  constexpr uint8_t TYPE_STATUS = 0x40;
  constexpr uint8_t TYPE_CONFIG = 0x41;
  constexpr uint8_t TYPE_EVENT_WATER_DONE = 0x42;

  constexpr uint8_t FIELD_BATT = 0x01;
  constexpr uint8_t FIELD_POT_COUNT = 0x02;
  constexpr uint8_t FIELD_SOIL_LIST = 0x03;
  constexpr uint8_t FIELD_SLEEP_SEC = 0x04;
  constexpr uint8_t FIELD_POT_MASK = 0x05;
  constexpr uint8_t FIELD_DURATION_LIST = 0x06;
  constexpr uint8_t FIELD_FW_VERSION = 0x07;

  struct TlvFieldView {
    uint8_t type;
    uint8_t len;
    const uint8_t* value;
  };

  struct BtBodyBuilder {
    uint8_t data[MAX_BODY_SIZE];
    size_t len;
  };

  using AdvertHandler = void(*)(const uint8_t* data, size_t len,
                                const NimBLEAdvertisedDevice* adv);

  bool btCommonInitSender();
  void btCommonSetAdvertHandler(AdvertHandler handler);
  void btCommonInstallScanCallbacks();

  BtSendResult btCommonSendCommand(const uint8_t targetMac[6],
                                   const uint8_t* payload, size_t len,
                                   int retries, uint32_t ackTimeoutMs,
                                   BtBeforeTransmitCb beforeTransmit = nullptr,
                                   void* context = nullptr);
  bool btCommonQueueAck(const uint8_t targetMac[6], const BtMessageId& messageId);
  BtSendResult btCommonSendAckAndWait(const uint8_t targetMac[6],
                                     const BtMessageId& messageId);
  void btCommonMarkAck(const uint8_t sourceMac[6], const BtMessageId& messageId);

  void btBodyBegin(BtBodyBuilder& body, uint8_t msgType);
  bool btTlvAppend(BtBodyBuilder& body, uint8_t type, const uint8_t* value, uint8_t len);
  bool btTlvAppendU8(BtBodyBuilder& body, uint8_t type, uint8_t value);
  bool btTlvAppendU16(BtBodyBuilder& body, uint8_t type, uint16_t value);
  bool btTlvAppendU32(BtBodyBuilder& body, uint8_t type, uint32_t value);
  bool btTlvNext(const uint8_t* tlvs, size_t tlvsLen, size_t& offset, TlvFieldView& field);
  bool btTlvReadRequiredU8(const uint8_t* tlvs, size_t tlvsLen, uint8_t type, uint8_t& out);
  bool btTlvReadRequiredU16(const uint8_t* tlvs, size_t tlvsLen, uint8_t type, uint16_t& out);
  bool btTlvReadRequiredU32(const uint8_t* tlvs, size_t tlvsLen, uint8_t type, uint32_t& out);
  bool btTlvReadOptionalU32(const uint8_t* tlvs, size_t tlvsLen, uint8_t type,
                            uint32_t& out, bool& found);
  bool btTlvReadRequiredBytes(const uint8_t* tlvs, size_t tlvsLen, uint8_t type,
                              const uint8_t*& out, uint8_t& outLen);
  bool btTlvReadRequiredU16Array(const uint8_t* tlvs, size_t tlvsLen, uint8_t type,
                                 uint16_t* out, size_t maxCount, size_t& outCount);
}

extern const uint8_t BROADCAST_MAC[6];

void extract_src_mac(const NimBLEAdvertisedDevice* adv, uint8_t outMac[6]);
size_t make_bt_packet(const uint8_t targetMac[6], const BtMessageId& messageId,
                      const uint8_t* payload, size_t payloadLen, uint8_t* out);
bool parse_bt_packet_header(const uint8_t* data, size_t len, uint8_t outTargetMac[6],
                            BtMessageId& messageId, const uint8_t*& payload,
                            size_t& payloadLen);
