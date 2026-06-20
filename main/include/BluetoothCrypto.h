#pragma once

#include <stddef.h>
#include <stdint.h>

struct BtMessageId;

bool btCryptoInit();
bool btEncryptPayload(const uint8_t* in, size_t inLen,
                      const uint8_t targetMac[6], const BtMessageId& messageId,
                      uint8_t* out, size_t& outLen);
bool btDecryptPayload(const uint8_t* in, size_t inLen,
                      const uint8_t targetMac[6], const BtMessageId& messageId,
                      uint8_t* out, size_t& outLen);
