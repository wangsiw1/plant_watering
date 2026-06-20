#include "BluetoothCrypto.h"
#include "BluetoothCommon.h"

#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "mbedtls/gcm.h"
#include "mbedtls/md.h"

namespace {
constexpr size_t GCM_TAG_SIZE = 16;
constexpr size_t GCM_IV_SIZE = 12;
const uint8_t NET_KEY[16] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};

mbedtls_gcm_context gGcm;
SemaphoreHandle_t gCryptoMutex = nullptr;
bool gCryptoReady = false;

void encodeMessageId(const BtMessageId& id, uint8_t out[12]) {
  for (int i = 0; i < 8; ++i) {
    out[i] = static_cast<uint8_t>(id.sessionId >> (56 - i * 8));
  }
  out[8] = static_cast<uint8_t>(id.sequence >> 24);
  out[9] = static_cast<uint8_t>(id.sequence >> 16);
  out[10] = static_cast<uint8_t>(id.sequence >> 8);
  out[11] = static_cast<uint8_t>(id.sequence);
}

void buildAuthenticatedHeader(const uint8_t targetMac[6], const BtMessageId& id,
                              uint8_t out[18]) {
  memcpy(out, targetMac, 6);
  encodeMessageId(id, out + 6);
}

bool deriveIv(const uint8_t targetMac[6], const BtMessageId& id,
              uint8_t outIv[GCM_IV_SIZE]) {
  uint8_t material[22];
  buildAuthenticatedHeader(targetMac, id, material);
  memcpy(material + 18, "btiv", 4);

  uint8_t hmac[32];
  const mbedtls_md_info_t* mdInfo = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (!mdInfo) return false;
  if (mbedtls_md_hmac(mdInfo, NET_KEY, sizeof(NET_KEY),
                      material, sizeof(material), hmac) != 0) {
    return false;
  }
  memcpy(outIv, hmac, GCM_IV_SIZE);
  return true;
}
}

bool btCryptoInit() {
  if (!gCryptoMutex) gCryptoMutex = xSemaphoreCreateMutex();
  if (!gCryptoMutex) return false;

  xSemaphoreTake(gCryptoMutex, portMAX_DELAY);
  if (!gCryptoReady) {
    mbedtls_gcm_init(&gGcm);
    gCryptoReady =
        mbedtls_gcm_setkey(&gGcm, MBEDTLS_CIPHER_ID_AES,
                           NET_KEY, sizeof(NET_KEY) * 8) == 0;
  }
  bool ready = gCryptoReady;
  xSemaphoreGive(gCryptoMutex);
  return ready;
}

bool btEncryptPayload(const uint8_t* in, size_t inLen,
                      const uint8_t targetMac[6], const BtMessageId& messageId,
                      uint8_t* out, size_t& outLen) {
  if (!in || !out || !btCryptoInit()) return false;

  uint8_t iv[GCM_IV_SIZE];
  uint8_t aad[18];
  uint8_t tag[GCM_TAG_SIZE];
  if (!deriveIv(targetMac, messageId, iv)) return false;
  buildAuthenticatedHeader(targetMac, messageId, aad);

  xSemaphoreTake(gCryptoMutex, portMAX_DELAY);
  int rc = mbedtls_gcm_crypt_and_tag(
      &gGcm, MBEDTLS_GCM_ENCRYPT, inLen, iv, sizeof(iv),
      aad, sizeof(aad), in, out, sizeof(tag), tag);
  xSemaphoreGive(gCryptoMutex);
  if (rc != 0) return false;

  memcpy(out + inLen, tag, sizeof(tag));
  outLen = inLen + sizeof(tag);
  return true;
}

bool btDecryptPayload(const uint8_t* in, size_t inLen,
                      const uint8_t targetMac[6], const BtMessageId& messageId,
                      uint8_t* out, size_t& outLen) {
  if (!in || !out || inLen < GCM_TAG_SIZE || !btCryptoInit()) return false;

  const size_t cipherLen = inLen - GCM_TAG_SIZE;
  uint8_t iv[GCM_IV_SIZE];
  uint8_t aad[18];
  if (!deriveIv(targetMac, messageId, iv)) return false;
  buildAuthenticatedHeader(targetMac, messageId, aad);

  xSemaphoreTake(gCryptoMutex, portMAX_DELAY);
  int rc = mbedtls_gcm_auth_decrypt(
      &gGcm, cipherLen, iv, sizeof(iv), aad, sizeof(aad),
      in + cipherLen, GCM_TAG_SIZE, in, out);
  xSemaphoreGive(gCryptoMutex);
  if (rc != 0) return false;

  outLen = cipherLen;
  return true;
}
