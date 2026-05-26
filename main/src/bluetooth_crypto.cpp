#include "BluetoothCommon.h"
#include "Utility.h"
#include <cstring>
#include "mbedtls/gcm.h"
#include "mbedtls/md.h"

using namespace BT_TLV;

/*
  Bluetooth Crypto Helpers

  Provides:
  - AES-GCM AEAD encryption/decryption for BLE payloads
  - IV derivation via HMAC-SHA256(key, mac||nonce||"btiv")
  - Extended advertising helpers (NimBLEExtAdvertising with legacy fallback)

  Security notes:
  - Uses a static network key placeholder. Replace with NVS-stored key for production.
  - IV is derived per packet using target MAC and nonce (no static IV).
  - Tag is 16 bytes appended to ciphertext.

  Future:
  - Consider ChaCha20-Poly1305 as an alternative AEAD for platforms without AES hardware.
*/

// --- AES-GCM AEAD implementation ---
// Derive a 12-byte AEAD IV from network key, target MAC and packet nonce.
// Current construction: IV = first 12 bytes of HMAC-SHA256(key, mac||nonce||"btiv").
// Future: consider ChaCha20-Poly1305 as an alternative AEAD for platforms
// without AES hardware acceleration.
static const uint8_t s_bt_net_key[16] = { 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F };

static bool derive_iv(const uint8_t mac[6], uint16_t nonce, uint8_t out_iv[12]) {
  // IV = first 12 bytes of HMAC-SHA256(key, mac||nonce||"btiv")
  uint8_t buf[6+2+4];
  memcpy(buf, mac, 6);
  buf[6] = (uint8_t)(nonce >> 8);
  buf[7] = (uint8_t)(nonce & 0xFF);
  memcpy(&buf[8], "btiv", 4);
  unsigned char hmac[32];
  const mbedtls_md_info_t *md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (!md_info) return false;
  if (mbedtls_md_hmac(md_info, s_bt_net_key, sizeof(s_bt_net_key), buf, sizeof(buf), hmac) != 0) return false;
  memcpy(out_iv, hmac, 12);
  return true;
}

bool btEncryptPayload(const uint8_t* in, size_t in_len, const uint8_t target_mac[6], uint16_t nonce, uint8_t* out, size_t &out_len) {
  const size_t tag_len = 16;
  if (!in || !out) return false;
  uint8_t iv[12]; if (!derive_iv(target_mac, nonce, iv)) return false;
  mbedtls_gcm_context gcm; mbedtls_gcm_init(&gcm);
  if (mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, s_bt_net_key, sizeof(s_bt_net_key)*8) != 0) { mbedtls_gcm_free(&gcm); return false; }
  unsigned char tag[tag_len];
  int rc = mbedtls_gcm_crypt_and_tag(&gcm, MBEDTLS_GCM_ENCRYPT, in_len, iv, sizeof(iv), NULL, 0, in, out, tag_len, tag);
  mbedtls_gcm_free(&gcm);
  if (rc != 0) return false;
  // append tag
  memcpy(out + in_len, tag, tag_len);
  out_len = in_len + tag_len;
  return true;
}

bool btDecryptPayload(const uint8_t* in, size_t in_len, const uint8_t target_mac[6], uint16_t nonce, uint8_t* out, size_t &out_len) {
  const size_t tag_len = 16;
  if (!in || !out) return false;
  if (in_len < tag_len) return false;
  size_t cipher_len = in_len - tag_len;
  const unsigned char* tag = in + cipher_len;
  uint8_t iv[12]; if (!derive_iv(target_mac, nonce, iv)) return false;
  mbedtls_gcm_context gcm; mbedtls_gcm_init(&gcm);
  if (mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, s_bt_net_key, sizeof(s_bt_net_key)*8) != 0) { mbedtls_gcm_free(&gcm); return false; }
  int rc = mbedtls_gcm_auth_decrypt(&gcm, cipher_len, iv, sizeof(iv), NULL, 0, tag, tag_len, in, out);
  mbedtls_gcm_free(&gcm);
  if (rc != 0) return false;
  out_len = cipher_len;
  return true;
}
