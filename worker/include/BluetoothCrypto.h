#pragma once


bool btEncryptPayload(const uint8_t* in, size_t in_len, const uint8_t target_mac[6], uint16_t nonce, uint8_t* out, size_t &out_len);
bool btDecryptPayload(const uint8_t* in, size_t in_len, const uint8_t target_mac[6], uint16_t nonce, uint8_t* out, size_t &out_len);
