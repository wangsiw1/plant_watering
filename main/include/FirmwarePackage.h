#pragma once

#include <stddef.h>
#include <stdint.h>

namespace FirmwarePackage {

constexpr size_t HEADER_SIZE = 64;
constexpr uint16_t FORMAT_VERSION = 1;
constexpr uint8_t ROLE_MAIN = 1;
constexpr uint8_t ROLE_WORKER = 2;
constexpr uint8_t CHIP_ESP32_C3 = 1;

struct Header {
  uint16_t formatVersion;
  uint16_t headerSize;
  uint8_t role;
  uint8_t chip;
  uint16_t hardwareTarget;
  uint32_t firmwareVersion;
  uint32_t imageSize;
  uint8_t imageSha256[32];
};

enum class ParseError : uint8_t {
  NONE,
  BAD_LENGTH,
  BAD_MAGIC,
  BAD_FORMAT,
  BAD_HEADER_SIZE,
  BAD_ROLE,
  BAD_CHIP,
  BAD_HARDWARE,
  BAD_VERSION,
  BAD_IMAGE_SIZE,
  BAD_RESERVED
};

ParseError parseMainHeader(const uint8_t* data, size_t len,
                           uint32_t maximumImageSize, Header& out);
ParseError parseWorkerHeader(const uint8_t* data, size_t len,
                             uint32_t maximumImageSize, Header& out);
const char* parseErrorName(ParseError error);

}  // namespace FirmwarePackage
