#include "FirmwarePackage.h"

#include <cstring>

#ifndef FW_HARDWARE_TARGET
#error "FW_HARDWARE_TARGET must be supplied by PlatformIO build_flags"
#endif

namespace FirmwarePackage {
namespace {

constexpr uint8_t MAGIC[8] = {'P', 'W', 'O', 'T', 'A', '0', '0', '1'};

uint16_t readU16(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t readU32(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

}  // namespace

ParseError parseHeader(const uint8_t* data, size_t len, uint8_t expectedRole,
                       uint16_t expectedHardware, bool checkHardware,
                       uint32_t maximumImageSize, Header& out) {
  if (!data || len != HEADER_SIZE) return ParseError::BAD_LENGTH;
  if (memcmp(data, MAGIC, sizeof(MAGIC)) != 0) return ParseError::BAD_MAGIC;

  out = {};
  out.formatVersion = readU16(data + 8);
  out.headerSize = readU16(data + 10);
  out.role = data[12];
  out.chip = data[13];
  out.hardwareTarget = readU16(data + 14);
  out.firmwareVersion = readU32(data + 16);
  out.imageSize = readU32(data + 20);
  memcpy(out.imageSha256, data + 24, sizeof(out.imageSha256));

  if (out.formatVersion != FORMAT_VERSION) return ParseError::BAD_FORMAT;
  if (out.headerSize != HEADER_SIZE) return ParseError::BAD_HEADER_SIZE;
  if (out.role != expectedRole) return ParseError::BAD_ROLE;
  if (out.chip != CHIP_ESP32_C3) return ParseError::BAD_CHIP;
  if (checkHardware && out.hardwareTarget != expectedHardware) {
    return ParseError::BAD_HARDWARE;
  }
  if (out.firmwareVersion == 0) return ParseError::BAD_VERSION;
  if (out.imageSize == 0 || out.imageSize > maximumImageSize) {
    return ParseError::BAD_IMAGE_SIZE;
  }
  for (size_t i = 56; i < HEADER_SIZE; ++i) {
    if (data[i] != 0) return ParseError::BAD_RESERVED;
  }
  return ParseError::NONE;
}

ParseError parseMainHeader(const uint8_t* data, size_t len,
                           uint32_t maximumImageSize, Header& out) {
  return parseHeader(data, len, ROLE_MAIN, FW_HARDWARE_TARGET, true,
                     maximumImageSize, out);
}

ParseError parseWorkerHeader(const uint8_t* data, size_t len,
                             uint32_t maximumImageSize, Header& out) {
  return parseHeader(data, len, ROLE_WORKER, 0, false, maximumImageSize, out);
}

const char* parseErrorName(ParseError error) {
  switch (error) {
    case ParseError::NONE: return "none";
    case ParseError::BAD_LENGTH: return "bad_header_length";
    case ParseError::BAD_MAGIC: return "bad_magic";
    case ParseError::BAD_FORMAT: return "bad_format";
    case ParseError::BAD_HEADER_SIZE: return "bad_header_size";
    case ParseError::BAD_ROLE: return "wrong_role";
    case ParseError::BAD_CHIP: return "wrong_chip";
    case ParseError::BAD_HARDWARE: return "wrong_hardware";
    case ParseError::BAD_VERSION: return "bad_version";
    case ParseError::BAD_IMAGE_SIZE: return "bad_image_size";
    case ParseError::BAD_RESERVED: return "bad_reserved";
    default: return "unknown";
  }
}

}  // namespace FirmwarePackage
