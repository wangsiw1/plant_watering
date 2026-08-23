#pragma once

#include <stdint.h>

enum class FirmwareVersionDecision : uint8_t {
  ALLOW,
  ALREADY_INSTALLED,
  BELOW_MINIMUM,
};

constexpr FirmwareVersionDecision firmwareVersionDecision(
    uint32_t currentVersion, uint32_t candidateVersion,
    uint32_t minimumAllowedVersion) {
  return candidateVersion == currentVersion
             ? FirmwareVersionDecision::ALREADY_INSTALLED
             : (candidateVersion < minimumAllowedVersion
                    ? FirmwareVersionDecision::BELOW_MINIMUM
                    : FirmwareVersionDecision::ALLOW);
}

static_assert(firmwareVersionDecision(4, 1, 2) ==
                  FirmwareVersionDecision::BELOW_MINIMUM,
              "versions below the floor must be rejected");
static_assert(firmwareVersionDecision(4, 2, 2) ==
                  FirmwareVersionDecision::ALLOW,
              "the minimum version must be accepted");
static_assert(firmwareVersionDecision(4, 3, 2) ==
                  FirmwareVersionDecision::ALLOW,
              "rollback above the floor must be accepted");
static_assert(firmwareVersionDecision(4, 4, 2) ==
                  FirmwareVersionDecision::ALREADY_INSTALLED,
              "the installed version must be rejected");
static_assert(firmwareVersionDecision(4, 5, 2) ==
                  FirmwareVersionDecision::ALLOW,
              "upgrades must be accepted");
