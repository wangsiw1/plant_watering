#pragma once

#include <stddef.h>
#include <stdint.h>

constexpr uint32_t WATER_INTERVAL_MIN = 60;
constexpr uint32_t WATER_INTERVAL_MAX = 2419200;
constexpr uint32_t DEFAULT_WATER_INTERVAL = 3600;

inline bool validWaterInterval(uint32_t waterIntervalSec) {
  return waterIntervalSec >= WATER_INTERVAL_MIN &&
         waterIntervalSec <= WATER_INTERVAL_MAX;
}

inline bool autoWateringCooldownComplete(uint32_t nowUtcSec,
                                         uint32_t lastWateringUtcSec,
                                         uint32_t waterIntervalSec) {
  if (lastWateringUtcSec == 0) return true;
  return nowUtcSec >= lastWateringUtcSec &&
         nowUtcSec - lastWateringUtcSec >= waterIntervalSec;
}

inline bool autoWateringPotEligible(uint32_t nowUtcSec,
                                    uint32_t lastWateringUtcSec,
                                    uint32_t waterIntervalSec,
                                    uint16_t rawSoil,
                                    uint16_t correctedSoil,
                                    uint16_t threshold) {
  return autoWateringCooldownComplete(nowUtcSec, lastWateringUtcSec,
                                      waterIntervalSec) &&
         rawSoil > 200 && correctedSoil > threshold;
}

inline bool applyCompletedAutoWatering(uint32_t* lastWateringUtcSec,
                                       size_t potCount, uint16_t potMask,
                                       uint32_t completedUtcSec) {
  if (!lastWateringUtcSec || completedUtcSec == 0) return false;
  bool changed = false;
  for (size_t pot = 0; pot < potCount && pot < 16; ++pot) {
    if ((potMask & (1u << pot)) == 0) continue;
    if (lastWateringUtcSec[pot] != completedUtcSec) {
      lastWateringUtcSec[pot] = completedUtcSec;
      changed = true;
    }
  }
  return changed;
}
