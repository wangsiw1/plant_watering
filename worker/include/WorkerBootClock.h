#pragma once

#include <stdint.h>

#include "soc/clk_tree_defs.h"

constexpr uint32_t WORKER_RTC_FAULT_PATTERN_CYCLE_MS = 2000;
constexpr uint32_t WORKER_RTC_FAULT_DISPLAY_MS = 60000;
constexpr uint32_t WORKER_RTC_FAULT_RETRY_SLEEP_US = 5000000;
constexpr uint32_t WORKER_RTC_FAULT_UPDATE_MS = 10;

inline bool workerBootRtcSourceIsExternal(
    soc_rtc_slow_clk_src_t source) {
  return source == SOC_RTC_SLOW_CLK_SRC_XTAL32K;
}

inline bool workerBootRtcFaultLedOn(uint32_t elapsedMs) {
  const uint32_t phaseMs = elapsedMs % WORKER_RTC_FAULT_PATTERN_CYCLE_MS;
  return phaseMs < 100u ||
         (phaseMs >= 200u && phaseMs < 300u) ||
         (phaseMs >= 400u && phaseMs < 500u);
}

inline bool workerBootRtcFaultShouldRetry(uint32_t elapsedMs) {
  return elapsedMs >= WORKER_RTC_FAULT_DISPLAY_MS;
}

// Validates the V2 worker RTC slow-clock source before any application
// subsystem starts. Returns immediately on targets that do not require XTAL32K.
void workerBootRtcClockGate();
