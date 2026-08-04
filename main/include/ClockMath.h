#pragma once

#include <stdint.h>

constexpr uint64_t CLOCK_DEFAULT_EPOCH_SEC = 946684800ULL;   // 2000-01-01 00:00 UTC
constexpr uint64_t CLOCK_MAX_SANE_EPOCH_SEC = 4102444800ULL; // 2100-01-01 00:00 UTC
constexpr uint64_t CLOCK_MIN_NTP_EPOCH_SEC = 1600000000ULL;

struct ClockInitialSelection {
  uint64_t epochSec;
  bool restored;
};

inline bool clockEpochIsSane(uint64_t epochSec) {
  return epochSec >= CLOCK_DEFAULT_EPOCH_SEC &&
         epochSec < CLOCK_MAX_SANE_EPOCH_SEC;
}

inline bool clockNtpEpochIsSane(uint64_t epochSec) {
  return epochSec >= CLOCK_MIN_NTP_EPOCH_SEC &&
         epochSec < CLOCK_MAX_SANE_EPOCH_SEC;
}

inline ClockInitialSelection clockSelectInitialEpoch(uint64_t retainedUtcSec,
                                                      uint64_t savedUtcSec) {
  bool retainedSane = clockEpochIsSane(retainedUtcSec);
  bool savedSane = clockEpochIsSane(savedUtcSec);
  if (!retainedSane && !savedSane) {
    return {CLOCK_DEFAULT_EPOCH_SEC, false};
  }
  if (!retainedSane) return {savedUtcSec, true};
  if (!savedSane) return {retainedUtcSec, true};
  return {retainedUtcSec > savedUtcSec ? retainedUtcSec : savedUtcSec, true};
}

inline uint32_t clockLocalTimeOfDaySec(uint64_t utcSec,
                                       int16_t tzOffsetMinutes) {
  int64_t local = static_cast<int64_t>(utcSec) +
                  static_cast<int64_t>(tzOffsetMinutes) * 60;
  return static_cast<uint32_t>((local % 86400 + 86400) % 86400);
}

inline bool clockEpochForLocalTimeOfDay(uint64_t referenceUtcSec,
                                        int16_t tzOffsetMinutes,
                                        uint32_t secOfDay,
                                        uint64_t& outUtcSec) {
  if (!clockEpochIsSane(referenceUtcSec) || secOfDay >= 86400) return false;
  int64_t localReference =
      static_cast<int64_t>(referenceUtcSec) +
      static_cast<int64_t>(tzOffsetMinutes) * 60;
  int64_t localDay =
      localReference - ((localReference % 86400 + 86400) % 86400);
  int64_t utc = localDay + secOfDay -
                static_cast<int64_t>(tzOffsetMinutes) * 60;
  if (utc <= 0) return false;
  outUtcSec = static_cast<uint64_t>(utc);
  return clockEpochIsSane(outUtcSec);
}

inline uint64_t clockRebaseTimestampPreservingAge(uint64_t oldNowUtcSec,
                                                   uint64_t newNowUtcSec,
                                                   uint64_t oldEventUtcSec) {
  if (oldEventUtcSec == 0) return 0;
  uint64_t age = oldNowUtcSec >= oldEventUtcSec
                     ? oldNowUtcSec - oldEventUtcSec
                     : 0;
  return newNowUtcSec > age ? newNowUtcSec - age : 1;
}
