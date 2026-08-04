#include "ClockManager.h"

#include "ClockMath.h"
#include "Utility.h"
#include "config.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_sntp.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <sys/time.h>
#include <time.h>

#include <cstring>

namespace {

constexpr int64_t NTP_TIMEOUT_US = 10000000LL;
constexpr int64_t CHECKPOINT_INTERVAL_US = 6LL * 60 * 60 * 1000000;

SemaphoreHandle_t gClockMutex = nullptr;
uint64_t gClockBaseUtc = 0;
int64_t gClockBaseUs = 0;
ClockSource gClockSource = ClockSource::UNINITIALIZED;
bool gNtpSyncPending = false;
int64_t gNtpDeadlineUs = 0;
int64_t gLastCheckpointUs = 0;
char gNtpSyncError[32] = {};
uint32_t gNtpAttemptGeneration = 0;

portMUX_TYPE gNtpCallbackMux = portMUX_INITIALIZER_UNLOCKED;
bool gNtpCallbackReady = false;
uint64_t gNtpCallbackEpoch = 0;
uint32_t gNtpCallbackActiveGeneration = 0;
uint32_t gNtpCallbackGeneration = 0;

void ensureClockMutex() {
  if (!gClockMutex) gClockMutex = xSemaphoreCreateMutex();
}

uint64_t currentEpochLocked(int64_t nowUs) {
  if (gClockSource == ClockSource::UNINITIALIZED || gClockBaseUtc == 0) return 0;
  int64_t elapsedUs = nowUs - gClockBaseUs;
  if (elapsedUs < 0) elapsedUs = 0;
  return gClockBaseUtc + static_cast<uint64_t>(elapsedUs / 1000000LL);
}

void setSystemEpoch(uint64_t epochSec) {
  timeval value{};
  value.tv_sec = static_cast<time_t>(epochSec);
  settimeofday(&value, nullptr);
}

void setNtpErrorLocked(const char* error) {
  if (!error) error = "unknown";
  strncpy(gNtpSyncError, error, sizeof(gNtpSyncError) - 1);
  gNtpSyncError[sizeof(gNtpSyncError) - 1] = '\0';
}

void stopNtp();

bool applyEpochCorrection(uint64_t newEpochSec, ClockSource source,
                          bool saveImmediately,
                          uint32_t expectedNtpGeneration = 0) {
  if (!clockEpochIsSane(newEpochSec)) return false;
  ensureClockMutex();

  xSemaphoreTake(gClockMutex, portMAX_DELAY);
  if (expectedNtpGeneration != 0 &&
      (!gNtpSyncPending || gNtpAttemptGeneration != expectedNtpGeneration)) {
    xSemaphoreGive(gClockMutex);
    return false;
  }
  if (expectedNtpGeneration != 0) stopNtp();
  int64_t nowUs = esp_timer_get_time();
  uint64_t oldNowUtc = currentEpochLocked(nowUs);
  Settings settings{};
  getSettingsSnapshot(settings);
  uint64_t rebasedLastWatering = clockRebaseTimestampPreservingAge(
      oldNowUtc, newEpochSec, settings.lastWateringUtcSec);

  gClockBaseUtc = newEpochSec;
  gClockBaseUs = nowUs;
  gClockSource = source;
  gLastCheckpointUs = nowUs;
  if (expectedNtpGeneration != 0) {
    gNtpSyncPending = false;
    gNtpDeadlineUs = 0;
    gNtpSyncError[0] = '\0';
  }
  setSystemEpoch(newEpochSec);
  if (rebasedLastWatering != settings.lastWateringUtcSec) {
    setLastWateringUtc(rebasedLastWatering);
  }
  setSavedUtc(newEpochSec, saveImmediately);
  xSemaphoreGive(gClockMutex);
  LOG("Clock corrected source=%s utc=%llu last_watering=%llu",
      clockSourceName(source), static_cast<unsigned long long>(newEpochSec),
      static_cast<unsigned long long>(rebasedLastWatering));
  return true;
}

void ntpTimeAvailable(timeval* value) {
  if (!value || value->tv_sec <= 0) return;
  portENTER_CRITICAL(&gNtpCallbackMux);
  gNtpCallbackEpoch = static_cast<uint64_t>(value->tv_sec);
  gNtpCallbackGeneration = gNtpCallbackActiveGeneration;
  gNtpCallbackReady = true;
  portEXIT_CRITICAL(&gNtpCallbackMux);
}

bool takeNtpCallback(uint64_t& epochSec, uint32_t& generation) {
  portENTER_CRITICAL(&gNtpCallbackMux);
  bool ready = gNtpCallbackReady;
  if (ready) {
    epochSec = gNtpCallbackEpoch;
    generation = gNtpCallbackGeneration;
    gNtpCallbackReady = false;
    gNtpCallbackEpoch = 0;
    gNtpCallbackGeneration = 0;
  }
  portEXIT_CRITICAL(&gNtpCallbackMux);
  return ready;
}

void stopNtp() {
  esp_sntp_stop();
}

void cancelNtpAttempt(bool clearError) {
  ensureClockMutex();
  xSemaphoreTake(gClockMutex, portMAX_DELAY);
  bool stopActiveNtp = gNtpSyncPending;
  if (stopActiveNtp) stopNtp();
  gNtpSyncPending = false;
  gNtpDeadlineUs = 0;
  if (++gNtpAttemptGeneration == 0) ++gNtpAttemptGeneration;
  uint32_t generation = gNtpAttemptGeneration;
  if (clearError) gNtpSyncError[0] = '\0';
  portENTER_CRITICAL(&gNtpCallbackMux);
  gNtpCallbackActiveGeneration = generation;
  gNtpCallbackReady = false;
  gNtpCallbackEpoch = 0;
  gNtpCallbackGeneration = 0;
  portEXIT_CRITICAL(&gNtpCallbackMux);
  xSemaphoreGive(gClockMutex);
}

}  // namespace

void clockManagerInit() {
  ensureClockMutex();
  sntp_set_time_sync_notification_cb(ntpTimeAvailable);

  Settings settings{};
  getSettingsSnapshot(settings);
  time_t retained = time(nullptr);
  uint64_t retainedUtc = retained > 0 ? static_cast<uint64_t>(retained) : 0;
  ClockInitialSelection initial =
      clockSelectInitialEpoch(retainedUtc, settings.savedUtcSec);
  uint64_t initialUtc = initial.epochSec;
  ClockSource source =
      initial.restored ? ClockSource::RESTORED : ClockSource::FALLBACK;

  xSemaphoreTake(gClockMutex, portMAX_DELAY);
  gClockBaseUtc = initialUtc;
  gClockBaseUs = esp_timer_get_time();
  gClockSource = source;
  gNtpSyncPending = false;
  gNtpDeadlineUs = 0;
  gLastCheckpointUs = gClockBaseUs;
  gNtpSyncError[0] = '\0';
  setSystemEpoch(initialUtc);
  xSemaphoreGive(gClockMutex);

  if (settings.savedUtcSec != initialUtc) setSavedUtc(initialUtc, true);
  LOG("Clock initialized source=%s utc=%llu saved=%llu retained=%llu",
      clockSourceName(source), static_cast<unsigned long long>(initialUtc),
      static_cast<unsigned long long>(settings.savedUtcSec),
      static_cast<unsigned long long>(retainedUtc));
}

void clockManagerLoop() {
  ensureClockMutex();
  uint64_t ntpEpoch = 0;
  uint32_t ntpGeneration = 0;
  if (takeNtpCallback(ntpEpoch, ntpGeneration)) {
    if (clockNtpEpochIsSane(ntpEpoch)) {
      applyEpochCorrection(ntpEpoch, ClockSource::NTP, true, ntpGeneration);
    } else {
      xSemaphoreTake(gClockMutex, portMAX_DELAY);
      if (gNtpSyncPending && gNtpAttemptGeneration == ntpGeneration) {
        stopNtp();
        gNtpSyncPending = false;
        gNtpDeadlineUs = 0;
        setNtpErrorLocked("invalid_time");
      }
      xSemaphoreGive(gClockMutex);
    }
  }

  int64_t nowUs = esp_timer_get_time();
  bool timedOut = false;
  bool wifiLost = false;
  bool checkpointDue = false;
  xSemaphoreTake(gClockMutex, portMAX_DELAY);
  if (gNtpSyncPending && WiFi.status() != WL_CONNECTED) {
    stopNtp();
    gNtpSyncPending = false;
    gNtpDeadlineUs = 0;
    setNtpErrorLocked("wifi_disconnected");
    wifiLost = true;
  } else if (gNtpSyncPending && nowUs >= gNtpDeadlineUs) {
    stopNtp();
    gNtpSyncPending = false;
    gNtpDeadlineUs = 0;
    setNtpErrorLocked("timeout");
    timedOut = true;
  }
  checkpointDue = gClockSource != ClockSource::UNINITIALIZED &&
                  nowUs - gLastCheckpointUs >= CHECKPOINT_INTERVAL_US;
  xSemaphoreGive(gClockMutex);

  if (timedOut || wifiLost) {
    LOG("Clock NTP failed reason=%s", timedOut ? "timeout" : "wifi_disconnected");
  }
  if (checkpointDue) clockCheckpointNow();
}

bool clockRequestNtpSync() {
  if (WiFi.status() != WL_CONNECTED) return false;
  ensureClockMutex();
  xSemaphoreTake(gClockMutex, portMAX_DELAY);
  if (gNtpSyncPending) {
    xSemaphoreGive(gClockMutex);
    return true;
  }
  gNtpSyncPending = true;
  gNtpDeadlineUs = esp_timer_get_time() + NTP_TIMEOUT_US;
  gNtpSyncError[0] = '\0';
  if (++gNtpAttemptGeneration == 0) ++gNtpAttemptGeneration;
  uint32_t generation = gNtpAttemptGeneration;
  portENTER_CRITICAL(&gNtpCallbackMux);
  gNtpCallbackActiveGeneration = generation;
  gNtpCallbackReady = false;
  gNtpCallbackEpoch = 0;
  gNtpCallbackGeneration = 0;
  portEXIT_CRITICAL(&gNtpCallbackMux);
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  xSemaphoreGive(gClockMutex);
  LOG("Clock NTP requested");
  return true;
}

void clockResetToDefault() {
  cancelNtpAttempt(true);
  applyEpochCorrection(CLOCK_DEFAULT_EPOCH_SEC, ClockSource::FALLBACK, true);
}

void clockCheckpointNow() {
  ensureClockMutex();
  xSemaphoreTake(gClockMutex, portMAX_DELAY);
  uint64_t utc = currentEpochLocked(esp_timer_get_time());
  if (utc == 0) {
    xSemaphoreGive(gClockMutex);
    return;
  }
  setSavedUtc(utc, true);
  gLastCheckpointUs = esp_timer_get_time();
  xSemaphoreGive(gClockMutex);
  LOG("Clock checkpoint utc=%llu", static_cast<unsigned long long>(utc));
}

bool clockSetUserTimeOfDaySec(uint32_t secOfDay) {
  if (secOfDay >= 86400) return false;
  cancelNtpAttempt(false);
  Settings settings{};
  getSettingsSnapshot(settings);
  uint64_t referenceUtc = clockGetCurrentEpochSec();
  uint64_t newUtc = 0;
  if (!clockEpochForLocalTimeOfDay(referenceUtc, settings.tzOffsetMinutes,
                                   secOfDay, newUtc)) {
    return false;
  }
  applyEpochCorrection(newUtc, ClockSource::MANUAL, true);
  return true;
}

uint32_t clockGetCurrentTimeOfDaySec() {
  uint64_t utc = clockGetCurrentEpochSec();
  if (utc == 0) return 0;
  Settings settings{};
  getSettingsSnapshot(settings);
  return clockLocalTimeOfDaySec(utc, settings.tzOffsetMinutes);
}

uint64_t clockGetCurrentEpochSec() {
  ensureClockMutex();
  xSemaphoreTake(gClockMutex, portMAX_DELAY);
  uint64_t utc = currentEpochLocked(esp_timer_get_time());
  xSemaphoreGive(gClockMutex);
  return utc;
}

bool clockIsValid() {
  ensureClockMutex();
  xSemaphoreTake(gClockMutex, portMAX_DELAY);
  bool valid = gClockSource != ClockSource::UNINITIALIZED && gClockBaseUtc != 0;
  xSemaphoreGive(gClockMutex);
  return valid;
}

void clockRecordLastWateringNow() {
  ensureClockMutex();
  xSemaphoreTake(gClockMutex, portMAX_DELAY);
  uint64_t utc = currentEpochLocked(esp_timer_get_time());
  if (utc != 0) setLastWateringUtc(utc);
  xSemaphoreGive(gClockMutex);
}

void clockGetStatus(ClockStatusSnapshot& out) {
  ensureClockMutex();
  xSemaphoreTake(gClockMutex, portMAX_DELAY);
  out.valid = gClockSource != ClockSource::UNINITIALIZED && gClockBaseUtc != 0;
  out.source = gClockSource;
  out.ntpSyncPending = gNtpSyncPending;
  memcpy(out.ntpSyncError, gNtpSyncError, sizeof(out.ntpSyncError));
  out.ntpSyncError[sizeof(out.ntpSyncError) - 1] = '\0';
  xSemaphoreGive(gClockMutex);
}

const char* clockSourceName(ClockSource source) {
  switch (source) {
    case ClockSource::FALLBACK: return "default";
    case ClockSource::RESTORED: return "restored";
    case ClockSource::MANUAL: return "manual";
    case ClockSource::NTP: return "ntp";
    case ClockSource::UNINITIALIZED:
    default: return "uninitialized";
  }
}
