#pragma once

#include <stdint.h>
#include <esp_system.h>

constexpr uint32_t AUTO_RECOVERY_MAGIC = 0x4155544Fu;  // "AUTO"
constexpr uint32_t AUTO_RECOVERY_VERSION = 1u;
constexpr uint32_t AUTO_RECOVERY_CHECK_SEED = 0xA35F19C7u;

struct AutoRecoveryRecord {
  uint32_t magic;
  uint32_t version;
  uint32_t armed;
  uint32_t check;
};

inline uint32_t autoRecoveryRecordCheck(const AutoRecoveryRecord& record) {
  return record.magic ^ record.version ^ record.armed ^
         AUTO_RECOVERY_CHECK_SEED;
}

inline AutoRecoveryRecord autoRecoveryMakeRecord(bool armed) {
  AutoRecoveryRecord record{
      AUTO_RECOVERY_MAGIC,
      AUTO_RECOVERY_VERSION,
      armed ? 1u : 0u,
      0u,
  };
  record.check = autoRecoveryRecordCheck(record);
  return record;
}

inline bool autoRecoveryRecordIsValid(const AutoRecoveryRecord& record) {
  return record.magic == AUTO_RECOVERY_MAGIC &&
         record.version == AUTO_RECOVERY_VERSION &&
         record.armed <= 1u && record.check == autoRecoveryRecordCheck(record);
}

inline bool autoRecoveryResetIsEligible(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      return true;
    default:
      return false;
  }
}

inline bool autoRecoveryShouldResume(esp_reset_reason_t reason,
                                     const AutoRecoveryRecord& record) {
  return autoRecoveryResetIsEligible(reason) &&
         autoRecoveryRecordIsValid(record) && record.armed == 1u;
}

// Evaluate the retained record against the current reset reason. Call once at
// the beginning of setup, before deciding whether Wi-Fi provisioning may block.
bool autoRecoveryInitialize();

// Keep the retained intent synchronized with the runtime auto-mode state.
void autoRecoverySetArmed(bool armed);

