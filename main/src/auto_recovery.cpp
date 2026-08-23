#include "AutoRecovery.h"

#include <esp_attr.h>

namespace {
RTC_NOINIT_ATTR AutoRecoveryRecord gAutoRecoveryRecord;
}

bool autoRecoveryInitialize() {
  esp_reset_reason_t reason = esp_reset_reason();
  bool resume = autoRecoveryShouldResume(reason, gAutoRecoveryRecord);
  if (!resume) gAutoRecoveryRecord = autoRecoveryMakeRecord(false);
  return resume;
}

void autoRecoverySetArmed(bool armed) {
  gAutoRecoveryRecord = autoRecoveryMakeRecord(armed);
}

