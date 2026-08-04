#pragma once

#include <stddef.h>
#include <stdint.h>

enum class OtaUpdateState : uint8_t {
  IDLE,
  RECEIVING_HEADER,
  RECEIVING_IMAGE_PREFIX,
  WRITING,
  VERIFYING,
  READY_TO_REBOOT,
  FAILED
};

struct OtaStatusSnapshot {
  OtaUpdateState state;
  bool active;
  uint32_t packageBytesReceived;
  uint32_t imageBytesReceived;
  uint32_t expectedImageBytes;
  uint32_t firmwareVersion;
  uint16_t hardwareTarget;
  uint32_t freeHeapAtStart;
  uint32_t largestBlockAtStart;
  char error[48];
};

void otaManagerInit();
bool otaUploadStart();
bool otaUploadConsume(const uint8_t* data, size_t len);
bool otaUploadFinish();
void otaUploadAbort(const char* reason);
void otaReleaseFailedUpload();

bool otaIsActive();
bool otaUploadSucceeded();
void otaGetStatus(OtaStatusSnapshot& out);
const char* otaStateName(OtaUpdateState state);
void otaManagerLoop();

// Call only after the application has completed its bounded startup checks.
bool otaMarkRunningAppValid();

uint32_t otaCurrentFirmwareVersion();
uint16_t otaCurrentHardwareTarget();
