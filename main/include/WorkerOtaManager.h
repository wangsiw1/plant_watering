#pragma once

#include <stddef.h>
#include <stdint.h>

enum class WorkerOtaJobState : uint8_t {
  IDLE,
  RECEIVING_PACKAGE,
  STORED,
  CONNECTING,
  CHECKING,
  TRANSFERRING,
  VERIFYING,
  REBOOTING,
  DONE,
  FAILED
};

struct WorkerOtaStatusSnapshot {
  WorkerOtaJobState state;
  bool active;
  uint8_t targetMac[6];
  uint32_t packageBytesReceived;
  uint32_t imageBytesSent;
  uint32_t expectedImageBytes;
  uint32_t packageVersion;
  uint16_t packageHardware;
  char message[80];
};

void workerOtaManagerInit();

bool workerOtaPackageUploadStart();
bool workerOtaPackageUploadConsume(const uint8_t* data, size_t len);
bool workerOtaPackageUploadFinish();
void workerOtaPackageUploadAbort(const char* reason);

bool workerOtaStartForWorker(const uint8_t mac[6]);
bool workerOtaIsActive();
void workerOtaGetStatus(WorkerOtaStatusSnapshot& out);
const char* workerOtaStateName(WorkerOtaJobState state);
void workerOtaManagerLoop();

