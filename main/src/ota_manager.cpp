#include "OtaManager.h"

#include "FirmwarePackage.h"
#include "FirmwareVersionPolicy.h"
#include "ClockManager.h"
#include "Utility.h"

#include <Arduino.h>
#include <cstring>
#include <esp_app_format.h>
#include <esp_ota_ops.h>
#include <freertos/FreeRTOS.h>
#include <mbedtls/sha256.h>

#ifndef FW_VERSION
#error "FW_VERSION must be supplied by PlatformIO build_flags"
#endif

#ifndef FW_MIN_ALLOWED_VERSION
#error "FW_MIN_ALLOWED_VERSION must be supplied by PlatformIO build_flags"
#endif

#if FW_MIN_ALLOWED_VERSION < 1
#error "FW_MIN_ALLOWED_VERSION must be greater than zero"
#endif

#if FW_MIN_ALLOWED_VERSION > FW_VERSION
#error "FW_MIN_ALLOWED_VERSION must not exceed FW_VERSION"
#endif

#ifndef FW_HARDWARE_TARGET
#error "FW_HARDWARE_TARGET must be supplied by PlatformIO build_flags"
#endif

extern "C" bool verifyRollbackLater() {
  return true;
}

namespace {

constexpr uint32_t MIN_FREE_HEAP_TO_START = 50 * 1024;
constexpr uint32_t MIN_LARGEST_BLOCK_TO_START = 24 * 1024;
constexpr uint32_t REBOOT_DELAY_MS = 2000;
constexpr size_t IMAGE_PREFIX_SIZE = sizeof(esp_image_header_t);

portMUX_TYPE gStatusMux = portMUX_INITIALIZER_UNLOCKED;
OtaStatusSnapshot gStatus{};

uint8_t gPackageHeader[FirmwarePackage::HEADER_SIZE] = {};
size_t gPackageHeaderReceived = 0;
FirmwarePackage::Header gParsedHeader{};
uint8_t gImagePrefix[IMAGE_PREFIX_SIZE] = {};
size_t gImagePrefixReceived = 0;
const esp_partition_t* gTargetPartition = nullptr;
esp_ota_handle_t gOtaHandle = 0;
bool gOtaBegun = false;
mbedtls_sha256_context gSha256;
bool gSha256Active = false;
uint32_t gRebootAtMs = 0;
bool gClockCheckpointedForReboot = false;

void copyError(char out[48], const char* error) {
  if (!error) error = "unknown";
  strncpy(out, error, 47);
  out[47] = '\0';
}

void updateState(OtaUpdateState state, const char* error = nullptr) {
  portENTER_CRITICAL(&gStatusMux);
  gStatus.state = state;
  if (error) copyError(gStatus.error, error);
  portEXIT_CRITICAL(&gStatusMux);
}

void updateProgress(uint32_t packageBytes, uint32_t imageBytes) {
  portENTER_CRITICAL(&gStatusMux);
  gStatus.packageBytesReceived = packageBytes;
  gStatus.imageBytesReceived = imageBytes;
  portEXIT_CRITICAL(&gStatusMux);
}

void freeSha256() {
  if (!gSha256Active) return;
  mbedtls_sha256_free(&gSha256);
  gSha256Active = false;
}

void abortOtaHandle() {
  if (!gOtaBegun) return;
  esp_ota_abort(gOtaHandle);
  gOtaBegun = false;
  gOtaHandle = 0;
}

void fail(const char* reason) {
  abortOtaHandle();
  freeSha256();
  updateState(OtaUpdateState::FAILED, reason);
  LOG("OTA failed reason=%s image_bytes=%lu expected=%lu free_heap=%lu largest=%lu",
      reason ? reason : "unknown",
      static_cast<unsigned long>(gStatus.imageBytesReceived),
      static_cast<unsigned long>(gStatus.expectedImageBytes),
      static_cast<unsigned long>(ESP.getFreeHeap()),
      static_cast<unsigned long>(ESP.getMaxAllocHeap()));
}

void resetSession() {
  abortOtaHandle();
  freeSha256();
  memset(gPackageHeader, 0, sizeof(gPackageHeader));
  memset(&gParsedHeader, 0, sizeof(gParsedHeader));
  memset(gImagePrefix, 0, sizeof(gImagePrefix));
  gPackageHeaderReceived = 0;
  gImagePrefixReceived = 0;
  gTargetPartition = nullptr;
  gRebootAtMs = 0;
  gClockCheckpointedForReboot = false;

  OtaStatusSnapshot status{};
  status.state = OtaUpdateState::RECEIVING_HEADER;
  status.active = true;
  status.freeHeapAtStart = ESP.getFreeHeap();
  status.largestBlockAtStart = ESP.getMaxAllocHeap();
  portENTER_CRITICAL(&gStatusMux);
  gStatus = status;
  portEXIT_CRITICAL(&gStatusMux);
}

bool statusIsFailed() {
  portENTER_CRITICAL(&gStatusMux);
  bool failed = gStatus.state == OtaUpdateState::FAILED;
  portEXIT_CRITICAL(&gStatusMux);
  return failed;
}

bool beginImageWrite() {
  esp_image_header_t imageHeader{};
  memcpy(&imageHeader, gImagePrefix, sizeof(imageHeader));
  if (imageHeader.magic != ESP_IMAGE_HEADER_MAGIC) {
    fail("bad_esp_image_magic");
    return false;
  }
  if (imageHeader.chip_id != ESP_CHIP_ID_ESP32C3) {
    fail("esp_image_wrong_chip");
    return false;
  }
  if (imageHeader.segment_count == 0 || imageHeader.segment_count > 16) {
    fail("bad_esp_segment_count");
    return false;
  }
  if (imageHeader.hash_appended != 1) {
    fail("esp_image_hash_missing");
    return false;
  }

  mbedtls_sha256_init(&gSha256);
  gSha256Active = true;
  if (mbedtls_sha256_starts_ret(&gSha256, 0) != 0) {
    fail("sha256_init_failed");
    return false;
  }
  esp_err_t err = esp_ota_begin(gTargetPartition, gParsedHeader.imageSize,
                                &gOtaHandle);
  if (err != ESP_OK) {
    fail("ota_begin_failed");
    return false;
  }
  gOtaBegun = true;
  updateState(OtaUpdateState::WRITING);
  LOG("OTA write begin partition=%s image_size=%lu version=%lu hardware=%u",
      gTargetPartition->label,
      static_cast<unsigned long>(gParsedHeader.imageSize),
      static_cast<unsigned long>(gParsedHeader.firmwareVersion),
      static_cast<unsigned>(gParsedHeader.hardwareTarget));
  return true;
}

bool writeImageData(const uint8_t* data, size_t len) {
  if (len == 0) return true;
  uint32_t received = gStatus.imageBytesReceived;
  if (received > gParsedHeader.imageSize ||
      len > static_cast<size_t>(gParsedHeader.imageSize - received)) {
    fail("image_larger_than_declared");
    return false;
  }
  if (mbedtls_sha256_update_ret(&gSha256, data, len) != 0) {
    fail("sha256_update_failed");
    return false;
  }
  if (esp_ota_write(gOtaHandle, data, len) != ESP_OK) {
    fail("ota_write_failed");
    return false;
  }
  updateProgress(gStatus.packageBytesReceived, received + len);
  return true;
}

bool parsePackageHeader() {
  gTargetPartition = esp_ota_get_next_update_partition(nullptr);
  if (!gTargetPartition) {
    fail("no_inactive_ota_partition");
    return false;
  }
  FirmwarePackage::ParseError error = FirmwarePackage::parseMainHeader(
      gPackageHeader, sizeof(gPackageHeader), gTargetPartition->size,
      gParsedHeader);
  if (error != FirmwarePackage::ParseError::NONE) {
    fail(FirmwarePackage::parseErrorName(error));
    return false;
  }
  switch (firmwareVersionDecision(FW_VERSION,
                                  gParsedHeader.firmwareVersion,
                                  FW_MIN_ALLOWED_VERSION)) {
    case FirmwareVersionDecision::ALREADY_INSTALLED:
      fail("already_up_to_date");
      return false;
    case FirmwareVersionDecision::BELOW_MINIMUM:
      fail("version_below_minimum");
      return false;
    case FirmwareVersionDecision::ALLOW:
      break;
  }
  if (gParsedHeader.imageSize < IMAGE_PREFIX_SIZE) {
    fail("image_too_small");
    return false;
  }

  portENTER_CRITICAL(&gStatusMux);
  gStatus.expectedImageBytes = gParsedHeader.imageSize;
  gStatus.firmwareVersion = gParsedHeader.firmwareVersion;
  gStatus.hardwareTarget = gParsedHeader.hardwareTarget;
  gStatus.state = OtaUpdateState::RECEIVING_IMAGE_PREFIX;
  portEXIT_CRITICAL(&gStatusMux);
  return true;
}

}  // namespace

void otaManagerInit() {
  OtaStatusSnapshot status{};
  status.state = OtaUpdateState::IDLE;
  portENTER_CRITICAL(&gStatusMux);
  gStatus = status;
  portEXIT_CRITICAL(&gStatusMux);
}

bool otaUploadStart() {
  portENTER_CRITICAL(&gStatusMux);
  bool alreadyActive = gStatus.active;
  portEXIT_CRITICAL(&gStatusMux);
  if (alreadyActive) return false;

  resetSession();
  if (gStatus.freeHeapAtStart < MIN_FREE_HEAP_TO_START) {
    fail("insufficient_free_heap");
    return false;
  }
  if (gStatus.largestBlockAtStart < MIN_LARGEST_BLOCK_TO_START) {
    fail("largest_heap_block_too_small");
    return false;
  }
  LOG("OTA upload start free_heap=%lu largest=%lu",
      static_cast<unsigned long>(gStatus.freeHeapAtStart),
      static_cast<unsigned long>(gStatus.largestBlockAtStart));
  return true;
}

bool otaUploadConsume(const uint8_t* data, size_t len) {
  if (!data || len == 0 || statusIsFailed()) return !statusIsFailed();

  uint32_t packageBytes = gStatus.packageBytesReceived;
  if (len > UINT32_MAX - packageBytes) {
    fail("package_size_overflow");
    return false;
  }
  updateProgress(packageBytes + len, gStatus.imageBytesReceived);

  while (len > 0 && !statusIsFailed()) {
    if (gPackageHeaderReceived < sizeof(gPackageHeader)) {
      size_t take = min(len, sizeof(gPackageHeader) - gPackageHeaderReceived);
      memcpy(gPackageHeader + gPackageHeaderReceived, data, take);
      gPackageHeaderReceived += take;
      data += take;
      len -= take;
      if (gPackageHeaderReceived == sizeof(gPackageHeader) &&
          !parsePackageHeader()) {
        return false;
      }
      continue;
    }

    if (gImagePrefixReceived < sizeof(gImagePrefix)) {
      size_t take = min(len, sizeof(gImagePrefix) - gImagePrefixReceived);
      memcpy(gImagePrefix + gImagePrefixReceived, data, take);
      gImagePrefixReceived += take;
      data += take;
      len -= take;
      if (gImagePrefixReceived == sizeof(gImagePrefix)) {
        if (!beginImageWrite() ||
            !writeImageData(gImagePrefix, sizeof(gImagePrefix))) {
          return false;
        }
      }
      continue;
    }

    return writeImageData(data, len);
  }
  return !statusIsFailed();
}

bool otaUploadFinish() {
  if (statusIsFailed()) return false;
  if (!gOtaBegun || gPackageHeaderReceived != sizeof(gPackageHeader) ||
      gImagePrefixReceived != sizeof(gImagePrefix)) {
    fail("incomplete_upload");
    return false;
  }
  if (gStatus.imageBytesReceived != gParsedHeader.imageSize ||
      gStatus.packageBytesReceived !=
          FirmwarePackage::HEADER_SIZE + gParsedHeader.imageSize) {
    fail("image_size_mismatch");
    return false;
  }

  updateState(OtaUpdateState::VERIFYING);
  uint8_t calculatedSha256[32];
  if (mbedtls_sha256_finish_ret(&gSha256, calculatedSha256) != 0) {
    fail("sha256_finish_failed");
    return false;
  }
  freeSha256();
  if (memcmp(calculatedSha256, gParsedHeader.imageSha256,
             sizeof(calculatedSha256)) != 0) {
    fail("sha256_mismatch");
    return false;
  }

  esp_err_t err = esp_ota_end(gOtaHandle);
  gOtaBegun = false;
  gOtaHandle = 0;
  if (err != ESP_OK) {
    fail("ota_image_validation_failed");
    return false;
  }
  if (esp_ota_set_boot_partition(gTargetPartition) != ESP_OK) {
    fail("set_boot_partition_failed");
    return false;
  }

  updateState(OtaUpdateState::READY_TO_REBOOT);
  gRebootAtMs = millis() + REBOOT_DELAY_MS;
  LOG("OTA verified version=%lu image_size=%lu reboot_ms=%lu free_heap=%lu largest=%lu",
      static_cast<unsigned long>(gParsedHeader.firmwareVersion),
      static_cast<unsigned long>(gParsedHeader.imageSize),
      static_cast<unsigned long>(gRebootAtMs),
      static_cast<unsigned long>(ESP.getFreeHeap()),
      static_cast<unsigned long>(ESP.getMaxAllocHeap()));
  return true;
}

void otaUploadAbort(const char* reason) {
  if (otaUploadSucceeded()) return;
  fail(reason ? reason : "upload_aborted");
}

void otaReleaseFailedUpload() {
  portENTER_CRITICAL(&gStatusMux);
  if (gStatus.state == OtaUpdateState::FAILED) gStatus.active = false;
  portEXIT_CRITICAL(&gStatusMux);
}

bool otaIsActive() {
  portENTER_CRITICAL(&gStatusMux);
  bool active = gStatus.active;
  portEXIT_CRITICAL(&gStatusMux);
  return active;
}

bool otaUploadSucceeded() {
  portENTER_CRITICAL(&gStatusMux);
  bool succeeded = gStatus.state == OtaUpdateState::READY_TO_REBOOT;
  portEXIT_CRITICAL(&gStatusMux);
  return succeeded;
}

void otaGetStatus(OtaStatusSnapshot& out) {
  portENTER_CRITICAL(&gStatusMux);
  out = gStatus;
  portEXIT_CRITICAL(&gStatusMux);
}

const char* otaStateName(OtaUpdateState state) {
  switch (state) {
    case OtaUpdateState::IDLE: return "idle";
    case OtaUpdateState::RECEIVING_HEADER: return "receiving_header";
    case OtaUpdateState::RECEIVING_IMAGE_PREFIX: return "receiving_image_prefix";
    case OtaUpdateState::WRITING: return "writing";
    case OtaUpdateState::VERIFYING: return "verifying";
    case OtaUpdateState::READY_TO_REBOOT: return "ready_to_reboot";
    case OtaUpdateState::FAILED: return "failed";
    default: return "unknown";
  }
}

void otaManagerLoop() {
  if (!otaUploadSucceeded() || gRebootAtMs == 0) return;
  if (static_cast<int32_t>(millis() - gRebootAtMs) < 0) return;
  if (!gClockCheckpointedForReboot) {
    clockCheckpointNow();
    gClockCheckpointedForReboot = true;
  }
  ESP.restart();
}

bool otaMarkRunningAppValid() {
  const esp_partition_t* running = esp_ota_get_running_partition();
  if (!running) return false;
  esp_ota_img_states_t state;
  esp_err_t err = esp_ota_get_state_partition(running, &state);
  if (err == ESP_ERR_NOT_SUPPORTED || err == ESP_ERR_NOT_FOUND) return true;
  if (err != ESP_OK) {
    LOG("OTA boot state read failed err=%d", static_cast<int>(err));
    return false;
  }
  if (state != ESP_OTA_IMG_PENDING_VERIFY) return true;
  err = esp_ota_mark_app_valid_cancel_rollback();
  LOG("OTA running app validation result=%d", static_cast<int>(err));
  return err == ESP_OK;
}

uint32_t otaCurrentFirmwareVersion() {
  return FW_VERSION;
}

uint16_t otaCurrentHardwareTarget() {
  return FW_HARDWARE_TARGET;
}
