#include "WorkerOtaService.h"

#include "Utility.h"
#include "Valve.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEExtAdvertising.h>
#include <cstring>
#include <esp_app_format.h>
#include <esp_ota_ops.h>
#include <mbedtls/sha256.h>

#ifndef FW_VERSION
#error "FW_VERSION must be supplied by PlatformIO build_flags"
#endif

#ifndef FW_HARDWARE_TARGET
#error "FW_HARDWARE_TARGET must be supplied by PlatformIO build_flags"
#endif

namespace {

constexpr char OTA_SERVICE_UUID[] = "f3641400-00b0-4240-ba50-05ca45bf8abc";
constexpr char OTA_INFO_UUID[] = "f3641401-00b0-4240-ba50-05ca45bf8abc";
constexpr char OTA_CONTROL_UUID[] = "f3641402-00b0-4240-ba50-05ca45bf8abc";
constexpr char OTA_DATA_UUID[] = "f3641403-00b0-4240-ba50-05ca45bf8abc";

constexpr uint8_t PACKAGE_MAGIC[8] = {'P', 'W', 'O', 'T', 'A', '0', '0', '1'};
constexpr uint16_t PACKAGE_FORMAT_VERSION = 1;
constexpr uint8_t PACKAGE_ROLE_WORKER = 2;
constexpr uint8_t PACKAGE_CHIP_ESP32_C3 = 1;
constexpr size_t PACKAGE_HEADER_SIZE = 64;
constexpr size_t IMAGE_PREFIX_SIZE = sizeof(esp_image_header_t);
constexpr uint32_t REBOOT_DELAY_MS = 1000;
constexpr uint8_t OTA_ADV_INSTANCE = 1;

enum class WorkerOtaState : uint8_t {
  IDLE = 0,
  PREPARED = 1,
  WRITING = 2,
  VERIFYING = 3,
  READY_TO_REBOOT = 4,
  FAILED = 5
};

enum class WorkerOtaError : uint8_t {
  NONE = 0,
  BAD_COMMAND = 1,
  BAD_HEADER = 2,
  WRONG_HARDWARE = 3,
  ALREADY_UP_TO_DATE = 4,
  NO_PARTITION = 5,
  TOO_LARGE = 6,
  BAD_IMAGE = 7,
  BAD_OFFSET = 8,
  WRITE_FAILED = 9,
  HASH_MISMATCH = 10,
  VALIDATION_FAILED = 11,
  SET_BOOT_FAILED = 12,
  ABORTED = 13
};

struct PackageHeader {
  uint8_t role;
  uint16_t hardwareTarget;
  uint32_t firmwareVersion;
  uint32_t imageSize;
  uint8_t imageSha256[32];
};

NimBLEServer* gServer = nullptr;
NimBLECharacteristic* gInfoCharacteristic = nullptr;
NimBLECharacteristic* gControlCharacteristic = nullptr;
NimBLECharacteristic* gDataCharacteristic = nullptr;
uint8_t gAllowedMainMac[6] = {};
uint32_t gWindowUntilMs = 0;
bool gAdvertisingWindow = false;
bool gConnected = false;

portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
WorkerOtaState gState = WorkerOtaState::IDLE;
WorkerOtaError gError = WorkerOtaError::NONE;
PackageHeader gHeader{};
uint8_t gImagePrefix[IMAGE_PREFIX_SIZE] = {};
size_t gImagePrefixReceived = 0;
uint32_t gImageBytesReceived = 0;
const esp_partition_t* gTargetPartition = nullptr;
esp_ota_handle_t gOtaHandle = 0;
bool gOtaBegun = false;
mbedtls_sha256_context gSha256;
bool gSha256Active = false;
uint32_t gRebootAtMs = 0;

uint16_t readU16(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t readU32(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

void writeU16(uint8_t* data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8);
}

void writeU32(uint8_t* data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8);
  data[2] = static_cast<uint8_t>(value >> 16);
  data[3] = static_cast<uint8_t>(value >> 24);
}

void setState(WorkerOtaState state, WorkerOtaError error = WorkerOtaError::NONE) {
  portENTER_CRITICAL(&gMux);
  gState = state;
  gError = error;
  portEXIT_CRITICAL(&gMux);
}

void freeSha256() {
  if (!gSha256Active) return;
  mbedtls_sha256_free(&gSha256);
  gSha256Active = false;
}

void abortOta() {
  if (gOtaBegun) {
    esp_ota_abort(gOtaHandle);
    gOtaBegun = false;
    gOtaHandle = 0;
  }
  freeSha256();
}

void resetSession() {
  abortOta();
  memset(&gHeader, 0, sizeof(gHeader));
  memset(gImagePrefix, 0, sizeof(gImagePrefix));
  gImagePrefixReceived = 0;
  gImageBytesReceived = 0;
  gTargetPartition = nullptr;
  gRebootAtMs = 0;
  setState(WorkerOtaState::IDLE);
}

void fail(WorkerOtaError error) {
  abortOta();
  setState(WorkerOtaState::FAILED, error);
  LOG("Worker OTA failed error=%u received=%lu expected=%lu",
      static_cast<unsigned>(error),
      static_cast<unsigned long>(gImageBytesReceived),
      static_cast<unsigned long>(gHeader.imageSize));
}

bool parsePackageHeader(const uint8_t* data, size_t len, PackageHeader& out) {
  if (!data || len != PACKAGE_HEADER_SIZE) return false;
  if (memcmp(data, PACKAGE_MAGIC, sizeof(PACKAGE_MAGIC)) != 0) return false;
  if (readU16(data + 8) != PACKAGE_FORMAT_VERSION) return false;
  if (readU16(data + 10) != PACKAGE_HEADER_SIZE) return false;
  out = {};
  out.role = data[12];
  uint8_t chip = data[13];
  out.hardwareTarget = readU16(data + 14);
  out.firmwareVersion = readU32(data + 16);
  out.imageSize = readU32(data + 20);
  memcpy(out.imageSha256, data + 24, sizeof(out.imageSha256));
  for (size_t i = 56; i < PACKAGE_HEADER_SIZE; ++i) {
    if (data[i] != 0) return false;
  }
  return out.role == PACKAGE_ROLE_WORKER &&
         chip == PACKAGE_CHIP_ESP32_C3 &&
         out.firmwareVersion != 0 &&
         out.imageSize >= IMAGE_PREFIX_SIZE;
}

bool beginImageWriteIfReady() {
  if (gImagePrefixReceived < sizeof(gImagePrefix) || gOtaBegun) return true;
  esp_image_header_t imageHeader{};
  memcpy(&imageHeader, gImagePrefix, sizeof(imageHeader));
  if (imageHeader.magic != ESP_IMAGE_HEADER_MAGIC ||
      imageHeader.chip_id != ESP_CHIP_ID_ESP32C3 ||
      imageHeader.segment_count == 0 || imageHeader.segment_count > 16 ||
      imageHeader.hash_appended != 1) {
    fail(WorkerOtaError::BAD_IMAGE);
    return false;
  }
  mbedtls_sha256_init(&gSha256);
  gSha256Active = true;
  if (mbedtls_sha256_starts(&gSha256, 0) != 0) {
    fail(WorkerOtaError::BAD_IMAGE);
    return false;
  }
  esp_err_t err = esp_ota_begin(gTargetPartition, gHeader.imageSize, &gOtaHandle);
  if (err != ESP_OK) {
    fail(WorkerOtaError::WRITE_FAILED);
    return false;
  }
  gOtaBegun = true;
  setState(WorkerOtaState::WRITING);
  return true;
}

bool writeImageBytes(const uint8_t* data, size_t len) {
  if (!data || len == 0) return true;
  if (gImageBytesReceived > gHeader.imageSize ||
      len > static_cast<size_t>(gHeader.imageSize - gImageBytesReceived)) {
    fail(WorkerOtaError::BAD_OFFSET);
    return false;
  }

  while (len > 0 && gImagePrefixReceived < sizeof(gImagePrefix)) {
    size_t take = min(len, sizeof(gImagePrefix) - gImagePrefixReceived);
    memcpy(gImagePrefix + gImagePrefixReceived, data, take);
    gImagePrefixReceived += take;
    data += take;
    len -= take;
    if (!beginImageWriteIfReady()) return false;
    if (gOtaBegun && !writeImageBytes(gImagePrefix, sizeof(gImagePrefix))) {
      return false;
    }
  }

  if (len == 0) return true;
  if (!gOtaBegun) return true;
  if (mbedtls_sha256_update(&gSha256, data, len) != 0 ||
      esp_ota_write(gOtaHandle, data, len) != ESP_OK) {
    fail(WorkerOtaError::WRITE_FAILED);
    return false;
  }
  gImageBytesReceived += len;
  return true;
}

void handlePrepare(const uint8_t* data, size_t len) {
  if (len != 1 + PACKAGE_HEADER_SIZE) {
    fail(WorkerOtaError::BAD_COMMAND);
    return;
  }
  resetSession();
  if (!parsePackageHeader(data + 1, PACKAGE_HEADER_SIZE, gHeader)) {
    fail(WorkerOtaError::BAD_HEADER);
    return;
  }
  if (gHeader.hardwareTarget != FW_HARDWARE_TARGET) {
    fail(WorkerOtaError::WRONG_HARDWARE);
    return;
  }
  if (gHeader.firmwareVersion <= FW_VERSION) {
    fail(WorkerOtaError::ALREADY_UP_TO_DATE);
    return;
  }
  gTargetPartition = esp_ota_get_next_update_partition(nullptr);
  if (!gTargetPartition) {
    fail(WorkerOtaError::NO_PARTITION);
    return;
  }
  if (gHeader.imageSize > gTargetPartition->size) {
    fail(WorkerOtaError::TOO_LARGE);
    return;
  }
  valveSetMask(0);
  setState(WorkerOtaState::PREPARED);
  LOG("Worker OTA prepared version=%lu hardware=%u image=%lu partition=%s",
      static_cast<unsigned long>(gHeader.firmwareVersion),
      static_cast<unsigned>(gHeader.hardwareTarget),
      static_cast<unsigned long>(gHeader.imageSize),
      gTargetPartition->label);
}

void handleCommit() {
  if (!gOtaBegun || gImageBytesReceived != gHeader.imageSize) {
    fail(WorkerOtaError::BAD_OFFSET);
    return;
  }
  setState(WorkerOtaState::VERIFYING);
  uint8_t calculated[32];
  if (mbedtls_sha256_finish(&gSha256, calculated) != 0) {
    fail(WorkerOtaError::VALIDATION_FAILED);
    return;
  }
  freeSha256();
  if (memcmp(calculated, gHeader.imageSha256, sizeof(calculated)) != 0) {
    fail(WorkerOtaError::HASH_MISMATCH);
    return;
  }
  esp_err_t err = esp_ota_end(gOtaHandle);
  gOtaBegun = false;
  gOtaHandle = 0;
  if (err != ESP_OK) {
    fail(WorkerOtaError::VALIDATION_FAILED);
    return;
  }
  if (esp_ota_set_boot_partition(gTargetPartition) != ESP_OK) {
    fail(WorkerOtaError::SET_BOOT_FAILED);
    return;
  }
  setState(WorkerOtaState::READY_TO_REBOOT);
  gRebootAtMs = millis() + REBOOT_DELAY_MS;
}

void writeInfoValue() {
  uint8_t info[28] = {};
  memcpy(info, "PWOTAI01", 8);
  portENTER_CRITICAL(&gMux);
  WorkerOtaState state = gState;
  WorkerOtaError error = gError;
  uint32_t received = gImageBytesReceived;
  portEXIT_CRITICAL(&gMux);
  info[8] = PACKAGE_ROLE_WORKER;
  info[9] = static_cast<uint8_t>(state);
  info[10] = static_cast<uint8_t>(error);
  info[11] = 0;
  writeU16(info + 12, FW_HARDWARE_TARGET);
  writeU16(info + 14, 0);
  writeU32(info + 16, FW_VERSION);
  const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
  writeU32(info + 20, next ? next->size : 0);
  writeU32(info + 24, received);
  if (gInfoCharacteristic) gInfoCharacteristic->setValue(info, sizeof(info));
}

class InfoCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic*, NimBLEConnInfo&) override {
    writeInfoValue();
  }
};

class ControlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo&) override {
    std::string value = characteristic->getValue();
    if (value.empty()) {
      fail(WorkerOtaError::BAD_COMMAND);
      return;
    }
    const uint8_t* data = reinterpret_cast<const uint8_t*>(value.data());
    switch (data[0]) {
      case 1:
        handlePrepare(data, value.size());
        break;
      case 2:
        handleCommit();
        break;
      case 3:
        fail(WorkerOtaError::ABORTED);
        break;
      default:
        fail(WorkerOtaError::BAD_COMMAND);
        break;
    }
    writeInfoValue();
  }
};

class DataCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo&) override {
    std::string value = characteristic->getValue();
    if (value.size() < 4) {
      fail(WorkerOtaError::BAD_COMMAND);
      return;
    }
    const uint8_t* data = reinterpret_cast<const uint8_t*>(value.data());
    uint32_t offset = readU32(data);
    if (offset != gImageBytesReceived) {
      fail(WorkerOtaError::BAD_OFFSET);
      return;
    }
    writeImageBytes(data + 4, value.size() - 4);
    writeInfoValue();
  }
};

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    (void)server;
    gConnected = true;
    connInfo.getAddress().toString();
    LOG("Worker OTA GATT connected");
  }

  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) override {
    gConnected = false;
    LOG("Worker OTA GATT disconnected reason=%d", reason);
    portENTER_CRITICAL(&gMux);
    WorkerOtaState state = gState;
    portEXIT_CRITICAL(&gMux);
    if (state == WorkerOtaState::PREPARED ||
        state == WorkerOtaState::WRITING ||
        state == WorkerOtaState::VERIFYING) {
      fail(WorkerOtaError::ABORTED);
    }
  }
};

InfoCallbacks gInfoCallbacks;
ControlCallbacks gControlCallbacks;
DataCallbacks gDataCallbacks;
ServerCallbacks gServerCallbacks;

}  // namespace

void workerOtaServiceBegin() {
  if (gServer) return;
  NimBLEDevice::setMTU(247);
  gServer = NimBLEDevice::createServer();
  gServer->setCallbacks(&gServerCallbacks);
  NimBLEService* service = gServer->createService(OTA_SERVICE_UUID);
  gInfoCharacteristic =
      service->createCharacteristic(OTA_INFO_UUID, NIMBLE_PROPERTY::READ, 32);
  gControlCharacteristic =
      service->createCharacteristic(OTA_CONTROL_UUID, NIMBLE_PROPERTY::WRITE, 80);
  gDataCharacteristic =
      service->createCharacteristic(OTA_DATA_UUID,
                                    NIMBLE_PROPERTY::WRITE |
                                        NIMBLE_PROPERTY::WRITE_NR,
                                    244);
  gInfoCharacteristic->setCallbacks(&gInfoCallbacks);
  gControlCharacteristic->setCallbacks(&gControlCallbacks);
  gDataCharacteristic->setCallbacks(&gDataCallbacks);
  writeInfoValue();
  gServer->start();
}

void workerOtaOpenWindow(const uint8_t allowedMainMac[6], uint32_t windowMs) {
  if (!gServer) workerOtaServiceBegin();
  if (allowedMainMac) memcpy(gAllowedMainMac, allowedMainMac, 6);
  gWindowUntilMs = millis() + windowMs;
  gAdvertisingWindow = true;
  NimBLEDevice::getScan()->stop();
  NimBLEExtAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->stop(OTA_ADV_INSTANCE);
  advertising->removeInstance(OTA_ADV_INSTANCE);
  NimBLEExtAdvertisement advert;
  advert.setConnectable(true);
  advert.setScannable(false);
  advert.setName("plant-worker-ota");
  advert.setCompleteServices(NimBLEUUID(OTA_SERVICE_UUID));
  if (advertising->setInstanceData(OTA_ADV_INSTANCE, advert)) {
    advertising->start(OTA_ADV_INSTANCE, windowMs);
  }
  LOG("Worker OTA GATT window opened ms=%lu",
      static_cast<unsigned long>(windowMs));
}

bool workerOtaIsActive() {
  portENTER_CRITICAL(&gMux);
  bool active = gState == WorkerOtaState::PREPARED ||
                gState == WorkerOtaState::WRITING ||
                gState == WorkerOtaState::VERIFYING ||
                gState == WorkerOtaState::READY_TO_REBOOT;
  portEXIT_CRITICAL(&gMux);
  return active;
}

void workerOtaServiceLoop() {
  if (gAdvertisingWindow && !gConnected &&
      static_cast<int32_t>(millis() - gWindowUntilMs) >= 0) {
    NimBLEDevice::getAdvertising()->stop(OTA_ADV_INSTANCE);
    NimBLEDevice::getAdvertising()->removeInstance(OTA_ADV_INSTANCE);
    gAdvertisingWindow = false;
    NimBLEDevice::getScan()->start(0);
    LOG("Worker OTA GATT window closed");
  }
  if (gRebootAtMs != 0 && static_cast<int32_t>(millis() - gRebootAtMs) >= 0) {
    ESP.restart();
  }
}
