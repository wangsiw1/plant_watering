#include "WorkerOtaManager.h"

#include "BluetoothMain.h"
#include "FirmwarePackage.h"
#include "Utility.h"
#include "config.h"

#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <NimBLEDevice.h>
#include <cstring>
#include <mbedtls/sha256.h>

namespace {

constexpr char OTA_SERVICE_UUID[] = "f3641400-00b0-4240-ba50-05ca45bf8abc";
constexpr char OTA_INFO_UUID[] = "f3641401-00b0-4240-ba50-05ca45bf8abc";
constexpr char OTA_CONTROL_UUID[] = "f3641402-00b0-4240-ba50-05ca45bf8abc";
constexpr char OTA_DATA_UUID[] = "f3641403-00b0-4240-ba50-05ca45bf8abc";
constexpr char WORKER_FW_PATH[] = "/worker_fw.ota";
constexpr char WORKER_FW_TMP_PATH[] = "/worker_fw.tmp";
constexpr uint32_t WORKER_OTA_SLOT_SIZE = 0x140000;
constexpr size_t TRANSFER_CHUNK_SIZE = 160;
constexpr uint32_t CONNECT_WINDOW_MS = 120000;

enum class RemoteWorkerState : uint8_t {
  IDLE = 0,
  PREPARED = 1,
  WRITING = 2,
  VERIFYING = 3,
  READY_TO_REBOOT = 4,
  FAILED = 5
};

struct RemoteWorkerInfo {
  uint8_t role;
  RemoteWorkerState state;
  uint8_t error;
  uint16_t hardwareTarget;
  uint32_t currentVersion;
  uint32_t otaSlotSize;
  uint32_t receivedBytes;
};

portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
WorkerOtaStatusSnapshot gStatus{};
File gUploadFile;
uint8_t gHeader[FirmwarePackage::HEADER_SIZE] = {};
size_t gHeaderReceived = 0;
FirmwarePackage::Header gParsedHeader{};
mbedtls_sha256_context gSha256;
bool gSha256Active = false;
bool gStorageAvailable = false;
size_t gUploadCapacityBytes = 0;
TaskHandle_t gWorkerOtaTask = nullptr;

void copyMessage(char out[80], const char* message) {
  if (!message) message = "";
  strncpy(out, message, 79);
  out[79] = '\0';
}

void setStatus(WorkerOtaJobState state, const char* message = nullptr) {
  portENTER_CRITICAL(&gMux);
  gStatus.state = state;
  if (message) copyMessage(gStatus.message, message);
  portEXIT_CRITICAL(&gMux);
}

void setProgress(uint32_t imageBytesSent) {
  portENTER_CRITICAL(&gMux);
  gStatus.imageBytesSent = imageBytesSent;
  portEXIT_CRITICAL(&gMux);
}

void setFailure(const char* message) {
  portENTER_CRITICAL(&gMux);
  gStatus.state = WorkerOtaJobState::FAILED;
  gStatus.active = false;
  copyMessage(gStatus.message, message);
  portEXIT_CRITICAL(&gMux);
  LOG("Worker OTA failed: %s", message ? message : "unknown");
}

void finishActiveJob(WorkerOtaJobState state, const char* message) {
  portENTER_CRITICAL(&gMux);
  gStatus.state = state;
  gStatus.active = false;
  if (message) copyMessage(gStatus.message, message);
  portEXIT_CRITICAL(&gMux);
}

bool isActiveLocked() {
  return gStatus.active;
}

void freeSha256() {
  if (!gSha256Active) return;
  mbedtls_sha256_free(&gSha256);
  gSha256Active = false;
}

void cleanupUpload(bool removeTemp) {
  if (gUploadFile) gUploadFile.close();
  freeSha256();
  memset(gHeader, 0, sizeof(gHeader));
  memset(&gParsedHeader, 0, sizeof(gParsedHeader));
  gHeaderReceived = 0;
  gUploadCapacityBytes = 0;
  if (removeTemp && gStorageAvailable) LittleFS.remove(WORKER_FW_TMP_PATH);
}

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

void writeU32(uint8_t* data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8);
  data[2] = static_cast<uint8_t>(value >> 16);
  data[3] = static_cast<uint8_t>(value >> 24);
}

const char* remoteErrorName(uint8_t error) {
  switch (error) {
    case 0: return "";
    case 1: return "Worker rejected command";
    case 2: return "Worker rejected firmware header";
    case 3: return "Wrong hardware target";
    case 4: return "Firmware is already up to date";
    case 5: return "Worker OTA partition unavailable";
    case 6: return "Firmware too large";
    case 7: return "Worker rejected ESP image";
    case 8: return "Transfer offset mismatch";
    case 9: return "Worker flash write failed";
    case 10: return "Worker SHA-256 mismatch";
    case 11: return "Worker image validation failed";
    case 12: return "Worker boot partition select failed";
    case 13: return "Worker OTA aborted";
    case 14: return "version_below_minimum";
    default: return "Worker OTA failed";
  }
}

bool parseWorkerInfo(const std::string& value, RemoteWorkerInfo& out) {
  if (value.size() != 28 || memcmp(value.data(), "PWOTAI01", 8) != 0) {
    return false;
  }
  const uint8_t* data = reinterpret_cast<const uint8_t*>(value.data());
  out.role = data[8];
  out.state = static_cast<RemoteWorkerState>(data[9]);
  out.error = data[10];
  out.hardwareTarget = readU16(data + 12);
  out.currentVersion = readU32(data + 16);
  out.otaSlotSize = readU32(data + 20);
  out.receivedBytes = readU32(data + 24);
  return true;
}

bool loadStoredHeader(FirmwarePackage::Header& out) {
  if (!gStorageAvailable) return false;
  File file = LittleFS.open(WORKER_FW_PATH, FILE_READ);
  if (!file) return false;
  uint8_t header[FirmwarePackage::HEADER_SIZE];
  size_t read = file.read(header, sizeof(header));
  file.close();
  if (read != sizeof(header)) return false;
  return FirmwarePackage::parseWorkerHeader(
             header, sizeof(header), WORKER_OTA_SLOT_SIZE, out) ==
         FirmwarePackage::ParseError::NONE;
}

bool sendOtaPrepareCommand(const uint8_t mac[6]) {
  BT_TLV::BtBodyBuilder body;
  BT_TLV::btBodyBegin(body, BT_TLV::TYPE_CMD_OTA_PREPARE);
  BtSendResult result = btMainSendCommand(mac, body.data, body.len, 3, 1200);
  return result.status == BtSendStatus::ACKED;
}

NimBLEClient* connectWorker(const uint8_t mac[6]) {
  NimBLEDevice::getScan()->stop();
  NimBLEDevice::setMTU(247);
  NimBLEClient* client = NimBLEDevice::createClient();
  client->setConnectionParams(12, 24, 0, 180);
  client->setConnectTimeout(8000);
  NimBLEAddress address(mac, BLE_ADDR_PUBLIC);
  if (!client->connect(address)) {
    NimBLEDevice::deleteClient(client);
    return nullptr;
  }
  client->exchangeMTU();
  return client;
}

void restartMainScan() {
  NimBLEScan* scan = NimBLEDevice::getScan();
  if (!scan->isScanning()) scan->start(0);
}

bool readRemoteInfo(NimBLERemoteCharacteristic* infoChar,
                    RemoteWorkerInfo& info) {
  if (!infoChar) return false;
  std::string value = infoChar->readValue();
  return parseWorkerInfo(value, info);
}

bool deployToWorker(const uint8_t mac[6], const FirmwarePackage::Header& header) {
  char macText[13];
  macToHexLower(mac, macText);
  setStatus(WorkerOtaJobState::CONNECTING, "Opening worker OTA window");
  if (!sendOtaPrepareCommand(mac)) {
    setFailure("Worker not reachable");
    return false;
  }

  setStatus(WorkerOtaJobState::CONNECTING, "Connecting over BLE");
  NimBLEClient* client = connectWorker(mac);
  if (!client) {
    setFailure("BLE connection failed");
    restartMainScan();
    return false;
  }

  bool ok = false;
  NimBLERemoteService* service = client->getService(OTA_SERVICE_UUID);
  NimBLERemoteCharacteristic* infoChar =
      service ? service->getCharacteristic(OTA_INFO_UUID) : nullptr;
  NimBLERemoteCharacteristic* controlChar =
      service ? service->getCharacteristic(OTA_CONTROL_UUID) : nullptr;
  NimBLERemoteCharacteristic* dataChar =
      service ? service->getCharacteristic(OTA_DATA_UUID) : nullptr;
  RemoteWorkerInfo info{};
  if (!service || !infoChar || !controlChar || !dataChar) {
    setFailure("Worker OTA service unavailable");
    goto done;
  }

  setStatus(WorkerOtaJobState::CHECKING, "Checking worker");
  if (!readRemoteInfo(infoChar, info)) {
    setFailure("Worker info read failed");
    goto done;
  }
  if (info.role != FirmwarePackage::ROLE_WORKER) {
    setFailure("Worker role mismatch");
    goto done;
  }
  if (info.hardwareTarget != header.hardwareTarget) {
    setFailure("Wrong hardware target");
    goto done;
  }
  if (header.firmwareVersion == info.currentVersion) {
    setFailure("already_up_to_date");
    goto done;
  }
  if (info.otaSlotSize == 0) {
    setFailure("Worker OTA partition unavailable");
    goto done;
  }
  if (header.imageSize > info.otaSlotSize) {
    setFailure("Firmware too large");
    goto done;
  }

  {
    uint8_t command[1 + FirmwarePackage::HEADER_SIZE];
    command[0] = 1;
    File file = LittleFS.open(WORKER_FW_PATH, FILE_READ);
    if (!file) {
      setFailure("Stored worker firmware missing");
      goto done;
    }
    if (file.read(command + 1, FirmwarePackage::HEADER_SIZE) !=
        FirmwarePackage::HEADER_SIZE) {
      file.close();
      setFailure("Stored worker firmware header unreadable");
      goto done;
    }
    file.close();
    if (!controlChar->writeValue(command, sizeof(command), true)) {
      setFailure("Worker prepare failed");
      goto done;
    }
  }

  if (!readRemoteInfo(infoChar, info)) {
    setFailure("Worker prepare status failed");
    goto done;
  }
  if (info.state == RemoteWorkerState::FAILED) {
    setFailure(remoteErrorName(info.error));
    goto done;
  }

  setStatus(WorkerOtaJobState::TRANSFERRING, "Updating");
  {
    File file = LittleFS.open(WORKER_FW_PATH, FILE_READ);
    if (!file) {
      setFailure("Stored worker firmware missing");
      goto done;
    }
    if (!file.seek(FirmwarePackage::HEADER_SIZE, SeekSet)) {
      file.close();
      setFailure("Stored worker firmware seek failed");
      goto done;
    }
    uint8_t chunk[4 + TRANSFER_CHUNK_SIZE];
    uint32_t offset = 0;
    while (offset < header.imageSize) {
      size_t want = min(static_cast<size_t>(TRANSFER_CHUNK_SIZE),
                        static_cast<size_t>(header.imageSize - offset));
      size_t got = file.read(chunk + 4, want);
      if (got != want) {
        file.close();
        setFailure("Stored worker firmware read failed");
        goto done;
      }
      writeU32(chunk, offset);
      if (!dataChar->writeValue(chunk, got + 4, true)) {
        file.close();
        setFailure("BLE transfer failed");
        goto done;
      }
      offset += got;
      setProgress(offset);
      if ((offset % (TRANSFER_CHUNK_SIZE * 8)) == 0) vTaskDelay(pdMS_TO_TICKS(1));
    }
    file.close();
  }

  setStatus(WorkerOtaJobState::VERIFYING, "Worker verifying");
  {
    uint8_t command = 2;
    if (!controlChar->writeValue(&command, 1, true)) {
      setFailure("Worker commit failed");
      goto done;
    }
  }
  vTaskDelay(pdMS_TO_TICKS(300));
  if (!readRemoteInfo(infoChar, info)) {
    setFailure("Worker verify status failed");
    goto done;
  }
  if (info.state == RemoteWorkerState::FAILED) {
    setFailure(remoteErrorName(info.error));
    goto done;
  }
  if (info.state != RemoteWorkerState::READY_TO_REBOOT) {
    setFailure("Worker did not accept firmware");
    goto done;
  }

  finishActiveJob(WorkerOtaJobState::DONE, "Updated, waiting for reconnect");
  ok = true;

done:
  if (client->isConnected()) client->disconnect();
  NimBLEDevice::deleteClient(client);
  restartMainScan();
  LOG("Worker OTA deploy result worker=%s ok=%u", macText, ok ? 1u : 0u);
  return ok;
}

void workerOtaTask(void* context) {
  uint8_t mac[6];
  memcpy(mac, context, 6);
  delete[] reinterpret_cast<uint8_t*>(context);

  FirmwarePackage::Header header{};
  if (!loadStoredHeader(header)) {
    setFailure("No worker firmware uploaded");
    gWorkerOtaTask = nullptr;
    vTaskDelete(nullptr);
    return;
  }
  portENTER_CRITICAL(&gMux);
  gStatus.expectedImageBytes = header.imageSize;
  gStatus.packageVersion = header.firmwareVersion;
  gStatus.packageHardware = header.hardwareTarget;
  portEXIT_CRITICAL(&gMux);

  deployToWorker(mac, header);
  gWorkerOtaTask = nullptr;
  vTaskDelete(nullptr);
}

}  // namespace

void workerOtaManagerInit() {
  WorkerOtaStatusSnapshot status{};
  gStorageAvailable = LittleFS.begin(true);
  if (!gStorageAvailable) {
    status.state = WorkerOtaJobState::FAILED;
    copyMessage(status.message, "Worker firmware storage unavailable");
    portENTER_CRITICAL(&gMux);
    gStatus = status;
    portEXIT_CRITICAL(&gMux);
    LOG("LittleFS mount failed; worker firmware storage unavailable");
    return;
  }

  status.state = LittleFS.exists(WORKER_FW_PATH) ? WorkerOtaJobState::STORED
                                                 : WorkerOtaJobState::IDLE;
  if (status.state == WorkerOtaJobState::STORED) {
    FirmwarePackage::Header header{};
    if (loadStoredHeader(header)) {
      status.expectedImageBytes = header.imageSize;
      status.packageVersion = header.firmwareVersion;
      status.packageHardware = header.hardwareTarget;
      copyMessage(status.message, "Worker firmware stored");
    } else {
      LittleFS.remove(WORKER_FW_PATH);
      status.state = WorkerOtaJobState::IDLE;
      copyMessage(status.message, "Stored worker firmware was invalid");
    }
  }
  portENTER_CRITICAL(&gMux);
  gStatus = status;
  portEXIT_CRITICAL(&gMux);
}

bool workerOtaPackageUploadStart() {
  if (!gStorageAvailable) {
    setFailure("Worker firmware storage unavailable");
    return false;
  }
  portENTER_CRITICAL(&gMux);
  bool active = isActiveLocked();
  portEXIT_CRITICAL(&gMux);
  if (active) return false;
  cleanupUpload(true);
  LittleFS.remove(WORKER_FW_TMP_PATH);
  LittleFS.remove(WORKER_FW_PATH);
  if (LittleFS.exists(WORKER_FW_TMP_PATH) ||
      LittleFS.exists(WORKER_FW_PATH)) {
    setFailure("Worker firmware storage cleanup failed");
    return false;
  }
  const size_t totalBytes = LittleFS.totalBytes();
  const size_t usedBytes = LittleFS.usedBytes();
  gUploadCapacityBytes = totalBytes >= usedBytes ? totalBytes - usedBytes : 0;
  gUploadFile = LittleFS.open(WORKER_FW_TMP_PATH, FILE_WRITE);
  if (!gUploadFile) {
    setFailure("Worker firmware storage open failed");
    return false;
  }
  mbedtls_sha256_init(&gSha256);
  gSha256Active = true;
  if (mbedtls_sha256_starts_ret(&gSha256, 0) != 0) {
    cleanupUpload(true);
    setFailure("worker_package_hash_start_failed");
    return false;
  }
  WorkerOtaStatusSnapshot status{};
  status.state = WorkerOtaJobState::RECEIVING_PACKAGE;
  status.active = true;
  copyMessage(status.message, "Receiving worker firmware");
  portENTER_CRITICAL(&gMux);
  gStatus = status;
  portEXIT_CRITICAL(&gMux);
  return true;
}

bool workerOtaPackageUploadConsume(const uint8_t* data, size_t len) {
  if (!data || len == 0 || !gUploadFile) return false;
  if (gUploadFile.write(data, len) != len) {
    copyMessage(gStatus.message, "worker_package_write_failed");
    return false;
  }

  portENTER_CRITICAL(&gMux);
  gStatus.packageBytesReceived += len;
  uint32_t packageBytes = gStatus.packageBytesReceived;
  portEXIT_CRITICAL(&gMux);

  while (len > 0) {
    if (gHeaderReceived < sizeof(gHeader)) {
      size_t take = min(len, sizeof(gHeader) - gHeaderReceived);
      memcpy(gHeader + gHeaderReceived, data, take);
      gHeaderReceived += take;
      data += take;
      len -= take;
      if (gHeaderReceived == sizeof(gHeader)) {
        FirmwarePackage::ParseError err = FirmwarePackage::parseWorkerHeader(
            gHeader, sizeof(gHeader), WORKER_OTA_SLOT_SIZE, gParsedHeader);
        if (err != FirmwarePackage::ParseError::NONE) {
          copyMessage(gStatus.message, FirmwarePackage::parseErrorName(err));
          return false;
        }
        const size_t packageSize =
            FirmwarePackage::HEADER_SIZE +
            static_cast<size_t>(gParsedHeader.imageSize);
        if (packageSize > gUploadCapacityBytes) {
          copyMessage(gStatus.message, "worker_package_too_large_for_storage");
          return false;
        }
        portENTER_CRITICAL(&gMux);
        gStatus.expectedImageBytes = gParsedHeader.imageSize;
        gStatus.packageVersion = gParsedHeader.firmwareVersion;
        gStatus.packageHardware = gParsedHeader.hardwareTarget;
        gStatus.packageBytesReceived = packageBytes;
        portEXIT_CRITICAL(&gMux);
      }
      continue;
    }
    if (mbedtls_sha256_update_ret(&gSha256, data, len) != 0) return false;
    data += len;
    len = 0;
  }
  return true;
}

bool workerOtaPackageUploadFinish() {
  if (!gUploadFile) return false;
  gUploadFile.close();
  if (gHeaderReceived != FirmwarePackage::HEADER_SIZE ||
      gStatus.packageBytesReceived !=
          FirmwarePackage::HEADER_SIZE + gParsedHeader.imageSize) {
    cleanupUpload(true);
    setFailure("worker_package_size_mismatch");
    return false;
  }
  uint8_t calculated[32];
  if (mbedtls_sha256_finish_ret(&gSha256, calculated) != 0) {
    cleanupUpload(true);
    setFailure("worker_package_hash_failed");
    return false;
  }
  freeSha256();
  if (memcmp(calculated, gParsedHeader.imageSha256, sizeof(calculated)) != 0) {
    cleanupUpload(true);
    setFailure("worker_package_sha256_mismatch");
    return false;
  }
  if (!LittleFS.rename(WORKER_FW_TMP_PATH, WORKER_FW_PATH)) {
    cleanupUpload(true);
    setFailure("worker_package_store_failed");
    return false;
  }

  portENTER_CRITICAL(&gMux);
  gStatus.state = WorkerOtaJobState::STORED;
  gStatus.active = false;
  gStatus.expectedImageBytes = gParsedHeader.imageSize;
  gStatus.packageVersion = gParsedHeader.firmwareVersion;
  gStatus.packageHardware = gParsedHeader.hardwareTarget;
  copyMessage(gStatus.message, "Worker firmware stored");
  portEXIT_CRITICAL(&gMux);
  cleanupUpload(false);
  return true;
}

void workerOtaPackageUploadAbort(const char* reason) {
  cleanupUpload(true);
  setFailure(reason ? reason : "worker_upload_aborted");
}

bool workerOtaStartForWorker(const uint8_t mac[6]) {
  if (!mac) return false;
  if (!gStorageAvailable) {
    setFailure("Worker firmware storage unavailable");
    return false;
  }
  if (!LittleFS.exists(WORKER_FW_PATH)) {
    setFailure("No worker firmware uploaded");
    return false;
  }
  WorkerConfig worker{};
  if (!findWorkerConfigByMac(mac, worker)) {
    setFailure("Worker not configured");
    return false;
  }
  portENTER_CRITICAL(&gMux);
  bool active = gStatus.active;
  if (!active) {
    memset(&gStatus, 0, sizeof(gStatus));
    gStatus.state = WorkerOtaJobState::CONNECTING;
    gStatus.active = true;
    memcpy(gStatus.targetMac, mac, 6);
    copyMessage(gStatus.message, "Queued");
  }
  portEXIT_CRITICAL(&gMux);
  if (active) return false;

  uint8_t* context = new uint8_t[6];
  memcpy(context, mac, 6);
  if (xTaskCreate(workerOtaTask, "workerOta", 6144, context, 2,
                  &gWorkerOtaTask) != pdPASS) {
    delete[] context;
    setFailure("Worker OTA task create failed");
    return false;
  }
  return true;
}

bool workerOtaIsActive() {
  portENTER_CRITICAL(&gMux);
  bool active = gStatus.active;
  portEXIT_CRITICAL(&gMux);
  return active;
}

void workerOtaGetStatus(WorkerOtaStatusSnapshot& out) {
  portENTER_CRITICAL(&gMux);
  out = gStatus;
  portEXIT_CRITICAL(&gMux);
}

const char* workerOtaStateName(WorkerOtaJobState state) {
  switch (state) {
    case WorkerOtaJobState::IDLE: return "idle";
    case WorkerOtaJobState::RECEIVING_PACKAGE: return "receiving_package";
    case WorkerOtaJobState::STORED: return "stored";
    case WorkerOtaJobState::CONNECTING: return "connecting";
    case WorkerOtaJobState::CHECKING: return "checking";
    case WorkerOtaJobState::TRANSFERRING: return "transferring";
    case WorkerOtaJobState::VERIFYING: return "verifying";
    case WorkerOtaJobState::REBOOTING: return "rebooting";
    case WorkerOtaJobState::DONE: return "done";
    case WorkerOtaJobState::FAILED: return "failed";
    default: return "unknown";
  }
}

void workerOtaManagerLoop() {
}
