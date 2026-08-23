Import("env")

import hashlib
import struct
from pathlib import Path


# This script is a PlatformIO post-build step. It does not replace the normal
# firmware.bin; it creates a separate .ota file that the web uploader accepts.
#
# The .ota file is:
#
#   64-byte project header + raw ESP application image
#
# The header lets the running firmware reject obvious wrong uploads before it
# erases or writes the inactive OTA partition. It is a correctness wrapper, not
# a security signature.

# File identifier for this project's OTA package format. The running firmware
# checks these exact 8 bytes first, so a raw firmware.bin or unrelated file is
# rejected as "bad_magic" instead of being treated as an OTA package.
OTA_MAGIC = b"PWOTA001"

# Increment this only if the 64-byte package header layout or semantics change.
# It must match FirmwarePackage::FORMAT_VERSION in main/include/FirmwarePackage.h.
OTA_FORMAT_VERSION = 1

# Role and chip IDs are project-local enum values. They must match
# main/include/FirmwarePackage.h. Worker OTA can use another role value later.
OTA_ROLE_MAIN = 1
OTA_ROLE_WORKER = 2
OTA_CHIP_ESP32_C3 = 1

# Little-endian header layout. Keep this in sync with the OTA Package Format
# section in tmp/OTA_IMPLEMENTATION_PLAN.md and with firmware_package.cpp.
OTA_HEADER = struct.Struct("<8sHHBBHII32s8s")


def project_int_option(name):
    # PlatformIO custom_* options are strings. int(..., 0) accepts decimal and
    # hex values, so custom_ota_slot_size = 0x170000 works.
    value = env.GetProjectOption(name)
    return int(value, 0)


# Release knobs come from main/platformio.ini:
#
#   custom_fw_version    Package firmware version written into the .ota header.
#                        Keep it equal to -D FW_VERSION for the same build.
#   custom_fw_role       "main" or "worker"; written as the package role.
#   custom_fw_hardware   Hardware ID for the selected environment. Keep it equal
#                        to -D FW_HARDWARE_TARGET for the same environment.
#   custom_ota_slot_size Maximum raw firmware.bin size allowed for the current
#                        project's partition table. Main uses 0x170000-byte
#                        slots; worker projects keep their own configured limit.
#
# The package format rejects version 0. Each running firmware role enforces its
# own FW_MIN_ALLOWED_VERSION and rejects reinstalling its current version.
firmware_version = project_int_option("custom_fw_version")
firmware_role_name = env.GetProjectOption("custom_fw_role", "main").strip().lower()
hardware_target = project_int_option("custom_fw_hardware")
ota_slot_size = project_int_option("custom_ota_slot_size")
if firmware_role_name == "main":
    firmware_role = OTA_ROLE_MAIN
elif firmware_role_name == "worker":
    firmware_role = OTA_ROLE_WORKER
else:
    raise ValueError("custom_fw_role must be 'main' or 'worker'")

if firmware_version <= 0:
    raise ValueError("custom_fw_version must be greater than zero")
if hardware_target <= 0 or hardware_target > 0xFFFF:
    raise ValueError("custom_fw_hardware must fit in a nonzero uint16")

def package_ota(target, source, env):
    image_path = Path(target[0].get_abspath())
    image = image_path.read_bytes()

    # ESP application images start with magic byte 0xE9. This catches accidental
    # packaging of an ELF, text file, partition table, or already-wrapped .ota.
    if not image or image[0] != 0xE9:
        raise ValueError(f"{image_path} is not an ESP application image")

    # Fail the build if the app cannot fit in the inactive OTA app slot. This is
    # cheaper and clearer than discovering the problem during a web upload.
    if len(image) > ota_slot_size:
        raise ValueError(
            f"firmware is {len(image)} bytes, exceeding OTA slot size "
            f"{ota_slot_size} bytes"
        )

    # Hash only the raw image bytes, not the project header. The device writes
    # only these raw image bytes to flash and compares the same SHA-256 at upload
    # finish before selecting the new boot partition.
    digest = hashlib.sha256(image).digest()
    header = OTA_HEADER.pack(
        OTA_MAGIC,
        OTA_FORMAT_VERSION,
        OTA_HEADER.size,
        firmware_role,
        OTA_CHIP_ESP32_C3,
        hardware_target,
        firmware_version,
        len(image),
        digest,
        bytes(8),
    )

    environment_name = env.subst("$PIOENV").replace("_", "-")
    # Keep firmware.bin untouched for serial flashing. The generated .ota file is
    # the one to upload through the web UI.
    output_path = image_path.with_name(
        f"firmware-{firmware_role_name}-{environment_name}-v{firmware_version}.ota"
    )
    output_path.write_bytes(header + image)
    print(
        "OTA package: "
        f"{output_path} image={len(image)} bytes "
        f"role={firmware_role_name} hardware={hardware_target} "
        f"version={firmware_version} "
        f"sha256={digest.hex()}"
    )


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", package_ota)
