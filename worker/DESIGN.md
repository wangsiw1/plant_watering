# ESP32-C3 pot plan watering system — Design & Notes

This document collects requirements, design decisions, implementation notes of software for the worker device of ESP32-C3 pot plant watering project.

Note: Hardware specifications other than ESP32 C3 is not in the scope.

## Overview

Goal: As a worker device, it will have a capacitive soil moisture sensor and a solenoid valve connected. The device will read soil sensor data and send to main device via Bluetooth broadcasting, active or deactive pump, or go to sleep following the command from main devic. 

### Details

- Hardware
  - ESP32 C3 Super Mini
  - Valve activation pin: GPIO 1
  - Battery pin: GPIO 4
  - Soil moisture sensor reading pin: GPIO 3

- Logics
  - Data sync interval
    - Cooldown timer until next data change between main and worker nodes
    - Worker nodes go to deep sleep during this interval, only wake up after timer
    - Minimum: 60 seconds
    - Maximum: 2,419,200 seconds (28 days)
    - Default: 3600 seconds (1 hour)
  - Data sync
    - TLV format
    - Main to worker
      - Bluetooth MAC address of target worker
      - Duration until next data sync
    - Worker to main
      - Bluetooth MAC address of main node
      - Soil moisture
      - Battery level
  - Bluetooth
    - Broadcast with ACKs
    - 31 bytes max payload for classic advertising
  - Workflow
    - Worker node
      - Always on by deafult, broadcasting data periodically with target node Bluetooth MAC FF:FF:FF:FF:FF:FF. 
      - After syncing with main node, receive sleep time and go to sleep, only wake up after and wait for main node probe to broadcast data
      - Monitor soil moisture sensor
    - Worker node connection procedure
      - Not exactly connection, but as long as main node is able to communicate with worker, mark it as connected
      - Worker node starts broadcasting data periodically after power up
      - If Bluetooth MAC address of worker node is added to list in main node, main node should proceed to standard device timing coordination
    - Device timing coordination procedure
      - Main node calculates next data sync time
      - With calculated delay time for each worker node, all worker should wake up around similar time
      - After watering procedure done, main node should calculate new time and send to workers, then workers go to sleep
        1. Main node broadcasts sleep time to the worker one by one
        2. Worker broadcasts ACK
        3. Worker go to sleep
    - Data sync procedure
      - After data sync interval plus extra small delay, assuming all worker nodes wake up, for each worker in the worker list:
        1. Main node broadcasts probe to the worker
        2. Worker listens and broadcasts data
        3. Main node broadcasts ACK
    - Watering procedure
      - After data sync, if all conditions meet the watering requirement
        1. Main node activates water pump
        2. For each node that soil moisture goes below the threshold:
          2.1 Main node broadcasts watering duration to worker
          2.2 Worker broadcasts ACK and activates valve
          2.3 Worker stops valve after duration and broadcasts back same watering command
          2.4 Main node broadcasts ACK
        3. Main node stops pump and proceeds to device timing coordination procedure
      - If conditions not meet, skip to device timing coordination procedure
    - Worker node retry and disconnect procudure
      - Either main node or worker node is not responding to message after certain time lenght, retry 3 times
      - If worker node does not respond after retries, main node marks it as disconnected and continue
      - If main node does not respond to worker node, reset the worker node itself and start over from worker node connection procedure

## Bluetooth payload format (compact + AEAD)

The worker uses the compact packet layout described below for both status advertisements
and command handling. When extended advertising is available a full STATUS (multiple soils)
can be sent in one advert; otherwise the compact format minimizes the need for chunking.

Envelope
- 2 bytes: Company Identifier (little-endian). For local testing use `0xFFFF`.
- Followed by payload starting at TargetMAC.

Compact packet layout
- CompanyID (2 bytes)
- TargetMAC (6 bytes)
- Nonce (2 bytes, big-endian)
- MsgPayload: [MsgType(1), ...]

MsgType examples
- `0x40` TYPE_STATUS: [TYPE_STATUS][Battery(1)][PotCount(1)][Soil1(2), ...]
- `0x32` TYPE_CMD_WATER: [TYPE_CMD_WATER][PotMask(2)][DurationArray(2*N)]
  - PotMask: 16-bit bitmask where bit `i` corresponds to pot `i`.
  - DurationArray: N 16-bit big-endian seconds, where N is the count of set bits in PotMask.
    Durations are listed in increasing pot-index order for the set bits.
    Example: if PotMask has bits for pots 0, 2, 3, then DurationArray = [dur_0, dur_2, dur_3].
- `0x30` TYPE_CMD_PROBE: probe request (no payload)
- `0x20` TYPE_ACK: acknowledgement

Encryption (AEAD)
- If `USE_BT_CRYPTO` is enabled the worker encrypts the MsgPayload using AES-GCM
  and appends the 16-byte tag. The receiver (main) will attempt to decrypt; on failure
  the receiver falls back to raw payload parsing to preserve interoperability.
- IV derivation: IV = first 12 bytes of HMAC-SHA256(network_key, target_mac || nonce || "btiv").

Extended advertising
- When `USE_EXT_ADV` is active and supported, use extended adverts to send larger STATUS
  payloads. Otherwise the worker uses legacy adverts with the compact header.

Legacy TLV compatibility
- TLV helpers are still available but the worker now prefers the compact format.
  The sender-side conversion from TLV to compact is only performed when the queued
  buffer clearly matches TLV semantics (length byte exactly matches remaining bytes).

Implementation notes
- `btWorkerAdvertiseStatus()` constructs the compact STATUS payload and queues it via
  `btCommonQueueCommand()` (which builds the CompanyID+header). `parse_compact_packet_header()`
  is used to parse and decrypt received compact packets.

Hardware build variants
-----------------------
This worker firmware supports multiple hardware variants selected at compile time via a
single `HW_TARGET_*` build flag. The flag determines pin mappings and the number of pots.

Examples:
- `HW_V1_1POT_REV_A` : original single-pot hardware (SOIL_PIN=3, VALVE_PIN=1)
- `HW_V2_8POT_REV_A` : 8-pot with multiplexer (3 select pins) + one shift-register chip
- `HW_V2_8POT_REV_B` : 8-pot variant with alternate pin assignments

Set the build flag in `platformio.ini` or your build command (see platformio envs in repository).
For multi-pot variants, `USE_EXT_ADV` should be enabled to avoid BLE legacy advertisement
truncation (the build configuration provides example envs).



