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

## Bluetooth TLV Payload

This project uses a compact TLV (Type-Length-Value) format carried inside the BLE
Advertisement `Manufacturer Specific Data` field (AD type 0xFF). Keep advertisement
packets small — the AD payload (Company ID + TLV bytes) must fit within 31 bytes.

Envelope
- 2 bytes: Company Identifier (little-endian). For local testing use `0xFFFF`.
- Followed by TLV bytes.

TLV field format
- 1 byte: Type
- 1 byte: Length (N)
- N bytes: Value

Byte order and limits
- Multi-byte integers: big-endian in TLV values (network order).
- Max adv payload: 31 bytes (CompanyID + TLV). Use scan-response or split messages only if needed.

Defined types (used in code)
- `0x01` TYPE_MAC  : 6 bytes — device MAC (worker -> main status)
- `0x02` TYPE_SOIL : 2 bytes — soil ADC (uint16)
- `0x03` TYPE_BATT : 1 byte  — battery percent
- `0x11` TYPE_NONCE: 2 bytes — nonce for request/ACK correlation (uint16)
 - `0x12` TYPE_TARGET: REMOVED; target MAC is carried using `0x01` TYPE_MAC
 - Command TLV types (embedded in the command body):
   - `0x30` TYPE_CMD_PROBE: probe request (len=0)
   - `0x31` TYPE_CMD_SYNC: sleep/sync request (len=4 — seconds)
   - `0x32` TYPE_CMD_WATER: water request (len=2 — duration seconds)
 - `0x20` TYPE_ACK  : 2 bytes — ACK TLV carrying a nonce

Command IDs (embedded in payload)
- `CMD_PROBE = 0x01` — ask worker to broadcast status immediately
- `CMD_SYNC  = 0x02` — set next wake/sync time (payload encodes delay)
- `CMD_WATER = 0x03` — instruct worker to open valve for a duration

Example: Probe (main -> worker)
- CompanyID(2)
- TYPE_MAC(1) LEN(1) MAC(6)  # target MAC carried using TYPE_MAC
- TYPE_NONCE(1) LEN(1) NONCE(2)
 - TYPE_CMD_PROBE(1) LEN(1=0)  # TLV field with type 0x30 and length 0

Example: Water 10 seconds (main -> worker)
- CompanyID(2)
- TYPE_MAC LEN MAC  # target MAC carried using TYPE_MAC
- TYPE_NONCE LEN NONCE
 - TYPE_CMD_WATER LEN [duration (2 bytes big-endian)]

Worker responses
- On command targeted to the worker, it should immediately advertise an ACK TLV:
  TYPE_ACK (2 bytes: nonce) — this lets the main mark the command acknowledged.
- The worker then performs the action (e.g., open valve) and may follow with a STATUS
  advertisement containing TYPE_MAC, TYPE_SOIL and TYPE_BATT.

ACK semantics and retries
- Main generates a monotonic uint16 nonce (non-zero) per command and retransmits the
  same TLV (same nonce) on timeout.
- Worker must tolerate duplicate commands and ignore repeated commands already acknowledged/executed.

Size guidance
- Keep TLV compact and avoid sending large blobs — a typical command fits well within 31 bytes.
- If more data is needed, use the scan response to include additional fields or design
  a small multi-step exchange.

Implementation notes
- Code helpers to parse and construct these TLVs are in `include/BluetoothCommon.h` and
  `src/bluetooth_common.cpp` so they can be reused by both main and worker firmware.



