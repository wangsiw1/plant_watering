# ESP32-C3 pot plan watering system — Design & Notes

This document collects requirements, design decisions, implementation notes of software for the main device of ESP32-C3 pot plant watering project.

Note: Hardware specifications other than ESP32 C3 is not in the scope.

## Overview

Goal: As a main device, it will have a non contact capacitive water level sensor and a water pump connnected. The device will collect soil sensor data from worker devices through Bluetooth broadcast, then based on the settings and sensor data, it will send commands to worker devices and start pump. This main device will also host an HTTP server that serves a web page for users to get info and config system. Detailed components are following:

- WiFi (WiFiProvisioner)
- mDNS
- OTA (ArduinoOTA, Async?)
- Wireless data reading and command sending (Bluetooth broadcast? No wifi channel issue, but range? packet loss?)
- HTTP server with a small web UI for worker device add/remove, settings, info, etc.
- Settings persistence across reboots
- Manual actions via the UI

## Functionalities

- Setup mDNS for easier web page access
- With WiFiProvisioner, let users connect to the on board WiFi first to select WiFi router to connect to. So that users can access web page directly through exsiting router without needing to change WiFi all the time.
- Bluetooth keeps listening the broadcast data and broadcasts commands when required. 
- ArduinoOTA for OTA. (Firmware image integrity check and rollback on fail need to be checked)
- Web page:
  - Show digital data from sensor
  - Last pump activation time
  - Set water pump activation time during the day
  - Cooldown after previous watering, no watering during this period, wait for water to go into soil
  - Manual pump activation
  - Enable/disable audo watering function based on schedule and soild moisture
  - Apply and save all inputs
  - Add worker node using Bluetooth MAC address
  - List of added worker nodes, each node has:
    - Coneection status, last watering time, and battery level
    - Soil moisture data and latest updated time
    - Moisture threshold of when should valve be activated to water the plant
    - Watering duration that how long a valve should be activated to water the palnt
    - Manual solenoid valve activation

### Details

- mDNS
  - Default settings
  - Domain name: plant-watering-\<WIFI_MAC_LAST_6_HEX\>.local

- WiFiProvisioner
  - Default settings

- ArduinoOTA
  - Default settings

- Web UI (Minimum style while easy to view)
  - Each input below has a tooltip explain the usage 
  - Title/header ("Plant Watering (\<name\>)", name is \<WIFI_MAC_LAST_6_HEX\> by default)
  - Name input (255 char max, empty by default. If non-empty, replace name in title)
  - Current time (hour:minute), and input to set manually
  - Last watering time(days/hours/minuts ago)
  - Water tank info from sensor: High/Low
  - Time range input of watering activation: 24-hour start and end
  - Interval of watering input (seconds with hardcoded min/max)
  - Interval of data sync input (seconds with hardcoded min/max)
  - Button to manually activate pump
  - Button to apply all changes
  - Major button to start auto watering
  - ID (12 hex digits of Bluetooth MAC address, upper or lower cases) input and add button to add worker node to list
  - Major button to apply changes to inputs
  - List of added worker nodes, each node has:
    - Status: Not connected/Connected, last valve activation time (days/hours/minuts ago), Battery: \<percentage\>
    - Soil moisture data reading (0-4095 analog), last update time (days/hours/minuts ago)
    - Plant name input (64 characters max)
    - Moisture threshold input (0-4095 analog range, with hardcoded min/max)
    - Watering duration input (seconds to water, with hardcoded min/max)
    - Button to manuall activate solenoid valve
    - Remove button to remove this node from list
  - Callapsed log list and system health info

- Hardware
  - ESP32 C3 Super Mini
  - Pump activation pin: GPIO 1
  - Water tank level sensor reading pin: GPIO 3

- Logics
  - Time
    - 24-Hour, minute level (HH:mm)
    - Input by user via web UI
    - Save user input, and save the epoch time of saving time, use them to calculate approximate current time(something like Current time = user_input_time + (current_ticks - tick_at_saving), be careful with 32-bit integer roll over)
    - Use this current time for watering time range when auto watering is enabled
  - Wifi
    - If WiFiProvisioner could not connect to any Wifi, setup on board Wifi, allow users connecting to ESP32 C3's AP and access HTTP server through default IP/mDNS
  - Watering
    - With major button switching between manual and auto, manual by default
    - Always follow water tank level sensor reading to force stop watering
    - Manual mode
      - Enable pump activation button to trigger pump
      - Enable valve activation button to send Bluetooth broadcast message to certain node to trigger valve
    - Auto mode
      - Disable pump activation button
      - Disable valve activation button
      - Only activate if water tank level is not low
      - Only activate during active time range
      - Only activate after watering interval
      - Only activate if one or more reported soil moisture below threshold
  - Watering interval
    - Cooldown timer that no watering triggers
    - Starts from end of last watering
    - Minimum: 60 seconds
    - Maximum: 2,419,200 seconds (28 days)
    - Default: 3600 seconds (1 hour)
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
    - Main node
      - Always on
      - Host HTTP server
      - Monitor water tank level sensor
      - Listen to any new worker broadcasting to join
      - Maintain time
    - Worker node
      - Always on by deafult, broadcasting data periodically with target node Bluetooth MAC FF:FF:FF:FF:FF:FF. 
      - After connecting to main node, receive sleep time and go to sleep, only wake up after and wait for main node probe to broadcast data
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

## Logging

With only very limited on-board memory and storage on ESP32 C3, long-time info level logging is challenging. For now only log warning level with date, time and minimum info:

- Sensor data history
  - Find water tank level is low
  - ...
- System health log
  - Find worker node disconnected
  - ...

## Security & privacy notes

The system is designed to run within a home LAN, not exposed to internet directly, and not using private information. The level of security requirement is considered low, but following points can be considered to be implemented:

- Web UI authentication (password, use "watering" as default for now, there is no physic button to reset in case password is changed and forgotten)
- Secured OTA update (ArduinoOTA supports username and password, use "admin" and "watering" for same reason)
- Bluetooth data encryption (unlikely unless easy to do)

## Edge cases & limitations

- There is no pump health info(not connected, broken, etc.)
- Very limited on-board memory and storage

## Next improvements (suggested tasks)

TBD

## Quick reference

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
- `0x12` TYPE_TARGET: 6 bytes — target MAC for commands
- `0x14` TYPE_PAYLOAD: n bytes — command-specific payload
- `0x20` TYPE_ACK  : 2 bytes — ACK TLV carrying a nonce

Command IDs (embedded in payload)
- `CMD_PROBE = 0x01` — ask worker to broadcast status immediately
- `CMD_SLEEP  = 0x02` — set next wake/sync time (payload encodes delay)
- `CMD_WATER = 0x03` — instruct worker to open valve for a duration

Example: Probe (main -> worker)
- CompanyID(2)
- TYPE_TARGET(1) LEN(1) MAC(6)
- TYPE_NONCE(1) LEN(1) NONCE(2)
- TYPE_PAYLOAD(1) LEN(1) [CMD_PROBE]

Example: Water 10 seconds (main -> worker)
- CompanyID(2)
- TYPE_TARGET LEN MAC
- TYPE_NONCE LEN NONCE
- TYPE_PAYLOAD LEN [CMD_WATER (1) | duration (2 bytes big-endian)]

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



