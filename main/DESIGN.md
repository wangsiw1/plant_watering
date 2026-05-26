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

## Bluetooth payload format (compact + AEAD)

This project moved from a TLV-first encoding to a compact, fixed-position header to
reduce per-field overhead and allow multiple soil readings inside a single advertisement.
When extended advertising is available the full STATUS payload can be sent in one advert;
otherwise the compact layout minimizes fragmentation over legacy 31-byte adverts.

Envelope
- 2 bytes: Company Identifier (little-endian). For local testing use `0xFFFF`.
- Followed by payload beginning at TargetMAC (see compact layout below).

Compact packet layout (preferred)
- CompanyID (2 bytes, little-endian)
- TargetMAC (6 bytes) — MAC of the recipient, or `FF:FF:FF:FF:FF:FF` for broadcast
- Nonce (2 bytes, big-endian) — per-packet nonce for correlation
- MsgPayload (remaining bytes) — begins with MsgType (1 byte) followed by type-specific fields

MsgType values
- `0x30` TYPE_CMD_PROBE  — probe (no payload)
- `0x31` TYPE_CMD_SLEEP  — [TYPE_CMD_SLEEP][4-byte seconds BE]
- `0x32` TYPE_CMD_WATER  — compact: [TYPE_CMD_WATER][PotMask(2)][DurationArray(2*N)]
  - PotMask: 16-bit bitmask where bit `i` corresponds to pot `i`.
  - DurationArray: N 16-bit big-endian seconds, where N is the count of set bits in PotMask.
    Durations are listed in increasing pot-index order for the set bits.
    Example: if PotMask has bits for pots 0, 2, 3, then DurationArray = [dur_0, dur_2, dur_3].
- `0x40` TYPE_STATUS    — [TYPE_STATUS][Battery(1)][PotCount(1)][Soil1(2), Soil2(2), ...]
- `0x41` TYPE_CONFIG    — announce potCount when unpaired
- `0x20` TYPE_ACK       — acknowledge (main/worker will mark pending nonce)

Encryption (AEAD)
- When `USE_BT_CRYPTO` is enabled the MsgPayload (bytes after TargetMAC+Nonce)
  is encrypted with AES-GCM and the 16-byte authentication tag is appended.
- IV derivation: IV = first 12 bytes of HMAC-SHA256(network_key, target_mac || nonce || "btiv").
- Implemented in `btEncryptPayload`/`btDecryptPayload` (mbedTLS GCM). The network key
  in code is a placeholder; provision keys securely in production (NVS/provisioning).
- `parse_compact_packet_header()` attempts decryption and on failure returns the raw
  payload. A reversed-MAC fallback is attempted to work around byte-order mismatches.

Extended Advertising
- When `USE_EXT_ADV` is enabled and stack support is present the system will use
  extended advertising (`NimBLEExtAdvertising`) to avoid 31-byte limits. Otherwise
  it falls back to legacy advertising.

Legacy TLV compatibility
- The project retains TLV parsing helpers for interoperability. When a command is
  queued via `btCommonQueueCommand()` the sender will convert TLV -> compact only
  when the queued buffer looks exactly like a TLV payload (the length byte must
  exactly match the remaining bytes). This prevents mis-detection where a compact
  payload's data byte (e.g., battery) could be mistaken for a TLV length.

Implementation notes
- `parse_compact_packet_header()` (in `src/bluetooth_common.cpp`) takes data starting
  after the CompanyID and returns the target MAC, nonce and a pointer/length to the
  (possibly decrypted) MsgPayload.
- AES-GCM code and IV derivation live in `src/bluetooth_crypto.cpp`.
- The web UI and `WorkerNode` structures were updated to store per-device arrays
  of soils and `potCount` so the compact STATUS payload maps directly to the UI.



