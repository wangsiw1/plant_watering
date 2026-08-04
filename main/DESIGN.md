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
  - List of added worker nodes grouped by worker device, each worker shows:
    - Battery level, last sync time, and RSSI
    - Worker name input
    - For each pot on that worker: soil moisture, last watering time, threshold, watering duration, pot name, and manual watering button

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
  - List of added worker nodes grouped by worker device:
    - Worker header shows MAC address, battery percentage, last sync time, and RSSI
    - Worker name input
    - Remove button removes the whole worker device from the list
    - Each pot row shows soil moisture data, last watering time, threshold, watering duration, and a manual watering button
    - Pot name and worker name are stored in fixed 65-byte buffers, so the web UI clamps names to 64 UTF-8 bytes total, which is about 64 ASCII characters or about 16 4-byte emoji/CJK characters
  - Callapsed log list and system health info

- Hardware
  - Main firmware supports multiple hardware targets selected at build time
  - `HW_TARGET_V1_REV_A`
    - Pump activation pin: GPIO 1
    - Water tank level sensor reading pin: GPIO 3
  - `HW_TARGET_V2_REV_A`
    - Pump activation pin: GPIO 3
    - Water tank level sensor reading pin: GPIO 4

- Logics
  - Time
    - 24-Hour, minute level (HH:mm)
    - Input by user via web UI
    - Local-first operation: NTP is optional and manual HH:mm input must work on a network without Internet access
    - Keep a hidden epoch day so multi-day watering intervals can be measured while only HH:mm is exposed to the user
    - Restore the later sane value from retained system time or the saved checkpoint; otherwise start at 2000-01-01 00:00 UTC as a usable default clock
    - Advance time from the 64-bit monotonic ESP timer and accept that elapsed time while fully powered off may be lost
    - Checkpoint approximate time every six hours and before an intentional main-controller OTA reboot
    - NTP synchronization is asynchronous and attempted at startup, after WiFi reconnect, after reset while connected, or by explicit user request; failure never invalidates the current clock
    - Preserve elapsed time since last watering when manual or NTP correction changes the hidden epoch
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
      - Only activate if one or more reported soil moisture is above threshold
      - Soil sensor ADC value increases as the soil gets drier
  - Watering interval
    - Cooldown timer that no watering triggers
    - Starts from end of last watering
    - Minimum: 60 seconds
    - Maximum: 2,419,200 seconds (28 days)
    - Default: 3600 seconds (1 hour)
  - Data sync interval
    - Cooldown timer until next data change between main and worker nodes
    - In auto mode, after each sync/watering cycle main sends the next sleep delay and workers go to deep sleep for this interval
    - In manual mode, main does not send sleep commands, so paired workers stay awake waiting for commands and unpaired workers keep advertising periodically
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
    - Extended advertising is required
  - Workflow
    - Main node
      - Always on
      - Host HTTP server
      - Monitor water tank level sensor
      - Only accepts status updates from worker MAC addresses that have already been added in the web UI
      - Maintain time
      - In manual mode, probe configured workers about every 30 seconds and do not send sleep commands afterward
      - In auto mode, probe workers on the configured data sync interval, evaluate watering, then send sleep commands for the next cycle
    - Worker node
      - When not paired with a main node, advertise status periodically with target node Bluetooth MAC FF:FF:FF:FF:FF:FF
      - When paired and main auto mode is off, stay awake waiting for commands from main
      - When paired and main sends a sleep command, go to deep sleep until the requested timer expires, then wake and wait for the next probe
      - Monitor soil moisture sensor
    - Worker node connection procedure
      - Not exactly connection, but as long as main node is able to communicate with a configured worker, mark it as connected
      - Worker node starts broadcasting data periodically after power up
      - Main node ignores worker status messages until that worker MAC address is added to the list
      - The first accepted command teaches the worker the main node MAC address
      - If Bluetooth MAC address of worker node is added to list in main node, main node should proceed to standard device timing coordination
    - Device timing coordination procedure
      - This procedure only runs in auto mode
      - Main node calculates next data sync time
      - With calculated delay time for each worker node, all worker should wake up around similar time
      - After watering procedure done, main node should calculate new time and send to workers, then workers go to sleep
        1. Main node broadcasts sleep time to the worker one by one
        2. Worker broadcasts ACK
        3. Worker go to sleep
    - Data sync procedure
      - In auto mode, after data sync interval plus extra small delay, assuming all worker nodes wake up, for each worker in the worker list:
        1. Main node broadcasts probe to the worker
        2. Worker listens and broadcasts data
        3. Main node broadcasts ACK
      - In manual mode, main still probes configured workers periodically, but skips the later sleep-coordination step
    - Watering procedure
      - After data sync, if all conditions meet the watering requirement
        1. For each worker with dry pots, main starts its water-command advertisement
        2. Main starts the pump as soon as that advertisement has started
        3. Worker completes its ACK advertisement and activates valves sequentially
        4. Worker stops the valves and sends `TYPE_EVENT_WATER_DONE`
        5. Main ACKs completion, stops the pump, and continues coordination
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

## Bluetooth Protocol

The main and worker firmware must be deployed as a matched pair. The protocol requires
BLE extended advertising; there is no legacy-advertising or plaintext fallback.

### Envelope

- Company identifier: 2 bytes, `0xFFFF` for this project.
- Target MAC: 6 bytes, or `FF:FF:FF:FF:FF:FF` for an unpaired status broadcast.
- Session ID: 8-byte big-endian random value generated at every boot.
- Sequence: 4-byte big-endian counter, nonzero and unique within the boot session.
- Encrypted body: AES-GCM ciphertext followed by a 16-byte authentication tag.

The authenticated body starts with one message-type byte. Remaining values use TLV
fields encoded as `[field type:1][length:1][value:length]`; integers are big-endian.

### Messages

- `0x20 TYPE_ACK`: no TLV fields. It repeats the acknowledged session and sequence.
- `0x30 TYPE_CMD_PROBE`: no TLV fields.
- `0x31 TYPE_CMD_SLEEP`: `FIELD_SLEEP_SEC` as one `uint32`.
- `0x32 TYPE_CMD_WATER`: `FIELD_POT_MASK` plus `FIELD_DURATION_LIST`.
- `0x40 TYPE_STATUS`: battery, pot count, and the ordered `uint16` soil list.
- `0x41 TYPE_CONFIG`: reserved placeholder; it is not currently transmitted.
- `0x42 TYPE_EVENT_WATER_DONE`: the completed pot mask.

ACK matching uses source MAC, session ID, and sequence. For an unpaired broadcast status,
the first authenticated ACK with the matching message ID completes the transaction but
does not pair the worker.

### Encryption

AES-GCM uses the target MAC, session ID, and sequence as authenticated header data.
The 12-byte IV is the first 12 bytes of:

`HMAC-SHA256(network_key, target_mac || session_id || sequence || "btiv")`

Decryption failure rejects the packet. The current compiled network key is a development
placeholder and must be provisioned securely for a production deployment.

### Runtime Flow

- One sender task owns the advertising instance for the complete advertisement lifetime.
  ACK jobs are inserted at the front of its bounded queue.
- Main probes one configured worker at a time, requires the probe ACK, and then waits for
  a newer status generation before using that worker's readings.
- Main ignores and does not ACK status from workers absent from its configured list.
- Worker learns a main MAC only from a valid supported command, never from an ACK.
- Worker scan callbacks only validate and queue commands. A control task sends the ACK,
  waits for its advertisement to finish, and then performs watering or deep sleep.
- For automatic watering, the pump starts when the water-command advertisement actually
  starts. A missing command ACK does not stop the pump; completion, timeout, or low tank
  level ends the watering window.
- Sleep commands are transactional. A worker enters deep sleep only after its ACK
  advertisement has completed.

## Intentional Placeholders

- `TYPE_CONFIG`, heap monitoring, and the incomplete 16-pot hardware target remain.
- Main and worker retain separate copies of Bluetooth common and crypto sources so each
  firmware directory remains independently manageable.



