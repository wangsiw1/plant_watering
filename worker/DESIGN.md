# ESP32-C3 pot plan watering system — Design & Notes

This document collects requirements, design decisions, implementation notes of software for the worker device of ESP32-C3 pot plant watering project.

Note: Hardware specifications other than ESP32 C3 is not in the scope.

## Overview

Goal: As a worker device, it will have a capacitive soil moisture sensor and a solenoid valve connected. The device will read soil sensor data and send to main device via Bluetooth broadcasting, active or deactive pump, or go to sleep following the command from main devic. 

### Details

- Hardware
  - Worker firmware supports multiple hardware targets selected at build time
  - `HW_V1_1POT_REV_A`
    - Pot count: 1
    - Valve activation pin: GPIO 1
    - Battery pin: GPIO 4
    - Soil moisture sensor reading pin: GPIO 3
  - `HW_V2_8POT_REV_A`
    - Pot count: 8
    - Multiplexed soil sensing, shift-register valve control, and separate battery enable/ADC pins
  - `HW_V2_8POT_REV_B`
    - Pot count: 8
    - Same general architecture as rev A with alternate pin assignments
  - `HW_TARGET_16POT_REV_A`
    - Pot count: 16
    - Multiplexed soil sensing with shift-register valve control

- Logics
  - Data sync interval
    - Cooldown timer until next data change between main and worker nodes
    - In auto mode, the worker enters deep sleep only after main sends a sleep command with the next delay
    - In manual mode, the worker does not sleep for this interval and instead stays awake if paired
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
    - Worker node
      - When not paired with a main node, advertise status periodically with target node Bluetooth MAC FF:FF:FF:FF:FF:FF
      - When paired and the main node is running manual mode, stay awake waiting for commands from that main node
      - When paired and the main node sends a sleep command, go to deep sleep, wake after the requested timer, and wait for the next probe
      - Monitor soil moisture sensor
    - Worker node connection procedure
      - Not exactly connection, but as long as main node is able to communicate with worker, mark it as connected
      - Worker node starts broadcasting data periodically after power up
      - Main node only proceeds after that worker MAC address is explicitly added to the configured list
      - The first accepted command teaches the worker which main-node MAC address to trust afterward
    - Device timing coordination procedure
      - This procedure only applies while the main node is in auto mode
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
      - In manual mode, the worker still responds to periodic probes from main, but does not expect a follow-up sleep command
    - Watering procedure
      - After data sync, if all conditions meet the watering requirement
        1. Main starts the water-command advertisement and then starts the pump
        2. Worker completes its ACK advertisement before operating any valve
        3. Worker waters selected pots sequentially and sends `TYPE_EVENT_WATER_DONE`
        4. Main ACKs completion and stops the pump
      - If conditions not meet, skip to device timing coordination procedure
    - Worker node retry and disconnect procudure
      - Either main node or worker node is not responding to message after certain time lenght, retry 3 times
      - If worker node does not respond after retries, main node marks it as disconnected and continue
      - If main node does not respond to worker node, reset the worker node itself and start over from worker node connection procedure

## Bluetooth Protocol

Main and worker firmware are a matched protocol pair. Extended advertising and AES-GCM
are mandatory; packets that cannot be authenticated are discarded.

The manufacturer-data envelope is:

- Company identifier: 2 bytes (`0xFFFF`).
- Target MAC: 6 bytes.
- Random boot session ID: 8 bytes, big-endian.
- Per-session sequence: 4 bytes, big-endian.
- AES-GCM encrypted body and 16-byte authentication tag.

The body begins with a message type followed by TLVs encoded as
`[field type][length][value]`. ACKs repeat the message ID in the envelope and are matched
with the advertiser MAC.

The IV is the first 12 bytes of:

`HMAC-SHA256(network_key, target_mac || session_id || sequence || "btiv")`

The target MAC, session ID, and sequence are also authenticated as GCM associated data.

Worker behavior:

- An unpaired status targets the broadcast MAC. Its ACK completes transmission without
  teaching the worker a main MAC.
- Only a valid probe, water, or sleep command can establish the retained main MAC.
- Scan callbacks authenticate and queue commands without operating hardware.
- The worker-control task completes the command ACK advertisement before acting.
- Water commands operate valves sequentially and finish with `TYPE_EVENT_WATER_DONE`.
- Sleep begins only after the sleep-command ACK advertisement is complete.
- One periodic task samples battery first and soil second every five seconds, publishing
  both through one protected sensor snapshot.

`TYPE_CONFIG` remains reserved and unused. Bluetooth common and crypto files intentionally
remain duplicated between the two independently managed firmware directories.

## Hardware Build Variants

This worker firmware supports multiple hardware variants selected at compile time via a
single `HW_*` build flag. The flag determines pin mappings and the number of pots.

Examples:
- `HW_V1_1POT_REV_A` : original single-pot hardware (SOIL_PIN=3, VALVE_PIN=1)
- `HW_V2_8POT_REV_A` : 8-pot with multiplexer (3 select pins) + one shift-register chip
- `HW_V2_8POT_REV_B` : 8-pot variant with alternate pin assignments

The 16-pot target remains an intentional placeholder with incomplete pin mapping.



