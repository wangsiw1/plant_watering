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
    - Requires the external 32.768 kHz RTC crystal
  - `HW_V2_8POT_REV_B`
    - Pot count: 8
    - Same general architecture as rev A with alternate pin assignments
    - Requires the external 32.768 kHz RTC crystal
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
      - Firmware version
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
- Status includes `FIELD_FW_VERSION` (`0x07`) as a big-endian `uint32`.

Worker OTA accepts upgrades and rollbacks at or above the build-defined
`FW_MIN_ALLOWED_VERSION`. It rejects the currently installed version and any package
below that floor before firmware image transfer begins.

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

## V2 RTC Clock Boot Gate

Both V2 8-pot workers require the external 32.768 kHz crystal as the RTC slow
clock. ESP-IDF requests that source with 3000 calibration cycles. The application
then checks the source selected by ESP-IDF at the beginning of `app_main()`, before
NVS, Arduino, sensors, valves, Bluetooth, or OTA are initialized.

- `SOC_RTC_SLOW_CLK_SRC_XTAL32K` passes the gate and normal startup continues.
- Any internal or invalid source fails the gate. GPIO8 is configured as the
  active-low onboard blue LED (low is on; high is off) and displays the RTC
  fault pattern below.
- A successful check produces no flash. The LED is explicitly turned off before
  startup continues, and `sensorBegin()` later takes ownership of GPIO8 as
  `MUX_SEL_PIN1`. No LED controller writes GPIO8 during normal operation; visible
  activity caused by multiplexer selection is incidental.

The RTC failure indication uses half-open intervals and is evaluated every 10 ms:

| Cycle interval | LED state |
|---|---|
| 0-100 ms | On |
| 100-200 ms | Off |
| 200-300 ms | On |
| 300-400 ms | Off |
| 400-500 ms | On |
| 500-2000 ms | Off |

This two-second cycle repeats 30 times, for exactly 60 seconds of fault
indication. The LED is then turned off and the worker enters timer-only deep sleep
for five seconds. Deep-sleep wake runs ESP-IDF clock initialization again, which
re-enables and calibrates the external crystal. A recovered crystal permits normal
startup; a continuing failure starts another indication and recovery cycle. The
fallback clock may make the five-second recovery interval imprecise, which is
acceptable for this retry.

Deep-sleep recovery retains the worker's `RTC_DATA_ATTR` main-device MAC and last
command ID. The XTAL32K watchdog is configured with a timeout value of 200 and
automatic backup-clock switching, so a crystal failure during deep sleep switches
RTC timing to the RC-derived backup and allows the timer wakeup to continue. No
runtime callback or fault task is installed: if the worker is awake when a failure
occurs, it may continue temporarily on the backup clock until its next deep-sleep
wake or reset.



