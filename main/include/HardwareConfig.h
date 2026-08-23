#pragma once
#include <Arduino.h>

// Hardware target selection
// Define exactly one of the HW_TARGET_* macros via build flags.
// Examples: -D HW_TARGET_V1_REV_A, -D HW_TARGET_V2_REV_A

// The ESP32-C3 SuperMini-style development board used by both main hardware
// revisions connects its onboard blue status LED to GPIO8. The LED sinks
// current through the GPIO, so its output is active-low.
#define STATUS_LED_PIN 8
#define STATUS_LED_ACTIVE_LOW 1

#if defined(HW_TARGET_V1_REV_A)

#define TANK_SENSOR_PIN 3
#define PUMP_PIN 1

#elif defined(HW_TARGET_V2_REV_A)

#define TANK_SENSOR_PIN 4
#define PUMP_PIN 3

#else
#error "Please define one HW_TARGET_* build flag (e.g. HW_TARGET_V2_REV_A)"
#endif
