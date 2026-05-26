#pragma once
#include <Arduino.h>

// Hardware target selection
// Define exactly one of the HW_TARGET_* macros via build flags.
// Examples: -D HW_V1_1POT_REV_A, -D HW_V2_8POT_REV_A, -D HW_TARGET_8P_REV_B

#if defined(HW_V1_1POT_REV_A)

#define WORKER_POT_COUNT 1
#define SOIL_PIN 3
#define VALVE_PIN 1
#define BATT_PIN 4

#elif defined(HW_V2_8POT_REV_A)

#define WORKER_POT_COUNT 8

#define SOIL_EN_PIN 6
#define MUX_ADC_PIN 5
#define MUX_SEL_PIN0 2
#define MUX_SEL_PIN1 8
#define MUX_SEL_PIN2 9

#define SHIFT_DATA_PIN 10
#define SHIFT_CLOCK_PIN 20
#define SHIFT_LATCH_PIN 21

#define BATT_EN_PIN 7
#define BATT_ADC_PIN 3

#define SHUTDOWN_PIN 4

#elif defined(HW_V2_8POT_REV_B)

#define WORKER_POT_COUNT 8

#define SOIL_EN_PIN 6
#define MUX_ADC_PIN 4
#define MUX_SEL_PIN0 2
#define MUX_SEL_PIN1 8
#define MUX_SEL_PIN2 9

#define SHIFT_DATA_PIN 10
#define SHIFT_CLOCK_PIN 20
#define SHIFT_LATCH_PIN 21

#define BATT_EN_PIN 7
#define BATT_ADC_PIN 3

#define SHUTDOWN_PIN 5

#elif defined(HW_TARGET_16POT_REV_A)

#define WORKER_POT_COUNT 16
#define MUX_ADC_PIN 3
#define MUX_SEL_PIN0 5
#define MUX_SEL_PIN1 6
#define MUX_SEL_PIN2 7
#define SHIFT_DATA_PIN 9
#define SHIFT_CLOCK_PIN 10
#define SHIFT_LATCH_PIN 11
#define BATT_PIN 4

#else
#error "Please define one HW_TARGET_* build flag (e.g. HW_V1_1POT_REV_A)"
#endif

// Derived constants
#define SHIFT_REG_CHIPS ((WORKER_POT_COUNT + 7) / 8)
