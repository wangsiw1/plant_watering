#pragma once

#include <stdint.h>
#include <math.h>

constexpr uint32_t STATUS_LED_PWM_FREQUENCY_HZ = 5000;
constexpr uint8_t STATUS_LED_PWM_RESOLUTION_BITS = 8;
constexpr uint32_t STATUS_LED_UPDATE_INTERVAL_MS = 20;
constexpr uint32_t STATUS_LED_WIFI_CYCLE_MS = 2000;
constexpr uint32_t STATUS_LED_CONNECTED_FLASH_MS = 250;

enum class StatusLedPattern : uint8_t {
  OFF,
  SYNCING,
  WATERING,
  WIFI_BREATHING,
  WIFI_CONNECTED,
  TANK_LOW,
  OTA_UPDATING,
};

struct StatusLedInputs {
  bool otaActive;
  bool tankSensorReady;
  bool tankLow;
  bool wifiConnected;
  bool wifiJustConnected;
  bool watering;
  bool syncing;
};

struct StatusLedConnectionState {
  bool wasConnected = false;
  bool flashActive = false;
  uint32_t flashStartedMs = 0;
};

struct StatusLedPatternState {
  StatusLedPattern pattern = StatusLedPattern::OFF;
  uint32_t startedMs = 0;
  bool initialized = false;
};

inline bool statusLedUpdateConnectionFlash(StatusLedConnectionState& state,
                                           bool connected,
                                           uint32_t nowMs) {
  if (connected && !state.wasConnected) {
    state.flashActive = true;
    state.flashStartedMs = nowMs;
  } else if (!connected) {
    state.flashActive = false;
  }
  state.wasConnected = connected;

  if (state.flashActive &&
      nowMs - state.flashStartedMs >= STATUS_LED_CONNECTED_FLASH_MS) {
    state.flashActive = false;
  }
  return state.flashActive;
}

inline uint32_t statusLedUpdatePatternPhase(StatusLedPatternState& state,
                                            StatusLedPattern selected,
                                            uint32_t nowMs) {
  if (!state.initialized || selected != state.pattern) {
    state.pattern = selected;
    state.startedMs = nowMs;
    state.initialized = true;
  }
  return nowMs - state.startedMs;
}

inline StatusLedPattern statusLedSelectPattern(const StatusLedInputs& inputs) {
  if (inputs.otaActive) return StatusLedPattern::OTA_UPDATING;
  if (inputs.tankSensorReady && inputs.tankLow) {
    return StatusLedPattern::TANK_LOW;
  }
  if (!inputs.wifiConnected) return StatusLedPattern::WIFI_BREATHING;
  if (inputs.wifiJustConnected) return StatusLedPattern::WIFI_CONNECTED;
  if (inputs.watering) return StatusLedPattern::WATERING;
  if (inputs.syncing) return StatusLedPattern::SYNCING;
  return StatusLedPattern::OFF;
}

inline uint8_t statusLedPatternBrightness(StatusLedPattern pattern,
                                          uint32_t elapsedMs) {
  switch (pattern) {
    case StatusLedPattern::OTA_UPDATING:
      return elapsedMs % 200u < 100u ? 255u : 0u;

    case StatusLedPattern::TANK_LOW:
      return elapsedMs % 1000u < 500u ? 255u : 0u;

    case StatusLedPattern::WIFI_BREATHING: {
      constexpr float STATUS_LED_PI = 3.14159265358979323846f;
      constexpr float MIN_BRIGHTNESS = 0.10f * 255.0f;
      constexpr float BRIGHTNESS_RANGE = 0.60f * 255.0f;
      float phase = static_cast<float>(elapsedMs % STATUS_LED_WIFI_CYCLE_MS) /
                    static_cast<float>(STATUS_LED_WIFI_CYCLE_MS);
      float eased = 0.5f - 0.5f * cosf(2.0f * STATUS_LED_PI * phase);
      return static_cast<uint8_t>(MIN_BRIGHTNESS +
                                  BRIGHTNESS_RANGE * eased + 0.5f);
    }

    case StatusLedPattern::WIFI_CONNECTED:
      return elapsedMs < STATUS_LED_CONNECTED_FLASH_MS ? 255u : 0u;

    case StatusLedPattern::WATERING: {
      uint32_t phaseMs = elapsedMs % 3000u;
      return (phaseMs < 100u ||
              (phaseMs >= 200u && phaseMs < 300u) ||
              (phaseMs >= 400u && phaseMs < 500u))
                 ? 255u
                 : 0u;
    }

    case StatusLedPattern::SYNCING: {
      uint32_t phaseMs = elapsedMs % 3000u;
      return (phaseMs < 300u)
                 ? 255u
                 : 0u;
    }

    case StatusLedPattern::OFF:
    default:
      return 0u;
  }
}

// Initializes GPIO8 PWM and the independent status task. Safe to call once.
bool statusLedBegin();
