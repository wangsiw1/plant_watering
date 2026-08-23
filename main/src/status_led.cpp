#include "StatusLed.h"

#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "HardwareConfig.h"
#include "OtaManager.h"
#include "Pump.h"
#include "Sensor.h"
#include "Utility.h"
#include "WorkerOtaManager.h"
#include "config.h"

namespace {
constexpr uint8_t STATUS_LED_LEDC_CHANNEL = 0;
constexpr uint16_t STATUS_LED_TASK_STACK_SIZE = 2048;
constexpr UBaseType_t STATUS_LED_TASK_PRIORITY = 1;

TaskHandle_t gStatusLedTask = nullptr;

uint8_t logicalBrightnessToDuty(uint8_t brightness) {
#if STATUS_LED_ACTIVE_LOW
  return static_cast<uint8_t>(255u - brightness);
#else
  return brightness;
#endif
}

void writeBrightness(uint8_t brightness) {
  ledcWrite(STATUS_LED_LEDC_CHANNEL,
            logicalBrightnessToDuty(brightness));
}

void statusLedTask(void*) {
  StatusLedConnectionState connectionState{};
  StatusLedPatternState patternState{};
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    uint32_t nowMs = millis();
    bool wifiConnected = WiFi.status() == WL_CONNECTED;
    bool connectionFlashActive = statusLedUpdateConnectionFlash(
        connectionState, wifiConnected, nowMs);

    RuntimeSnapshot runtime{};
    getRuntimeSnapshot(runtime);
    bool tankReady = tankSensorIsReady();
    StatusLedInputs inputs{
        otaIsActive() || workerOtaIsActive(),
        tankReady,
        tankReady && isTankLow(),
        wifiConnected,
        connectionFlashActive,
        runtime.state == WATERING || pumpIsOn(),
        runtime.state == SYNCING,
    };
    StatusLedPattern selected = statusLedSelectPattern(inputs);
    uint32_t patternElapsedMs =
        statusLedUpdatePatternPhase(patternState, selected, nowMs);
    writeBrightness(statusLedPatternBrightness(
        patternState.pattern, patternElapsedMs));
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(STATUS_LED_UPDATE_INTERVAL_MS));
  }
}
}  // namespace

bool statusLedBegin() {
  if (gStatusLedTask) return true;

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, STATUS_LED_ACTIVE_LOW ? HIGH : LOW);
  uint32_t actualFrequency = ledcSetup(
      STATUS_LED_LEDC_CHANNEL, STATUS_LED_PWM_FREQUENCY_HZ,
      STATUS_LED_PWM_RESOLUTION_BITS);
  if (actualFrequency == 0) {
    LOG("Status LED PWM setup failed");
    return false;
  }
  ledcAttachPin(STATUS_LED_PIN, STATUS_LED_LEDC_CHANNEL);
  writeBrightness(0);

  if (xTaskCreate(statusLedTask, "statusLedTask",
                  STATUS_LED_TASK_STACK_SIZE, nullptr,
                  STATUS_LED_TASK_PRIORITY, &gStatusLedTask) != pdPASS) {
    gStatusLedTask = nullptr;
    writeBrightness(0);
    LOG("Status LED task creation failed");
    return false;
  }
  LOG("Status LED initialized pin=%u pwm_hz=%lu",
      static_cast<unsigned>(STATUS_LED_PIN),
      static_cast<unsigned long>(actualFrequency));
  return true;
}
