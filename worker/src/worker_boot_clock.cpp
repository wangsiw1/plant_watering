#include "WorkerBootClock.h"

#include "HardwareConfig.h"

#if WORKER_REQUIRES_EXT_RTC_XTAL

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "soc/rtc.h"

#if !CONFIG_RTC_CLK_SRC_EXT_CRYS
#error "V2 worker requires CONFIG_RTC_CLK_SRC_EXT_CRYS"
#endif

#if !CONFIG_ESP_XT_WDT
#error "V2 worker requires CONFIG_ESP_XT_WDT"
#endif

#if !CONFIG_ESP_XT_WDT_BACKUP_CLK_ENABLE
#error "V2 worker requires CONFIG_ESP_XT_WDT_BACKUP_CLK_ENABLE"
#endif

namespace {
constexpr char TAG[] = "worker_boot_clock";
constexpr gpio_num_t BOOT_LED_GPIO =
    static_cast<gpio_num_t>(WORKER_BOOT_LED_PIN);
constexpr uint32_t BOOT_LED_ON_LEVEL =
    WORKER_BOOT_LED_ACTIVE_LOW ? 0u : 1u;
constexpr uint32_t BOOT_LED_OFF_LEVEL =
    WORKER_BOOT_LED_ACTIVE_LOW ? 1u : 0u;

static_assert(WORKER_BOOT_LED_PIN == MUX_SEL_PIN1,
              "Boot LED must share the runtime mux-select pin");

uint32_t elapsedMilliseconds(TickType_t startedAt) {
  const TickType_t elapsedTicks = xTaskGetTickCount() - startedAt;
  return static_cast<uint32_t>(
      (static_cast<uint64_t>(elapsedTicks) * 1000u) / configTICK_RATE_HZ);
}

void writeBootLed(bool on) {
  gpio_set_level(BOOT_LED_GPIO,
                 on ? BOOT_LED_ON_LEVEL : BOOT_LED_OFF_LEVEL);
}
}  // namespace

#endif  // WORKER_REQUIRES_EXT_RTC_XTAL

void workerBootRtcClockGate() {
#if WORKER_REQUIRES_EXT_RTC_XTAL
  gpio_reset_pin(BOOT_LED_GPIO);
  gpio_set_direction(BOOT_LED_GPIO, GPIO_MODE_OUTPUT);
  writeBootLed(false);

  const soc_rtc_slow_clk_src_t source = rtc_clk_slow_src_get();
  if (workerBootRtcSourceIsExternal(source)) return;

  ESP_LOGE(TAG, "RTC slow clock source=%u, expected external XTAL32K",
           static_cast<unsigned>(source));

  const TickType_t startedAt = xTaskGetTickCount();
  TickType_t lastUpdate = startedAt;
  for (;;) {
    const uint32_t elapsedMs = elapsedMilliseconds(startedAt);
    if (workerBootRtcFaultShouldRetry(elapsedMs)) break;
    writeBootLed(workerBootRtcFaultLedOn(elapsedMs));
    vTaskDelayUntil(&lastUpdate, pdMS_TO_TICKS(WORKER_RTC_FAULT_UPDATE_MS));
  }

  writeBootLed(false);
  ESP_LOGE(TAG, "Retrying XTAL32K after 5-second recovery sleep");
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_deep_sleep(WORKER_RTC_FAULT_RETRY_SLEEP_US);
#endif
}
