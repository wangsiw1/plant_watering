#include <Arduino.h>
#include "nvs_flash.h"
#include "BluetoothWorker.h"
#include "Sensor.h"
#include "Valve.h"
#include "Battery.h"

// Default periodic status interval (seconds) while not connected to main
static const uint32_t STATUS_INTERVAL = 10;
static const uint32_t SENSOR_INTERVAL = 5;

void TaskSensors(void *pvParameters) {
	(void) pvParameters;
	for (;;) {
		readWorkerSensors();
		vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL * 1000));
	}
}

void setup() {
	Serial.begin(115200);
	// analogSetAttenuation(ADC_11db);
	vTaskDelay(pdMS_TO_TICKS(3000));
	char btmac[13]; getBtMacHex(btmac);
	LOG("Bluetooth MAC address: %s", btmac);
	
  	esp_log_level_set("gpio", ESP_LOG_WARN);
  	esp_log_level_set("NimBLE", ESP_LOG_WARN);
	
    battBegin();
    sensorBegin();
    valveBegin();
    readWorkerSensors();

	btWorkerBegin();
    
	xTaskCreate(TaskSensors, "sensorTask", 3072, NULL, 2, NULL);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(STATUS_INTERVAL * 1000));
	// No communication from main over certain time, assuming lost connection, reset main mac and start broadcast again
	if (btLastCommOverdue()) mainMacReset();
	if (!mainMacIsSet()) {
		// read sensor and advertise periodically
		btWorkerAdvertiseStatus();
	}
}

// If you are using C++ (which you are for NimBLE), 
// app_main must be declared with extern "C"
extern "C" void app_main() {
	esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 1. Initialize the Arduino stack
    initArduino();

    // 2. Call your setup function
    setup();

    // 3. In ESP-IDF, you usually create a task for the loop 
    // or just run a while(1) here.
    while (true) {
        loop();
        // Crucial: vTaskDelay prevents the Watchdog Timer from 
        // screaming because the loop is hogging the CPU.
        vTaskDelay(1 / portTICK_PERIOD_MS); 
    }
}
