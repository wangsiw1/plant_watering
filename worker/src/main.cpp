#include <Arduino.h>
#include "BluetoothWorker.h"
#include "Sensor.h"
#include "Valve.h"
#include "Battery.h"

// Default periodic status interval (seconds) while not connected to main
static const uint32_t STATUS_INTERVAL = 10;
static const uint32_t SENSOR_INTERVAL = 5;

void TaskSensor(void *pvParameters) {
	(void) pvParameters;
	for (;;) {
		readSoilMoisture();
		vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL * 1000));
	}
}

void TaskBatt(void *pvParameters) {
	(void) pvParameters;
	for (;;) {
		readBattLevel();
		vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL * 1000));
	}
}

void setup() {
	Serial.begin(115200);
	vTaskDelay(pdMS_TO_TICKS(3000));
	LOG("Bluetooth MAC address: %s", getBtMac());
	
    valveBegin();
    sensorBegin();
    battBegin();

	btWorkerBegin();
    
	xTaskCreate(TaskSensor, "sensorTask", 2048, NULL, 2, NULL);
	xTaskCreate(TaskBatt, "battTask", 2048, NULL, 2, NULL);
}

void loop() {
    vTaskDelay(STATUS_INTERVAL * 1000);
	// No communication from main over certain time, assuming lost connection, reset main mac and start broadcast again
	if (btLastCommOverdue()) mainMacReset();
    if (!mainMacIsSet()) {
        // read sensor and advertise periodically
        uint16_t soil = getSoilMoisture();
        uint8_t batt = getBattLevel();
        btWorkerAdvertiseStatus(soil, batt);
    }
}
