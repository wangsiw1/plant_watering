#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <vector>
#include <WiFiProvisioner.h>
#include "Config.h"
#include "Pump.h"
#include "Sensor.h"
#include "WebUI.h"
#include "BluetoothMain.h"
#include "Utility.h"
#include "WateringManager.h"

extern "C" {
	void vApplicationIdleHook();
}

// --- FreeRTOS tasks ---
void TaskWeb(void *pvParameters) {
	(void) pvParameters;
	for (;;) {
		webHandleClientLoop();
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void TaskSensor(void *pvParameters) {
	(void) pvParameters;
	for (;;) {
		gTankLevel = analogRead(3);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void TaskWatering(void *pvParameters) {
	(void) pvParameters;
	for (;;) {
		static unsigned long lastCheck = 0;
		unsigned long t = millis();

		if (t - lastCheck <= 1000) {
			vTaskDelay(pdMS_TO_TICKS(500));
			continue;
		}
		lastCheck = t;

		unsigned long now_s = t / 1000;

		// Data sync: probe workers if it's time or auto is not enabled
		if (!autoEnabled || now_s - lastDataSync >= settings.dataSyncInterval) {
			int nc = btMainNodeCount();
			for (int i = 0; i < nc; ++i) {
				const WorkerNode* n = btMainNodeAt(i);
				if (!n) continue;
				uint8_t probePayload[1]; probePayload[0] = 0x01; // CMD_PROBE
				btMainQueueCommand(n->mac, probePayload, 1, 1, 500);
				vTaskDelay(pdMS_TO_TICKS(150));
			}
			lastDataSync = now_s;
		}

		if (!autoEnabled) {
			vTaskDelay(pdMS_TO_TICKS(500));
			continue;
		}

		// Check watering interval
		if (now_s - lastWateringEnd < settings.waterInterval) {
			vTaskDelay(pdMS_TO_TICKS(500));
			continue;
		}

		unsigned long secOfDay = getCurrentTimeOfDaySec();
		bool inWindow = (settings.activeStart <= settings.activeEnd) ? (secOfDay >= settings.activeStart && secOfDay <= settings.activeEnd) : (secOfDay >= settings.activeStart || secOfDay <= settings.activeEnd);
		if (!inWindow) {
			vTaskDelay(pdMS_TO_TICKS(500));
			continue;
		}

		// evaluate worker soils and collect targets
		const uint16_t DEFAULT_THRESHOLD = 2000;
		const uint16_t DEFAULT_DURATION = 5; // seconds per valve
		std::vector<const WorkerConfig*> toWater;
		for (int wi = 0; wi < workerListCount; ++wi) {
			const WorkerConfig &wc = workerList[wi];
			const WorkerNode* n = btMainFindNodeByMac(wc.mac);
			uint16_t thr = wc.threshold;
			// skip if no data or not synced within recent interval
			if (!n || (now_s > n->lastSeen && now_s - n->lastSeen > settings.dataSyncInterval)) continue;
			if (n->soil > 0 && n->soil < thr) toWater.push_back(&workerList[wi]);
		}

		if (toWater.empty()) {
			vTaskDelay(pdMS_TO_TICKS(500));
			continue;
		}

		// Start pump and instruct workers
		pumpOn();
		startWatering(toWater);
		pumpOff();

		lastWateringEnd = millis() / 1000;

		uint32_t sleepSec = calculateSleepSec(now_s);
		for (int wi = 0; wi < workerListCount; ++wi) {
			const WorkerConfig &wc = workerList[wi];
			uint8_t payload[1 + 4];
			payload[0] = 0x02; // CMD_SLEEP
			// encode big-endian uint32 seconds
			payload[1] = (uint8_t)((sleepSec >> 24) & 0xFF);
			payload[2] = (uint8_t)((sleepSec >> 16) & 0xFF);
			payload[3] = (uint8_t)((sleepSec >> 8) & 0xFF);
			payload[4] = (uint8_t)(sleepSec & 0xFF);
			btMainQueueCommand(wc.mac, payload, sizeof(payload), 2, 700);
			vTaskDelay(pdMS_TO_TICKS(50));
		}
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

void setup() {
	Serial.begin(115200);

	loadSettings();

	pumpBegin();
	sensorBegin();
	btMainBegin();

	// Create the WiFiProvisioner instance
  	WiFiProvisioner provisioner;
	
	// Configure to hide additional fields
	provisioner.getConfig().SHOW_INPUT_FIELD = false; // No additional input field
	provisioner.getConfig().SHOW_RESET_FIELD = false; // No reset field

	// Start provisioning
  	provisioner.startProvisioning();

	String mdnsName = "plant-watering-" + getWifiMacLast6();
	if (!MDNS.begin(mdnsName.c_str())) {
		Serial.println("Error starting mDNS");
	} else {
		Serial.printf("mDNS started: %s.local\n", mdnsName.c_str());
	}

	ArduinoOTA.setHostname(mdnsName.c_str());
	ArduinoOTA.setPassword("watering");
	ArduinoOTA.begin();

	webBegin();

	xTaskCreate(TaskWeb, "webTask", 4096, NULL, 1, NULL);
	xTaskCreate(TaskSensor, "sensorTask", 2048, NULL, 2, NULL);
	xTaskCreate(TaskWatering, "waterTask", 4096, NULL, 2, NULL);
}

void loop() {
	// Let FreeRTOS tasks do the work. Keep loop empty.
	ArduinoOTA.handle();
	vTaskDelay(pdMS_TO_TICKS(1000));
}
