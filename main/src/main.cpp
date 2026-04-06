#include <Arduino.h>
#include <WiFi.h>
// #include <ArduinoOTA.h>
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

// BT MAC: 10003bcc9cde

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
		readTankLevel();
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void TaskWatering(void *pvParameters) {
	(void) pvParameters;
	for (;;) {
    static unsigned long lastCheck = millis();
		unsigned long t = millis();
		unsigned long now_s = t / 1000;
		if (!autoEnabled) state = READY;

		// Data sync: probe workers if it's time or auto is not enabled
		if (!autoEnabled || now_s - lastDataSync >= settings.dataSyncInterval) {
			// If auto is not enabled, sync every 30 seconds
			if (!autoEnabled && t - lastCheck <= 30000) {
				vTaskDelay(pdMS_TO_TICKS(500));
				continue;
			}
			lastCheck = t;
			state = SYNCING;

			int nc = btMainNodeCount();
			for (int i = 0; i < nc; ++i) {
				const WorkerNode* n = btMainNodeAt(i);
				if (!n) continue;
				uint8_t probePayload[2]; probePayload[0] = BT_TLV::TYPE_CMD_PROBE; probePayload[1] = 0; // TLV: type + len=0
				btMainQueueCommand(n->mac, probePayload, sizeof(probePayload), 1, 500);
				vTaskDelay(pdMS_TO_TICKS(150));
			}
			lastDataSync = now_s;
		}

		// Check watering interval
		if (!autoEnabled || now_s - lastWateringEnd < settings.waterInterval) {
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
		state = WATERING;
		std::vector<const WorkerConfig*> toWater;
		for (int wi = 0; wi < workerListCount; ++wi) {
			const WorkerConfig &wc = workerList[wi];
			const WorkerNode* n = btMainFindNodeByMac(wc.mac);
			uint16_t thr = wc.threshold;
			// skip if no data or not synced within recent interval
			if (!n || (now_s > n->lastSync && now_s - n->lastSync > settings.dataSyncInterval)) continue;
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

		state = SLEEPING;
		uint32_t sleepSec = calculateSleepSec(now_s);
		for (int wi = 0; wi < workerListCount; ++wi) {
			const WorkerConfig &wc = workerList[wi];
			uint8_t payload[6];
			payload[0] = BT_TLV::TYPE_CMD_SLEEP; // TLV type
			payload[1] = 4; // length
			// encode big-endian uint32 seconds
			payload[2] = (uint8_t)((sleepSec >> 24) & 0xFF);
			payload[3] = (uint8_t)((sleepSec >> 16) & 0xFF);
			payload[4] = (uint8_t)((sleepSec >> 8) & 0xFF);
			payload[5] = (uint8_t)(sleepSec & 0xFF);
			btMainQueueCommand(wc.mac, payload, sizeof(payload), 2, 700);
			vTaskDelay(pdMS_TO_TICKS(50));
		}
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

void setup() {
	Serial.begin(115200);
	vTaskDelay(pdMS_TO_TICKS(3000));
	LOG("Bluetooth MAC address: %s", getBtMac());

	loadSettings();

	pumpBegin();
	sensorBegin();
	btMainBegin();

	// Create the WiFiProvisioner instance
  	WiFiProvisioner provisioner;
	provisioner.onSuccess([](const char *ssid, const char *password, const char *input) {
		saveWifiCred(ssid, password);
	});
	
	// Configure to hide additional fields
	provisioner.getConfig().SHOW_INPUT_FIELD = false; // No additional input field
	provisioner.getConfig().SHOW_RESET_FIELD = false; // No reset field

	// Start provisioning
	if (!connectToWiFi()) {
		provisioner.startProvisioning();
    	LOG("Waiting for provisioning...");
		while (WiFi.status() != WL_CONNECTED) {
			delay(500);
		}
		LOG("Provisioning complete, continuing setup.");
	}

	String mdnsName = "plant-watering-" + getWifiMacLast6();
	if (!MDNS.begin(mdnsName.c_str())) {
		LOG("Error starting mDNS");
	} else {
		LOG("mDNS started: %s.local", mdnsName.c_str());
	}

	// ArduinoOTA.setHostname(mdnsName.c_str());
	// ArduinoOTA.setPassword("watering");
	// ArduinoOTA.begin();

	webBegin();

	xTaskCreate(TaskWeb, "webTask", 4096, NULL, 1, NULL);
	xTaskCreate(TaskSensor, "sensorTask", 2048, NULL, 2, NULL);
	xTaskCreate(TaskWatering, "waterTask", 4096, NULL, 2, NULL);
}

void loop() {
	// Let FreeRTOS tasks do the work. Keep loop empty.
	// ArduinoOTA.handle();
	vTaskDelay(pdMS_TO_TICKS(1000));
}
