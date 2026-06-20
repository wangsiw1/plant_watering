
#include "Utility.h"
#include "heapTrace.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"

void dumpHeap(const char* label) {
    LOG("\n>>>HEAP_DUMP_START label=%s t=%lu free=%lu maxBlock=%lu minEver=%lu\n",
        label, millis(), 
        ESP.getFreeHeap(), 
        ESP.getMaxAllocHeap(),
        ESP.getMinFreeHeap());
    heap_caps_dump(MALLOC_CAP_8BIT); // output goes directly to serial via esp_rom_printf
    LOG("<<<HEAP_DUMP_END label=%s\n\n", label);
    Serial.flush();
}

static void heapMonitorTask(void* pv) {
    // Wait for system to fully boot and stabilise
    vTaskDelay(pdMS_TO_TICKS(15000));
    dumpHeap("BASELINE");

    uint32_t lastFree = ESP.getFreeHeap();
    static const uint32_t LEAK_THRESHOLD = 512; // dump if >512 bytes lost since last check
    static const uint32_t CHECK_INTERVAL  = 60000; // check every 60 seconds

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL));

        uint32_t curFree  = ESP.getFreeHeap();
        uint32_t maxBlock = ESP.getMaxAllocHeap();

        // Always log the one-liner — lightweight, good for trend graphing
        LOG("[HEAP] t=%lu free=%lu maxBlock=%lu minEver=%lu delta=%d\n",
            millis(), curFree, maxBlock, ESP.getMinFreeHeap(),
            (int)curFree - (int)lastFree);
        Serial.flush();

        // Full dump only when a meaningful drop occurs
        if (lastFree > curFree && (lastFree - curFree) >= LEAK_THRESHOLD) {
            dumpHeap("LEAK_DETECTED");
        }

        lastFree = curFree;
    }
}

void heapMonitorInit() {
    xTaskCreate(heapMonitorTask, "heapMon", 3072, NULL, 1, NULL);
}
