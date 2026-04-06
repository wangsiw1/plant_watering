#include "Utility.h"
#include "config.h"


static String getMacAddr(esp_mac_type_t device) {
	uint8_t mac[6];
	esp_read_mac(mac, device);
	char buf[13];
	sprintf(buf, "%02X%02X%02X%02X%02X%02X", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
	String s(buf);
	s.toLowerCase();
	return s;
}

bool macFromHexString(const String &macHex, uint8_t out[6]) {
  String s = macHex;
  s.replace(":", "");
  s.replace("-", "");
  s.trim();
  if (s.length() != 12) return false;
  for (int i=0;i<6;i++) {
    String part = s.substring(i*2, i*2+2);
    char buf[3]; part.toCharArray(buf,3);
    char* endptr;
    long v = strtol(buf, &endptr, 16);
    if (*endptr != '\0') return false;
    out[i] = (uint8_t)v;
  }
  return true;
}

const String& getWifiMacLast6()
{
    static String value = getMacAddr(ESP_MAC_WIFI_STA).substring(6);
    return value;
}

const String& getBtMac()
{
    static String value = getMacAddr(ESP_MAC_BT);
    return value;
}

void setUserTimeOfDaySec(uint32_t secOfDay) {
  // store into global settings and persist via saveSettings()
  settings.savedTimeOfDaySec = secOfDay % 86400;
  settings.savedMillis = millis();
  saveSettings();
}

uint32_t getCurrentTimeOfDaySec() {
  // compute delta millis wrap-safe
  unsigned long now = millis();
  unsigned long delta = now - settings.savedMillis;
  uint32_t addSec = (uint32_t)(delta / 1000UL);
  return (settings.savedTimeOfDaySec + addSec) % 86400u;
}

uint32_t calculateSleepSec(unsigned long now_s) {
  // Calculate sleep duration until next data sync and instruct workers to sleep
  // Aim for workers to wake around next data sync plus a small skew (5s).
  unsigned long sinceLastSync = (now_s > lastDataSync) ? (now_s - lastDataSync) : 0;
  // desired delay from now until next sync moment
  long desired = (long)settings.dataSyncInterval + 5 - (long)sinceLastSync;
  if (desired < 60) desired = 60; // enforce minimum sleep
  if (desired > (long)settings.dataSyncInterval) desired = settings.dataSyncInterval; // cap to configured max
  uint32_t sleepSec = (uint32_t)desired;

  return sleepSec;
}
