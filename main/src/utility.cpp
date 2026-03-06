#include "Utility.h"
#include "config.h"


static String macWifiLast6() {
	uint8_t mac[6];
	esp_read_mac(mac, ESP_MAC_WIFI_STA);
	char buf[13];
	sprintf(buf, "%02X%02X%02X%02X%02X%02X", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
	String s(buf);
	s.toLowerCase();
	return s.substring(6);
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
    static String value = macWifiLast6();
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
