#include "Utility.h"

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

const String& getBtMac()
{
    static String value = getMacAddr(ESP_MAC_BT);
    return value;
}

// Trimmed mean: sort values and remove min/max before averaging
uint16_t trimmedMean(uint16_t *values, uint16_t count, uint8_t trimCount) {
  if (count <= 2 * trimCount) return 0; // Not enough samples after trimming
  
  // Simple bubble sort for small arrays
  for (uint16_t i = 0; i < count - 1; i++) {
    for (uint16_t j = 0; j < count - i - 1; j++) {
      if (values[j] > values[j + 1]) {
        uint16_t temp = values[j];
        values[j] = values[j + 1];
        values[j + 1] = temp;
      }
    }
  }
  
  // Sum middle values after trimming min and max
  uint32_t sum = 0;
  uint16_t validCount = count - 2 * trimCount;
  for (uint8_t i = trimCount; i < count - trimCount; i++) {
    sum += values[i];
  }
  
  return (uint16_t)(sum / validCount);
}
