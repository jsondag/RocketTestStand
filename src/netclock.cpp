#include "globals.h"
#include <time.h>

void applyTimezone() {
  int idx = settings.timezoneIndex;
  if (idx < 0 || idx >= kTimezoneCount) idx = TZ_DEFAULT_INDEX;
  setenv("TZ", kTimezones[idx].posix, 1);
  tzset();
}

void scanWifiNetworks() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  wifiCount = WiFi.scanNetworks();
  if (wifiCount > 16) wifiCount = 16;
  for (int i = 0; i < wifiCount; i++) {
    wifiNetworks[i].ssid = WiFi.SSID(i);
    wifiNetworks[i].rssi = WiFi.RSSI(i);
    wifiNetworks[i].secure = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
  }
}

bool connectWiFi(const String& ssid, const String& password) {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.begin(ssid.c_str(), password.c_str());
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(100);
  }
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }
  if (ssid.length() > 0) {
    ssid.toCharArray(settings.wifiSsid, sizeof(settings.wifiSsid));
  }
  if (password.length() > 0) {
    password.toCharArray(settings.wifiPassword, sizeof(settings.wifiPassword));
  }
  saveSettings();
  wifiAutoState = WIFI_AUTO_DONE;
  return true;
}

bool syncTimeWithNTP() {
  configTzTime(kTimezones[settings.timezoneIndex].posix, NTP_SERVER);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 10000)) {
    return false;
  }
  if (timeinfo.tm_year + 1900 < 2024) {
    return false;
  }
  timeAvailable = true;
  return true;
}

bool formatLocalTime(char* buf, size_t bufLen, const char* fmt) {
  if (!timeAvailable) return false;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 10)) return false;
  if (timeinfo.tm_year + 1900 < 2024) return false;
  strftime(buf, bufLen, fmt, &timeinfo);
  return true;
}
