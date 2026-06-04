#include "globals.h"
#include <time.h>
#include <WebServer.h>

static WebServer httpServer(80);

static String urlEncode(const String& input) {
  String output;
  for (size_t i = 0; i < input.length(); i++) {
    char c = input[i];
    if (('0' <= c && c <= '9') || ('A' <= c && c <= 'Z') || ('a' <= c && c <= 'z') || c == '-' || c == '_' || c == '.' || c == '~') {
      output += c;
    } else if (c == ' ') {
      output += '+';
    } else {
      char buf[4];
      snprintf(buf, sizeof(buf), "%%%02X", (uint8_t)c);
      output += buf;
    }
  }
  return output;
}

static void handleWebRoot() {
  String html = "<html><head><meta charset='utf-8'><title>RocketTestStand</title>"
                "<style>body{font-family:sans-serif;}a{color:#1E90FF;}ul{padding-left:16px;}</style></head><body>";
  html += "<h2>Rocket Test Stand SD</h2>";
  html += "<p>IP: " + String(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "Not connected") + "</p>";
  if (!SD.exists("/")) {
    html += "<p><strong>SD card unavailable.</strong></p>";
  } else {
    html += "<ul>";
    File root = SD.open("/");
    if (root) {
      File entry = root.openNextFile();
      while (entry) {
        if (!entry.isDirectory()) {
          String name = entry.name();
          if (name.startsWith("/")) name = name.substring(1);
          html += "<li><a href=\"/download?file=" + urlEncode(name) + "\">" + name + "</a></li>";
        }
        entry.close();
        entry = root.openNextFile();
      }
      root.close();
    }
    html += "</ul>";
  }
  html += "</body></html>";
  httpServer.send(200, "text/html", html);
}

static void handleWebDownload() {
  String fileName = httpServer.arg("file");
  if (fileName.length() == 0) {
    httpServer.send(400, "text/plain", "Missing file parameter");
    return;
  }
  if (fileName.indexOf("..") >= 0) {
    httpServer.send(400, "text/plain", "Invalid file path");
    return;
  }
  String filePath = fileName;
  if (!filePath.startsWith("/")) {
    filePath = "/" + filePath;
  }
  if (!SD.exists(filePath)) {
    httpServer.send(404, "text/plain", "File not found");
    return;
  }
  File file = SD.open(filePath, FILE_READ);
  if (!file) {
    httpServer.send(500, "text/plain", "Unable to open file");
    return;
  }
  httpServer.sendHeader("Content-Disposition", "attachment; filename=\"" + fileName + "\"");
  httpServer.streamFile(file, "application/octet-stream");
  file.close();
}

void beginWebServer() {
  httpServer.on("/", HTTP_GET, handleWebRoot);
  httpServer.on("/download", HTTP_GET, handleWebDownload);
  httpServer.onNotFound([]() {
    httpServer.send(404, "text/plain", "Not found");
  });
  httpServer.begin();
}

void handleWebServer() {
  httpServer.handleClient();
}

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
