#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <HX711.h>
#include <RTClib.h>
#include <EEPROM.h>

// --- Hardware pin configuration ---
#define HX711_DOUT 26
#define HX711_SCK 25

#define TFT_CS 15
#define TFT_DC 2
#define TFT_RST 4

#define SD_CS 5

#define TOUCH_CS 16
#define TOUCH_IRQ 17

#define I2C_SDA 21
#define I2C_SCL 22

#define NTP_SERVER "pool.ntp.org"
#define SETTINGS_FILENAME "/settings.ini"
#define EEPROM_MAGIC 0x52415350
#define EEPROM_SIZE 1024

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
HX711 scale;
RTC_DS3231 rtc;

struct AppSettings {
  char wifiSsid[32];
  char wifiPassword[64];
  float calibrationFactor;
  bool rtcEnabled;
  float startThreshold;
  uint16_t preCaptureMs;
  uint16_t postCaptureMs;
  bool waitForEjection;
};

struct EepromSettings {
  uint32_t magic;
  AppSettings settings;
};

struct WifiNetwork {
  String ssid;
  int rssi;
  bool secure;
};

struct Sample {
  unsigned long timeMs;
  float forceN;
};

struct TouchPoint {
  int16_t x;
  int16_t y;
  bool valid;
};

enum ScreenMode {
  SCREEN_HOME,
  SCREEN_SCALE,
  SCREEN_CAPTURE,
  SCREEN_LOAD_FILE,
  SCREEN_REVIEW,
  SCREEN_SETTINGS,
  SCREEN_WIFI_SCAN,
  SCREEN_WIFI_PASSWORD,
  SCREEN_CALIBRATION,
  SCREEN_RTC
};

AppSettings settings;
bool sdPresent = false;
bool rtcAvailable = false;
ScreenMode currentScreen = SCREEN_HOME;

Sample samples[1200];
int sampleCount = 0;
float offsetTare = 0.0f;

WifiNetwork wifiNetworks[16];
int wifiCount = 0;
int wifiScroll = 0;
int selectedWifi = -1;

String passwordEntry;
String weightEntry;
String statusLine;

String loadedFileName;
String savedFileList[16];
int savedFileCount = 0;
int savedFileScroll = 0;

bool touchActive = false;

void touchBegin() {
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);
  pinMode(TOUCH_IRQ, INPUT_PULLUP);
}

uint16_t xptRead(uint8_t command) {
  command &= 0xF0;
  SPI.transfer(command);
  uint16_t hi = SPI.transfer(0x00);
  uint16_t lo = SPI.transfer(0x00);
  return ((hi << 8) | lo) >> 3;
}

TouchPoint getTouchPoint() {
  TouchPoint point = {0, 0, false};
  if (digitalRead(TOUCH_IRQ) != LOW) {
    return point;
  }

  digitalWrite(TOUCH_CS, LOW);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  delayMicroseconds(10);
  uint16_t rawx = xptRead(0xD0);
  uint16_t rawy = xptRead(0x90);
  SPI.endTransaction();
  digitalWrite(TOUCH_CS, HIGH);

  if (rawx < 200 || rawx > 3800 || rawy < 200 || rawy > 3800) {
    return point;
  }

  int x = map(rawx, 3800, 200, 0, tft.width() - 1);
  int y = map(rawy, 200, 3800, 0, tft.height() - 1);
  x = constrain(x, 0, tft.width() - 1);
  y = constrain(y, 0, tft.height() - 1);
  point.x = x;
  point.y = y;
  point.valid = true;
  return point;
}

bool touchPressed() {
  return digitalRead(TOUCH_IRQ) == LOW;
}

bool getTouchedPoint(TouchPoint &p) {
  if (!touchPressed()) {
    touchActive = false;
    return false;
  }
  if (touchActive) {
    return false;
  }
  touchActive = true;
  delay(40);
  p = getTouchPoint();
  while (touchPressed()) {
    delay(20);
  }
  touchActive = false;
  return p.valid;
}

bool hitTest(const TouchPoint &p, int x, int y, int w, int h) {
  return p.valid && p.x >= x && p.x < x + w && p.y >= y && p.y < y + h;
}

void drawHeader(const char* title) {
  tft.fillScreen(ILI9341_BLACK);
  tft.fillRect(0, 0, tft.width(), 24, ILI9341_DARKGREY);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(30, 4);
  tft.print(title);
  tft.drawFastHLine(0, 24, tft.width(), ILI9341_GREEN);
}

void drawHamburger() {
  tft.fillRect(0, 0, 28, 24, ILI9341_DARKGREY);
  tft.fillRect(6, 6, 16, 3, ILI9341_WHITE);
  tft.fillRect(6, 11, 16, 3, ILI9341_WHITE);
  tft.fillRect(6, 16, 16, 3, ILI9341_WHITE);
}

void drawFooter(const char* text) {
  tft.fillRect(0, 216, tft.width(), 24, ILI9341_BLACK);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(1);
  tft.setCursor(4, 220);
  tft.print(text);
}

void drawButton(int x, int y, int w, int h, const String& label) {
  tft.fillRoundRect(x, y, w, h, 6, ILI9341_NAVY);
  tft.drawRoundRect(x, y, w, h, 6, ILI9341_WHITE);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  int16_t tx = x + 8;
  int16_t ty = y + (h / 2) - 8;
  tft.setCursor(tx, ty);
  tft.print(label);
}

void drawSmallButton(int x, int y, int w, int h, const String& label, uint16_t color = ILI9341_BLUE) {
  tft.fillRoundRect(x, y, w, h, 6, color);
  tft.drawRoundRect(x, y, w, h, 6, ILI9341_WHITE);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  int16_t tx = x + 6;
  int16_t ty = y + (h / 2) - 8;
  tft.setCursor(tx, ty);
  tft.print(label);
}

void drawTextBlock(int x, int y, int w, int h, const String& text) {
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(x, y);
  tft.print(text);
}

void resetDefaultSettings() {
  strcpy(settings.wifiSsid, "");
  strcpy(settings.wifiPassword, "");
  settings.calibrationFactor = -7050.0f;
  settings.rtcEnabled = true;
  settings.startThreshold = 0.5f;
  settings.preCaptureMs = 500;
  settings.postCaptureMs = 1200;
  settings.waitForEjection = true;
}

bool saveSettingsToEEPROM() {
  EepromSettings eepromData;
  eepromData.magic = EEPROM_MAGIC;
  eepromData.settings = settings;
  EEPROM.put(0, eepromData);
  return EEPROM.commit();
}

bool saveSettingsToSD() {
  if (!sdPresent) return false;
  SD.remove(SETTINGS_FILENAME);
  File file = SD.open(SETTINGS_FILENAME, FILE_WRITE);
  if (!file) return false;
  file.printf("ssid=%s\n", settings.wifiSsid);
  file.printf("password=%s\n", settings.wifiPassword);
  file.printf("calibrationFactor=%.6f\n", settings.calibrationFactor);
  file.printf("rtcEnabled=%d\n", settings.rtcEnabled ? 1 : 0);
  file.printf("startThreshold=%.3f\n", settings.startThreshold);
  file.printf("preCaptureMs=%u\n", settings.preCaptureMs);
  file.printf("postCaptureMs=%u\n", settings.postCaptureMs);
  file.printf("waitForEjection=%d\n", settings.waitForEjection ? 1 : 0);
  file.close();
  return true;
}

bool loadSettingsFromEEPROM() {
  EepromSettings eepromData;
  EEPROM.get(0, eepromData);
  if (eepromData.magic != EEPROM_MAGIC) {
    return false;
  }
  settings = eepromData.settings;
  return true;
}

bool loadSettingsFromSD() {
  if (!sdPresent) return false;
  File file = SD.open(SETTINGS_FILENAME);
  if (!file) return false;

  resetDefaultSettings();

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line.startsWith("#")) continue;
    int idx = line.indexOf('=');
    if (idx < 0) continue;
    String key = line.substring(0, idx);
    String value = line.substring(idx + 1);
    if (key == "ssid") {
      value.toCharArray(settings.wifiSsid, sizeof(settings.wifiSsid));
    } else if (key == "password") {
      value.toCharArray(settings.wifiPassword, sizeof(settings.wifiPassword));
    } else if (key == "calibrationFactor") {
      settings.calibrationFactor = value.toFloat();
    } else if (key == "rtcEnabled") {
      settings.rtcEnabled = value.toInt() != 0;
    } else if (key == "startThreshold") {
      settings.startThreshold = value.toFloat();
    } else if (key == "preCaptureMs") {
      settings.preCaptureMs = value.toInt();
    } else if (key == "postCaptureMs") {
      settings.postCaptureMs = value.toInt();
    } else if (key == "waitForEjection") {
      settings.waitForEjection = value.toInt() != 0;
    }
  }
  file.close();
  return true;
}

bool saveSettings() {
  bool ok = false;
  if (sdPresent) {
    ok = saveSettingsToSD();
    saveSettingsToEEPROM();
    return ok;
  }
  return saveSettingsToEEPROM();
}

bool loadSettings() {
  bool loaded = false;
  if (sdPresent) {
    loaded = loadSettingsFromSD();
  }
  if (!loaded) {
    loaded = loadSettingsFromEEPROM();
  }
  if (!loaded) {
    resetDefaultSettings();
  }
  return true;
}

void applyScaleSettings() {
  scale.set_scale(settings.calibrationFactor);
}

void drawHomeScreen() {
  drawHeader("Rocket Stand");
  drawHamburger();
  drawButton(16, 40, 140, 80, "Scale");
  drawButton(164, 40, 140, 80, "Capture");
  drawButton(16, 136, 140, 80, "Browse");
  drawButton(164, 136, 140, 80, "Settings");
  drawFooter("Tap a button or open menu.");
}

void drawScaleScreen() {
  drawHeader("Scale");
  drawHamburger();
  float raw = scale.get_units(5) - offsetTare;
  float forceN = raw * 9.80665f;
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(24, 60);
  tft.print(forceN, 2);
  tft.print(" N");
  drawSmallButton(16, 180, 140, 44, "Tare");
  drawSmallButton(164, 180, 140, 44, "Home", ILI9341_RED);
  drawFooter("Tap Tare or Home.");
}

void drawCaptureScreen() {
  drawHeader("Capture");
  drawHamburger();
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(16, 50);
  tft.print("Start threshold: ");
  tft.setCursor(16, 80);
  tft.print(settings.startThreshold, 2);
  tft.print(" N");
  tft.setCursor(16, 110);
  tft.print("Post hold: ");
  tft.print(settings.postCaptureMs);
  tft.print(" ms");
  drawButton(16, 150, 140, 64, "Start");
  drawSmallButton(164, 180, 140, 44, "Home", ILI9341_RED);
  drawFooter("Tare before capture for best results.");
}

void drawReviewScreen() {
  drawHeader("Load Review");
  drawHamburger();
  if (sampleCount <= 0) {
    drawTextBlock(16, 60, 280, 140, "No file loaded.");
  } else {
    tft.drawRect(8, 40, tft.width() - 16, 150, ILI9341_WHITE);
    float maxForce = 0.001f;
    for (int i = 0; i < sampleCount; i++) {
      maxForce = max(maxForce, samples[i].forceN);
    }
    int left = 10;
    int top = 42;
    int graphW = tft.width() - 20;
    int graphH = 146;
    unsigned long duration = samples[sampleCount - 1].timeMs;
    float xScale = duration > 0 ? (float)graphW / duration : 1.0f;
    float yScale = graphH / max(5.0f, maxForce * 1.1f);
    for (int i = 1; i < sampleCount; i++) {
      int x1 = left + (int)(samples[i - 1].timeMs * xScale);
      int y1 = top + graphH - (int)(samples[i - 1].forceN * yScale);
      int x2 = left + (int)(samples[i].timeMs * xScale);
      int y2 = top + graphH - (int)(samples[i].forceN * yScale);
      tft.drawLine(x1, y1, x2, y2, ILI9341_YELLOW);
    }
    tft.setTextSize(1);
    tft.setCursor(16, 196);
    tft.setTextColor(ILI9341_CYAN);
    tft.print("File: ");
    tft.print(loadedFileName);
  }
  drawSmallButton(164, 180, 140, 44, "Home", ILI9341_RED);
  drawFooter("Review loaded thrust curve.");
}

void drawSettingsMenu() {
  drawHeader("Settings");
  drawHamburger();
  drawButton(16, 40, 140, 64, "WiFi");
  drawButton(164, 40, 140, 64, "Calibrate");
  drawButton(16, 112, 140, 64, "RTC");
  drawButton(164, 112, 140, 64, "Back");
  drawFooter("Tap a settings page.");
}

void drawWifiScanScreen() {
  drawHeader("WiFi Scan");
  drawHamburger();
  for (int i = 0; i < 4; i++) {
    int idx = wifiScroll + i;
    int y = 40 + i * 44;
    if (idx >= wifiCount) break;
    String label = wifiNetworks[idx].ssid;
    if (wifiNetworks[idx].secure) label += " *";
    drawSmallButton(16, y, 288, 38, label, ILI9341_DARKGREY);
  }
  if (wifiCount == 0) {
    drawTextBlock(16, 60, 280, 100, "No networks found.");
  }
  drawSmallButton(16, 220, 140, 20, "Rescan");
  drawSmallButton(164, 220, 140, 20, "Home", ILI9341_RED);
  drawFooter("Tap SSID to connect.");
}

void drawRtcScreen() {
  drawHeader("RTC Settings");
  drawHamburger();
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(16, 50);
  if (rtcAvailable) {
    tft.print("RTC detected.");
  } else {
    tft.print("RTC not found.");
  }
  tft.setCursor(16, 90);
  tft.print("Enabled: ");
  tft.print(settings.rtcEnabled ? "Yes" : "No");
  drawSmallButton(16, 140, 140, 44, settings.rtcEnabled ? "Disable" : "Enable");
  drawSmallButton(164, 140, 140, 44, "Sync NTP");
  drawSmallButton(16, 190, 140, 44, "Home", ILI9341_RED);
  drawFooter("Enable RTC if installed.");
}

void drawCalibrationScreen(float measuredMass) {
  drawHeader("Calibration");
  drawHamburger();
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(16, 40);
  tft.print("Factor: ");
  tft.print(settings.calibrationFactor, 0);
  tft.setCursor(16, 70);
  tft.print("Mass: ");
  tft.print(measuredMass, 2);
  tft.print(" g");
  tft.setCursor(16, 100);
  tft.print("Target: ");
  tft.print(weightEntry);
  tft.print(" g");
  drawSmallButton(16, 140, 140, 44, "Tare");
  drawSmallButton(164, 140, 140, 44, "Measure");
  drawSmallButton(16, 190, 140, 44, "Weight");
  drawSmallButton(164, 190, 140, 44, "Calibrate");
  drawSmallButton(16, 240 - 44, 140, 44, "Home", ILI9341_RED);
  drawFooter("Tare, measure, enter known weight, calibrate.");
}

void drawLoadFilesScreen() {
  drawHeader("File Browser");
  drawHamburger();
  for (int i = 0; i < 4; i++) {
    int idx = savedFileScroll + i;
    int y = 40 + i * 44;
    if (idx >= savedFileCount) break;
    drawSmallButton(16, y, 288, 38, savedFileList[idx], ILI9341_DARKGREY);
  }
  if (savedFileCount == 0) {
    drawTextBlock(16, 60, 280, 100, "No .eng files found.");
  }
  int totalPages = max(1, (savedFileCount + 3) / 4);
  int currentPage = (savedFileScroll / 4) + 1;
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(1);
  tft.setCursor(16, 200);
  tft.print("Page ");
  tft.print(currentPage);
  tft.print("/");
  tft.print(totalPages);
  drawSmallButton(16, 220, 68, 20, "Up");
  drawSmallButton(90, 220, 68, 20, "Down");
  drawSmallButton(164, 220, 68, 20, "Refresh");
  drawSmallButton(238, 220, 68, 20, "Home", ILI9341_RED);
  drawFooter("Tap a file to open it.");
}

void drawMessage(const String& message) {
  tft.fillRect(0, 216, tft.width(), 24, ILI9341_BLACK);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(1);
  tft.setCursor(4, 220);
  tft.print(message);
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

void refreshSavedFileList() {
  savedFileCount = 0;
  savedFileScroll = 0;
  if (!sdPresent) return;
  File root = SD.open("/");
  if (!root) return;
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    if (!entry.isDirectory()) {
      String name = entry.name();
      if (name.endsWith(".eng") && savedFileCount < 16) {
        savedFileList[savedFileCount++] = name;
      }
    }
    entry.close();
  }
  root.close();
}

void loadEngFile(const String& filename) {
  File file = SD.open(filename);
  sampleCount = 0;
  if (!file) return;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line.startsWith("#") || line.startsWith("data_ms")) continue;
    int comma = line.indexOf(',');
    if (comma > 0) {
      if (sampleCount < 1200) {
        samples[sampleCount].timeMs = line.substring(0, comma).toInt();
        samples[sampleCount].forceN = line.substring(comma + 1).toFloat();
        sampleCount++;
      }
    }
  }
  file.close();
}

void drawKeyboard(const String& title, const String& value, bool password, const char* const keys[], int rows, bool showShift, bool shiftActive) {
  drawHeader(title.c_str());
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(16, 32);
  tft.print(password ? String(value.length()) : value);
  int rowHeight = 34;
  int top = 70;
  for (int r = 0; r < rows; r++) {
    const char* row = keys[r];
    int cols = strlen(row);
    int cellW = (tft.width() - 32) / cols;
    for (int c = 0; c < cols; c++) {
      int x = 16 + c * cellW;
      int y = top + r * rowHeight;
      String label(1, row[c]);
      drawSmallButton(x, y, cellW - 4, rowHeight - 6, label, ILI9341_DARKGREY);
    }
  }
  drawSmallButton(16, top + rows * rowHeight, 140, 40, showShift ? (shiftActive ? "SHIFT" : "shift") : "DEL");
  drawSmallButton(166, top + rows * rowHeight, 140, 40, "OK", ILI9341_GREEN);
  drawSmallButton(16, top + rows * rowHeight + 46, 140, 40, "Back", ILI9341_RED);
}

String inputTextKeyboard(const String& title, const String& initial, bool passwordMode, bool allowDot) {
  String value = initial;
  bool shiftActive = false;
  const char* alphaKeys[] = {"QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM", "1234567890", "@.-_/"};
  const char* numberKeys[] = {"1234567", "89.0"};
  while (true) {
    if (passwordMode) {
      drawKeyboard(title, value, true, alphaKeys, 5, true, shiftActive);
    } else {
      drawKeyboard(title, value, false, numberKeys, 2, false, false);
    }
    TouchPoint p;
    if (!getTouchedPoint(p)) {
      delay(20);
      continue;
    }
    if (hitTest(p, 16, 70 + 5 * 34, 140, 40)) {
      if (passwordMode) {
        shiftActive = !shiftActive;
      } else {
        if (!value.isEmpty()) value.remove(value.length() - 1);
      }
    }
    if (hitTest(p, 166, 70 + 5 * 34, 140, 40)) {
      return value;
    }
    if (hitTest(p, 16, 70 + 5 * 34 + 46, 140, 40)) {
      return initial;
    }
    int rows = passwordMode ? 5 : 2;
    for (int r = 0; r < rows; r++) {
      const char* row = passwordMode ? alphaKeys[r] : numberKeys[r];
      int cols = strlen(row);
      int cellW = (tft.width() - 32) / cols;
      for (int c = 0; c < cols; c++) {
        int x = 16 + c * cellW;
        int y = 70 + r * 34;
        if (hitTest(p, x, y, cellW - 4, 28)) {
          char ch = row[c];
          if (passwordMode && isalpha(ch)) {
            if (!shiftActive) ch = tolower(ch);
          }
          if (!passwordMode && ch == '.') {
            if (!value.contains('.')) value += '.';
          } else {
            value += ch;
          }
          break;
        }
      }
    }
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
  return true;
}

bool syncTimeWithNTP() {
  configTime(0, 0, NTP_SERVER);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 10000)) {
    return false;
  }
  if (settings.rtcEnabled && rtcAvailable) {
    rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
  }
  return true;
}

void saveAndShowStatus(const String& message) {
  saveSettings();
  statusLine = message;
  drawMessage(message);
}

void drawSettingsScreen() {
  drawSettingsMenu();
}

void resetTouchBuffer() {
  touchActive = false;
}

void setup() {
  Serial.begin(115200);
  SPI.begin(18, 19, 23, SD_CS);
  tft.begin();
  tft.setRotation(1);
  tft.setTextWrap(true);
  touchBegin();

  sdPresent = SD.begin(SD_CS);
  if (!sdPresent) {
    Serial.println("SD init failed, using EEPROM fallback.");
  }
  EEPROM.begin(EEPROM_SIZE);
  loadSettings();
  applyScaleSettings();

  Wire.begin(I2C_SDA, I2C_SCL);
  rtcAvailable = rtc.begin();

  scale.begin(HX711_DOUT, HX711_SCK);
  scale.tare();
  offsetTare = scale.get_units(10);

  drawHomeScreen();
}

void runCaptureRoutine() {
  drawHeader("Capturing...");
  tft.drawRect(8, 32, tft.width() - 16, 160, ILI9341_WHITE);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(16, 208);
  tft.print("Waiting for thrust...");

  sampleCount = 0;
  unsigned long startTime = millis();
  float baseline = 0.0f;
  int baselineCount = 0;
  while (millis() - startTime < settings.preCaptureMs) {
    float f = (scale.get_units(5) - offsetTare) * 9.80665f;
    baseline += f;
    baselineCount++;
    delay(20);
  }
  baseline /= max(1, baselineCount);
  float threshold = baseline + settings.startThreshold;
  float endThreshold = baseline + settings.startThreshold * 0.5f;

  bool capturing = false;
  unsigned long captureStart = 0;
  unsigned long lastAbove = 0;
  unsigned long lastSample = millis();
  float maxForce = 0.0f;
  float totalImpulse = 0.0f;
  bool foundEject = false;
  unsigned long ejectTime = 0;

  while (true) {
    unsigned long now = millis();
    if (now - lastSample < 20) continue;
    lastSample = now;
    float f = (scale.get_units(5) - offsetTare) * 9.80665f;
    if (!capturing) {
      if (f >= threshold) {
        capturing = true;
        captureStart = now;
        lastAbove = now;
        sampleCount = 0;
      }
    }
    if (capturing) {
      if (sampleCount < 1200) {
        samples[sampleCount++] = {now - captureStart, f};
      }
      if (f > maxForce) maxForce = f;
      if (f > endThreshold) {
        lastAbove = now;
      }
      totalImpulse += f * 0.02f;
      if (settings.waitForEjection && !foundEject && now - lastAbove > 1000 && f > threshold * 1.5f) {
        foundEject = true;
        ejectTime = now;
      }
      if (now - lastAbove >= settings.postCaptureMs) {
        break;
      }
      if (sampleCount % 10 == 0) {
        int left = 10;
        int top = 34;
        int graphW = tft.width() - 20;
        int graphH = 156;
        float maxF = 0.001f;
        for (int i = 0; i < sampleCount; i++) {
          maxF = max(maxF, samples[i].forceN);
        }
        float xScale = (float)graphW / max(1.0f, (float)samples[sampleCount - 1].timeMs);
        float yScale = graphH / max(5.0f, maxF * 1.1f);
        tft.fillRect(left, top, graphW, graphH, ILI9341_BLACK);
        for (int i = 1; i < sampleCount; i++) {
          int x1 = left + (int)(samples[i - 1].timeMs * xScale);
          int y1 = top + graphH - (int)(samples[i - 1].forceN * yScale);
          int x2 = left + (int)(samples[i].timeMs * xScale);
          int y2 = top + graphH - (int)(samples[i].forceN * yScale);
          tft.drawLine(x1, y1, x2, y2, ILI9341_YELLOW);
        }
      }
    }
  }

  float burnTime = (lastAbove - captureStart) / 1000.0f;
  float delaySecs = 0.0f;
  if (settings.waitForEjection && foundEject) {
    delaySecs = (ejectTime - lastAbove) / 1000.0f;
  }
  String rating = "?";
  const struct { const char* letter; float low; float high; } ratingTable[] = {
    {"A", 1.26f, 2.50f}, {"B", 2.51f, 5.00f}, {"C", 5.01f, 10.00f}, {"D", 10.01f, 20.00f},
    {"E", 20.01f, 40.00f}, {"F", 40.01f, 80.00f}, {"G", 80.01f, 160.00f}, {"H", 160.01f, 320.00f},
    {"J", 320.01f, 640.00f}, {"K", 640.01f, 1280.00f}
  };
  for (auto &entry : ratingTable) {
    if (totalImpulse >= entry.low && totalImpulse <= entry.high) {
      rating = String(entry.letter);
      break;
    }
  }
  if (delaySecs > 0.0f) {
    int delayVal = constrain((int)round(delaySecs), 1, 99);
    rating += String(delayVal);
  }
  String filename = "/RASP_" + String(millis()) + ".eng";
  if (rtcAvailable) {
    DateTime now = rtc.now();
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    filename = String("/RASP_") + buf + ".eng";
  }
  File file = SD.open(filename, FILE_WRITE);
  if (file) {
    if (rtcAvailable) {
      DateTime now = rtc.now();
      file.printf("timestamp=%04u-%02u-%02u %02u:%02u:%02u\n", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    } else {
      file.printf("timestamp=%lu\n", millis() / 1000);
    }
    file.printf("total_impulse_Ns=%.3f\n", totalImpulse);
    file.printf("burn_time_s=%.3f\n", burnTime);
    file.printf("max_force_N=%.3f\n", maxForce);
    file.printf("delay_s=%.3f\n", delaySecs);
    file.printf("rating=%s\n", rating.c_str());
    file.printf("data_ms,force_N\n");
    for (int i = 0; i < sampleCount; i++) {
      file.printf("%lu,%.4f\n", samples[i].timeMs, samples[i].forceN);
    }
    file.close();
    statusLine = "Saved " + filename + "";
  } else {
    statusLine = "Save failed";
  }
  drawMessage(statusLine);
  delay(1500);
}

void loop() {
  TouchPoint touch;
  if (getTouchedPoint(touch)) {
    if (hitTest(touch, 0, 0, 28, 24)) {
      currentScreen = SCREEN_SETTINGS;
      drawSettingsScreen();
      return;
    }
    switch (currentScreen) {
      case SCREEN_HOME:
        if (hitTest(touch, 16, 40, 140, 80)) {
          currentScreen = SCREEN_SCALE;
          drawScaleScreen();
        } else if (hitTest(touch, 164, 40, 140, 80)) {
          currentScreen = SCREEN_CAPTURE;
          drawCaptureScreen();
        } else if (hitTest(touch, 16, 136, 140, 80)) {
          currentScreen = SCREEN_LOAD_FILE;
          refreshSavedFileList();
          drawLoadFilesScreen();
        } else if (hitTest(touch, 164, 136, 140, 80)) {
          currentScreen = SCREEN_SETTINGS;
          drawSettingsScreen();
        }
        break;
      case SCREEN_SCALE:
        if (hitTest(touch, 16, 180, 140, 44)) {
          scale.tare();
          offsetTare = scale.get_units(10);
          drawMessage("Tared.");
          drawScaleScreen();
        } else if (hitTest(touch, 164, 180, 140, 44)) {
          currentScreen = SCREEN_HOME;
          drawHomeScreen();
        }
        break;
      case SCREEN_CAPTURE:
        if (hitTest(touch, 16, 150, 140, 64)) {
          runCaptureRoutine();
          currentScreen = SCREEN_HOME;
          drawHomeScreen();
        } else if (hitTest(touch, 164, 180, 140, 44)) {
          currentScreen = SCREEN_HOME;
          drawHomeScreen();
        }
        break;
      case SCREEN_LOAD_FILE:
        if (hitTest(touch, 16, 220, 68, 20)) {
          if (savedFileScroll >= 4) {
            savedFileScroll -= 4;
          }
          drawLoadFilesScreen();
        } else if (hitTest(touch, 90, 220, 68, 20)) {
          if (savedFileScroll + 4 < savedFileCount) {
            savedFileScroll += 4;
          }
          drawLoadFilesScreen();
        } else if (hitTest(touch, 164, 220, 68, 20)) {
          refreshSavedFileList();
          drawLoadFilesScreen();
        } else if (hitTest(touch, 238, 220, 68, 20)) {
          currentScreen = SCREEN_HOME;
          drawHomeScreen();
        } else {
          for (int i = 0; i < 4; i++) {
            int idx = savedFileScroll + i;
            int y = 40 + i * 44;
            if (idx < savedFileCount && hitTest(touch, 16, y, 288, 38)) {
              loadedFileName = savedFileList[idx];
              loadEngFile(loadedFileName);
              currentScreen = SCREEN_REVIEW;
              drawReviewScreen();
              break;
            }
          }
        }
        break;
      case SCREEN_REVIEW:
        if (hitTest(touch, 164, 180, 140, 44)) {
          currentScreen = SCREEN_HOME;
          drawHomeScreen();
        }
        break;
      case SCREEN_SETTINGS:
        if (hitTest(touch, 16, 40, 140, 64)) {
          currentScreen = SCREEN_WIFI_SCAN;
          scanWifiNetworks();
          drawWifiScanScreen();
        } else if (hitTest(touch, 164, 40, 140, 64)) {
          currentScreen = SCREEN_CALIBRATION;
          weightEntry = "";
          drawCalibrationScreen(0.0f);
        } else if (hitTest(touch, 16, 112, 140, 64)) {
          currentScreen = SCREEN_RTC;
          drawRtcScreen();
        } else if (hitTest(touch, 164, 112, 140, 64)) {
          currentScreen = SCREEN_HOME;
          drawHomeScreen();
        }
        break;
      case SCREEN_WIFI_SCAN:
        if (hitTest(touch, 16, 220, 140, 20)) {
          scanWifiNetworks();
          drawWifiScanScreen();
        } else if (hitTest(touch, 164, 220, 140, 20)) {
          currentScreen = SCREEN_HOME;
          drawHomeScreen();
        } else {
          for (int i = 0; i < 4; i++) {
            int idx = wifiScroll + i;
            int y = 40 + i * 44;
            if (idx < wifiCount && hitTest(touch, 16, y, 288, 38)) {
              selectedWifi = idx;
              String ssid = wifiNetworks[idx].ssid;
              passwordEntry = "";
              String prompt = "Password for " + ssid;
              passwordEntry = inputTextKeyboard(prompt, passwordEntry, true, false);
              if (connectWiFi(ssid, passwordEntry)) {
                bool synced = syncTimeWithNTP();
                drawMessage(synced ? "WiFi connected, NTP synced." : "WiFi connected, NTP failed.");
              } else {
                drawMessage("WiFi connect failed.");
              }
              currentScreen = SCREEN_SETTINGS;
              drawSettingsScreen();
              break;
            }
          }
        }
        break;
      case SCREEN_CALIBRATION:
        if (hitTest(touch, 16, 140, 140, 44)) {
          scale.tare();
          offsetTare = scale.get_units(10);
          drawMessage("Tare complete.");
          drawCalibrationScreen(0.0f);
        } else if (hitTest(touch, 164, 140, 140, 44)) {
          float measured = (scale.get_units(10) - offsetTare) * 1000.0f;
          drawMessage("Measured " + String(measured, 2) + " g");
          drawCalibrationScreen(measured);
        } else if (hitTest(touch, 16, 190, 140, 44)) {
          weightEntry = inputTextKeyboard("Enter weight (g)", weightEntry, false, true);
          drawCalibrationScreen(0.0f);
        } else if (hitTest(touch, 164, 190, 140, 44)) {
          float target = weightEntry.toFloat();
          if (target > 0.0f) {
            float objectUnits = scale.get_units(10) - offsetTare;
            float kg = target / 1000.0f;
            if (kg > 0.0f) {
              settings.calibrationFactor = objectUnits / kg;
              applyScaleSettings();
              saveSettings();
              drawMessage("Calibration saved.");
            }
          } else {
            drawMessage("Enter a valid weight.");
          }
          drawCalibrationScreen(0.0f);
        } else if (hitTest(touch, 16, 240 - 44, 140, 44)) {
          currentScreen = SCREEN_HOME;
          drawHomeScreen();
        }
        break;
      case SCREEN_RTC:
        if (hitTest(touch, 16, 140, 140, 44)) {
          if (rtcAvailable) {
            settings.rtcEnabled = !settings.rtcEnabled;
            saveSettings();
            drawRtcScreen();
          } else {
            drawMessage("RTC not available.");
          }
        } else if (hitTest(touch, 164, 140, 140, 44)) {
          if (WiFi.status() == WL_CONNECTED) {
            bool synced = syncTimeWithNTP();
            drawMessage(synced ? "NTP sync complete." : "NTP sync failed.");
          } else {
            drawMessage("Not connected to WiFi.");
          }
        } else if (hitTest(touch, 16, 190, 140, 44)) {
          currentScreen = SCREEN_HOME;
          drawHomeScreen();
        }
        break;
    }
  }
  delay(30);
}
