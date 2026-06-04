// Rocket Test Stand firmware
// 2026 Jeff Sondag
// AGPLv3 license

#include "globals.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// --- Hardware singletons ---
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
HX711 scale;
RTC_DS3231 rtc;
SPIClass spiSD(HSPI);

// --- Timezone table ---
const TzEntry kTimezones[] = {
  {"Baker (UTC-12)",        "BAK12"},
  {"Pago Pago",             "SST11"},
  {"Hawaii",                "HST10"},
  {"Alaska",                "AKST9AKDT,M3.2.0,M11.1.0"},
  {"US Pacific",            "PST8PDT,M3.2.0,M11.1.0"},
  {"US Mountain",           "MST7MDT,M3.2.0,M11.1.0"},
  {"Arizona",               "MST7"},
  {"US Central",            "CST6CDT,M3.2.0,M11.1.0"},
  {"Mexico City",           "CST6CDT,M4.1.0,M10.5.0"},
  {"US Eastern",            "EST5EDT,M3.2.0,M11.1.0"},
  {"Bogota / Lima",         "COT5"},
  {"Atlantic / Halifax",    "AST4ADT,M3.2.0,M11.1.0"},
  {"Caracas",               "VET4"},
  {"Newfoundland",          "NST3:30NDT,M3.2.0,M11.1.0"},
  {"Sao Paulo",             "BRT3"},
  {"Buenos Aires",          "ART3"},
  {"Rio de Janeiro",        "BRT3"},
  {"South Georgia",         "GST2"},
  {"Azores",                "AZOT1AZOST,M3.5.0/0,M10.5.0/1"},
  {"UTC",                   "UTC0"},
  {"London / Dublin",       "GMT0BST,M3.5.0/1,M10.5.0"},
  {"Lisbon",                "WET0WEST,M3.5.0/1,M10.5.0"},
  {"Casablanca",            "WET0"},
  {"Lagos / Algiers",       "WAT-1"},
  {"Berlin / Paris",        "CET-1CEST,M3.5.0,M10.5.0/3"},
  {"Madrid / Rome",         "CET-1CEST,M3.5.0,M10.5.0/3"},
  {"Cairo",                 "EET-2"},
  {"Johannesburg",          "SAST-2"},
  {"Athens / Helsinki",     "EET-2EEST,M3.5.0/3,M10.5.0/4"},
  {"Istanbul",              "TRT-3"},
  {"Moscow",                "MSK-3"},
  {"Nairobi",               "EAT-3"},
  {"Tehran",                "IRST-3:30IRDT,J79/24,J263/24"},
  {"Dubai / Abu Dhabi",     "GST-4"},
  {"Kabul",                 "AFT-4:30"},
  {"Karachi",               "PKT-5"},
  {"India / Sri Lanka",     "IST-5:30"},
  {"Kathmandu",             "NPT-5:45"},
  {"Dhaka",                 "BDT-6"},
  {"Yangon",                "MMT-6:30"},
  {"Bangkok / Jakarta",     "ICT-7"},
  {"Beijing / Singapore",   "CST-8"},
  {"Hong Kong / Taipei",    "HKT-8"},
  {"Perth",                 "AWST-8"},
  {"Tokyo / Seoul",         "JST-9"},
  {"Adelaide",              "ACST-9:30ACDT,M10.1.0,M4.1.0/3"},
  {"Brisbane",              "AEST-10"},
  {"Sydney / Melbourne",    "AEST-10AEDT,M10.1.0,M4.1.0/3"},
  {"Solomon Islands",       "SBT-11"},
  {"Auckland",              "NZST-12NZDT,M9.5.0,M4.1.0/3"},
  {"Fiji",                  "FJT-12"},
  {"Samoa",                 "WST-13"},
  {"Kiritimati",            "LINT-14"},
};
const int kTimezoneCount = sizeof(kTimezones) / sizeof(kTimezones[0]);
const int8_t TZ_DEFAULT_INDEX = 7;

static_assert(sizeof(EepromSettings) <= EEPROM_SIZE,
              "EepromSettings exceeds EEPROM_SIZE; bump EEPROM_SIZE or trim AppSettings");

// --- Application state ---
AppSettings settings;
uint8_t activeScaleBackend = SCALE_BACKEND_AUTO;
bool sdPresent = false;
bool rtcAvailable = false;
volatile bool timeAvailable = false;

WifiAutoState wifiAutoState = WIFI_AUTO_IDLE;
unsigned long wifiAutoStateMs = 0;
unsigned long wifiAutoNextRetryMs = 0;
ScreenMode currentScreen = SCREEN_HOME;

Sample samples[CAPTURE_MAX_SAMPLES];
Sample captureCopy[CAPTURE_MAX_SAMPLES / 2];
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
String loadedFileRating;
int tzPickerScroll = 0;
String savedFileList[16];
int savedFileCount = 0;
int savedFileScroll = 0;

bool captureResultShowGraph = true;
float captureResultImpulse = 0.0f;
float captureResultAvgThrust = 0.0f;
float captureResultBurnTime = 0.0f;
float captureResultMaxForce = 0.0f;
float captureResultDelay = 0.0f;
String captureResultRating;
String captureResultSaveStatus;
unsigned long captureResultThrustStartMs = 0;
unsigned long captureResultThrustEndMs = 0;
unsigned long captureResultEjectMs = 0;
bool captureResultEjectDetected = false;

bool touchActive = false;
uint8_t displayRotation = 1;
uint16_t touchScreenMinimumX = 200;
uint16_t touchScreenMaximumX = 3700;
uint16_t touchScreenMinimumY = 240;
uint16_t touchScreenMaximumY = 3800;
const uint16_t TOUCH_Z_MIN = 1;
bool runtimeDebugFeaturesEnabled = DEBUG_FEATURES_DEFAULT != 0;
int paramsPage = 0;

long sensorOffsetRaw = 0;
uint8_t adsReg0Active = 0x0E;

volatile long adsLastRaw = 0;
volatile uint32_t adsReadCount = 0;
volatile uint32_t adsReadyCount = 0;
volatile uint32_t adsNotReadyCount = 0;
volatile uint32_t adsDrdyLowCount = 0;
volatile uint32_t adsDrdyHighCount = 0;
volatile long adsMinRawSeen = 0;
volatile long adsMaxRawSeen = 0;
volatile float adsAbsDeltaEma = 0.0f;
volatile float adsReadRateHzEma = 0.0f;
volatile uint32_t adsSpikeCount = 0;
volatile uint32_t adsClipCount = 0;
volatile uint32_t adsAllZeroFrameCount = 0;
volatile uint32_t adsAllOneFrameCount = 0;
volatile unsigned long adsLastReadUs = 0;

const int SCALE_TREND_POINTS = 120;
float scaleTrendGrams[SCALE_TREND_POINTS];
int scaleTrendHead = 0;
int scaleTrendCount = 0;
float scaleLastFilteredGrams = 0.0f;
uint32_t scaleTrendSampleCounter = 0;
const int SCALE_CAPTURE_MARKERS_MAX = 4;
uint32_t scaleCaptureMarkerSample[SCALE_CAPTURE_MARKERS_MAX];
float scaleCaptureMarkerGrams[SCALE_CAPTURE_MARKERS_MAX];
uint16_t scaleCaptureMarkerId[SCALE_CAPTURE_MARKERS_MAX];
int scaleCaptureMarkerCount = 0;
uint16_t scaleCaptureMarkerNextId = 1;
bool scaleCaptureListDirty = true;

void setup() {
  Serial.begin(115200);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);

  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin();
  applyRotation(displayRotation);
  tft.setTextWrap(true);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(16, 120);
  tft.print("Booting...");
  touchBegin();

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  delay(100);

  Serial.print("Attempting SD.begin(CS=");
  Serial.print(SD_CS);
  Serial.println(") on ESP32-2432S028...");

  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdPresent = SD.begin(SD_CS, spiSD, 1000000);
  if (!sdPresent) {
    Serial.println("SD init failed at 1MHz, retrying at default speed...");
    sdPresent = SD.begin(SD_CS, spiSD);
  }

  if (sdPresent) {
    Serial.println("SD card initialized successfully!");
  } else {
    Serial.println("SD card initialization failed. Using EEPROM fallback.");
  }
  EEPROM.begin(EEPROM_SIZE);
  loadSettings();
  displayRotation = settings.rotate180 ? 2 : 1;
  applyRotation(displayRotation);

  Serial.println("Initializing scale...");
  Serial.flush();
  bool scaleReady = initScale();
  Serial.println("[DEBUG] initScale completed");
  Serial.flush();
  if (!scaleReady) {
    statusLine = "Scale ADC not detected";
    drawMessage(statusLine);
    delay(500);
  }

  Serial.println("[DEBUG] Drawing home screen...");
  Serial.flush();

  drawHomeScreen();
  Serial.println("[DEBUG] Home screen drawn");
  Serial.flush();

  if (settings.wifiSsid[0] != '\0') {
    Serial.print("[WIFI-AUTO] Attempting connect to '");
    Serial.print(settings.wifiSsid);
    Serial.println("' in background.");
    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(settings.wifiSsid, settings.wifiPassword);
    wifiAutoState = WIFI_AUTO_CONNECTING;
    wifiAutoStateMs = millis();
  }
  beginWebServer();
}

void loop() {
  TouchPoint touch;
  static unsigned long lastScaleUiMs = 0;
  static unsigned long lastCalUiMs = 0;
  if (getTouchedPoint(touch)) {
    if (runtimeDebugFeaturesEnabled) {
      tft.fillCircle(touch.x, touch.y, 2, ILI9341_MAGENTA);
    }
    if (hitTest(touch, 0, 0, 28, 24)) {
      if (currentScreen == SCREEN_HOME) {
        // root screen: no-op
      } else if (currentScreen == SCREEN_WIFI_SCAN || currentScreen == SCREEN_PARAMETERS || currentScreen == SCREEN_CALIBRATION) {
        currentScreen = SCREEN_SETTINGS;
        drawSettingsScreen();
      } else if (currentScreen == SCREEN_TIMEZONE) {
        currentScreen = SCREEN_PARAMETERS;
        drawParametersScreen();
      } else if (currentScreen == SCREEN_REVIEW) {
        currentScreen = SCREEN_LOAD_FILE;
        drawLoadFilesScreen();
      } else {
        currentScreen = SCREEN_HOME;
        drawHomeScreen();
      }
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
        if (hitTest(touch, 12, 180, 140, 44)) {
          drawFooter("Taring... hold still");
          scaleTareSensor();
          offsetTare = readUnitsAverage(10);

          scaleTrendHead = 0;
          scaleTrendCount = 0;
          scaleTrendSampleCounter = 0;
          drawScaleTrendGraph();
          scaleCaptureListDirty = true;
          drawScaleCaptureList();
          drawMessage("Tared. Graph reset.");
        } else if (hitTest(touch, 168, 180, 140, 44)) {
          addScaleCaptureMarker(scaleLastFilteredGrams);
          drawScaleTrendGraph();
          drawScaleCaptureList();
          drawMessage("Captured " + String(scaleLastFilteredGrams, 1) + " g");
        }
        break;
      case SCREEN_CAPTURE:
        if (hitTest(touch, 90, 150, 140, 64)) {
          runCaptureRoutine();
        }
        break;
      case SCREEN_CAPTURE_RESULT:
        if (hitTest(touch, 12, 178, 92, 34)) {
          captureResultShowGraph = true;
          drawCaptureResultScreen();
        } else if (hitTest(touch, 114, 178, 92, 34)) {
          captureResultShowGraph = false;
          drawCaptureResultScreen();
        } else if (hitTest(touch, 216, 178, 92, 34)) {
          currentScreen = SCREEN_HOME;
          drawHomeScreen();
        }
        break;
      case SCREEN_LOAD_FILE:
        if (hitTest(touch, 40, 202, 110, 34)) {
          if (savedFileScroll >= 4) {
            savedFileScroll -= 4;
          }
          drawLoadFilesScreen();
        } else if (hitTest(touch, 170, 202, 110, 34)) {
          if (savedFileScroll + 4 < savedFileCount) {
            savedFileScroll += 4;
          }
          drawLoadFilesScreen();
        } else {
          for (int i = 0; i < 4; i++) {
            int idx = savedFileScroll + i;
            int y = 40 + i * 38;
            if (idx < savedFileCount && hitTest(touch, 16, y, 288, 32)) {
              loadedFileName = savedFileList[idx];
              loadEngFile(loadedFileName);
              currentScreen = SCREEN_REVIEW;
              drawReviewScreen();
              break;
            }
          }
        }
        break;
      case SCREEN_TIMEZONE: {
        const int rowsVisible = 5;
        const int rowH = 32;
        if (hitTest(touch, 40, 202, 110, 34)) {
          if (tzPickerScroll >= rowsVisible) tzPickerScroll -= rowsVisible;
          else tzPickerScroll = 0;
          drawTimezonePickerScreen();
        } else if (hitTest(touch, 170, 202, 110, 34)) {
          int maxScroll = max(0, kTimezoneCount - rowsVisible);
          if (tzPickerScroll + rowsVisible <= maxScroll) tzPickerScroll += rowsVisible;
          else tzPickerScroll = maxScroll;
          drawTimezonePickerScreen();
        } else {
          for (int i = 0; i < rowsVisible; i++) {
            int idx = tzPickerScroll + i;
            if (idx >= kTimezoneCount) break;
            int y = 32 + i * rowH;
            if (hitTest(touch, 8, y, 304, rowH - 4)) {
              settings.timezoneIndex = (int8_t)idx;
              applyTimezone();
              if (WiFi.status() == WL_CONNECTED) {
                configTzTime(kTimezones[idx].posix, NTP_SERVER);
              }
              saveSettings();
              currentScreen = SCREEN_PARAMETERS;
              drawParametersScreen();
              drawMessage(String("TZ: ") + kTimezones[idx].label);
              break;
            }
          }
        }
        break;
      }
      case SCREEN_REVIEW:
        if (hitTest(touch, 180, 212, 132, 26)) {
          if (sdPresent && loadedFileName.length() > 0) {
            String path = loadedFileName;
            if (path[0] != '/') path = String("/") + path;
            SD.remove(path);
            Serial.print("[DELETE] Removed ");
            Serial.println(path);
          }
          sampleCount = 0;
          loadedFileName = "";
          loadedFileRating = "";
          refreshSavedFileList();
          currentScreen = SCREEN_LOAD_FILE;
          drawLoadFilesScreen();
        }
        break;
      case SCREEN_SETTINGS:
        if (hitTest(touch, 16, 40, 140, 64)) {
          currentScreen = SCREEN_WIFI_SCAN;
          drawHeader("WiFi Scan");
          drawHamburger();
          drawTextBlock(16, 100, 288, 40, "Scanning networks...");
          drawFooter("Please wait.");
          wifiScroll = 0;
          scanWifiNetworks();
          drawWifiScanScreen();
        } else if (hitTest(touch, 164, 40, 140, 64)) {
          currentScreen = SCREEN_CALIBRATION;
          weightEntry = "";
          drawCalibrationScreen(0.0f);
        } else if (hitTest(touch, 90, 112, 140, 64)) {
          paramsPage = 0;
          currentScreen = SCREEN_PARAMETERS;
          drawParametersScreen();
        }
        break;
      case SCREEN_PARAMETERS: {
        if (hitTest(touch, 40, 202, 110, 34)) {
          if (paramsPage > 0) {
            paramsPage--;
            drawParametersScreen();
          }
        } else if (hitTest(touch, 170, 202, 110, 34)) {
          if (paramsPage < PARAMS_PAGE_COUNT - 1) {
            paramsPage++;
            drawParametersScreen();
          }
        } else if (hitTest(touch, 210, paramRowY(0) + 4, 22, 22) && paramRowVisible(0)) {
          settings.rotate180 = (settings.rotate180 == 1) ? 0 : 1;
          displayRotation = settings.rotate180 ? 2 : 1;
          applyRotation(displayRotation);
          saveSettings();
          resetTouchBuffer();
          drawParametersScreen();
          drawMessage(String("Rotate 180 ") + (settings.rotate180 ? "ON" : "OFF"));
        } else if (hitTest(touch, 246, paramRowY(1) + 5, 58, 20) && paramRowVisible(1)) {
          String entered = inputTextKeyboard("Start threshold N", String(settings.startThreshold, 1), false, true);
          float v = entered.toFloat();
          if (v > 0.0f && v < 2000.0f) {
            settings.startThreshold = v;
            saveSettings();
            drawParametersScreen();
          } else {
            drawMessage("Invalid start threshold");
          }
        } else if (hitTest(touch, 220, paramRowY(2) + 5, 84, 20) && paramRowVisible(2)) {
          String entered = inputTextKeyboard("Pre capture sec", String((float)settings.preCaptureMs / 1000.0f, 1), false, true);
          float v = entered.toFloat();
          if (v >= 0.10f && v <= 12.00f) {
            settings.preCaptureMs = (uint16_t)roundf(v * 1000.0f);
            saveSettings();
            drawParametersScreen();
          } else {
            drawMessage("Use 0.10..12.00 s");
          }
        } else if (hitTest(touch, 220, paramRowY(3) + 5, 84, 20) && paramRowVisible(3)) {
          String entered = inputTextKeyboard("Post capture sec", String((float)settings.postCaptureMs / 1000.0f, 1), false, true);
          float v = entered.toFloat();
          if (v >= 0.10f && v <= 10.00f) {
            settings.postCaptureMs = (uint16_t)roundf(v * 1000.0f);
            saveSettings();
            drawParametersScreen();
          } else {
            drawMessage("Use 0.10..10.00 s");
          }
        } else if (hitTest(touch, 220, paramRowY(4) + 5, 84, 20) && paramRowVisible(4)) {
          String entered = inputTextKeyboard("End holdoff ms", String(settings.thrustEndHoldoffMs), false, false);
          int v = entered.toInt();
          if (v >= 20 && v <= 500) {
            settings.thrustEndHoldoffMs = (uint16_t)v;
            saveSettings();
            drawParametersScreen();
          } else {
            drawMessage("Use 20..500 ms");
          }
        } else if (hitTest(touch, 220, paramRowY(5) + 5, 84, 20) && paramRowVisible(5)) {
          String entered = inputTextKeyboard("Ejection wait sec", String(settings.ejectionWaitSeconds, 1), false, true);
          float v = entered.toFloat();
          if (v >= 0.0f && v <= 15.0f) {
            settings.ejectionWaitSeconds = v;
            saveSettings();
            drawParametersScreen();
          } else {
            drawMessage("Use 0..15.0 s (0=off)");
          }
        } else if (hitTest(touch, 220, paramRowY(6) + 5, 84, 20) && paramRowVisible(6)) {
          String entered = inputTextKeyboard("Eject detect N", String(settings.ejectionDetectForceN, 1), false, true);
          float v = entered.toFloat();
          if (v >= 0.2f && v <= 30.0f) {
            settings.ejectionDetectForceN = v;
            saveSettings();
            drawParametersScreen();
          } else {
            drawMessage("Use 0.2..30.0 N");
          }
        } else if (hitTest(touch, 220, paramRowY(7) + 5, 84, 20) && paramRowVisible(7)) {
          String entered = inputTextKeyboard("Max burn sec", String((float)settings.maxBurnMs / 1000.0f, 1), false, true);
          float v = entered.toFloat();
          if (v >= 0.5f && v <= 30.0f) {
            settings.maxBurnMs = (uint16_t)roundf(v * 1000.0f);
            saveSettings();
            drawParametersScreen();
          } else {
            drawMessage("Use 0.5..30.0 s");
          }
        } else if (hitTest(touch, 170, paramRowY(8) + 5, 134, 20) && paramRowVisible(8)) {
          tzPickerScroll = max(0, settings.timezoneIndex - 2);
          int maxScroll = max(0, kTimezoneCount - 5);
          if (tzPickerScroll > maxScroll) tzPickerScroll = maxScroll;
          currentScreen = SCREEN_TIMEZONE;
          drawTimezonePickerScreen();
        }
        break;
      }
      case SCREEN_WIFI_SCAN: {
        const int rowH = 34;
        const int rowsVisible = 5;
        if (hitTest(touch, 16, 218, 68, 20)) {
          if (wifiScroll >= rowsVisible) {
            wifiScroll -= rowsVisible;
          } else {
            wifiScroll = 0;
          }
          drawWifiScanScreen();
        } else if (hitTest(touch, 90, 218, 68, 20)) {
          if (wifiScroll + rowsVisible < wifiCount) {
            wifiScroll += rowsVisible;
          }
          drawWifiScanScreen();
        } else if (hitTest(touch, 164, 218, 68, 20)) {
          drawHeader("WiFi Scan");
          drawHamburger();
          drawTextBlock(16, 100, 288, 40, "Scanning networks...");
          drawFooter("Please wait.");
          wifiScroll = 0;
          scanWifiNetworks();
          drawWifiScanScreen();
        } else if (hitTest(touch, 238, 218, 68, 20)) {
          currentScreen = SCREEN_HOME;
          drawHomeScreen();
        } else {
          for (int i = 0; i < rowsVisible; i++) {
            int idx = wifiScroll + i;
            int y = 32 + i * rowH;
            if (idx < wifiCount && hitTest(touch, 16, y, 288, rowH - 4)) {
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
      }
      case SCREEN_CALIBRATION:
        if (hitTest(touch, 16, 122, 92, 34)) {
          drawMessage("Taring (1.5s average)...");
          scaleTareSensor();
          offsetTare = readUnitsAverage(20);
          drawMessage("Tare complete.");
          drawCalibrationScreen(0.0f);
        } else if (hitTest(touch, 16, 164, 140, 34)) {
          weightEntry = inputTextKeyboard("Enter weight (g)", weightEntry, false, true);
          drawCalibrationScreen(0.0f);
        } else if (hitTest(touch, 164, 164, 140, 34)) {
          float target = weightEntry.toFloat();
          if (target > 0.0f) {
            if (activeScaleBackend == SCALE_BACKEND_ADS1220 && adsAllZeroFrameCount > 10) {
              drawMessage("Calibration blocked: ADS comm fault (all-zero frames).");
              break;
            }
            float kg = target / 1000.0f;
            long rawAvg = readRawAverage(20, 40);
            long rawDelta = rawAvg - getSensorOffsetRaw();
            if (kg > 0.0f && rawDelta != 0) {
              float prevFactor = settings.calibrationFactor;
              settings.calibrationFactor = ((float)rawDelta / kg) * LOAD_POLARITY;
              Serial.print("CAL rawAvg=");
              Serial.print(rawAvg);
              Serial.print(" rawDelta=");
              Serial.print(rawDelta);
              Serial.print(" target_g=");
              Serial.print(target, 2);
              Serial.print(" prevFactor=");
              Serial.print(prevFactor, 3);
              Serial.print(" newFactor=");
              Serial.println(settings.calibrationFactor, 3);
              applyScaleSettings();
              saveSettings();
              drawCalibrationScreen(0.0f);
              drawMessage("Cal saved. Factor updated.");
            } else {
              drawMessage("No usable delta; check load, wiring, and ADS comm.");
            }
          } else {
            drawMessage("Enter a valid weight.");
          }
        }
        break;
    }
  }

  if (currentScreen == SCREEN_SCALE) {
    unsigned long now = millis();
    if (now - lastScaleUiMs > 80) {
      drawScaleValue();
      lastScaleUiMs = now;
    }
  }

  emitScaleDiagnosticsSerial();

  if (currentScreen == SCREEN_CALIBRATION) {
    static bool calMassInitialized = false;
    static float calMassFiltered = 0.0f;
    unsigned long now = millis();
    if (now - lastCalUiMs > 80) {
      float measured = netLoadKgFromUnits(readUnitsAverage(CAL_UI_RAW_SAMPLES)) * 1000.0f;
      if (!calMassInitialized) {
        calMassFiltered = measured;
        calMassInitialized = true;
      } else {
        calMassFiltered = calMassFiltered * 0.50f + measured * 0.50f;
      }
      tft.fillRect(16, 66, tft.width() - 32, 22, ILI9341_BLACK);
      tft.setTextSize(2);
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(16, 68);
      tft.print("Mass: ");
      tft.print(calMassFiltered, 1);
      tft.print(" g");
      lastCalUiMs = now;
    }
  }

  if (currentScreen == SCREEN_HOME) {
    static unsigned long lastClockMs = 0;
    unsigned long now = millis();
    if (now - lastClockMs > 1000UL) {
      drawHomeClock();
      lastClockMs = now;
    }
  }

  handleWebServer();

  // Background WiFi/NTP state machine.
  {
    unsigned long now = millis();
    switch (wifiAutoState) {
      case WIFI_AUTO_IDLE:
        if (settings.wifiSsid[0] != '\0' && now >= wifiAutoNextRetryMs) {
          WiFi.mode(WIFI_STA);
          WiFi.setAutoReconnect(true);
          WiFi.begin(settings.wifiSsid, settings.wifiPassword);
          wifiAutoState = WIFI_AUTO_CONNECTING;
          wifiAutoStateMs = now;
        }
        break;
      case WIFI_AUTO_CONNECTING:
        if (WiFi.status() == WL_CONNECTED) {
          Serial.print("[WIFI-AUTO] Connected, IP=");
          Serial.println(WiFi.localIP());
          configTzTime(kTimezones[settings.timezoneIndex].posix, NTP_SERVER);
          wifiAutoState = WIFI_AUTO_NTP_WAIT;
          wifiAutoStateMs = now;
        } else if (now - wifiAutoStateMs > 30000UL) {
          Serial.println("[WIFI-AUTO] Connect timed out; will retry.");
          wifiAutoState = WIFI_AUTO_FAILED;
          wifiAutoNextRetryMs = now + 60000UL;
        }
        break;
      case WIFI_AUTO_NTP_WAIT: {
        struct tm timeinfo;
        if (getLocalTime(&timeinfo, 0) && (timeinfo.tm_year + 1900) >= 2024) {
          timeAvailable = true;
          char buf[32];
          strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
          Serial.print("[WIFI-AUTO] NTP time set: ");
          Serial.println(buf);
          if (currentScreen == SCREEN_HOME) {
            drawHomeClock();
          }
          wifiAutoState = WIFI_AUTO_DONE;
        } else if (now - wifiAutoStateMs > 30000UL) {
          Serial.println("[WIFI-AUTO] NTP timed out.");
          wifiAutoState = WIFI_AUTO_FAILED;
          wifiAutoNextRetryMs = now + 300000UL;
        }
        break;
      }
      case WIFI_AUTO_DONE:
        break;
      case WIFI_AUTO_FAILED:
        if (now >= wifiAutoNextRetryMs) {
          wifiAutoState = WIFI_AUTO_IDLE;
        }
        break;
    }
  }

  // Hot SD insertion/removal probe.
  static unsigned long lastSdProbeMs = 0;
  {
    unsigned long now = millis();
    if (now - lastSdProbeMs > 2000UL) {
      lastSdProbeMs = now;
      if (!sdPresent) {
        SD.end();
        spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
        bool mounted = SD.begin(SD_CS, spiSD, 1000000);
        if (!mounted) mounted = SD.begin(SD_CS, spiSD);
        if (mounted) {
          sdPresent = true;
          Serial.println("SD card detected (hot-mount).");
          loadSettingsFromSD();
          refreshSavedFileList();
          if (currentScreen == SCREEN_HOME) {
            drawMessage("SD card mounted.");
          }
        }
      } else {
        if (SD.cardType() == CARD_NONE) {
          sdPresent = false;
          SD.end();
          Serial.println("SD card removed.");
          if (currentScreen == SCREEN_HOME) {
            drawMessage("SD card removed.");
          }
        }
      }
    }
  }

  delay(30);
}
