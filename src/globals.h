#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <HX711.h>
#include "config.h"

// --- Shared types ---
struct AppSettings {
  char wifiSsid[32];
  char wifiPassword[64];
  float calibrationFactor;
  uint8_t rotate180;
  float startThreshold;
  uint16_t preCaptureMs;
  uint16_t postCaptureMs;
  float ejectionWaitSeconds;
  float ejectionDetectForceN;
  uint16_t thrustEndHoldoffMs;
  uint16_t maxBurnMs;
  int8_t timezoneIndex;
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
  SCREEN_CAPTURE_RESULT,
  SCREEN_LOAD_FILE,
  SCREEN_REVIEW,
  SCREEN_SETTINGS,
  SCREEN_PARAMETERS,
  SCREEN_WIFI_SCAN,
  SCREEN_WIFI_PASSWORD,
  SCREEN_CALIBRATION,
  SCREEN_TIMEZONE
};

enum WifiAutoState {
  WIFI_AUTO_IDLE,
  WIFI_AUTO_CONNECTING,
  WIFI_AUTO_NTP_WAIT,
  WIFI_AUTO_DONE,
  WIFI_AUTO_FAILED
};

struct TzEntry { const char* label; const char* posix; };

// --- Hardware singletons ---
extern Adafruit_ILI9341 tft;
extern HX711 scale;
extern RTC_DS3231 rtc;
extern SPIClass spiSD;

// --- Timezone table ---
extern const TzEntry kTimezones[];
extern const int kTimezoneCount;
extern const int8_t TZ_DEFAULT_INDEX;

// --- Application state ---
extern AppSettings settings;
extern uint8_t activeScaleBackend;
extern bool sdPresent;
extern bool rtcAvailable;
extern volatile bool timeAvailable;

extern WifiAutoState wifiAutoState;
extern unsigned long wifiAutoStateMs;
extern unsigned long wifiAutoNextRetryMs;
extern ScreenMode currentScreen;

extern Sample samples[];
extern Sample captureCopy[];
extern int sampleCount;
extern float offsetTare;

extern WifiNetwork wifiNetworks[];
extern int wifiCount;
extern int wifiScroll;
extern int selectedWifi;

extern String passwordEntry;
extern String weightEntry;
extern String statusLine;

extern String loadedFileName;
extern String loadedFileRating;
extern int tzPickerScroll;
extern String savedFileList[];
extern int savedFileCount;
extern int savedFileScroll;

extern bool captureResultShowGraph;
extern float captureResultImpulse;
extern float captureResultAvgThrust;
extern float captureResultBurnTime;
extern float captureResultMaxForce;
extern float captureResultDelay;
extern String captureResultRating;
extern String captureResultSaveStatus;
extern unsigned long captureResultThrustStartMs;
extern unsigned long captureResultThrustEndMs;
extern unsigned long captureResultEjectMs;
extern bool captureResultEjectDetected;

extern bool touchActive;
extern uint8_t displayRotation;
extern uint16_t touchScreenMinimumX;
extern uint16_t touchScreenMaximumX;
extern uint16_t touchScreenMinimumY;
extern uint16_t touchScreenMaximumY;
extern const uint16_t TOUCH_Z_MIN;
extern bool runtimeDebugFeaturesEnabled;
extern int paramsPage;

extern long sensorOffsetRaw;
extern uint8_t adsReg0Active;

extern volatile long adsLastRaw;
extern volatile uint32_t adsReadCount;
extern volatile uint32_t adsReadyCount;
extern volatile uint32_t adsNotReadyCount;
extern volatile uint32_t adsDrdyLowCount;
extern volatile uint32_t adsDrdyHighCount;
extern volatile long adsMinRawSeen;
extern volatile long adsMaxRawSeen;
extern volatile float adsAbsDeltaEma;
extern volatile float adsReadRateHzEma;
extern volatile uint32_t adsSpikeCount;
extern volatile uint32_t adsClipCount;
extern volatile uint32_t adsAllZeroFrameCount;
extern volatile uint32_t adsAllOneFrameCount;
extern volatile unsigned long adsLastReadUs;

extern const int SCALE_TREND_POINTS;
extern float scaleTrendGrams[];
extern int scaleTrendHead;
extern int scaleTrendCount;
extern float scaleLastFilteredGrams;
extern uint32_t scaleTrendSampleCounter;
extern const int SCALE_CAPTURE_MARKERS_MAX;
extern uint32_t scaleCaptureMarkerSample[];
extern float scaleCaptureMarkerGrams[];
extern uint16_t scaleCaptureMarkerId[];
extern int scaleCaptureMarkerCount;
extern uint16_t scaleCaptureMarkerNextId;
extern bool scaleCaptureListDirty;

// --- scale.cpp ---
const char* scaleBackendLabel(uint8_t backend);
unsigned long captureSampleIntervalUs();
unsigned long captureSampleRateSps();
unsigned long captureSampleIntervalMsRoundedUp();
bool initScale();
bool sensorReady();
long sensorReadRaw();
long readRawAverage(byte times = 1, unsigned long timeoutPerReadMs = 25);
float rawToUnits(long raw);
float netLoadKgFromUnits(float units);
float readUnitsAverage(byte times = 1, unsigned long timeoutPerReadMs = 25);
void applyScaleSettings();
long captureTimedBaselineRaw(unsigned long durationMs, unsigned long timeoutPerReadMs = 25);
void scaleTareSensor();
long getSensorOffsetRaw();
void resetAdsLiveDiagnostics();
void emitScaleDiagnosticsSerial();
void addScaleCaptureMarker(float grams);

// --- storage.cpp ---
void resetDefaultSettings();
bool saveSettingsToEEPROM();
bool saveSettingsToSD();
bool loadSettingsFromEEPROM();
bool loadSettingsFromSD();
bool saveSettings();
bool loadSettings();
void refreshSavedFileList();
void loadEngFile(const String& filename);
void saveAndShowStatus(const String& message);

// --- netclock.cpp ---
void applyTimezone();
void scanWifiNetworks();
bool connectWiFi(const String& ssid, const String& password);
bool syncTimeWithNTP();
bool formatLocalTime(char* buf, size_t bufLen, const char* fmt);

// --- ui_draw.cpp ---
bool touchPressed();
void touchBegin();
uint16_t xptRead(uint8_t command);
TouchPoint getTouchPoint();
bool getTouchedPoint(TouchPoint &p);
bool hitTest(const TouchPoint &p, int x, int y, int w, int h);
void drawHeader(const char* title);
void drawHamburger();
void drawFooter(const char* text, uint16_t color = ILI9341_CYAN);
void drawButton(int x, int y, int w, int h, const String& label);
void drawSmallButton(int x, int y, int w, int h, const String& label, uint16_t color = ILI9341_BLUE);
void drawTextBlock(int x, int y, int w, int h, const String& text);
void drawGraphYAxisLabels(int left, int top, int graphW, int graphH, float maxForce);
void drawMessage(const String& message);
void drawKeyboard(const String& title, const String& value, bool password, const char* const keys[], int rows, bool showShift, bool shiftActive);
String inputTextKeyboard(const String& title, const String& initial, bool passwordMode, bool allowDot);
void resetTouchBuffer();
void applyRotation(uint8_t rot);
void drawScaleCaptureList();
void drawScaleTrendGraph();
void drawScaleDiagnostics();

// --- ui_screens.cpp ---
void drawHomeClock();
void drawHomeScreen();
void drawScaleValue();
void drawScaleScreen();
void drawCaptureScreen();
void drawReviewScreen();
void drawSettingsMenu();
void drawSettingsScreen();
void drawParametersScreen();
void drawCaptureResultScreen();
void drawCaptureCharacterization(float totalImpulse, float averageThrust, float burnTime, float maxForceMain, float delaySecs, const String& rating, const String& saveStatus);
void drawWifiScanScreen();
void drawCalibrationScreen(float measuredMass);
void drawTimezonePickerScreen();
void drawLoadFilesScreen();

// --- capture.cpp ---
void runCaptureRoutine();
