#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// --- Hardware pin configuration ---
#define SCALE_BACKEND_AUTO 0
#define SCALE_BACKEND_HX711 1
#define SCALE_BACKEND_ADS1220 2

// Set to SCALE_BACKEND_HX711 or SCALE_BACKEND_ADS1220.
#define LOADCELL_ADC SCALE_BACKEND_ADS1220

#include <HX711.h>
#define HX711_DOUT 22
#define HX711_SCK 27
// Bit-banged ADS1220 pins (pick pins that do not conflict with display/touch).
#define ADS1220_CS 17
#define ADS1220_SCLK 22
#define ADS1220_MOSI 16
#define ADS1220_MISO 35
#define ADS1220_DRDY 27

#define TFT_CS 15
#define TFT_DC 2
#define TFT_RST -1
#define TFT_BL 21
#define TFT_MOSI 13
#define TFT_MISO 12
#define TFT_SCLK 14

#define SD_CS 5

#define TOUCH_CS 33
#define TOUCH_IRQ 36
#define TOUCH_CLK 25
#define TOUCH_MOSI 32
#define TOUCH_MISO 39

#define I2C_SDA 18
#define I2C_SCL 19

#define NTP_SERVER "pool.ntp.org"
#define SETTINGS_FILENAME "/settings.ini"
#define EEPROM_MAGIC 0x52415350
#define EEPROM_SIZE 1024
#define DEFAULT_CAL_FACTOR -50230.0f
#define MIN_REASONABLE_CAL_FACTOR_ABS_HX711 50000.0f
#define MIN_REASONABLE_CAL_FACTOR_ABS_ADS1220 100.0f
#define CAPTURE_MAX_SAMPLES 4800
#define HX711_SAMPLE_RATE_SPS 80
#define ADS1220_SAMPLE_RATE_SPS 600
#define MAX_STORED_BURN_MS 5000
// Set to -1.0f or +1.0f to match load cell wiring polarity.
#define LOAD_POLARITY -1.0f
#define TARE_AVERAGE_DURATION_MS 1500UL
#define SCALE_UI_RAW_SAMPLES 8
#define CAL_UI_RAW_SAMPLES 6
#define DEBUG_FEATURES_DEFAULT 0

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
HX711 scale;
RTC_DS3231 rtc;

const char* scaleBackendLabel(uint8_t backend) {
  switch (backend) {
    case SCALE_BACKEND_HX711: return "HX711";
    case SCALE_BACKEND_ADS1220: return "ADS1220";
    default: return "Auto";
  }
}

unsigned long captureSampleIntervalUs() {
  unsigned long sampleRateSps = (LOADCELL_ADC == SCALE_BACKEND_ADS1220) ? ADS1220_SAMPLE_RATE_SPS : HX711_SAMPLE_RATE_SPS;
  return 1000000UL / max(1UL, sampleRateSps);
}

unsigned long captureSampleRateSps() {
  return (LOADCELL_ADC == SCALE_BACKEND_ADS1220) ? ADS1220_SAMPLE_RATE_SPS : HX711_SAMPLE_RATE_SPS;
}

unsigned long captureSampleIntervalMsRoundedUp() {
  return (captureSampleIntervalUs() + 999UL) / 1000UL;
}

struct AppSettings {
  char wifiSsid[32];
  char wifiPassword[64];
  float calibrationFactor;
  uint8_t scaleBackend;
  bool rtcEnabled;
  uint8_t rotate180;
  float startThreshold;
  uint16_t preCaptureMs;
  uint16_t postCaptureMs;
  bool waitForEjection;
  float ejectionWaitSeconds;
  float ejectionDetectForceN;
  uint16_t thrustEndHoldoffMs;
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
  SCREEN_RTC
};

AppSettings settings;
uint8_t activeScaleBackend = SCALE_BACKEND_AUTO;
bool sdPresent = false;
bool rtcAvailable = false;
ScreenMode currentScreen = SCREEN_HOME;

Sample samples[CAPTURE_MAX_SAMPLES];
Sample captureCopy[CAPTURE_MAX_SAMPLES];
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
uint8_t displayRotation = 1; // 1 and 3 are the two landscape orientations on ILI9341
uint16_t touchScreenMinimumX = 200;
uint16_t touchScreenMaximumX = 3700;
uint16_t touchScreenMinimumY = 240;
uint16_t touchScreenMaximumY = 3800;
const uint16_t TOUCH_Z_MIN = 1;
bool runtimeDebugFeaturesEnabled = DEBUG_FEATURES_DEFAULT != 0;

// Forward declarations
bool initScale();
long readRawAverage(byte times, unsigned long timeoutPerReadMs);
float readUnitsAverage(byte times = 1, unsigned long timeoutPerReadMs = 25);
void applyScaleSettings();
void scaleTareSensor();
long getSensorOffsetRaw();
void resetAdsLiveDiagnostics();
void emitScaleDiagnosticsSerial();
void drawScaleTrendGraph();
void drawScaleDiagnostics();

// Capture/task synchronization
SemaphoreHandle_t samplesMutex = NULL;
TaskHandle_t samplerTaskHandle = NULL;
volatile bool captureRequested = false;
volatile bool captureInProgress = false;
volatile bool captureFinished = false;
unsigned long captureStartMs = 0;
long sensorOffsetRaw = 0;
uint8_t adsReg0Active = 0x0E; // Default ADS1220 mux/gain: AIN0-AIN1, PGA=128

// ADS1220 live diagnostics (best-effort counters for field debugging).
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

void addScaleCaptureMarker(float grams) {
  uint32_t sampleIdx = (scaleTrendSampleCounter > 0) ? (scaleTrendSampleCounter - 1) : 0;
  if (scaleCaptureMarkerCount < SCALE_CAPTURE_MARKERS_MAX) {
    scaleCaptureMarkerSample[scaleCaptureMarkerCount] = sampleIdx;
    scaleCaptureMarkerGrams[scaleCaptureMarkerCount] = grams;
    scaleCaptureMarkerId[scaleCaptureMarkerCount] = scaleCaptureMarkerNextId++;
    scaleCaptureMarkerCount++;
    scaleCaptureListDirty = true;
    return;
  }

  for (int i = 1; i < SCALE_CAPTURE_MARKERS_MAX; i++) {
    scaleCaptureMarkerSample[i - 1] = scaleCaptureMarkerSample[i];
    scaleCaptureMarkerGrams[i - 1] = scaleCaptureMarkerGrams[i];
    scaleCaptureMarkerId[i - 1] = scaleCaptureMarkerId[i];
  }
  scaleCaptureMarkerSample[SCALE_CAPTURE_MARKERS_MAX - 1] = sampleIdx;
  scaleCaptureMarkerGrams[SCALE_CAPTURE_MARKERS_MAX - 1] = grams;
  scaleCaptureMarkerId[SCALE_CAPTURE_MARKERS_MAX - 1] = scaleCaptureMarkerNextId++;
  scaleCaptureListDirty = true;
}

void drawScaleCaptureList() {
  const int lx = 214;
  const int ly = 26;
  const int lw = 102;
  const int lh = 72;
  tft.fillRect(lx, ly, lw, lh, ILI9341_BLACK);
  tft.drawRect(lx, ly, lw, lh, ILI9341_DARKGREY);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(lx + 3, ly + 2);
  tft.print("Captures");
  tft.setTextColor(ILI9341_WHITE);
  for (int i = 0; i < scaleCaptureMarkerCount; i++) {
    int y = ly + 12 + i * 14;
    tft.setCursor(lx + 3, y);
    tft.print(scaleCaptureMarkerId[i]);
    tft.print(": ");
    tft.print(scaleCaptureMarkerGrams[i], 1);
    tft.print("g");
  }
  scaleCaptureListDirty = false;
}

bool sensorReady() {
  switch (activeScaleBackend) {
    case SCALE_BACKEND_HX711:
      return scale.is_ready();
    case SCALE_BACKEND_ADS1220: {
      bool drdyLow = digitalRead(ADS1220_DRDY) == LOW;
      if (drdyLow) {
        adsDrdyLowCount++;
      } else {
        adsDrdyHighCount++;
      }
      bool ready = drdyLow;
      if (ready) {
        adsReadyCount++;
      } else {
        adsNotReadyCount++;
      }
      return ready;
    }
    default:
      return false;
  }
}

long sensorReadRaw() {
  if (activeScaleBackend == SCALE_BACKEND_HX711) {
    return scale.read();
  }
  if (activeScaleBackend == SCALE_BACKEND_ADS1220) {
  auto adsShiftByte = [](uint8_t out) -> uint8_t {
    uint8_t in = 0;
    for (int i = 7; i >= 0; --i) {
      digitalWrite(ADS1220_MOSI, (out >> i) & 0x01);
      digitalWrite(ADS1220_SCLK, HIGH);
      delayMicroseconds(1);
      digitalWrite(ADS1220_SCLK, LOW);
      in = (uint8_t)((in << 1) | (digitalRead(ADS1220_MISO) ? 1 : 0));
      delayMicroseconds(1);
    }
    return in;
  };

  digitalWrite(ADS1220_CS, LOW);
  // In continuous-conversion mode, data can be shifted out directly.
  uint32_t b0 = adsShiftByte(0x00);
  uint32_t b1 = adsShiftByte(0x00);
  uint32_t b2 = adsShiftByte(0x00);
  digitalWrite(ADS1220_CS, HIGH);
  if (b0 == 0x00 && b1 == 0x00 && b2 == 0x00) {
    adsAllZeroFrameCount++;
  }
  if (b0 == 0xFF && b1 == 0xFF && b2 == 0xFF) {
    adsAllOneFrameCount++;
  }
  int32_t raw24 = (int32_t)((b0 << 16) | (b1 << 8) | b2);
  if (raw24 & 0x00800000) raw24 |= 0xFF000000;
  long raw = (long)raw24;
  long prevRaw = adsLastRaw;
  unsigned long nowUs = micros();

  if (adsReadCount == 0) {
    adsMinRawSeen = raw;
    adsMaxRawSeen = raw;
    adsAbsDeltaEma = 0.0f;
    adsReadRateHzEma = 0.0f;
    adsSpikeCount = 0;
    adsClipCount = 0;
  } else {
    long absDelta = labs(raw - prevRaw);
    adsAbsDeltaEma = adsAbsDeltaEma * 0.95f + ((float)absDelta * 0.05f);

    float spikeThreshold = max(4000.0f, adsAbsDeltaEma * 8.0f + 200.0f);
    if ((float)absDelta > spikeThreshold) {
      adsSpikeCount++;
    }

    unsigned long dtUs = nowUs - adsLastReadUs;
    if (dtUs > 0) {
      float hz = 1000000.0f / (float)dtUs;
      if (adsReadRateHzEma <= 0.01f) {
        adsReadRateHzEma = hz;
      } else {
        adsReadRateHzEma = adsReadRateHzEma * 0.90f + hz * 0.10f;
      }
    }
  }

  if (raw < adsMinRawSeen) adsMinRawSeen = raw;
  if (raw > adsMaxRawSeen) adsMaxRawSeen = raw;
  if (raw > 8300000L || raw < -8300000L) {
    adsClipCount++;
  }

  adsLastReadUs = nowUs;
  adsLastRaw = raw;
  adsReadCount++;
  return raw;
  }
  return 0;
}

bool touchPressed() {
  return digitalRead(TOUCH_IRQ) == LOW;
}

void touchBegin() {
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);
  pinMode(TOUCH_CLK, OUTPUT);
  digitalWrite(TOUCH_CLK, LOW);
  pinMode(TOUCH_MOSI, OUTPUT);
  digitalWrite(TOUCH_MOSI, LOW);
  pinMode(TOUCH_MISO, INPUT);
  pinMode(TOUCH_IRQ, INPUT);
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
}

uint16_t xptRead(uint8_t command) {
  uint16_t data = 0;
  digitalWrite(TOUCH_CS, LOW);

  for (int i = 7; i >= 0; i--) {
    digitalWrite(TOUCH_MOSI, (command >> i) & 0x01);
    digitalWrite(TOUCH_CLK, HIGH);
    delayMicroseconds(1);
    digitalWrite(TOUCH_CLK, LOW);
  }

  for (int i = 0; i < 16; i++) {
    digitalWrite(TOUCH_CLK, HIGH);
    data = (data << 1) | (digitalRead(TOUCH_MISO) ? 1 : 0);
    delayMicroseconds(1);
    digitalWrite(TOUCH_CLK, LOW);
  }

  digitalWrite(TOUCH_CS, HIGH);
  return data >> 3;  // 12-bit result
}

int median9(int *values, int count) {
  for (int i = 1; i < count; i++) {
    int key = values[i];
    int j = i - 1;
    while (j >= 0 && values[j] > key) {
      values[j + 1] = values[j];
      j--;
    }
    values[j + 1] = key;
  }
  return values[count / 2];
}

bool initScale() {
  auto initHx711Backend = [&]() -> bool {
    activeScaleBackend = SCALE_BACKEND_HX711;
    scale.begin(HX711_DOUT, HX711_SCK);
    unsigned long startMs = millis();
    while (!scale.is_ready() && millis() - startMs < 2000) {
      delay(10);
    }
    if (!scale.is_ready()) return false;
    applyScaleSettings();
    scale.tare();
    sensorOffsetRaw = scale.get_offset();
    offsetTare = readUnitsAverage(10);
    return true;
  };

  auto initAds1220Backend = [&]() -> bool {
    activeScaleBackend = SCALE_BACKEND_ADS1220;
    pinMode(ADS1220_CS, OUTPUT);
    digitalWrite(ADS1220_CS, HIGH);
    pinMode(ADS1220_SCLK, OUTPUT);
    digitalWrite(ADS1220_SCLK, LOW);
    pinMode(ADS1220_MOSI, OUTPUT);
    digitalWrite(ADS1220_MOSI, LOW);
    pinMode(ADS1220_MISO, INPUT);
    pinMode(ADS1220_DRDY, INPUT_PULLUP);

    auto adsShiftByte = [](uint8_t out) -> uint8_t {
      uint8_t in = 0;
      for (int i = 7; i >= 0; --i) {
        digitalWrite(ADS1220_MOSI, (out >> i) & 0x01);
        digitalWrite(ADS1220_SCLK, HIGH);
        delayMicroseconds(1);
        digitalWrite(ADS1220_SCLK, LOW);
        in = (uint8_t)((in << 1) | (digitalRead(ADS1220_MISO) ? 1 : 0));
        delayMicroseconds(1);
      }
      return in;
    };

    auto adsCommand = [&](uint8_t cmd) {
      digitalWrite(ADS1220_CS, LOW);
      adsShiftByte(cmd);
      digitalWrite(ADS1220_CS, HIGH);
    };

    auto adsWriteReg0 = [&](uint8_t reg0) {
      digitalWrite(ADS1220_CS, LOW);
      adsShiftByte(0x40); // WREG starting at REG0, write 1 register
      adsShiftByte(reg0);
      digitalWrite(ADS1220_CS, HIGH);
      adsReg0Active = reg0;
    };

    auto adsReadRegs = [&](uint8_t startReg, uint8_t count, uint8_t *out) {
      digitalWrite(ADS1220_CS, LOW);
      adsShiftByte((uint8_t)(0x20 | ((startReg & 0x03) << 2) | ((count - 1) & 0x03))); // RREG
      for (uint8_t i = 0; i < count; i++) {
        out[i] = adsShiftByte(0x00);
      }
      digitalWrite(ADS1220_CS, HIGH);
    };

    adsCommand(0x06); // RESET
    delay(2);
    // Configure ADS1220 registers:
    // REG0: MUX AIN0-AIN1, PGA=128 for highest sensitivity
    // REG1: DR=600 SPS, normal mode, temp sensor OFF, internal reference
    // REG2/REG3: defaults suitable for bridge measurement
    digitalWrite(ADS1220_CS, LOW);
    adsShiftByte(0x43); // WREG starting at REG0, write 4 registers
    adsShiftByte(0x0E); // REG0 default: AIN0-AIN1, PGA=128
    adsShiftByte(0xA4); // REG1: 600 SPS with default conversion control bits
    adsShiftByte(0x10); // REG2
    adsShiftByte(0x00); // REG3
    digitalWrite(ADS1220_CS, HIGH);
    delay(2);
    adsCommand(0x08); // START/SYNC

    uint8_t regs[4] = {0, 0, 0, 0};
    adsReadRegs(0, 4, regs);
    Serial.print("ADS regs r0=0x");
    Serial.print(regs[0], HEX);
    Serial.print(" r1=0x");
    Serial.print(regs[1], HEX);
    Serial.print(" r2=0x");
    Serial.print(regs[2], HEX);
    Serial.print(" r3=0x");
    Serial.println(regs[3], HEX);

    unsigned long startMs = millis();
    bool readySeen = false;
    while (millis() - startMs < 3000) {
      if (sensorReady()) {
        readySeen = true;
        break;
      }
      delay(1);
    }
    if (!readySeen) {
      Serial.println("ADS1220 DRDY not observed, using timed read fallback");
    }

    // Wiring confirmed: signal+ on AIN0, signal- on AIN1.
    adsCommand(0x0A); // POWERDOWN
    delay(1);
    adsWriteReg0(0x0E); // AIN0-AIN1, PGA=128
    adsCommand(0x08); // START/SYNC
    Serial.println("ADS mux locked: AIN0-AIN1 (PGA=128)");

    applyScaleSettings();
    sensorOffsetRaw = readRawAverage(12, 40);
    offsetTare = readUnitsAverage(10);
    return true;
  };

  if (LOADCELL_ADC == SCALE_BACKEND_HX711) {
    if (initHx711Backend()) return true;
    Serial.println("HX711 not detected (fixed LOADCELL_ADC)");
    activeScaleBackend = SCALE_BACKEND_AUTO;
    return false;
  }

  if (LOADCELL_ADC == SCALE_BACKEND_ADS1220) {
    if (initAds1220Backend()) return true;
    Serial.println("ADS1220 not detected (fixed LOADCELL_ADC)");
    activeScaleBackend = SCALE_BACKEND_AUTO;
    return false;
  }

  Serial.println("Invalid LOADCELL_ADC value");
  activeScaleBackend = SCALE_BACKEND_AUTO;
  return false;
}

TouchPoint getTouchPoint() {
  TouchPoint point = {0, 0, false};
  if (!touchPressed()) {
    return point;
  }

  int xs[9];
  int ys[9];
  int good = 0;
  xptRead(0x90);
  xptRead(0xD0);
  for (int i = 0; i < 9; i++) {
    int rx = (int)xptRead(0x90);  // Y+ ADC channel
    int ry = (int)xptRead(0xD0);  // X+ ADC channel
    if (rx < 100 || rx > 4000 || ry < 100 || ry > 4000) continue;
    xs[good] = rx;
    ys[good] = ry;
    good++;
    delay(2);
  }

  if (good < 5) {
    return point;
  }

  int rawX = median9(xs, good);
  int rawY = median9(ys, good);

  if (!touchPressed()) {
    return point;
  }

  int baseX = map(rawX, touchScreenMinimumX, touchScreenMaximumX, 0, tft.width() - 1);
  int baseY = map(rawY, touchScreenMaximumY, touchScreenMinimumY, 0, tft.height() - 1);

  int x = baseX;
  int y = baseY;
  // Match touch transform to display MADCTL flip selection.
  if (displayRotation == 1 || displayRotation == 3) {
    x = (tft.width() - 1) - x;
  }
  if (displayRotation == 2 || displayRotation == 3) {
    y = (tft.height() - 1) - y;
  }

  x = constrain(x, 0, tft.width() - 1);
  // Bottom-weighted Y trim: near top adds ~0 px, near bottom adds ~11 px.
  // This corrects the observed "dot increasingly above finger" behavior.
  int h = tft.height() - 1;
  if (h > 0) {
    y += (y * 11) / h;
  }

  // In 180 mode this panel reports touches slightly low; trim upward.
  if (displayRotation == 2) {
    y -= 12;
  }

  y = constrain(y, 0, tft.height() - 1);
  point.x = x;
  point.y = y;
  point.valid = true;
  return point;
}

bool getTouchedPoint(TouchPoint &p) {
  static unsigned long lastTouchMs = 0;
  static bool held = false;
  TouchPoint current = getTouchPoint();
  if (!current.valid) {
    held = false;
    return false;
  }
  if (held) {
    return false;
  }
  unsigned long now = millis();
  if (now - lastTouchMs < 180) {
    return false;
  }
  p = current;
  held = true;
  lastTouchMs = now;
  return p.valid;
}

bool hitTest(const TouchPoint &p, int x, int y, int w, int h) {
  return p.valid && p.x >= x && p.x < x + w && p.y >= y && p.y < y + h;
}

// Read helpers avoid hidden blocking and keep timing under explicit firmware control.
long readRawAverage(byte times = 1, unsigned long timeoutPerReadMs = 25) {
  long sum = 0;
  int got = 0;
  unsigned long start = millis();
  while (got < times && (millis() - start) < (unsigned long)times * timeoutPerReadMs) {
    if (sensorReady()) {
      sum += sensorReadRaw();
      got++;
    } else {
      delay(1);
    }
  }
  // If ADS readiness never asserted (e.g. DRDY pin not wired), force direct reads.
  if (got == 0 && activeScaleBackend == SCALE_BACKEND_ADS1220) {
    for (int i = 0; i < times; i++) {
      sum += sensorReadRaw();
      delay(2);
    }
    return sum / max(1, (int)times);
  }
  if (got == 0) return 0;
  return sum / got;
}

float rawToUnits(long raw) {
  return ((double)raw - (double)sensorOffsetRaw) / settings.calibrationFactor;
}

float netLoadKgFromUnits(float units) {
  return (units - offsetTare) * LOAD_POLARITY;
}

float readUnitsAverage(byte times, unsigned long timeoutPerReadMs) {
  long raw = readRawAverage(times, timeoutPerReadMs);
  return rawToUnits(raw);
}

// Sampler task: waits for captureRequested and performs fixed-rate capture into samples[].
void samplerTask(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    if (!captureRequested) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    // begin capture
    captureInProgress = true;
    captureFinished = false;
    sampleCount = 0;
    unsigned long preStart = millis();
    float baseline = 0.0f;
    int baselineCount = 0;
    while (millis() - preStart < settings.preCaptureMs) {
      if (sensorReady()) {
        long r = sensorReadRaw();
        float unit = rawToUnits(r);
        float f = netLoadKgFromUnits(unit) * 9.80665f;
        baseline += f;
        baselineCount++;
      }
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    baseline /= max(1, baselineCount);
    float threshold = baseline + settings.startThreshold;
    float endThreshold = baseline + settings.startThreshold * 0.5f;

    bool capturing = false;
    unsigned long lastAbove = 0;
    unsigned long lastSampleUs = micros();
    float maxForce = 0.0f;
    float totalImpulse = 0.0f;
    bool foundEject = false;
    unsigned long ejectTime = 0;
    unsigned long localCaptureStartMs = 0;

    const unsigned long sampleIntervalUs = captureSampleIntervalUs();
    while (true) {
      unsigned long nowUs = micros();
      if (nowUs - lastSampleUs < sampleIntervalUs) {
        vTaskDelay(pdMS_TO_TICKS(0));
        continue;
      }
      lastSampleUs += sampleIntervalUs;
      if (!sensorReady()) continue;
      long raw = sensorReadRaw();
      float unit = rawToUnits(raw);
      float f = netLoadKgFromUnits(unit) * 9.80665f;
      unsigned long nowMs = millis();
      if (!capturing) {
        if (f >= threshold) {
          capturing = true;
          localCaptureStartMs = nowMs;
          lastAbove = nowMs;
          // reset sample buffer
          xSemaphoreTake(samplesMutex, portMAX_DELAY);
          sampleCount = 0;
          xSemaphoreGive(samplesMutex);
        }
      }
      if (capturing) {
        xSemaphoreTake(samplesMutex, portMAX_DELAY);
        if (sampleCount < CAPTURE_MAX_SAMPLES) {
          unsigned long sampleTime = nowMs - localCaptureStartMs;
          samples[sampleCount++] = {sampleTime, f};
        }
        xSemaphoreGive(samplesMutex);
        if (f > maxForce) maxForce = f;
        if (f > endThreshold) lastAbove = nowMs;
        const float sampleIntervalSec = (sampleIntervalUs / 1000000.0f);
        totalImpulse += f * sampleIntervalSec;
        if (settings.waitForEjection && !foundEject && (nowMs - lastAbove) > 1000 && f > threshold * 1.5f) {
          foundEject = true;
          ejectTime = nowMs;
        }
        if (nowMs - lastAbove >= settings.postCaptureMs) {
          // capture complete
          captureStartMs = localCaptureStartMs;
          break;
        }
      }
    }

    // done
    captureRequested = false;
    captureInProgress = false;
    captureFinished = true;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
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
  // Square-tail back arrow for clearer visual shape.
  tft.fillTriangle(6, 12, 13, 6, 13, 18, ILI9341_WHITE);
  tft.fillRect(13, 10, 10, 4, ILI9341_WHITE);
}

void drawFooter(const char* text, uint16_t color = ILI9341_CYAN) {
  tft.fillRect(0, 216, tft.width(), 24, ILI9341_BLACK);
  tft.setTextColor(color);
  tft.setTextSize(1);
  tft.setCursor(4, 220);
  tft.print(text);
}

void drawButton(int x, int y, int w, int h, const String& label) {
  tft.fillRoundRect(x, y, w, h, 6, ILI9341_NAVY);
  tft.drawRoundRect(x, y, w, h, 6, ILI9341_WHITE);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  int nl = label.indexOf('\n');
  if (nl < 0) {
    int16_t textW = label.length() * 12;
    int16_t tx = x + max(4, (w - textW) / 2);
    int16_t ty = y + (h - 16) / 2;
    tft.setCursor(tx, ty);
    tft.print(label);
  } else {
    String line1 = label.substring(0, nl);
    String line2 = label.substring(nl + 1);
    int16_t textW1 = line1.length() * 12;
    int16_t textW2 = line2.length() * 12;
    int16_t tx1 = x + max(4, (w - textW1) / 2);
    int16_t tx2 = x + max(4, (w - textW2) / 2);
    int16_t ty1 = y + (h - 32) / 2;
    int16_t ty2 = ty1 + 16;
    tft.setCursor(tx1, ty1);
    tft.print(line1);
    tft.setCursor(tx2, ty2);
    tft.print(line2);
  }
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

void drawGraphYAxisLabels(int left, int top, int graphW, int graphH, float maxForce) {
  int stepN = (maxForce > 40.0f) ? 10 : 5;
  int topN = ((int)ceil(maxForce / stepN)) * stepN;
  if (topN <= 0) topN = stepN;
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_DARKGREY);
  for (int n = stepN; n <= topN; n += stepN) {
    int y = top + graphH - (int)((float)n * graphH / (float)topN);
    tft.drawFastHLine(left, y, graphW, ILI9341_DARKGREY);
    int labelX = max(0, left - 20);
    tft.setCursor(labelX, y - 3);
    tft.print(n);
  }
}

void resetDefaultSettings() {
  strcpy(settings.wifiSsid, "");
  strcpy(settings.wifiPassword, "");
  // CYD + HX711 + 1 kg class load cells are typically around 180k..260k counts/kg.
  settings.calibrationFactor = DEFAULT_CAL_FACTOR;
  settings.scaleBackend = SCALE_BACKEND_AUTO;
  settings.rtcEnabled = true;
  settings.rotate180 = 0;
  settings.startThreshold = 0.5f;
  settings.preCaptureMs = 500;
  settings.postCaptureMs = 1200;
  settings.waitForEjection = true;
  settings.ejectionWaitSeconds = 10.0f;
  settings.ejectionDetectForceN = 1.5f;
  settings.thrustEndHoldoffMs = 90;
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
  file.printf("scaleBackend=%u\n", settings.scaleBackend);
  file.printf("rtcEnabled=%d\n", settings.rtcEnabled ? 1 : 0);
  file.printf("rotate180=%u\n", settings.rotate180);
  file.printf("startThreshold=%.3f\n", settings.startThreshold);
  file.printf("preCaptureMs=%u\n", settings.preCaptureMs);
  file.printf("postCaptureMs=%u\n", settings.postCaptureMs);
  file.printf("waitForEjection=%d\n", settings.waitForEjection ? 1 : 0);
  file.printf("ejectionWaitSeconds=%.1f\n", settings.ejectionWaitSeconds);
  file.printf("ejectionDetectForceN=%.1f\n", settings.ejectionDetectForceN);
  file.printf("thrustEndHoldoffMs=%u\n", settings.thrustEndHoldoffMs);
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
    } else if (key == "scaleBackend") {
      settings.scaleBackend = (uint8_t)constrain(value.toInt(), 0, 2);
    } else if (key == "rtcEnabled") {
      settings.rtcEnabled = value.toInt() != 0;
    } else if (key == "rotate180") {
      settings.rotate180 = (uint8_t)constrain(value.toInt(), 0, 1);
    } else if (key == "startThreshold") {
      settings.startThreshold = value.toFloat();
    } else if (key == "preCaptureMs") {
      settings.preCaptureMs = value.toInt();
    } else if (key == "postCaptureMs") {
      settings.postCaptureMs = value.toInt();
    } else if (key == "waitForEjection") {
      settings.waitForEjection = value.toInt() != 0;
    } else if (key == "ejectionWaitSeconds") {
      float parsed = value.toFloat();
      settings.ejectionWaitSeconds = isfinite(parsed) ? constrain(parsed, 0.5f, 15.0f) : 10.0f;
    } else if (key == "ejectionDetectForceN") {
      float parsed = value.toFloat();
      settings.ejectionDetectForceN = isfinite(parsed) ? constrain(parsed, 0.2f, 30.0f) : 1.5f;
    } else if (key == "thrustEndHoldoffMs") {
      settings.thrustEndHoldoffMs = (uint16_t)constrain(value.toInt(), 20, 500);
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
  bool loadedFromSD = false;
  bool loaded = false;
  if (sdPresent) {
    loadedFromSD = loadSettingsFromSD();
    loaded = loadedFromSD;
  }
  if (!loaded) {
    loaded = loadSettingsFromEEPROM();
  }
  if (!loaded) {
    resetDefaultSettings();
  }
  if (settings.rotate180 > 1) {
    settings.rotate180 = 0;
  }
  if (settings.scaleBackend > SCALE_BACKEND_ADS1220) {
    settings.scaleBackend = SCALE_BACKEND_AUTO;
  }
  if (!isfinite(settings.ejectionWaitSeconds) || settings.ejectionWaitSeconds < 0.5f || settings.ejectionWaitSeconds > 15.0f) {
    settings.ejectionWaitSeconds = 10.0f;
  }
  if (!isfinite(settings.ejectionDetectForceN) || settings.ejectionDetectForceN < 0.2f || settings.ejectionDetectForceN > 30.0f) {
    settings.ejectionDetectForceN = 1.5f;
  }
  if (settings.thrustEndHoldoffMs < 20 || settings.thrustEndHoldoffMs > 500) {
    settings.thrustEndHoldoffMs = 90;
  }
  // If SD is available but settings.ini was missing/unreadable, seed it from the active settings.
  if (sdPresent && (!SD.exists(SETTINGS_FILENAME) || !loadedFromSD)) {
    saveSettingsToSD();
  }
  return true;
}

void applyScaleSettings() {
  // Use backend-specific calibration sanity ranges.
  float minCalAbs = MIN_REASONABLE_CAL_FACTOR_ABS_ADS1220;
  if (activeScaleBackend == SCALE_BACKEND_HX711) {
    minCalAbs = MIN_REASONABLE_CAL_FACTOR_ABS_HX711;
  }
  if (!isfinite(settings.calibrationFactor) || fabs(settings.calibrationFactor) < minCalAbs) {
    settings.calibrationFactor = DEFAULT_CAL_FACTOR;
  }
  if (activeScaleBackend == SCALE_BACKEND_HX711) {
    scale.set_scale(settings.calibrationFactor);
  }
}

long captureTimedBaselineRaw(unsigned long durationMs, unsigned long timeoutPerReadMs = 25) {
  long sum = 0;
  uint32_t got = 0;
  unsigned long startMs = millis();

  while ((millis() - startMs) < durationMs) {
    if (sensorReady()) {
      sum += sensorReadRaw();
      got++;
    } else {
      delay(1);
    }
  }

  // If DRDY readiness is unavailable, still gather a timed baseline on ADS.
  if (got == 0 && activeScaleBackend == SCALE_BACKEND_ADS1220) {
    unsigned long fallbackStartMs = millis();
    while ((millis() - fallbackStartMs) < durationMs) {
      sum += sensorReadRaw();
      got++;
      delay(2);
    }
  }

  if (got == 0) {
    return 0;
  }
  return sum / (long)got;
}

void scaleTareSensor() {
  if (activeScaleBackend == SCALE_BACKEND_HX711) {
    sensorOffsetRaw = captureTimedBaselineRaw(TARE_AVERAGE_DURATION_MS, 20);
    scale.set_offset(sensorOffsetRaw);
    return;
  }

  if (activeScaleBackend == SCALE_BACKEND_ADS1220) {
    sensorOffsetRaw = captureTimedBaselineRaw(TARE_AVERAGE_DURATION_MS, 20);
    return;
  }

  sensorOffsetRaw = captureTimedBaselineRaw(TARE_AVERAGE_DURATION_MS, 25);
}

long getSensorOffsetRaw() {
  return sensorOffsetRaw;
}

void resetAdsLiveDiagnostics() {
  adsMinRawSeen = adsLastRaw;
  adsMaxRawSeen = adsLastRaw;
  adsAbsDeltaEma = 0.0f;
  adsReadRateHzEma = 0.0f;
  adsSpikeCount = 0;
  adsClipCount = 0;
  adsAllZeroFrameCount = 0;
  adsAllOneFrameCount = 0;
  adsLastReadUs = micros();
}

void drawHomeScreen() {
  drawHeader("Rocket Stand");
  drawButton(16, 40, 140, 80, "Scale");
  drawButton(164, 40, 140, 80, "Capture\nThrust");
  drawButton(16, 136, 140, 80, "Saved\nCurves");
  drawButton(164, 136, 140, 80, "Settings");
  // drawFooter("Tap a button or open menu.", ILI9341_WHITE);
}

void drawScaleValue() {
  static float filteredGrams = 0.0f;
  static bool initialized = false;
  static unsigned long lastGraphMs = 0;

  float grams = netLoadKgFromUnits(readUnitsAverage(SCALE_UI_RAW_SAMPLES)) * 1000.0f;
  if (!initialized) {
    filteredGrams = grams;
    initialized = true;
  } else {
    filteredGrams = filteredGrams * 0.85f + grams * 0.15f;
  }

  scaleLastFilteredGrams = filteredGrams;

  // Keep mass readout clear region left of the Captures panel (x >= 214).
  tft.fillRect(20, 50, 188, 42, ILI9341_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(24, 60);
  tft.print(filteredGrams, 1);
  tft.print(" g");

  scaleTrendGrams[scaleTrendHead] = filteredGrams;
  scaleTrendHead = (scaleTrendHead + 1) % SCALE_TREND_POINTS;
  if (scaleTrendCount < SCALE_TREND_POINTS) {
    scaleTrendCount++;
  }
  scaleTrendSampleCounter++;

  unsigned long now = millis();
  if (now - lastGraphMs >= 220UL) {
    drawScaleTrendGraph();
    lastGraphMs = now;
  }
  if (scaleCaptureListDirty) {
    drawScaleCaptureList();
  }
  if (runtimeDebugFeaturesEnabled) {
    drawScaleDiagnostics();
  }
}

void emitScaleDiagnosticsSerial() {
  if (!runtimeDebugFeaturesEnabled) {
    return;
  }
  if (activeScaleBackend != SCALE_BACKEND_ADS1220) {
    return;
  }

  static unsigned long lastPrintMs = 0;
  static uint32_t lastReadCount = 0;
  static uint32_t lastReadyCount = 0;
  static uint32_t lastNotReadyCount = 0;
  static long lastRaw = 0;

  unsigned long now = millis();
  if (now - lastPrintMs < 1000UL) {
    return;
  }

  // Force one fresh sensor sample so telemetry works even off the Scale screen.
  float liveGrams = netLoadKgFromUnits(readUnitsAverage(1, 8)) * 1000.0f;
  scaleLastFilteredGrams = liveGrams;

  uint32_t reads = adsReadCount;
  uint32_t ready = adsReadyCount;
  uint32_t notReady = adsNotReadyCount;
  uint32_t spikes = adsSpikeCount;
  uint32_t clips = adsClipCount;
  uint32_t allZero = adsAllZeroFrameCount;
  uint32_t allOne = adsAllOneFrameCount;
  long raw = adsLastRaw;
  long minRaw = adsMinRawSeen;
  long maxRaw = adsMaxRawSeen;
  long span = maxRaw - minRaw;
  long dRaw = raw - lastRaw;
  uint32_t dReads = reads - lastReadCount;
  uint32_t dReady = ready - lastReadyCount;
  uint32_t dNotReady = notReady - lastNotReadyCount;
  uint32_t readyChecks = dReady + dNotReady;
  uint32_t readyPct = readyChecks > 0 ? (dReady * 100UL) / readyChecks : 0;

  Serial.print("SCALE_DIAG g=");
  Serial.print(liveGrams, 1);
  Serial.print(" raw=");
  Serial.print(raw);
  Serial.print(" dRaw=");
  Serial.print(dRaw);
  Serial.print(" span=");
  Serial.print(span);
  Serial.print(" nse=");
  Serial.print(adsAbsDeltaEma, 1);
  Serial.print(" rps=");
  Serial.print(dReads);
  Serial.print(" hz=");
  Serial.print(adsReadRateHzEma, 1);
  Serial.print(" ready=");
  Serial.print(readyPct);
  Serial.print(" drdy=");
  Serial.print(digitalRead(ADS1220_DRDY) == LOW ? "L" : "H");
  Serial.print(" spk=");
  Serial.print(spikes);
  Serial.print(" clip=");
  Serial.print(clips);
  Serial.print(" zf=");
  Serial.print(allZero);
  Serial.print(" of=");
  Serial.print(allOne);
  Serial.print(" cfg0=0x");
  Serial.println(adsReg0Active, HEX);

  lastPrintMs = now;
  lastReadCount = reads;
  lastReadyCount = ready;
  lastNotReadyCount = notReady;
  lastRaw = raw;
}

void drawScaleDiagnostics() {
  // Keep diagnostics to the left side so it never overwrites the Captures panel.
  tft.fillRect(16, 26, 194, 24, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_YELLOW);

  if (activeScaleBackend != SCALE_BACKEND_ADS1220) {
    tft.setCursor(16, 30);
    tft.print("Backend: ");
    tft.print(scaleBackendLabel(activeScaleBackend));
    return;
  }

  static unsigned long lastDiagMs = 0;
  static uint32_t lastReadCount = 0;
  static uint32_t lastReadyCount = 0;
  static uint32_t lastNotReadyCount = 0;
  static long lastRaw = 0;

  unsigned long now = millis();
  unsigned long dt = (lastDiagMs == 0) ? 1 : max(1UL, now - lastDiagMs);
  uint32_t reads = adsReadCount;
  uint32_t ready = adsReadyCount;
  uint32_t notReady = adsNotReadyCount;
  long raw = adsLastRaw;
  long minRaw = adsMinRawSeen;
  long maxRaw = adsMaxRawSeen;
  float noise = adsAbsDeltaEma;
  float readRateHz = adsReadRateHzEma;
  uint32_t spikes = adsSpikeCount;
  uint32_t clips = adsClipCount;

  uint32_t dReads = reads - lastReadCount;
  uint32_t dReady = ready - lastReadyCount;
  uint32_t dNotReady = notReady - lastNotReadyCount;
  long dRaw = raw - lastRaw;
  uint32_t readsPerSec = (dReads * 1000UL) / dt;
  uint32_t totalReadyChecks = dReady + dNotReady;
  uint32_t readyPct = (totalReadyChecks > 0) ? (dReady * 100UL) / totalReadyChecks : 0;
  long span = maxRaw - minRaw;
  bool signalLive = (span > 2000L) || (noise > 120.0f);

  tft.setCursor(16, 26);
  tft.print("RAW:");
  tft.print(raw);
  tft.print(" d:");
  tft.print(dRaw);
  tft.print(" r/s:");
  tft.print(readsPerSec);
  tft.print("~");
  tft.print((int)readRateHz);

  tft.setCursor(16, 34);
  tft.print("Span:");
  tft.print(span);
  tft.print(" Nse:");
  tft.print(noise, 0);
  tft.print(" Spk:");
  tft.print(spikes);
  tft.print(" Clp:");
  tft.print(clips);

  tft.setCursor(16, 42);
  tft.print("DRDY:");
  tft.print(digitalRead(ADS1220_DRDY) == LOW ? "L" : "H");
  tft.print(" Ready:");
  tft.print(readyPct);
  tft.print("% Sig:");
  tft.print(signalLive ? "LIVE" : "FLAT");
  tft.print(" Cfg0:0x");
  tft.print(adsReg0Active, HEX);

  lastDiagMs = now;
  lastReadCount = reads;
  lastReadyCount = ready;
  lastNotReadyCount = notReady;
  lastRaw = raw;
}

void drawScaleTrendGraph() {
  const int gx = 16;
  const int gy = 102;
  const int gw = 288;
  const int gh = 72;

  tft.fillRect(gx, gy, gw, gh, ILI9341_BLACK);
  tft.drawRect(gx, gy, gw, gh, ILI9341_WHITE);

  if (scaleTrendCount < 2) {
    return;
  }

  float minG = 1e9f;
  float maxG = -1e9f;
  for (int i = 0; i < scaleTrendCount; i++) {
    int idx = (scaleTrendHead - scaleTrendCount + i + SCALE_TREND_POINTS) % SCALE_TREND_POINTS;
    float g = scaleTrendGrams[idx];
    minG = min(minG, g);
    maxG = max(maxG, g);
  }

  if ((maxG - minG) < 10.0f) {
    float center = (maxG + minG) * 0.5f;
    minG = center - 5.0f;
    maxG = center + 5.0f;
  }

  if (minG < 0.0f && maxG > 0.0f) {
    int y0 = gy + gh - 1 - (int)((0.0f - minG) * (gh - 1) / (maxG - minG));
    tft.drawFastHLine(gx + 1, y0, gw - 2, ILI9341_DARKGREY);
  }

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_DARKGREY);
  tft.setCursor(gx + 2, gy + 2);
  tft.print(maxG, 0);
  tft.setCursor(gx + 2, gy + gh - 10);
  tft.print(minG, 0);

  int prevX = gx;
  int prevY = gy + gh / 2;
  for (int i = 0; i < scaleTrendCount; i++) {
    int idx = (scaleTrendHead - scaleTrendCount + i + SCALE_TREND_POINTS) % SCALE_TREND_POINTS;
    float g = scaleTrendGrams[idx];
    int x = gx + (int)((long)i * (gw - 1) / max(1, scaleTrendCount - 1));
    int y = gy + gh - 1 - (int)((g - minG) * (gh - 1) / (maxG - minG));
    if (i > 0) {
      tft.drawLine(prevX, prevY, x, y, ILI9341_YELLOW);
    }
    prevX = x;
    prevY = y;
  }

  if (scaleCaptureMarkerCount > 0 && scaleTrendCount > 1) {
    uint32_t startSample = (scaleTrendSampleCounter >= (uint32_t)scaleTrendCount)
      ? (scaleTrendSampleCounter - (uint32_t)scaleTrendCount)
      : 0;
    for (int i = 0; i < scaleCaptureMarkerCount; i++) {
      uint32_t s = scaleCaptureMarkerSample[i];
      if (s < startSample || s >= scaleTrendSampleCounter) continue;
      uint32_t rel = s - startSample;
      int x = gx + (int)((uint64_t)rel * (uint64_t)(gw - 1) / (uint64_t)max(1, scaleTrendCount - 1));
      tft.drawFastVLine(x, gy + 1, gh - 2, ILI9341_RED);
    }
  }
}

void drawScaleScreen() {
  drawHeader("Scale");
  drawHamburger();
  scaleTrendHead = 0;
  scaleTrendCount = 0;
  scaleTrendSampleCounter = 0;
  scaleCaptureMarkerCount = 0;
  scaleCaptureMarkerNextId = 1;
  scaleCaptureListDirty = true;
  scaleLastFilteredGrams = 0.0f;
  tft.fillRect(20, 50, tft.width() - 40, 42, ILI9341_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(24, 60);
  tft.print("0.0 g");
  drawScaleTrendGraph();
  drawScaleCaptureList();
  drawSmallButton(12, 180, 140, 44, "Tare");
  drawSmallButton(168, 180, 140, 44, "Capture", ILI9341_GREEN);
  drawFooter("Taring... hold still");
  scaleTareSensor();
  offsetTare = readUnitsAverage(20);
  drawFooter("Tare complete. Capture marks graph.");
}

void drawCaptureScreen() {
  drawHeader("Capture");
  drawHamburger();
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(16, 48);
  tft.print("Start threshold: ");
  tft.print(settings.startThreshold, 2);
  tft.print(" N");
  tft.setCursor(16, 76);
  tft.print("Post capture: ");
  tft.print((float)settings.postCaptureMs / 1000.0f, 1);
  tft.print(" s");
  tft.setCursor(16, 104);
  tft.print("Eject wait: ");
  tft.print(settings.ejectionWaitSeconds, 1);
  tft.print(" s");
  tft.setCursor(16, 132);
  tft.print("Eject detect: ");
  tft.print(settings.ejectionDetectForceN, 1);
  tft.print(" N");
  drawButton(90, 150, 140, 64, "Start");
  drawFooter("Main thrust metrics ignore ejection spike.");
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
    float graphTopN = ((float)(((int)ceil(maxForce / ((maxForce > 40.0f) ? 10.0f : 5.0f)))) * ((maxForce > 40.0f) ? 10.0f : 5.0f));
    float xScale = duration > 0 ? (float)graphW / duration : 1.0f;
    float yScale = graphH / max(5.0f, graphTopN);
    drawGraphYAxisLabels(left, top, graphW, graphH, maxForce);
    for (int i = 1; i < sampleCount; i++) {
      int x1 = left + (int)(samples[i - 1].timeMs * xScale);
      int y1 = top + graphH - (int)(max(0.0f, samples[i - 1].forceN) * yScale);
      int x2 = left + (int)(samples[i].timeMs * xScale);
      int y2 = top + graphH - (int)(max(0.0f, samples[i].forceN) * yScale);
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
  drawButton(164, 112, 140, 64, "Params");
  drawFooter("Open Params for capture + rotate.");
}

void drawParametersScreen() {
  drawHeader("Parameters");
  drawHamburger();
  tft.setTextColor(ILI9341_WHITE);

  bool rot180 = settings.rotate180 == 1;
  tft.setTextSize(2);
  tft.setCursor(16, 42);
  tft.print("Rotate 180");
  tft.drawRect(210, 40, 22, 22, ILI9341_WHITE);
  if (rot180) {
    tft.fillRect(214, 44, 14, 14, ILI9341_GREEN);
  }

  tft.setCursor(16, 70);
  tft.print("Start Threshold (N):");
  tft.fillRoundRect(246, 64, 58, 20, 4, ILI9341_DARKGREY);
  tft.drawRoundRect(246, 64, 58, 20, 4, ILI9341_WHITE);
  tft.setCursor(250, 66);
  tft.print(settings.startThreshold, 1);

  tft.setCursor(16, 92);
  tft.print("Pre Capture (S):");
  tft.fillRoundRect(220, 86, 84, 20, 4, ILI9341_DARKGREY);
  tft.drawRoundRect(220, 86, 84, 20, 4, ILI9341_WHITE);
  tft.setCursor(226, 88);
  tft.print((float)settings.preCaptureMs / 1000.0f, 1);

  tft.setCursor(16, 114);
  tft.print("Post Capture (S):");
  tft.fillRoundRect(220, 108, 84, 20, 4, ILI9341_DARKGREY);
  tft.drawRoundRect(220, 108, 84, 20, 4, ILI9341_WHITE);
  tft.setCursor(226, 110);
  tft.print((float)settings.postCaptureMs / 1000.0f, 1);

  tft.setCursor(16, 136);
  tft.print("End Holdoff (ms):");
  tft.fillRoundRect(220, 130, 84, 20, 4, ILI9341_DARKGREY);
  tft.drawRoundRect(220, 130, 84, 20, 4, ILI9341_WHITE);
  tft.setCursor(226, 132);
  tft.print(settings.thrustEndHoldoffMs);

  tft.setCursor(16, 158);
  tft.print("Eject wait (S):");
  tft.fillRoundRect(220, 152, 84, 20, 4, ILI9341_DARKGREY);
  tft.drawRoundRect(220, 152, 84, 20, 4, ILI9341_WHITE);
  tft.setCursor(226, 154);
  tft.print(settings.ejectionWaitSeconds, 1);

  tft.setCursor(16, 180);
  tft.print("Eject Detect (N):");
  tft.fillRoundRect(220, 174, 84, 20, 4, ILI9341_DARKGREY);
  tft.drawRoundRect(220, 174, 84, 20, 4, ILI9341_WHITE);
  tft.setCursor(226, 176);
  tft.print(settings.ejectionDetectForceN, 1);

  tft.setCursor(16, 202);
  tft.print("Debug UI/Diag:");
  tft.drawRect(210, 200, 22, 22, ILI9341_WHITE);
  if (runtimeDebugFeaturesEnabled) {
    tft.fillRect(214, 204, 14, 14, ILI9341_GREEN);
  }

  drawFooter("Debug toggle is runtime only.");
}

void drawCaptureResultScreen() {
  drawHeader("Capture Result");
  drawHamburger();
  int left = 10;
  int top = 34;
  int graphW = tft.width() - 20;
  int graphH = 136;

  if (captureResultShowGraph) {
    tft.drawRect(8, 32, tft.width() - 16, 140, ILI9341_WHITE);
    float maxForce = 0.001f;
    for (int i = 0; i < sampleCount; i++) {
      maxForce = max(maxForce, samples[i].forceN);
    }
    float graphTopN = ((float)(((int)ceil(maxForce / ((maxForce > 40.0f) ? 10.0f : 5.0f)))) * ((maxForce > 40.0f) ? 10.0f : 5.0f));
    float xScale = (sampleCount > 1) ? (float)graphW / max(1.0f, (float)samples[sampleCount - 1].timeMs) : 1.0f;
    float yScale = graphH / max(5.0f, graphTopN);
    drawGraphYAxisLabels(left, top, graphW, graphH, maxForce);
    for (int i = 1; i < sampleCount; i++) {
      int x1 = left + (int)(samples[i - 1].timeMs * xScale);
      int y1 = top + graphH - (int)(max(0.0f, samples[i - 1].forceN) * yScale);
      int x2 = left + (int)(samples[i].timeMs * xScale);
      int y2 = top + graphH - (int)(max(0.0f, samples[i].forceN) * yScale);
      tft.drawLine(x1, y1, x2, y2, ILI9341_YELLOW);
    }

    int xStart = left + (int)(captureResultThrustStartMs * xScale);
    int xCutoff = left + (int)(captureResultThrustEndMs * xScale);
    xStart = constrain(xStart, left, left + graphW - 1);
    xCutoff = constrain(xCutoff, left, left + graphW - 1);
    tft.drawFastVLine(xStart, top, graphH, ILI9341_GREEN);
    tft.drawFastVLine(xCutoff, top, graphH, ILI9341_RED);
    if (captureResultEjectDetected) {
      int xEject = left + (int)(captureResultEjectMs * xScale);
      xEject = constrain(xEject, left, left + graphW - 1);
      tft.drawFastVLine(xEject, top, graphH, ILI9341_BLUE);
    }
  } else {
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(16, 40);
    tft.print("Class: ");
    tft.print(captureResultRating);
    tft.setCursor(16, 64);
    tft.print("Impulse: ");
    tft.print(captureResultImpulse, 2);
    tft.print(" Ns");
    tft.setCursor(16, 88);
    tft.print("Avg: ");
    tft.print(captureResultAvgThrust, 2);
    tft.print(" N");
    tft.setCursor(16, 112);
    tft.print("Burn: ");
    tft.print(captureResultBurnTime, 2);
    tft.print(" s");
    tft.setCursor(16, 136);
    tft.print("Peak: ");
    tft.print(captureResultMaxForce, 2);
    tft.print(" N");
    tft.setCursor(16, 160);
    tft.print("Delay: ");
    tft.print(captureResultDelay, 2);
    tft.print(" s");
  }

  drawSmallButton(12, 178, 92, 34, "Graph", captureResultShowGraph ? ILI9341_GREEN : ILI9341_BLUE);
  drawSmallButton(114, 178, 92, 34, "Stats", captureResultShowGraph ? ILI9341_BLUE : ILI9341_GREEN);
  drawSmallButton(216, 178, 92, 34, "Home", ILI9341_RED);
  drawFooter(captureResultSaveStatus.c_str(), captureResultSaveStatus.startsWith("Saved ") ? ILI9341_GREEN : ILI9341_WHITE);
}

void drawCaptureCharacterization(float totalImpulse, float averageThrust, float burnTime, float maxForceMain, float delaySecs, const String& rating, const String& saveStatus) {
  drawHeader("Characterization");
  drawHamburger();
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);

  tft.setCursor(16, 40);
  tft.print("Class: ");
  tft.print(rating);

  tft.setCursor(16, 64);
  tft.print("Impulse: ");
  tft.print(totalImpulse, 2);
  tft.print(" Ns");

  tft.setCursor(16, 88);
  tft.print("Avg: ");
  tft.print(averageThrust, 2);
  tft.print(" N");

  tft.setCursor(16, 112);
  tft.print("Burn: ");
  tft.print(burnTime, 2);
  tft.print(" s");

  tft.setCursor(16, 136);
  tft.print("Peak: ");
  tft.print(maxForceMain, 2);
  tft.print(" N");

  tft.setCursor(16, 160);
  tft.print("Delay: ");
  tft.print(delaySecs, 2);
  tft.print(" s");

  drawFooter(saveStatus.c_str(), saveStatus.startsWith("Saved ") ? ILI9341_GREEN : ILI9341_WHITE);
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
  drawFooter("RTC only.");
}

void drawCalibrationScreen(float measuredMass) {
  drawHeader("Calibration");
  drawHamburger();
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(16, 40);
  tft.print("Factor: ");
  tft.print(settings.calibrationFactor, 1);

  tft.setCursor(16, 68);
  tft.print("Mass: ");
  tft.print(measuredMass, 1);
  tft.print(" g");

  tft.setCursor(16, 94);
  tft.print("Target: ");
  tft.print(weightEntry);
  tft.print(" g");

  // Compact non-overlapping controls that fit fully above footer.
  drawSmallButton(16, 122, 92, 34, "Tare");
  drawSmallButton(16, 164, 140, 34, "Weight");
  drawSmallButton(164, 164, 140, 34, "Calibrate");

  drawFooter("Live mass. Set target, then Calibrate.");
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
      if (sampleCount < CAPTURE_MAX_SAMPLES) {
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
  if (password) {
    tft.print(String(value.length()));
  } else {
    tft.print(value);
  }

  int top = password ? 58 : 66;
  int rowHeight = password ? 28 : 46;
  for (int r = 0; r < rows; r++) {
    const char* row = keys[r];
    int cols = strlen(row);
    int cellW = (tft.width() - 32) / cols;
    for (int c = 0; c < cols; c++) {
      int x = 16 + c * cellW;
      int y = top + r * rowHeight;
      String label = String((char)row[c]);
      drawSmallButton(x, y, cellW - 4, rowHeight - 4, label, ILI9341_BLUE);
    }
  }

  int controlY = top + rows * rowHeight + 4;
  drawSmallButton(8, controlY, 96, 34, showShift ? (shiftActive ? "SHIFT" : "shift") : "DEL");
  drawSmallButton(112, controlY, 96, 34, "Back", ILI9341_RED);
  drawSmallButton(216, controlY, 96, 34, "OK", ILI9341_GREEN);
}

String inputTextKeyboard(const String& title, const String& initial, bool passwordMode, bool allowDot) {
  String value = initial;
  bool shiftActive = false;
  const char* alphaKeys[] = {"QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM", "1234567890", "@.-_/"};
  const char* numberKeys[] = {"1234567", "89.0"};
  bool needsRedraw = true;
  while (true) {
    int rows = passwordMode ? 5 : 2;
    int top = passwordMode ? 58 : 66;
    int rowHeight = passwordMode ? 28 : 46;
    int controlY = top + rows * rowHeight + 4;

    if (needsRedraw) {
      if (passwordMode) {
        drawKeyboard(title, value, true, alphaKeys, rows, true, shiftActive);
      } else {
        drawKeyboard(title, value, false, numberKeys, rows, false, false);
      }
      needsRedraw = false;
    }

    TouchPoint p;
    if (!getTouchedPoint(p)) {
      delay(20);
      continue;
    }

    if (hitTest(p, 8, controlY, 96, 34)) {
      if (passwordMode) {
        shiftActive = !shiftActive;
      } else {
        if (!value.isEmpty()) value.remove(value.length() - 1);
      }
      needsRedraw = true;
      continue;
    }

    if (hitTest(p, 112, controlY, 96, 34)) {
      return initial;
    }

    if (hitTest(p, 216, controlY, 96, 34)) {
      return value;
    }

    bool changed = false;
    for (int r = 0; r < rows; r++) {
      const char* row = passwordMode ? alphaKeys[r] : numberKeys[r];
      int cols = strlen(row);
      int cellW = (tft.width() - 32) / cols;
      for (int c = 0; c < cols; c++) {
        int x = 16 + c * cellW;
        int y = top + r * rowHeight;
        if (hitTest(p, x, y, cellW - 4, rowHeight - 4)) {
          char ch = row[c];
          if (passwordMode && isalpha(ch)) {
            if (!shiftActive) ch = tolower(ch);
          }
          if (!passwordMode && ch == '.') {
            if (allowDot && value.indexOf('.') < 0) {
              value += '.';
              changed = true;
            }
          } else {
            value += ch;
            changed = true;
          }
          break;
        }
      }
      if (changed) break;
    }
    if (changed) {
      needsRedraw = true;
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

// This CYD panel is natively landscape (320x240). Adafruit setRotation(1) sends
// MADCTL MV=1 which swaps row/col and forces hardware portrait (240x320) -- wrong.
// We always call setRotation(1) for correct 320x240 software dimensions, then
// override MADCTL to keep MV=0 (native landscape). The Rotate button cycles the
// 4 landscape MADCTL variants so the user can find the correct physical orientation.
static const uint8_t cydMadctl[4] = {
  0x08,  // option 0: landscape, no flip      (BGR)
  0x48,  // option 1: landscape, MX flip      (MX|BGR)      <- expected correct for CYD
  0x88,  // option 2: landscape, MY flip      (MY|BGR)      <- 180 partner for option 1
  0xC8,  // option 3: landscape, MX+MY flip   (MX|MY|BGR)   <- mirror mode (unused)
};
void applyRotation(uint8_t rot) {
  rot &= 0x03;
  tft.setRotation(1);  // Always landscape: width=320, height=240
  tft.sendCommand(0x36, &cydMadctl[rot], 1);  // 0x36 = ILI9341_MADCTL
}

void setup() {
  Serial.begin(115200);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  // Keep other SPI devices deselected while initializing TFT.
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

  sdPresent = SD.begin(SD_CS);
  if (!sdPresent) {
    Serial.println("SD init failed, using EEPROM fallback.");
  }
  EEPROM.begin(EEPROM_SIZE);
  loadSettings();
  displayRotation = settings.rotate180 ? 2 : 1;
  applyRotation(displayRotation);

  Wire.begin(I2C_SDA, I2C_SCL);
  rtcAvailable = rtc.begin();

  bool scaleReady = initScale();
  if (!scaleReady) {
    statusLine = "Scale ADC not detected";
    drawMessage(statusLine);
    delay(500);
  }

  // create mutex and start sampler task
  samplesMutex = xSemaphoreCreateMutex();
  if (samplesMutex == NULL) {
    Serial.println("Failed to create samples mutex");
  }
  xTaskCreatePinnedToCore(samplerTask, "sampler", 4096, NULL, 2, &samplerTaskHandle, 1);

  drawHomeScreen();
}

void runCaptureRoutine() {
  drawHeader("Capturing...");
  drawHamburger();
  tft.drawRect(8, 32, tft.width() - 16, 160, ILI9341_WHITE);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(16, 208);
  tft.print("Waiting for thrust...");

  // Start capture from a fresh tare so static motor mass is removed from thresholding.
  scaleTareSensor();
  offsetTare = readUnitsAverage(20);

  sampleCount = 0;
  const unsigned long sampleIntervalUs = captureSampleIntervalUs();
  const unsigned long sampleIntervalMs = captureSampleIntervalMsRoundedUp();
  int preCapSamples = (int)((settings.preCaptureMs + sampleIntervalMs - 1) / sampleIntervalMs);
  preCapSamples = constrain(preCapSamples, 8, CAPTURE_MAX_SAMPLES / 2);

  enum CapturePhase { WAIT_THRUST, MAIN_THRUST, EJECTION_WAIT };
  CapturePhase phase = WAIT_THRUST;

  int preCount = 0;
  int preHead = 0;
  uint8_t startAboveCount = 0;
  unsigned long lastSampleUs = micros();
  unsigned long captureBaseAbsMs = millis();
  unsigned long captureStart = 0;
  unsigned long lastAbove = 0;
  unsigned long belowSinceMs = 0;
  unsigned long thrustQuietSinceMs = 0;
  unsigned long lastGraphRefreshMs = 0;
  unsigned long mainEndMs = 0;
  unsigned long mainEndAbsMs = 0;
  float maxForceMain = 0.0f;
  float maxForceAll = 0.0f;
  bool foundEject = false;
  unsigned long ejectTimeMs = 0;
  unsigned long ejectionDeadlineMs = 0;
  unsigned long quietSinceMs = 0;
  bool reachedMaxRuntime = false;
  float minAfterMain = 0.0f;
  bool ejectionArmed = false;

  bool baselineInitialized = false;
  float baseline = 0.0f;
  float baselineNoise = 0.1f;
  float filteredForce = 0.0f;
  float startDelta = settings.startThreshold;
  float endDelta = max(0.35f, settings.startThreshold * 0.45f);
  // Rate-adaptive filter: maintain ~20ms smoothing time constant at any sample rate.
  const float filterAlpha = expf(-(float)captureSampleIntervalUs() / 20000.0f);
  unsigned long waitStartMs = millis();
  unsigned long routineStartMs = waitStartMs;
  unsigned long lastSampleMs = waitStartMs;
  unsigned long maxCaptureRuntimeMs =
    (unsigned long)settings.preCaptureMs +
    (unsigned long)MAX_STORED_BURN_MS +
    (unsigned long)settings.postCaptureMs +
    (unsigned long)(settings.ejectionWaitSeconds * 1000.0f)
    + 2000UL;  // extra buffer to account for late ejection detection and post-capture tail

  while (true) {
    TouchPoint capTouch;
    if (getTouchedPoint(capTouch) && hitTest(capTouch, 0, 0, 28, 24)) {
      currentScreen = SCREEN_CAPTURE;
      drawCaptureScreen();
      drawMessage("Capture canceled.");
      return;
    }

    unsigned long nowUs = micros();
    if (nowUs - lastSampleUs < sampleIntervalUs) continue;
    lastSampleUs += sampleIntervalUs;
    if (!sensorReady()) {
      // ADS can occasionally miss DRDY transitions; fail safe instead of hanging forever.
      if (activeScaleBackend == SCALE_BACKEND_ADS1220 && (millis() - lastSampleMs) > 250UL) {
        // Fall through by forcing one best-effort sample read.
      } else {
        if ((millis() - lastSampleMs) > 2500UL) {
          currentScreen = SCREEN_CAPTURE;
          drawCaptureScreen();
          drawMessage("Capture aborted: sensor stalled.");
          return;
        }
        continue;
      }
    }
    long raw = sensorReadRaw();
    float unit = rawToUnits(raw);
    float f = netLoadKgFromUnits(unit) * 9.80665f;
    unsigned long nowMs = millis();
    lastSampleMs = nowMs;

    if ((nowMs - routineStartMs) >= maxCaptureRuntimeMs) {
      reachedMaxRuntime = true;
      if (phase == WAIT_THRUST) {
        break;
      }
      break;
    }

    if (!baselineInitialized) {
      baseline = f;
      filteredForce = f;
      baselineInitialized = true;
    } else {
      filteredForce = filteredForce * filterAlpha + f * (1.0f - filterAlpha);
    }

    if (phase == WAIT_THRUST) {
      // Keep a rolling pre-capture ring so burn onset is preserved.
      samples[preHead] = {nowMs, f};
      preHead = (preHead + 1) % preCapSamples;
      if (preCount < preCapSamples) preCount++;

      float err = filteredForce - baseline;
      float baselineTrack = (nowMs - waitStartMs < 700UL) ? 0.02f : 0.004f;
      baseline += err * baselineTrack;
      baselineNoise = baselineNoise * 0.97f + fabsf(err) * 0.03f;
      startDelta = max(settings.startThreshold, baselineNoise * 4.0f + 0.25f);
      endDelta = max(0.30f, startDelta * 0.40f);

      bool aboveDynamic = fabsf(filteredForce - baseline) >= startDelta;
      bool aboveAbsolute = fabsf(filteredForce) >= settings.startThreshold;
      bool aboveRawAbsolute = fabsf(f) >= settings.startThreshold;
      if (aboveDynamic || aboveAbsolute || aboveRawAbsolute) {
        if (startAboveCount < 255) startAboveCount++;
      } else {
        startAboveCount = 0;
      }
      if (startAboveCount >= 3) {
        // Linearize rolling pre-capture ring into samples[0..preCount-1].
        for (int i = 0; i < preCount; i++) {
          int idx = (preHead - preCount + i + preCapSamples) % preCapSamples;
          captureCopy[i] = samples[idx];
        }
        captureBaseAbsMs = (preCount > 0) ? captureCopy[0].timeMs : nowMs;
        for (int i = 0; i < preCount; i++) {
          samples[i].timeMs = captureCopy[i].timeMs - captureBaseAbsMs;
          samples[i].forceN = captureCopy[i].forceN;
        }
        sampleCount = preCount;

        phase = MAIN_THRUST;
        captureStart = nowMs;
        lastAbove = nowMs;
        belowSinceMs = 0;
        thrustQuietSinceMs = 0;
        mainEndAbsMs = nowMs;
        mainEndMs = nowMs - captureBaseAbsMs;
        tft.fillRect(16, 206, 300, 16, ILI9341_BLACK);
        tft.setCursor(16, 208);
        tft.print("Capturing main thrust...");
      }

      if (nowMs - waitStartMs >= 20000UL) {
        currentScreen = SCREEN_CAPTURE;
        drawCaptureScreen();
        drawMessage("No thrust detected. Check threshold/wiring.");
        return;
      }
    }

    if (phase != WAIT_THRUST) {
      unsigned long sampleTime = nowMs - captureBaseAbsMs;
      bool shouldStoreSample = false;
      if (phase == MAIN_THRUST) {
        unsigned long storedBurnMs = (nowMs >= captureStart) ? (nowMs - captureStart) : 0;
        shouldStoreSample = storedBurnMs <= MAX_STORED_BURN_MS;
      }
      if (shouldStoreSample && sampleCount < CAPTURE_MAX_SAMPLES) {
        samples[sampleCount++] = {sampleTime, f};
      }
      if (f > maxForceAll) maxForceAll = f;

      if (phase == MAIN_THRUST) {
        if (f > maxForceMain) maxForceMain = f;
        // Mirror the post-processing algorithm exactly: raw f vs baseline+startDelta.
        // That algorithm reliably finds end-of-thrust on stored data; using it live
        // gives identical detection with no filter-lag or threshold-mismatch issues.
        bool aboveEot = fabsf(f) >= (baseline + startDelta);

        if (aboveEot) {
          belowSinceMs = 0;
          mainEndAbsMs = nowMs;
          mainEndMs = sampleTime;
        } else if (belowSinceMs == 0) {
          belowSinceMs = nowMs;
        }

        if (belowSinceMs != 0 && (nowMs - belowSinceMs) >= settings.thrustEndHoldoffMs) {
          mainEndAbsMs = belowSinceMs;
          mainEndMs = mainEndAbsMs - captureBaseAbsMs;
          if (settings.waitForEjection) {
            phase = EJECTION_WAIT;
            ejectionDeadlineMs = nowMs + (unsigned long)(settings.ejectionWaitSeconds * 1000.0f);
            minAfterMain = filteredForce;
            ejectionArmed = false;
            tft.fillRect(16, 206, 300, 16, ILI9341_BLACK);
            tft.setCursor(16, 208);
            tft.print("Main done, waiting ejection...");
          } else {
            break;
          }
        }
        // If detection never fired, force transition at MAX_STORED_BURN_MS.
        // Guard with phase check so this doesn't run after holdoff already transitioned.
        unsigned long burnElapsedMs = (nowMs >= captureStart) ? (nowMs - captureStart) : 0;
        if (phase == MAIN_THRUST && burnElapsedMs >= (unsigned long)MAX_STORED_BURN_MS) {
          mainEndMs = sampleTime;
          mainEndAbsMs = nowMs;
          if (settings.waitForEjection) {
            phase = EJECTION_WAIT;
            ejectionDeadlineMs = nowMs + (unsigned long)(settings.ejectionWaitSeconds * 1000.0f);
            minAfterMain = filteredForce;
            ejectionArmed = false;
            tft.fillRect(16, 206, 300, 16, ILI9341_BLACK);
            tft.setCursor(16, 208);
            tft.print("Max burn time, waiting ejection...");
          } else {
            break;
          }
        }
      } else if (phase == EJECTION_WAIT) {
        minAfterMain = min(minAfterMain, filteredForce);
        float armLevel = baseline + max(0.15f, endDelta * 0.8f);
        if (!ejectionArmed && filteredForce <= armLevel) {
          ejectionArmed = true;
        }
        if (!ejectionArmed && (nowMs - mainEndAbsMs) >= 250UL) {
          ejectionArmed = true;
        }

        float riseFromValley = filteredForce - minAfterMain;
        if (!foundEject && ejectionArmed && riseFromValley >= settings.ejectionDetectForceN) {
          foundEject = true;
          ejectTimeMs = sampleTime;
          unsigned long settleDeadline = nowMs + (unsigned long)settings.postCaptureMs;
          if (settleDeadline < ejectionDeadlineMs) {
            ejectionDeadlineMs = settleDeadline;
          }
        }

        // Wait the full ejectionWaitSeconds for the ejection charge.
        // ejectionDeadlineMs is shortened to now+postCaptureMs once ejection is detected.
        if (nowMs >= ejectionDeadlineMs) {
          break;
        }
      }

      if (sampleCount > 1 && (nowMs - lastGraphRefreshMs) >= 140UL) {
        lastGraphRefreshMs = nowMs;
        int left = 10;
        int top = 34;
        int graphW = tft.width() - 20;
        int graphH = 156;
        float maxF = 0.001f;
        for (int i = 0; i < sampleCount; i++) {
          maxF = max(maxF, samples[i].forceN);
        }
        float graphTopN = ((float)(((int)ceil(maxF / ((maxF > 40.0f) ? 10.0f : 5.0f)))) * ((maxF > 40.0f) ? 10.0f : 5.0f));
        float xScale = (float)graphW / max(1.0f, (float)samples[sampleCount - 1].timeMs);
        float yScale = graphH / max(5.0f, graphTopN);
        tft.fillRect(left, top, graphW, graphH, ILI9341_BLACK);
        drawGraphYAxisLabels(left, top, graphW, graphH, maxF);
        for (int i = 1; i < sampleCount; i++) {
          int x1 = left + (int)(samples[i - 1].timeMs * xScale);
          int y1 = top + graphH - (int)(max(0.0f, samples[i - 1].forceN) * yScale);
          int x2 = left + (int)(samples[i].timeMs * xScale);
          int y2 = top + graphH - (int)(max(0.0f, samples[i].forceN) * yScale);
          tft.drawLine(x1, y1, x2, y2, ILI9341_YELLOW);
        }
      }
    }
  }

  if (mainEndMs == 0 && sampleCount > 0) {
    mainEndMs = samples[sampleCount - 1].timeMs;
  }

  // Stats should use only the threshold-defined thrust window (exclude pre-capture).
  float thrustThreshold = baseline + startDelta;
  unsigned long thrustStartMs = 0;
  unsigned long thrustEndMs = mainEndMs;
  bool thrustWindowFound = false;
  unsigned long belowCandidateMs = 0;
  const unsigned long thrustEndHoldoffMs = (unsigned long)settings.thrustEndHoldoffMs;
  for (int i = 0; i < sampleCount; i++) {
    unsigned long t = samples[i].timeMs;
    bool above = samples[i].forceN >= thrustThreshold;
    if (!thrustWindowFound) {
      if (above) {
        thrustWindowFound = true;
        thrustStartMs = t;
        thrustEndMs = t;
      }
      continue;
    }
    if (above) {
      thrustEndMs = t;
      belowCandidateMs = 0;
    } else if (belowCandidateMs == 0) {
      belowCandidateMs = t;
    } else if ((t - belowCandidateMs) >= thrustEndHoldoffMs) {
      thrustEndMs = belowCandidateMs;
      break;
    }
  }
  if (!thrustWindowFound) {
    thrustStartMs = 0;
    thrustEndMs = mainEndMs;
  }

  float mainImpulse = 0.0f;
  float windowDurationSec = 0.0f;
  for (int i = 1; i < sampleCount; i++) {
    unsigned long t1 = samples[i - 1].timeMs;
    unsigned long t2 = samples[i].timeMs;
    if (t2 <= thrustStartMs) continue;
    if (t1 >= thrustEndMs) break;
    float dt = (t2 - t1) / 1000.0f;
    float f1 = max(0.0f, samples[i - 1].forceN);
    float f2 = max(0.0f, samples[i].forceN);
    mainImpulse += 0.5f * (f1 + f2) * dt;
    windowDurationSec += dt;
  }

  float averageThrust = (windowDurationSec > 0.0f) ? (mainImpulse / windowDurationSec) : 0.0f;
  float burnTime = (thrustEndMs > thrustStartMs) ? ((thrustEndMs - thrustStartMs) / 1000.0f) : 0.0f;
  float delaySecs = 0.0f;
  if (foundEject && ejectTimeMs >= thrustEndMs) {
    delaySecs = (ejectTimeMs - thrustEndMs) / 1000.0f;
  }
  String rating = "?";
  const struct { const char* letter; float low; float high; } ratingTable[] = {
    {"A", 1.26f, 2.50f}, {"B", 2.51f, 5.00f}, {"C", 5.01f, 10.00f}, {"D", 10.01f, 20.00f},
    {"E", 20.01f, 40.00f}, {"F", 40.01f, 80.00f}, {"G", 80.01f, 160.00f}, {"H", 160.01f, 320.00f},
    {"J", 320.01f, 640.00f}, {"K", 640.01f, 1280.00f}
  };
  for (auto &entry : ratingTable) {
    if (mainImpulse >= entry.low && mainImpulse <= entry.high) {
      rating = String(entry.letter);
      break;
    }
  }
  // Standard motor designation: {Letter}{AvgThrust}-{Delay}  e.g. "C10-5"
  rating += String((int)round(averageThrust));
  if (foundEject && delaySecs > 0.0f) {
    int delayVal = constrain((int)round(delaySecs), 0, 99);
    rating += "-" + String(delayVal);
  }

  String saveStatus;
  String filename = "/RASP_" + String(millis()) + ".eng";
  if (rtcAvailable) {
    DateTime now = rtc.now();
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    filename = String("/RASP_") + buf + ".eng";
  }
  if (!sdPresent) {
    saveStatus = "Not saved (no SD card).";
  } else {
    File file = SD.open(filename, FILE_WRITE);
    if (file) {
    if (rtcAvailable) {
      DateTime now = rtc.now();
      file.printf("timestamp=%04u-%02u-%02u %02u:%02u:%02u\n", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    } else {
      file.printf("timestamp=%lu\n", millis() / 1000);
    }
    file.printf("total_impulse_Ns=%.3f\n", mainImpulse);
    file.printf("avg_thrust_N=%.3f\n", averageThrust);
    file.printf("burn_time_s=%.3f\n", burnTime);
    file.printf("max_force_N=%.3f\n", maxForceMain);
    file.printf("delay_s=%.3f\n", delaySecs);
    file.printf("ejection_detected=%d\n", foundEject ? 1 : 0);
    file.printf("rating=%s\n", rating.c_str());
    file.printf("data_ms,force_N\n");
    for (int i = 0; i < sampleCount; i++) {
      file.printf("%lu,%.4f\n", samples[i].timeMs, samples[i].forceN);
    }
    file.close();
      saveStatus = "Saved " + filename;
    } else {
      saveStatus = "Save failed";
    }
  }

  captureResultImpulse = mainImpulse;
  captureResultAvgThrust = averageThrust;
  captureResultBurnTime = burnTime;
  captureResultMaxForce = maxForceMain;
  captureResultDelay = delaySecs;
  captureResultRating = rating;
  captureResultSaveStatus = saveStatus;
  captureResultThrustStartMs = thrustStartMs;
  captureResultThrustEndMs = thrustEndMs;
  captureResultEjectMs = ejectTimeMs;
  captureResultEjectDetected = foundEject;
  captureResultShowGraph = true;
  currentScreen = SCREEN_CAPTURE_RESULT;
  drawCaptureResultScreen();
  if (reachedMaxRuntime) {
    drawMessage("Capture reached max runtime.");
  }
}

void loop() {
  TouchPoint touch;
  static unsigned long lastCaptureUiMs = 0;
  static unsigned long lastScaleUiMs = 0;
  static unsigned long lastCalUiMs = 0;
  if (getTouchedPoint(touch)) {
    if (runtimeDebugFeaturesEnabled) {
      tft.fillCircle(touch.x, touch.y, 2, ILI9341_MAGENTA);
    }
    if (hitTest(touch, 0, 0, 28, 24)) {
      if (currentScreen == SCREEN_HOME) {
        // root screen: no-op
      } else if (currentScreen == SCREEN_WIFI_SCAN || currentScreen == SCREEN_PARAMETERS || currentScreen == SCREEN_RTC || currentScreen == SCREEN_CALIBRATION) {
        currentScreen = SCREEN_SETTINGS;
        drawSettingsScreen();
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

          // Reset trend graph only. Keep capture list history on screen.
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
          currentScreen = SCREEN_PARAMETERS;
          drawParametersScreen();
        }
        break;
      case SCREEN_PARAMETERS:
        if (hitTest(touch, 210, 40, 22, 22)) {
          bool rot180 = settings.rotate180 == 1;
          settings.rotate180 = rot180 ? 0 : 1;
          displayRotation = settings.rotate180 ? 2 : 1;
          applyRotation(displayRotation);
          saveSettings();
          resetTouchBuffer();
          drawParametersScreen();
          drawMessage(String("Rotate 180 ") + (rot180 ? "OFF" : "ON"));
        } else if (hitTest(touch, 210, 200, 22, 22)) {
          runtimeDebugFeaturesEnabled = !runtimeDebugFeaturesEnabled;
          if (!runtimeDebugFeaturesEnabled && currentScreen == SCREEN_SCALE) {
            tft.fillRect(16, 26, 194, 24, ILI9341_BLACK);
          }
          drawParametersScreen();
          drawMessage(String("Debug ") + (runtimeDebugFeaturesEnabled ? "ON (runtime)" : "OFF"));
        } else if (hitTest(touch, 246, 64, 58, 20)) {
          String entered = inputTextKeyboard("Start threshold N", String(settings.startThreshold, 1), false, true);
          float v = entered.toFloat();
          if (v > 0.0f && v < 2000.0f) {
            settings.startThreshold = v;
            saveSettings();
            drawParametersScreen();
          } else {
            drawMessage("Invalid start threshold");
          }
        } else if (hitTest(touch, 220, 86, 84, 20)) {
          String entered = inputTextKeyboard("Pre capture sec", String((float)settings.preCaptureMs / 1000.0f, 1), false, true);
          float v = entered.toFloat();
          if (v >= 0.10f && v <= 12.00f) {
            settings.preCaptureMs = (uint16_t)roundf(v * 1000.0f);
            saveSettings();
            drawParametersScreen();
          } else {
            drawMessage("Use 0.10..12.00 s");
          }
        } else if (hitTest(touch, 220, 108, 84, 20)) {
          String entered = inputTextKeyboard("Post capture sec", String((float)settings.postCaptureMs / 1000.0f, 1), false, true);
          float v = entered.toFloat();
          if (v >= 0.10f && v <= 10.00f) {
            settings.postCaptureMs = (uint16_t)roundf(v * 1000.0f);
            saveSettings();
            drawParametersScreen();
          } else {
            drawMessage("Use 0.10..10.00 s");
          }
        } else if (hitTest(touch, 220, 130, 84, 20)) {
          String entered = inputTextKeyboard("End holdoff ms", String(settings.thrustEndHoldoffMs), false, false);
          int v = entered.toInt();
          if (v >= 20 && v <= 500) {
            settings.thrustEndHoldoffMs = (uint16_t)v;
            saveSettings();
            drawParametersScreen();
          } else {
            drawMessage("Use 20..500 ms");
          }
        } else if (hitTest(touch, 220, 152, 84, 20)) {
          String entered = inputTextKeyboard("Ejection wait sec", String(settings.ejectionWaitSeconds, 1), false, true);
          float v = entered.toFloat();
          if (v >= 0.5f && v <= 15.0f) {
            settings.ejectionWaitSeconds = v;
            saveSettings();
            drawParametersScreen();
          } else {
            drawMessage("Use 0.5..15.0 s");
          }
        } else if (hitTest(touch, 220, 174, 84, 20)) {
          String entered = inputTextKeyboard("Eject detect N", String(settings.ejectionDetectForceN, 1), false, true);
          float v = entered.toFloat();
          if (v >= 0.2f && v <= 30.0f) {
            settings.ejectionDetectForceN = v;
            saveSettings();
            drawParametersScreen();
          } else {
            drawMessage("Use 0.2..30.0 N");
          }
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
              // Correct calibration from first principles: factor = counts / kg.
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
          // Keep footer messages visible instead of immediately overwriting them.
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
        }
        break;
    }
  }

  if (currentScreen == SCREEN_SCALE) {
    unsigned long now = millis();
    if (now - lastScaleUiMs > 180) {
      drawScaleValue();
      lastScaleUiMs = now;
    }
  }

  emitScaleDiagnosticsSerial();

  if (currentScreen == SCREEN_CALIBRATION) {
    static bool calMassInitialized = false;
    static float calMassFiltered = 0.0f;
    unsigned long now = millis();
    if (now - lastCalUiMs > 180) {
      float measured = netLoadKgFromUnits(readUnitsAverage(CAL_UI_RAW_SAMPLES)) * 1000.0f;
      if (!calMassInitialized) {
        calMassFiltered = measured;
        calMassInitialized = true;
      } else {
        calMassFiltered = calMassFiltered * 0.80f + measured * 0.20f;
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

  // Poll capture status and update UI without blocking
  if (captureInProgress) {
    unsigned long now = millis();
    if (now - lastCaptureUiMs > 200) {
      // draw a lightweight progress graph from current samples
      xSemaphoreTake(samplesMutex, portMAX_DELAY);
      int sc = sampleCount;
      // compute simple max
      float maxF = 0.001f;
      for (int i = 0; i < sc; i++) maxF = max(maxF, samples[i].forceN);
      int left = 10;
      int top = 34;
      int graphW = tft.width() - 20;
      int graphH = 156;
      float xScale = sc > 1 ? (float)graphW / max(1.0f, (float)samples[sc - 1].timeMs) : 1.0f;
      float graphTopN = ((float)(((int)ceil(maxF / ((maxF > 40.0f) ? 10.0f : 5.0f)))) * ((maxF > 40.0f) ? 10.0f : 5.0f));
      float yScale = graphH / max(5.0f, graphTopN);
      tft.fillRect(left, top, graphW, graphH, ILI9341_BLACK);
      drawGraphYAxisLabels(left, top, graphW, graphH, maxF);
      for (int i = 1; i < sc; i++) {
        int x1 = left + (int)(samples[i - 1].timeMs * xScale);
        int y1 = top + graphH - (int)(max(0.0f, samples[i - 1].forceN) * yScale);
        int x2 = left + (int)(samples[i].timeMs * xScale);
        int y2 = top + graphH - (int)(max(0.0f, samples[i].forceN) * yScale);
        tft.drawLine(x1, y1, x2, y2, ILI9341_YELLOW);
      }
      xSemaphoreGive(samplesMutex);
      lastCaptureUiMs = now;
    }
  }

  if (captureFinished) {
    // copy samples under mutex then process/save
    xSemaphoreTake(samplesMutex, portMAX_DELAY);
    int sc = sampleCount;
    for (int i = 0; i < sc; i++) captureCopy[i] = samples[i];
    xSemaphoreGive(samplesMutex);
    // compute metrics and save file (same as previous logic)
    float maxForce = 0.001f;
    for (int i = 0; i < sc; i++) maxForce = max(maxForce, captureCopy[i].forceN);
    unsigned long lastAbove = sc ? captureCopy[sc - 1].timeMs : 0;
    unsigned long captureStartLocal = captureStartMs;
    float totalImpulse = 0.0f;
    for (int i = 0; i < sc; i++) {
      totalImpulse += captureCopy[i].forceN * (captureSampleIntervalUs() / 1000000.0f);
    }
    float burnTime = sc ? (captureCopy[sc - 1].timeMs / 1000.0f) : 0.0f;
    float delaySecs = 0.0f;
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
      for (int i = 0; i < sc; i++) {
        file.printf("%lu,%.4f\n", captureCopy[i].timeMs, captureCopy[i].forceN);
      }
      file.close();
      statusLine = "Saved " + filename;
    } else {
      statusLine = "Save failed";
    }
    drawMessage(statusLine);
    // reset flag
    captureFinished = false;
    captureInProgress = false;
  }

  delay(30);
}
