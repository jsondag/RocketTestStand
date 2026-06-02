#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <ADS1220_WE.h>

// Display pins
#define TFT_CS 15
#define TFT_DC 2
#define TFT_RST -1
#define TFT_BL 21
#define TFT_MOSI 13
#define TFT_MISO 12
#define TFT_SCLK 14

// ADS1220 pins
#define ADS1220_CS 17
#define ADS1220_SCLK 22
#define ADS1220_MOSI 16
#define ADS1220_MISO 35
#define ADS1220_DRDY 27

// Set true if REFP0/REFN0 are wired and stable.
static const bool USE_EXTERNAL_REF0 = true;
static const float EXTERNAL_REF0_VOLTS = 4.86f;

static const int GRAPH_X = 8;
static const int GRAPH_Y = 34;
static const int GRAPH_W = 304;
static const int GRAPH_H = 170;
static const int GRAPH_POINTS = GRAPH_W;

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);
SPIClass adsSpi(HSPI);
ADS1220_WE ads(&adsSpi, ADS1220_CS, ADS1220_DRDY, ADS1220_MOSI, ADS1220_MISO, ADS1220_SCLK);

int32_t rawHistory[GRAPH_POINTS];
int historyHead = 0;
int historyCount = 0;

unsigned long lastSampleMs = 0;
unsigned long lastUiMs = 0;
unsigned long lastDiagMs = 0;

uint32_t diagCount = 0;
int32_t diagMin = 0;
int32_t diagMax = 0;
float diagAbsDeltaEma = 0.0f;
int32_t lastRaw = 0;
bool haveLastRaw = false;

void drawStaticUi() {
  tft.fillScreen(ILI9341_BLACK);
  tft.fillRect(0, 0, 320, 24, ILI9341_DARKGREY);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(8, 4);
  tft.print("ADS1220 Debug");

  tft.drawRect(GRAPH_X, GRAPH_Y, GRAPH_W, GRAPH_H, ILI9341_WHITE);
  tft.drawFastHLine(GRAPH_X + 1, GRAPH_Y + GRAPH_H / 2, GRAPH_W - 2, ILI9341_DARKGREY);

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(8, 210);
  tft.print("Raw graph + serial diagnostics");
}

void pushRaw(int32_t raw) {
  rawHistory[historyHead] = raw;
  historyHead = (historyHead + 1) % GRAPH_POINTS;
  if (historyCount < GRAPH_POINTS) {
    historyCount++;
  }
}

void drawGraphAndStats(int32_t raw, float mv) {
  tft.fillRect(0, 24, 320, 10, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(8, 25);
  tft.print("raw=");
  tft.print(raw);
  tft.print(" mV=");
  tft.print(mv, 3);
  tft.print(" drdy=");
  tft.print(digitalRead(ADS1220_DRDY) == LOW ? "L" : "H");

  tft.fillRect(GRAPH_X + 1, GRAPH_Y + 1, GRAPH_W - 2, GRAPH_H - 2, ILI9341_BLACK);
  if (historyCount < 2) {
    return;
  }

  int32_t minRaw = INT32_MAX;
  int32_t maxRaw = INT32_MIN;
  for (int i = 0; i < historyCount; i++) {
    int idx = (historyHead - historyCount + i + GRAPH_POINTS) % GRAPH_POINTS;
    int32_t v = rawHistory[idx];
    if (v < minRaw) minRaw = v;
    if (v > maxRaw) maxRaw = v;
  }

  int32_t span = maxRaw - minRaw;
  if (span < 200) {
    int32_t mid = (maxRaw + minRaw) / 2;
    minRaw = mid - 100;
    maxRaw = mid + 100;
    span = 200;
  }

  int prevX = GRAPH_X + 1;
  int prevY = GRAPH_Y + GRAPH_H / 2;
  for (int i = 0; i < historyCount; i++) {
    int idx = (historyHead - historyCount + i + GRAPH_POINTS) % GRAPH_POINTS;
    int32_t v = rawHistory[idx];

    int x = GRAPH_X + 1 + (i * (GRAPH_W - 3)) / max(1, historyCount - 1);
    int y = GRAPH_Y + 1 + (GRAPH_H - 3) - (int)((int64_t)(v - minRaw) * (GRAPH_H - 3) / span);

    if (i > 0) {
      tft.drawLine(prevX, prevY, x, y, ILI9341_GREEN);
    }
    prevX = x;
    prevY = y;
  }

  tft.setTextColor(ILI9341_DARKGREY);
  tft.setTextSize(1);
  tft.setCursor(GRAPH_X + 2, GRAPH_Y + 2);
  tft.print(maxRaw);
  tft.setCursor(GRAPH_X + 2, GRAPH_Y + GRAPH_H - 10);
  tft.print(minRaw);
}

void printDiagnostics1Hz(int32_t raw, float mv) {
  if (diagCount == 0) {
    diagMin = raw;
    diagMax = raw;
  } else {
    if (raw < diagMin) diagMin = raw;
    if (raw > diagMax) diagMax = raw;
  }

  if (haveLastRaw) {
    int32_t d = raw - lastRaw;
    diagAbsDeltaEma = diagAbsDeltaEma * 0.90f + fabsf((float)d) * 0.10f;
  } else {
    diagAbsDeltaEma = 0.0f;
    haveLastRaw = true;
  }

  lastRaw = raw;
  diagCount++;

  unsigned long now = millis();
  if (now - lastDiagMs < 1000UL) {
    return;
  }

  int32_t span = diagMax - diagMin;
  Serial.print("ADSDBG raw=");
  Serial.print(raw);
  Serial.print(" mv=");
  Serial.print(mv, 4);
  Serial.print(" drdy=");
  Serial.print(digitalRead(ADS1220_DRDY) == LOW ? "L" : "H");
  Serial.print(" samples=");
  Serial.print(diagCount);
  Serial.print(" min=");
  Serial.print(diagMin);
  Serial.print(" max=");
  Serial.print(diagMax);
  Serial.print(" span=");
  Serial.print(span);
  Serial.print(" nse=");
  Serial.print(diagAbsDeltaEma, 1);
  Serial.print(" gain=");
  Serial.print((int)ads.getGainFactor());
  Serial.print(" pgaBypass=");
  Serial.println(ads.isPGABypassed() ? 1 : 0);

  diagCount = 0;
  lastDiagMs = now;
}

void setupAds() {
  Serial.println("[ADS] init start");
  if (!ads.init()) {
    Serial.println("[ADS] init failed: no response");
    while (true) {
      delay(250);
    }
  }

  ads.setCompareChannels(ADS1220_MUX_0_1);
  ads.setConversionMode(ADS1220_CONTINUOUS);
  ads.setOperatingMode(ADS1220_NORMAL_MODE);
  ads.setDataRate(ADS1220_DR_LVL_5);

  ads.setGain(ADS1220_GAIN_1);
  ads.bypassPGA(true);

  if (USE_EXTERNAL_REF0) {
    ads.setVRefSource(ADS1220_VREF_REFP0_REFN0);
    ads.setVRefValue_V(EXTERNAL_REF0_VOLTS);
    Serial.print("[ADS] reference: REFP0/REFN0 = ");
    Serial.print(EXTERNAL_REF0_VOLTS, 3);
    Serial.println(" V");
  } else {
    ads.setIntVRef();
    Serial.println("[ADS] reference: internal 2.048V");
  }

  Serial.print("[ADS] gain factor: ");
  Serial.println((int)ads.getGainFactor());
  Serial.print("[ADS] PGA bypass: ");
  Serial.println(ads.isPGABypassed() ? "yes" : "no");
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== ADS1220 Debug Firmware ===");

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin();
  tft.setRotation(1);
  drawStaticUi();

  pinMode(ADS1220_DRDY, INPUT_PULLUP);
  setupAds();
  Serial.println("[SYS] running");
}

void loop() {
  unsigned long now = millis();
  if (now - lastSampleMs < 20UL) {
    return;
  }
  lastSampleMs = now;

  int32_t raw = ads.getRawData();
  float mv = ads.getVoltage_mV();

  pushRaw(raw);
  printDiagnostics1Hz(raw, mv);

  if (now - lastUiMs >= 100UL) {
    lastUiMs = now;
    drawGraphAndStats(raw, mv);
  }
}
