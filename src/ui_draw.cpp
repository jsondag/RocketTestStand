#include "globals.h"

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
  return data >> 3;
}

static int median9(int *values, int count) {
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
    int rx = (int)xptRead(0x90);
    int ry = (int)xptRead(0xD0);
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
  if (displayRotation == 1 || displayRotation == 3) {
    x = (tft.width() - 1) - x;
  }
  if (displayRotation == 2 || displayRotation == 3) {
    y = (tft.height() - 1) - y;
  }

  x = constrain(x, 0, tft.width() - 1);
  int h = tft.height() - 1;
  if (h > 0) {
    y += (y * 11) / h;
  }

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
  tft.fillTriangle(6, 12, 13, 6, 13, 18, ILI9341_WHITE);
  tft.fillRect(13, 10, 10, 4, ILI9341_WHITE);
}

void drawFooter(const char* text, uint16_t color) {
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

void drawSmallButton(int x, int y, int w, int h, const String& label, uint16_t color) {
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
  (void)w; (void)h;
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

void drawMessage(const String& message) {
  tft.fillRect(0, 216, tft.width(), 24, ILI9341_BLACK);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(1);
  tft.setCursor(4, 220);
  tft.print(message);
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

void drawScaleDiagnostics() {
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

void drawKeyboard(const String& title, const String& value, bool password, const char* const keys[], int rows, bool showShift, bool shiftActive) {
  drawHeader(title.c_str());
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(16, 32);
  const int maxChars = 24;
  String shown = value;
  if ((int)shown.length() > maxChars) {
    shown = "..." + shown.substring(shown.length() - (maxChars - 3));
  }
  if (shown.length() == 0) {
    tft.setTextColor(ILI9341_DARKGREY);
    tft.print("<empty>");
  } else {
    tft.print(shown);
  }

  int top = password ? 58 : 66;
  int rowHeight = password ? 28 : 46;
  for (int r = 0; r < rows; r++) {
    const char* row = keys[r];
    int cols = strlen(row);
    if (cols <= 0) continue;
    int cellW = (tft.width() - 32) / cols;
    for (int c = 0; c < cols; c++) {
      int x = 16 + c * cellW;
      int y = top + r * rowHeight;
      char displayCh = row[c];
      if (password && isalpha((unsigned char)displayCh) && !shiftActive) {
        displayCh = tolower(displayCh);
      }
      String label = String((char)displayCh);
      drawSmallButton(x, y, cellW - 4, rowHeight - 4, label, ILI9341_BLUE);
    }
  }

  int controlY = top + rows * rowHeight + 4;
  drawSmallButton(8, controlY, 60, 34, "DEL", ILI9341_DARKGREY);
  if (showShift) {
    drawSmallButton(72, controlY, 60, 34, shiftActive ? "SHIFT" : "shift");
  }
  drawSmallButton(136, controlY, 76, 34, "Back", ILI9341_RED);
  drawSmallButton(216, controlY, 96, 34, "OK", ILI9341_GREEN);
}

String inputTextKeyboard(const String& title, const String& initial, bool passwordMode, bool allowDot) {
  String value = initial;
  bool shiftActive = false;
  const char* alphaKeys[] = {"QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM", "1234567890", "!@#$%&*-_./?"};
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

    if (hitTest(p, 8, controlY, 60, 34)) {
      if (!value.isEmpty()) value.remove(value.length() - 1);
      needsRedraw = true;
      continue;
    }

    if (passwordMode && hitTest(p, 72, controlY, 60, 34)) {
      shiftActive = !shiftActive;
      needsRedraw = true;
      continue;
    }

    if (hitTest(p, 136, controlY, 76, 34)) {
      return initial;
    }

    if (hitTest(p, 216, controlY, 96, 34)) {
      return value;
    }

    bool changed = false;
    for (int r = 0; r < rows; r++) {
      const char* row = passwordMode ? alphaKeys[r] : numberKeys[r];
      int cols = strlen(row);
      if (cols <= 0) continue;
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

void resetTouchBuffer() {
  touchActive = false;
}

static const uint8_t cydMadctl[4] = {
  0x08,
  0x48,
  0x88,
  0xC8,
};

void applyRotation(uint8_t rot) {
  rot &= 0x03;
  tft.setRotation(1);
  tft.sendCommand(0x36, &cydMadctl[rot], 1);
}
