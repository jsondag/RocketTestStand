#include "globals.h"

void drawHomeClock() {
  tft.fillRect(0, 222, tft.width(), 18, ILI9341_BLACK);
  if (!timeAvailable) return;
  char buf[24];
  if (!formatLocalTime(buf, sizeof(buf), "%Y-%m-%d  %H:%M:%S")) return;
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  int textW = (int)strlen(buf) * 12;
  int x = max(0, (tft.width() - textW) / 2);
  tft.setCursor(x, 224);
  tft.print(buf);
}

void drawHomeScreen() {
  drawHeader("Rocket Stand");
  drawButton(16, 40, 140, 80, "Scale");
  drawButton(164, 40, 140, 80, "Capture\nThrust");
  drawButton(16, 136, 140, 80, "Saved\nCurves");
  drawButton(164, 136, 140, 80, "Settings");
  drawHomeClock();
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
    filteredGrams = filteredGrams * 0.55f + grams * 0.45f;
  }

  scaleLastFilteredGrams = filteredGrams;

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
  if (settings.ejectionWaitSeconds <= 0.0f) {
    tft.print("Off");
  } else {
    tft.print(settings.ejectionWaitSeconds, 1);
    tft.print(" s");
  }
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
    tft.drawRect(8, 40, tft.width() - 16, 144, ILI9341_WHITE);
    float maxForce = 0.001f;
    for (int i = 0; i < sampleCount; i++) {
      maxForce = max(maxForce, samples[i].forceN);
    }
    int left = 10;
    int top = 42;
    int graphW = tft.width() - 20;
    int graphH = 140;
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

    float impulseNs = 0.0f;
    for (int i = 1; i < sampleCount; i++) {
      float dt = (samples[i].timeMs - samples[i - 1].timeMs) / 1000.0f;
      impulseNs += 0.5f * (samples[i].forceN + samples[i - 1].forceN) * dt;
    }
    float durSec = duration / 1000.0f;

    tft.setTextSize(1);
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(16, 188);
    tft.print("Class: ");
    tft.print(loadedFileRating.length() ? loadedFileRating : String("?"));
    tft.print("  Imp ");
    tft.print(impulseNs, 1);
    tft.print("Ns  Pk ");
    tft.print(maxForce, 1);
    tft.print("N  ");
    tft.print(durSec, 2);
    tft.print("s");
    tft.setTextColor(ILI9341_CYAN);
    tft.setCursor(16, 200);
    tft.print("File: ");
    tft.print(loadedFileName);
  }
  drawSmallButton(180, 212, 132, 26, "Delete", ILI9341_RED);
}

void drawSettingsMenu() {
  drawHeader("Settings");
  drawHamburger();
  drawButton(16, 40, 140, 64, "WiFi");
  drawButton(164, 40, 140, 64, "Calibrate");
  drawButton(90, 112, 140, 64, "Parameters");
  drawFooter("Open Parameters for capture + rotate.");
}

void drawSettingsScreen() {
  drawSettingsMenu();
}

void drawParametersScreen() {
  drawHeader("Parameters");
  drawHamburger();
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);

  int ry;

  ry = paramRowY(0);
  if (paramRowVisible(0)) {
    tft.setCursor(16, ry + 8);
    tft.print("Rotate 180");
    tft.drawRect(210, ry + 4, 22, 22, ILI9341_WHITE);
    if (settings.rotate180 == 1) tft.fillRect(214, ry + 8, 14, 14, ILI9341_GREEN);
  }

  ry = paramRowY(1);
  if (paramRowVisible(1)) {
    tft.setCursor(16, ry + 7);
    tft.print("Start Threshold (N):");
    tft.fillRoundRect(246, ry + 5, 58, 20, 4, ILI9341_DARKGREY);
    tft.drawRoundRect(246, ry + 5, 58, 20, 4, ILI9341_WHITE);
    tft.setCursor(250, ry + 7);
    tft.print(settings.startThreshold, 1);
  }

  ry = paramRowY(2);
  if (paramRowVisible(2)) {
    tft.setCursor(16, ry + 7);
    tft.print("Pre Capture (S):");
    tft.fillRoundRect(220, ry + 5, 84, 20, 4, ILI9341_DARKGREY);
    tft.drawRoundRect(220, ry + 5, 84, 20, 4, ILI9341_WHITE);
    tft.setCursor(226, ry + 7);
    tft.print((float)settings.preCaptureMs / 1000.0f, 1);
  }

  ry = paramRowY(3);
  if (paramRowVisible(3)) {
    tft.setCursor(16, ry + 7);
    tft.print("Post Capture (S):");
    tft.fillRoundRect(220, ry + 5, 84, 20, 4, ILI9341_DARKGREY);
    tft.drawRoundRect(220, ry + 5, 84, 20, 4, ILI9341_WHITE);
    tft.setCursor(226, ry + 7);
    tft.print((float)settings.postCaptureMs / 1000.0f, 1);
  }

  ry = paramRowY(4);
  if (paramRowVisible(4)) {
    tft.setCursor(16, ry + 7);
    tft.print("End Holdoff (ms):");
    tft.fillRoundRect(220, ry + 5, 84, 20, 4, ILI9341_DARKGREY);
    tft.drawRoundRect(220, ry + 5, 84, 20, 4, ILI9341_WHITE);
    tft.setCursor(226, ry + 7);
    tft.print(settings.thrustEndHoldoffMs);
  }

  ry = paramRowY(5);
  if (paramRowVisible(5)) {
    tft.setCursor(16, ry + 7);
    tft.print("Eject wait (S):");
    tft.fillRoundRect(220, ry + 5, 84, 20, 4, ILI9341_DARKGREY);
    tft.drawRoundRect(220, ry + 5, 84, 20, 4, ILI9341_WHITE);
    tft.setCursor(226, ry + 7);
    tft.print(settings.ejectionWaitSeconds, 1);
  }

  ry = paramRowY(6);
  if (paramRowVisible(6)) {
    tft.setCursor(16, ry + 7);
    tft.print("Eject Detect (N):");
    tft.fillRoundRect(220, ry + 5, 84, 20, 4, ILI9341_DARKGREY);
    tft.drawRoundRect(220, ry + 5, 84, 20, 4, ILI9341_WHITE);
    tft.setCursor(226, ry + 7);
    tft.print(settings.ejectionDetectForceN, 1);
  }

  ry = paramRowY(7);
  if (paramRowVisible(7)) {
    tft.setCursor(16, ry + 7);
    tft.print("Max Burn (S):");
    tft.fillRoundRect(220, ry + 5, 84, 20, 4, ILI9341_DARKGREY);
    tft.drawRoundRect(220, ry + 5, 84, 20, 4, ILI9341_WHITE);
    tft.setCursor(226, ry + 7);
    tft.print((float)settings.maxBurnMs / 1000.0f, 1);
  }

  ry = paramRowY(8);
  if (paramRowVisible(8)) {
    tft.setCursor(16, ry + 7);
    tft.print("Time Zone:");
    tft.fillRoundRect(170, ry + 5, 134, 20, 4, ILI9341_DARKGREY);
    tft.drawRoundRect(170, ry + 5, 134, 20, 4, ILI9341_WHITE);
    tft.setCursor(176, ry + 7);
    int tzIdx = settings.timezoneIndex;
    if (tzIdx < 0 || tzIdx >= kTimezoneCount) tzIdx = TZ_DEFAULT_INDEX;
    tft.print(kTimezones[tzIdx].label);
  }

  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(1);
  char pageBuf[24];
  snprintf(pageBuf, sizeof(pageBuf), "Page %d/%d", paramsPage + 1, PARAMS_PAGE_COUNT);
  int pw = strlen(pageBuf) * 6;
  tft.setCursor((tft.width() - pw) / 2, 192);
  tft.print(pageBuf);
  drawSmallButton(40,  202, 110, 34, "Prev");
  drawSmallButton(170, 202, 110, 34, "Next");
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
  const int rowH = 34;
  const int rowsVisible = 5;
  for (int i = 0; i < rowsVisible; i++) {
    int idx = wifiScroll + i;
    if (idx >= wifiCount) break;
    int y = 32 + i * rowH;
    String label = wifiNetworks[idx].ssid;
    if (wifiNetworks[idx].secure) label += " *";
    drawSmallButton(16, y, 288, rowH - 4, label, ILI9341_DARKGREY);
  }
  if (wifiCount == 0) {
    drawTextBlock(16, 60, 280, 100, "No networks found.");
  }
  int totalPages = max(1, (wifiCount + rowsVisible - 1) / rowsVisible);
  int currentPage = (wifiScroll / rowsVisible) + 1;
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(1);
  tft.fillRect(16, 200, 200, 12, ILI9341_BLACK);
  tft.setCursor(16, 202);
  tft.print("Page ");
  tft.print(currentPage);
  tft.print("/");
  tft.print(totalPages);
  tft.print("  (");
  tft.print(wifiCount);
  tft.print(" found)");
  drawSmallButton(16, 218, 68, 20, "Up");
  drawSmallButton(90, 218, 68, 20, "Down");
  drawSmallButton(164, 218, 68, 20, "Rescan");
  drawSmallButton(238, 218, 68, 20, "Home", ILI9341_RED);
  drawFooter("Tap SSID to connect.");
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

  drawSmallButton(16, 122, 92, 34, "Tare");
  drawSmallButton(16, 164, 140, 34, "Weight");
  drawSmallButton(164, 164, 140, 34, "Calibrate");

  drawFooter("Live mass. Set target, then Calibrate.");
}

void drawTimezonePickerScreen() {
  drawHeader("Time Zone");
  drawHamburger();
  const int rowsVisible = 5;
  const int rowH = 32;
  if (tzPickerScroll < 0) tzPickerScroll = 0;
  int maxScroll = max(0, kTimezoneCount - rowsVisible);
  if (tzPickerScroll > maxScroll) tzPickerScroll = maxScroll;
  for (int i = 0; i < rowsVisible; i++) {
    int idx = tzPickerScroll + i;
    if (idx >= kTimezoneCount) break;
    int y = 32 + i * rowH;
    bool selected = (idx == settings.timezoneIndex);
    drawSmallButton(8, y, 304, rowH - 4, kTimezones[idx].label,
                    selected ? ILI9341_DARKGREEN : ILI9341_DARKGREY);
  }
  int totalPages = max(1, (kTimezoneCount + rowsVisible - 1) / rowsVisible);
  int currentPage = (tzPickerScroll / rowsVisible) + 1;
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(1);
  char pageBuf[24];
  snprintf(pageBuf, sizeof(pageBuf), "Page %d/%d", currentPage, totalPages);
  int pw = strlen(pageBuf) * 6;
  tft.setCursor((tft.width() - pw) / 2, 192);
  tft.print(pageBuf);
  drawSmallButton(40,  202, 110, 34, "Prev");
  drawSmallButton(170, 202, 110, 34, "Next");
}

void drawLoadFilesScreen() {
  drawHeader("File Browser");
  drawHamburger();
  for (int i = 0; i < 4; i++) {
    int idx = savedFileScroll + i;
    int y = 40 + i * 38;
    if (idx >= savedFileCount) break;
    drawSmallButton(16, y, 288, 32, savedFileList[idx], ILI9341_DARKGREY);
  }
  if (savedFileCount == 0) {
    drawTextBlock(16, 60, 280, 100, "No .eng files found.");
  }
  int totalPages = max(1, (savedFileCount + 3) / 4);
  int currentPage = (savedFileScroll / 4) + 1;
  tft.fillRect(0, 188, tft.width(), 12, ILI9341_BLACK);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(1);
  char pageBuf[24];
  snprintf(pageBuf, sizeof(pageBuf), "Page %d/%d", currentPage, totalPages);
  int pw = strlen(pageBuf) * 6;
  tft.setCursor((tft.width() - pw) / 2, 190);
  tft.print(pageBuf);
  drawSmallButton(40,  202, 110, 34, "Prev");
  drawSmallButton(170, 202, 110, 34, "Next");
}
