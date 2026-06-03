#include "globals.h"

void runCaptureRoutine() {
  drawHeader("Capturing...");
  drawHamburger();
  tft.drawRect(8, 32, tft.width() - 16, 160, ILI9341_WHITE);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(16, 208);
  tft.print("Waiting for thrust...");

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

  float accumForceSum = 0.0f;
  unsigned long accumTimeSum = 0;
  int accumCount = 0;
  const int ACCUM_INTERVAL = 6;

  bool baselineInitialized = false;
  float baseline = 0.0f;
  float baselineNoise = 0.1f;
  float filteredForce = 0.0f;
  float startDelta = settings.startThreshold;
  float endDelta = max(0.35f, settings.startThreshold * 0.45f);
  const float filterAlpha = expf(-(float)captureSampleIntervalUs() / 20000.0f);
  unsigned long waitStartMs = millis();
  unsigned long routineStartMs = waitStartMs;
  unsigned long lastSampleMs = waitStartMs;
  unsigned long maxCaptureRuntimeMs =
    20000UL +
    (unsigned long)settings.maxBurnMs +
    (unsigned long)settings.postCaptureMs +
    (unsigned long)(settings.ejectionWaitSeconds * 1000.0f)
    + 5000UL;

  (void)lastAbove; (void)thrustQuietSinceMs; (void)quietSinceMs; (void)maxForceAll;

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
      if (activeScaleBackend == SCALE_BACKEND_ADS1220 && (millis() - lastSampleMs) > 250UL) {
        // Force one best-effort sample read below.
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
        maxCaptureRuntimeMs = (nowMs - routineStartMs) +
          (unsigned long)settings.maxBurnMs +
          (unsigned long)settings.postCaptureMs +
          (unsigned long)(settings.ejectionWaitSeconds * 1000.0f) +
          5000UL;
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
        shouldStoreSample = storedBurnMs <= (unsigned long)settings.maxBurnMs;
      }
      if (shouldStoreSample) {
        accumForceSum += f;
        accumTimeSum += sampleTime;
        accumCount++;
        if (accumCount >= ACCUM_INTERVAL && sampleCount < CAPTURE_MAX_SAMPLES) {
          samples[sampleCount++] = {
            (unsigned long)(accumTimeSum / (unsigned long)accumCount),
            accumForceSum / (float)accumCount
          };
          accumForceSum = 0.0f;
          accumTimeSum = 0;
          accumCount = 0;
        }
      } else {
        if (accumCount > 0 && sampleCount < CAPTURE_MAX_SAMPLES) {
          samples[sampleCount++] = {
            (unsigned long)(accumTimeSum / (unsigned long)accumCount),
            accumForceSum / (float)accumCount
          };
          accumForceSum = 0.0f;
          accumTimeSum = 0;
          accumCount = 0;
        }
      }
      if (f > maxForceAll) maxForceAll = f;

      if (phase == MAIN_THRUST) {
        if (f > maxForceMain) maxForceMain = f;
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
          if (settings.ejectionWaitSeconds > 0.0f) {
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
        unsigned long burnElapsedMs = (nowMs >= captureStart) ? (nowMs - captureStart) : 0;
        if (phase == MAIN_THRUST && burnElapsedMs >= (unsigned long)settings.maxBurnMs) {
          mainEndMs = sampleTime;
          mainEndAbsMs = nowMs;
          if (settings.ejectionWaitSeconds > 0.0f) {
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
        (void)graphTopN;
        float xScale = (float)graphW / max(1.0f, (float)samples[sampleCount - 1].timeMs);
        float yScale = graphH / max(5.0f, (float)graphTopN);
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
  if (mainImpulse >= 1.26f) {
    static const char* kLetters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    float upper = 2.5f;
    for (int i = 0; kLetters[i] != '\0'; i++) {
      if (mainImpulse <= upper) { rating = String(kLetters[i]); break; }
      upper *= 2.0f;
    }
  } else if (mainImpulse >= 0.626f) {
    rating = "1/2A";
  } else if (mainImpulse >= 0.313f) {
    rating = "1/4A";
  }
  rating += String((int)round(averageThrust));
  if (foundEject && delaySecs > 0.0f) {
    int delayVal = constrain((int)round(delaySecs), 0, 99);
    rating += "-" + String(delayVal);
  }

  String saveStatus;
  static uint32_t raspFileCounter = 0;
  String filename;
  char tsBuf[32];
  bool haveStamp = formatLocalTime(tsBuf, sizeof(tsBuf), "%Y%m%d_%H%M%S");
  if (sdPresent) {
    for (int attempt = 0; attempt < 32; attempt++) {
      if (haveStamp) {
        filename = String("/RASP_") + tsBuf;
        if (attempt > 0) filename += "_" + String(attempt);
        filename += ".eng";
      } else {
        filename = "/RASP_" + String(millis()) + "_" + String(raspFileCounter++) + ".eng";
      }
      if (!SD.exists(filename)) break;
    }
  } else {
    filename = haveStamp
      ? (String("/RASP_") + tsBuf + ".eng")
      : ("/RASP_" + String(millis()) + "_" + String(raspFileCounter++) + ".eng");
  }
  if (!sdPresent) {
    saveStatus = "Not saved (no SD card).";
  } else {
    File file = SD.open(filename, FILE_WRITE);
    if (file) {
      String delays = (foundEject && delaySecs > 0.0f)
        ? String((int)round(delaySecs))
        : "P";
      file.printf("; Captured by RocketTestStand\n");
      char humanStamp[32];
      if (formatLocalTime(humanStamp, sizeof(humanStamp), "%Y-%m-%d %H:%M:%S")) {
        file.printf("; Recorded %s\n", humanStamp);
      }
      file.printf("%s 0 0 %s 0 0 UNK\n", rating.c_str(), delays.c_str());
      const long preWindowMs = 100;
      long originMs = (long)thrustStartMs - preWindowMs;
      int firstIdx = 0;
      while (firstIdx < sampleCount - 1 && (long)samples[firstIdx].timeMs < originMs) firstIdx++;
      long burnEndMs = (long)thrustStartMs + (long)(burnTime * 1000.0f + 0.5f);
      int lastIdx = sampleCount - 1;
      while (lastIdx > firstIdx && (long)samples[lastIdx].timeMs > burnEndMs + 50) lastIdx--;
      int windowCount = lastIdx - firstIdx + 1;
      const int maxPoints = 200;
      int bucketSize = max(1, windowCount / maxPoints);
      for (int i = firstIdx; i <= lastIdx; ) {
        int bucketEnd = min(i + bucketSize, lastIdx + 1);
        float sumT = 0, sumF = 0;
        int n = 0;
        for (int j = i; j < bucketEnd; j++) {
          sumT += (long)samples[j].timeMs - originMs;
          sumF += max(0.0f, samples[j].forceN);
          n++;
        }
        file.printf("   %.4f %.4f\n", (sumT / n) / 1000.0f, sumF / n);
        i = bucketEnd;
      }
      float finalT = ((long)thrustStartMs - originMs + (long)(burnTime * 1000.0f + 0.5f)) / 1000.0f;
      file.printf("   %.4f 0.0\n", finalT);
      file.printf(";\n");
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
