#include "globals.h"

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
      adsShiftByte(0x40);
      adsShiftByte(reg0);
      digitalWrite(ADS1220_CS, HIGH);
      adsReg0Active = reg0;
    };

    auto adsReadRegs = [&](uint8_t startReg, uint8_t count, uint8_t *out) {
      digitalWrite(ADS1220_CS, LOW);
      adsShiftByte((uint8_t)(0x20 | ((startReg & 0x03) << 2) | ((count - 1) & 0x03)));
      for (uint8_t i = 0; i < count; i++) {
        out[i] = adsShiftByte(0x00);
      }
      digitalWrite(ADS1220_CS, HIGH);
    };

    adsCommand(0x06);
    delay(2);
    digitalWrite(ADS1220_CS, LOW);
    adsShiftByte(0x43);
    adsShiftByte(0x0E);
    adsShiftByte(0xA4);
    adsShiftByte(0x10);
    adsShiftByte(0x00);
    digitalWrite(ADS1220_CS, HIGH);
    delay(2);
    adsCommand(0x08);

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

    adsCommand(0x0A);
    delay(1);
    adsWriteReg0(0x0E);
    adsCommand(0x08);
    Serial.println("ADS mux locked: AIN0-AIN1 (PGA=128)");

    applyScaleSettings();
    sensorOffsetRaw = readRawAverage(12, 40);
    offsetTare = readUnitsAverage(10);
    return true;
  };

  if (LOADCELL_ADC == SCALE_BACKEND_HX711) {
    if (initHx711Backend()) return true;
    Serial.println("HX711 not detected (fixed LOADCELL_ADC)");
    return false;
  }

  if (LOADCELL_ADC == SCALE_BACKEND_ADS1220) {
    if (initAds1220Backend()) return true;
    Serial.println("ADS1220 not detected (fixed LOADCELL_ADC)");
    return false;
  }

  Serial.println("Invalid LOADCELL_ADC value");
  return false;
}

long readRawAverage(byte times, unsigned long timeoutPerReadMs) {
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
  float cf = settings.calibrationFactor;
  if (!isfinite(cf) || fabsf(cf) < 1.0f) cf = DEFAULT_CAL_FACTOR;
  return ((double)raw - (double)sensorOffsetRaw) / cf;
}

float netLoadKgFromUnits(float units) {
  return (units - offsetTare) * LOAD_POLARITY;
}

float readUnitsAverage(byte times, unsigned long timeoutPerReadMs) {
  long raw = readRawAverage(times, timeoutPerReadMs);
  return rawToUnits(raw);
}

void applyScaleSettings() {
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

long captureTimedBaselineRaw(unsigned long durationMs, unsigned long timeoutPerReadMs) {
  (void)timeoutPerReadMs;
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
  } else if (activeScaleBackend == SCALE_BACKEND_ADS1220) {
    sensorOffsetRaw = captureTimedBaselineRaw(TARE_AVERAGE_DURATION_MS, 20);
  } else {
    sensorOffsetRaw = captureTimedBaselineRaw(TARE_AVERAGE_DURATION_MS, 25);
  }

  // Match tare behavior everywhere: compute residual unit offset over the same time window.
  long residualRaw = captureTimedBaselineRaw(TARE_AVERAGE_DURATION_MS, 20);
  offsetTare = rawToUnits(residualRaw);
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
