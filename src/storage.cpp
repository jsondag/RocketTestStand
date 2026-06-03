#include "globals.h"

void resetDefaultSettings() {
  strcpy(settings.wifiSsid, "");
  strcpy(settings.wifiPassword, "");
  settings.calibrationFactor     = DEFAULT_CAL_FACTOR;
  settings.rotate180             = 0;
  settings.startThreshold        = DEFAULT_START_THRESHOLD;
  settings.preCaptureMs          = DEFAULT_PRE_CAPTURE_MS;
  settings.postCaptureMs         = DEFAULT_POST_CAPTURE_MS;
  settings.ejectionWaitSeconds   = DEFAULT_EJECTION_WAIT_S;
  settings.ejectionDetectForceN  = DEFAULT_EJECTION_DETECT_N;
  settings.thrustEndHoldoffMs    = DEFAULT_THRUST_END_HOLDOFF_MS;
  settings.maxBurnMs             = DEFAULT_MAX_BURN_MS;
  settings.timezoneIndex         = TZ_DEFAULT_INDEX;
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
  file.printf("rotate180=%u\n", settings.rotate180);
  file.printf("startThreshold=%.3f\n", settings.startThreshold);
  file.printf("preCaptureMs=%u\n", settings.preCaptureMs);
  file.printf("postCaptureMs=%u\n", settings.postCaptureMs);
  file.printf("ejectionWaitSeconds=%.1f\n", settings.ejectionWaitSeconds);
  file.printf("ejectionDetectForceN=%.1f\n", settings.ejectionDetectForceN);
  file.printf("thrustEndHoldoffMs=%u\n", settings.thrustEndHoldoffMs);
  file.printf("maxBurnMs=%u\n", settings.maxBurnMs);
  file.printf("timezoneIndex=%d\n", (int)settings.timezoneIndex);
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
    if (line.length() > 256) {
      file.close();
      return false;
    }
    line.trim();
    if (line.length() == 0 || line.startsWith("#")) continue;
    int idx = line.indexOf('=');
    if (idx < 0) continue;
    String key = line.substring(0, idx);
    String value = line.substring(idx + 1);
    key.trim();
    value.trim();
    if (key == "ssid") {
      value.toCharArray(settings.wifiSsid, sizeof(settings.wifiSsid));
    } else if (key == "password") {
      value.toCharArray(settings.wifiPassword, sizeof(settings.wifiPassword));
    } else if (key == "calibrationFactor") {
      settings.calibrationFactor = value.toFloat();
    } else if (key == "rotate180") {
      settings.rotate180 = (uint8_t)constrain(value.toInt(), 0, 1);
    } else if (key == "startThreshold") {
      settings.startThreshold = value.toFloat();
    } else if (key == "preCaptureMs") {
      settings.preCaptureMs = value.toInt();
    } else if (key == "postCaptureMs") {
      settings.postCaptureMs = value.toInt();
    } else if (key == "ejectionWaitSeconds") {
      float parsed = value.toFloat();
      settings.ejectionWaitSeconds = isfinite(parsed) ? constrain(parsed, 0.0f, 15.0f) : 10.0f;
    } else if (key == "ejectionDetectForceN") {
      float parsed = value.toFloat();
      settings.ejectionDetectForceN = isfinite(parsed) ? constrain(parsed, 0.2f, 30.0f) : 1.5f;
    } else if (key == "thrustEndHoldoffMs") {
      settings.thrustEndHoldoffMs = (uint16_t)constrain(value.toInt(), 20, 500);
    } else if (key == "maxBurnMs") {
      settings.maxBurnMs = (uint16_t)constrain(value.toInt(), 500, 30000);
    } else if (key == "timezoneIndex") {
      int v = value.toInt();
      settings.timezoneIndex = (int8_t)constrain(v, 0, kTimezoneCount - 1);
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
  if (!isfinite(settings.ejectionWaitSeconds) || settings.ejectionWaitSeconds < 0.0f || settings.ejectionWaitSeconds > 15.0f) {
    settings.ejectionWaitSeconds = 10.0f;
  }
  if (!isfinite(settings.ejectionDetectForceN) || settings.ejectionDetectForceN < 0.2f || settings.ejectionDetectForceN > 30.0f) {
    settings.ejectionDetectForceN = 1.5f;
  }
  if (settings.thrustEndHoldoffMs < 20 || settings.thrustEndHoldoffMs > 500) {
    settings.thrustEndHoldoffMs = 90;
  }
  if (settings.maxBurnMs < 500 || settings.maxBurnMs > 30000) {
    settings.maxBurnMs = 5000;
  }
  if (settings.timezoneIndex < 0 || settings.timezoneIndex >= kTimezoneCount) {
    settings.timezoneIndex = TZ_DEFAULT_INDEX;
  }
  applyTimezone();
  if (sdPresent && (!SD.exists(SETTINGS_FILENAME) || !loadedFromSD)) {
    saveSettingsToSD();
  }
  return true;
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
  String path = filename;
  if (path.length() == 0) return;
  if (path[0] != '/') path = String("/") + path;
  Serial.print("[LOAD] Opening ");
  Serial.println(path);
  File file = SD.open(path);
  if (!file) {
    Serial.println("[LOAD] SD.open failed");
    sampleCount = 0;
    loadedFileRating = "";
    return;
  }
  sampleCount = 0;
  loadedFileRating = "";
  bool headerSeen = false;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    if (line.length() > 256) {
      break;
    }
    line.trim();
    if (line.length() == 0 || line.startsWith(";")) continue;
    if (!headerSeen) {
      headerSeen = true;
      int sp = line.indexOf(' ');
      loadedFileRating = (sp > 0) ? line.substring(0, sp) : line;
      continue;
    }
    int sp = line.indexOf(' ');
    if (sp > 0) {
      float t = line.substring(0, sp).toFloat();
      float f = line.substring(sp + 1).toFloat();
      if (sampleCount < CAPTURE_MAX_SAMPLES) {
        samples[sampleCount].timeMs = (unsigned long)(t * 1000.0f + 0.5f);
        samples[sampleCount].forceN = f;
        sampleCount++;
      }
    }
  }
  file.close();
}

void saveAndShowStatus(const String& message) {
  saveSettings();
  statusLine = message;
  drawMessage(message);
}
