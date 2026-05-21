# Rocket Test Stand for ESP32 CYD

This project implements a rocket motor test stand on an ESP32 CYD board using a 10 kg load cell, HX711 ADC board, a SPI display, and an SD card. It provides:

- Basic scale mode with tare and calibration
- Motor thrust capture with auto start/end detection
- Real-time thrust curve graphing on-screen
- RASP-style `.eng` file logging to onboard SD card
- Rocket rating estimate (e.g. `E12-6`) based on impulse and ejection delay
- Optional DS3231 RTC support
- Optional WiFi/NTP time sync for RTC and file timestamping
- File load/review for previously saved thrust curves

## Project Files

- `platformio.ini` - PlatformIO build configuration for ESP32
- `src/main.cpp` - Main application code
- `README.md` - Hardware wiring and setup notes

## Hardware Connections

### ESP32 CYD -> HX711

- `HX711_DOUT` -> GPIO 26
- `HX711_SCK` -> GPIO 25
- `HX711 VCC` -> 3.3V
- `HX711 GND` -> GND
- Load cell wires connect to HX711 input terminals per the HX711 board documentation

### ESP32 CYD -> TFT SPI Display

- `TFT_CS` -> GPIO 15
- `TFT_DC` -> GPIO 2
- `TFT_RST` -> GPIO 4
- `TFT_MOSI` -> GPIO 23
- `TFT_MISO` -> GPIO 19
- `TFT_SCLK` -> GPIO 18
- `TFT_BL` -> optional backlight pin if supported
- `TFT VCC` -> 3.3V
- `TFT GND` -> GND

### ESP32 CYD -> SD Card (SPI)

- `SD_CS` -> GPIO 5
- `SD_SCLK` -> GPIO 18
- `SD_MISO` -> GPIO 19
- `SD_MOSI` -> GPIO 23
- `SD_VCC` -> 3.3V
- `SD_GND` -> GND

> The display and SD card share the SPI bus. The SD card uses `CS=GPIO 5`, and the display uses `CS=GPIO 15`.

### Optional DS3231 RTC

- `RTC_SDA` -> GPIO 21
- `RTC_SCL` -> GPIO 22
- `RTC_VCC` -> 3.3V
- `RTC_GND` -> GND

### Touchscreen (XPT2046-style)

- `TOUCH_CS` -> GPIO 16
- `TOUCH_IRQ` -> GPIO 17
- `TOUCH_VCC` -> 3.3V
- `TOUCH_GND` -> GND

The touchscreen shares the SPI bus with the display and SD card. The driver reads raw XPT2046 touch coordinates from `TOUCH_CS` and `TOUCH_IRQ`.

## Code Settings

Open `src/main.cpp` to change these values:

- `settings.calibrationFactor` - initial HX711 scale calibration constant
- `settings.startThreshold` - thrust threshold to begin capture (N)
- `settings.preCaptureMs` - pre-capture baseline interval (ms)
- `settings.postCaptureMs` - post-capture hold interval after thrust ends (ms)
- `settings.waitForEjection` - toggle whether to look for ejection charge delay
- `settings.rtcEnabled` - whether RTC updates and file timestamping use the DS3231 when available

WiFi network configuration is entered through the touchscreen WiFi settings page.

## Operation

1. Build and flash using PlatformIO
2. Insert an SD card and verify it initializes
3. Optionally connect the DS3231 and use the touchscreen settings menu to enable RTC or sync RTC time via WiFi/NTP
4. Use the touchscreen interface to:
   - view a scale
   - tare the load cell
   - calibrate with a known mass and on-screen number keyboard
   - start thrust capture
   - load previously saved `.eng` files

## File Format

Saved files use `.eng` extension and include metadata:

- `timestamp`
- `total_impulse_Ns`
- `burn_time_s`
- `max_force_N`
- `delay_s`
- `rating`
- `data_ms,force_N`

The filename includes the capture timestamp.

## Notes

- Make sure the HX711 and load cell are powered from the same 3.3V supply as the ESP32.
- If the RTC is not connected, filenames are created with a fallback timestamp string.
- Settings are saved to `settings.ini` on the SD card when present.
- If no SD card is available, settings fall back to EEPROM storage.
- No physical buttons are required; the system uses the touchscreen UI.
