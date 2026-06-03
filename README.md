# Rocket Test Stand for ESP32 CYD

Firmware for an ESP32 CYD-based rocket motor test stand with touchscreen UI, on-device graphing, and .eng export.

Main features:
- Scale mode with tare and calibration
- Thrust capture with auto start/end logic and ejection delay analysis
- Graph and characterization results after each run, with NAR/TRA letter rating (1/4A through Z) computed from total impulse
- File browser for saved curves with on-device review (graph + classification + delete)
- SD card logging in RASP `.eng` format with date-stamped filenames
- WiFi auto-connect and non-blocking NTP time sync at boot; worldwide timezone presets (POSIX TZ rules including DST)
- Settings persisted to both EEPROM and SD `/settings.ini` (SD wins on load)

## Important ADC Selection Note

ADC model selection is fixed in code (not selectable on device).

In src/main.cpp:

- LOADCELL_ADC = SCALE_BACKEND_HX711
- or LOADCELL_ADC = SCALE_BACKEND_ADS1220

No auto-detect fallback is used when this is set. The firmware initializes only the selected ADC path.

## Pin Connections

The list below matches current pin definitions in src/main.cpp.

### ADC Option 1: HX711

- HX711_DOUT -> GPIO 22
- HX711_SCK -> GPIO 27
- HX711 VCC -> 3.3V
- HX711 GND -> GND

Load cell wires to HX711 (typical 4-wire cell):
- Red -> E+
- Black -> E-
- Green -> A+ (Signal+)
- White -> A- (Signal-)

### ADC Option 2: ADS1220 (bit-banged SPI)

- ADS1220_CS -> GPIO 17
- ADS1220_SCLK -> GPIO 22
- ADS1220_MOSI -> GPIO 16
- ADS1220_MISO -> GPIO 35
- ADS1220_DRDY -> GPIO 27
- ADS1220 AVDD/DVDD -> 3.3V
- ADS1220 GND -> GND
- ADS1220 REFNO -> GND
- ADS1220 REFPO -> 5V

Load cell wiring for ADS1220 depends on breakout labels, but differential signal should be wired as:
- Signal+ -> AIN0 (Green)
- Signal- -> AIN1 (White)
- Excitation- -> GND (Black)
- Excitation+ -> 5V (Red)

### CYD TFT (as used by firmware)

- TFT_CS -> GPIO 15
- TFT_DC -> GPIO 2
- TFT_RST -> -1 (not connected)
- TFT_BL -> GPIO 21
- TFT_MOSI -> GPIO 13
- TFT_MISO -> GPIO 12
- TFT_SCLK -> GPIO 14

### SD Card

- SD_CS -> GPIO 5
- SPI pins are shared with TFT bus in this design

### Touch Controller (bit-banged)

- TOUCH_CS -> GPIO 33
- TOUCH_IRQ -> GPIO 36
- TOUCH_CLK -> GPIO 25
- TOUCH_MOSI -> GPIO 32
- TOUCH_MISO -> GPIO 39

### Optional RTC (DS3231)

- I2C_SDA -> GPIO 18
- I2C_SCL -> GPIO 19

The DS3231 is not currently used; time is obtained from NTP after WiFi connects.

## Safety and Wiring Guidance

- Do not wire both ADC modules to the shared control/data pins at the same time unless you have proper hardware isolation.
- Use only one ADC module connected to the active LOADCELL_ADC selection.
- Calibrate after changing ADC model, load cell, wiring polarity, or mechanical setup.

## Build and Flash

From project root:

1. platformio run
2. platformio run -t upload

Default PlatformIO environment is esp32dev.

## Settings

Configurable via on-device Settings → Parameters:

- Rotate 180° (display orientation)
- Start Threshold (N) — capture trigger
- Pre / Post Capture (s)
- End Holdoff (ms)
- Ejection Wait (s) — set 0 to disable
- Ejection Detect (N)
- Max Burn (s)
- Time Zone — worldwide presets with DST rules

WiFi credentials are entered via Settings → WiFi (scan + on-screen keyboard) and persist to SD `/settings.ini`. On boot, if an SSID is saved the device auto-connects in the background and updates the home-screen clock once NTP returns valid time. Boot is never delayed waiting on the network.

## Project Files

- `platformio.ini` — PlatformIO environment and dependencies
- `src/main.cpp` — Firmware (UI, capture pipeline, SD/EEPROM persistence, WiFi/NTP, file review)
- `README.md` — Wiring and setup notes
