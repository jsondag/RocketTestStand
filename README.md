# Rocket Test Stand for ESP32 CYD

Firmware for an ESP32 CYD-based rocket motor test stand with touchscreen UI, on-device graphing, and .eng export.

Main features:
- Scale mode with tare and calibration
- Thrust capture with auto start/end logic and ejection delay analysis
- Graph and characterization results after each run, with NAR/TRA letter rating computed from total impulse
- File browser for saved curves with on-device review (graph + classification + delete)
- SD card logging in RASP `.eng` format with date-stamped filenames
- SD card files hosted over wifi. Browse to IP address displayed on home screen.
- WiFi auto-connect and NTP time sync at boot; worldwide timezone presets.
- Settings persisted to both EEPROM and SD `/settings.ini` (SD wins on load)

### Video demo
[![Video Demo](https://img.youtube.com/vi/MpfrThASmfk/hqdefault.jpg)](https://youtube.com/shorts/MpfrThASmfk?si=fK43k_C76Hq3I9Sh)

## ADC Selection

In src/config.h:

- LOADCELL_ADC = SCALE_BACKEND_HX711
- or LOADCELL_ADC = SCALE_BACKEND_ADS1220

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

### ADC Option 2: ADS1220 (Preferred)

- ADS1220_CS -> GPIO 17
- ADS1220_SCLK -> GPIO 22
- ADS1220_MOSI -> GPIO 16
- ADS1220_MISO -> GPIO 35
- ADS1220_DRDY -> GPIO 27
- ADS1220 AVDD/DVDD -> 3.3V
- ADS1220 GND -> GND
- ADS1220 REFNO -> GND
- ADS1220 REFPO -> 3.3V

Load cell wiring for ADS1220 depends on breakout labels, but differential signal should be wired as:
- Signal+ -> AIN0 (Green)
- Signal- -> AIN1 (White)
- Excitation- -> GND (Black)
- Excitation+ -> 3.3V (Red)


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
- Start Threshold (N) - capture trigger force threshold
- Pre / Post Capture (s)
- End Holdoff (ms) - Time to wait if force hits 0 during main thrust before deciding burn is over. Prevents noise, or vibration from ending main thrust early.
- Ejection Wait (s) - Max wait time for ejection delay spike. Set 0 to disable.
- Ejection Detect (N) - Minimum force to trigger detection of fired ejection charge.
- Max Burn (s) - Maximum allowable burn time.
- Time Zone - worldwide presets with DST rules

WiFi credentials are entered via Settings → WiFi (scan + on-screen keyboard) and persist to SD `/settings.ini`. They may also be saved to the SD card directly. On boot, if an SSID is saved the device auto-connects in the background and updates the home-screen clock once NTP returns valid time. Boot is never delayed waiting on the network.

## Calibration
Calibration is mandatory, and should be done periodically, or any time a change is made. Each load cell, and ADC combination is unique. The default calibration factor is for my specific ADC chip, and 50KG load cell.

To calibrate, go to Settings -> Calibrate.
- With no load on the cell, press `Tare`. 
- Press `Weight`, enter the known weight of a and object.
- Place the weighed object on the load cell.
- Allow the Mass value to settle, and press `calibrate`.

Calibration factor will be shown on screen, and saved to EEPROM, and SD card.

## Project Files

- `platformio.ini` - PlatformIO environment and dependencies
- `src/config.h` - Build-time pin and feature defines
- `src/globals.h` - Shared types, extern declarations, and function prototypes
- `src/main.cpp` - Slim glue: global definitions, `setup()` and `loop()`
- `src/scale.cpp` - Load-cell backend and filtering (HX711/ADS1220)
- `src/storage.cpp` - Settings persistence (EEPROM/SD) and file I/O
- `src/netclock.cpp` - WiFi, NTP sync and timezone handling
- `src/ui_draw.cpp` - Drawing primitives, touch handling and keyboard
- `src/ui_screens.cpp` - Per-screen draw routines and UI layout
- `src/capture.cpp` - Capture state machine and `.eng` export