#pragma once

// Hardware pin map and tunable defaults.

// --- ADC backend selection ---
#define SCALE_BACKEND_HX711    1
#define SCALE_BACKEND_ADS1220  2

#define LOADCELL_ADC SCALE_BACKEND_ADS1220

// --- HX711 pins (kept for builds that select HX711) ---
#define HX711_DOUT 22
#define HX711_SCK  27

// --- Bit-banged ADS1220 pins ---
#define ADS1220_CS    17
#define ADS1220_SCLK  22
#define ADS1220_MOSI  16
#define ADS1220_MISO  35
#define ADS1220_DRDY  27

// --- ILI9341 display ---
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST  -1
#define TFT_BL   21
#define TFT_MOSI 13
#define TFT_MISO 12
#define TFT_SCLK 14

// --- SD card (HSPI) ---
#define SD_CS   5
#define SD_SCK  18
#define SD_MISO 19
#define SD_MOSI 23

// --- XPT2046 touch ---
#define TOUCH_CS   33
#define TOUCH_IRQ  36
#define TOUCH_CLK  25
#define TOUCH_MOSI 32
#define TOUCH_MISO 39

// --- I2C (RTC, currently unused) ---
#define I2C_SDA 4
#define I2C_SCL 13

#define NTP_SERVER         "pool.ntp.org"
#define SETTINGS_FILENAME  "/settings.ini"
#define EEPROM_MAGIC       0x52415354
#define EEPROM_SIZE        1024

#define DEFAULT_CAL_FACTOR                   -67668.9f
#define MIN_REASONABLE_CAL_FACTOR_ABS_HX711  50000.0f
#define MIN_REASONABLE_CAL_FACTOR_ABS_ADS1220 100.0f

#define CAPTURE_MAX_SAMPLES      4800
#define HX711_SAMPLE_RATE_SPS      80
#define ADS1220_SAMPLE_RATE_SPS   600

// Capture / detection defaults.
#define DEFAULT_START_THRESHOLD       2.0f
#define CAPTURE_PRE_MS                150
#define CAPTURE_POST_MS               200

#define DEFAULT_EJECTION_WAIT_S       10.0f
#define DEFAULT_EJECTION_DETECT_N     1.0f
#define DEFAULT_THRUST_END_HOLDOFF_MS 90
#define DEFAULT_MAX_BURN_MS           5000

// Wiring polarity for the load cell.
#define LOAD_POLARITY            -1.0f
#define TARE_AVERAGE_DURATION_MS 1500UL
#define SCALE_UI_RAW_SAMPLES     8
#define DEBUG_FEATURES_DEFAULT   0

// Parameters screen layout.
#define PARAMS_ROW_H         34
#define PARAMS_CONTENT_TOP   30
#define PARAMS_ROWS          8
#define PARAMS_ROWS_PER_PAGE 5
#define PARAMS_PAGE_COUNT    ((PARAMS_ROWS + PARAMS_ROWS_PER_PAGE - 1) / PARAMS_ROWS_PER_PAGE)

#define SAVED_FILE_MAX       128

#define paramRowOnPage(n)   (paramsPage == ((n) / PARAMS_ROWS_PER_PAGE))
#define paramRowY(n)        (PARAMS_CONTENT_TOP + ((n) % PARAMS_ROWS_PER_PAGE) * PARAMS_ROW_H)
#define paramRowVisible(n)  (paramRowOnPage(n))
