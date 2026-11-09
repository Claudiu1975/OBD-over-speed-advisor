/*
  ESP32-S3 Truck Speed Advisor (CAN / J1939 / FMS version)

  Reads the vehicle speed from the truck's CAN bus (SAE J1939, 250 kbit/s, 29-bit IDs)
  through an external 3.3 V CAN transceiver and the ESP32-S3 built-in TWAI controller.
  Alerts with a buzzer and a flashing display when a configurable limit is exceeded.

  The CAN controller runs in LISTEN-ONLY mode: it never transmits and never
  acknowledges frames, so it cannot disturb the vehicle bus.

  Speed messages used (SAE J1939 / FMS standard):
    CCVS  PGN 65265 (0xFEF1)  bytes 2-3  wheel-based vehicle speed, 1/256 km/h per bit
    TCO1  PGN 65132 (0xFE6C)  bytes 7-8  tachograph vehicle speed,  1/256 km/h per bit

  Disclaimer: Hobby project only. Not a certified speedometer or speed limiter.
  Use at your own risk. See README.md for the full disclaimer and safety notes.
*/

#include <Arduino.h>
#include <Preferences.h>
#include <stdio.h>
#include "driver/twai.h"

// ========== DISPLAY SELECTION ==========
// 0 = SSD1306 128x32 I2C
// 1 = SSD1306 128x64 I2C
// 2 = ST7735S SPI
// 3 = ST7789V SPI
#define DISPLAY_TYPE 1

#if DISPLAY_TYPE == 0 || DISPLAY_TYPE == 1
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
#elif DISPLAY_TYPE == 2
  #include <Adafruit_GFX.h>
  #include <Adafruit_ST7735.h>
#elif DISPLAY_TYPE == 3
  #include <Adafruit_GFX.h>
  #include <Adafruit_ST7789.h>
#endif

// ========== SPEED SOURCE ==========
// 0 = auto: wheel-based speed (CCVS), falls back to tachograph speed (TCO1)
// 1 = CCVS only
// 2 = TCO1 only
#define SPEED_SOURCE        0

// Only accept speed frames from one source address (0x00-0xFD). 0xFF = accept any.
#define SPEED_SOURCE_ADDR   0xFF

// No valid speed frame for this long -> speed is treated as unknown (no alert).
#define CAN_TIMEOUT_MS      2000

// 1 = bench test without a truck: simulated speed ramp 0..110 km/h, no CAN needed.
#define SIMULATE_SPEED      0

// 1 = print every new CAN source address / PGN seen on the bus to the Serial Monitor.
// Use it on first connection to check that the bus is alive and speed frames are present.
#define DEBUG_PGN_LOG       1

// ========== PINS ==========
#define CAN_RX_PIN      16   // transceiver RXD (R) -> ESP32 RX
#define CAN_TX_PIN      17   // ESP32 TX -> transceiver TXD (D). Never driven: listen-only.
#define BUZZER_PIN      4
#define BTN_UP_PIN      5
#define BTN_DOWN_PIN    6

#if DISPLAY_TYPE == 0 || DISPLAY_TYPE == 1
  #define I2C_SDA       8
  #define I2C_SCL       9
  #define OLED_ADDR     0x3C
#else
  #define TFT_MOSI      11
  #define TFT_SCK       12
  #define TFT_CS        10
  #define TFT_DC        7
  #define TFT_RST       15
#endif

// ========== THRESHOLD & TIMING ==========
float thresholdKmh = 90.0f;
const float STEP = 0.5f;

const uint32_t ALERT_PHASE1_MS = 45000;   // 45 s -> start increasing beep frequency
const uint32_t ALERT_PHASE2_MS = 55000;   // 55 s -> continuous tone

// ========== J1939 ==========
const uint32_t PGN_CCVS = 0xFEF1;   // 65265
const uint32_t PGN_TCO1 = 0xFE6C;   // 65132

// ========== OBJECTS ==========
Preferences prefs;

#if DISPLAY_TYPE == 0
  Adafruit_SSD1306 display(128, 32, &Wire, -1);
#elif DISPLAY_TYPE == 1
  Adafruit_SSD1306 display(128, 64, &Wire, -1);
#elif DISPLAY_TYPE == 2
  Adafruit_ST7735 display(TFT_CS, TFT_DC, TFT_RST);
#elif DISPLAY_TYPE == 3
  Adafruit_ST7789 display(TFT_CS, TFT_DC, TFT_RST);
#endif

// ========== STATE ==========
enum Mode { MODE_SET, MODE_RUN };
Mode currentMode = MODE_SET;
unsigned long lastButtonMs = 0;
unsigned long alertStartMs = 0;
bool alertActive = false;
bool displayOn = true;
unsigned long lastBeepToggle = 0;
bool buzzerState = false;

bool canStarted = false;
bool ccvsValid = false, tco1Valid = false;
float ccvsKmh = 0, tco1Kmh = 0;
unsigned long ccvsMs = 0, tco1Ms = 0;
unsigned long frameCount = 0;
bool forceRedraw = true;

// Current speed as selected by getSpeed()
float curSpeed = 0;
bool curSpeedValid = false;
const char *curSource = "--";

// ---------------------------------------------------------------------------
// CAN
// ---------------------------------------------------------------------------
bool canBegin() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
  g.rx_queue_size = 64;   // J1939 buses are busy; keep a deep queue
  twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    Serial.println(F("TWAI driver install failed"));
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println(F("TWAI start failed"));
    return false;
  }
  Serial.println(F("TWAI started: 250 kbit/s, listen-only"));
  return true;
}

// J1939 speed fields are 1/256 km/h per bit. Values above 0xFAFF are
// "error" / "not available" markers.
static bool decodeSpeed(uint8_t lo, uint8_t hi, float &kmh) {
  uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);
  if (raw > 0xFAFF) return false;
  kmh = raw / 256.0f;
  return true;
}

#if DEBUG_PGN_LOG
static uint32_t seenKeys[96];
static uint8_t seenCount = 0;

static void logIfNew(uint32_t pgn, uint8_t sa) {
  uint32_t key = (pgn << 8) | sa;
  for (uint8_t i = 0; i < seenCount; i++) {
    if (seenKeys[i] == key) return;
  }
  if (seenCount < (sizeof(seenKeys) / sizeof(seenKeys[0]))) {
    seenKeys[seenCount++] = key;
    Serial.printf("New J1939 frame: PGN %lu (0x%05lX) from SA 0x%02X\n",
                  (unsigned long)pgn, (unsigned long)pgn, sa);
  }
}
#endif

void pollCan() {
#if SIMULATE_SPEED
  return;
#else
  if (!canStarted) return;

  // Recover from bus-off / stopped states
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 1000) {
    lastCheck = millis();
    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK) {
      if (st.state == TWAI_STATE_BUS_OFF) twai_initiate_recovery();
      else if (st.state == TWAI_STATE_STOPPED) twai_start();
    }
  }

  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) {
    if (!m.extd || m.rtr) continue;            // J1939 uses 29-bit data frames only
    frameCount++;

    uint32_t pgn = (m.identifier >> 8) & 0x3FFFF;
    uint8_t sa = m.identifier & 0xFF;

#if DEBUG_PGN_LOG
    logIfNew(pgn, sa);
#endif

#if SPEED_SOURCE_ADDR != 0xFF
    if (sa != SPEED_SOURCE_ADDR) continue;
#endif

    float v;
    if (pgn == PGN_CCVS && m.data_length_code >= 3) {
      if (decodeSpeed(m.data[1], m.data[2], v)) {
        ccvsKmh = v; ccvsMs = millis(); ccvsValid = true;
      }
    } else if (pgn == PGN_TCO1 && m.data_length_code >= 8) {
      if (decodeSpeed(m.data[6], m.data[7], v)) {
        tco1Kmh = v; tco1Ms = millis(); tco1Valid = true;
      }
    }
  }
#endif
}

// Chooses the speed to use. Returns false when no fresh speed is available.
bool getSpeed(float &kmh, const char *&src) {
#if SIMULATE_SPEED
  // Triangle wave 0 -> 110 -> 0 km/h over 60 s
  float ph = (millis() % 60000) / 30000.0f;
  kmh = (ph < 1.0f ? ph : 2.0f - ph) * 110.0f;
  src = "SIM";
  return true;
#else
  unsigned long now = millis();
  bool ccvsFresh = ccvsValid && (now - ccvsMs) < CAN_TIMEOUT_MS;
  bool tco1Fresh = tco1Valid && (now - tco1Ms) < CAN_TIMEOUT_MS;

#if SPEED_SOURCE == 1
  tco1Fresh = false;
#elif SPEED_SOURCE == 2
  ccvsFresh = false;
#endif

  if (ccvsFresh) { kmh = ccvsKmh; src = "CCVS"; return true; }
  if (tco1Fresh) { kmh = tco1Kmh; src = "TCO1"; return true; }
  return false;
#endif
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);

  prefs.begin("truckadv", false);
  thresholdKmh = prefs.getFloat("limit", thresholdKmh);
  if (thresholdKmh < 5.0f || thresholdKmh > 200.0f) thresholdKmh = 90.0f;

#if !SIMULATE_SPEED
  canStarted = canBegin();
#endif

#if DISPLAY_TYPE == 0 || DISPLAY_TYPE == 1
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.setTextColor(SSD1306_WHITE);
#else
  #if DISPLAY_TYPE == 2
    display.initR(INITR_BLACKTAB);  // common for many ST7735S; try other INITR_ if colors are wrong
  #else
    display.init(240, 320);         // adjust resolution for your ST7789V module
  #endif
  display.setRotation(1);
  display.fillScreen(ST77XX_BLACK);
  display.setTextColor(ST77XX_WHITE);
#endif

  display.setTextSize(1);
  lastButtonMs = millis();
  showSetScreen();
}

void loop() {
  pollCan();
  curSpeedValid = getSpeed(curSpeed, curSource);

  handleButtons();

  if (currentMode == MODE_SET && millis() - lastButtonMs > 5000) {
    currentMode = MODE_RUN;                 // 5 s idle -> run mode
    forceRedraw = true;
    prefs.putFloat("limit", thresholdKmh);  // remember the limit across power cycles
  }

  updateAlert();
  updateDisplay();

#if DEBUG_PGN_LOG && !SIMULATE_SPEED
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 5000) {
    lastPrint = millis();
    if (curSpeedValid) {
      Serial.printf("Speed %.1f km/h (%s), %lu frames\n", curSpeed, curSource, frameCount);
    } else {
      Serial.printf("No speed data, %lu frames received\n", frameCount);
    }
  }
#endif

  delay(20);
}

void handleButtons() {
  static bool upPrev = HIGH, downPrev = HIGH;
  bool upNow = digitalRead(BTN_UP_PIN);
  bool downNow = digitalRead(BTN_DOWN_PIN);

  if (upNow == LOW && upPrev == HIGH) {
    thresholdKmh += STEP;
    if (thresholdKmh > 200.0f) thresholdKmh = 200.0f;
    lastButtonMs = millis();
    currentMode = MODE_SET;
    showSetScreen();
  }
  if (downNow == LOW && downPrev == HIGH) {
    thresholdKmh -= STEP;
    if (thresholdKmh < 5.0f) thresholdKmh = 5.0f;
    lastButtonMs = millis();
    currentMode = MODE_SET;
    showSetScreen();
  }
  upPrev = upNow;
  downPrev = downNow;
}

void updateAlert() {
  // No fresh speed data -> never alert
  bool over = curSpeedValid && curSpeed > thresholdKmh;

  if (over && !alertActive) {
    alertActive = true;
    alertStartMs = millis();
  } else if (!over) {
    alertActive = false;
    displayOn = true;
    digitalWrite(BUZZER_PIN, LOW);
    buzzerState = false;
  }

  if (!alertActive) return;

  unsigned long elapsed = millis() - alertStartMs;
  unsigned long interval;

  if (elapsed < ALERT_PHASE1_MS) {
    interval = 1000;          // 1 Hz short beeps
  } else if (elapsed < ALERT_PHASE2_MS) {
    // linear increase from 1000 ms down to ~100 ms
    float t = (elapsed - ALERT_PHASE1_MS) / 10000.0f;
    interval = 1000 - (int)(t * 900);
    if (interval < 100) interval = 100;
  } else {
    interval = 0;             // continuous
  }

  if (interval == 0) {
    digitalWrite(BUZZER_PIN, HIGH);
    displayOn = (millis() / 200) % 2 == 0;  // fast flash
  } else {
    if (millis() - lastBeepToggle >= interval) {
      buzzerState = !buzzerState;
      digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
      lastBeepToggle = millis();
      displayOn = buzzerState;
    }
  }
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------
const char *canStatusText() {
#if SIMULATE_SPEED
  return "SIM";
#else
  if (!canStarted) return "CAN ERR";
  return curSpeedValid ? "CAN OK" : "NO DATA";
#endif
}

void showSetScreen() {
#if DISPLAY_TYPE == 0
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(20, 0);
  display.print(F("LIMIT"));
  display.setTextSize(2);
  char buf[8];
  snprintf(buf, sizeof(buf), "%.1f", thresholdKmh);
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((128 - w) / 2, 12);
  display.print(buf);
  display.display();
#elif DISPLAY_TYPE == 1
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(28, 4);
  display.print(F("SPEED LIMIT"));
  display.drawLine(10, 16, 118, 16, SSD1306_WHITE);

  display.setTextSize(3);
  char buf[8];
  snprintf(buf, sizeof(buf), "%.1f", thresholdKmh);
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((128 - w) / 2, 22);
  display.print(buf);

  display.setTextSize(1);
  display.setCursor(46, 50);
  display.print(F("km/h"));
  display.setCursor(4, 56);
  display.print(canStatusText());
  display.display();
#else
  display.fillScreen(ST77XX_BLACK);
  display.setTextSize(2);
  display.setCursor(30, 10);
  display.print(F("LIMIT"));
  display.setTextSize(4);
  char buf[8];
  snprintf(buf, sizeof(buf), "%.1f", thresholdKmh);
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((display.width() - w) / 2, 50);
  display.print(buf);
  display.setTextSize(2);
  display.setCursor(70, 100);
  display.print(F("km/h"));
  display.setCursor(10, display.height() - 20);
  display.print(canStatusText());
#endif
}

void updateDisplay() {
  if (currentMode == MODE_SET) return;

  // Redraw only when something visible changed (avoids TFT flicker)
  int speedInt = curSpeedValid ? (int)(curSpeed + 0.5f) : -1;
  static int lastSpeedInt = -2;
  static bool lastOn = false, lastValid = false;
  static float lastLimit = -1;
  static const char *lastSrc = nullptr;
  if (!forceRedraw && speedInt == lastSpeedInt && displayOn == lastOn &&
      curSpeedValid == lastValid && thresholdKmh == lastLimit && curSource == lastSrc) {
    return;
  }
  forceRedraw = false;
  lastSpeedInt = speedInt; lastOn = displayOn; lastValid = curSpeedValid;
  lastLimit = thresholdKmh; lastSrc = curSource;

  char spdBuf[8];
  if (curSpeedValid) snprintf(spdBuf, sizeof(spdBuf), "%d", speedInt);
  else snprintf(spdBuf, sizeof(spdBuf), "--");
  bool show = !alertActive || displayOn;

#if DISPLAY_TYPE == 0
  display.clearDisplay();
  if (show) {
    display.setTextSize(2);
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(spdBuf, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((128 - w) / 2, 4);
    display.print(spdBuf);
    display.setTextSize(1);
    display.setCursor(0, 24);
    display.print(curSource);
    display.print(F(" L"));
    display.print(thresholdKmh, 0);
  }
  display.display();
#elif DISPLAY_TYPE == 1
  display.clearDisplay();
  if (show) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(canStatusText());
    display.setCursor(128 - 6 * strlen(curSource), 0);
    display.print(curSource);
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

    display.setTextSize(3);
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(spdBuf, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((128 - w) / 2, 18);
    display.print(spdBuf);

    display.setTextSize(1);
    display.setCursor(50, 46);
    display.print(F("km/h"));
    display.setCursor(0, 56);
    display.print(F("Limit "));
    display.print(thresholdKmh, 1);
  }
  display.display();
#else
  display.fillScreen(ST77XX_BLACK);
  if (show) {
    display.setTextSize(1);
    display.setCursor(5, 5);
    display.print(canStatusText());
    display.setCursor(display.width() - 40, 5);
    display.print(curSource);

    display.setTextSize(6);
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(spdBuf, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((display.width() - w) / 2, 40);
    display.print(spdBuf);

    display.setTextSize(2);
    display.setCursor((display.width() - 60) / 2, 100);
    display.print(F("km/h"));
    display.setCursor(10, display.height() - 30);
    display.print(F("Limit "));
    display.print(thresholdKmh, 1);
  }
#endif
}
