/*
  ESP32 Paddle COF Tester (continuous bidirectional test, NAU7802 load cell amp)
  - DRV8825 + NEMA17
  - NAU7802 load cell amp (I2C)
  - SSD1306 OLED (I2C)
  - Button: START -> INPUT_PULLUP, wire to GND (see BTN_START define)
  - Limit switch -> INPUT_PULLUP, active-LOW, wire to GND (see PIN_LIMIT define)
  - Pin assignments vary by ESP32 variant - see USER CONFIG section below
  - On normal boot: homes to limit switch automatically if not already pressed
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <PaddleDNA.h>
#include <math.h>
#include "CofCalculation.h"

// ----------------------------- USER CONFIG ----------------------------------
// NOTE: Pin assignments below match PCB schematic (ESP32-S3-ZERO)
// ESP32-S3 Pin Restrictions: Avoid GPIO 26-32 (reserved/problematic)
// See schematic for hardware pullups and connections

#define I2C_SDA 12     // Shared I2C bus (OLED + RFID)
#define I2C_SCL 11     // Shared I2C bus (OLED + RFID)

#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_ADDR    0x3C

#define NAU_SDA  5   // NAU7802 I2C data (Wire1)
#define NAU_SCL  6    // NAU7802 I2C clock (Wire1)

#define PIN_STEP  7   // DRV8825 step pin
#define PIN_DIR   2  // DRV8825 direction pin
#define PIN_EN    3  // DRV8825 enable (Active LOW)

#define PIN_LIMIT  4  // Limit switch input (active-LOW with 10K pullup)
#define LIMIT_ACTIVE_LOW 1

#define BTN_START  10 // Start button (active-LOW with 10K pullup)
                      // All functions on one button: press=run, hold=tare, boot+hold=calibrate
                      // Hold 3s during motion/test/cal = abort

#define RGB_LED_PIN 21   // Onboard RGB LED (ESP32-S3-Zero)

const int   FULL_STEPS_PER_REV = 200;     // 1.8° motor
const float MICROSTEP           = 16.0;   // 1/16 microstepping
const float REVS_TOTAL          = 19.0;   // 6.0 inches total
const float INCHES_TOTAL        = 6.0;
const float REVS_PER_INCH       = REVS_TOTAL / INCHES_TOTAL;
const float STEPS_PER_REV       = FULL_STEPS_PER_REV * MICROSTEP;
const float STEPS_PER_INCH      = STEPS_PER_REV * REVS_PER_INCH;

const float SEG_LOWER_IN   = 2.5;  // ignore: lowering
const float SEG_NOISE_IN   = 0.0;  // (removed: settling now handled by SEG_TRIM_IN)
const float SEG_MEASURE_IN = 3.0;  // total measurement segment (includes trim regions)
const float SEG_TRIM_IN    = 0.25; // settle/trim at start and end (actual measurement: 2.5")

const int    STEP_PULSE_US   = 150; // motion speed (lower = faster)
const int    HOME_STEP_US    = 300; // homing speed
const int    BACKOFF_STEPS   = 600; // homing backoff
const bool   DIR_FORWARD     = true;
const bool   DIR_HOME_TOWARD_LIMIT = !DIR_FORWARD;

const uint32_t DEBOUNCE_MS   = 30;
const uint32_t LONG_PRESS_MS = 1200;
const uint32_t ABORT_HOLD_MS = 3000;  // Hold START 3s during motion to abort

const float CAL_WEIGHT_LB    = 2.883;   // calibration weight
const float NORMAL_FORCE_LB  = 2.59;  // test normal force
const int HX_SAMPLES_TARE    = 20;      // averaging for tare
const int HX_SAMPLES_MEAS    = 5;       // (unused by non-blocking read)

// Machine identification for PaddleDNA (update these for each machine)
const int MACHINE_ID = 00002;  // Machine identifier for display

// Machine UUID: Generate unique UUID per machine
// Example UUID for Friction Tester #2: a1b2c3d4-e5f6-7890-abcd-ef1234567890
const uint8_t MACHINE_UUID[16] = {
  0x68, 0xdf, 0x84, 0x98, 0x85, 0x73, 0x46, 0xc6,
  0xa8, 0xb8, 0xfe, 0xdc, 0xc0, 0xdf, 0x07, 0x36
};

// Stub private key (32 bytes of 0xCC for MVP - crypto is stub implementation)
const uint8_t PRIVATE_KEY[32] = {
  0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
  0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
  0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
  0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC
};

// Hardcoded timestamp for 2026-01-12 00:00:00 UTC
const uint32_t FIXED_TIMESTAMP = 1768176000;
// ----------------------------------------------------------------------------

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire);
NAU7802 nau;
Preferences prefs;
Adafruit_NeoPixel rgbLed(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// PaddleDNA components
PaddleDNA::NFC nfc;
PaddleDNA::Crypto crypto;
PaddleDNA::MeasurementAccumulator* accumulator = nullptr;

bool  g_hasResult = false;
float g_lastAvgLb = 0.0f;
float g_lastCOF   = 0.0f;

// ======================== DUAL-CORE ARCHITECTURE ============================
// Motion phases for tracking
enum MotionPhase {
  PHASE_NONE,
  PHASE_LOWERING,
  PHASE_MEASURING_FWD,
  PHASE_MEASURING_REV,
  PHASE_RETURNING,
  PHASE_HOMING
};

// Motion commands
enum MotionCommand {
  CMD_NONE,
  CMD_HOME,
  CMD_HOME_FORCE,  // Home without abort checking (used after abort)
  CMD_MOVE,
  CMD_MEASURE_MOVE,
  CMD_ENABLE,
  CMD_DISABLE
};

struct MotionRequest {
  MotionCommand cmd;
  long steps;
  bool direction;
  int pulseUs;
  MotionPhase phase;
};

// Global state (shared between cores)
volatile MotionPhase g_currentPhase = PHASE_NONE;
volatile bool g_collectSamples = false;  // Signal Core 0 to sample
volatile bool g_abortRequested = false;  // Abort flag (set by Core 1 button check)
volatile uint32_t g_abortBtnDownAt = 0;  // Tracks when abort button was first pressed

// Sample storage (Core 0 writes, Core 1 never touches)
#define MAX_SAMPLES_PER_PASS 2000
float g_fwdSamples[MAX_SAMPLES_PER_PASS];
float g_revSamples[MAX_SAMPLES_PER_PASS];
volatile long g_fwdSampleCount = 0;
volatile long g_revSampleCount = 0;

// Inter-core communication
QueueHandle_t motionCommandQueue = NULL;
SemaphoreHandle_t motionCompleteSemaphore = NULL;
TaskHandle_t forceSamplingTaskHandle = NULL;
// ============================================================================

// =============================== UI STATE ===================================
// Single-button device, mono 128x64 OLED. Layout grid:
//   header  y=0..11   (machine id + status pill)
//   body    y=12..51  (40px primary content)
//   footer  y=52..63  (button-hint line)
// All ui_* state is read/written by Core 0; g_lastForceSampleLb is written
// by the force-sampling task (also Core 0) and read by ui_tick — volatile.
// Progress is time-based on Core 0, so no inter-core publishing is needed.

enum UiPhase {
  UI_BOOT,        // splash; ui_tick is a no-op
  UI_IDLE,
  UI_HOMING,      // animated dots, no progress bar
  UI_RUNNING,     // progress bar + label/force
  UI_DONE,        // result + NFC prompt
  UI_SAVED,
  UI_ABORTED,
  UI_CAL,
  UI_ERROR,
};

UiPhase     g_uiPhase = UI_BOOT;
const char* g_uiLabel = "";
uint8_t     g_uiPhaseStartPct = 0;
uint8_t     g_uiPhaseEndPct   = 0;
uint32_t    g_phaseStartMs    = 0;     // wall-clock time at phase start
uint32_t    g_phaseDurationMs = 0;     // expected phase duration
bool        g_uiShowForce     = false;
uint8_t     g_uiProgressPct   = 0;
float       g_uiCofValue      = 0.0f;
int         g_uiCalStep       = 0;
const char* g_uiCalLine1      = "";
const char* g_uiCalLine2      = "";
const char* g_uiCalFooter     = "";
uint8_t     g_uiAnimFrame     = 0;
uint32_t    g_uiLastDrawMs    = 0;

volatile float g_lastForceSampleLb = 0.0f;
// ============================================================================

const char* PREFS_NAMESPACE = "cof";
const char* KEY_CAL         = "calib";
const char* KEY_TARE        = "tare";

float g_calibration = 1000.0f; // counts per lb
long  g_tareRaw     = 0;       // tare offset (raw counts)

struct Btn {
  uint8_t pin;
  bool last;
  uint32_t lastChange;
  uint32_t downAt;
  bool longSent;
};
Btn btnStart{BTN_START, true, 0, 0, false};

struct RunResult { float avgFrictionLb; float cof; float avgBias; };

// Prototypes
void   stepperEnable(bool on);
void   setDir(bool forward);
void   doStepBlocking(int pulseUs);
bool   limitHit();
void   saveCalibration();
void   loadCalibration();
long   nauReadRawAvg(int n);
float  rawToPounds(long raw);
void   doCalibration3lb();
void   homeToLimit();
void   homeToLimitSafe();
void   homeToLimitForce();
bool   checkAbortButton();
void   moveStepsBlocking(long steps, bool forward, int pulseUs);
RunResult runTest();
bool   readButton(Btn& b, bool& shortPress, bool& longPress);
void   setLED(uint8_t r, uint8_t g, uint8_t b);
void   ledOff();
void   rainbowCycle(int durationMs);
void   pulseLED(uint8_t r, uint8_t g, uint8_t b, int times, int pulseMs);
uint32_t colorWheel(byte pos);
bool   writeToRFID(float cofValue);
void   dumpTestDataCSV();

// UI module
void   ui_drawHeader(const char* statusPill);
void   ui_drawFooter(const char* hint);
void   ui_screenSplash();
void   ui_screenIdle();
void   ui_screenHoming();
void   ui_screenRunning();
void   ui_screenDone();
void   ui_screenSaved();
void   ui_screenAborted(const char* primaryLine);
void   ui_screenCalStep();
void   ui_screenError(const char* title, const char* line1, const char* line2);
void   ui_tick();

// Dual-core function prototypes
void   mainTask(void* parameter);
void   motionTask(void* parameter);
void   forceSamplingTask(void* parameter);
void   executePureMove(long steps, bool forward, int pulseUs);
bool   executeHome();
bool   requestMotion(MotionRequest req, uint32_t timeoutMs = 60000);
void   homeToLimitSafe();
void   moveStepsBlockingSafe(long steps, bool forward, int pulseUs);

// ----------------------------- Utils ----------------------------------------
void stepperEnable(bool on) { digitalWrite(PIN_EN, on ? LOW : HIGH); }
void setDir(bool forward)   { digitalWrite(PIN_DIR, forward ? HIGH : LOW); }

void doStepBlocking(int pulseUs) {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(pulseUs);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(pulseUs);
}

bool limitHit() {
  int val = digitalRead(PIN_LIMIT);
  return LIMIT_ACTIVE_LOW ? (val == LOW) : (val == HIGH);
}

// ----------------------------- RGB LED Functions ----------------------------
void setLED(uint8_t r, uint8_t g, uint8_t b) {
  rgbLed.setPixelColor(0, rgbLed.Color(r, g, b));
  rgbLed.show();
}

void ledOff() {
  rgbLed.clear();
  rgbLed.show();
}

uint32_t colorWheel(byte pos) {
  // Color wheel helper: 0-255 maps through rainbow
  pos = 255 - pos;
  if (pos < 85) {
    return rgbLed.Color(255 - pos * 3, 0, pos * 3);
  } else if (pos < 170) {
    pos -= 85;
    return rgbLed.Color(0, pos * 3, 255 - pos * 3);
  } else {
    pos -= 170;
    return rgbLed.Color(pos * 3, 255 - pos * 3, 0);
  }
}

void rainbowCycle(int durationMs) {
  uint32_t startTime = millis();
  while (millis() - startTime < durationMs) {
    byte wheelPos = (byte)(((millis() - startTime) * 255) / durationMs);
    rgbLed.setPixelColor(0, colorWheel(wheelPos));
    rgbLed.show();
    delay(10);
  }
}

void pulseLED(uint8_t r, uint8_t g, uint8_t b, int times, int pulseMs) {
  for (int i = 0; i < times; i++) {
    // Fade in
    for (int brightness = 0; brightness <= 255; brightness += 15) {
      uint8_t br = (r * brightness) / 255;
      uint8_t bg = (g * brightness) / 255;
      uint8_t bb = (b * brightness) / 255;
      setLED(br, bg, bb);
      delay(pulseMs / 34); // 34 steps total (17 up, 17 down)
    }
    // Fade out
    for (int brightness = 255; brightness >= 0; brightness -= 15) {
      uint8_t br = (r * brightness) / 255;
      uint8_t bg = (g * brightness) / 255;
      uint8_t bb = (b * brightness) / 255;
      setLED(br, bg, bb);
      delay(pulseMs / 34);
    }
    ledOff();
    if (i < times - 1) delay(100); // pause between pulses
  }
}

// ============================== UI MODULE ===================================
// All screens follow the same grid: 12px header / 40px body / 12px footer.
// Each ui_screen* clears, draws header/body/footer, calls oled.display().
// ui_tick() is invoked from requestMotion()'s semaphore polling loop so the
// progress bar and homing animation stay live without a separate UI task.

static int ui_centerX(const char* s, uint8_t size) {
  uint8_t charW = (size == 2) ? 12 : (size == 3 ? 18 : 6);
  int w = (int)strlen(s) * charW;
  int x = (OLED_WIDTH - w) / 2;
  return x < 0 ? 0 : x;
}

void ui_drawHeader(const char* statusPill) {
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 2);
  oled.print(F("COF Tester"));

  if (statusPill && statusPill[0]) {
    int textPx = (int)strlen(statusPill) * 6;
    int pillW  = textPx + 4;
    int pillX  = OLED_WIDTH - pillW;
    oled.fillRect(pillX, 0, pillW, 11, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setCursor(pillX + 2, 2);
    oled.print(statusPill);
    oled.setTextColor(SSD1306_WHITE);
  }

  oled.drawLine(0, 11, OLED_WIDTH, 11, SSD1306_WHITE);
}

void ui_drawFooter(const char* hint) {
  if (!hint || !hint[0]) return;
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(ui_centerX(hint, 1), 55);
  oled.print(hint);
}

void ui_screenSplash() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(ui_centerX("PADDLE", 2), 14);
  oled.print(F("PADDLE"));
  oled.setCursor(ui_centerX("COF", 2), 32);
  oled.print(F("COF"));

  oled.setTextSize(1);
  char id[8];
  snprintf(id, sizeof(id), "FT-%03d", MACHINE_ID);
  oled.setCursor(ui_centerX(id, 1), 54);
  oled.print(id);

  oled.display();
}

void ui_screenIdle() {
  oled.clearDisplay();
  ui_drawHeader("READY");
  oled.setTextColor(SSD1306_WHITE);

  if (g_hasResult) {
    oled.setTextSize(1);
    oled.setCursor(ui_centerX("Last test", 1), 16);
    oled.print(F("Last test"));

    char cofStr[8];
    dtostrf(g_lastCOF, 1, 3, cofStr);
    oled.setTextSize(3);
    oled.setCursor(ui_centerX(cofStr, 3), 26);
    oled.print(cofStr);
  } else {
    oled.setTextSize(2);
    oled.setCursor(ui_centerX("PADDLE", 2), 18);
    oled.print(F("PADDLE"));
    oled.setCursor(ui_centerX("COF", 2), 36);
    oled.print(F("COF"));
  }

  ui_drawFooter("Press button to run");
  oled.display();
}

void ui_screenHoming() {
  oled.clearDisplay();
  ui_drawHeader("RUNNING");

  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(ui_centerX("Homing", 2), 22);
  oled.print(F("Homing"));

  // Animated dots: 0..3
  oled.setTextSize(1);
  uint8_t dots = g_uiAnimFrame & 0x03;
  oled.setCursor(58, 44);
  for (uint8_t i = 0; i < dots; i++) oled.print('.');

  ui_drawFooter("Hold to abort");
  oled.display();
}

void ui_screenRunning() {
  oled.clearDisplay();
  ui_drawHeader("RUNNING");
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);

  // Phase label (left) and percent (right) above the bar
  oled.setCursor(2, 14);
  oled.print(g_uiLabel);

  uint8_t pct = g_uiProgressPct > 100 ? 100 : g_uiProgressPct;
  char pctStr[6];
  snprintf(pctStr, sizeof(pctStr), "%u%%", pct);
  int pctPx = (int)strlen(pctStr) * 6;
  oled.setCursor(OLED_WIDTH - pctPx - 2, 14);
  oled.print(pctStr);

  // Progress bar
  const int barX = 4, barY = 26, barW = 120, barH = 10;
  oled.drawRect(barX, barY, barW, barH, SSD1306_WHITE);
  int fillW = ((barW - 2) * pct) / 100;
  if (fillW > 0) oled.fillRect(barX + 1, barY + 1, fillW, barH - 2, SSD1306_WHITE);

  // Live force readout (only during measurement phases)
  if (g_uiShowForce) {
    char line[24];
    snprintf(line, sizeof(line), "Force  %.3f lb", fabsf(g_lastForceSampleLb));
    oled.setCursor(ui_centerX(line, 1), 42);
    oled.print(line);
  }

  ui_drawFooter("Hold to abort");
  oled.display();
}

void ui_screenDone() {
  oled.clearDisplay();
  ui_drawHeader("TAP TAG");
  oled.setTextColor(SSD1306_WHITE);

  char cofStr[8];
  dtostrf(g_uiCofValue, 1, 3, cofStr);
  oled.setTextSize(3);
  oled.setCursor(ui_centerX(cofStr, 3), 14);
  oled.print(cofStr);

  oled.setTextSize(1);
  oled.setCursor(ui_centerX("Tap NFC tag", 1), 42);
  oled.print(F("Tap NFC tag"));

  ui_drawFooter("Hold button to skip");
  oled.display();
}

void ui_screenSaved() {
  oled.clearDisplay();
  ui_drawHeader("SAVED");
  oled.setTextColor(SSD1306_WHITE);

  char cofStr[8];
  dtostrf(g_uiCofValue, 1, 3, cofStr);
  oled.setTextSize(3);
  oled.setCursor(ui_centerX(cofStr, 3), 22);
  oled.print(cofStr);

  oled.setTextSize(1);
  oled.setCursor(ui_centerX("saved to tag", 1), 54);
  oled.print(F("saved to tag"));

  oled.display();
}

void ui_screenAborted(const char* primaryLine) {
  oled.clearDisplay();
  ui_drawHeader("ABORTED");
  oled.setTextColor(SSD1306_WHITE);

  const char* line = (primaryLine && primaryLine[0]) ? primaryLine : "Cancelled";
  oled.setTextSize(2);
  oled.setCursor(ui_centerX(line, 2), 22);
  oled.print(line);

  oled.setTextSize(1);
  oled.setCursor(ui_centerX("Returning home", 1), 46);
  oled.print(F("Returning home"));

  oled.display();
}

void ui_screenCalStep() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);

  // Header: "CAL  Step N/2" (or just "CAL" when step==0)
  oled.setCursor(0, 2);
  if (g_uiCalStep > 0) {
    char header[16];
    snprintf(header, sizeof(header), "CAL  Step %d/2", g_uiCalStep);
    oled.print(header);
  } else {
    oled.print(F("CAL"));
  }
  oled.drawLine(0, 11, OLED_WIDTH, 11, SSD1306_WHITE);

  if (g_uiCalLine1 && g_uiCalLine1[0]) {
    oled.setCursor(2, 18);
    oled.print(g_uiCalLine1);
  }
  if (g_uiCalLine2 && g_uiCalLine2[0]) {
    oled.setCursor(2, 32);
    oled.print(g_uiCalLine2);
  }

  ui_drawFooter(g_uiCalFooter);
  oled.display();
}

void ui_screenError(const char* title, const char* line1, const char* line2) {
  oled.clearDisplay();
  ui_drawHeader("ERROR");
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);

  if (title && title[0]) {
    oled.setCursor(ui_centerX(title, 1), 18);
    oled.print(title);
  }
  if (line1 && line1[0]) {
    oled.setCursor(ui_centerX(line1, 1), 32);
    oled.print(line1);
  }
  if (line2 && line2[0]) {
    oled.setCursor(ui_centerX(line2, 1), 44);
    oled.print(line2);
  }
  oled.display();
}

// Transient redraw, called from requestMotion()'s polling loop.
// Time-based progress: pct = startPct + (elapsed/duration) * span.
// No throttle — the OLED I2C transfer (~25ms at 400kHz) is the natural
// pacing limit, which gives ~30 Hz when the bus is otherwise idle.
void ui_tick() {
  uint32_t now = millis();
  g_uiLastDrawMs = now;
  g_uiAnimFrame++;

  if (g_uiPhase == UI_RUNNING) {
    uint32_t elapsed = now - g_phaseStartMs;
    uint32_t duration = g_phaseDurationMs ? g_phaseDurationMs : 1;
    if (elapsed > duration) elapsed = duration;
    int span = (int)g_uiPhaseEndPct - (int)g_uiPhaseStartPct;
    g_uiProgressPct = g_uiPhaseStartPct + (uint8_t)((elapsed * span) / duration);
    ui_screenRunning();
  } else if (g_uiPhase == UI_HOMING) {
    ui_screenHoming();
  }
}

// ----------------------------- Calibration ----------------------------------
void saveCalibration() {
  prefs.begin(PREFS_NAMESPACE, false);
  prefs.putFloat(KEY_CAL, g_calibration);
  prefs.putLong(KEY_TARE, g_tareRaw);  // Use putLong to preserve full value
  prefs.end();
}

void loadCalibration() {
  prefs.begin(PREFS_NAMESPACE, true);
  float cal = prefs.getFloat(KEY_CAL, NAN);
  long tare  = prefs.getLong(KEY_TARE, 0);  // Use getLong to match putLong
  prefs.end();
  if (!isnan(cal)) g_calibration = cal;
  g_tareRaw = tare;
}

long nauReadRawAvg(int n) {
  long sum = 0;
  for (int i=0; i<n; i++) {
    while (!nau.available()) delay(1);
    sum += nau.getReading();
  }
  return sum / n;
}

float rawToPounds(long raw) {
  if (g_calibration == 0.0f) {
    Serial.println("ERROR: Division by zero - g_calibration is 0!");
    return 0.0f;
  }
  return (float)(raw - g_tareRaw) / g_calibration;
}

void doCalibration3lb() {
  g_abortRequested = false;
  g_abortBtnDownAt = 0;
  g_uiPhase = UI_CAL;

  // ---- Position sled at furthest end ----
  g_uiCalStep   = 0;
  g_uiCalLine1  = "Positioning sled";
  g_uiCalLine2  = "Stand clear";
  g_uiCalFooter = "Hold to abort";
  ui_screenCalStep();
  setLED(255, 150, 0);

  homeToLimitSafe();
  if (g_abortRequested) goto cal_abort;

  {
  const long calPositionSteps = lround((SEG_LOWER_IN + SEG_MEASURE_IN) * STEPS_PER_INCH);

  MotionRequest req;
  req.cmd = CMD_ENABLE;
  requestMotion(req, 1000);

  req.cmd = CMD_MOVE;
  req.steps = calPositionSteps;
  req.direction = DIR_FORWARD;
  req.pulseUs = STEP_PULSE_US;
  req.phase = PHASE_NONE;
  requestMotion(req);

  if (g_abortRequested) goto cal_abort;
  ledOff();

  // ---- Step 1: Tare (zero-load) ----
  g_uiCalStep   = 1;
  g_uiCalLine1  = "Remove all load";
  g_uiCalLine2  = "from sled";
  g_uiCalFooter = "Tap to tare";
  ui_screenCalStep();

  bool sp = false, lp = false;
  while (!sp && !lp) {
    readButton(btnStart, sp, lp);
    delay(10);
  }

  g_uiCalLine1  = "Taring...";
  g_uiCalLine2  = "Hold still";
  g_uiCalFooter = "";
  ui_screenCalStep();
  setLED(255, 0, 0);
  g_tareRaw = nauReadRawAvg(HX_SAMPLES_TARE);
  ledOff();

  // ---- Step 2: Known weight ----
  char weightStr[10];
  dtostrf(CAL_WEIGHT_LB, 1, 3, weightStr);
  char placeLine[24];
  snprintf(placeLine, sizeof(placeLine), "Place %s lb", weightStr);

  g_uiCalStep   = 2;
  g_uiCalLine1  = placeLine;
  g_uiCalLine2  = "weight on sled";
  g_uiCalFooter = "Tap to sample";
  ui_screenCalStep();

  sp = false;
  lp = false;
  while (!sp && !lp) {
    readButton(btnStart, sp, lp);
    delay(10);
  }

  g_uiCalLine1  = "Sampling...";
  g_uiCalLine2  = "Hold still";
  g_uiCalFooter = "";
  ui_screenCalStep();

  long raw3 = nauReadRawAvg(HX_SAMPLES_TARE);
  long delta = raw3 - g_tareRaw;

  if (abs(delta) < 100) {
    ui_screenError("Calibration failed", "Signal too small", "Check wiring");
    pulseLED(255, 0, 0, 1, 250);
    delay(2200);

    g_uiCalStep   = 0;
    g_uiCalLine1  = "Returning...";
    g_uiCalLine2  = "";
    g_uiCalFooter = "";
    ui_screenCalStep();
    homeToLimitSafe();

    MotionRequest reqDisable;
    reqDisable.cmd = CMD_DISABLE;
    requestMotion(reqDisable, 1000);
    return;
  }

  g_calibration = (float)delta / CAL_WEIGHT_LB;
  saveCalibration();

  // Done summary
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 2);
  oled.print(F("CAL  Done"));
  oled.drawLine(0, 11, OLED_WIDTH, 11, SSD1306_WHITE);

  char buf[24];
  snprintf(buf, sizeof(buf), "Cnt/lb : %.2f", g_calibration);
  oled.setCursor(2, 16); oled.print(buf);
  snprintf(buf, sizeof(buf), "Tare   : %ld", g_tareRaw);
  oled.setCursor(2, 28); oled.print(buf);
  snprintf(buf, sizeof(buf), "Delta  : %ld", delta);
  oled.setCursor(2, 40); oled.print(buf);
  ui_drawFooter("Saved to flash");
  oled.display();
  pulseLED(0, 255, 0, 1, 200);
  delay(1800);

  // ---- Return to home ----
  g_uiCalStep   = 0;
  g_uiCalLine1  = "Returning...";
  g_uiCalLine2  = "";
  g_uiCalFooter = "";
  ui_screenCalStep();
  setLED(255, 150, 0);

  homeToLimitSafe();

  {
    MotionRequest reqDisable;
    reqDisable.cmd = CMD_DISABLE;
    requestMotion(reqDisable, 1000);
  }

  ledOff();
  }  // end normal calibration block

  return;

cal_abort:
  {
    Serial.println("CALIBRATION ABORTED");
    g_collectSamples = false;
    g_abortRequested = false;
    g_abortBtnDownAt = 0;
    ui_screenAborted("Cal cancelled");
    setLED(255, 0, 0);

    while (digitalRead(BTN_START) == LOW) delay(10);

    g_uiPhase = UI_HOMING;
    ui_screenHoming();
    homeToLimitForce();

    MotionRequest reqDis;
    reqDis.cmd = CMD_DISABLE;
    requestMotion(reqDis, 1000);

    ledOff();
    delay(1200);
  }
}

// ----------------------------- Motion ---------------------------------------
bool g_motionActive = false; // used to suppress OLED live updates during motion

// ==================== DUAL-CORE MOTION FUNCTIONS ============================

// Check if START button held for ABORT_HOLD_MS; returns true if abort triggered
// Safe to call from any core (digitalRead only, no I2C)
bool checkAbortButton() {
  if (digitalRead(BTN_START) == LOW) {
    if (g_abortBtnDownAt == 0) g_abortBtnDownAt = millis();
    else if (millis() - g_abortBtnDownAt >= ABORT_HOLD_MS) {
      g_abortRequested = true;
      g_abortBtnDownAt = 0;
      return true;
    }
  } else {
    g_abortBtnDownAt = 0;
  }
  return false;
}

// Core 1: Pure stepping function (NO load cell, NO I2C, NO Serial in critical loop).
// Progress reporting is done time-based on Core 0 (millis() vs phase duration) —
// no inter-core publishing here, just clean stepping with periodic abort checks.
void executePureMove(long steps, bool forward, int pulseUs) {
  setDir(forward);
  for (long i = 0; i < steps; i++) {
    if ((i & 0xFF) == 0 && checkAbortButton()) break;  // Abort check every ~256 steps
    doStepBlocking(pulseUs);
  }
}

// Core 1: Homing sequence (called from motion task)
// When abortable=true, checks for abort button during homing
bool executeHome(bool abortable = true) {
  const uint32_t HOMING_TIMEOUT_MS = 100000; // 100 second timeout

  stepperEnable(true);
  setDir(DIR_HOME_TOWARD_LIMIT);

  // First approach
  int stepCount = 0;
  uint32_t startTime = millis();
  while (!limitHit()) {
    if (millis() - startTime > HOMING_TIMEOUT_MS) {
      stepperEnable(false);
      return false;
    }
    if (abortable && (++stepCount & 0x1FF) == 0 && checkAbortButton()) {
      stepperEnable(false);
      return false;
    }
    doStepBlocking(HOME_STEP_US);
  }

  // Back off
  setDir(!DIR_HOME_TOWARD_LIMIT);
  for (int i = 0; i < BACKOFF_STEPS; i++) {
    doStepBlocking(HOME_STEP_US);
  }

  // Second approach
  setDir(DIR_HOME_TOWARD_LIMIT);
  startTime = millis();
  while (!limitHit()) {
    if (millis() - startTime > HOMING_TIMEOUT_MS) {
      stepperEnable(false);
      return false;
    }
    if (abortable && (++stepCount & 0x1FF) == 0 && checkAbortButton()) {
      stepperEnable(false);
      return false;
    }
    doStepBlocking(HOME_STEP_US);
  }

  // Final back off
  setDir(!DIR_HOME_TOWARD_LIMIT);
  for (int i = 0; i < BACKOFF_STEPS / 2; i++) {
    doStepBlocking(HOME_STEP_US);
  }

  stepperEnable(false);
  return true;
}

// Core 1: Motion task (runs exclusively on Core 1)
void motionTask(void* parameter) {
  // Disable watchdog on Core 1
  disableCore1WDT();

  Serial.println("Motion task started on Core 1");
  Serial.print("Motion task running on core: ");
  Serial.println(xPortGetCoreID());

  MotionRequest req;

  while (true) {
    // Wait for motion command (yields CPU while waiting)
    if (xQueueReceive(motionCommandQueue, &req, portMAX_DELAY) == pdTRUE) {
      g_motionActive = true;

      // Execute command with NO interruptions
      switch (req.cmd) {
        case CMD_HOME:
          g_currentPhase = PHASE_HOMING;
          executeHome(true);
          break;

        case CMD_HOME_FORCE:
          g_currentPhase = PHASE_HOMING;
          executeHome(false);
          break;

        case CMD_MOVE:
          // Simple move (lowering or returning)
          g_currentPhase = req.phase;
          executePureMove(req.steps, req.direction, req.pulseUs);
          break;

        case CMD_MEASURE_MOVE:
          // Critical measurement phase
          g_currentPhase = req.phase;
          g_collectSamples = true;  // Signal Core 0 to start sampling

          executePureMove(req.steps, req.direction, req.pulseUs);

          g_collectSamples = false;  // Stop sampling
          break;

        case CMD_ENABLE:
          stepperEnable(true);
          break;

        case CMD_DISABLE:
          stepperEnable(false);
          break;
      }

      g_motionActive = false;
      g_currentPhase = PHASE_NONE;

      // Signal completion
      xSemaphoreGive(motionCompleteSemaphore);
    }
  }
}

// Core 0: Force sampling task (runs in parallel on Core 0)
void forceSamplingTask(void* parameter) {
  Serial.println("Force sampling task started on Core 0");
  Serial.print("Force sampling task running on core: ");
  Serial.println(xPortGetCoreID());

  while (true) {
    // Wait for sampling signal
    if (g_collectSamples) {

      // Determine which buffer to use
      float* sampleBuffer = NULL;
      volatile long* sampleCount = NULL;
      long maxSamples = MAX_SAMPLES_PER_PASS;

      if (g_currentPhase == PHASE_MEASURING_FWD) {
        sampleBuffer = g_fwdSamples;
        sampleCount = &g_fwdSampleCount;
        *sampleCount = 0;  // Reset counter
      } else if (g_currentPhase == PHASE_MEASURING_REV) {
        sampleBuffer = g_revSamples;
        sampleCount = &g_revSampleCount;
        *sampleCount = 0;  // Reset counter
      }

      // Sample as fast as possible while motion is active
      if (sampleBuffer != NULL && sampleCount != NULL) {
        while (g_collectSamples && *sampleCount < maxSamples) {
          if (nau.available()) {
            long raw = nau.getReading();
            float lbs = rawToPounds(raw);
            sampleBuffer[*sampleCount] = lbs;
            g_lastForceSampleLb = lbs;        // Publish for UI live readout
            (*sampleCount)++;
          }
          vTaskDelay(1);  // Yield briefly (~1ms)
        }
      }
    } else {
      vTaskDelay(10);  // Idle, check every 10ms
    }
  }
}

// Core 0: Request motion from Core 1 (wrapper function).
// Polls the completion semaphore in 30ms slices so ui_tick() can keep the
// progress bar / homing animation live without a dedicated UI task.
bool requestMotion(MotionRequest req, uint32_t timeoutMs) {
  if (xQueueSend(motionCommandQueue, &req, pdMS_TO_TICKS(100)) != pdTRUE) {
    Serial.println("ERROR: Motion queue full");
    return false;
  }

  uint32_t deadline = millis() + timeoutMs;
  while ((int32_t)(deadline - millis()) > 0) {
    if (xSemaphoreTake(motionCompleteSemaphore, pdMS_TO_TICKS(30)) == pdTRUE) {
      return true;
    }
    ui_tick();
  }
  Serial.println("ERROR: Motion timeout");
  return false;
}

// Core 0: High-level wrapper functions
void homeToLimitSafe() {
  MotionRequest req;
  req.cmd = CMD_HOME;
  req.steps = 0;
  req.direction = false;
  req.pulseUs = HOME_STEP_US;
  req.phase = PHASE_HOMING;
  requestMotion(req);
}

// Home without abort checking (used after abort to ensure safe return)
void homeToLimitForce() {
  MotionRequest req;
  req.cmd = CMD_HOME_FORCE;
  req.steps = 0;
  req.direction = false;
  req.pulseUs = HOME_STEP_US;
  req.phase = PHASE_HOMING;
  requestMotion(req);
}

void moveStepsBlockingSafe(long steps, bool forward, int pulseUs) {
  MotionRequest req;
  req.cmd = CMD_MOVE;
  req.steps = steps;
  req.direction = forward;
  req.pulseUs = pulseUs;
  req.phase = PHASE_NONE;
  requestMotion(req);
}

// ============================================================================

void homeToLimit() {
  const uint32_t HOMING_TIMEOUT_MS = 100000; // 100 second timeout

  g_motionActive = true;
  stepperEnable(true);
  setDir(DIR_HOME_TOWARD_LIMIT);

  // First approach with timeout
  uint32_t startTime = millis();
  while (!limitHit()) {
    if (millis() - startTime > HOMING_TIMEOUT_MS) {
      Serial.println("ERROR: Homing timeout on first approach!");
      stepperEnable(false);
      g_motionActive = false;
      return;
    }
    doStepBlocking(HOME_STEP_US);
  }
  setLED(0, 255, 0); // Green when limit is hit
  delay(200);

  setDir(!DIR_HOME_TOWARD_LIMIT);
  for (int i=0; i<BACKOFF_STEPS; i++) doStepBlocking(HOME_STEP_US);
  setDir(DIR_HOME_TOWARD_LIMIT);

  // Second approach with timeout
  startTime = millis();
  while (!limitHit()) {
    if (millis() - startTime > HOMING_TIMEOUT_MS) {
      Serial.println("ERROR: Homing timeout on second approach!");
      stepperEnable(false);
      g_motionActive = false;
      return;
    }
    doStepBlocking(HOME_STEP_US);
  }
  setLED(0, 255, 0); // Green when limit is hit (second time)
  delay(200);
  setDir(!DIR_HOME_TOWARD_LIMIT);
  for (int i=0; i<BACKOFF_STEPS/2; i++) doStepBlocking(HOME_STEP_US);

  ledOff();
  stepperEnable(false);
  g_motionActive = false;
}

void moveStepsBlocking(long steps, bool forward, int pulseUs) {
  setDir(forward);
  for (long i=0; i<steps; i++) doStepBlocking(pulseUs);
}


RunResult runTest() {
  const long steps_lower   = lround(SEG_LOWER_IN   * STEPS_PER_INCH);
  const long steps_noise   = lround(SEG_NOISE_IN   * STEPS_PER_INCH);
  const long steps_measure = lround(SEG_MEASURE_IN * STEPS_PER_INCH);

  g_fwdSampleCount = 0;
  g_revSampleCount = 0;
  g_abortRequested = false;
  g_abortBtnDownAt = 0;

  // Initial homing — animated "Homing" screen via ui_tick during requestMotion.
  g_uiPhase = UI_HOMING;
  ui_screenHoming();
  homeToLimitSafe();
  if (g_abortRequested) goto abort_cleanup;

  {
  // Switch to single progress-bar screen for the rest of the test.
  // Phase % bands sum to 100: lower 0-20, fwd 20-49, pause 49-51, rev 51-80, return 80-100.
  // Each phase's expected duration is (steps * 2 * pulseUs) / 1000 ms.
  const uint32_t dur_lower   = (uint32_t)(steps_lower + steps_noise) * 2u * STEP_PULSE_US / 1000u;
  const uint32_t dur_measure = (uint32_t)steps_measure              * 2u * STEP_PULSE_US / 1000u;
  const uint32_t dur_return  = (uint32_t)(steps_noise + steps_lower) * 2u * STEP_PULSE_US / 1000u;

  g_uiPhase = UI_RUNNING;
  g_uiProgressPct = 0;
  ui_screenRunning();

  MotionRequest req;
  req.cmd = CMD_ENABLE;
  requestMotion(req, 1000);

  // Lowering: 0% -> 20%
  setLED(255, 150, 0);
  g_uiLabel = "Lowering";
  g_uiShowForce = false;
  g_uiPhaseStartPct = 0;
  g_uiPhaseEndPct   = 20;
  g_phaseDurationMs = dur_lower;
  g_phaseStartMs    = millis();

  req.cmd = CMD_MOVE;
  req.steps = steps_lower + steps_noise;
  req.direction = DIR_FORWARD;
  req.pulseUs = STEP_PULSE_US;
  req.phase = PHASE_LOWERING;
  requestMotion(req);
  if (g_abortRequested) goto abort_cleanup;

  // Forward measurement: 20% -> 49%
  setLED(0, 255, 255);  // Cyan
  g_uiLabel = "Forward";
  g_uiShowForce = true;
  g_uiPhaseStartPct = 20;
  g_uiPhaseEndPct   = 49;
  g_phaseDurationMs = dur_measure;
  g_phaseStartMs    = millis();

  req.cmd = CMD_MEASURE_MOVE;
  req.steps = steps_measure;
  req.direction = DIR_FORWARD;
  req.pulseUs = STEP_PULSE_US;
  req.phase = PHASE_MEASURING_FWD;
  requestMotion(req);
  if (g_abortRequested) goto abort_cleanup;

  // Direction-reversal pause: 49% -> 51% (timed, 600ms)
  g_uiLabel = "Reversing";
  g_uiShowForce = false;
  g_uiPhaseStartPct = 49;
  g_uiPhaseEndPct   = 51;
  g_phaseDurationMs = 600;
  g_phaseStartMs    = millis();
  while ((int32_t)(millis() - g_phaseStartMs) < 600) {
    ui_tick();
    delay(20);
  }

  // Reverse measurement: 51% -> 80%
  setLED(255, 0, 255);  // Magenta
  g_uiLabel = "Reverse";
  g_uiShowForce = true;
  g_uiPhaseStartPct = 51;
  g_uiPhaseEndPct   = 80;
  g_phaseDurationMs = dur_measure;
  g_phaseStartMs    = millis();

  req.cmd = CMD_MEASURE_MOVE;
  req.steps = steps_measure;
  req.direction = !DIR_FORWARD;
  req.pulseUs = STEP_PULSE_US;
  req.phase = PHASE_MEASURING_REV;
  requestMotion(req);
  if (g_abortRequested) goto abort_cleanup;

  // Return: 80% -> 100%
  setLED(255, 150, 0);
  g_uiLabel = "Returning";
  g_uiShowForce = false;
  g_uiPhaseStartPct = 80;
  g_uiPhaseEndPct   = 100;
  g_phaseDurationMs = dur_return;
  g_phaseStartMs    = millis();

  req.cmd = CMD_MOVE;
  req.steps = steps_noise + steps_lower;
  req.direction = !DIR_FORWARD;
  req.pulseUs = STEP_PULSE_US;
  req.phase = PHASE_RETURNING;
  requestMotion(req);

  // Final precision homing — bar already at 100%, leave RUNNING screen up.
  homeToLimitSafe();

  req.cmd = CMD_DISABLE;
  requestMotion(req, 1000);
  }

  goto test_complete;

abort_cleanup:
  {
    Serial.println("TEST ABORTED - homing...");
    g_collectSamples = false;
    g_abortRequested = false;
    g_abortBtnDownAt = 0;
    g_uiPhase = UI_ABORTED;
    ui_screenAborted("Cancelled");
    setLED(255, 0, 0);

    while (digitalRead(BTN_START) == LOW) delay(10);

    g_uiPhase = UI_HOMING;
    ui_screenHoming();
    homeToLimitForce();

    MotionRequest reqDis;
    reqDis.cmd = CMD_DISABLE;
    requestMotion(reqDis, 1000);

    ledOff();
    delay(800);

    RunResult abortResult;
    abortResult.avgFrictionLb = 0;
    abortResult.cof = 0;
    abortResult.avgBias = 0;
    return abortResult;
  }

test_complete:
  Serial.println("\n===== TEST COMPLETE =====");
  Serial.print("Forward pass samples: ");
  Serial.println(g_fwdSampleCount);
  Serial.print("Reverse pass samples: ");
  Serial.println(g_revSampleCount);
  Serial.print("Total samples: ");
  Serial.println(g_fwdSampleCount + g_revSampleCount);
  Serial.println("========================\n");

  float trimFraction = SEG_TRIM_IN / SEG_MEASURE_IN;
  CofResult cr = calculateCOF(g_fwdSamples, g_fwdSampleCount,
                               g_revSamples, g_revSampleCount,
                               NORMAL_FORCE_LB, trimFraction,
                               avgPercentileBand);

  Serial.print("Paired samples used: ");
  Serial.println(cr.pairedCount);
  Serial.print("Avg friction force:  ");
  Serial.print(cr.avgForceLb, 4);
  Serial.println(" lb");
  Serial.print("Avg positional bias: ");
  Serial.print(cr.avgBias, 4);
  Serial.println(" lb");
  Serial.print("Final COF:           ");
  Serial.println(cr.cof, 4);
  Serial.println("========================\n");

  // Quick green flash — operator's eyes are on the screen, not the LED.
  pulseLED(0, 255, 0, 1, 200);

  RunResult rr;
  rr.avgFrictionLb = cr.avgForceLb;
  rr.cof = cr.cof;
  rr.avgBias = cr.avgBias;
  return rr;
}

// ----------------------------- CSV Data Dump --------------------------------
void dumpTestDataCSV() {
  // Raw samples (both passes, untrimmed)
  Serial.println("---CSV_START---");
  Serial.println("pass,index,force_lb");
  for (long i = 0; i < g_fwdSampleCount; i++) {
    Serial.print("FWD,");
    Serial.print(i);
    Serial.print(",");
    Serial.println(g_fwdSamples[i], 4);
  }
  for (long i = 0; i < g_revSampleCount; i++) {
    Serial.print("REV,");
    Serial.print(i);
    Serial.print(",");
    Serial.println(g_revSamples[i], 4);
  }
  Serial.println("---CSV_END---");

  // Paired data (position-matched, trimmed)
  float trimFraction = SEG_TRIM_IN / SEG_MEASURE_IN;
  dumpPairedDataCSV(g_fwdSamples, g_fwdSampleCount,
                    g_revSamples, g_revSampleCount,
                    trimFraction);
}

// ----------------------------- Buttons --------------------------------------
bool readButton(Btn& b, bool& shortPress, bool& longPress) {
  shortPress = false;
  longPress  = false;
  bool cur = digitalRead(b.pin); // INPUT_PULLUP: LOW when pressed
  uint32_t now = millis();

  if (cur != b.last && (now - b.lastChange) > DEBOUNCE_MS) {
    b.last = cur;
    b.lastChange = now;
    if (cur == LOW) { b.downAt = now; b.longSent = false; }
    else {
      uint32_t held = now - b.downAt;
      if (!b.longSent && held >= DEBOUNCE_MS && held < LONG_PRESS_MS) shortPress = true;
    }
  }
  if (b.last == LOW && !b.longSent && (now - b.downAt) >= LONG_PRESS_MS) { longPress = true; b.longSent = true; }
  return (shortPress || longPress);
}

// ----------------------------- RFID Functions -------------------------------

// Write measurement to RFID tag - single 5-minute poll, no retry screens.
// Returns: true on success, false on skip (button hold), tag-full, or timeout.
// Caller is expected to have already drawn ui_screenDone() before invoking.
bool writeToRFID(float cofValue) {
  Serial.println("Starting RFID write process...");
  Serial.print("COF value: ");
  Serial.println(cofValue, 3);

  PaddleDNA::Measurement measurement(
    PaddleDNA::MeasurementType::CoF,
    MACHINE_UUID,
    FIXED_TIMESTAMP,
    cofValue
  );

  const unsigned long TAG_WAIT_TIMEOUT = 300000;  // 5 minutes
  const unsigned long SKIP_HOLD_MS     = 2000;
  unsigned long startTime    = millis();
  unsigned long lastPollTime = 0;
  bool ledState = false;

  while (millis() - startTime < TAG_WAIT_TIMEOUT) {
    // Skip if button held >= 2s
    if (digitalRead(BTN_START) == LOW) {
      unsigned long holdStart = millis();
      while (digitalRead(BTN_START) == LOW && (millis() - holdStart < SKIP_HOLD_MS)) {
        delay(10);
      }
      if (millis() - holdStart >= SKIP_HOLD_MS) {
        ledOff();
        ui_screenAborted("Skipped");
        setLED(255, 150, 0);
        delay(800);
        ledOff();
        return false;
      }
    }

    // Poll every 250ms
    if (millis() - lastPollTime < 250) {
      delay(10);
      continue;
    }
    lastPollTime = millis();

    // Blink blue while polling
    ledState = !ledState;
    if (ledState) setLED(0, 0, 255); else ledOff();

    String msg;
    PaddleDNA::AccumulateResult result = accumulator->accumulate(measurement, &msg);

    Serial.print("Accumulate result: ");
    Serial.print((int)result);
    Serial.print(" - ");
    Serial.println(msg);

    switch (result) {
      case PaddleDNA::AccumulateResult::Success:
        ledOff();
        g_uiCofValue = cofValue;
        g_uiPhase = UI_SAVED;
        ui_screenSaved();
        pulseLED(0, 255, 0, 1, 200);
        delay(700);
        return true;

      case PaddleDNA::AccumulateResult::TagFull:
        ledOff();
        ui_screenError("Tag full", "Use a new tag", "");
        pulseLED(255, 0, 0, 1, 250);
        delay(2200);
        return false;

      case PaddleDNA::AccumulateResult::NoTag:
      case PaddleDNA::AccumulateResult::ReadError:
      case PaddleDNA::AccumulateResult::WriteError:
      case PaddleDNA::AccumulateResult::InvalidPayload:
      case PaddleDNA::AccumulateResult::CryptoError:
        // Keep polling silently — stay on the Tap-NFC screen
        break;
    }
  }

  // 5-minute timeout reached
  ledOff();
  ui_screenError("NFC timeout", "Tag not written", "");
  pulseLED(255, 0, 0, 1, 250);
  delay(2200);
  return false;
}

// ----------------------------- Setup / Loop ---------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n\n=== ESP32 Paddle COF Tester Starting ===");

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_LIMIT, INPUT_PULLUP); // active-LOW
  pinMode(BTN_START, INPUT_PULLUP); // active-LOW
  Serial.println("GPIO pins configured");

  stepperEnable(false);
  Serial.println("Stepper disabled");

  // Initialize RGB LED
  Serial.print("Initializing RGB LED on pin ");
  Serial.println(RGB_LED_PIN);
  rgbLed.begin();
  rgbLed.setBrightness(50);
  rgbLed.show();
  Serial.println("RGB LED initialized, testing colors...");

  // Test LED with primary colors
  setLED(255, 0, 0); // Red
  Serial.println("LED: RED");
  delay(300);
  setLED(0, 255, 0); // Green
  Serial.println("LED: GREEN");
  delay(300);
  setLED(0, 0, 255); // Blue
  Serial.println("LED: BLUE");
  delay(300);
  ledOff();
  Serial.println("LED: OFF");

  Serial.println("Initializing I2C and OLED...");
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);  // Critical delay for ESP32-S3 I2C stability
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  // IMPORTANT: setClock must come AFTER oled.begin() — the Adafruit BusIO
  // library forces 100 kHz inside begin(), which makes a full-frame display()
  // take ~80 ms and starves the progress-bar redraw loop. 400 kHz drops it
  // to ~20 ms; PN532 (also on Wire) is fine up to 400 kHz.
  Wire.setClock(400000);
  oled.clearDisplay();
  oled.display();
  Serial.println("OLED ready");

  // Initialize PaddleDNA NFC
  Serial.println("Initializing PaddleDNA NFC...");
  if (!nfc.begin(Wire)) {
    Serial.println(F("NFC initialization failed!"));
    ui_screenError("NFC init failed", "Check connections", "");
    pulseLED(255, 0, 0, 3, 250);
    delay(2500);
    // Continue anyway - allow force measurements without RFID
  } else {
    Serial.println("NFC initialized successfully");
  }

  // Initialize PaddleDNA Crypto
  Serial.println("Initializing PaddleDNA Crypto...");
  if (!crypto.begin(MACHINE_UUID, PRIVATE_KEY)) {
    Serial.println(F("Crypto initialization failed!"));
    ui_screenError("Crypto init failed", "", "");
    pulseLED(255, 0, 0, 3, 250);
    delay(2500);
    // Continue anyway
  } else {
    Serial.println("Crypto initialized successfully");
  }

  // Create MeasurementAccumulator (paddle UUID auto-detected from tag)
  Serial.println("Creating MeasurementAccumulator...");
  accumulator = new PaddleDNA::MeasurementAccumulator(nfc, crypto, 9);
  Serial.println("MeasurementAccumulator created successfully");

  Serial.println("Initializing NAU7802 load cell...");
  Wire1.begin(NAU_SDA, NAU_SCL);
  if (!nau.begin(Wire1)) {
    Serial.println("ERROR: NAU7802 not detected!");
    ui_screenError("Load cell missing", "NAU7802 not found", "Check wiring");
    pulseLED(255, 0, 0, 3, 250);
    delay(2500);
  }
  nau.setGain(NAU7802_GAIN_128);
  nau.setSampleRate(NAU7802_SPS_320);
  nau.calibrateAFE();
  loadCalibration();
  Serial.print("Calibration loaded: ");
  Serial.print(g_calibration);
  Serial.print(" counts/lb, Tare: ");
  Serial.println(g_tareRaw);

  // Auto-tare on boot. Doesn't affect COF (paired math cancels offset),
  // but keeps the live force overlay honest after thermal/mechanical drift.
  setLED(255, 0, 0);
  g_tareRaw = nauReadRawAvg(HX_SAMPLES_TARE);
  ledOff();
  Serial.print("Auto-tare on boot: ");
  Serial.println(g_tareRaw);

  // ========== DUAL-CORE TASK INITIALIZATION ==========
  Serial.println("\n=== Initializing Dual-Core Architecture ===");
  Serial.print("setup() running on core: ");
  Serial.println(xPortGetCoreID());

  // Create inter-core communication
  Serial.println("Creating motion command queue...");
  motionCommandQueue = xQueueCreate(5, sizeof(MotionRequest));
  if (motionCommandQueue == NULL) {
    Serial.println("ERROR: Failed to create motion queue!");
  }

  Serial.println("Creating motion complete semaphore...");
  motionCompleteSemaphore = xSemaphoreCreateBinary();
  if (motionCompleteSemaphore == NULL) {
    Serial.println("ERROR: Failed to create semaphore!");
  }

  Serial.println("Creating motion task on Core 1 (high priority)...");
  BaseType_t motionTaskCreated = xTaskCreatePinnedToCore(
    motionTask,           // Function
    "Motion",             // Name
    4096,                 // Stack size (bytes)
    NULL,                 // Parameter
    3,                    // Priority (high - above default 1)
    NULL,                 // Task handle
    1                     // Core 1
  );

  if (motionTaskCreated != pdPASS) {
    Serial.println("ERROR: Failed to create motion task!");
  } else {
    Serial.println("Motion task created successfully");
  }

  Serial.println("Creating force sampling task on Core 0 (medium priority)...");
  BaseType_t samplingTaskCreated = xTaskCreatePinnedToCore(
    forceSamplingTask,
    "ForceSample",
    4096,
    NULL,
    2,                    // Priority (medium)
    &forceSamplingTaskHandle,
    0                     // Core 0
  );

  if (samplingTaskCreated != pdPASS) {
    Serial.println("ERROR: Failed to create sampling task!");
  } else {
    Serial.println("Force sampling task created successfully");
  }

  delay(200);  // Let tasks initialize
  Serial.println("=== Dual-Core Architecture Initialized ===\n");
  // ====================================================

  g_uiPhase = UI_BOOT;
  ui_screenSplash();
  Serial.println("Starting rainbow cycle...");
  rainbowCycle(2000); // 2 second rainbow cycle on power-up
  Serial.println("Rainbow cycle complete");
  delay(500);

  // Initialization homing — splash stays on screen (ui_tick is a no-op for UI_BOOT)
  Serial.println("Checking limit switch...");
  if (!limitHit()) {
    Serial.println("Not at limit, starting homing sequence...");
    homeToLimitSafe();
    Serial.println("Homing complete");
  } else {
    Serial.println("Already at home position");
  }

  stepperEnable(false);

  // Boot calibration: if START button is held during power-up, enter calibration
  if (digitalRead(BTN_START) == LOW) {
    Serial.println("START button held at boot - entering calibration mode");
    g_uiCalStep   = 0;
    g_uiCalLine1  = "Release button";
    g_uiCalLine2  = "to begin calibration";
    g_uiCalFooter = "";
    g_uiPhase = UI_CAL;
    ui_screenCalStep();
    while (digitalRead(BTN_START) == LOW) delay(10);
    delay(200);
    doCalibration3lb();
  }

  // Spawn the user-facing task on Core 0. Arduino's loopTask defaults to
  // Core 1 — same core as motionTask (priority 3) — and a higher-priority
  // running task starves loopTask, so requestMotion()'s polling loop never
  // runs and ui_tick never fires. Core 0 has no such contention; motionTask
  // owns Core 1 by itself.
  Serial.println("Creating main UI/test task on Core 0...");
  BaseType_t mainTaskCreated = xTaskCreatePinnedToCore(
    mainTask,
    "Main",
    8192,                 // generous stack — runTest, writeToRFID, NFC are deep
    NULL,
    1,                    // same as Arduino's default loopTask priority
    NULL,
    0                     // Core 0
  );
  if (mainTaskCreated != pdPASS) {
    Serial.println("ERROR: Failed to create main task!");
  } else {
    Serial.println("Main task created on Core 0");
  }

  Serial.println("=== Setup complete, entering main loop ===\n");
}

// Default Arduino loop runs on Core 1, where motionTask lives. We can't do
// any real work here without contending with motionTask, so this stub just
// yields. All real work happens in mainTask on Core 0.
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// Runs on Core 0. Owns the idle screen, button polling, runTest, and the
// NFC write flow. requestMotion() blocks here while motionTask runs on
// Core 1 — but xSemaphoreTake() actually yields properly and ui_tick()
// gets to fire from the polling loop.
void mainTask(void* parameter) {
  Serial.printf("Main task running on core %d\n", xPortGetCoreID());

  while (true) {
    // Idle screen — last result is the hero, 1 tap kicks off a test.
    Serial.println("Entering idle state");
    ledOff();
    g_uiPhase = UI_IDLE;
    ui_screenIdle();

    g_motionActive = false;
    while (true) {
      bool sp=false, lp=false;
      readButton(btnStart, sp, lp);
      if (sp) {
        Serial.println("START button pressed - Running test...");
        RunResult r = runTest();

        // Aborted test returns zeros — go straight back to idle.
        if (r.cof == 0 && r.avgFrictionLb == 0) {
          Serial.println("Test was aborted, returning to idle");
          break;
        }

        g_lastAvgLb = r.avgFrictionLb;
        g_lastCOF   = r.cof;
        g_hasResult = true;

        Serial.print("Test complete! COF: ");
        Serial.println(r.cof, 3);

        dumpTestDataCSV();

        // Auto-flow: result + NFC prompt on the same screen, no "press to continue" gate.
        g_uiPhase = UI_DONE;
        g_uiCofValue = r.cof;
        ui_screenDone();

        Serial.println("Entering RFID write mode...");
        writeToRFID(r.cof);  // draws its own success/skip/timeout screen + delay

        break;  // back to idle (now showing the new "Last test")
      }
      delay(10);
    }
  }
}
