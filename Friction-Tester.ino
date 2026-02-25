/*
  ESP32 Paddle COF Tester (continuous bidirectional test, non-blocking HX711)
  - DRV8825 + NEMA17
  - HX711 load cell amp
  - SSD1306 OLED (I2C)
  - Buttons: START, ZERO/CAL -> INPUT_PULLUP, wire to GND (see BTN_START, BTN_ZERO defines)
  - Limit switch -> INPUT_PULLUP, active-LOW, wire to GND (see PIN_LIMIT define)
  - Pin assignments vary by ESP32 variant - see USER CONFIG section below
  - On normal boot: homes to limit switch automatically if not already pressed
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HX711.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <PaddleDNA.h>
#include <math.h>

// ----------------------------- USER CONFIG ----------------------------------
// NOTE: Pin assignments below match PCB schematic (ESP32-S3-ZERO)
// ESP32-S3 Pin Restrictions: Avoid GPIO 26-32 (reserved/problematic)
// See schematic for hardware pullups and connections

#define I2C_SDA 12     // Shared I2C bus (OLED + RFID)
#define I2C_SCL 11     // Shared I2C bus (OLED + RFID)

#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_ADDR    0x3C

#define HX_DOUT  5    // HX711 load cell data
#define HX_SCK   6    // HX711 load cell clock

#define PIN_STEP  7   // DRV8825 step pin
#define PIN_DIR   2  // DRV8825 direction pin
#define PIN_EN    3  // DRV8825 enable (Active LOW)

#define PIN_LIMIT  4  // Limit switch input (active-LOW with 10K pullup)
#define LIMIT_ACTIVE_LOW 1

#define BTN_START  10 // Start button (active-LOW with 10K pullup)
#define BTN_ZERO   9  // Zero/Calibration button (active-LOW with 10K pullup)

#define RGB_LED_PIN 48   // Onboard RGB LED (ESP32-S3)

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

const float CAL_WEIGHT_LB    = 3.883;   // calibration weight
const float NORMAL_FORCE_LB  = 3.59;  // test normal force
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
HX711 scale;
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
Btn btnZero {BTN_ZERO,  true, 0, 0, false};

struct RunResult { float avgFrictionLb; float cof; };
struct MeasureResult {
  float* samples;  // array of all samples
  long count;      // number of samples
  double avgLb;    // computed percentile average
};

// Prototypes
void   stepperEnable(bool on);
void   setDir(bool forward);
void   doStepBlocking(int pulseUs);
bool   limitHit();
void   oledHeader(const char* line1);
void   oledKV(const char* k, const String& v);
void   showSplash();
void   saveCalibration();
void   loadCalibration();
long   hxReadRawAvg(int n);
float  rawToPounds(long raw);
void   tareNow();
void   doCalibration3lb();
void   homeToLimit();
void   moveStepsBlocking(long steps, bool forward, int pulseUs);
MeasureResult measureDuringMove(long steps, bool forward, int pulseUs, long trimSteps);
double calculatePercentileAverage(float* samples, long count);
int    compareFloats(const void* a, const void* b);
RunResult runTest();
bool   readButton(Btn& b, bool& shortPress, bool& longPress);
void   updateLiveForceLine(bool forceClear=false);
void   setLED(uint8_t r, uint8_t g, uint8_t b);
void   ledOff();
void   rainbowCycle(int durationMs);
void   pulseLED(uint8_t r, uint8_t g, uint8_t b, int times, int pulseMs);
uint32_t colorWheel(byte pos);
void   displayTestResults(float cof, int machineID);
void   displayRFIDSuccess();
void   displayRFIDRetry(int attemptsLeft);
void   displayRFIDFinalFailure();
bool   writeToRFID(float cofValue);
void   dumpTestDataCSV();

// Dual-core function prototypes
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

void oledHeader(const char* line1) {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println(F("ESP32 Paddle COF"));
  oled.drawLine(0, 10, OLED_WIDTH, 10, SSD1306_WHITE);
  oled.setCursor(0, 14);
  oled.println(line1);
}

void oledKV(const char* k, const String& v) {
  oled.print(k);
  oled.print(F(": "));
  oled.println(v);
}

void showSplash() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println(F("ESP32 Paddle COF Tester"));
  oled.println(F("DRV8825 + HX711 + OLED"));
  oled.display();
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

long hxReadRawAvg(int n) {
  long sum = 0;
  for (int i=0; i<n; i++) {
    while (!scale.is_ready()) delay(1);
    sum += scale.read();
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

void tareNow() {
  setLED(255, 0, 0); // Red during tare
  g_tareRaw = hxReadRawAvg(HX_SAMPLES_TARE);
  saveCalibration();
  ledOff();
}

void doCalibration3lb() {
  // ---- Move carriage to furthest position for calibration ----
  oledHeader("CAL: Positioning...");
  oled.println(F("Moving to cal position"));
  oled.display();
  setLED(255, 150, 0); // Yellow during positioning

  // First home to ensure consistent starting point
  homeToLimitSafe();

  // Move to furthest position (lowering + measurement distance)
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

  ledOff();

  // ---- Step 1: Tare (zero-load) ----
  oledHeader("CAL: Step 1/2 (Tare)");
  oled.println(F("Remove all load"));
  oled.println(F("Press START to tare"));
  oled.display();

  // Wait for START button press (debounced)
  bool sp = false, lp = false;
  while (!sp && !lp) {
    readButton(btnStart, sp, lp);
    delay(10);
  }

  oledHeader("CAL: Taring...");
  oled.display();
  setLED(255, 0, 0); // Red during tare
  g_tareRaw = hxReadRawAvg(HX_SAMPLES_TARE);
  ledOff();

  // ---- Step 2: Known weight ----
  String headerStr = "CAL: Step 2/2 (" + String(CAL_WEIGHT_LB, 3) + " lb)";
  oledHeader(headerStr.c_str());
  oled.print(F("Place "));
  oled.print(CAL_WEIGHT_LB, 3);
  oled.println(F(" lb weight"));
  oled.println(F("Press START to sample"));
  oled.display();

  // Wait for START button press (debounced)
  sp = false;
  lp = false;
  while (!sp && !lp) {
    readButton(btnStart, sp, lp);
    delay(10);
  }

  long raw3 = hxReadRawAvg(HX_SAMPLES_TARE);
  long delta = raw3 - g_tareRaw; // counts due to calibration weight

  if (abs(delta) < 100) {
    oledHeader("CAL FAILED");
    oled.println(F("Signal too small"));
    oled.display();
    delay(2000);

    // Return carriage to home even on failure
    oledHeader("Returning...");
    oled.display();
    homeToLimitSafe();

    MotionRequest reqDisable;
    reqDisable.cmd = CMD_DISABLE;
    requestMotion(reqDisable, 1000);
    return;
  }

  // counts per lb
  g_calibration = (float)delta / CAL_WEIGHT_LB;
  saveCalibration();

  oledHeader("CAL DONE");
  String countsLabel = "Counts@" + String(CAL_WEIGHT_LB, 3) + "lb";
  oledKV(countsLabel.c_str(), String(delta));
  oledKV("Cal (cnt/lb)", String(g_calibration, 2));
  oledKV("TareRaw", String(g_tareRaw));
  oled.display();
  delay(1500);

  // ---- Return carriage to home position ----
  oledHeader("CAL: Returning...");
  oled.println(F("Moving to home"));
  oled.display();
  setLED(255, 150, 0); // Yellow during return

  homeToLimitSafe();

  // Disable stepper
  MotionRequest reqDisable;
  reqDisable.cmd = CMD_DISABLE;
  requestMotion(reqDisable, 1000);

  ledOff();
}

// ----------------------------- Motion ---------------------------------------
bool g_motionActive = false; // used to suppress OLED live updates during motion

// ==================== DUAL-CORE MOTION FUNCTIONS ============================

// Core 1: Pure stepping function (NO HX711, NO I2C, NO Serial in critical loop)
void executePureMove(long steps, bool forward, int pulseUs) {
  setDir(forward);

  for (long i = 0; i < steps; i++) {
    doStepBlocking(pulseUs);
    // That's it! Pure stepping only
  }
}

// Core 1: Homing sequence (called from motion task)
bool executeHome() {
  const uint32_t HOMING_TIMEOUT_MS = 100000; // 100 second timeout

  stepperEnable(true);
  setDir(DIR_HOME_TOWARD_LIMIT);

  // First approach
  uint32_t startTime = millis();
  while (!limitHit()) {
    if (millis() - startTime > HOMING_TIMEOUT_MS) {
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
          executeHome();
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
          if (scale.is_ready()) {
            long raw = scale.read();
            sampleBuffer[*sampleCount] = rawToPounds(raw);
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

// Core 0: Request motion from Core 1 (wrapper function)
bool requestMotion(MotionRequest req, uint32_t timeoutMs) {
  // Send command to Core 1
  if (xQueueSend(motionCommandQueue, &req, pdMS_TO_TICKS(100)) != pdTRUE) {
    Serial.println("ERROR: Motion queue full");
    return false;
  }

  // Wait for completion
  if (xSemaphoreTake(motionCompleteSemaphore, pdMS_TO_TICKS(timeoutMs)) != pdTRUE) {
    Serial.println("ERROR: Motion timeout");
    return false;
  }

  return true;
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

// Comparison function for qsort
int compareFloats(const void* a, const void* b) {
  float fa = *(const float*)a;
  float fb = *(const float*)b;
  if (fa < fb) return -1;
  if (fa > fb) return 1;
  return 0;
}

// Calculate average of samples from 85th to 95th percentile
// (disregard top 5%, average next 10%)
double calculatePercentileAverage(float* samples, long count) {
  if (count < 10) {
    // Not enough samples for percentile calculation, just average all
    double sum = 0.0;
    for (long i = 0; i < count; i++) {
      sum += fabs(samples[i]);
    }
    return sum / (double)count;
  }

  // Sort samples by absolute value (we need a temporary array for this)
  float* absSamples = (float*)malloc(count * sizeof(float));
  if (!absSamples) {
    Serial.println("ERROR: Failed to allocate memory for sorting");
    return 0.0;
  }

  for (long i = 0; i < count; i++) {
    absSamples[i] = fabs(samples[i]);
  }

  qsort(absSamples, count, sizeof(float), compareFloats);

  // Calculate indices for 85th and 95th percentiles
  // Top 5% means we discard from 95th percentile to 100th
  // We want 85th to 95th percentile
  long idx85 = (long)(count * 0.85);
  long idx95 = (long)(count * 0.95);

  if (idx85 >= idx95) idx85 = idx95 - 1;
  if (idx85 < 0) idx85 = 0;

  // Average the samples in this range
  double sum = 0.0;
  long rangeCount = idx95 - idx85;
  for (long i = idx85; i < idx95; i++) {
    sum += absSamples[i];
  }

  free(absSamples);

  return (rangeCount > 0) ? (sum / (double)rangeCount) : 0.0;
}

// Non-blocking measurement during motion
MeasureResult measureDuringMove(long steps, bool forward, int pulseUs, long trimSteps) {
  setDir(forward);

  // Allocate array for samples (estimated max ~2000 samples per pass)
  const long MAX_SAMPLES = 2000;
  float* samples = (float*)malloc(MAX_SAMPLES * sizeof(float));
  long nF = 0;

  if (!samples) {
    Serial.println("ERROR: Failed to allocate sample array");
    MeasureResult mr;
    mr.samples = NULL;
    mr.count = 0;
    mr.avgLb = 0.0;
    return mr;
  }

  // Keep tight timing - no LED updates in this critical loop
  for (long i = 0; i < steps; i++) {
    doStepBlocking(pulseUs);        // keep motion timing tight

    // Only collect samples in the measurement window (skip start and end trim regions)
    bool inMeasurementWindow = (i >= trimSteps) && (i < (steps - trimSteps));

    // Non-blocking HX711 read (only if data is ready and in measurement window)
    if (inMeasurementWindow && scale.is_ready() && nF < MAX_SAMPLES) {
      long raw = scale.read();
      float lbs = rawToPounds(raw);
      samples[nF] = lbs;
      nF++;
    }
  }

  MeasureResult mr;
  mr.samples = samples;
  mr.count = nF;
  mr.avgLb = calculatePercentileAverage(samples, nF);
  return mr;
}

RunResult runTest() {
  const long steps_lower   = lround(SEG_LOWER_IN   * STEPS_PER_INCH);
  const long steps_noise   = lround(SEG_NOISE_IN   * STEPS_PER_INCH);
  const long steps_measure = lround(SEG_MEASURE_IN * STEPS_PER_INCH);
  const long steps_trim    = lround(SEG_TRIM_IN    * STEPS_PER_INCH);

  // Reset sample counters
  g_fwdSampleCount = 0;
  g_revSampleCount = 0;

  // Homing
  oledHeader("Homing...");
  oled.display();
  homeToLimitSafe();

  // Lowering (no sampling)
  oledHeader("Running (forward)...");
  oled.println(F("Lowering..."));
  oled.display();
  setLED(255, 150, 0);  // Yellow

  MotionRequest req;

  // Enable stepper
  req.cmd = CMD_ENABLE;
  requestMotion(req, 1000);

  // Lower phase
  req.cmd = CMD_MOVE;
  req.steps = steps_lower + steps_noise;  // Combined lowering + noise segments
  req.direction = DIR_FORWARD;
  req.pulseUs = STEP_PULSE_US;
  req.phase = PHASE_LOWERING;
  requestMotion(req);

  // Forward measurement pass
  oledHeader("Measuring (FWD)...");
  oled.display();
  setLED(0, 255, 255);  // Cyan

  req.cmd = CMD_MEASURE_MOVE;
  req.steps = steps_measure;
  req.direction = DIR_FORWARD;
  req.pulseUs = STEP_PULSE_US;
  req.phase = PHASE_MEASURING_FWD;
  requestMotion(req);

  // Pause between passes
  delay(600);

  // Reverse measurement pass
  oledHeader("Measuring (REV)...");
  oled.display();
  setLED(255, 0, 255);  // Magenta

  req.cmd = CMD_MEASURE_MOVE;
  req.steps = steps_measure;
  req.direction = !DIR_FORWARD;
  req.pulseUs = STEP_PULSE_US;
  req.phase = PHASE_MEASURING_REV;
  requestMotion(req);

  // Return
  oledHeader("Returning...");
  oled.display();
  setLED(255, 150, 0);  // Yellow

  req.cmd = CMD_MOVE;
  req.steps = steps_noise + steps_lower;  // Combined noise + lower segments
  req.direction = !DIR_FORWARD;
  req.pulseUs = STEP_PULSE_US;
  req.phase = PHASE_RETURNING;
  requestMotion(req);

  homeToLimitSafe();

  // Disable stepper
  req.cmd = CMD_DISABLE;
  requestMotion(req, 1000);

  // ========== SERIAL REPORTING ==========
  Serial.println("\n===== TEST COMPLETE =====");
  Serial.print("Forward pass samples: ");
  Serial.println(g_fwdSampleCount);
  Serial.print("Reverse pass samples: ");
  Serial.println(g_revSampleCount);
  Serial.print("Total samples: ");
  Serial.println(g_fwdSampleCount + g_revSampleCount);
  Serial.println("========================\n");

  // Calculate COF using collected samples (trim samples from start and end)
  // Trim calculation: skip first and last trimSteps worth of samples
  long trimSamplesPerSide = (g_fwdSampleCount * steps_trim) / steps_measure;
  if (trimSamplesPerSide < 0) trimSamplesPerSide = 0;

  // For forward samples: skip first and last trimSamplesPerSide samples
  long fwdStartIdx = trimSamplesPerSide;
  long fwdEndIdx = g_fwdSampleCount - trimSamplesPerSide;
  if (fwdEndIdx <= fwdStartIdx) {
    fwdStartIdx = 0;
    fwdEndIdx = g_fwdSampleCount;
  }
  long fwdTrimmedCount = fwdEndIdx - fwdStartIdx;

  // Create trimmed forward array
  float* fwdTrimmed = g_fwdSamples + fwdStartIdx;

  // For reverse samples: skip first and last trimSamplesPerSide samples
  long revStartIdx = trimSamplesPerSide;
  long revEndIdx = g_revSampleCount - trimSamplesPerSide;
  if (revEndIdx <= revStartIdx) {
    revStartIdx = 0;
    revEndIdx = g_revSampleCount;
  }
  long revTrimmedCount = revEndIdx - revStartIdx;

  // Create trimmed reverse array
  float* revTrimmed = g_revSamples + revStartIdx;

  double fwdAvg = calculatePercentileAverage(fwdTrimmed, fwdTrimmedCount);
  double revAvg = calculatePercentileAverage(revTrimmed, revTrimmedCount);

  Serial.print("Forward 85-95%ile avg (trimmed): ");
  Serial.print(fwdAvg, 4);
  Serial.println(" lb");
  Serial.print("Reverse 85-95%ile avg (trimmed): ");
  Serial.print(revAvg, 4);
  Serial.println(" lb");

  // Equal-weighted bidirectional average (standard COF practice)
  // Each direction gets 50% weight to cancel directional bias
  // (cable drag, surface grain, tare offset, load cell bias)
  double avgLbTwoPass;
  if (fwdTrimmedCount > 0 && revTrimmedCount > 0) {
    avgLbTwoPass = (fabs(fwdAvg) + fabs(revAvg)) / 2.0;
  } else if (fwdTrimmedCount > 0) {
    avgLbTwoPass = fabs(fwdAvg);
    Serial.println("WARNING: No reverse samples — using forward only");
  } else if (revTrimmedCount > 0) {
    avgLbTwoPass = fabs(revAvg);
    Serial.println("WARNING: No forward samples — using reverse only");
  } else {
    avgLbTwoPass = 0.0;
    Serial.println("ERROR: No samples collected in either direction");
  }

  float cof = (NORMAL_FORCE_LB > 0) ? (avgLbTwoPass / NORMAL_FORCE_LB) : 0.0f;

  Serial.print("Final COF: ");
  Serial.println(cof, 4);
  Serial.println("========================\n");

  // Test complete - pulse green 3 times
  pulseLED(0, 255, 0, 3, 300);

  RunResult rr;
  rr.avgFrictionLb = (float)avgLbTwoPass;
  rr.cof = cof;
  return rr;
}

// ----------------------------- CSV Data Dump --------------------------------
void dumpTestDataCSV() {
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
// Display COF results with RFID prompt
void displayTestResults(float cof, int machineID) {
  oled.clearDisplay();

  char cofStr[10];
  dtostrf(cof, 1, 3, cofStr);

  // Display results - split screen vertically
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  // Left side: COF value
  oled.setCursor(0, 8);
  oled.println("COF");
  oled.setCursor(0, 18);
  oled.println("----------");
  oled.setCursor(0, 28);
  oled.setTextSize(2);
  oled.println(cofStr);

  // Right side: Present NFC prompt
  oled.setTextSize(1);
  oled.setCursor(75, 20);
  oled.println("Present");
  oled.setCursor(80, 30);
  oled.println("NFC");

  oled.display();
}

// Display RFID write success
void displayRFIDSuccess() {
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(20, 24);
  oled.println("Success!");
  oled.display();
  pulseLED(0, 255, 0, 2, 300); // Green pulse
  delay(1500);
}

// Display RFID retry prompt
void displayRFIDRetry(int attemptsLeft) {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(30, 20);
  oled.println("Try again");
  oled.setCursor(15, 35);
  oled.print("(");
  oled.print(attemptsLeft);
  oled.print(" attempts left)");
  oled.display();
  setLED(255, 150, 0); // Orange/yellow for retry
  delay(1000);
  ledOff();
}

// Display RFID write final failure
void displayRFIDFinalFailure() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(15, 20);
  oled.println("Write failed");
  oled.setCursor(10, 35);
  oled.println("Continuing...");
  oled.display();
  pulseLED(255, 0, 0, 2, 300); // Red pulse for failure
  delay(3000);
}

// Write measurement to RFID tag with polling and retry
// Returns: true if successful, false if aborted or failed after 5 retries
bool writeToRFID(float cofValue) {
  Serial.println("Starting RFID write process...");
  Serial.print("COF value: ");
  Serial.println(cofValue, 3);

  // Create measurement (CoF type)
  PaddleDNA::Measurement measurement(
    PaddleDNA::MeasurementType::CoF,
    MACHINE_UUID,
    FIXED_TIMESTAMP,
    cofValue
  );

  const int MAX_RETRIES = 5;
  int attemptNumber = 0;
  const unsigned long TAG_WAIT_TIMEOUT = 30000;  // 30 seconds per attempt

  while (attemptNumber < MAX_RETRIES) {
    unsigned long attemptStartTime = millis();
    unsigned long lastPollTime = 0;

    Serial.print("Attempt ");
    Serial.print(attemptNumber + 1);
    Serial.print(" of ");
    Serial.println(MAX_RETRIES);

    // Poll for tag until timeout or success/error
    while (millis() - attemptStartTime < TAG_WAIT_TIMEOUT) {
      // Check for abort (button hold 2+ seconds)
      if (digitalRead(BTN_START) == LOW) {
        unsigned long holdStart = millis();
        while (digitalRead(BTN_START) == LOW && (millis() - holdStart < 2000)) {
          delay(10);
        }
        if (millis() - holdStart >= 2000) {
          // Abort
          oled.clearDisplay();
          oled.setTextSize(1);
          oled.setTextColor(SSD1306_WHITE);
          oled.setCursor(30, 24);
          oled.println("Aborted");
          oled.display();
          setLED(255, 0, 0);
          delay(1000);
          ledOff();
          return false;
        }
      }

      // Poll for tag every 250ms
      if (millis() - lastPollTime < 250) {
        delay(10);
        continue;
      }
      lastPollTime = millis();

      // Blink LED during polling (blue)
      static bool ledState = false;
      if (ledState) {
        setLED(0, 0, 255);
      } else {
        ledOff();
      }
      ledState = !ledState;

      // Try to accumulate measurement
      String msg;
      PaddleDNA::AccumulateResult result = accumulator->accumulate(measurement, &msg);

      Serial.print("Accumulate result: ");
      Serial.print((int)result);
      Serial.print(" - ");
      Serial.println(msg);

      switch (result) {
        case PaddleDNA::AccumulateResult::Success:
          ledOff();
          displayRFIDSuccess();
          return true;

        case PaddleDNA::AccumulateResult::NoTag:
          // Keep polling silently
          break;

        case PaddleDNA::AccumulateResult::TagFull:
          ledOff();
          oled.clearDisplay();
          oled.setTextSize(1);
          oled.setTextColor(SSD1306_WHITE);
          oled.setCursor(15, 16);
          oled.println("Tag is full!");
          oled.setCursor(10, 32);
          oled.println("Use a new tag");
          oled.display();
          pulseLED(255, 0, 0, 3, 300);
          delay(3000);
          return false;

        case PaddleDNA::AccumulateResult::ReadError:
        case PaddleDNA::AccumulateResult::WriteError:
        case PaddleDNA::AccumulateResult::InvalidPayload:
        case PaddleDNA::AccumulateResult::CryptoError:
          // Error encountered - count as a failed attempt
          ledOff();
          attemptNumber++;

          if (attemptNumber < MAX_RETRIES) {
            // Show retry message
            displayRFIDRetry(MAX_RETRIES - attemptNumber);
          }
          // Break out of polling loop to start next attempt
          goto next_attempt;
      }
    }

    // If we get here, timeout occurred (no tag detected)
    ledOff();
    attemptNumber++;
    if (attemptNumber < MAX_RETRIES) {
      displayRFIDRetry(MAX_RETRIES - attemptNumber);
    }

    next_attempt:
    continue;
  }

  // All retries exhausted
  ledOff();
  displayRFIDFinalFailure();
  return false;
}

// ----------------------------- Live Force Overlay ---------------------------
unsigned long g_lastForceDrawMs = 0;
void updateLiveForceLine(bool forceClear) {
  if (g_motionActive) return; // avoid OLED writes during motion
  if (!scale.is_ready()) return;

  unsigned long now = millis();
  if (!forceClear && (now - g_lastForceDrawMs) < 1000) return; // 1 Hz
  g_lastForceDrawMs = now;

  long raw = scale.read();
  float lbs = rawToPounds(raw);

  // Draw a single-line overlay at the bottom without clearing the whole screen
  // Clear the bottom 10px band
  oled.fillRect(0, OLED_HEIGHT-12, OLED_WIDTH, 12, SSD1306_BLACK);
  oled.setCursor(0, OLED_HEIGHT-10);
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.print(F("Force (lb): "));
  oled.println(String(lbs, 3));
  oled.display();
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
  pinMode(BTN_ZERO,  INPUT_PULLUP); // active-LOW
  Serial.println("GPIO pins configured");

  stepperEnable(false);
  Serial.println("Stepper disabled");

  // Initialize RGB LED
  Serial.print("Initializing RGB LED on pin ");
  Serial.println(RGB_LED_PIN);
  rgbLed.begin();
  rgbLed.setBrightness(50); // Not too bright
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
  oled.clearDisplay();
  oled.display();
  Serial.println("OLED ready");

  // Initialize PaddleDNA NFC
  Serial.println("Initializing PaddleDNA NFC...");
  if (!nfc.begin(Wire)) {
    Serial.println(F("NFC initialization failed!"));
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0, 20);
    oled.println("NFC INIT FAILED!");
    oled.setCursor(0, 35);
    oled.println("Check connections");
    oled.display();
    pulseLED(255, 0, 0, 5, 300); // Red pulse error
    delay(3000);
    // Continue anyway - allow force measurements without RFID
  } else {
    Serial.println("NFC initialized successfully");
  }

  // Initialize PaddleDNA Crypto
  Serial.println("Initializing PaddleDNA Crypto...");
  if (!crypto.begin(MACHINE_UUID, PRIVATE_KEY)) {
    Serial.println(F("Crypto initialization failed!"));
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0, 20);
    oled.println("CRYPTO INIT FAILED!");
    oled.display();
    pulseLED(255, 0, 0, 5, 300);
    delay(3000);
    // Continue anyway
  } else {
    Serial.println("Crypto initialized successfully");
  }

  // Create MeasurementAccumulator (paddle UUID auto-detected from tag)
  Serial.println("Creating MeasurementAccumulator...");
  accumulator = new PaddleDNA::MeasurementAccumulator(nfc, crypto, 9);
  Serial.println("MeasurementAccumulator created successfully");

  Serial.println("Initializing HX711 load cell...");
  scale.begin(HX_DOUT, HX_SCK);
  loadCalibration();
  Serial.print("Calibration loaded: ");
  Serial.print(g_calibration);
  Serial.print(" counts/lb, Tare: ");
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

  showSplash();
  Serial.println("Starting rainbow cycle...");
  rainbowCycle(2000); // 2 second rainbow cycle on power-up
  Serial.println("Rainbow cycle complete");
  delay(500);

  // Initialization homing sequence
  Serial.println("Checking limit switch...");
  oledHeader("Initializing...");
  oled.println(F("Checking limit..."));
  oled.display();

  if (!limitHit()) {
    Serial.println("Not at limit, starting homing sequence...");
    oled.println(F("Homing..."));
    oled.display();
    homeToLimitSafe();
    Serial.println("Homing complete");
    oled.println(F("Homed"));
    oled.display();
    delay(400);
  } else {
    Serial.println("Already at home position");
    oled.println(F("Already at home"));
    oled.display();
    delay(400);
  }

  stepperEnable(false);
  Serial.println("=== Setup complete, entering main loop ===\n");
}

void loop() {
  // Idle screen
  Serial.println("Entering idle state");
  ledOff(); // Turn off LED during idle
  oledHeader("Idle");
  if (g_hasResult) {
    oledKV("Last COF",   String(g_lastCOF, 3));
  }
  oled.println(F("Start=Run | Zero=Tar/Cal"));
  oled.display();

  // keep updating live force once per second while idle
  g_motionActive = false;
  while (true) {
    bool sp=false, lp=false, sz=false, lz=false;
    readButton(btnStart, sp, lp);
    readButton(btnZero,  sz, lz);
    if (sp || lp || sz || lz) {
      updateLiveForceLine(true); // clear the overlay line before changing screen
      if (sz) {
        Serial.println("ZERO button short press - Taring...");
        oledHeader("Taring..."); oled.display();
        tareNow();
        oledHeader("Tare Done");
        oledKV("TareRaw", String(g_tareRaw));
        if (g_hasResult) { oledKV("Last COF", String(g_lastCOF, 3)); }
        oled.display();
        Serial.println("Tare complete");
        delay(600);
        break; // return to idle refresh loop
      }
      if (lz) {
        Serial.println("ZERO button long press - Starting calibration...");
        doCalibration3lb();
        break; // return to idle
      }
      if (sp) {
        Serial.println("START button pressed - Running test...");
        RunResult r = runTest();
        g_lastAvgLb = r.avgFrictionLb;
        g_lastCOF   = r.cof;
        g_hasResult = true;

        Serial.print("Test complete! COF: ");
        Serial.println(r.cof, 3);

        dumpTestDataCSV();

        // Display results with "Present NFC tag..." message
        displayTestResults(r.cof, MACHINE_ID);

        // Write to RFID tag (with retry and abort handling)
        Serial.println("Entering RFID write mode...");
        bool rfidSuccess = writeToRFID(r.cof);

        if (rfidSuccess) {
          Serial.println("RFID write successful");
        } else {
          Serial.println("RFID write failed or aborted");
        }

        // Show final result screen
        oledHeader("Result");
        oledKV("COF", String(r.cof, 3));
        oled.print(F("N = "));
        oled.print(NORMAL_FORCE_LB, 2);
        oled.println(F(" lb"));
        if (rfidSuccess) {
          oled.println(F("Data saved to tag"));
        }
        oled.println(F("Press any button..."));
        oled.display();

        // Wait here showing the result until any button input
        while (true) {
          bool a=false,b=false,c=false,d=false;
          readButton(btnStart, a, b);
          readButton(btnZero,  c, d);
          if (a || b || c || d) break;
          updateLiveForceLine();
          delay(10);
        }
        break; // back to idle
      }
    }
    updateLiveForceLine();
    delay(10);
  }
}
