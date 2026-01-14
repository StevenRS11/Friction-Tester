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
#include <math.h>

// ----------------------------- USER CONFIG ----------------------------------
// NOTE: Pin assignments below are for specific ESP32 variant in use
// Adjust these based on your hardware configuration

#define I2C_SDA 8
#define I2C_SCL 9

#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_ADDR    0x3C

#define HX_DOUT  5
#define HX_SCK   6

#define PIN_STEP  17
#define PIN_DIR   16
#define PIN_EN    47   // Active LOW

#define PIN_LIMIT  18  // Limit switch input
#define LIMIT_ACTIVE_LOW 1

#define BTN_START   20  // Start button input
#define BTN_ZERO    21  // Zero/Calibration button input

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

const int    STEP_PULSE_US   = 300; // motion speed (lower = faster)how
const int    HOME_STEP_US    = 600; // homing speed
const int    BACKOFF_STEPS   = 600; // homing backoff
const bool   DIR_FORWARD     = true;
const bool   DIR_HOME_TOWARD_LIMIT = !DIR_FORWARD;

const uint32_t DEBOUNCE_MS   = 30;
const uint32_t LONG_PRESS_MS = 1200;

const float CAL_WEIGHT_LB    = 3.883;   // calibration weight
const float NORMAL_FORCE_LB  = 3.59;  // test normal force
const int HX_SAMPLES_TARE    = 20;      // averaging for tare
const int HX_SAMPLES_MEAS    = 5;       // (unused by non-blocking read)
// ----------------------------------------------------------------------------

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire);
HX711 scale;
Preferences prefs;
Adafruit_NeoPixel rgbLed(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

bool  g_hasResult = false;
float g_lastAvgLb = 0.0f;
float g_lastCOF   = 0.0f;

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
}

// ----------------------------- Motion ---------------------------------------
bool g_motionActive = false; // used to suppress OLED live updates during motion

void homeToLimit() {
  const uint32_t HOMING_TIMEOUT_MS = 20000; // 20 second timeout

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

  oledHeader("Homing...");
  oled.display();
  homeToLimit();

  oledHeader("Running (forward)...");
  oled.println(F("Lowering..."));
  oled.display();

  g_motionActive = true;
  stepperEnable(true);

  // Yellow during lowering phase
  setLED(255, 150, 0);
  moveStepsBlocking(steps_lower,  DIR_FORWARD, STEP_PULSE_US); // ignore
  moveStepsBlocking(steps_noise,  DIR_FORWARD, STEP_PULSE_US); // (currently 0, settling handled by trim)

  oledHeader("Measuring (FWD)...");
  oled.display();
  // Cyan during forward measurement (first 0.25" and last 0.25" are trim/settling)
  setLED(0, 255, 255);
  MeasureResult fwd = measureDuringMove(steps_measure, DIR_FORWARD, STEP_PULSE_US, steps_trim);

  const int END_PAUSE_MS = 600;
  delay(END_PAUSE_MS);

  oledHeader("Measuring (REV)...");
  oled.display();
  // Magenta during reverse measurement (first 0.25" and last 0.25" are trim/settling)
  setLED(255, 0, 255);
  MeasureResult rev = measureDuringMove(steps_measure, !DIR_FORWARD, STEP_PULSE_US, steps_trim);

  oledHeader("Returning...");
  oled.display();
  // Yellow during return
  setLED(255, 150, 0);
  moveStepsBlocking(steps_noise,  !DIR_FORWARD, STEP_PULSE_US); // (currently 0, settling handled by trim)
  moveStepsBlocking(steps_lower,  !DIR_FORWARD, STEP_PULSE_US); // ignore

  homeToLimit();
  stepperEnable(false);
  g_motionActive = false;

  // Debug output: show sample counts and percentile averages
  Serial.print("Forward samples: ");
  Serial.print(fwd.count);
  Serial.print(", 85-95%ile avg: ");
  Serial.println(fwd.avgLb, 4);
  Serial.print("Reverse samples: ");
  Serial.print(rev.count);
  Serial.print(", 85-95%ile avg: ");
  Serial.println(rev.avgLb, 4);

  // Sample-weighted average of percentile averages across the two passes
  double weightedSum = fabs(fwd.avgLb) * (double)fwd.count + fabs(rev.avgLb) * (double)rev.count;
  long   totalCount  = fwd.count + rev.count;
  double avgLbTwoPass = (totalCount > 0) ? (weightedSum / (double)totalCount) : 0.0;

  // Free allocated memory
  if (fwd.samples) free(fwd.samples);
  if (rev.samples) free(rev.samples);

  // Test complete - pulse green 3 times
  pulseLED(0, 255, 0, 3, 300);

  RunResult rr{};
  rr.avgFrictionLb = (float)avgLbTwoPass;

  if (NORMAL_FORCE_LB == 0.0f) {
    Serial.println("ERROR: Division by zero - NORMAL_FORCE_LB is 0!");
    rr.cof = 0.0f;
  } else {
    rr.cof = (float)(avgLbTwoPass / NORMAL_FORCE_LB);
  }

  return rr;
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
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oled.clearDisplay();
  oled.display();
  Serial.println("OLED ready");

  Serial.println("Initializing HX711 load cell...");
  scale.begin(HX_DOUT, HX_SCK);
  loadCalibration();
  Serial.print("Calibration loaded: ");
  Serial.print(g_calibration);
  Serial.print(" counts/lb, Tare: ");
  Serial.println(g_tareRaw);

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
    homeToLimit();
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
        oledHeader("Result");
        oledKV("COF", String(r.cof, 3));
        oled.println(F("Assuming N = 3 lb"));
        oled.println(F("Press START to test again"));
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
