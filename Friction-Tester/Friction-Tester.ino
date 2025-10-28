/*
  ESP32 Paddle COF Tester (continuous bidirectional test, non-blocking HX711)
  - DRV8825 + NEMA17
  - HX711 load cell amp
  - SSD1306 OLED (I2C)
  - Buttons: START (GPIO18), ZERO/CAL (GPIO19) -> INPUT_PULLUP, wire to GND
  - Limit switch: GPIO32 -> INPUT_PULLUP, active-LOW, wire to GND
  - On normal boot: homes to limit switch automatically if not already pressed
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HX711.h>
#include <Preferences.h>
#include <math.h>

// ----------------------------- USER CONFIG ----------------------------------
#define I2C_SDA 22
#define I2C_SCL 23

#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_ADDR    0x3C

#define HX_DOUT  33
#define HX_SCK   25

#define PIN_STEP  17
#define PIN_DIR   16
#define PIN_EN    27   // Active LOW

#define PIN_LIMIT  32
#define LIMIT_ACTIVE_LOW 1

#define BTN_START   18
#define BTN_ZERO    19

const int   FULL_STEPS_PER_REV = 200;     // 1.8° motor
const float MICROSTEP           = 16.0;   // 1/16 microstepping
const float REVS_TOTAL          = 19.0;   // 6.0 inches total
const float INCHES_TOTAL        = 6.0;
const float REVS_PER_INCH       = REVS_TOTAL / INCHES_TOTAL;
const float STEPS_PER_REV       = FULL_STEPS_PER_REV * MICROSTEP;
const float STEPS_PER_INCH      = STEPS_PER_REV * REVS_PER_INCH;

const float SEG_LOWER_IN   = 2.5;  // ignore: lowering
const float SEG_NOISE_IN   = 0.5;  // ignore: settle
const float SEG_MEASURE_IN = 3.0;  // measure window

const int    STEP_PULSE_US   = 600; // motion speed (lower = faster)
const int    HOME_STEP_US    = 600; // homing speed
const int    BACKOFF_STEPS   = 800; // homing backoff
const bool   DIR_FORWARD     = true;
const bool   DIR_HOME_TOWARD_LIMIT = !DIR_FORWARD;

const uint32_t DEBOUNCE_MS   = 30;
const uint32_t LONG_PRESS_MS = 1200;

const float CAL_WEIGHT_LB    = 3.085;   // calibration weight
const float NORMAL_FORCE_LB  = 2.7842;  // test normal force
const int HX_SAMPLES_TARE    = 20;      // averaging for tare
const int HX_SAMPLES_MEAS    = 5;       // (unused by non-blocking read)
// ----------------------------------------------------------------------------

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire);
HX711 scale;
Preferences prefs;

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
struct MeasureResult { double avgLb; long count; };

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
MeasureResult measureDuringMove(long steps, bool forward, int pulseUs);
RunResult runTest();
bool   readButton(Btn& b, bool& shortPress, bool& longPress);
void   updateLiveForceLine(bool forceClear=false);

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

void saveCalibration() {
  prefs.begin(PREFS_NAMESPACE, false);
  prefs.putFloat(KEY_CAL, g_calibration);
  prefs.putInt(KEY_TARE, (int)g_tareRaw);
  prefs.end();
}

void loadCalibration() {
  prefs.begin(PREFS_NAMESPACE, true);
  float cal = prefs.getFloat(KEY_CAL, NAN);
  int tare  = prefs.getInt(KEY_TARE, 0);
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

float rawToPounds(long raw) { return (float)(raw - g_tareRaw) / g_calibration; }

void tareNow() { g_tareRaw = hxReadRawAvg(HX_SAMPLES_TARE); saveCalibration(); }

void doCalibration3lb() {
  // ---- Step 1: Tare (zero-load) ----
  oledHeader("CAL: Step 1/2 (Tare)");
  oled.println(F("Remove all load"));
  oled.println(F("Press START to tare"));
  oled.display();

  while (digitalRead(BTN_START) == HIGH) delay(5);
  while (digitalRead(BTN_START) == LOW)  delay(5);

  oledHeader("CAL: Taring...");
  oled.display();
  g_tareRaw = hxReadRawAvg(HX_SAMPLES_TARE);

  // ---- Step 2: Known weight ----
  oledHeader("CAL: Step 2/2 (3 lb)");
  oled.println(F("Place 3.00 lb weight"));
  oled.println(F("Press START to sample"));
  oled.display();

  while (digitalRead(BTN_START) == HIGH) delay(5);
  while (digitalRead(BTN_START) == LOW)  delay(5);

  long raw3 = hxReadRawAvg(HX_SAMPLES_TARE);
  long delta = raw3 - g_tareRaw; // counts due to 3 lb

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
  oledKV("Counts@3lb", String(delta));
  oledKV("Cal (cnt/lb)", String(g_calibration, 2));
  oledKV("TareRaw", String(g_tareRaw));
  oled.display();
  delay(1500);
}

// ----------------------------- Motion ---------------------------------------
bool g_motionActive = false; // used to suppress OLED live updates during motion

void homeToLimit() {
  g_motionActive = true;
  stepperEnable(true);
  setDir(DIR_HOME_TOWARD_LIMIT);
  while (!limitHit()) doStepBlocking(HOME_STEP_US);

  setDir(!DIR_HOME_TOWARD_LIMIT);
  for (int i=0; i<BACKOFF_STEPS; i++) doStepBlocking(HOME_STEP_US);
  setDir(DIR_HOME_TOWARD_LIMIT);
  while (!limitHit()) doStepBlocking(HOME_STEP_US);
  setDir(!DIR_HOME_TOWARD_LIMIT);
  for (int i=0; i<BACKOFF_STEPS/2; i++) doStepBlocking(HOME_STEP_US);

  stepperEnable(false);
  g_motionActive = false;
}

void moveStepsBlocking(long steps, bool forward, int pulseUs) {
  setDir(forward);
  for (long i=0; i<steps; i++) doStepBlocking(pulseUs);
}

// Non-blocking measurement during motion
MeasureResult measureDuringMove(long steps, bool forward, int pulseUs) {
  setDir(forward);
  double sumF = 0.0;
  long   nF   = 0;
  for (long i = 0; i < steps; i++) {
    doStepBlocking(pulseUs);        // keep motion timing tight
    if (scale.is_ready()) {         // read only if ready (no waiting)
      long raw = scale.read();
      float lbs = rawToPounds(raw);
      sumF += lbs;
      nF++;
    }
  }
  MeasureResult mr;
  mr.avgLb = (nF > 0) ? (sumF / (double)nF) : 0.0;
  mr.count = nF;
  return mr;
}

RunResult runTest() {
  const long steps_lower   = lround(SEG_LOWER_IN   * STEPS_PER_INCH);
  const long steps_noise   = lround(SEG_NOISE_IN   * STEPS_PER_INCH);
  const long steps_measure = lround(SEG_MEASURE_IN * STEPS_PER_INCH);

  oledHeader("Homing...");
  oled.display();
  homeToLimit();

  oledHeader("Running (forward)...");
  oled.println(F("Lowering..."));
  oled.display();

  g_motionActive = true;
  stepperEnable(true);
  moveStepsBlocking(steps_lower,  DIR_FORWARD, STEP_PULSE_US); // ignore
  moveStepsBlocking(steps_noise,  DIR_FORWARD, STEP_PULSE_US); // ignore

  oledHeader("Measuring (FWD)...");
  oled.display();
  MeasureResult fwd = measureDuringMove(steps_measure, DIR_FORWARD, STEP_PULSE_US);

  const int END_PAUSE_MS = 600;
  delay(END_PAUSE_MS);

  oledHeader("Measuring (REV)...");
  oled.display();
  MeasureResult rev = measureDuringMove(steps_measure, !DIR_FORWARD, STEP_PULSE_US);

  oledHeader("Returning...");
  oled.display();
  moveStepsBlocking(steps_noise,  !DIR_FORWARD, STEP_PULSE_US); // ignore
  moveStepsBlocking(steps_lower,  !DIR_FORWARD, STEP_PULSE_US); // ignore

  homeToLimit();
  stepperEnable(false);
  g_motionActive = false;

  // Sample-weighted average of magnitudes across the two passes
  double weightedSum = fabs(fwd.avgLb) * (double)fwd.count + fabs(rev.avgLb) * (double)rev.count;
  long   totalCount  = fwd.count + rev.count;
  double avgLbTwoPass = (totalCount > 0) ? (weightedSum / (double)totalCount) : 0.0;

  RunResult rr{};
  rr.avgFrictionLb = (float)avgLbTwoPass;
  rr.cof = (float)(avgLbTwoPass / NORMAL_FORCE_LB);
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
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_LIMIT, INPUT_PULLUP); // active-LOW
  pinMode(BTN_START, INPUT_PULLUP); // active-LOW
  pinMode(BTN_ZERO,  INPUT_PULLUP); // active-LOW

  stepperEnable(false);

  Wire.begin(I2C_SDA, I2C_SCL);
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oled.clearDisplay();
  oled.display();

  scale.begin(HX_DOUT, HX_SCK);
  loadCalibration();

  showSplash();
  delay(500);

  // Initialization homing sequence
  oledHeader("Initializing...");
  oled.println(F("Checking limit..."));
  oled.display();

  if (!limitHit()) {
    oled.println(F("Homing..."));
    oled.display();
    homeToLimit();
    oled.println(F("Homed"));
    oled.display();
    delay(400);
  } else {
    oled.println(F("Already at home"));
    oled.display();
    delay(400);
  }

  stepperEnable(false);
}

void loop() {
  // Idle screen
  oledHeader("Idle");
  oledKV("Cal", String(g_calibration, 2));
  oledKV("TareRaw", String(g_tareRaw));
  if (g_hasResult) {
    oledKV("Last F (lb)", String(g_lastAvgLb, 3));
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
        oledHeader("Taring..."); oled.display();
        tareNow();
        oledHeader("Tare Done");
        oledKV("TareRaw", String(g_tareRaw));
        if (g_hasResult) { oledKV("Last COF", String(g_lastCOF, 3)); }
        oled.display();
        delay(600);
        break; // return to idle refresh loop
      }
      if (lz) {
        doCalibration3lb();
        break; // return to idle
      }
      if (sp) {
        RunResult r = runTest();
        g_lastAvgLb = r.avgFrictionLb;
        g_lastCOF   = r.cof;
        g_hasResult = true;

        oledHeader("Result");
        oledKV("Avg F (lb)", String(r.avgFrictionLb, 3));
        oledKV("COF", String(r.cof, 3));
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
