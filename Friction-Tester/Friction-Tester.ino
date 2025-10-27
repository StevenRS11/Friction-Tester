/*
  ESP32 Paddle COF Tester (DIR/STEP swapped)
  - DRV8825 stepper (DIR/STEP/EN)
  - HX711 load cell amp for tangential (friction) force
  - SSD1306 128x64 OLED (I2C)
  - Two buttons: START, ZERO/CAL (short press = tare; long press = 3 lb calibration)
  - Limit switch for homing zero inches
  - Travel: 19 full revolutions = 6.0 inches
    * Ignore first 2.5" (lowering)
    * Ignore next 0.5" (noise)
    * Measure last 3.0"
  - COF = (avg friction force) / (3.0 lb normal force)
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HX711.h>
#include <Preferences.h>

// ----------------------------- USER CONFIG ----------------------------------
// I2C pins (OLED)
#define I2C_SDA 22
#define I2C_SCL 23

// OLED
#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_ADDR    0x3C

// HX711 pins
#define HX_DOUT  33
#define HX_SCK   25

// DRV8825 pins (STEP and DIR swapped)
#define PIN_STEP  17
#define PIN_DIR   16
#define PIN_EN    27   // Active LOW (LOW = enabled)

// Limit switch
#define PIN_LIMIT  32
#define LIMIT_ACTIVE_LOW 1 // 1 if switch pulls pin LOW when triggered

// Buttons
#define BTN_START   34
#define BTN_ZERO    35

// Stepper configuration
const int   FULL_STEPS_PER_REV = 200;     // 1.8° motor
const float MICROSTEP            = 16.0;  // DRV8825 microstep setting
const float REVS_TOTAL          = 19.0;   // 19 full revs = 6 inches
const float INCHES_TOTAL        = 6.0;
const float REVS_PER_INCH       = REVS_TOTAL / INCHES_TOTAL;
const float STEPS_PER_REV       = FULL_STEPS_PER_REV * MICROSTEP;
const float STEPS_PER_INCH      = STEPS_PER_REV * REVS_PER_INCH;

// Segment lengths (inches)
const float SEG_LOWER_IN   = 2.5;
const float SEG_NOISE_IN   = 0.5;
const float SEG_MEASURE_IN = 3.0;

// Motion parameters
const int    STEP_PULSE_US   = 600;
const int    HOME_STEP_US    = 1000;
const int    BACKOFF_STEPS   = 800;
const bool   DIR_FORWARD     = true;
const bool   DIR_HOME_TOWARD_LIMIT = !DIR_FORWARD;

// Buttons timing
const uint32_t DEBOUNCE_MS   = 30;
const uint32_t LONG_PRESS_MS = 1200;

// Known normal force
const float NORMAL_FORCE_LB  = 3.0;

// HX711 averaging
const int HX_SAMPLES_TARE    = 20;
const int HX_SAMPLES_MEAS    = 5;
// ----------------------------------------------------------------------------

struct RunResult {
  float avgFrictionLb;
  float cof;
};

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire);
HX711 scale;
Preferences prefs;

const char* PREFS_NAMESPACE = "cof";
const char* KEY_CAL         = "calib";
const char* KEY_TARE        = "tare";

float g_calibration = 1000.0f;
long  g_tareRaw     = 0;

struct Btn {
  uint8_t pin;
  bool last;
  uint32_t lastChange;
  uint32_t downAt;
  bool longSent;
};
Btn btnStart{BTN_START, true, 0, 0, false};
Btn btnZero {BTN_ZERO,  true, 0, 0, false};

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
  oled.print(": ");
  oled.println(v);
}

void showSplash() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println(F("ESP32 Paddle COF Tester"));
  oled.println(F("DRV8825 + HX711 + OLED"));
  oled.println();
  oled.println(F("BTN Start: Run Test"));
  oled.println(F("BTN Zero : Tare/Cal"));
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

float rawToPounds(long raw) {
  return (float)(raw - g_tareRaw) / g_calibration;
}

void tareNow() {
  g_tareRaw = hxReadRawAvg(HX_SAMPLES_TARE);
  saveCalibration();
}

void doCalibration3lb() {
  oledHeader("Calibration (3 lb)");
  oled.println(F("1) Remove load"));
  oled.println(F("2) Press START to tare"));
  oled.display();

  while (digitalRead(BTN_START) == HIGH) delay(5);
  while (digitalRead(BTN_START) == LOW) delay(5);
  g_tareRaw = hxReadRawAvg(HX_SAMPLES_TARE);

  oledHeader("Calibration (3 lb)");
  oled.println(F("Place 3 lb weight"));
  oled.println(F("Press START to confirm"));
  oled.display();

  while (digitalRead(BTN_START) == HIGH) delay(5);
  while (digitalRead(BTN_START) == LOW) delay(5);

  long raw3 = hxReadRawAvg(HX_SAMPLES_TARE);
  long delta = raw3 - g_tareRaw;
  if (abs(delta) < 100) {
    oledHeader("Calibration FAILED");
    oled.display();
    delay(2000);
    return;
  }
  g_calibration = (float)delta / NORMAL_FORCE_LB;
  saveCalibration();

  oledHeader("Calibration DONE");
  oledKV("Cal", String(g_calibration, 2));
  oledKV("TareRaw", String(g_tareRaw));
  oled.display();
  delay(1500);
}

void homeToLimit() {
  stepperEnable(true);
  setDir(DIR_HOME_TOWARD_LIMIT);

  while (!limitHit()) doStepBlocking(HOME_STEP_US);

  setDir(!DIR_HOME_TOWARD_LIMIT);
  for (int i=0; i<800; i++) doStepBlocking(HOME_STEP_US);
  setDir(DIR_HOME_TOWARD_LIMIT);
  while (!limitHit()) doStepBlocking(HOME_STEP_US);
  setDir(!DIR_HOME_TOWARD_LIMIT);
  for (int i=0; i<400; i++) doStepBlocking(HOME_STEP_US);
}

void moveStepsBlocking(long steps, bool forward, int pulseUs) {
  setDir(forward);
  for (long i=0; i<steps; i++) doStepBlocking(pulseUs);
}



RunResult runTest() {
  long steps_lower   = lround(SEG_LOWER_IN   * STEPS_PER_INCH);
  long steps_noise   = lround(SEG_NOISE_IN   * STEPS_PER_INCH);
  long steps_measure = lround(SEG_MEASURE_IN * STEPS_PER_INCH);

  oledHeader("Homing...");
  oled.display();
  homeToLimit();

  oledHeader("Running Test...");
  oled.println(F("Lowering..."));
  oled.display();
  stepperEnable(true);
  moveStepsBlocking(steps_lower, DIR_FORWARD, STEP_PULSE_US);

  oledHeader("Running Test...");
  oled.println(F("Noise zone..."));
  oled.display();
  moveStepsBlocking(steps_noise, DIR_FORWARD, STEP_PULSE_US);

  oledHeader("Running Test...");
  oled.println(F("Measuring..."));
  oled.display();

  double sumF = 0.0;
  long nF = 0;
  for (long i=0; i<steps_measure; i++) {
    doStepBlocking(STEP_PULSE_US);
    long raw = hxReadRawAvg(HX_SAMPLES_MEAS);
    float lbs = rawToPounds(raw);
    sumF += lbs;
    nF++;
  }

  oledHeader("Returning...");
  oled.display();
  moveStepsBlocking(steps_lower + steps_noise + steps_measure, !DIR_FORWARD, STEP_PULSE_US);
  stepperEnable(false);

  RunResult rr{};
  rr.avgFrictionLb = (nF > 0) ? sumF / nF : 0;
  rr.cof = fabs(rr.avgFrictionLb) / NORMAL_FORCE_LB;
  return rr;
}

bool readButton(Btn& b, bool& shortPress, bool& longPress) {
  shortPress = false;
  longPress  = false;
  bool cur = digitalRead(b.pin);
  uint32_t now = millis();

  if (cur != b.last && (now - b.lastChange) > DEBOUNCE_MS) {
    b.last = cur;
    b.lastChange = now;
    if (cur == LOW) {
      b.downAt = now;
      b.longSent = false;
    } else {
      uint32_t held = now - b.downAt;
      if (!b.longSent && held >= DEBOUNCE_MS && held < LONG_PRESS_MS)
        shortPress = true;
    }
  }

  if (b.last == LOW && !b.longSent && (now - b.downAt) >= LONG_PRESS_MS) {
    longPress = true;
    b.longSent = true;
  }
  return (shortPress || longPress);
}

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_LIMIT, INPUT);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_ZERO,  INPUT_PULLUP);

  stepperEnable(false);

  Wire.begin(I2C_SDA, I2C_SCL);
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oled.clearDisplay();
  oled.display();

  //scale.begin(HX_DOUT, HX_SCK);
  loadCalibration();

  showSplash();
  delay(1200);
}

void loop() {
  oledHeader("Idle");
  oledKV("Cal", String(g_calibration, 2));
  oledKV("TareRaw", String(g_tareRaw));
  oled.println(F("Start=Run | Zero=Tar/Cal"));
  oled.display();

  bool sp=false, lp=false, sz=false, lz=false;
  while (true) {
    readButton(btnStart, sp, lp);
    readButton(btnZero,  sz, lz);
    if (sp || lp || sz || lz) break;
    delay(10);
  }

  if (sz) {
    oledHeader("Taring...");
    oled.display();
    tareNow();
    oledHeader("Tare Done");
    oledKV("TareRaw", String(g_tareRaw));
    oled.display();
    delay(800);
    return;
  }

  if (lz) {
    doCalibration3lb();
    return;
  }

  if (sp) {
    RunResult r = runTest();
    oledHeader("Result");
    oledKV("Avg F (lb)", String(r.avgFrictionLb, 3));
    oledKV("COF", String(r.cof, 3));
    oled.display();
    delay(5000);
    return;
  }
}
