# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an ESP32-based Coefficient of Friction (COF) tester for paddle surfaces. The device uses a stepper motor to drag a test paddle across a surface while measuring friction force with a load cell. It performs bidirectional tests (forward and reverse passes) to calculate the average COF.

**Hardware Stack:**
- ESP32 microcontroller
- DRV8825 stepper driver with NEMA17 motor (1.8° steps, 1/16 microstepping)
- HX711 load cell amplifier (24-bit ADC for force measurement)
- SSD1306 OLED display (128x64, I2C)
- Limit switch for homing (GPIO32, active-LOW)
- Two control buttons: START (GPIO18) and ZERO/CAL (GPIO19), both active-LOW with INPUT_PULLUP

## Build and Upload

This is an Arduino project for ESP32. To build and upload:

```bash
# Using Arduino IDE:
# 1. Open Friction-Tester/Friction-Tester.ino
# 2. Select board: ESP32 Dev Module (or appropriate ESP32 variant)
# 3. Select the correct COM port
# 4. Click Upload

# Using arduino-cli:
arduino-cli compile --fqbn esp32:esp32:esp32 Friction-Tester/Friction-Tester.ino
arduino-cli upload -p <PORT> --fqbn esp32:esp32:esp32 Friction-Tester/Friction-Tester.ino

# Monitor serial output:
arduino-cli monitor -p <PORT>
```

**Required Libraries:**
- Wire (built-in)
- Adafruit_GFX
- Adafruit_SSD1306
- HX711 (Bogdan Necula's library)
- Preferences (ESP32 built-in)

## Architecture

### Motion Control Architecture

The system uses a linear motion profile divided into segments:

1. **Lowering segment** (2.5"): Initial descent ignored during measurement
2. **Noise settling segment** (0.5"): Allows vibrations to stabilize
3. **Measurement segment** (3.0"): Active data collection window
4. Total travel: 6.0 inches = 19 motor revolutions

**Key calculations:**
- `STEPS_PER_REV = 200 steps × 16 microsteps = 3200`
- `STEPS_PER_INCH = 3200 × (19/6) ≈ 10,133 steps/inch`
- Motion speed controlled by `STEP_PULSE_US` (300µs = ~1667 steps/sec)

### Measurement Architecture

**Non-blocking HX711 reading during motion:**
The system uses `scale.is_ready()` checks during motion to opportunistically read force samples without blocking the stepper timing. This maintains consistent motion speed while gathering as many samples as possible.

```
measureDuringMove():
  for each step:
    doStepBlocking()     // tight timing for smooth motion
    if scale.is_ready(): // non-blocking check
      read and accumulate force sample
```

**COF Calculation:**
- Bidirectional test: forward pass + reverse pass
- Sample-weighted average: `COF = (|fwd.force| × fwd.samples + |rev.force| × rev.samples) / (total_samples × NORMAL_FORCE_LB)`
- Assumes `NORMAL_FORCE_LB = 2.7842 lb` from paddle weight

### Calibration System

Uses ESP32 Preferences (NVS) to persist two values:
- `g_calibration`: counts per pound (float)
- `g_tareRaw`: zero-load offset (long)

**Calibration procedure (long-press ZERO button):**
1. Tare: averages 20 HX711 readings with no load
2. Known weight: user places 3.085 lb weight, system samples and calculates `counts/lb`

**Quick tare (short-press ZERO button):**
Re-zeros without recalculating calibration factor.

### State Machine & Button Handling

**Main loop structure:**
```
loop():
  Display idle screen
  while(waiting):
    readButton() for START and ZERO
    updateLiveForceLine() at 1Hz
    on button press -> execute action -> break to refresh
```

**Button debouncing (Friction-Tester.ino:329-346):**
- Uses state machine per button with timestamps
- `DEBOUNCE_MS = 30ms`
- Short press: release before `LONG_PRESS_MS` (1200ms)
- Long press: held ≥1200ms (fires once via `longSent` flag)

### Homing Logic

**Auto-homing on boot (setup():394-410):**
- Checks if limit already pressed → skip homing
- Otherwise: performs full homing sequence

**Homing sequence (homeToLimit():237-252):**
1. Move toward limit at slow speed (`HOME_STEP_US = 600µs`)
2. Back off 600 steps
3. Slowly approach limit again
4. Back off 300 steps for consistent starting position

**Critical detail:** Uses `g_motionActive` flag to suppress OLED live updates during motion (prevents I2C conflicts with step timing).

## Key Configuration Constants

Located in USER CONFIG section (lines 19-65):

**Pin assignments:**
- I2C: SDA=8, SCL=9
- HX711: DOUT=5, SCK=6
- Stepper: STEP=17, DIR=16, EN=47
- Limit: GPIO18 (note: shares pin with BTN_START, likely a documentation inconsistency)
- Buttons: START=18, ZERO=19

**Motion parameters:**
- `STEP_PULSE_US`: Lower values = faster motion (300µs default)
- `HOME_STEP_US`: Homing speed (600µs = slower/safer)
- `BACKOFF_STEPS`: Distance to back off from limit (600 steps)

**Test geometry:**
- `REVS_TOTAL`, `INCHES_TOTAL`: Defines steps-to-distance conversion
- `SEG_LOWER_IN`, `SEG_NOISE_IN`, `SEG_MEASURE_IN`: Test segment lengths

**Calibration:**
- `CAL_WEIGHT_LB`: Known calibration weight (3.085 lb)
- `NORMAL_FORCE_LB`: Expected normal force during test (2.7842 lb)
- `HX_SAMPLES_TARE`: Averaging samples for calibration (20)

## Common Modifications

**To change test speed:**
Modify `STEP_PULSE_US` (line 52). Lower = faster, but too low may cause missed steps.

**To adjust measurement window:**
Modify `SEG_MEASURE_IN` (line 50). Ensure total travel doesn't exceed mechanical limits.

**To change normal force assumption:**
Update `NORMAL_FORCE_LB` (line 62) and recalibrate if paddle weight changes.

**To adjust homing behavior:**
Modify `HOME_STEP_US` (speed) or `BACKOFF_STEPS` (line 54) for more/less aggressive homing.
