# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an ESP32-S3-based Coefficient of Friction (COF) tester for paddle surfaces. The device uses a stepper motor to drag a test paddle across a surface while measuring friction force with a load cell. It performs bidirectional tests (forward and reverse passes) to calculate the average COF, then writes results to NFC tags via the PaddleDNA library.

**Hardware Stack:**
- ESP32-S3-ZERO microcontroller
- DRV8825 stepper driver with NEMA17 motor (1.8° steps, 1/16 microstepping)
- HX711 load cell amplifier (24-bit ADC for force measurement)
- SSD1306 OLED display (128x64, I2C)
- PN532 NFC reader (I2C, shared bus with OLED)
- Onboard RGB LED (WS2812/NeoPixel on GPIO48)
- Limit switch for homing (GPIO4, active-LOW)
- Two control buttons: START (GPIO10) and ZERO/CAL (GPIO9), both active-LOW with external 10K pullups

## Build and Upload

This is an Arduino project for ESP32-S3. To build and upload:

```bash
# Using arduino-cli:
arduino-cli compile --fqbn esp32:esp32:esp32s3 Friction-Tester.ino
arduino-cli upload -p <PORT> --fqbn esp32:esp32:esp32s3 Friction-Tester.ino

# Monitor serial output (115200 baud):
arduino-cli monitor -p <PORT>
```

**Required Libraries:**
- Adafruit_GFX
- Adafruit_SSD1306
- Adafruit_NeoPixel
- HX711 (Bogdan Necula's library)
- PaddleDNA (custom library for NFC measurement storage)

## Architecture

### Dual-Core Architecture (FreeRTOS)

The system uses ESP32's dual-core capability to separate time-critical motion from force sampling:

**Core 1 - Motion Task** (`motionTask()`, priority 3):
- Executes pure stepping operations with tight timing
- Handles homing, lowering, measurement passes, and return moves
- Receives commands via `motionCommandQueue` (FreeRTOS queue)
- Signals completion via `motionCompleteSemaphore`

**Core 0 - Force Sampling Task** (`forceSamplingTask()`, priority 2):
- Monitors `g_collectSamples` flag during measurement phases
- Reads HX711 opportunistically via `scale.is_ready()` non-blocking checks
- Stores samples in `g_fwdSamples[]` or `g_revSamples[]` based on current phase
- Yields via `vTaskDelay(1)` to avoid starving other Core 0 tasks

**Inter-core Communication:**
```
Core 0 (main loop)           Core 1 (motion task)
       |                            |
       |--[MotionRequest]-->        |  (via motionCommandQueue)
       |                     [execute motion]
       |                            |
       |<--[semaphore]------        |  (via motionCompleteSemaphore)
```

### Motion Profile

The system uses a linear motion profile divided into segments:

1. **Lowering segment** (2.5"): Initial descent, no data collection
2. **Forward measurement** (3.0" total):
   - First 0.25": Trim zone (motion, no sampling)
   - Middle 2.5": Active data collection window
   - Last 0.25": Trim zone (motion, no sampling)
3. **Pause** (600ms): Direction reversal
4. **Reverse measurement** (3.0" total): Same trim/measure/trim structure
5. **Return** (2.5"): Return to home position

**Key calculations:**
- `STEPS_PER_REV = 200 × 16 = 3200 steps`
- `STEPS_PER_INCH = 3200 × (19/6) ≈ 10,133 steps/inch`
- Motion speed: `STEP_PULSE_US = 300µs` (~1667 steps/sec)

### Measurement & COF Calculation

**Percentile-based averaging** (`calculatePercentileAverage()`):
1. Sort samples by absolute value
2. Discard top 5% (noise spikes)
3. Average 85th-95th percentile range

**Bidirectional sample-weighted average:**
```
COF = (|fwd_avg| × fwd_samples + |rev_avg| × rev_samples) / (total_samples × NORMAL_FORCE_LB)
```

### NFC/RFID Integration

After test completion, results are written to NFC tags using PaddleDNA library:

**Components:**
- `PaddleDNA::NFC nfc`: Handles PN532 communication
- `PaddleDNA::Crypto crypto`: Signs measurements (stub implementation)
- `PaddleDNA::MeasurementAccumulator accumulator`: Manages tag payload

**Write flow** (`writeToRFID()`):
1. Display "Present NFC" prompt with COF result
2. Poll for tag every 250ms (blue LED blink)
3. On tag detection, call `accumulator->accumulate(measurement)`
4. Handle retry logic (5 attempts max)
5. Abort on 2-second button hold

### Calibration System

Uses ESP32 Preferences (NVS) to persist:
- `g_calibration`: counts per pound (float)
- `g_tareRaw`: zero-load offset (long)

**Calibration procedure (long-press ZERO button):**
1. Tare: averages 20 HX711 readings with no load
2. Known weight: user places calibration weight (`CAL_WEIGHT_LB`), system samples and calculates counts/lb

**Quick tare (short-press ZERO button):**
Re-zeros without recalculating calibration factor.

## Key Configuration Constants

Located in USER CONFIG section (lines 22-96):

**Pin assignments (ESP32-S3-ZERO):**
- I2C: SDA=12, SCL=11 (shared: OLED + NFC)
- HX711: DOUT=5, SCK=6
- Stepper: STEP=7, DIR=2, EN=3
- Limit: GPIO4
- Buttons: START=10, ZERO=9
- RGB LED: GPIO48

**Motion parameters:**
- `STEP_PULSE_US = 300`: Motion speed (lower = faster)
- `HOME_STEP_US = 600`: Homing speed (slower/safer)
- `BACKOFF_STEPS = 600`: Distance to back off from limit

**Test geometry:**
- `SEG_LOWER_IN = 2.5`: Lowering phase distance
- `SEG_MEASURE_IN = 3.0`: Total measurement segment length
- `SEG_TRIM_IN = 0.25`: Trim distance at start/end of measurement

**Calibration:**
- `CAL_WEIGHT_LB = 3.883`: Known calibration weight in pounds
- `NORMAL_FORCE_LB = 3.59`: Expected normal force during test
- `HX_SAMPLES_TARE = 20`: Averaging samples for calibration

**Machine identity:**
- `MACHINE_ID`: Display identifier
- `MACHINE_UUID[16]`: Unique ID embedded in NFC payloads

## Common Modifications

**To change test speed:**
Modify `STEP_PULSE_US`. Lower = faster, but too low may cause missed steps.

**To adjust measurement window:**
Modify `SEG_MEASURE_IN` for total segment length and `SEG_TRIM_IN` for settling distances. Actual measurement distance = `SEG_MEASURE_IN - (2 × SEG_TRIM_IN)`.

**To change normal force assumption:**
Update `NORMAL_FORCE_LB` and recalibrate if paddle weight changes.

**To configure for a new machine:**
Update `MACHINE_ID` and generate a new `MACHINE_UUID`.

## LED Status Indicators

- **RGB cycle on boot**: Startup self-test
- **Red**: Tare in progress
- **Green (pulse)**: Limit switch hit / test complete
- **Yellow**: Lowering or returning phase
- **Cyan**: Forward measurement pass
- **Magenta**: Reverse measurement pass
- **Blue (blink)**: Waiting for NFC tag
- **Off**: Idle state
