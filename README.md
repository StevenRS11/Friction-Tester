# ESP32 Paddle COF Tester

## Overview

This is an ESP32-based Coefficient of Friction (COF) tester for paddle surfaces. The device uses a stepper motor to drag a test paddle across a surface while measuring friction force with a load cell. It performs bidirectional tests (forward and reverse passes) to calculate the average COF using percentile-based averaging.

## Hardware Requirements

- **Microcontroller**: ESP32 (S3 variant configured, adjust pins for your board)
- **Motor Driver**: DRV8825 stepper driver
- **Motor**: NEMA17 stepper motor (1.8° steps, configured for 1/16 microstepping)
- **Load Cell**: HX711 load cell amplifier (24-bit ADC for force measurement)
- **Display**: SSD1306 OLED display (128x64, I2C)
- **Sensors**: Limit switch for homing (active-LOW with INPUT_PULLUP)
- **Controls**: Two buttons - START and ZERO/CAL (active-LOW with INPUT_PULLUP)
- **LED**: Onboard RGB LED (ESP32-S3)

## Pin Configuration

**IMPORTANT**: Pin assignments in the code are configured for a specific ESP32 variant. Before uploading, verify and adjust the following pin definitions in the USER CONFIG section:

```cpp
#define I2C_SDA 8
#define I2C_SCL 9
#define HX_DOUT  5
#define HX_SCK   6
#define PIN_STEP  17
#define PIN_DIR   16
#define PIN_EN    47
#define PIN_LIMIT  18
#define BTN_START   20
#define BTN_ZERO    21
#define RGB_LED_PIN 48
```

## Quick Start

1. Install required libraries:
   - Wire (built-in)
   - Adafruit_GFX
   - Adafruit_SSD1306
   - HX711 (Bogdan Necula's library)
   - Preferences (ESP32 built-in)
   - Adafruit_NeoPixel

2. Adjust pin definitions for your ESP32 board

3. Upload the code using Arduino IDE or arduino-cli:
   ```bash
   arduino-cli compile --fqbn esp32:esp32:esp32 Friction-Tester/Friction-Tester.ino
   arduino-cli upload -p <PORT> --fqbn esp32:esp32:esp32 Friction-Tester/Friction-Tester.ino
   ```

4. Calibrate the load cell (long-press ZERO button)

5. Run tests (press START button)

## Features

### Measurement Method
- **Percentile-based averaging**: Discards top 5% of measurements, averages 85th-95th percentile
- **Bidirectional testing**: Forward and reverse passes for averaged results
- **Intelligent trimming**: Skips first and last 0.25" of each pass to avoid transient effects
- **Non-blocking sampling**: Opportunistically samples HX711 during motion without disrupting stepper timing

### Motion Profile
- **Lowering phase**: 2.5" initial descent (ignored)
- **Measurement segment**: 3.0" total
  - First 0.25": Settling/trim (motion but no sampling)
  - Middle 2.5": Active data collection
  - Last 0.25": Trim (motion but no sampling)
- **Total travel**: 5.5 inches per test

### Calibration System
- Persistent storage using ESP32 NVS (Preferences)
- Two-step calibration: Tare + Known weight
- Quick tare function (short-press ZERO)
- Full calibration (long-press ZERO)

## Configuration

Key constants in USER CONFIG section:

### Motion Parameters
- `STEP_PULSE_US`: Step pulse width (300µs default, lower = faster)
- `HOME_STEP_US`: Homing speed (600µs, slower for safety)
- `SEG_LOWER_IN`: Lowering distance (2.5")
- `SEG_MEASURE_IN`: Total measurement segment (3.0")
- `SEG_TRIM_IN`: Trim distance at start/end (0.25")

### Calibration
- `CAL_WEIGHT_LB`: Known calibration weight (adjust for your weight)
- `NORMAL_FORCE_LB`: Expected normal force during test (adjust for paddle weight)

## Known Issues & Future Improvements

### HIGH PRIORITY (Should be addressed before production use)

1. **Outdated Distance Comment**
   - Location: Line ~45-46
   - Issue: Comment says "6.0 inches total" but actual is 5.5" (SEG_NOISE_IN=0)
   - Impact: Could confuse during mechanical setup

2. **Trim Bounds Validation**
   - Location: Line ~450 (runTest)
   - Issue: No check if `SEG_TRIM_IN >= SEG_MEASURE_IN/2`
   - Impact: Could create zero or negative measurement window
   - Fix: Add validation in setup() or configuration section

3. **MAX_SAMPLES Overflow**
   - Location: Line ~410 (measureDuringMove)
   - Issue: Hardcoded 2000 sample limit, silently drops samples if exceeded
   - Impact: Could lose data on slow motion or long segments
   - Fix: Dynamic allocation based on segment length or warning when limit reached

4. **Memory Allocation Error Handling**
   - Location: Line ~372 (calculatePercentileAverage)
   - Issue: Returns 0.0 on malloc failure (indistinguishable from actual zero reading)
   - Impact: Silent failure could produce invalid test results
   - Fix: Return error code or use -1.0f as error indicator

5. **Hardcoded Display Text**
   - Location: Line ~697
   - Issue: "Assuming N = 3 lb" should use `NORMAL_FORCE_LB` dynamically
   - Impact: Misleading display when NORMAL_FORCE_LB is changed

### MEDIUM PRIORITY (Code quality improvements)

6. **Function Name Mismatch**
   - Location: Line ~268
   - Issue: `doCalibration3lb()` doesn't use 3lb anymore
   - Fix: Rename to `doCalibration()` or `doFullCalibration()`

7. **String Heap Fragmentation**
   - Location: Lines 285, 299, 312, 322
   - Issue: Arduino `String` class causes heap fragmentation on embedded systems
   - Impact: Could lead to crashes on long-running devices
   - Fix: Use `snprintf()` with char buffers instead

8. **millis() Overflow**
   - Location: Lines ~527, 548, debounce logic
   - Issue: After 49.7 days, `millis()` wraps to 0
   - Impact: Time difference calculations could fail
   - Fix: Use proper overflow-safe subtraction: `(millis() - startTime)`

9. **No I2C/OLED Error Checking**
   - Location: Line ~605 (setup)
   - Issue: `oled.begin()` return value ignored
   - Impact: Code continues even if display fails
   - Fix: Check return value and halt or flag error

10. **No Calibration Validation**
    - Location: Line ~246 (loadCalibration)
    - Issue: Loads values without sanity checking
    - Impact: Corrupt NVS could cause nonsensical measurements
    - Fix: Validate range (e.g., 100-10000 counts/lb is reasonable)

11. **NVS Flash Wear**
    - Location: Lines ~234-238 (saveCalibration)
    - Issue: Every tare writes to flash (~100K write cycle limit)
    - Impact: Could wear out flash over years of use
    - Fix: Implement write counter or only save on significant changes

### LOW PRIORITY (Nice to have)

12. **Magic Numbers Throughout**
    - Examples:
      - `100` (line ~309 - calibration delta threshold)
      - `600` (line ~474 - END_PAUSE_MS)
      - `50` (line ~587 - LED brightness)
      - `10` (line ~362 - percentile sample threshold)
    - Fix: Convert to named constants

13. **Global State Variables**
    - Location: Lines ~76-85, 321
    - Issue: Makes testing difficult, potential interrupt issues
    - Fix: Consider encapsulating in state struct

14. **Percentile Edge Case**
    - Location: Lines ~387-388
    - Issue: With exactly 10 samples, only 1 sample averaged
    - Fix: Require minimum 20+ samples for valid percentile

15. **Unused Variable**
    - Location: Line ~77
    - Issue: `g_lastAvgLb` is set but never displayed
    - Fix: Display on results screen or remove

16. **No Position Verification**
    - Issue: Assumes stepper never misses steps
    - Impact: Accumulated errors if steps are lost
    - Fix: Consider adding encoder feedback

17. **No Watchdog Timer**
    - Issue: Long operations could hang without recovery
    - Fix: Implement ESP32 watchdog for critical sections

18. **Inconsistent Memory Management**
    - Location: measureDuringMove function
    - Issue: Caller must remember to free allocated memory
    - Impact: Easy to leak memory on error paths
    - Fix: Consider RAII pattern or return struct by value

19. **Undocumented Sample Rate**
    - Issue: Unknown HX711 samples/second
    - Impact: Can't predict if 2000 samples is adequate
    - Fix: Document expected rates, add diagnostic output

## Recent Changes (v1.0)

- ✅ Fixed critical pin configuration comments
- ✅ Added 20-second timeout to homing (prevents infinite loop on limit switch failure)
- ✅ Added division-by-zero protection (g_calibration, NORMAL_FORCE_LB)
- ✅ Fixed calibration storage (now uses putLong/getLong for proper data preservation)
- ✅ Updated calibration to use debounced button reading (prevents false triggers)
- ✅ Implemented percentile-based averaging (85th-95th percentile)
- ✅ Added measurement trimming (0.25" start/end settling zones)
- ✅ Removed redundant noise settling segment
- ✅ Fixed calibration weight display (now dynamic based on CAL_WEIGHT_LB)

## Contributing

When addressing any of the known issues above, please:
1. Test thoroughly on actual hardware
2. Update this README with changes
3. Add comments explaining the fix
4. Consider backward compatibility with stored calibration data

## License

[Add your license here]

## Authors

[Add author information here]
