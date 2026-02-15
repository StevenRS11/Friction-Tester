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

### HIGH PRIORITY — Measurement Accuracy

1. **~~Sample-Weighted Bidirectional Average~~ (FIXED)**
   - Location: `runTest()`, COF calculation
   - Issue: Used `(|fwd| × N_fwd + |rev| × N_rev) / (N_fwd + N_rev)` instead of equal-weight `(|fwd| + |rev|) / 2`. If forward and reverse sample counts differ, one direction dominates, allowing directional bias (cable drag, surface grain) to leak through — defeating the purpose of bidirectional testing.
   - Fix: Changed to equal-weighted average: `COF = (fwdAvg + revAvg) / (2 × NORMAL_FORCE_LB)`

2. **85th–95th Percentile Selection Overestimates Friction**
   - Location: `calculatePercentileAverage()` (~line 687)
   - Issue: Sorts samples by absolute value and averages only the 85th–95th percentile band. This systematically selects the high end of the force distribution, overestimating steady-state kinetic friction. A median or broader central percentile (e.g., 25th–75th) would be more representative of steady-state sliding force.
   - Impact: COF values are biased upward relative to true kinetic friction
   - Fix: Consider switching to median or IQR-based average

3. **No Auto-Tare Before Test**
   - Location: `runTest()` (~line 772)
   - Issue: The test never calls `tareNow()`. The tare offset (`g_tareRaw`) is whatever was last manually set or loaded from NVS. Load cell drift (thermal, mechanical) between the last tare and the test maps directly into measured friction force. Since normal force is a hardcoded constant (not measured), drift does not cancel out.
   - Impact: Thermal drift directly biases COF
   - Fix: Add `tareNow()` call at the start of `runTest()`, before homing

4. **Post-Hoc Trim Assumes Uniform Sample Spacing**
   - Location: `runTest()` (~lines 864–889)
   - Issue: In the dual-core path, `forceSamplingTask()` collects samples over the entire measurement move. Trimming is done afterward by index: `trimSamplesPerSide = (sampleCount × steps_trim) / steps_measure`. This assumes samples are evenly distributed in time. If HX711 data-ready rate fluctuates, the index-based trim won't correspond exactly to the physical 0.25" trim regions.
   - Impact: Moderate — trim boundaries may be spatially inaccurate
   - Fix: Timestamp each sample and trim by elapsed time, or accept the approximation

5. **No Validation Both Passes Have Sufficient Data**
   - Location: `runTest()` (~lines 868–905)
   - Issue: If one direction yields 0 samples (e.g., HX711 hangs), the code still computes a result. With equal weighting this produces `(valid + 0) / 2`, halving the real COF. With very few samples (<10), the percentile filter falls back to a simple average with no outlier rejection.
   - Impact: Silent bad result on hardware fault
   - Fix: Require minimum sample count per pass (e.g., 20); abort and warn if not met

6. **Outdated Distance Comment**
   - Location: Line ~45-46
   - Issue: Comment says "6.0 inches total" but actual is 5.5" (SEG_NOISE_IN=0)
   - Impact: Could confuse during mechanical setup

7. **Trim Bounds Validation**
   - Location: `runTest()`
   - Issue: No check if `SEG_TRIM_IN >= SEG_MEASURE_IN/2`
   - Impact: Could create zero or negative measurement window
   - Fix: Add validation in setup() or configuration section

8. **MAX_SAMPLES Overflow**
   - Location: `measureDuringMove()` and `forceSamplingTask()`
   - Issue: Hardcoded 2000 sample limit, silently drops samples if exceeded
   - Impact: Could lose data on slow motion or long segments
   - Fix: Dynamic allocation based on segment length or warning when limit reached

9. **Memory Allocation Error Handling**
   - Location: `calculatePercentileAverage()`
   - Issue: Returns 0.0 on malloc failure (indistinguishable from actual zero reading)
   - Impact: Silent failure could produce invalid test results
   - Fix: Return error code or use -1.0f as error indicator

10. **~10ms Sampling Start Latency**
    - Location: `forceSamplingTask()` (~line 579)
    - Issue: When idle, the sampling task sleeps for 10ms. When Core 1 begins a measurement move, up to 10ms of initial motion goes unsampled (~16 steps / 0.0016"). This falls within the trim region but makes the effective trim slightly asymmetric (longer at start than end).
    - Impact: Low — within trim margin
    - Fix: Reduce idle delay or use a semaphore to wake the sampling task immediately

### MEDIUM PRIORITY — Code Quality

11. **Dead Code: `measureDuringMove()` Function**
    - Location: ~line 732
    - Issue: The old single-core measurement function is never called from `runTest()` after the dual-core refactor. It allocates its own sample buffer and does inline trimming — completely separate from the dual-core path.
    - Impact: Confusing; someone could accidentally call the wrong path
    - Fix: Remove or clearly mark as deprecated

12. **No Tare Between Forward and Reverse Passes**
    - Location: `runTest()` (~line 820)
    - Issue: The 600ms pause between passes does not re-tare. If the load cell baseline drifts during the ~18-second forward pass, the reverse pass inherits that drift. A quick baseline reading during the pause could compensate.
    - Impact: Moderate on sensitive load cells
    - Fix: Read baseline during the 600ms pause and subtract from reverse samples

13. **No Outlier Rejection in Tare**
    - Location: `hxReadRawAvg()` (~line 342)
    - Issue: All 20 tare samples are averaged equally. An electrical spike during tare biases the offset.
    - Fix: Use median or trimmed-mean instead of simple average

14. **Function Name Mismatch**
    - Location: `doCalibration3lb()`
    - Issue: Doesn't use 3lb anymore — uses `CAL_WEIGHT_LB`
    - Fix: Rename to `doCalibration()` or `doFullCalibration()`

15. **String Heap Fragmentation**
    - Location: Various display and calibration functions
    - Issue: Arduino `String` class causes heap fragmentation on embedded systems
    - Impact: Could lead to crashes on long-running devices
    - Fix: Use `snprintf()` with char buffers instead

16. **millis() Overflow**
    - Location: Debounce logic, homing timeouts
    - Issue: After 49.7 days, `millis()` wraps to 0
    - Impact: Time difference calculations could fail
    - Fix: Use proper overflow-safe subtraction: `(millis() - startTime)`

17. **No I2C/OLED Error Checking**
    - Location: `setup()`
    - Issue: `oled.begin()` return value ignored
    - Impact: Code continues even if display fails
    - Fix: Check return value and halt or flag error

18. **No Calibration Validation**
    - Location: `loadCalibration()`
    - Issue: Loads values without sanity checking
    - Impact: Corrupt NVS could cause nonsensical measurements
    - Fix: Validate range (e.g., 100-10000 counts/lb is reasonable)

19. **NVS Flash Wear**
    - Location: `saveCalibration()`
    - Issue: Every tare writes to flash (~100K write cycle limit)
    - Impact: Could wear out flash over years of use
    - Fix: Implement write counter or only save on significant changes

### LOW PRIORITY — Nice to Have

20. **Magic Numbers Throughout**
    - Examples: `100` (calibration delta threshold), `600` (pause between passes), `50` (LED brightness), `10` (percentile sample threshold)
    - Fix: Convert to named constants

21. **Global State Variables**
    - Issue: Makes testing difficult, potential interrupt issues
    - Fix: Consider encapsulating in state struct

22. **Percentile Edge Case**
    - Issue: With exactly 10 samples, only 1 sample averaged in the 85th–95th band
    - Fix: Require minimum 20+ samples for valid percentile

23. **Unused Variable**
    - Issue: `g_lastAvgLb` is set but never displayed
    - Fix: Display on results screen or remove

24. **No Position Verification**
    - Issue: Assumes stepper never misses steps
    - Fix: Consider adding encoder feedback

25. **No Watchdog Timer**
    - Issue: Long operations could hang without recovery
    - Fix: Implement ESP32 watchdog for critical sections

26. **Inconsistent Memory Management**
    - Location: `measureDuringMove()` (dead code)
    - Issue: Caller must remember to free allocated memory
    - Fix: Remove dead code, or use RAII pattern

27. **Undocumented Sample Rate**
    - Issue: Unknown HX711 samples/second in actual operation
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
