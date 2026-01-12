# Friction Test Procedure - Physical Operation

This document explains the exact physical test procedure, including motion sequences, force sensing methodology, and data processing.

## Physical Motion Sequence

### 1. Homing (`homeToLimit()` - Friction-Tester.ino:342-386)
- Stepper moves the test paddle **upward** toward the limit switch at the top
- When limit is hit, backs off 600 steps, approaches slowly again, then backs off 300 steps
- This establishes a consistent "home" starting position

### 2. Lowering Phase (2.5 inches)
- Paddle descends 2.5" onto the testing surface
- **No force data is collected during this phase** - it's purely positioning
- LED shows yellow during lowering
- This gets the paddle into contact with the surface

### 3. Forward Measurement Pass (3.0 inches total)
- Paddle continues moving **forward** (down) for 3.0 inches
- **BUT** only the middle 2.5" is actually measured:
  - First 0.25": Motion happens but **no sampling** (settling/trim zone)
  - Middle 2.5": **Active force measurement window**
  - Last 0.25": Motion happens but **no sampling** (trim zone)
- LED shows cyan during forward measurement

### 4. Pause (600ms)
- Brief pause at the bottom of travel

### 5. Reverse Measurement Pass (3.0 inches)
- Paddle reverses direction, moving **backward** (up) for 3.0 inches
- Same trimming logic: only middle 2.5" is measured
- LED shows magenta during reverse measurement

### 6. Return (2.5 inches)
- Paddle lifts back up through the lowering distance
- No data collection

### 7. Re-home
- Returns to limit switch for next test

## How Force Sensing Works During Motion

The system uses non-blocking HX711 reading during motion (`measureDuringMove()` - Friction-Tester.ino:449):

```cpp
for (long i = 0; i < steps; i++) {
    doStepBlocking(pulseUs);  // Take one stepper step (tight timing)

    // Check if we're in the measurement window (skip first/last 0.25")
    bool inMeasurementWindow = (i >= trimSteps) && (i < (steps - trimSteps));

    // Non-blocking HX711 read
    if (inMeasurementWindow && scale.is_ready() && nF < MAX_SAMPLES) {
        long raw = scale.read();      // Read load cell
        float lbs = rawToPounds(raw); // Convert to pounds
        samples[nF] = lbs;            // Store sample
        nF++;
    }
}
```

### Key Aspects:
- The stepper moves **continuously at constant speed** (~1667 steps/sec)
- The HX711 load cell is checked **opportunistically** using `scale.is_ready()`
- When data is available, it grabs a force sample **without blocking the motion**
- Typically collects ~1000 samples per pass during the 2.5" measurement window
- Tight stepper timing is maintained for smooth, consistent motion

## Data Processing

After both passes complete (`runTest()` - Friction-Tester.ino:537-567):

### 1. Percentile Filtering
Each pass's samples are sorted, and only the **85th-95th percentile** is averaged (`calculatePercentileAverage()` - Friction-Tester.ino:404-446):
- Discards the top 5% (likely noise spikes/vibration)
- Takes the next 10% tier (stable friction forces)
- This robust statistical approach eliminates outliers

### 2. Bidirectional Averaging
Forward and reverse averages are combined using **sample-weighted averaging**:

```
weighted_avg = (|fwd.force| × fwd.samples + |rev.force| × rev.samples) / total_samples
```

This accounts for any variance in sample count between passes.

### 3. COF Calculation
```
COF = avg_friction_force / NORMAL_FORCE_LB
```

Where `NORMAL_FORCE_LB = 3.59 lb` (configured on line 70) represents the weight pressing down on the surface.

## Physical Geometry Summary

| Parameter | Value | Purpose |
|-----------|-------|---------|
| Total travel | ~5.5 inches vertical | Full motion range |
| Measurement zones | Two passes of 2.5" each | Bidirectional friction data |
| Trim zones | 0.25" at entry/exit of each pass | Force stabilization |
| Speed | ~300µs per microstep | Constant friction measurement |
| Steps per inch | ~10,133 steps/inch | Motion resolution |

## Motion Diagram

```
[LIMIT SWITCH] ← Home position
      ↓ (2.5" lowering - no data)
      ↓
[START TRIM 0.25"] ← First contact with surface
      ↓ (2.5" FORWARD MEASUREMENT)
[END TRIM 0.25"]
      ↓
[BOTTOM] ← Pause 600ms
      ↑
[START TRIM 0.25"]
      ↑ (2.5" REVERSE MEASUREMENT)
[END TRIM 0.25"]
      ↑ (2.5" return - no data)
[LIMIT SWITCH] ← Home for next test
```

## Summary

The test essentially drags the paddle across the surface twice (down and back up) through the middle portion of travel, continuously sampling friction force while maintaining steady motion speed. The bidirectional measurement averages out any directional bias, and the percentile filtering ensures only stable friction values are used in the final COF calculation.
