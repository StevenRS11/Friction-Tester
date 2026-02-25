#ifndef COF_CALCULATION_H
#define COF_CALCULATION_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Pluggable averaging strategy
// ---------------------------------------------------------------------------
// Any function matching this signature can be used as an averaging method.
// Input: array of positive friction values and count.
// Output: a single representative average value.
typedef double (*AveragingFn)(const float* samples, long count);

// ---------------------------------------------------------------------------
// Result of a COF calculation
// ---------------------------------------------------------------------------
struct CofResult {
  float cof;          // final coefficient of friction
  float avgForceLb;   // average friction force (lb) after averaging strategy
  float avgBias;      // mean positional bias (lb) — diagnostic
  long  pairedCount;  // number of position-matched pairs used
};

// ---------------------------------------------------------------------------
// Main entry point
// ---------------------------------------------------------------------------
// Pairs forward/reverse samples by physical position, computes per-pair
// friction via the midpoint method, then applies the given averaging strategy.
//
//   trimFraction — fraction of each pass to discard at start/end
//                  (e.g. 0.25/3.0 ≈ 0.0833 for current geometry)
//   avgFn        — averaging strategy to apply to paired friction values
//
CofResult calculateCOF(const float* fwdSamples, long fwdCount,
                        const float* revSamples, long revCount,
                        float normalForceLb,
                        float trimFraction,
                        AveragingFn avgFn);

// ---------------------------------------------------------------------------
// Built-in averaging strategies
// ---------------------------------------------------------------------------

// Sort values, average the 85th–95th percentile window (discard top 5%).
double avgPercentileBand(const float* samples, long count);

// Average only values within one standard deviation of the mean.
double avgWithinOneStdDev(const float* samples, long count);

// ---------------------------------------------------------------------------
// Diagnostic CSV dump
// ---------------------------------------------------------------------------
// Prints paired data to Serial:
//   pos_index, fwd_force, rev_force, friction, bias
// Recomputes pairs on-the-fly (no extra memory beyond stack).
void dumpPairedDataCSV(const float* fwdSamples, long fwdCount,
                       const float* revSamples, long revCount,
                       float trimFraction);

#endif // COF_CALCULATION_H
