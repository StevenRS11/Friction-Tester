#include "CofCalculation.h"
#include <stdlib.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static int compareFloats(const void* a, const void* b) {
  float fa = *(const float*)a;
  float fb = *(const float*)b;
  if (fa < fb) return -1;
  if (fa > fb) return  1;
  return 0;
}

// Compute trim offsets and pair count from raw counts + trim fraction.
// Writes back through out-pointers; returns false if no valid pairs remain.
static bool computeTrimParams(long fwdCount, long revCount,
                              float trimFraction,
                              long* fwdStart, long* revStart,
                              long* pairedCount) {
  long fwdTrim = (long)(fwdCount * trimFraction);
  long revTrim = (long)(revCount * trimFraction);

  long fwdTrimmed = fwdCount - 2 * fwdTrim;
  long revTrimmed = revCount - 2 * revTrim;

  if (fwdTrimmed <= 0 || revTrimmed <= 0) return false;

  *fwdStart    = fwdTrim;
  *revStart    = revTrim;
  *pairedCount = min(fwdTrimmed, revTrimmed);
  return true;
}

// ---------------------------------------------------------------------------
// Built-in averaging strategies
// ---------------------------------------------------------------------------

double avgPercentileBand(const float* samples, long count) {
  if (count < 10) {
    double sum = 0.0;
    for (long i = 0; i < count; i++) sum += samples[i];
    return (count > 0) ? (sum / (double)count) : 0.0;
  }

  // Need a mutable copy for sorting
  float* sorted = (float*)malloc(count * sizeof(float));
  if (!sorted) {
    Serial.println("ERROR: avgPercentileBand malloc failed");
    return 0.0;
  }
  memcpy(sorted, samples, count * sizeof(float));
  qsort(sorted, count, sizeof(float), compareFloats);

  long idx85 = (long)(count * 0.85);
  long idx95 = (long)(count * 0.95);
  if (idx85 >= idx95) idx85 = idx95 - 1;
  if (idx85 < 0) idx85 = 0;

  double sum = 0.0;
  long rangeCount = idx95 - idx85;
  for (long i = idx85; i < idx95; i++) {
    sum += sorted[i];
  }

  free(sorted);
  return (rangeCount > 0) ? (sum / (double)rangeCount) : 0.0;
}

double avgWithinOneStdDev(const float* samples, long count) {
  if (count <= 0) return 0.0;

  // Pass 1: mean
  double sum = 0.0;
  for (long i = 0; i < count; i++) sum += samples[i];
  double mean = sum / (double)count;

  // Pass 2: standard deviation
  double sqSum = 0.0;
  for (long i = 0; i < count; i++) {
    double diff = samples[i] - mean;
    sqSum += diff * diff;
  }
  double stddev = sqrt(sqSum / (double)count);

  // Pass 3: average values within 1σ of mean
  double lo = mean - stddev;
  double hi = mean + stddev;
  double filteredSum = 0.0;
  long   filteredCount = 0;

  for (long i = 0; i < count; i++) {
    if (samples[i] >= lo && samples[i] <= hi) {
      filteredSum += samples[i];
      filteredCount++;
    }
  }

  return (filteredCount > 0) ? (filteredSum / (double)filteredCount) : mean;
}

// ---------------------------------------------------------------------------
// Core COF calculation — paired midpoint method
// ---------------------------------------------------------------------------

CofResult calculateCOF(const float* fwdSamples, long fwdCount,
                        const float* revSamples, long revCount,
                        float normalForceLb,
                        float trimFraction,
                        AveragingFn avgFn) {

  CofResult result = { 0.0f, 0.0f, 0.0f, 0 };

  // --- Trim & pair count ---------------------------------------------------
  long fwdStart = 0, revStart = 0, pairCount = 0;
  if (!computeTrimParams(fwdCount, revCount, trimFraction,
                         &fwdStart, &revStart, &pairCount)) {
    Serial.println("ERROR: No valid pairs after trimming");
    return result;
  }

  // --- Build paired friction array -----------------------------------------
  float* pairedFriction = (float*)malloc(pairCount * sizeof(float));
  if (!pairedFriction) {
    Serial.println("ERROR: calculateCOF malloc failed");
    return result;
  }

  double biasSum = 0.0;

  for (long i = 0; i < pairCount; i++) {
    float fwd = fwdSamples[fwdStart + i];
    float rev = revSamples[revStart + (pairCount - 1 - i)]; // flip for position

    pairedFriction[i] = fabsf(fwd - rev) / 2.0f;
    biasSum += (fwd + rev) / 2.0;
  }

  // --- Apply averaging strategy --------------------------------------------
  double avgForce = avgFn(pairedFriction, pairCount);
  free(pairedFriction);

  // --- Assemble result -----------------------------------------------------
  result.avgForceLb  = (float)avgForce;
  result.avgBias     = (float)(biasSum / (double)pairCount);
  result.pairedCount = pairCount;
  result.cof = (normalForceLb > 0.0f) ? (float)(avgForce / normalForceLb)
                                       : 0.0f;
  return result;
}

// ---------------------------------------------------------------------------
// Diagnostic paired-data CSV dump
// ---------------------------------------------------------------------------

void dumpPairedDataCSV(const float* fwdSamples, long fwdCount,
                       const float* revSamples, long revCount,
                       float trimFraction) {

  long fwdStart = 0, revStart = 0, pairCount = 0;
  if (!computeTrimParams(fwdCount, revCount, trimFraction,
                         &fwdStart, &revStart, &pairCount)) {
    Serial.println("---PAIRED_CSV_START---");
    Serial.println("ERROR: no valid pairs");
    Serial.println("---PAIRED_CSV_END---");
    return;
  }

  Serial.println("---PAIRED_CSV_START---");
  Serial.println("pos_index,fwd_force,rev_force,friction,bias");

  for (long i = 0; i < pairCount; i++) {
    float fwd = fwdSamples[fwdStart + i];
    float rev = revSamples[revStart + (pairCount - 1 - i)];
    float friction = fabsf(fwd - rev) / 2.0f;
    float bias     = (fwd + rev) / 2.0f;

    Serial.print(i);
    Serial.print(',');
    Serial.print(fwd, 4);
    Serial.print(',');
    Serial.print(rev, 4);
    Serial.print(',');
    Serial.print(friction, 4);
    Serial.print(',');
    Serial.println(bias, 4);
  }

  Serial.println("---PAIRED_CSV_END---");
}
