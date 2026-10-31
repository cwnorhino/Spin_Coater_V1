#pragma once
#include "config.h"

// Closed-loop trim PI over the feedforward table (5 Hz, +/-TRIM_MAX_US).
// Freeze (hold last trim) when tach is stale/absent -> falls back to table only.
struct TrimPI {
  float integ = 0;
  int trim = 0;
  uint32_t lastMs = 0;
  void reset() { integ = 0; trim = 0; lastMs = 0; }
  // setRpm/cmd basis: error = target - measured. Returns bounded trim in us.
  int update(int targetRpm, int measRpm, bool tachOk, uint32_t nowMs, float kp = 0.03f, float ki = 0.008f) {
    if (!tachOk || targetRpm < 1) { lastMs = nowMs; return trim; }
    if (lastMs == 0) lastMs = nowMs;
    float dt = (nowMs - lastMs) / 1000.0f;
    lastMs = nowMs;
    if (dt <= 0 || dt > 1.0f) dt = TRIM_UPDATE_MS / 1000.0f;
    float err = (float)(targetRpm - measRpm);
    // Deadband: don't chase noise within ~2%.
    if (err < 0) err = -err < targetRpm * 0.02f ? 0 : err;
    else if (err < targetRpm * 0.02f) err = 0;
    integ += err * dt;
    float maxI = TRIM_MAX_US / (ki > 0 ? ki : 1);
    if (integ > maxI) integ = maxI;
    if (integ < -maxI) integ = -maxI;
    float out = kp * err + ki * integ;
    if (out > TRIM_MAX_US) { out = TRIM_MAX_US; }
    if (out < -TRIM_MAX_US) { out = -TRIM_MAX_US; }
    trim = (int)out;
    return trim;
  }
};
