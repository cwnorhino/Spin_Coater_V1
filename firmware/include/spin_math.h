#pragma once
#include "config.h"
#include <math.h>

// Ease curves, t in [0,1] -> [0,1]
static inline float ease01(float t, uint8_t ease) {
  if (t <= 0) return 0;
  if (t >= 1) return 1;
  switch (ease) {
    case 1: return t * t;                        // Ease-In (quadratic)
    case 2: return 1.0f - (1.0f - t) * (1.0f - t); // Ease-Out
    case 3: {                                    // S-Curve (smoothstep)
      return t * t * (3.0f - 2.0f * t);
    }
    default: return t;                           // Linear
  }
}

static inline const char* easeName(uint8_t e) {
  switch (e) {
    case 1: return "EaseIn";
    case 2: return "EaseOut";
    case 3: return "S-Curve";
    default: return "Linear";
  }
}

struct SpinProfile {
  int targetRpm = DEF_TARGET_RPM;
  int rampUpS   = DEF_RAMP_UP_S;
  int holdS     = DEF_HOLD_S;
  int rampDnS   = DEF_RAMP_DN_S;
  uint8_t ease  = DEF_EASE;
  int maxRpmCap = RPM_SAFE_MAX;
};

// Profile phases for UI / telemetry
enum SpinPhase : uint8_t { PHASE_UP, PHASE_HOLD, PHASE_DOWN, PHASE_DONE };

static inline uint32_t profileTotalMs(const SpinProfile &p) {
  return ((uint32_t)p.rampUpS + (uint32_t)p.holdS + (uint32_t)p.rampDnS) * 1000UL;
}

static inline SpinPhase profilePhase(uint32_t elapsedMs, const SpinProfile &p) {
  uint32_t upMs   = (uint32_t)p.rampUpS * 1000UL;
  uint32_t holdMs = (uint32_t)p.holdS   * 1000UL;
  if (elapsedMs < upMs) return PHASE_UP;
  if (elapsedMs < upMs + holdMs) return PHASE_HOLD;
  if (elapsedMs < profileTotalMs(p)) return PHASE_DOWN;
  return PHASE_DONE;
}

// Commanded RPM (before safety clamp) at elapsed time; 0 once the profile is done.
static inline int rampRpm(uint32_t elapsedMs, const SpinProfile &p) {
  uint32_t upMs   = (uint32_t)p.rampUpS * 1000UL;
  uint32_t holdMs = (uint32_t)p.holdS   * 1000UL;
  uint32_t dnMs   = (uint32_t)p.rampDnS * 1000UL;
  switch (profilePhase(elapsedMs, p)) {
    case PHASE_UP:   // upMs > 0 here
      return (int)(p.targetRpm * ease01((float)elapsedMs / (float)upMs, p.ease));
    case PHASE_HOLD:
      return p.targetRpm;
    case PHASE_DOWN: { // dnMs > 0 here
      float d = (float)(elapsedMs - upMs - holdMs) / (float)dnMs;
      return (int)(p.targetRpm * (1.0f - ease01(d, p.ease)));
    }
    default:
      return 0;
  }
}

// ---- Throttle mapping (open-loop estimate) ----
// We map RPM -> throttle fraction with a deadband-aware curve, then -> us.
// thr = (rpm / rpmFull) ^ 0.9  (slight boost low-end to overcome deadband)
static inline float rpmToThr01(int rpm) {
  float full = rpmAtFullThrottle();
  float f = (float)rpm / full;
  if (f <= 0) return 0;
  if (f >= 1) return 1;
  return powf(f, 0.9f);
}

static inline int thrToUs(float thr) {
  if (thr <= 0.001f) return ESC_MIN_US;
  // Reserve deadband: thr maps from SPIN point to SAFE max
  int us = ESC_SPIN_US + (int)((ESC_SAFE_US - ESC_SPIN_US) * thr);
  if (us < ESC_MIN_US) us = ESC_MIN_US;
  if (us > ESC_SAFE_US) us = ESC_SAFE_US;
  return us;
}

static inline int rpmToUs(int rpm) { return thrToUs(rpmToThr01(rpm)); }

static inline int usToRpmEst(int us) {
  if (us <= ESC_SPIN_US) return 0;
  float thr = (float)(us - ESC_SPIN_US) / (float)(ESC_SAFE_US - ESC_SPIN_US);
  if (thr < 0) thr = 0;
  if (thr > 1) thr = 1;
  // invert powf taper: rpm = full * thr^(1/0.9)
  return (int)(rpmAtFullThrottle() * powf(thr, 1.0f / 0.9f));
}

// Tach pulse period (us between pulses) -> RPM.
static inline int tachPeriodToRpm(uint32_t periodUs, uint8_t pulsesPerRev) {
  if (periodUs == 0 || pulsesPerRev == 0) return 0;
  return (int)(60000000ULL / ((uint64_t)periodUs * pulsesPerRev));
}

// Estimated phase current (no sensor in base BOM — software model only).
// A2212 + light chuck: idle ~0.8A, full ~12-15A. Quadratic fan-like load.
// NOTE: max of this model is 14.8A (< EST_CURRENT_TRIP_A), so the derate in
// Safety never engages with the current constants. It is not a current sensor.
static inline float estCurrentA(int rpm) {
  float full = rpmAtFullThrottle();
  float f = (float)rpm / full;
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  return 0.8f + 14.0f * f * f;
}
