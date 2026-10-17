#pragma once
#include <stdint.h>
#include <math.h>

// Pure-logic mirror of firmware ramp/safety math for native unit tests.
// Keep constants in sync with include/config.h and include/spin_math.h.

#define T_RPM_MIN           300
#define T_RPM_ABSOLUTE_MAX  10000
#define T_EST_TRIP_A        18.0f
#define T_MOTOR_KV          1400.0f
#define T_SUPPLY_V          12.0f
#define T_EFF               0.72f
#define T_SPIN_US           1090
#define T_SAFE_US           1850

static inline float t_rpmFull() { return T_MOTOR_KV * T_SUPPLY_V * T_EFF; }

static inline float t_ease01(float t, int ease) {
  if (t <= 0) return 0;
  if (t >= 1) return 1;
  switch (ease) {
    case 1: return t * t;
    case 2: return 1.0f - (1.0f - t) * (1.0f - t);
    case 3: return t * t * (3.0f - 2.0f * t);
    default: return t;
  }
}

static inline float t_rpmToThr01(int rpm) {
  float f = (float)rpm / t_rpmFull();
  if (f <= 0) return 0;
  if (f >= 1) return 1;
  return powf(f, 0.9f);
}

static inline int t_thrToUs(float thr) {
  if (thr <= 0.001f) return 1000;
  int us = T_SPIN_US + (int)((T_SAFE_US - T_SPIN_US) * thr);
  if (us < 1000) us = 1000;
  if (us > T_SAFE_US) us = T_SAFE_US;
  return us;
}

static inline int t_rpmToUs(int rpm) { return t_thrToUs(t_rpmToThr01(rpm)); }

static inline float t_estCurrentA(int rpm) {
  float f = (float)rpm / t_rpmFull();
  if (f < 0) f = 0; if (f > 1) f = 1;
  return 0.8f + 14.0f * f * f;
}

// Mirrors Safety::update clamp logic (fault flags omitted; clamp is what matters).
static inline int t_safetyClamp(int cmdRpm) {
  int rpm = cmdRpm < 0 ? 0 : cmdRpm;
  if (rpm > T_RPM_ABSOLUTE_MAX) rpm = T_RPM_ABSOLUTE_MAX;
  float iest = t_estCurrentA(rpm);
  if (iest > T_EST_TRIP_A) {
    float fmax = sqrtf((T_EST_TRIP_A - 0.8f) / 14.0f);
    if (fmax < 0) fmax = 0; if (fmax > 1) fmax = 1;
    int der = (int)(t_rpmFull() * fmax);
    if (der < rpm) rpm = der;
  }
  return rpm;
}

// Ramp target at elapsed ms, before safety clamp. Returns commanded RPM.
static inline int t_rampRpm(uint32_t elapsedMs, int target, int upS, int holdS, int dnS, int ease) {
  uint32_t up = (uint32_t)upS * 1000, hold = (uint32_t)holdS * 1000, dn = (uint32_t)dnS * 1000;
  uint32_t total = up + hold + dn;
  if (elapsedMs < up)    return (int)(target * t_ease01((float)elapsedMs / (float)up, ease));
  if (elapsedMs < up+hold) return target;
  if (elapsedMs < total) return (int)(target * (1.0f - t_ease01((float)(elapsedMs-up-hold)/(float)dn, ease)));
  return 0;
}
