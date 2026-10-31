#pragma once
#include "config.h"
#include "spin_math.h"
#ifdef ARDUINO
#include <Preferences.h>
#endif

// Dense us->rpm table from the ~60s auto sweep (Option A).
// rpmToUsCal(): piecewise-linear interp, clamps outside range.
// Invalid table -> caller falls back to KV-model rpmToUs() + UNCAL badge.
struct RpmCal {
  bool valid = false;
  uint8_t n = 0;
  int us[CAL_POINTS] = {0};
  int rpm[CAL_POINTS] = {0};
};

struct FbCal {
  bool hasMin = false;
  int usMin = FB_FALLBACK_US;     // spin-start throttle
  int rpmMin = FB_FALLBACK_RPM;   // measured rpm at spin-start
  int cap = RPM_SAFE_MAX;         // run cap (default 8000, editable, <= ABSOLUTE)
  int fbMinRpm = FB_FALLBACK_RPM; // manual fallback floor
  int fbMaxRpm = RPM_SAFE_MAX;    // manual fallback cap
};

static inline int rpmToUsCal(int rpm, const RpmCal &c) {
  if (!c.valid || c.n < 2) return rpmToUs(rpm);
  if (rpm <= c.rpm[0]) return c.us[0];
  for (uint8_t i = 1; i < c.n; i++) {
    if (rpm <= c.rpm[i]) {
      int r0 = c.rpm[i - 1], r1 = c.rpm[i];
      int u0 = c.us[i - 1], u1 = c.us[i];
      if (r1 <= r0) return u1;
      float f = (float)(rpm - r0) / (float)(r1 - r0);
      return u0 + (int)(f * (u1 - u0));
    }
  }
  return c.us[c.n - 1];
}

static inline int usToRpmCal(int us, const RpmCal &c) {
  if (!c.valid || c.n < 2) return usToRpmEst(us);
  if (us <= c.us[0]) return c.rpm[0];
  for (uint8_t i = 1; i < c.n; i++) {
    if (us <= c.us[i]) {
      int u0 = c.us[i - 1], u1 = c.us[i];
      int r0 = c.rpm[i - 1], r1 = c.rpm[i];
      if (u1 <= u0) return r1;
      float f = (float)(us - u0) / (float)(u1 - u0);
      return r0 + (int)(f * (r1 - r0));
    }
  }
  return c.rpm[c.n - 1];
}

#ifdef ARDUINO
static inline void calibLoad(RpmCal &c, FbCal &f) {
  Preferences p;
  if (!p.begin("cal", true)) return;
  c.valid = p.getBool("valid", false);
  c.n = p.getUChar("n", 0);
  if (c.n > CAL_POINTS) c.n = 0;
  for (uint8_t i = 0; i < c.n; i++) {
    c.us[i] = p.getInt(("u" + String(i)).c_str(), 0);
    c.rpm[i] = p.getInt(("r" + String(i)).c_str(), 0);
  }
  // Range-check loaded table: strictly increasing, sane bounds.
  bool ok = c.valid && c.n >= 2;
  for (uint8_t i = 0; ok && i < c.n; i++)
    ok = c.us[i] >= ESC_MIN_US && c.us[i] <= ESC_SAFE_US && c.rpm[i] >= 0 && c.rpm[i] <= RPM_ABSOLUTE_MAX;
  for (uint8_t i = 1; ok && i < c.n; i++) ok = c.us[i] > c.us[i - 1] && c.rpm[i] >= c.rpm[i - 1];
  if (!ok) { c.valid = false; c.n = 0; }
  f.hasMin = p.getBool("hmin", false);
  f.usMin = p.getInt("umin", FB_FALLBACK_US);
  f.rpmMin = p.getInt("rmin", FB_FALLBACK_RPM);
  f.cap = p.getInt("cap", RPM_SAFE_MAX);
  f.fbMinRpm = p.getInt("fmin", FB_FALLBACK_RPM);
  f.fbMaxRpm = p.getInt("fmax", RPM_SAFE_MAX);
  if (f.cap > RPM_ABSOLUTE_MAX) f.cap = RPM_ABSOLUTE_MAX;
  if (f.cap < RPM_MIN) f.cap = RPM_SAFE_MAX;
  p.end();
}

static inline void calibSaveTable(const RpmCal &c) {
  Preferences p;
  if (!p.begin("cal", false)) return;
  p.putBool("valid", c.valid);
  p.putUChar("n", c.n);
  for (uint8_t i = 0; i < c.n; i++) {
    p.putInt(("u" + String(i)).c_str(), c.us[i]);
    p.putInt(("r" + String(i)).c_str(), c.rpm[i]);
  }
  p.end();
}

static inline void calibSaveFb(const FbCal &f) {
  Preferences p;
  if (!p.begin("cal", false)) return;
  p.putBool("hmin", f.hasMin);
  p.putInt("umin", f.usMin);
  p.putInt("rmin", f.rpmMin);
  p.putInt("cap", f.cap);
  p.putInt("fmin", f.fbMinRpm);
  p.putInt("fmax", f.fbMaxRpm);
  p.end();
}
#endif
