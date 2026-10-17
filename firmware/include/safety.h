#pragma once
#include "spin_math.h"

// Central safety interlock: every commanded RPM passes through here.
// Returns clamped RPM. Sets faultCode if a hard fault occurred.
enum FaultCode : uint8_t {
  FAULT_NONE = 0,
  FAULT_ESTOP,
  FAULT_OVER_RPM_CMD,   // UI/logic asked above absolute max
  FAULT_OVER_RPM_MEAS,  // tachometer measured overspeed
  FAULT_OVER_CURRENT,   // estimated (or sensed) current trip
  FAULT_TACHO_LOST,     // spinning fast but no tach pulses
  FAULT_RAMP_CFG,       // degenerate profile (0 times etc.)
};

struct SafetyState {
  FaultCode fault = FAULT_NONE;
  bool estop = false;
  bool currentLimited = false;  // warning, not fault (auto-derated)
  uint32_t overRpmSince = 0;
  uint32_t spinNoTachoSince = 0;
  bool tachoPresent = false;    // latched once pulses observed
};

static inline const char* faultName(FaultCode f) {
  switch (f) {
    case FAULT_ESTOP: return "E-STOP";
    case FAULT_OVER_RPM_CMD: return "OVER RPM CMD";
    case FAULT_OVER_RPM_MEAS: return "OVER RPM MEAS";
    case FAULT_OVER_CURRENT: return "OVER CURRENT";
    case FAULT_TACHO_LOST: return "TACHO LOST";
    case FAULT_RAMP_CFG: return "BAD PROFILE";
    default: return "OK";
  }
}

class Safety {
 public:
  SafetyState s;

  void reset() { s = SafetyState(); }
  void pressEstop() { s.estop = true; s.fault = FAULT_ESTOP; }
  void clear() { s = SafetyState(); }
  bool hasFault() const { return s.fault != FAULT_NONE; }

  // Clamp + derate. measRpm < 0 means "no tachometer".
  int update(int cmdRpm, int measRpm, uint32_t nowMs) {
    if (s.estop) { s.fault = FAULT_ESTOP; return 0; }

    int rpm = cmdRpm;
    if (rpm < 0) rpm = 0;

    // Absolute ceiling — never exceed, and flag if logic tried to.
    if (rpm > RPM_ABSOLUTE_MAX) { rpm = RPM_ABSOLUTE_MAX; s.fault = FAULT_OVER_RPM_CMD; }

    // Estimated-current derate (software model, no sensor in base BOM).
    // Solve 0.8 + 14 f^2 = TRIP for max rpm, clamp there with warning.
    s.currentLimited = false;
    float iest = estCurrentA(rpm);
    if (iest > EST_CURRENT_TRIP_A) {
      float fmax = sqrtf((EST_CURRENT_TRIP_A - 0.8f) / 14.0f);
      if (fmax < 0) fmax = 0; if (fmax > 1) fmax = 1;
      rpm = (int)(rpmAtFullThrottle() * fmax);
      s.currentLimited = true;
    }

    // Measured overspeed (tach only): > cap+500 sustained 1 s -> fault.
    if (measRpm >= 0) {
      if (measRpm > 0) s.tachoPresent = true;
      if (measRpm > RPM_ABSOLUTE_MAX + 300) {
        if (s.overRpmSince == 0) s.overRpmSince = nowMs;
        if (nowMs - s.overRpmSince > 1000) { s.fault = FAULT_OVER_RPM_MEAS; return 0; }
      } else {
        s.overRpmSince = 0;
      }
      // Tacho-loss: commanding >1500 with zero pulses for 5 s after spin start.
      if (rpm > 1500 && measRpm == 0 && s.tachoPresent) {
        if (s.spinNoTachoSince == 0) s.spinNoTachoSince = nowMs;
        if (nowMs - s.spinNoTachoSince > 5000) { s.fault = FAULT_TACHO_LOST; return 0; }
      } else {
        s.spinNoTachoSince = 0;
      }
    }
    return rpm;
  }
};
