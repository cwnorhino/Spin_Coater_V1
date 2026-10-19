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
  FAULT_TACHO_LOST,     // commanding speed but no tach pulses (sensor lost / stall)
  FAULT_RAMP_CFG,       // degenerate profile (0 times etc.)
};

struct SafetyState {
  FaultCode fault = FAULT_NONE;
  bool estop = false;
  bool currentLimited = false;  // warning, not fault (auto-derated)
  bool overRpmTiming = false;
  uint32_t overRpmSince = 0;
  bool noTachoTiming = false;
  uint32_t spinNoTachoSince = 0;
  bool tachoPresent = false;    // latched once pulses observed
};

static inline const char* faultName(FaultCode f) {
  switch (f) {
    case FAULT_ESTOP: return "E-STOP";
    case FAULT_OVER_RPM_CMD: return "OVER RPM CMD";
    case FAULT_OVER_RPM_MEAS: return "OVER RPM MEAS";
    case FAULT_OVER_CURRENT: return "OVER CURRENT";
    case FAULT_TACHO_LOST: return "NO TACH/STALL";
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
  // capRpm: profile RPM cap (clamped to RPM_ABSOLUTE_MAX). Any latched fault -> 0.
  int update(int cmdRpm, int measRpm, uint32_t nowMs, int capRpm = RPM_SAFE_MAX) {
    if (s.estop) { s.fault = FAULT_ESTOP; return 0; }
    if (s.fault != FAULT_NONE) return 0;  // latched until clear()

    int rpm = cmdRpm;
    if (rpm < 0) rpm = 0;

    // Absolute ceiling — never exceed; logic asking for it is a hard fault.
    if (rpm > RPM_ABSOLUTE_MAX) { s.fault = FAULT_OVER_RPM_CMD; return 0; }

    // Profile cap (chuck rating) — silent clamp.
    if (capRpm > RPM_ABSOLUTE_MAX) capRpm = RPM_ABSOLUTE_MAX;
    if (rpm > capRpm) rpm = capRpm;

    // Estimated-current derate (software model, no sensor in base BOM).
    // Solve 0.8 + 14 f^2 = TRIP for max rpm, clamp there with warning.
    s.currentLimited = false;
    float iest = estCurrentA(rpm);
    if (iest > EST_CURRENT_TRIP_A) {
      float fmax = sqrtf((EST_CURRENT_TRIP_A - 0.8f) / 14.0f);
      if (fmax < 0) fmax = 0;
      if (fmax > 1) fmax = 1;
      int der = (int)(rpmAtFullThrottle() * fmax);
      if (der < rpm) rpm = der;
      s.currentLimited = true;
    }

    // Measured overspeed (tach only): > cap+margin sustained 1 s -> fault.
    if (measRpm >= 0) {
      if (measRpm > 0) s.tachoPresent = true;
      if (measRpm > capRpm + OVERSPEED_MARGIN_RPM) {
        if (!s.overRpmTiming) { s.overRpmTiming = true; s.overRpmSince = nowMs; }
        if (nowMs - s.overRpmSince > 1000) { s.fault = FAULT_OVER_RPM_MEAS; return 0; }
      } else {
        s.overRpmTiming = false;
      }
      // Tach fitted: commanding > TACHO_LOST_MIN_CMD_RPM with zero pulses for
      // 5 s means the sensor is lost/misaligned or the motor stalled.
      if (rpm > TACHO_LOST_MIN_CMD_RPM && measRpm == 0) {
        if (!s.noTachoTiming) { s.noTachoTiming = true; s.spinNoTachoSince = nowMs; }
        if (nowMs - s.spinNoTachoSince > 5000) { s.fault = FAULT_TACHO_LOST; return 0; }
      } else {
        s.noTachoTiming = false;
      }
    }
    return rpm;
  }
};
