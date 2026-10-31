#include <unity.h>
// Tests the real firmware headers (config.h builds without Arduino.h natively).
#include "spin_math.h"
#include "safety.h"
#include "calib.h"
#include "feedback.h"

void setUp(void) {}
void tearDown(void) {}

static SpinProfile makeProfile(int rpm, int up, int hold, int dn, uint8_t ease) {
  SpinProfile p;
  p.targetRpm = rpm; p.rampUpS = up; p.holdS = hold; p.rampDnS = dn; p.ease = ease;
  return p;
}

// Ease curves stay in [0,1] and are monotonic
static void test_ease_monotonic(void) {
  for (int ease = 0; ease <= 3; ease++) {
    float prev = -1;
    for (int i = 0; i <= 100; i++) {
      float v = ease01(i / 100.0f, ease);
      TEST_ASSERT_TRUE_MESSAGE(v >= 0.0f && v <= 1.0f, "ease out of range");
      TEST_ASSERT_TRUE_MESSAGE(v >= prev - 1e-6f, "ease not monotonic");
      prev = v;
    }
  }
}

static void test_ease_endpoints(void) {
  TEST_ASSERT_EQUAL_FLOAT(0, ease01(0, 0));
  TEST_ASSERT_EQUAL_FLOAT(1, ease01(1, 3));
  TEST_ASSERT_EQUAL_FLOAT(0.5f, ease01(0.5f, 3)); // S-curve symmetric at midpoint
}

// Ramp: starts 0, hits target at end of up-phase, returns to 0 at total
static void test_ramp_shape(void) {
  SpinProfile p = makeProfile(3000, 10, 30, 10, 3);
  TEST_ASSERT_EQUAL_INT(0, rampRpm(0, p));
  TEST_ASSERT_EQUAL_INT(1500, rampRpm(5000, p));   // S-curve midpoint
  TEST_ASSERT_EQUAL_INT(3000, rampRpm(10000, p));
  TEST_ASSERT_EQUAL_INT(3000, rampRpm(39999, p));  // last ms of hold
  TEST_ASSERT_EQUAL_INT(1500, rampRpm(45000, p));  // ramp-down midpoint
  TEST_ASSERT_TRUE(rampRpm(49999, p) < 10);        // almost stopped
  TEST_ASSERT_EQUAL_INT(0, rampRpm(50000, p));     // done
  TEST_ASSERT_EQUAL_UINT32(50000, profileTotalMs(p));
}

static void test_ramp_phases(void) {
  SpinProfile p = makeProfile(3000, 10, 0, 10, 0);  // zero hold allowed
  TEST_ASSERT_EQUAL(PHASE_UP, profilePhase(9999, p));
  TEST_ASSERT_EQUAL(PHASE_DOWN, profilePhase(10000, p));
  TEST_ASSERT_EQUAL(PHASE_DONE, profilePhase(20000, p));
  TEST_ASSERT_EQUAL_INT(3000, rampRpm(10000, p));
}

// Safety clamp: profile cap + absolute ceiling
static void test_safety_ceiling(void) {
  Safety s;
  TEST_ASSERT_EQUAL_INT(RPM_SAFE_MAX, s.update(9000, -1, 0, RPM_SAFE_MAX));
  TEST_ASSERT_FALSE(s.hasFault());
  TEST_ASSERT_EQUAL_INT(0, s.update(-500, -1, 0, RPM_SAFE_MAX));
  TEST_ASSERT_EQUAL_INT(0, s.update(20000, -1, 0, RPM_SAFE_MAX)); // above absolute max
  TEST_ASSERT_EQUAL(FAULT_OVER_RPM_CMD, s.s.fault);
  TEST_ASSERT_EQUAL_INT(0, s.update(3000, -1, 10, RPM_SAFE_MAX)); // latched until clear
  s.clear();
  TEST_ASSERT_EQUAL_INT(3000, s.update(3000, -1, 20, RPM_SAFE_MAX));
}

static void test_safety_current_derate(void) {
  Safety s;
  // Derate never raises the command and never fires at a modest 3000 RPM
  for (int rpm = 0; rpm <= RPM_ABSOLUTE_MAX; rpm += 250) {
    int out = s.update(rpm, -1, 0, RPM_ABSOLUTE_MAX);
    TEST_ASSERT_TRUE(out <= rpm);
    TEST_ASSERT_TRUE_MESSAGE(estCurrentA(out) <= EST_CURRENT_TRIP_A + 0.01f, "derate failed");
  }
  TEST_ASSERT_EQUAL_INT(3000, s.update(3000, -1, 0, RPM_SAFE_MAX));
  TEST_ASSERT_FALSE(s.s.currentLimited);
}

static void test_safety_estop(void) {
  Safety s;
  s.pressEstop();
  TEST_ASSERT_EQUAL_INT(0, s.update(3000, -1, 0, RPM_SAFE_MAX));
  TEST_ASSERT_EQUAL(FAULT_ESTOP, s.s.fault);
  s.clear();
  TEST_ASSERT_FALSE(s.hasFault());
}

// Measured overspeed must persist > 1 s before tripping
static void test_safety_overspeed(void) {
  Safety s;
  int over = RPM_SAFE_MAX + OVERSPEED_MARGIN_RPM + 1;
  TEST_ASSERT_EQUAL_INT(5000, s.update(5000, over, 1000, RPM_SAFE_MAX));
  TEST_ASSERT_EQUAL_INT(5000, s.update(5000, over, 2000, RPM_SAFE_MAX));
  TEST_ASSERT_FALSE(s.hasFault());
  TEST_ASSERT_EQUAL_INT(0, s.update(5000, over, 2001, RPM_SAFE_MAX));
  TEST_ASSERT_EQUAL(FAULT_OVER_RPM_MEAS, s.s.fault);
  // A dip below the threshold restarts the timer
  Safety t;
  t.update(5000, over, 1000, RPM_SAFE_MAX);
  t.update(5000, 5000, 1500, RPM_SAFE_MAX);
  t.update(5000, over, 1600, RPM_SAFE_MAX);
  t.update(5000, over, 2500, RPM_SAFE_MAX);
  TEST_ASSERT_FALSE(t.hasFault());
}

// Tach fitted: commanding > threshold with 0 RPM for > 5 s -> fault
static void test_safety_tacho_lost(void) {
  Safety s;
  s.update(3000, 0, 1000, RPM_SAFE_MAX);
  s.update(3000, 0, 6000, RPM_SAFE_MAX);
  TEST_ASSERT_FALSE(s.hasFault());
  TEST_ASSERT_EQUAL_INT(0, s.update(3000, 0, 6001, RPM_SAFE_MAX));
  TEST_ASSERT_EQUAL(FAULT_TACHO_LOST, s.s.fault);
  // No tach (-1): never trips
  Safety n;
  n.update(3000, -1, 0, RPM_SAFE_MAX);
  TEST_ASSERT_EQUAL_INT(3000, n.update(3000, -1, 60000, RPM_SAFE_MAX));
  TEST_ASSERT_FALSE(n.hasFault());
}

// Throttle mapping never exceeds SAFE_US and respects deadband
static void test_throttle_bounds(void) {
  TEST_ASSERT_TRUE(rpmToUs(0) <= ESC_SPIN_US);
  TEST_ASSERT_TRUE(rpmToUs(12000) <= ESC_SAFE_US);
  TEST_ASSERT_TRUE(rpmToUs(RPM_ABSOLUTE_MAX) <= ESC_SAFE_US);
  TEST_ASSERT_TRUE(rpmToUs(3000) > ESC_SPIN_US); // a real setpoint actually spins
  int prev = 0;
  for (int rpm = 0; rpm <= RPM_ABSOLUTE_MAX; rpm += 100) {
    int us = rpmToUs(rpm);
    TEST_ASSERT_TRUE_MESSAGE(us >= prev, "throttle map not monotonic");
    prev = us;
  }
}

static void test_tach_period(void) {
  TEST_ASSERT_EQUAL_INT(1000, tachPeriodToRpm(60000, 1));   // 60 ms/rev
  TEST_ASSERT_EQUAL_INT(10000, tachPeriodToRpm(6000, 1));
  TEST_ASSERT_EQUAL_INT(1000, tachPeriodToRpm(30000, 2));   // 2 pulses/rev
  TEST_ASSERT_EQUAL_INT(0, tachPeriodToRpm(0, 1));
  TEST_ASSERT_EQUAL_INT(TACH_MAX_RPM, tachPeriodToRpm(TACH_MIN_PERIOD_US, TACH_PULSES_PER_REV));
}

// Table interp: clamps, lerps, invalid falls back to KV curve
static void test_calib_interp(void) {
  RpmCal c;
  c.valid = true; c.n = 3;
  c.us[0] = 1150; c.rpm[0] = 500;
  c.us[1] = 1300; c.rpm[1] = 2000;
  c.us[2] = 1500; c.rpm[2] = 5000;
  TEST_ASSERT_EQUAL_INT(1150, rpmToUsCal(400, c));    // below range clamps
  TEST_ASSERT_EQUAL_INT(1500, rpmToUsCal(6000, c));   // above range clamps
  int mid = rpmToUsCal(1250, c);                      // halfway 500->2000
  TEST_ASSERT_TRUE(mid > 1150 && mid < 1300);
  TEST_ASSERT_EQUAL_INT(2000, usToRpmCal(1300, c));
  RpmCal bad; // invalid -> KV fallback, must stay in bounds
  int us = rpmToUsCal(3000, bad);
  TEST_ASSERT_TRUE(us > ESC_SPIN_US && us <= ESC_SAFE_US);
}

// Best-guess throttle: table > locked anchor point > KV curve
static void test_best_anchor(void) {
  RpmCal c; FbCal f;
  int us = rpmToUsBest(3000, c, f);
  TEST_ASSERT_TRUE(us > ESC_SPIN_US && us <= ESC_SAFE_US);
  f.usRef = 1400; f.rpmRef = 3000;
  TEST_ASSERT_EQUAL_INT(1400, rpmToUsBest(3000, c, f));
  TEST_ASSERT_EQUAL_INT(1410, rpmToUsBest(3100, c, f));
  c.valid = true; c.n = 2;
  c.us[0] = 1150; c.rpm[0] = 500;
  c.us[1] = 1500; c.rpm[1] = 5000;
  TEST_ASSERT_EQUAL_INT(1150, rpmToUsBest(400, c, f));  // table wins over anchor
}
static void test_trim_cap(void) {
  TrimPI t;
  int out = 0;
  for (int i = 0; i < 200; i++) out = t.update(3000, 0, true, 1000 + i * 200);
  TEST_ASSERT_TRUE(out <= TRIM_MAX_US && out >= -TRIM_MAX_US);
  TrimPI t2;
  int out2 = 0;
  for (int i = 0; i < 200; i++) out2 = t2.update(3000, 6000, true, 1000 + i * 200);
  TEST_ASSERT_TRUE(out2 >= -TRIM_MAX_US && out2 <= TRIM_MAX_US);
}

int main(int argc, char** argv) {
  (void)argc; (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_ease_monotonic);
  RUN_TEST(test_ease_endpoints);
  RUN_TEST(test_ramp_shape);
  RUN_TEST(test_ramp_phases);
  RUN_TEST(test_safety_ceiling);
  RUN_TEST(test_safety_current_derate);
  RUN_TEST(test_safety_estop);
  RUN_TEST(test_safety_overspeed);
  RUN_TEST(test_safety_tacho_lost);
  RUN_TEST(test_throttle_bounds);
  RUN_TEST(test_tach_period);
  RUN_TEST(test_calib_interp);
  RUN_TEST(test_best_anchor);
  RUN_TEST(test_trim_cap);
  return UNITY_END();
}
