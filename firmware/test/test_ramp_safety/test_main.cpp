#include <unity.h>
#include "test_math.h"

void setUp(void) {}
void tearDown(void) {}

// Ease curves stay in [0,1] and are monotonic
static void test_ease_monotonic(void) {
  for (int ease = 0; ease <= 3; ease++) {
    float prev = -1;
    for (int i = 0; i <= 100; i++) {
      float v = t_ease01(i / 100.0f, ease);
      TEST_ASSERT_TRUE_MESSAGE(v >= 0.0f && v <= 1.0f, "ease out of range");
      TEST_ASSERT_TRUE_MESSAGE(v >= prev - 1e-6f, "ease not monotonic");
      prev = v;
    }
  }
}

static void test_ease_endpoints(void) {
  TEST_ASSERT_EQUAL_FLOAT(0, t_ease01(0, 0));
  TEST_ASSERT_EQUAL_FLOAT(1, t_ease01(1, 3));
  TEST_ASSERT_EQUAL_FLOAT(0.5f, t_ease01(0.5f, 3)); // S-curve symmetric at midpoint
}

// Ramp: starts 0, hits target at end of up-phase, returns to 0 at total
static void test_ramp_shape(void) {
  TEST_ASSERT_EQUAL_INT(0, t_rampRpm(0, 3000, 10, 30, 10, 3));
  TEST_ASSERT_EQUAL_INT(3000, t_rampRpm(10000, 3000, 10, 30, 10, 3));
  TEST_ASSERT_EQUAL_INT(3000, t_rampRpm(20000, 3000, 10, 30, 10, 3)); // hold
  TEST_ASSERT_EQUAL_INT(3000, t_rampRpm(39999, 3000, 10, 30, 10, 3)); // near end of ramp-down
  TEST_ASSERT_EQUAL_INT(0, t_rampRpm(50000, 3000, 10, 30, 10, 3));   // done
}

// Safety clamp: absolute ceiling + current derate never exceed trip
static void test_safety_ceiling(void) {
  TEST_ASSERT_TRUE(t_safetyClamp(20000) <= T_RPM_ABSOLUTE_MAX);
  TEST_ASSERT_EQUAL_INT(0, t_safetyClamp(-500));
}

static void test_safety_current_derate(void) {
  // At full throttle est current ~14.8A; clamp must keep est <= trip + small eps
  int clamped = t_safetyClamp(12000);
  float i = t_estCurrentA(clamped);
  TEST_ASSERT_TRUE_MESSAGE(i <= T_EST_TRIP_A + 0.01f, "derate failed to hold current under trip");
  // And derate must not fire at a modest 3000 RPM
  TEST_ASSERT_EQUAL_INT(3000, t_safetyClamp(3000));
}

// Throttle mapping never exceeds SAFE_US and respects deadband
static void test_throttle_bounds(void) {
  TEST_ASSERT_TRUE(t_rpmToUs(0) <= T_SPIN_US);
  TEST_ASSERT_TRUE(t_rpmToUs(12000) <= T_SAFE_US);
  TEST_ASSERT_TRUE(t_rpmToUs(3000) > T_SPIN_US); // a real setpoint actually spins
}

int main(int argc, char** argv) {
  (void)argc; (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_ease_monotonic);
  RUN_TEST(test_ease_endpoints);
  RUN_TEST(test_ramp_shape);
  RUN_TEST(test_safety_ceiling);
  RUN_TEST(test_safety_current_derate);
  RUN_TEST(test_throttle_bounds);
  return UNITY_END();
}
