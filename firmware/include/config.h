#pragma once
#include <Arduino.h>

// ===================== SpinCoater Config =====================
// ESP32 DevKit (CP2102) pin map — all safe (no strapping pins)

#define ESC_PIN        13   // ESC signal (servo PWM 1000-2000us)
#define OLED_SDA       21
#define OLED_SCL       22
#define OLED_ADDR      0x3C
#define OLED_W         128
#define OLED_H         64

// 4x4 membrane keypad
#define KEY_ROWS 4
#define KEY_COLS 4
// Safe GPIOs (avoid 0,2,5,12,15 strapping + 34-39 input-only for rows/cols)
static const uint8_t KEY_ROW_PINS[KEY_ROWS] = {14, 27, 26, 25};
static const uint8_t KEY_COL_PINS[KEY_COLS] = {33, 32, 18, 19};

// Future / optional (wire later, code already supports)
#define TACH_PIN       34   // IR/Hall tachometer: 1 pulse per rev, INPUT_PULLUP, interrupt
#define ESTOP_PIN      35   // Normally-open button to GND, INPUT_PULLUP (optional)
#define ISENSE_PIN     36   // Analog current sensor (ACS712/INA analog out), optional
#define BUZZER_PIN     23   // Active buzzer via NPN or passive via LEDC (optional)

// ESC pulse widths (microseconds, standard servo)
#define ESC_MIN_US     1000
#define ESC_MAX_US     2000
#define ESC_SPIN_US    1090  // below this most A2212 won't spin — deadband top
#define ESC_SAFE_US    1850  // never exceed this (keeps headroom below 2000)
#define ESC_ARM_US     1000
#define ESC_ARM_TIME_MS 3000

// Motor model: A2212 1400KV on 12V
#define MOTOR_KV       1400.0f
#define SUPPLY_V       12.0f
#define MOTOR_EFF      0.72f  // loaded efficiency fudge (prop/chuck losses)
static inline float rpmAtFullThrottle() { return MOTOR_KV * SUPPLY_V * MOTOR_EFF; } // ~12096

// Safety limits (software interlocks — independent of UI)
#define RPM_MIN           300
#define RPM_SAFE_MAX      8000    // default cap: safe for 3D-printed chuck
#define RPM_ABSOLUTE_MAX 10000    // firmware will NEVER command above this
#define EST_CURRENT_TRIP_A 18.0f  // software estimated-current trip (30A ESC, 15A SMPS)
#define RAMP_MIN_S        1
#define RAMP_MAX_S        300
#define HOLD_MAX_S        600

// Profile defaults
#define DEF_TARGET_RPM 3000
#define DEF_RAMP_UP_S  10
#define DEF_HOLD_S     30
#define DEF_RAMP_DN_S  10
// Ease: 0=Linear 1=EaseIn 2=EaseOut 3=S-Curve
#define DEF_EASE 3

// UI keys (membrane 4x4: 0-9, A-D, *, #)
#define KEY_UP    'A'
#define KEY_DOWN  'B'
#define KEY_OK    'C'
#define KEY_BACK  'D'
#define KEY_ESTOP '*'
#define KEY_RUN   '#'
