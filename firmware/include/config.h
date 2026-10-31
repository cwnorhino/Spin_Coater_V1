#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stdint.h>   // native unit tests
#endif

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

// Optional inputs. GPIO34-39 are input-only and have NO internal pull-up or
// pull-down, so each of these needs an external pull-up (10k to 3V3).
// A floating GPIO34/35 reads LOW/noisy: spurious E-stops and tach counts.
#define USE_ESTOP_PIN  0    // 1 = hardware E-stop wired on ESTOP_PIN (0 = no button fitted)
#define USE_TACHO      1    // 1 = reflective IR tachometer on TACH_PIN (closed-loop)

#define TACH_PIN       23   // reflective IR OUT here, falling edge per pulse (has pull-up + interrupt)
#define ESTOP_PIN      35   // E-stop input
// Level that means "STOP". NO button to GND + pull-up -> LOW.
// Fail-safe option: NC button to GND + pull-up -> HIGH (broken wire = stop).
#define ESTOP_ACTIVE_LEVEL LOW
#define ISENSE_PIN     36   // Analog current sensor (ACS712/INA analog out), optional
#define BUZZER_PIN     -1   // not fitted (was 23, now tach)

// Tachometer
#define TACH_PULSES_PER_REV 1
#define TACH_MAX_RPM        25000   // pulses faster than this are rejected as noise
#define TACH_MIN_PERIOD_US  (60000000UL / ((uint32_t)TACH_MAX_RPM * TACH_PULSES_PER_REV))
#define TACH_TIMEOUT_US     1500000UL // no pulse for 1.5 s -> 0 RPM (400rpm = 150ms/rev)

// Calibration (Option A: dense table + min-detect)
#define CAL_POINTS        12    // us->rpm table points, ~5s each => ~60s sweep
#define CAL_TOP_RPM       6000  // sweep top; never near cap (no high-RPM stress)
#define CAL_DWELL_MS      5000  // per point: 2.5s settle + 2.5s average
#define CAL_MIN_STABLE    5     // need >=5 stable points for a valid table
#define CAL_LOW_RPM       1000  // need >=1 stable point below this
#define FB_MIN_STEP_US    10    // min-detect ramp step
#define FB_MIN_STEP_MS    250   // min-detect step period
#define FB_MIN_MAX_US     1400  // give up min-detect above this -> fallback entry
#define FB_FALLBACK_RPM   500   // prefill if auto min-detect fails
#define FB_FALLBACK_US    1150
#define TRIM_MAX_US       150   // closed-loop trim authority (+/-)
#define TRIM_UPDATE_MS    200   // 5 Hz trim

// ESC pulse widths (microseconds, standard servo)
// Uncalibrated SimonK: stop <= 1060us, full throttle >= 1860us. Calibrate the
// ESC to 1000-2000us so ESC_SAFE_US really leaves headroom (1850 = ~85%).
#define ESC_MIN_US     1000
#define ESC_MAX_US     2000
#define ESC_SPIN_US    1090  // below this most A2212 won't spin — deadband top
#define ESC_SAFE_US    1850  // never exceed this
#define ESC_ARM_US     1000
#define ESC_ARM_TIME_MS 3000
#define ESC_PWM_HZ     50
#define ESC_TIMER_BITS 16    // LEDC resolution; library default 10 bits = 19.5us steps

// Motor model: A2212 1400KV on 12V
#define MOTOR_KV       1400.0f
#define SUPPLY_V       12.0f
#define MOTOR_EFF      0.72f  // loaded efficiency fudge (prop/chuck losses)
static inline float rpmAtFullThrottle() { return MOTOR_KV * SUPPLY_V * MOTOR_EFF; } // ~12096

// Safety limits (software interlocks — independent of UI)
#define RPM_MIN           400
#define RPM_SAFE_MAX      8000    // default cap: safe for 3D-printed chuck
#define RPM_ABSOLUTE_MAX 10000    // firmware will NEVER command above this
#define OVERSPEED_MARGIN_RPM 500  // tach: measured > cap + margin for 1 s -> fault
#define TACHO_LOST_MIN_CMD_RPM 1500
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

// Temp debug: boot straight into KEY TEST (set 0 for normal menu boot).
#define BOOT_KEYTEST 0

// Firmware version (shown on every screen + Serial boot; OTA compares this)
#define FW_VERSION "1.3.1"

// Self-update: menu 5 UPDATE joins this hotspot and pulls firmware.bin
// from the latest GitHub release, then flashes itself.
#define OTA_SSID "Terminal 2"
#define OTA_PASS "mfvr7566"
#define OTA_FW_URL "https://github.com/cwnorhino/Spin_Coater_V1/releases/latest/download/firmware.bin"

// UI keys (membrane 4x4: 0-9, A-D, *, #)
#define KEY_UP    'A'
#define KEY_DOWN  'B'
#define KEY_OK    'C'
#define KEY_BACK  'D'
#define KEY_ESTOP '*'
#define KEY_RUN   '#'
