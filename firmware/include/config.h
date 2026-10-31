#pragma once
#include <Arduino.h>

// ===================== Spin Coater config =====================
#define FW_VERSION "2.0.0"

// ---- pins (ESP32 DevKit, no strapping pins) ----
#define ESC_PIN   13
#define TACH_PIN  23          // IR module OUT via 20k/10k divider, falling edge = 1 pulse
#define OLED_SDA  21
#define OLED_SCL  22
#define OLED_ADDR 0x3C
static const byte KEY_ROW_PINS[4] = {14, 27, 26, 25};
static const byte KEY_COL_PINS[4] = {33, 32, 18, 19};

// ---- ESC (SimonK 30A, A2212 1400KV, 15V supply) ----
// 1400KV x 15V = ~21000 rpm unloaded at full throttle, so the pulse ceilings
// are kept LOW. 8000 rpm needs roughly 1370us; raise these only without a chuck.
#define ESC_MIN_US   1000
#define ESC_MAX_US   2000
#define ESC_SAFE_US  1500     // closed-loop ceiling
#define TEST_MAX_US  1400     // raw throttle test ceiling
#define ESC_ARM_MS   3000

// ---- speed limits ----
#define RPM_MIN  300
#define RPM_MAX  8000         // chuck rating

// ---- tach ----
#define TACH_PPR           1           // pulses per revolution (2 if it double counts)
#define TACH_MAX_RPM       25000
#define TACH_MIN_PERIOD_US (60000000UL / ((uint32_t)TACH_MAX_RPM * TACH_PPR))
#define TACH_TIMEOUT_US    800000UL    // no pulse for 0.8s = stopped
#define TACH_HOLDOFF_MAX_US 40000UL    // max chatter hold-off after a valid pulse

// ---- control tuning ----
#define TICK_MS        100
#define SEEK_START_US  1080
#define SEEK_STEP_US   3         // per tick -> 30us/s creep
#define SEEK_MAX_US    1400      // no spin by here -> fault
#define SLEW_RPM_S     500.0f    // setpoint slew rate
#define KP_US_PER_RPM  0.015f
#define STEP_MAX_US    10.0f     // max throttle move per tick
#define STALL_MS       1500

// ---- OTA: GitHub release asset named firmware.bin ----
#define OTA_FW_URL "https://github.com/cwnorhino/Spin_Coater_V1/releases/latest/download/firmware.bin"
// WIFI_SSID / WIFI_PASS live in secrets.h (not committed)