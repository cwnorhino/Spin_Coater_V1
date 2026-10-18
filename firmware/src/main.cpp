#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <Keypad.h>
#include "config.h"
#include "spin_math.h"
#include "safety.h"

// ---- Globals ----
static Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
static Servo esc;
static Safety safety;
static SpinProfile prof;
static Preferences prefs;

static const byte KEY_MAP[KEY_ROWS][KEY_COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
static byte key_row_pins[KEY_ROWS] = { KEY_ROW_PINS[0], KEY_ROW_PINS[1], KEY_ROW_PINS[2], KEY_ROW_PINS[3] };
static byte key_col_pins[KEY_COLS] = { KEY_COL_PINS[0], KEY_COL_PINS[1], KEY_COL_PINS[2], KEY_COL_PINS[3] };
static Keypad keys = Keypad(makeKeymap(KEY_MAP), key_row_pins, key_col_pins, KEY_ROWS, KEY_COLS);

// Runtime state
static int measRpm = -1;          // tach RPM, -1 = no tach fitted
static int cmdRpm = 0;            // last safety-clamped command
static int cmdUs = ESC_MIN_US;    // last ESC pulse width written
static bool spinActive = false;
static uint32_t spinStartMs = 0;
static bool redrawNow = true;

// Tach: ISR records the time between pulses (period method — full resolution
// at any speed, unlike counting 1 pulse/rev in a short window).
#if USE_TACHO
static portMUX_TYPE tachMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t tachLastUs = 0;
static volatile uint32_t tachPeriodUs = 0;   // 0 = no pulse seen yet
#endif

// ---- Hardware setup ----
void setupESC() {
  esc.setPeriodHertz(ESC_PWM_HZ);   // standard ESC 50Hz
  esc.attach(ESC_PIN, ESC_MIN_US, ESC_MAX_US);
  esc.setTimerWidth(ESC_TIMER_BITS); // must follow attach(): attach() resets width to 10 bits
  esc.writeMicroseconds(ESC_ARM_US);
  delay(ESC_ARM_TIME_MS);           // arm window
}

void setupDisplay() {
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    // fatal: loop blink on GPIO2 (onboard LED on most devkits)
    pinMode(2, OUTPUT);
    while (1) { digitalWrite(2, millis() & 0x200); delay(200); }
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.display();
}

#if USE_TACHO
// Tach ISR — wire a photointerrupter / hall sensor to TACH_PIN.
void IRAM_ATTR onTach() {
  uint32_t now = micros();
  portENTER_CRITICAL_ISR(&tachMux);
  uint32_t dt = now - tachLastUs;
  if (dt >= TACH_MIN_PERIOD_US) {   // shorter = noise/bounce, ignore
    tachPeriodUs = dt;
    tachLastUs = now;
  }
  portEXIT_CRITICAL_ISR(&tachMux);
}
#endif

void setupTacho() {
#if USE_TACHO
  pinMode(TACH_PIN, INPUT);         // GPIO34: no internal pull-up, fit 10k to 3V3
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), onTach, FALLING);
#endif
}

static bool estopPinActive() {
#if USE_ESTOP_PIN
  return digitalRead(ESTOP_PIN) == ESTOP_ACTIVE_LEVEL;
#else
  return false;
#endif
}

void setupEstop() {
#if USE_ESTOP_PIN
  pinMode(ESTOP_PIN, INPUT);        // GPIO35: no internal pull-up, fit 10k to 3V3
  if (estopPinActive()) safety.pressEstop();
#endif
}

// ---- RPM feedback ----
// Returns measured RPM, 0 if stopped, -1 if no tachometer is fitted.
static int readTachRpm() {
#if USE_TACHO
  uint32_t last, period;
  portENTER_CRITICAL(&tachMux);
  last = tachLastUs;
  period = tachPeriodUs;
  portEXIT_CRITICAL(&tachMux);
  if (period == 0) return 0;
  uint32_t since = micros() - last;
  if (since > TACH_TIMEOUT_US) return 0;
  if (since > period) period = since;  // slowing down: speed is at most this
  return tachPeriodToRpm(period, TACH_PULSES_PER_REV);
#else
  return -1;
#endif
}

// ---- Profile storage (NVS) ----
static void loadProfile() {
  SpinProfile p;
  if (prefs.begin("spin", true)) {
    p.targetRpm = prefs.getInt("rpm", p.targetRpm);
    p.rampUpS   = prefs.getInt("up", p.rampUpS);
    p.holdS     = prefs.getInt("hold", p.holdS);
    p.rampDnS   = prefs.getInt("dn", p.rampDnS);
    p.ease      = prefs.getUChar("ease", p.ease);
    prefs.end();
  }
  // Re-validate stored values against the current limits.
  SpinProfile d;
  if (p.targetRpm < RPM_MIN || p.targetRpm > p.maxRpmCap) p.targetRpm = d.targetRpm;
  if (p.rampUpS < RAMP_MIN_S || p.rampUpS > RAMP_MAX_S) p.rampUpS = d.rampUpS;
  if (p.holdS < 0 || p.holdS > HOLD_MAX_S) p.holdS = d.holdS;
  if (p.rampDnS < RAMP_MIN_S || p.rampDnS > RAMP_MAX_S) p.rampDnS = d.rampDnS;
  if (p.ease > 3) p.ease = d.ease;
  prof = p;
}

static void saveProfile() {
  if (!prefs.begin("spin", false)) return;
  prefs.putInt("rpm", prof.targetRpm);
  prefs.putInt("up", prof.rampUpS);
  prefs.putInt("hold", prof.holdS);
  prefs.putInt("dn", prof.rampDnS);
  prefs.putUChar("ease", prof.ease);
  prefs.end();
}

// ---- Main spin state machine ----
static void stopSpin() {
  esc.writeMicroseconds(ESC_MIN_US);
  spinActive = false;
  cmdRpm = 0;
  cmdUs = ESC_MIN_US;
}

static void startSpin() {
  spinActive = true;
  spinStartMs = millis();
  Serial.println("t_ms,cmd_rpm,esc_us,tach_rpm");
}

// D on the fault screen. Refused while the E-stop input is held or the tach
// still sees the rotor turning.
static void tryClearFault() {
  if (estopPinActive()) return;
  if (measRpm > 0) return;
  safety.clear();
}

// CSV over Serial at 10 Hz during a run (for RPM-vs-us calibration / tuning).
static void logTelemetry(uint32_t now, uint32_t elapsed) {
  static uint32_t lastLog = 0;
  if (now - lastLog < 100) return;
  lastLog = now;
  Serial.printf("%lu,%d,%d,%d\n", (unsigned long)elapsed, cmdRpm, cmdUs, measRpm);
}

// ---- UI ----
// 128x64 at text size 1 = 21 chars x 8 lines.
static void drawScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  if (safety.hasFault()) {
    display.println("!! FAULT !!");
    display.println(faultName(safety.s.fault));
    display.println();
    display.println("Motor output cut.");
    if (estopPinActive()) display.println("E-stop input active");
    else if (measRpm > 0) display.println("Wait: rotor turning");
    else display.println("D: clear fault");
  } else if (!spinActive) {
    display.println("SPIN COATER v1.1");
    display.printf("RPM : %d (cap %d)\n", prof.targetRpm, prof.maxRpmCap);
    display.printf("Up  : %ds\n", prof.rampUpS);
    display.printf("Hold: %ds\n", prof.holdS);
    display.printf("Down: %ds\n", prof.rampDnS);
    display.printf("Ease: %s\n", easeName(prof.ease));
    display.println("#:RUN  D:EDIT");
    display.println("*:E-STOP");
  } else {
    static const char* const PHASE_NAMES[] = {"RAMP UP", "HOLD", "RAMP DN", "DONE"};
    uint32_t el = millis() - spinStartMs;
    uint32_t total = profileTotalMs(prof);
    unsigned long leftS = el < total ? (total - el + 999) / 1000 : 0;
    display.printf("%-8s %4lus left\n", PHASE_NAMES[profilePhase(el, prof)], leftS);
    display.printf("Set : %d rpm\n", prof.targetRpm);
    display.printf("Cmd : %d rpm\n", cmdRpm);
    if (measRpm >= 0) display.printf("Tach: %d rpm\n", measRpm);
    else display.println("Tach: not fitted");
    display.printf("ESC : %dus\n", cmdUs);
    if (safety.s.currentLimited) display.println("CUR LIM");
    display.println("D:STOP  *:E-STOP");
  }
  display.display();
}

// Key poll for the blocking menus. Sets abort if E-stop (key or pin) latched.
static char menuKey(bool &abort) {
  char k = keys.getKey();             // '*' latches E-stop via the event listener
  if (estopPinActive()) safety.pressEstop();
  abort = safety.hasFault();
  if (k == NO_KEY) delay(10);
  return k;
}

// ---- Edit-number helper ----
// Digits type a value, A/B step it, # deletes a digit.
// Returns true on C (val updated), false on D / E-stop (val unchanged).
static bool editNumber(int &val, int lo, int hi, int step, const char* lbl) {
  int v = val;
  bool typing = false;
  bool redraw = true;
  while (true) {
    if (redraw) {
      display.clearDisplay(); display.setCursor(0, 0);
      display.setTextSize(1); display.printf("%s (%d-%d)\n", lbl, lo, hi);
      display.setTextSize(2); display.printf("%d\n", v);
      display.setTextSize(1);
      display.println();
      display.println("0-9:type  #:del");
      display.printf("A/B: +/-%d\n", step);
      display.println("C:OK  D:cancel");
      display.display();
      redraw = false;
    }
    bool abort;
    char k = menuKey(abort);
    if (abort) return false;
    if (k == NO_KEY) continue;
    redraw = true;
    if (k >= '0' && k <= '9') {
      if (!typing) { v = 0; typing = true; }
      if (v < 10000) v = v * 10 + (k - '0');  // max 5 digits
    } else if (k == KEY_RUN) {
      if (typing) v /= 10;
    } else if (k == KEY_UP) {
      typing = false;
      v = constrain(v, lo, hi);
      v = min(hi, v + step);
    } else if (k == KEY_DOWN) {
      typing = false;
      v = constrain(v, lo, hi);
      v = max(lo, v - step);
    } else if (k == KEY_OK) {
      if (v < lo || v > hi) {           // show the clamped value, C again to accept
        v = constrain(v, lo, hi);
        typing = false;
        continue;
      }
      val = v;
      return true;
    } else if (k == KEY_BACK) {
      return false;
    }
  }
}

static bool editEase(uint8_t &ease) {
  uint8_t e = ease;
  bool redraw = true;
  while (true) {
    if (redraw) {
      display.clearDisplay(); display.setCursor(0, 0);
      display.setTextSize(1); display.println("Ease curve");
      display.setTextSize(2); display.println(easeName(e));
      display.setTextSize(1);
      display.println();
      display.println("0:Lin 1:In 2:Out 3:S");
      display.println("A/B: cycle");
      display.println("C:OK  D:cancel");
      display.display();
      redraw = false;
    }
    bool abort;
    char k = menuKey(abort);
    if (abort) return false;
    if (k == NO_KEY) continue;
    redraw = true;
    if (k >= '0' && k <= '3') e = k - '0';
    else if (k == KEY_UP) e = (e + 1) % 4;
    else if (k == KEY_DOWN) e = (e + 3) % 4;
    else if (k == KEY_OK) { ease = e; return true; }
    else if (k == KEY_BACK) return false;
  }
}

// Edits a copy of the profile; commits (and saves) only if every field is confirmed.
static void menuEdit() {
  SpinProfile p = prof;
  bool ok = editNumber(p.targetRpm, RPM_MIN, p.maxRpmCap, 100, "Target RPM")
         && editNumber(p.rampUpS, RAMP_MIN_S, RAMP_MAX_S, 1, "Ramp up s")
         && editNumber(p.holdS, 0, HOLD_MAX_S, 5, "Hold s")
         && editNumber(p.rampDnS, RAMP_MIN_S, RAMP_MAX_S, 1, "Ramp down s")
         && editEase(p.ease);
  if (ok) { prof = p; saveProfile(); }
  redrawNow = true;
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  setupDisplay();
  display.println("Arming ESC..."); display.display();
  setupESC();                       // blocks ESC_ARM_TIME_MS at ESC_ARM_US
  setupTacho();
  setupEstop();
  loadProfile();
  keys.addEventListener([](char e){ if (e == KEY_ESTOP) safety.pressEstop(); });
  redrawNow = true;
}

// ---- Loop ----
void loop() {
  if (estopPinActive()) safety.pressEstop();
  measRpm = readTachRpm();

  char k = keys.getKey();           // '*' also latches E-stop via the event listener
  if (k != NO_KEY) {
    redrawNow = true;
    switch (k) {
      case KEY_ESTOP: safety.pressEstop(); stopSpin(); break;
      case KEY_BACK:
        if (spinActive) stopSpin();
        else if (safety.hasFault()) tryClearFault();
        else menuEdit();
        break;
      case KEY_RUN:
        if (!spinActive && !safety.hasFault()) startSpin();
        break;
      default: break;
    }
  }

  if (safety.hasFault()) {
    stopSpin();
  } else if (spinActive) {
    uint32_t now = millis();
    uint32_t elapsed = now - spinStartMs;
    if (elapsed >= profileTotalMs(prof)) {
      stopSpin();
      redrawNow = true;
    } else {
      // THE interlock: cap / current-derate / tach trips apply to the OUTPUT.
      int safeRpm = safety.update(rampRpm(elapsed, prof), measRpm, now, prof.maxRpmCap);
      if (safety.hasFault()) {
        stopSpin();
        redrawNow = true;
      } else {
        cmdRpm = safeRpm;
        cmdUs = constrain(rpmToUs(safeRpm), ESC_MIN_US, ESC_SAFE_US);
        esc.writeMicroseconds(cmdUs);
        logTelemetry(now, elapsed);
      }
    }
  }

  // Throttle OLED redraw to ~8 Hz (I2C frame is ~25ms; every-loop is wasteful)
  static uint32_t lastDraw = 0;
  uint32_t nowD = millis();
  if (redrawNow || nowD - lastDraw >= 120) {
    drawScreen();
    lastDraw = nowD;
    redrawNow = false;
  }
}
