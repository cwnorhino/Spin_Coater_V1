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
#include "calib.h"
#include "feedback.h"

// ---- Globals ----
static Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
static Servo esc;
static Safety safety;
static SpinProfile prof;
static Preferences prefs;
static RpmCal rpmCal;
static FbCal fbCal;
static TrimPI trimPI;

// Rotated 180° vs label order (see AGENTS.md). DO NOT revert to naive map.
static const byte KEY_MAP[KEY_ROWS][KEY_COLS] = {
  {'D','C','B','A'},
  {'#','9','6','3'},
  {'0','8','5','2'},
  {'*','7','4','1'}
};
static byte key_row_pins[KEY_ROWS] = { KEY_ROW_PINS[0], KEY_ROW_PINS[1], KEY_ROW_PINS[2], KEY_ROW_PINS[3] };
static byte key_col_pins[KEY_COLS] = { KEY_COL_PINS[0], KEY_COL_PINS[1], KEY_COL_PINS[2], KEY_COL_PINS[3] };
static Keypad keys = Keypad(makeKeymap(KEY_MAP), key_row_pins, key_col_pins, KEY_ROWS, KEY_COLS);

// Runtime state
static int measRpm = -1;          // tach RPM, -1 = no tach fitted
static int cmdRpm = 0;            // last safety-clamped command
static int cmdUs = ESC_MIN_US;    // last ESC pulse width written
static int trimUs = 0;            // closed-loop trim component
static bool spinActive = false;
static uint32_t spinStartMs = 0;
static bool redrawNow = true;

enum AppMode : uint8_t { APP_MENU, APP_RUN_SETUP, APP_RUNNING, APP_CAL_RPM, APP_CAL_FB };
static AppMode appMode = APP_MENU;
static uint8_t menuIdx = 2;       // default on RUN

// Tach: ISR records the time between pulses (period method — full resolution
// at any speed, unlike counting 1 pulse/rev in a short window).
static portMUX_TYPE tachMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t tachLastUs = 0;
static volatile uint32_t tachPeriodUs = 0;   // 0 = no pulse seen yet

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

// ---- Splash: V1 fades in/out, any key skips ----
static void splashV1() {
  display.clearDisplay();
  display.setTextSize(4);
  // center "V1" on 128x64 at size 4 (48x32 glyph box)
  display.setCursor(40, 16);
  display.print("V1");
  display.display();
  // fake fade on SSD1306 (no backlight): invert pulses read as fade
  for (int i = 0; i < 3; i++) {
    if (keys.getKey() != NO_KEY) break;
    delay(220);
    display.invertDisplay(true);
    delay(160);
    display.invertDisplay(false);
  }
  delay(350);
  display.clearDisplay();
  display.display();
}

// IRAM_ATTR on ESP32 (no-op natively for unit tests).
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
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

void setupTacho() {
#if USE_TACHO
  pinMode(TACH_PIN, INPUT);         // reflective module drives OUT push-pull; no 10k needed
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

// Median of N tach samples (rejects single missed-pulse glitches).
static int tachMedian(int n, int gapMs) {
  int buf[16];
  if (n > 16) n = 16;
  for (int i = 0; i < n; i++) {
    if (estopPinActive()) safety.pressEstop();
    buf[i] = readTachRpm();
    delay(gapMs);
  }
  // insertion sort
  for (int i = 1; i < n; i++) {
    int v = buf[i], j = i - 1;
    while (j >= 0 && buf[j] > v) { buf[j + 1] = buf[j]; j--; }
    buf[j + 1] = v;
  }
  return buf[n / 2];
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
  int cap = fbCal.cap > 0 ? fbCal.cap : RPM_SAFE_MAX;
  if (p.targetRpm < RPM_MIN || p.targetRpm > cap) p.targetRpm = d.targetRpm;
  if (p.rampUpS < RAMP_MIN_S || p.rampUpS > RAMP_MAX_S) p.rampUpS = d.rampUpS;
  if (p.holdS < 0 || p.holdS > HOLD_MAX_S) p.holdS = d.holdS;
  if (p.rampDnS < RAMP_MIN_S || p.rampDnS > RAMP_MAX_S) p.rampDnS = d.rampDnS;
  if (p.ease > 3) p.ease = d.ease;
  p.maxRpmCap = cap;
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

// Closed-loop throttle: table feedforward + capped PI trim.
static int closedLoopUs(int targetRpm, int measured, uint32_t nowMs) {
  int base = rpmToUsCal(targetRpm, rpmCal);
  if (targetRpm <= 0) { trimPI.reset(); return ESC_MIN_US; }
  bool tachOk = measured >= 0 && !(targetRpm > 800 && measured == 0 && (nowMs - spinStartMs) < 3000);
  // First 3 s allow spool-up before trim engages (avoids windup on deadband).
  if ((nowMs - spinStartMs) < 3000 && (appMode == APP_RUNNING)) tachOk = false;
  trimUs = trimPI.update(targetRpm, measured < 0 ? targetRpm : measured, tachOk, nowMs);
  int us = base + trimUs;
  if (us < ESC_MIN_US) us = ESC_MIN_US;
  if (us > ESC_SAFE_US) us = ESC_SAFE_US;
  return us;
}

// ---- Main spin state machine ----
static void stopSpin() {
  esc.writeMicroseconds(ESC_MIN_US);
  spinActive = false;
  trimPI.reset();
  trimUs = 0;
  cmdRpm = 0;
  cmdUs = ESC_MIN_US;
}

static void startSpin() {
  spinActive = true;
  appMode = APP_RUNNING;
  spinStartMs = millis();
  trimPI.reset();
  Serial.println("t_ms,cmd_rpm,esc_us,tach_rpm,trim_us");
}

// D on the fault screen. Refused while the E-stop input is held or the tach
// still sees the rotor turning.
static void tryClearFault() {
  if (estopPinActive()) return;
  if (measRpm > 0) return;
  safety.clear();
}

// CSV over Serial at 10 Hz (for RPM-vs-us calibration / tuning).
static void logTelemetry(uint32_t now, uint32_t elapsed) {
  static uint32_t lastLog = 0;
  if (now - lastLog < 100) return;
  lastLog = now;
  Serial.printf("%lu,%d,%d,%d,%d\n", (unsigned long)elapsed, cmdRpm, cmdUs, measRpm, trimUs);
}

// ---- UI ----
// 128x64 at text size 1 = 21 chars x 8 lines.
static void drawMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("SPIN COATER V1");
  const char* rows[3] = {"1 CAL RPM MAP", "2 CAL FEEDBACK", "3 RUN SPIN"};
  for (uint8_t i = 0; i < 3; i++) {
    if (i == menuIdx) display.print(">"); else display.print(" ");
    display.print(rows[i]);
    if (i == 0) display.print(rpmCal.valid ? " [OK]" : " [--]");
    if (i == 1) display.print(fbCal.hasMin ? " [OK]" : " [--]");
    display.println();
  }
  display.println();
  display.printf("Cap:%d Min:%d\n", fbCal.cap, fbCal.hasMin ? fbCal.rpmMin : fbCal.fbMinRpm);
  display.println("A/B:move C:ok *:STOP");
  display.display();
}

static void drawScreen() {
  if (appMode == APP_MENU && !spinActive) { drawMenu(); return; }
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
    if (!rpmCal.valid) display.println("MAP: UNCAL (KV est)");
    else display.printf("MAP: cal %d pts\n", rpmCal.n);
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
    if (measRpm >= 0) display.printf("Tach: %d rpm\n", measRpm);
    else display.println("Tach: not fitted");
    display.printf("Cmd : %d rpm\n", cmdRpm);
    display.printf("ESC : %dus T:%+d\n", cmdUs, trimUs);
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

// ---- Cal 1: ~60s dense us->rpm sweep (12 pts, chuck only, top ~6000rpm) ----
static void calRpmSweep() {
  appMode = APP_CAL_RPM;
  display.clearDisplay(); display.setCursor(0, 0);
  display.println("CAL RPM MAP");
  display.println("Chuck only!");
  display.println("Top ~6000rpm");
  display.println("C:start D:abort");
  display.display();
  bool abort = false;
  while (true) {
    char k = menuKey(abort);
    if (abort) { appMode = APP_MENU; return; }
    if (k == KEY_BACK) { appMode = APP_MENU; redrawNow = true; return; }
    if (k == KEY_OK || k == KEY_RUN) break;
  }
  int usStart = fbCal.hasMin ? fbCal.usMin + 10 : ESC_SPIN_US + 30;
  int usEnd = rpmToUs(CAL_TOP_RPM);   // KV estimate for sweep top
  if (usEnd > ESC_SAFE_US - 40) usEnd = ESC_SAFE_US - 40;
  if (usEnd < usStart + 200) usEnd = usStart + 200;
  RpmCal c;
  c.n = 0;
  Serial.println("cal_pt,esc_us,tach_rpm,stable");
  for (uint8_t i = 0; i < CAL_POINTS; i++) {
    int us = usStart + (int)((long)(usEnd - usStart) * i / (CAL_POINTS - 1));
    esc.writeMicroseconds(us);
    uint32_t t0 = millis();
    // settle half, then sample
    while (millis() - t0 < CAL_DWELL_MS / 2) {
      if (estopPinActive()) safety.pressEstop();
      if (safety.hasFault() || keys.getKey() == KEY_BACK) {
        esc.writeMicroseconds(ESC_MIN_US); appMode = APP_MENU; redrawNow = true; return;
      }
      measRpm = readTachRpm();
      display.clearDisplay(); display.setCursor(0, 0);
      display.printf("Pt %d/%d settle\n", i + 1, CAL_POINTS);
      display.printf("us %d\n", us);
      display.printf("tach %d rpm\n", measRpm);
      display.println("D:abort");
      display.display();
      delay(200);
    }
    int med = tachMedian(12, 200);
    int s1 = readTachRpm(); delay(200);
    int s2 = readTachRpm();
    int lo = min(s1, min(s2, med)), hi = max(s1, max(s2, med));
    bool stable = med > 0 && (hi - lo) <= max(50, med / 10);
    measRpm = med;
    Serial.printf("%d,%d,%d,%d\n", i, us, med, stable ? 1 : 0);
    display.clearDisplay(); display.setCursor(0, 0);
    display.printf("Pt %d/%d %s\n", i + 1, CAL_POINTS, stable ? "OK" : "SKIP");
    display.printf("us %d\n", us);
    display.printf("%d rpm\n", med);
    display.display();
    delay(400);
    if (stable && c.n < CAL_POINTS) { c.us[c.n] = us; c.rpm[c.n] = med; c.n++; }
    if (safety.hasFault()) { esc.writeMicroseconds(ESC_MIN_US); appMode = APP_MENU; return; }
  }
  esc.writeMicroseconds(ESC_MIN_US);
  // validate: >=5 stable pts incl one <1000rpm
  uint8_t low = 0;
  for (uint8_t i = 0; i < c.n; i++) if (c.rpm[i] < CAL_LOW_RPM) low++;
  if (c.n >= CAL_MIN_STABLE && low >= 1) {
    c.valid = true;
    rpmCal = c;
    calibSaveTable(rpmCal);
    display.clearDisplay(); display.setCursor(0, 0);
    display.printf("SAVED %d pts\n", c.n);
    display.println("C:done");
    display.display();
  } else {
    display.clearDisplay(); display.setCursor(0, 0);
    display.println("CAL FAILED");
    display.printf("stable %d (need %d)\n", c.n, CAL_MIN_STABLE);
    display.println("C:fallback D:menu");
    display.display();
    while (true) {
      bool ab; char k = menuKey(ab);
      if (ab || k == KEY_BACK) break;
      if (k == KEY_OK || k == KEY_RUN) {
        editNumber(fbCal.fbMinRpm, RPM_MIN, RPM_ABSOLUTE_MAX, 50, "Fallback min RPM");
        editNumber(fbCal.fbMaxRpm, RPM_MIN, RPM_ABSOLUTE_MAX, 100, "Fallback max RPM");
        fbCal.cap = fbCal.fbMaxRpm;
        prof.maxRpmCap = fbCal.cap;
        calibSaveFb(fbCal);
        break;
      }
    }
  }
  appMode = APP_MENU; redrawNow = true;
}

// ---- Cal 2: feedback min-detect + limits (never spins high) ----
static void calFeedback() {
  appMode = APP_CAL_FB;
  display.clearDisplay(); display.setCursor(0, 0);
  display.println("CAL FEEDBACK");
  display.println("Finds spin-start");
  display.println("C:start D:abort");
  display.display();
  bool abort = false;
  while (true) {
    char k = menuKey(abort);
    if (abort) { appMode = APP_MENU; return; }
    if (k == KEY_BACK) { appMode = APP_MENU; redrawNow = true; return; }
    if (k == KEY_OK || k == KEY_RUN) break;
  }
  int us = ESC_MIN_US;
  int lockUs = 0, lockRpm = 0;
  uint8_t agree = 0;
  int lastMed = 0;
  Serial.println("fb_us,tach_rpm");
  while (us <= FB_MIN_MAX_US) {
    esc.writeMicroseconds(us);
    if (estopPinActive()) safety.pressEstop();
    if (safety.hasFault() || keys.getKey() == KEY_BACK) {
      esc.writeMicroseconds(ESC_MIN_US); appMode = APP_MENU; redrawNow = true; return;
    }
    measRpm = readTachRpm();
    Serial.printf("%d,%d\n", us, measRpm);
    display.clearDisplay(); display.setCursor(0, 0);
    display.println("Seeking min...");
    display.printf("us %d\n", us);
    display.printf("tach %d rpm\n", measRpm);
    display.println("D:abort");
    display.display();
    delay(FB_MIN_STEP_MS);
    if (measRpm > 0) {
      int med = tachMedian(5, 200);
      if (lastMed > 0 && abs(med - lastMed) * 100 <= lastMed * 15 + 200) agree++;
      else agree = 1;
      lastMed = med;
      if (agree >= 3) {
        // 1 s sustain hold
        delay(1000);
        int confirm = tachMedian(5, 200);
        if (confirm > 0 && abs(confirm - med) * 100 <= med * 20 + 200) {
          lockUs = us; lockRpm = (med + confirm) / 2;
          break;
        }
        agree = 0;
      }
    } else { agree = 0; lastMed = 0; }
    us += FB_MIN_STEP_US;
  }
  esc.writeMicroseconds(ESC_MIN_US);
  if (lockUs > 0) {
    fbCal.hasMin = true;
    fbCal.usMin = lockUs;
    fbCal.rpmMin = lockRpm;
    fbCal.fbMinRpm = lockRpm;
    calibSaveFb(fbCal);
    display.clearDisplay(); display.setCursor(0, 0);
    display.println("MIN LOCKED");
    display.printf("us %d\n", lockUs);
    display.printf("%d rpm\n", lockRpm);
    display.println("C:cont");
    display.display();
    bool done = false;
    while (!done) {
      bool ab; char k = menuKey(ab);
      if (ab || k == KEY_BACK) break;
      if (k == KEY_OK || k == KEY_RUN) done = true;
    }
  } else {
    display.clearDisplay(); display.setCursor(0, 0);
    display.println("NO LOCK");
    display.println("Type fallback:");
    display.display();
    delay(600);
    editNumber(fbCal.fbMinRpm, RPM_MIN, RPM_ABSOLUTE_MAX, 50, "Fallback min RPM");
    fbCal.hasMin = false;
    fbCal.rpmMin = fbCal.fbMinRpm;
    calibSaveFb(fbCal);
  }
  // cap edit (default 8000, never auto-tests high)
  int cap = fbCal.cap;
  if (editNumber(cap, RPM_MIN, RPM_ABSOLUTE_MAX, 100, "Max cap RPM")) {
    fbCal.cap = cap;
    fbCal.fbMaxRpm = cap;
    prof.maxRpmCap = cap;
    calibSaveFb(fbCal);
    saveProfile();
  }
  appMode = APP_MENU; redrawNow = true;
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  setupDisplay();
  splashV1();
  display.println("Arming ESC..."); display.display();
  setupESC();                       // blocks ESC_ARM_TIME_MS at ESC_ARM_US
  setupTacho();
  setupEstop();
  calibLoad(rpmCal, fbCal);
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
    if (k == KEY_ESTOP) { safety.pressEstop(); stopSpin(); appMode = APP_MENU; }
    else if (appMode == APP_MENU && !spinActive) {
      if (k == KEY_UP || k == '2') menuIdx = (menuIdx + 2) % 3;
      else if (k == KEY_DOWN || k == '8') menuIdx = (menuIdx + 1) % 3;
      else if (k >= '1' && k <= '3') menuIdx = k - '1';
      else if (k == KEY_OK || k == KEY_RUN) {
        if (menuIdx == 0) calRpmSweep();
        else if (menuIdx == 1) calFeedback();
        else { appMode = APP_RUN_SETUP; menuEdit(); appMode = spinActive ? APP_RUNNING : APP_MENU; }
      } else if (k == KEY_BACK && safety.hasFault()) tryClearFault();
    }
    else if (appMode == APP_RUN_SETUP) { /* menuEdit owns keys */ }
    else {
      switch (k) {
        case KEY_ESTOP: safety.pressEstop(); stopSpin(); appMode = APP_MENU; break;
        case KEY_BACK:
          if (spinActive) { stopSpin(); appMode = APP_MENU; }
          else if (safety.hasFault()) tryClearFault();
          else if (appMode == APP_RUNNING) appMode = APP_MENU;
          else menuEdit();
          break;
        case KEY_RUN:
          if (!spinActive && !safety.hasFault()) startSpin();
          break;
        default: break;
      }
    }
  }

  if (safety.hasFault()) {
    stopSpin();
    if (appMode == APP_RUNNING) appMode = APP_MENU;
  } else if (spinActive) {
    uint32_t now = millis();
    uint32_t elapsed = now - spinStartMs;
    if (elapsed >= profileTotalMs(prof)) {
      stopSpin();
      appMode = APP_MENU;
      redrawNow = true;
    } else {
      // THE interlock: cap / current-derate / tach trips apply to the OUTPUT.
      int safeRpm = safety.update(rampRpm(elapsed, prof), measRpm, now, prof.maxRpmCap);
      if (safety.hasFault()) {
        stopSpin();
        appMode = APP_MENU;
        redrawNow = true;
      } else {
        cmdRpm = safeRpm;
        cmdUs = closedLoopUs(safeRpm, measRpm, now);   // table feedforward + trim
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
