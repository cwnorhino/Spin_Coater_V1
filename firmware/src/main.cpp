#include <Arduino.h>
#include <Wire.h>
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
static volatile long tachCount = 0;
static int lastRpmEst = 0;
static bool spinActive = false;
static uint32_t spinStartMs = 0;
static uint32_t lastTachCheckMs = 0;

// ---- Hardware setup ----
void setupESC() {
  esc.setPeriodHertz(50);           // standard ESC 50Hz
  esc.attach(ESC_PIN, ESC_MIN_US, ESC_MAX_US);
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

// Tach ISR — wire a photointerrupter / hall +IRLED to TACH_PIN.
void IRAM_ATTR onTach() { tachCount++; }

void setupTacho() {
  pinMode(TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), onTach, FALLING);
}

void setupEstop() {
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  if (!digitalRead(ESTOP_PIN)) safety.pressEstop();
}

// ---- RPM feedback ----
// (tachometer RPM is computed in loop() over a 100ms window)

// ---- Main spin state machine ----
static void stopSpin() {
  esc.writeMicroseconds(ESC_MIN_US);
  spinActive = false;
  lastRpmEst = 0;
}

// Returns the SAFE-CLAMPED throttle us for the profile at elapsed time.
static int computeTargetUs(uint32_t elapsedMs, const SpinProfile &p) {
  uint32_t upMs   = (uint32_t)p.rampUpS   * 1000;
  uint32_t holdMs = (uint32_t)p.holdS     * 1000;
  uint32_t dnMs   = (uint32_t)p.rampDnS   * 1000;
  uint32_t total  = upMs + holdMs + dnMs;

  int cmdRpm;
  if (elapsedMs < upMs) {
    float f = ease01((float)elapsedMs / (float)upMs, p.ease);
    cmdRpm = (int)(p.targetRpm * f);
  } else if (elapsedMs < upMs + holdMs) {
    cmdRpm = p.targetRpm;
  } else if (elapsedMs < total) {
    float d = (float)(elapsedMs - upMs - holdMs) / (float)dnMs;
    cmdRpm = (int)(p.targetRpm * (1.0f - ease01(d, p.ease)));
  } else {
    if (spinActive) {
      stopSpin();
      safety.update(0, lastRpmEst, millis());
    }
    return ESC_MIN_US;
  }

  // THE interlock: current-derate / over-rpm clamp applies to the OUTPUT.
  int safeRpm = safety.update(cmdRpm, lastRpmEst, millis());
  return rpmToUs(safeRpm);
}

// ---- UI ----
static void drawScreen() {
  display.clearDisplay();
  display.setCursor(0, 0);
  if (safety.hasFault()) {
    display.print("FAULT: "); display.println(faultName(safety.s.fault));
    display.println("Press D to clear");
  } else if (!spinActive) {
    display.println("SPIN COATER v1.0");
    display.println();
    display.print("RPM  : ");   display.println(prof.targetRpm);
    display.print("Up   : ");   display.print(prof.rampUpS); display.println("s");
    display.print("Hold : ");   display.print(prof.holdS); display.println("s");
    display.print("Down : ");   display.print(prof.rampDnS); display.println("s");
    display.print("Ease : ");   display.println(easeName(prof.ease));
    display.print("Cap  : ");   display.print(prof.maxRpmCap); display.println(" RPM");
    display.println("A/B:RPM +/-, C:RUN, D:menu exit");
    display.println("*:E-STOP, #:fine");
  } else {
    display.println("SPINNING");
    display.print("Set  : "); display.println(prof.targetRpm);
    display.print("Est  : "); display.println(lastRpmEst);
    display.print("Thr% : "); display.println((int)(rpmToThr01(prof.targetRpm)*100));
    if (safety.s.currentLimited) display.println("CUR LIM");
    display.println("*:STOP  D:abort");
  }
  display.display();
}

// ---- Edit-number helper ----
static int editNumber(int &val, int lo, int hi, const char* lbl) {
  display.clearDisplay(); display.setCursor(0,0);
  display.print(lbl); display.print(" "); display.println(val);
  display.println("A:+1  B:-1  C:+10  D:-10  #:fine");
  display.display();
  while (true) {
    char k = keys.getKey();
    if (k == NO_KEY) { delay(20); continue; }
    switch (k) {
      case KEY_UP:   val = min(hi, val+1); break;
      case KEY_DOWN: val = max(lo, val-1); break;
      case KEY_ESTOP: safety.pressEstop(); return val;
      case KEY_OK: return val;
      case KEY_BACK: return val;
      default: break;
    }
    display.clearDisplay(); display.setCursor(0,0);
    display.print(lbl); display.print(" "); display.println(val);
    display.display();
  }
}

static void menuEdit() {
  if (safety.hasFault()) { keys.waitForKey(); return; }
  display.clearDisplay(); display.setCursor(0,0);
  display.println("MENU 1/5"); display.println("C=RUN  D=BACK"); display.display(); keys.waitForKey();
  editNumber(prof.targetRpm, RPM_MIN, prof.maxRpmCap, "RPM");
  editNumber(prof.rampUpS, RAMP_MIN_S, RAMP_MAX_S, "UP s");
  editNumber(prof.holdS,   0, HOLD_MAX_S, "HOLD s");
  editNumber(prof.rampDnS, RAMP_MIN_S, RAMP_MAX_S, "DOWN s");
  display.clearDisplay(); display.setCursor(0,0);
  display.println("EASE 0-Lin 1-In 2-Out 3-S"); display.display();
  while (true) {
    char k = keys.getKey(); if (k==NO_KEY){delay(20);continue;}
    if (k=='0'){prof.ease=0;} else if (k=='1'){prof.ease=1;}
    else if (k=='2'){prof.ease=2;} else if (k=='3'){prof.ease=3;}
    else if (k==KEY_BACK||k==KEY_OK) break;
    display.clearDisplay(); display.setCursor(0,0);
    display.print("Ease "); display.println(easeName(prof.ease)); display.display();
  }
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  setupDisplay();
  setupESC();
  setupTacho();
  setupEstop();
  display.println("Arming ESC..."); display.display();
  delay(500);
  keys.addEventListener([](char e){ if(e==KEY_ESTOP) safety.pressEstop(); });
}

// ---- Loop ----
void loop() {
  // estop button polling (pin)
  if (!digitalRead(ESTOP_PIN)) safety.pressEstop();

  char k = keys.getKey();
  if (k != NO_KEY) {
    switch (k) {
      case KEY_BACK:   if (spinActive) { stopSpin(); } else { menuEdit(); } break;
      case KEY_RUN:    if (!safety.hasFault()) { spinActive=true; spinStartMs=millis(); } break;
      case KEY_ESTOP:  safety.pressEstop(); stopSpin(); break;
      case KEY_OK:     if (!spinActive) drawScreen(); break;
      default:         break;
    }
  }

  if (safety.hasFault()) {
    stopSpin();
    drawScreen();
    return;
  }

  if (spinActive) {
    uint32_t elapsed = millis() - spinStartMs;
    int us = computeTargetUs(elapsed, prof);
    esc.writeMicroseconds(constrain(us, ESC_MIN_US, ESC_SAFE_US));

    // RPM via tach (100ms window)
    static uint32_t lastMs = 0; static long lastCount = 0;
    uint32_t now = millis();
    if (now - lastMs >= 100) {
      long c; noInterrupts(); c = tachCount; interrupts();
      long delta = c - lastCount; lastCount = c; lastMs = now;
      // pulses in 100ms * 10 (1 pulse/rev) = RPM
      lastRpmEst = delta * 10;
      if (!safety.s.tachoPresent && lastRpmEst > 50) safety.s.tachoPresent = true;
      lastTachCheckMs = now;
    }
  }

  // Throttle OLED redraw to ~8 Hz (I2C frame is ~25ms; every-loop is wasteful)
  static uint32_t lastDraw = 0;
  uint32_t nowD = millis();
  if (nowD - lastDraw >= 120) { drawScreen(); lastDraw = nowD; }
}
