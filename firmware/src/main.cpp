// Spin coater firmware
// ESP32 + A2212 1400KV + SimonK 30A ESC + reflective IR tach + OLED + 4x4 keypad
//
// Idle:    0-9 type RPM   A/B +-100   # RUN   C raw throttle test   D update firmware   * STOP
// Running: A/B change RPM live   digits + # set new RPM   D or * stop
// Raw test: A/B +-10us   C/D +-1us   0 = 1000us   * exit
//
// SEEK creeps the pulse width up from just above the ESC deadband until the
// tach sees the rotor turning. RUN then slews the setpoint to the target and
// nudges the pulse width each 100ms from the tach error.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <Keypad.h>
#include "config.h"
#include "tach.h"
#include "ota.h"

// Straight label-order map (physical 1/2/3/A on the top row).
static const byte KEY_MAP[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
static byte rowPins[4] = { KEY_ROW_PINS[0], KEY_ROW_PINS[1], KEY_ROW_PINS[2], KEY_ROW_PINS[3] };
static byte colPins[4] = { KEY_COL_PINS[0], KEY_COL_PINS[1], KEY_COL_PINS[2], KEY_COL_PINS[3] };
static Keypad keys = Keypad(makeKeymap(KEY_MAP), rowPins, colPins, 4, 4);

static Adafruit_SSD1306 display(128, 64, &Wire, -1);
static Servo esc;

enum State : uint8_t { S_IDLE, S_SEEK, S_RUN, S_TEST, S_FAULT };
static State state = S_IDLE;
static const char* faultMsg = "";

static int   target = 1000;      // typed / displayed target
static int   runTarget = 1000;   // target the controller uses
static bool  typing = false;
static float sp = 0;             // slewed setpoint
static float us = ESC_MIN_US;    // current throttle pulse
static int   rpm = 0;            // smoothed tach rpm
static uint32_t lastGoodMs = 0, runStartMs = 0;

static void writeUs(float v, float hi = ESC_SAFE_US) {
  if (v < ESC_MIN_US) v = ESC_MIN_US;
  if (v > hi) v = hi;
  us = v;
  esc.writeMicroseconds((int)v);
}

static void stopMotor() {
  writeUs(ESC_MIN_US);
  sp = 0;
  state = S_IDLE;
}

static void fault(const char* msg) {
  writeUs(ESC_MIN_US);
  sp = 0;
  faultMsg = msg;
  state = S_FAULT;
}

static void startRun() {
  runTarget = constrain(target, RPM_MIN, RPM_MAX);
  target = runTarget;
  tachReset();
  rpm = 0; sp = 0;
  writeUs(SEEK_START_US);
  runStartMs = lastGoodMs = millis();
  state = S_SEEK;
  Serial.println("t_ms,target,setpoint,rpm,us");
}

// ---- control tick (every TICK_MS) ----
static void control(uint32_t now) {
  bool fresh = tachRead(rpm);

  if (state == S_SEEK) {
    if (rpm > 0) {                       // rotor turning -> hand over to the loop
      sp = rpm;
      lastGoodMs = now;
      state = S_RUN;
    } else {
      writeUs(us + SEEK_STEP_US);
      if (us > SEEK_MAX_US) fault("NO SPIN (tach/ESC?)");
    }
  } else if (state == S_RUN) {
    if (rpm > 0) lastGoodMs = now;
    if (now - lastGoodMs > STALL_MS) { fault("STALL / TACH LOST"); return; }
    if (rpm > RPM_MAX + 500) { fault("OVERSPEED"); return; }

    float maxd = SLEW_RPM_S * TICK_MS / 1000.0f;
    float d = (float)runTarget - sp;
    if (d > maxd) d = maxd;
    if (d < -maxd) d = -maxd;
    sp += d;

    if (fresh && rpm > 0) {              // only act on fresh tach data
      float step = KP_US_PER_RPM * (sp - (float)rpm);
      if (step > STEP_MAX_US) step = STEP_MAX_US;
      if (step < -STEP_MAX_US) step = -STEP_MAX_US;
      writeUs(us + step);
    }
  }

  if (state == S_SEEK || state == S_RUN) {
    Serial.printf("%lu,%d,%d,%d,%d\n", (unsigned long)(now - runStartMs),
                  runTarget, (int)sp, rpm, (int)us);
  }
}

// ---- keys ----
static void handleKey(char k) {
  if (k == '*') {                        // STOP always works
    if (state != S_FAULT) stopMotor();
    return;
  }

  if (state == S_FAULT) {
    if ((k == 'D' || k == '#') && rpm == 0) state = S_IDLE;
    return;
  }

  if (state == S_TEST) {
    if (k == 'A') writeUs(us + 10, TEST_MAX_US);
    else if (k == 'B') writeUs(us - 10, TEST_MAX_US);
    else if (k == 'C') writeUs(us + 1, TEST_MAX_US);
    else if (k == 'D') writeUs(us - 1, TEST_MAX_US);
    else if (k == '0') writeUs(ESC_MIN_US);
    return;
  }

  if (k >= '0' && k <= '9') {
    if (!typing) { target = 0; typing = true; }
    if (target < 10000) target = target * 10 + (k - '0');
  } else if (k == 'A' || k == 'B') {
    typing = false;
    target = constrain(target + (k == 'A' ? 100 : -100), RPM_MIN, RPM_MAX);
    if (state == S_RUN || state == S_SEEK) runTarget = target;
  } else if (k == '#') {
    typing = false;
    target = constrain(target, RPM_MIN, RPM_MAX);
    if (state == S_IDLE) startRun();
    else runTarget = target;
  } else if (k == 'C' && state == S_IDLE) {
    writeUs(ESC_MIN_US);
    state = S_TEST;
  } else if (k == 'D') {
    if (state == S_IDLE) otaUpdate(display, keys);   // motor is off here
    else stopMotor();
  }
}

// ---- display ----
static void draw() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  switch (state) {
    case S_IDLE:
      display.printf("SPIN COATER %s\n", FW_VERSION);
      display.printf("Set : %d rpm%s\n", target, typing ? "_" : "");
      display.printf("Tach: %d rpm\n", rpm);
      display.printf("Range %d-%d\n", RPM_MIN, RPM_MAX);
      display.println();
      display.println("0-9 type  A/B +-100");
      display.println("#:RUN C:test D:upd");
      display.println("*:STOP");
      break;
    case S_SEEK:
    case S_RUN:
      display.println(state == S_SEEK ? "STARTING..." : "RUNNING");
      display.printf("Set : %d rpm\n", runTarget);
      display.printf("Ramp: %d rpm\n", (int)sp);
      display.printf("Tach: %d rpm\n", rpm);
      display.printf("ESC : %d us\n", (int)us);
      display.println();
      display.println("A/B:+-100 #:apply");
      display.println("D/*:STOP");
      break;
    case S_TEST:
      display.println("RAW THROTTLE TEST");
      display.setTextSize(2);
      display.printf("%d us\n", (int)us);
      display.setTextSize(1);
      display.printf("Tach: %d rpm\n", rpm);
      display.printf("Pulses %lu Rej %lu\n", (unsigned long)tachCount, (unsigned long)tachRejected);
      display.println("A/B +-10  C/D +-1");
      display.println("0:1000us  *:exit");
      break;
    case S_FAULT:
      display.println("!! FAULT !!");
      display.println(faultMsg);
      display.println();
      display.println("Motor output cut.");
      display.println(rpm > 0 ? "Wait: rotor turning" : "D/#: clear");
      break;
  }
  display.display();
}

void setup() {
  Serial.begin(115200);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    pinMode(2, OUTPUT);
    while (1) { digitalWrite(2, millis() & 0x200); delay(200); }
  }
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Arming ESC...");
  display.display();

  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, ESC_MIN_US, ESC_MAX_US);
  esc.setTimerWidth(16);               // must follow attach(): attach resets width
  esc.writeMicroseconds(ESC_MIN_US);
  delay(ESC_ARM_MS);

  tachBegin();
  Serial.printf("Spin coater %s ready\n", FW_VERSION);
}

void loop() {
  char k = keys.getKey();
  if (k != NO_KEY) handleKey(k);

  static uint32_t lastTick = 0, lastDraw = 0;
  uint32_t now = millis();
  if (now - lastTick >= TICK_MS) {
    lastTick = now;
    if (state == S_SEEK || state == S_RUN) control(now);
    else tachRead(rpm);
  }
  if (now - lastDraw >= 120) {
    lastDraw = now;
    draw();
  }
}