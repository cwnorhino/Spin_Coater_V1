#pragma once
#include "config.h"

// Period-method tachometer. The ISR stores the time between valid pulses.
// A hold-off rejects comparator chatter: after a valid pulse, any edge sooner
// than 60% of the last period (clamped to [TACH_MIN_PERIOD_US, TACH_HOLDOFF_MAX_US])
// is ignored. The cap keeps it from locking out real pulses while accelerating.
static volatile uint32_t tachCount = 0, tachRejected = 0;
static volatile uint32_t tachLastUs = 0, tachPeriodUs = 0;
static float tachSmooth = 0;
static uint32_t tachSeenCount = 0;

static void IRAM_ATTR onTach() {
  uint32_t now = micros();
  uint32_t dt = now - tachLastUs;
  if (dt > TACH_TIMEOUT_US) {          // first pulse after standstill: no period yet
    tachPeriodUs = 0;
    tachLastUs = now;
    tachCount++;
    return;
  }
  uint32_t hold = tachPeriodUs ? (tachPeriodUs / 10) * 6 : TACH_HOLDOFF_MAX_US;
  if (hold > TACH_HOLDOFF_MAX_US) hold = TACH_HOLDOFF_MAX_US;
  if (hold < TACH_MIN_PERIOD_US) hold = TACH_MIN_PERIOD_US;
  if (dt < hold) { tachRejected++; return; }
  tachPeriodUs = dt;
  tachLastUs = now;
  tachCount++;
}

static void tachBegin() {
  pinMode(TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), onTach, FALLING);
}

static void tachReset() { tachSmooth = 0; }

// Updates rpm (smoothed, 0 = stopped). Returns true if a new pulse arrived.
static bool tachRead(int &rpm) {
  uint32_t cnt = tachCount;
  bool fresh = (cnt != tachSeenCount);
  tachSeenCount = cnt;
  uint32_t since = micros() - tachLastUs;
  uint32_t per = tachPeriodUs;
  if (since > TACH_TIMEOUT_US || per == 0 || per > TACH_TIMEOUT_US) {
    tachSmooth = 0;
  } else if (fresh) {
    float raw = 60000000.0f / ((float)per * TACH_PPR);
    tachSmooth = (tachSmooth <= 0) ? raw : 0.6f * tachSmooth + 0.4f * raw;
  }
  rpm = (int)tachSmooth;
  return fresh;
}