#include <Arduino.h>
#include <ESP32Servo.h>

#define ESC_PIN 23
Servo esc;

void setup() {
  Serial.begin(115200);
  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, 1000, 2000);
  esc.writeMicroseconds(1000);
  Serial.println("ESC test loaded (armed at 1000us)");
}

void loop() {
  // Sweep 1000-1200us slowly
  for (int us = 1000; us <= 1200; us += 5) {
    esc.writeMicroseconds(us);
    Serial.printf("us=%d\n", us);
    delay(200);
  }
  delay(2000);
}
