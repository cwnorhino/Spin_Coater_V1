#include <Arduino.h>

// Simple IR receiver test: print raw pulse timings on GPIO 34
#define IR_PIN 23

void setup() {
  Serial.begin(115200);
  pinMode(IR_PIN, INPUT);
  Serial.println("IR test loaded (GPIO23)");
}

void loop() {
  Serial.println(digitalRead(IR_PIN));
  delay(50);
}
