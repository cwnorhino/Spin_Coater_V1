#include <Arduino.h>
#include <Keypad.h>

#define ROWS 4
#define COLS 4
char keys[ROWS][COLS] = {
  {'D','C','B','A'},
  {'#','9','6','3'},
  {'0','8','5','2'},
  {'*','7','4','1'}
};
byte rowPins[ROWS] = {14, 27, 26, 25};
byte colPins[COLS] = {33, 32, 18, 19};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Numpad test loaded");
}

void loop() {
  char k = keypad.getKey();
  if (k != NO_KEY) {
    Serial.print("Pressed: ");
    Serial.println(k);
  }
  delay(10);
}
