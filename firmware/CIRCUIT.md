# SpinCoater Circuit - Worded Build Guide

Build in this order. Do not power anything until the checks at the end.

Think of it as two separate worlds that only touch at one GND wire.
High current world with thick wires, and logic world on breadboard with thin wires.

## HIGH CURRENT WORLD - keep this OFF the breadboard

1. Wall mains into your 12V 15A SMPS. Connect Live, Neutral, Earth to the L, N and ground-symbol screws. Earth the metal case. Put a switch on the Live side if you can. Do this with mains unplugged, and get help if you have never wired mains before.

2. From SMPS output plus and minus, run two thick wires, red and black, 14 or 16 AWG, to the ESC thick red and black. Use bullet connectors or an XT60 in the middle so you can unplug. This path can pull 10 to 15 amps, breadboard will melt, so never route it through breadboard.

3. Solder a big electrolytic capacitor right at the ESC power input, 1000uF 25V or bigger. Plus leg to ESC red, minus leg to ESC black. This absorbs motor spikes and saves the ESC. Watch polarity, reversed will pop.

4. ESC three motor wires, usually blue yellow orange, to the three A2212 motor wires with bullet connectors. Any order works. If it spins backwards later, swap any two.

5. ESC thin servo lead has 3 small wires. Usually white or yellow is signal, black or brown is ground, red is 5V BEC output. Connect signal to ESP32 GPIO13. Connect black to ESP32 GND. Leave the red BEC wire taped and floating, do NOT connect it anywhere. Your 5V comes from the buck, and two 5V sources fighting will cause trouble.

## LOGIC POWER - this is what feeds the brain

6. Same SMPS 12V plus and minus also go to the input of your step-down buck module, VIN plus and VIN minus. Add a 1N4007 diode in series on the plus wire before the module, band facing toward the module, for reverse protection. Keep this pair twisted together.

7. Set the buck output to 5V BEFORE connecting ESP32. Power SMPS with nothing else on output, turn the trim pot, check with multimeter until you read 5.0V.

8. Buck output 5V plus goes to ESP32 VIN pin, also labeled 5V on some boards. Buck output minus goes to ESP32 GND. From that same point, run a second pair to OLED VCC and OLED GND. This is your logic 5V rail, safe to run on breadboard rails.

9. Tie all grounds together at one star point near the buck output minus. SMPS minus, buck minus, ESC thin black, ESP32 GND, OLED GND, keypad GND if it has one, future tach GND. One common ground is mandatory or I2C and ESC signal will float.

## LOGIC SIGNALS on breadboard

10. OLED 4 pin version. VCC to 5V as above, GND to GND rail, SDA to ESP32 GPIO21, SCL to ESP32 GPIO22. No extra resistors, the OLED board already has pullups.

11. Keypad membrane 8 pins. Four are rows, four are columns. Hold it with pins up, usually left 4 are rows, right 4 are cols, but verify with continuity beep mode, press a key and see which row touches which col. Then wire rows to GPIO14, 27, 26, 25 in order, cols to GPIO33, 32, 18, 19 in order. The firmware uses internal pullups so no resistors needed. Keep these as short Dupont jumpers.

12. Tachometer (reflective IR module, 3-pin VCC/GND/OUT). VCC to ESP32 3V3 (**not 5V**), GND to GND rail, OUT to GPIO23. Falling edge, 1 pulse/rev (white stripe on matte-black chuck, 5–15 mm gap, tune onboard pot). GPIO23 has an internal pull-up enabled in firmware — no external 10k needed (module drives push-pull).

13. E-stop button: NOT FITTED — `USE_ESTOP_PIN 0` in firmware (GPIO35 left unconnected; `*` key still triggers software E-stop). If you add a button later: normally-open push button, one leg to GPIO35, other to GND, plus 10k from GPIO35 to 3V3, then set `USE_ESTOP_PIN 1`.

14. Buzzer: not fitted — skipped.

15. Small 100nF ceramic capacitor across ESP32 VIN and GND right at the board pins, for stability when the motor kicks.

## USB RULE

When flashing from laptop over USB, unplug the buck 5V wire from ESP32 VIN. When running the motor from SMPS plus buck, unplug USB. Never have both 5V sources connected at once.

## POWER-ON CHECK SEQUENCE

With everything wired but SMPS unplugged, beep out with multimeter: no short between 12V plus and minus, no short between 5V and GND, capacitor polarity correct, BEC red wire not touching anything.

Then unplug ESC signal wire, power SMPS alone, verify 12V at ESC input and 5.0V at ESP32 VIN. Power off.

Reconnect signal, keep motor with no chuck load, power on. ESP32 boots, OLED shows SPIN COATER menu, ESC beeps its arm tones after 3 seconds at 1000us. Only then press # to run (D edits the profile when idle, stops a run, or clears a fault; * is E-STOP).

First spin at 2000 RPM max, no sample, hand on E-stop, check direction and vibration. Then go up slowly.

## What not to do

- Do not run motor power through breadboard rails.
- Do not connect ESC red BEC wire.
- Do not power OLED from 3V3 if it flickers, use 5V.
- Do not share USB and buck 5V.
- Do not skip the 1000uF cap or the ESC will see voltage spikes and reset.

That is the whole loop. SMPS feeds ESC directly for muscle and feeds buck for brains, ESP32 talks one-way PWM to ESC on GPIO13, reads buttons and drives OLED on I2C, and later the tach on GPIO34 closes the loop for real RPM protection.
