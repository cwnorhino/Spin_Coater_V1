# SpinCoater v1 — ESP32 Spin Coater Controller

Full controller for a brushless spin-coater: A2212 1400KV motor + SimonK 30A ESC
driven by an ESP32 DevKit (CP2102), with a 0.96" SSD1306 OLED UI, 4x4 keypad,
tachometer feedback and layered software safety.

## BOM wiring map

| Part | Connects to | Notes |
|---|---|---|
| 12V 15A SMPS | ESC V+/V-, common GND with ESP32 via buck | Main motor rail |
| Step-down 12V→5V 5A | ESP32 5V pin, OLED VCC | Logic rail |
| A2212 1400KV motor | ESC 3 motor wires (any order; swap 2 to reverse) | |
| SimonK 30A ESC | signal → GPIO13, power → 12V | |
| SSD1306 OLED (I2C) | SDA→21, SCL→22, VCC→5V, GND | addr 0x3C |
| 4x4 keypad | rows→14,27,26,25 cols→33,32,18,19 | |
| Tachometer (IR/hall) | GPIO34 (INPUT_PULLUP, add 10k ext pull if noisy) | optional but recommended |
| E-stop button | GPIO35 → GND | optional |
| Buzzer (active) | GPIO23 via NPN | optional |

**Common ground is mandatory** — SMPS GND, buck GND, ESC GND, ESP32 GND all tie together.

## Safety layers

1. ESC arming (3s at 1000µs) — motor can't spin during boot
2. Deadband floor: commands below `ESC_SPIN_US` (1090µs) never spin the motor
3. Absolute firmware ceiling: 10,000 RPM (never commanded above)
4. Software current model: auto-derates throttle above ~18A estimated
5. Measured overspeed trip: tach >10,300 RPM for 1s → fault + stop
6. Tach-loss trip: commanding >1,500 RPM with zero pulses for 5s → fault
7. Hardware E-stop pin: immediate cut to 1000µs
8. Deadman on profile times: `RAMP_MIN_S`..`RAMP_MAX_S`, `HOLD_MAX_S` caps

## UI keys (4x4 membrane)

- `A/B` RPM up/down (long-press style: repeated)
- `C` enter edit menu (RPM → Up → Hold → Down → Ease)
- `#` RUN
- `*` E-STOP / STOP
- `D` BACK / abort
- `0-3` select ease curve in menu (Linear/EaseIn/EaseOut/S-Curve)

## Build & flash

```
pio run -e esp32dev
pio run -e esp32dev -t upload
```
