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
| Tachometer (IR/hall) | GPIO34 + **external 10k pull-up to 3V3** | optional but recommended; set `USE_TACHO 1` |
| E-stop button | GPIO35 → GND + **external 10k pull-up to 3V3** | `USE_ESTOP_PIN 1` (default); set 0 if not fitted |
| Buzzer (active) | GPIO23 via NPN | optional |

**Common ground is mandatory** — SMPS GND, buck GND, ESC GND, ESP32 GND all tie together.

## Safety layers

1. ESC arming (3s at 1000µs) — motor can't spin during boot
2. Deadband floor: commands below `ESC_SPIN_US` (1090µs) never spin the motor
3. RPM caps: profile cap `RPM_SAFE_MAX` (8,000, chuck rating) is clamped at the
   output; a command above `RPM_ABSOLUTE_MAX` (10,000) is a hard fault.
   These are **open-loop estimates** (KV model) until a tach is fitted.
4. Software current model: derates above `EST_CURRENT_TRIP_A`. With the current
   constants the model peaks at 14.8 A, so this never engages — it is not a sensor.
5. Measured overspeed trip (tach): > cap + 500 RPM for 1 s → fault + stop
6. Tach-lost / stall trip (tach): commanding > 1,500 RPM with 0 RPM measured for 5 s → fault
7. E-stop (`*` key or GPIO35): latched fault, ESC cut to 1000µs (motor coasts —
   SimonK has no active brake by default)
8. Profile limits: `RAMP_MIN_S`..`RAMP_MAX_S`, `HOLD_MAX_S`, stored values re-validated at boot

Faults are latched. `D` clears a fault once the E-stop input is released and
(with a tach) the rotor has stopped.

## UI keys (4x4 membrane)

Main screen:
- `#` RUN profile (ignored while running or faulted)
- `D` edit profile (idle) / stop (running) / clear fault (fault screen)
- `*` E-STOP

Edit menu (Target RPM → Ramp up → Hold → Ramp down → Ease):
- `0-9` type a value, `#` delete a digit
- `A`/`B` step up/down (100 RPM, 1 s ramp, 5 s hold)
- `C` OK / next field (an out-of-range value is clamped and shown; `C` again accepts)
- `D` cancel the whole edit (nothing changed)
- Ease: `0-3` or `A`/`B`, then `C`

The profile is saved to flash (NVS) when the menu is confirmed and restored at boot.

## Serial telemetry

During a run, 115200 baud CSV at 10 Hz: `t_ms,cmd_rpm,esc_us,tach_rpm`
(`tach_rpm` = -1 when no tach). Use it to build an RPM-vs-µs calibration.

## Build & flash

```
pio run -e esp32dev
pio run -e esp32dev -t upload
pio test -e native          # ramp / safety unit tests (real headers)
```
