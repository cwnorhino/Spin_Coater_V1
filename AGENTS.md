# Spin Coater V1 — Agent Notes

## Keypad mapping (4x4 membrane, pins 1-8)
Wiring is straight-through: rows 1-4 -> GPIO14,27,26,25, cols 5-8 ->
GPIO33,32,18,19. Verified 2026-10-31 via on-device KEY TEST (physical
1/2/3/A arrive as electrical row 0 in order; with the old rotated map,
physical B/C/D arrived as digits 3/2/1). Use the naive label-order map:

```cpp
static const byte KEY_MAP[KEY_ROWS][KEY_COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
```

The old rotated map (`D,C,B,A / #,9,6,3 / ...`) is WRONG for this unit —
do not restore it.
Pins must stay as above (GPIO12/13 are strapping/ESC, cannot be used).
