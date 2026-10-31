# Spin Coater V1 — Agent Notes

## Keypad mapping (4x4 membrane, pins 1-8)
Physical keypad is rotated 180° relative to label order. With row pins
GPIO14,27,26,25 and col pins GPIO33,32,18,19, use this KEY_MAP:

```cpp
static const byte KEY_MAP[KEY_ROWS][KEY_COLS] = {
  {'D','C','B','A'},
  {'#','9','6','3'},
  {'0','8','5','2'},
  {'*','7','4','1'}
};
```

Do NOT revert to the naive label-order map — it prints diagonally wrong.
Pins must stay as above (GPIO12/13 are strapping/ESC, cannot be used).
