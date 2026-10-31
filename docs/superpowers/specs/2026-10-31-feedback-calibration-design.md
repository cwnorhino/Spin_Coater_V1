# Spin Coater V1 — Feedback + Calibration Design (Option A, approved 2026-10-31)

## Decision
Dense lookup table (12-pt, ~60s sweep) + feedforward + capped trim PI closed loop.
Reflective IR module (1 pulse/rev), powered 3V3, OUT->GPIO34, no 10k needed (module drives push-pull).

## Modules
- `calib.h`: `RpmCal{valid,n,us[12],rpm[12]}`, `FbCal{usMin,rpmMin,cap,fbMinRpm,fbMaxRpm,hasMin}`. NVS `cal`. Interp `rpmToUsCal()` lerp, clamp; invalid -> KV curve + UNCAL badge.
- `tacho`: ISR period + loop median-of-samples, timeout 1.5s (400rpm=150ms/rev), stability gate +-10/15%.
- `feedback.h`: base us from table + PI trim +-150us @5Hz, anti-windup, freeze on stale tach.
- `safety.h`: tach-lost 5s (>800rpm cmd) / 8s (low cmd, anti cog false-trip). Absolute 10k hard fault.
- `ui`: splash V1 contrast fade 1.2s -> main menu (1 CAL RPM [OK/---], 2 CAL FEEDBACK, 3 RUN) -> sub-screens. KEY_MAP = AGENTS.md rotated map. USE_TACHO=1, RPM_MIN=400.

## Cal 1: RPM map (~60s)
Menu 1, chuck-only, C start / D abort (cut 1000us). 12 pts us-spaced just-above-spin..~6000rpm, 5s each (2.5 settle + 2.5 avg). OLED `Pt i/12 us->rpm stable?` + bar. Need >=5 stable incl one <1000rpm else FAILED + manual fallback entry. Save NVS + Serial CSV.

## Cal 2: Feedback limits (no high test)
Menu 2. Min-detect: 1000us +10us/250ms until 3 reads agree +-15% >0 for ~1s + 1s sustain hold. Records usMin/rpmMin. Max fixed 8000 editable. Fail -> type fallback (prefill 500rpm/1150us). Floor: cmd<rpmMin snaps 0. Enables tach trips.

## Run (closed loop)
Setup: Target (400..cap), Up/Hold/Down, Ease on one scroll screen. RUN: phase+time-left, Set vs Tach, trim bar, ESC us @8Hz. D = ramp-down stop, * = E-STOP cut. Table feedforward + 5Hz PI trim.

## Tests
Native: existing ramp/safety + new table-interp/clamp/trim-cap tests. Bench: 60s CSV sane, 400rpm hold +-10%, abort cuts.
