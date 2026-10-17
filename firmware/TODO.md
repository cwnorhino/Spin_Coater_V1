# SpinCoater TODO — Future (do NOT implement yet)

Status: parked on 2026-10-17 per Debyte request. Firmware v1 stays as-is (OLED + keypad only).

## 1. Bluetooth live graph
- Goal: stream live RPM (commanded vs estimated/measured) to phone over Bluetooth for tuning ease curves.
- Decide: BLE (NimBLE, works alongside WiFi) vs Classic Bluetooth Serial (simpler, but kills WiFi AP option on ESP32).
- Recommended: BLE with Nordic UART Service (NUS) + notify at 10 Hz.
- Data packet: elapsed_ms, cmd_rpm, safe_rpm, est_rpm, throttle_us, fault_code, current_est_A.
- Phone side: nRF Connect / custom app, or reuse the WiFi HTML page via BLE-WebBluetooth (Chrome Android).
- Safety: Bluetooth is telemetry + setpoint preview ONLY. Never allow raw throttle_us writes. All setpoints go through safety.update().
- Tests: native test for packet encode/decode; bench test with nRF Connect showing 10 Hz stream during a 30 s spin.

## 2. WiFi hotspot "spincoater-wax" + beautiful HTML dashboard
- Goal: ESP32 starts an open (or WPA2: password TBD) AP named exactly spincoater-wax. User joins from phone, opens http://192.168.4.1 and sees a full dashboard.
- Stack: WiFi AP mode (fixed IP 192.168.4.1) + ESPAsyncWebServer or Arduino WebServer + Server-Sent Events or WebSocket at 10 Hz.
- Dashboard (single page, no internet — all CSS/JS inlined or in SPIFFS/LittleFS):
  - Live chart: commanded RPM vs measured RPM vs throttle %, scrolling 60 s window.
  - Ease curve preview: small static graph of selected Linear / EaseIn / EaseOut / S-Curve.
  - Controls mirror: target RPM, ramp-up, hold, ramp-down, ease select, RUN / STOP / CLEAR FAULT buttons.
  - Safety panel: fault code, current-limit flag, est current A, tach status, RPM cap.
  - Dark lab theme, big touch buttons, works on small phone screen.
- Performance: page load < 2 s on phone, telemetry 10 Hz, OLED redraw stays at 8 Hz, no blocking in loop().
- Captive portal (optional): DNS redirect so any URL opens the dashboard.
- Safety: same rule as Bluetooth — UI sends profile params only, firmware clamps via Safety class. E-STOP button = highest priority, works even if chart socket drops.
- Storage: move HTML into LittleFS file(s) when it grows past ~30 KB; keep v1 inline fallback.
- Tests: bench test with phone connected to spincoater-wax, run full profile, confirm chart matches OLED values within 200 ms.

## Order when we do this
1. Tachometer wired first (else measured-RPM line on graph is flat).
2. Then WiFi AP + HTML (bigger payoff than Bluetooth alone).
3. Then BLE (only if WiFi AP + OLED is not enough).
4. Optional: log runs to CSV on phone / SPIFFS download.

## Notes
- ESP32 radio can do WiFi AP + BLE together with tuning, but NOT Classic BT + WiFi reliably. If both wireless features are wanted, pick BLE, not Classic.
- Power: WiFi AP adds ~80-100 mA on the 5 V rail — the 5 A buck has headroom, no change needed.
