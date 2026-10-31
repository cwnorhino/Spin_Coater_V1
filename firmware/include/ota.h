#pragma once
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <Adafruit_SSD1306.h>
#include <Keypad.h>
#include "config.h"
#include "secrets.h"

// Self-update: join the WiFi hotspot, download firmware.bin from the latest
// GitHub release, flash it, reboot. Call only while the motor is stopped.

static Adafruit_SSD1306* otaDisp = nullptr;
static int otaLastPct = -1;

static void otaShow(const String &a, const String &b = "", const String &c = "", const String &d = "") {
  otaDisp->clearDisplay();
  otaDisp->setTextSize(1);
  otaDisp->setCursor(0, 0);
  otaDisp->println(a);
  otaDisp->println(b);
  otaDisp->println(c);
  otaDisp->println(d);
  otaDisp->display();
}

static void otaProgress(int cur, int total) {
  if (total <= 0) return;
  int pct = (int)(100L * cur / total);
  if (pct == otaLastPct) return;     // redraw only when % changes
  otaLastPct = pct;
  String bar;
  int bars = pct * 21 / 100;
  for (int i = 0; i < 21; i++) bar += (i < bars ? '#' : '-');
  otaShow("Downloading...", String(pct) + "% " + String(cur / 1024) + "/" + String(total / 1024) + " KB", bar);
}

static void otaWaitKey(Keypad &keys) {
  while (keys.getKey() == NO_KEY) delay(20);
}

static void otaUpdate(Adafruit_SSD1306 &disp, Keypad &keys) {
  otaDisp = &disp;
  otaShow("UPDATE FIRMWARE", String("current ") + FW_VERSION, String("wifi: ") + WIFI_SSID, "#:go   *:cancel");
  while (true) {
    char k = keys.getKey();
    if (k == '*' || k == 'D') return;
    if (k == '#') break;
    delay(10);
  }

  otaShow("Connecting...", WIFI_SSID, "*:cancel");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    if (keys.getKey() == '*') { WiFi.mode(WIFI_OFF); return; }
    delay(100);
  }
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_OFF);
    otaShow("WiFi FAILED", "check hotspot is on", "2.4GHz + password", "any key: back");
    otaWaitKey(keys);
    return;
  }

  otaShow("Connected", "Fetching bin...");
  Serial.printf("OTA start, current %s\n", FW_VERSION);
  WiFiClientSecure client;
  client.setInsecure();              // GitHub cert not pinned
  client.setTimeout(15000);
  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(false);
  otaLastPct = -1;
  httpUpdate.onProgress(otaProgress);
  t_httpUpdate_return ret = httpUpdate.update(client, OTA_FW_URL, FW_VERSION);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  if (ret == HTTP_UPDATE_OK) {
    otaShow("UPDATE OK", "rebooting...");
    Serial.println("OTA OK, rebooting");
    delay(1500);
    ESP.restart();
  } else {
    otaShow("UPDATE FAILED", httpUpdate.getLastErrorString(), "err " + String(httpUpdate.getLastError()), "any key: back");
    Serial.printf("OTA FAIL %d %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
    otaWaitKey(keys);
  }
}