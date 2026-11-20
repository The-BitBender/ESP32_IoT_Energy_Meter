#pragma once
// =============================================================================
//  Ui.h - 128x64 OLED user interface (u8g2, full-buffer, hardware I2C)
//  All icons are drawn with primitives, so there are no bitmap tables to maintain.
// =============================================================================
#include <U8g2lib.h>
#include <Wire.h>
#include "config.h"
#include "Button.h"
#include "NetLink.h"

#define OLED_IS_SH1106 1      // many "1.3 inch SSD1306" modules are really SH1106: set 1 if the picture is shifted 2px / has garbage on the edge

#if OLED_IS_SH1106
static U8G2_SH1106_128X64_NONAME_F_HW_I2C  u8g2(U8G2_R0, U8X8_PIN_NONE);
#else
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
#endif

#define F_TITLE u8g2_font_helvB08_tr
#define F_TXT   u8g2_font_6x10_tr
#define F_TINY  u8g2_font_5x7_tr
#define F_BIG   u8g2_font_logisoso16_tr
#define F_HUGE  u8g2_font_logisoso22_tr

enum Page : uint8_t { PAGE_ENERGY = 0, PAGE_POWER, PAGE_SUPPLY, PAGE_ONLINE, PAGE_ALERT, PAGE_COUNT };

class Ui {
 public:
  void begin() {
    u8g2.begin();
    u8g2.setBusClock(400000);
  }

  // ---------------------------------------------------------------- splash
  void splash() {
    const uint32_t t0 = millis();
    uint32_t el;
    while ((el = millis() - t0) < 1800) {
      u8g2.clearBuffer();
      bolt(8, 10, 34);
      u8g2.setFont(F_TITLE);
      u8g2.drawStr(46, 24, "Smart");
      u8g2.drawStr(46, 36, "Energy Meter");
      u8g2.setFont(F_TINY);
      u8g2.drawStr(46, 47, DEVICE_ID);
      u8g2.drawRFrame(4, 54, 120, 8, 3);
      u8g2.drawBox(7, 57, (int)((114UL * el) / 1800UL), 2);
      u8g2.sendBuffer();
      delay(30);
    }
  }

  // ------------------------------------------------- boot: pick the gateway
  // tick() is called every loop so the meter keeps being serviced meanwhile.
  Gateway selectGateway(Button& btn, Gateway preset, void (*tick)()) {
    int sel = (int)preset;
    uint32_t t0 = millis(), lastDraw = 0;
    for (;;) {
      if (tick) tick();
      BtnEvent e = btn.poll();
      if (e == BtnEvent::Next)   { sel ^= 1; t0 = millis(); }
      if (e == BtnEvent::Select) return (Gateway)sel;

      uint32_t el = millis() - t0;
      if (BOOT_SELECT_TIMEOUT_MS && el >= BOOT_SELECT_TIMEOUT_MS) return (Gateway)sel;

      if (millis() - lastDraw < 50) continue;
      lastDraw = millis();

      u8g2.clearBuffer();
      u8g2.setFont(F_TITLE);
      u8g2.drawStr(2, 10, "Internet Gateway");
      if (BOOT_SELECT_TIMEOUT_MS) {
        char b[8];
        snprintf(b, sizeof b, "%us", (unsigned)((BOOT_SELECT_TIMEOUT_MS - el + 999) / 1000));
        u8g2.setFont(F_TINY);
        textR(126, 9, b);
      }
      u8g2.drawHLine(0, 13, 128);

      gatewayCard(3,  16, sel == 0, 0);
      gatewayCard(65, 16, sel == 1, 1);

      if (BOOT_SELECT_TIMEOUT_MS) {
        u8g2.drawBox(3, 53, (int)(122UL * (BOOT_SELECT_TIMEOUT_MS - el) / BOOT_SELECT_TIMEOUT_MS), 2);
      }
      u8g2.setFont(F_TINY);
      textC(64, 63, "Click=next  Hold=select");
      u8g2.sendBuffer();
    }
  }

  // ------------------------------------------------------------ main frame
  // Call every loop(). Handles the button, then redraws at ~15 fps.
  void frame(BtnEvent ev, const NetState& ns, const EnergyReadings& r, bool calibrating) {
    const bool overlay = overlayActive(ns);

    // ---- input ----
    if (_menu) {
      menuEvent(ev, ns);
    } else if (ev == BtnEvent::Select) {
      _menu = true; _menuIdx = 0;
    } else if (ev == BtnEvent::Next && !overlay) {
      _page = (_page + 1) % PAGE_COUNT;
    }

#if AUTO_JUMP_ON_ANOMALY
    if (ns.anomalyCount > 0 && _prevAnom == 0 && !_menu && !overlay) _page = PAGE_ALERT;
#endif
    _prevAnom = ns.anomalyCount;

    if (millis() - _lastDraw < 66) return;
    _lastDraw = millis();

    // ---- draw ----
    u8g2.clearBuffer();
    if (calibrating)      drawCalibration();
    else if (_menu)       drawMenu(ns);
    else if (overlay)     drawOverlay(ns);
    else {
      switch (_page) {
        case PAGE_ENERGY: statusBar(ns, "Energy"); pageEnergy(ns, r); break;
        case PAGE_POWER:  statusBar(ns, "Power");  pagePower(r);      break;
        case PAGE_SUPPLY: statusBar(ns, "Supply"); pageSupply(r);     break;
        case PAGE_ONLINE: statusBar(ns, "Online"); pageOnline(ns);    break;
        default:          statusBar(ns, "Alerts"); pageAlert(ns);     break;
      }
      pageDots();
    }
    u8g2.sendBuffer();
  }

 private:
  uint8_t  _page = PAGE_ENERGY;
  bool     _menu = false;
  uint8_t  _menuIdx = 0;
  uint8_t  _prevAnom = 0;
  uint32_t _lastDraw = 0;

  // ====================================================================
  //  small helpers
  // ====================================================================
  static void textR(int xr, int y, const char* s) { u8g2.drawStr(xr - u8g2.getStrWidth(s), y, s); }
  static void textC(int xc, int y, const char* s)  { u8g2.drawStr(xc - u8g2.getStrWidth(s) / 2, y, s); }

  // text that scrolls sideways when wider than w
  static void marquee(int x, int y, int w, const char* s) {
    int tw = u8g2.getStrWidth(s);
    u8g2.setClipWindow(x, max(0, y - 10), x + w, y + 3);
    if (tw <= w) {
      u8g2.drawStr(x, y, s);
    } else {
      const int gap = 24;
      int off = (millis() / 35) % (tw + gap);
      u8g2.drawStr(x - off, y, s);
      u8g2.drawStr(x - off + tw + gap, y, s);
    }
    u8g2.setMaxClipWindow();
  }

  static void fmtAge(char* b, size_t n, uint32_t at) {
    if (at == 0) { snprintf(b, n, "--"); return; }
    uint32_t s = (millis() - at) / 1000;
    if (s < 60)        snprintf(b, n, "%us ago", (unsigned)s);
    else if (s < 3600) snprintf(b, n, "%um ago", (unsigned)(s / 60));
    else               snprintf(b, n, "%uh ago", (unsigned)(s / 3600));
  }

  static const char* anomalyName(const char* t) {
    static const struct { const char* k; const char* v; } map[] = {
      {"CURRENT_SHUNT_THEFT",   "Current shunt"},
      {"UNMETERED_BYPASS",      "Meter bypass"},
      {"REVERSE_CURRENT",       "Reverse current"},
      {"FREQUENCY_INSTABILITY", "Freq unstable"},
      {"VOLTAGE_DROP_HIGH_DRAW","V-drop, high draw"},
      {"SURGE_SPIKE",           "Surge spike"},
      {"UNDERVOLTAGE",          "Undervoltage"},
      {"OVERVOLTAGE",           "Overvoltage"},
      {"PEAK_POWER_EXCEEDED",    "Peak power exceeded"},
      {"POWER_FACTOR_POOR",     "Poor power factor"},
      {"RELAY_FAILURE",         "Relay failure"},
      {"FREQUENCY_INSTABILITY", "Freq unstable"}
    };
    for (auto& m : map) if (!strcmp(t, m.k)) return m.v;
    static char buf[32];  // unknown type
    size_t i = 0;
    for (; t[i] && i < sizeof(buf) - 1; i++) buf[i] = (t[i] == '_') ? ' ' : (i ? tolower(t[i]) : t[i]);
    buf[i] = 0;
    return buf;
  }

  // ====================================================================
  //  icons (primitives only)
  // ====================================================================
  // lightning bolt, h = pixel height
  static void bolt(int x, int y, int h) {
    auto X = [&](int v) { return x + (v * h) / 24; };
    auto Y = [&](int v) { return y + (v * h) / 24; };
    u8g2.drawTriangle(X(14), Y(0),  X(2),  Y(14), X(12), Y(14));
    u8g2.drawTriangle(X(10), Y(10), X(22), Y(10), X(8),  Y(24));
  }

  // WiFi fan: dot at (cx,by), up to 3 arcs
  static void wifiIcon(int cx, int by, int s, int arcs) {
    u8g2.drawDisc(cx, by, s >= 3 ? 2 : 1);
    for (int i = 1; i <= 3 && i <= arcs; i++) {
      float r = s * i + 1;
      for (int a = -45; a <= 45; a += 3) {
        float rad = a * 0.0174533f;
        u8g2.drawPixel(cx + (int)roundf(sinf(rad) * r), by - (int)roundf(cosf(rad) * r));
        if (s >= 3) u8g2.drawPixel(cx + (int)roundf(sinf(rad) * (r + 1)), by - (int)roundf(cosf(rad) * (r + 1)));
      }
    }
  }

  // cellular signal bars, 4 bars growing right, bottom at by
  static void barsIcon(int x, int by, int n, int bw, int hs) {
    for (int i = 0; i < 4; i++) {
      int h = hs * (i + 1), bx = x + i * (bw + 1);
      if (i < n) u8g2.drawBox(bx, by - h + 1, bw, h);
      else       u8g2.drawHLine(bx, by, bw);
    }
  }

  // warning triangle with "!"  (w = base width)
  static void warnIcon(int x, int y, int w) {
    int h = (w * 87) / 100, cx = x + w / 2, t = (w >= 20) ? 3 : 1;
    u8g2.drawTriangle(cx, y, x, y + h, x + w, y + h);
    u8g2.setDrawColor(0);
    u8g2.drawBox(cx - t / 2, y + h / 3, t, h / 3);
    u8g2.drawBox(cx - t / 2, y + h - 2 - t / 2 - 1, t, t);
    u8g2.setDrawColor(1);
  }

  static void checkIcon(int cx, int cy) {
    u8g2.drawCircle(cx, cy, 11);
    u8g2.drawCircle(cx, cy, 10);
    for (int d = 0; d < 2; d++) {
      u8g2.drawLine(cx - 5, cy + d,     cx - 2, cy + 4 + d);
      u8g2.drawLine(cx - 2, cy + 4 + d, cx + 5, cy - 4 + d);
    }
  }

  static void spinner(int cx, int cy, int r) {
    uint8_t phase = (millis() / 90) % 8;
    for (int i = 0; i < 8; i++) {
      float a = i * 0.785398f - 1.5708f;
      int x = cx + (int)roundf(cosf(a) * r), y = cy + (int)roundf(sinf(a) * r);
      int d = (phase - i + 8) % 8;                       // 0 = head of the tail
      if (d == 0)      u8g2.drawDisc(x, y, 2);
      else if (d < 3)  u8g2.drawDisc(x, y, 1);
      else             u8g2.drawPixel(x, y);
    }
  }

  // ====================================================================
  //  boot gateway card
  // ====================================================================
  static void gatewayCard(int x, int y, bool selected, int kind) {
    const int w = 60, h = 34, cx = x + w / 2;
    if (selected) { u8g2.drawRBox(x, y, w, h, 4); u8g2.setDrawColor(0); }
    else          { u8g2.drawRFrame(x, y, w, h, 4); }

    if (kind == 0) barsIcon(cx - 7, y + 21, 4, 3, 4);            // 4G
    else           wifiIcon(cx, y + 20, 4, 3);                   // WiFi

    u8g2.setFont(F_TXT);
    textC(cx, y + 31, kind == 0 ? "4G GSM" : "WiFi");
    u8g2.setDrawColor(1);
  }

  // ====================================================================
  //  dashboard chrome
  // ====================================================================
  void statusBar(const NetState& ns, const char* title) {
    // gateway + signal
    if (ns.gateway == Gateway::WIFI) {
      wifiIcon(7, 10, 2, ns.linkUp ? min((int)ns.bars, 3) : 0);
    } else {
      barsIcon(0, 10, ns.linkUp ? ns.bars : 0, 2, 2);
      u8g2.setFont(F_TINY);
      u8g2.drawStr(13, 9, "4G");
    }
    if (!ns.linkUp) u8g2.drawLine(0, 11, 14, 1);                 // slash = offline

    u8g2.setFont(F_TITLE);
    u8g2.drawStr(26, 10, title);

    // relay badge
    u8g2.setFont(F_TINY);
    const char* rs = ns.relayOn ? "ON" : "OFF";
    int tx = 88 + (19 - u8g2.getStrWidth(rs)) / 2;
    if (ns.relayOn) { u8g2.drawRBox(88, 2, 19, 9, 2); u8g2.setDrawColor(0); u8g2.drawStr(tx, 9, rs); u8g2.setDrawColor(1); }
    else            { u8g2.drawRFrame(88, 2, 19, 9, 2); u8g2.drawStr(tx, 9, rs); }

    // right slot: blinking warning if anomaly, else a small "uploaded" arrow
    if (ns.anomalyCount > 0) {
      if ((millis() / 500) & 1) warnIcon(112, 1, 12);
    } else if (ns.lastPostMs && (uint32_t)(millis() - ns.lastPostMs) < 800) {
      u8g2.drawTriangle(118, 1, 113, 6, 123, 6);
      u8g2.drawBox(116, 6, 5, 5);
    }
    u8g2.drawHLine(0, 12, 128);
  }

  void pageDots() {
    for (int i = 0; i < PAGE_COUNT; i++) {
      int y = 22 + i * 9;
      if (i == _page) u8g2.drawDisc(125, y, 2);
      else            u8g2.drawPixel(125, y);
    }
  }

  // ====================================================================
  //  dashboard pages
  // ====================================================================
  void pageEnergy(const NetState& ns, const EnergyReadings& r) {
    float kwh = ns.baseKWh + r.energy_import_Wh / 1000.0f;
    char v[16];
    if (kwh < 1000)        snprintf(v, sizeof v, "%.3f", kwh);
    else if (kwh < 100000) snprintf(v, sizeof v, "%.1f", kwh);
    else                   snprintf(v, sizeof v, "%.0f", kwh);

    u8g2.setFont(F_TINY);
    u8g2.drawStr(2, 22, "TOTAL ENERGY");
    bolt(106, 15, 14);

    u8g2.setFont(F_HUGE);
    u8g2.drawStr(2, 46, v);
    int w = u8g2.getStrWidth(v);
    u8g2.setFont(F_TXT);
    u8g2.drawStr(2 + w + 3, 46, "kWh");

    u8g2.drawHLine(0, 50, 118);
    if (ns.relayOn) u8g2.drawDisc(5, 57, 3); else u8g2.drawCircle(5, 57, 3);
    u8g2.setFont(F_TXT);
    u8g2.drawStr(12, 61, ns.relayOn ? "Relay ON" : "Relay OFF");
    char p[16];
    snprintf(p, sizeof p, "%.0f W", r.active_power_W);
    textR(118, 61, p);
  }

  void pagePower(const EnergyReadings& r) {
    char b[20];
    u8g2.setFont(F_TINY);
    u8g2.drawStr(2, 22, "ACTIVE POWER");
    snprintf(b, sizeof b, "%.1f", r.active_power_W);
    u8g2.setFont(F_BIG);
    u8g2.drawStr(2, 40, b);
    int w = u8g2.getStrWidth(b);
    u8g2.setFont(F_TXT);
    u8g2.drawStr(2 + w + 3, 40, "W");

    u8g2.drawHLine(0, 44, 118);
    u8g2.setFont(F_TXT);
    u8g2.drawStr(2, 54, "Apparent");
    snprintf(b, sizeof b, "%.1f VA", r.apparent_power_VA);
    textR(118, 54, b);
    u8g2.drawStr(2, 63, "Reactive");
    snprintf(b, sizeof b, "%.1f var", r.reactive_power_var);
    textR(118, 63, b);
  }

  void pageSupply(const EnergyReadings& r) {
    char b[20];
    u8g2.setFont(F_TINY);
    u8g2.drawStr(2, 22, "VOLTS");
    u8g2.drawStr(62, 22, "AMPS");
    u8g2.setFont(F_BIG);
    snprintf(b, sizeof b, "%.1f", r.voltage_V);
    u8g2.drawStr(2, 41, b);
    snprintf(b, sizeof b, r.current_A < 100 ? "%.2f" : "%.1f", r.current_A);
    u8g2.drawStr(62, 41, b);

    u8g2.drawHLine(0, 46, 118);
    u8g2.setFont(F_TXT);
    snprintf(b, sizeof b, "%.2f Hz", freqOf(r));
    u8g2.drawStr(2, 60, b);
    snprintf(b, sizeof b, "PF %.3f", r.power_factor);
    textR(118, 60, b);
  }

  void row(int y, const char* label, const char* val) {
    u8g2.setFont(F_TINY); u8g2.drawStr(2, y, label);
    u8g2.setFont(F_TXT);  u8g2.drawStr(50, y, val);
  }

  void pageOnline(const NetState& ns) {
    char b[24];
    u8g2.setFont(F_TINY); u8g2.drawStr(2, 22, "VIA");
    u8g2.setFont(F_TXT);
    if (ns.gateway == Gateway::WIFI) marquee(50, 22, 68, ns.ssid[0] ? ns.ssid : "WiFi");
    else                             u8g2.drawStr(50, 22, "4G Cellular");

    row(31, "LINK", ns.linkUp ? "Connected" : "Down");

    if (ns.lastHttp == 0)                          snprintf(b, sizeof b, "--");
    else if (ns.lastHttp >= 200 && ns.lastHttp < 300) snprintf(b, sizeof b, "OK (%d)", ns.lastHttp);
    else                                           snprintf(b, sizeof b, "Err (%d)", ns.lastHttp);
    row(40, "SERVER", b);

    char age[16]; fmtAge(age, sizeof age, ns.lastPostMs);
    row(49, "LAST POST", age);

    snprintf(b, sizeof b, "%lu / %lu", (unsigned long)ns.okCount, (unsigned long)ns.failCount);
    row(58, "OK / FAIL", b);
  }

  void pageAlert(const NetState& ns) {
    if (!ns.anomalyKnown) {
      spinner(22, 38, 10);
      u8g2.setFont(F_TXT);
      u8g2.drawStr(42, 36, "Waiting for");
      u8g2.drawStr(42, 47, "server...");
      return;
    }
    if (ns.anomalyCount == 0) {
      checkIcon(18, 36);
      u8g2.setFont(F_TXT);
      u8g2.drawStr(38, 32, "ALL CLEAR");
      u8g2.setFont(F_TINY);
      u8g2.drawStr(38, 43, "No active");
      u8g2.drawStr(38, 52, "anomalies");
      return;
    }

    uint8_t idx = (ns.anomalyCount > 1) ? (millis() / 3500) % ns.anomalyCount : 0;
    const AnomalyInfo& a = ns.anomalies[idx];
    bool severe = !strcasecmp(a.severity, "HIGH") || !strcasecmp(a.severity, "CRITICAL");
    if (!severe || ((millis() / 400) & 1)) warnIcon(2, 17, 22);

    char b[28];
    u8g2.setFont(F_TINY);
    snprintf(b, sizeof b, "%s SEVERITY", a.severity[0] ? a.severity : "ACTIVE");
    u8g2.drawStr(30, 23, b);
    if (ns.anomalyCount > 1) { snprintf(b, sizeof b, "%u/%u", idx + 1, ns.anomalyCount); textR(118, 23, b); }

    u8g2.setFont(F_TXT);
    marquee(30, 35, 88, anomalyName(a.type));
    u8g2.drawHLine(0, 41, 118);
    u8g2.setFont(F_TXT);
    marquee(2, 53, 116, a.title);

    u8g2.setFont(F_TINY);
    char age[16]; fmtAge(age, sizeof age, ns.lastPostMs);
    snprintf(b, sizeof b, "Checked %s", age);
    u8g2.drawStr(2, 63, b);
  }

  // ====================================================================
  //  full-screen states
  // ====================================================================
  static bool overlayActive(const NetState& ns) {
    return ns.stage == NetStage::Provisioning ||
           (!ns.baseValid && (ns.stage == NetStage::Connecting || ns.stage == NetStage::Failed));
  }

  void drawOverlay(const NetState& ns) {
    if (ns.stage == NetStage::Provisioning)      drawProvisioning();
    else if (ns.stage == NetStage::Failed)       drawFailed(ns);
    else                                         drawConnecting(ns);
  }

  void drawConnecting(const NetState& ns) {
    spinner(20, 32, 13);
    char t[20];
    snprintf(t, sizeof t, "Connecting%.*s", (int)((millis() / 400) % 4), "...");
    u8g2.setFont(F_TXT);
    u8g2.drawStr(46, 26, t);
    u8g2.setFont(F_TINY);
    marquee(46, 38, 80, ns.step);
    u8g2.drawStr(46, 52, ns.gateway == Gateway::WIFI ? "via WiFi" : "via 4G cellular");
    u8g2.drawStr(46, 62, "Hold=menu");
  }

  void drawFailed(const NetState& ns) {
    warnIcon(4, 8, 24);
    u8g2.setFont(F_TXT);
    u8g2.drawStr(36, 20, "Internet");
    u8g2.drawStr(36, 31, "connection");
    u8g2.drawStr(36, 42, "failed");
    u8g2.setFont(F_TINY);
    marquee(2, 53, 124, ns.failMsg);
    long left = (long)(ns.retryAtMs - millis());
    char b[24];
    snprintf(b, sizeof b, "Retry in %lds", left > 0 ? (left + 999) / 1000 : 0);
    u8g2.drawStr(2, 62, b);
    textR(126, 62, "Hold=menu");
  }

  void drawProvisioning() {
    u8g2.setFont(F_TITLE);
    u8g2.drawStr(2, 10, "WiFi Setup");
    wifiIcon(114, 11, 2, 1 + (millis() / 350) % 3);              // animated
    u8g2.drawHLine(0, 13, 128);
    u8g2.setFont(F_TINY);
    u8g2.drawStr(2, 23, "Join this WiFi on your phone:");
    u8g2.setFont(F_TXT);
    u8g2.drawStr(2, 34, WIFI_AP_NAME);
    u8g2.setFont(F_TINY);
    if (WIFI_AP_PASS[0]) { char b[32]; snprintf(b, sizeof b, "Password: %s", WIFI_AP_PASS); u8g2.drawStr(2, 44, b); }
    else                 u8g2.drawStr(2, 44, "(open network)");
    u8g2.drawStr(2, 53, "then open " WIFI_AP_IP_TEXT);
    u8g2.drawStr(2, 62, "Hold=menu");
  }

  void drawCalibration() {
    u8g2.setFont(F_TITLE);
    u8g2.drawStr(2, 10, "CALIBRATION");
    u8g2.drawHLine(0, 13, 128);
    u8g2.setFont(F_TXT);
    u8g2.drawStr(2, 30, "Follow the prompts");
    u8g2.drawStr(2, 42, "in the serial");
    u8g2.drawStr(2, 54, "monitor (115200)");
  }

  // ====================================================================
  //  menu (hold on any page)
  // ====================================================================
  static uint8_t menuBuild(const NetState& ns, const char** out) {
    uint8_t n = 0;
    out[n++] = "Back";
    out[n++] = "Change gateway";
    if (ns.gateway == Gateway::WIFI) out[n++] = "Forget WiFi";
    out[n++] = "Reboot";
    return n;
  }

  void drawMenu(const NetState& ns) {
    const char* it[4];
    uint8_t n = menuBuild(ns, it);
    u8g2.setFont(F_TITLE);
    u8g2.drawStr(2, 10, "Menu");
    u8g2.setFont(F_TINY);
    textR(126, 9, "Click=next Hold=ok");
    u8g2.drawHLine(0, 13, 128);
    u8g2.setFont(F_TXT);
    for (uint8_t i = 0; i < n; i++) {
      int y = 15 + i * 12;
      if (i == _menuIdx) { u8g2.drawRBox(2, y, 124, 11, 2); u8g2.setDrawColor(0); u8g2.drawStr(8, y + 9, it[i]); u8g2.setDrawColor(1); }
      else               { u8g2.drawStr(8, y + 9, it[i]); }
    }
  }

  void menuEvent(BtnEvent ev, const NetState& ns) {
    const char* it[4];
    uint8_t n = menuBuild(ns, it);
    if (_menuIdx >= n) _menuIdx = 0;
    if (ev == BtnEvent::Next) { _menuIdx = (_menuIdx + 1) % n; return; }
    if (ev != BtnEvent::Select) return;

    const char* c = it[_menuIdx];
    if (!strcmp(c, "Back")) { _menu = false; }
    else if (!strcmp(c, "Forget WiFi")) { g_forgetWifiReq = true; _menu = false; }
    else if (!strcmp(c, "Change gateway")) restartScreen("Restarting...");   // boot prompt appears again
    else if (!strcmp(c, "Reboot"))         restartScreen("Rebooting...");
  }

  void restartScreen(const char* msg) {
    relaySet(false);                                     // never restart with the load connected
    u8g2.clearBuffer();
    u8g2.setFont(F_TXT);
    textC(64, 36, msg);
    u8g2.sendBuffer();
    delay(700);
    ESP.restart();
  }
};