#pragma once
// =============================================================================
//  NetLink.h - internet gateway (4G GSM or WiFi), server handshake, shared state
//
//  Everything that touches the modem / WiFi runs inside netTask(). The UI only
//  reads a mutex-protected NetState snapshot, so the display never blocks on
//  the network and the network never blocks on the display.
// =============================================================================
#include "config.h"
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <TinyGsmClient.h>
#include "EnergyMeter.h"

enum class Gateway : uint8_t { GSM = 0, WIFI = 1 };
enum class NetStage : uint8_t { Connecting, Provisioning, Failed, Online };

#define MAX_ANOMALIES 4
struct AnomalyInfo { char type[32]; char severity[12]; char title[64]; };

struct NetState {
  NetStage stage    = NetStage::Connecting;
  Gateway  gateway  = Gateway::GSM;
  bool     linkUp   = false;           // WiFi associated / GPRS attached
  uint8_t  bars     = 0;               // 0..4
  char     step[28] = "Starting";      // text shown under the spinner
  char     ssid[33] = "";
  char     failMsg[28] = "";
  uint32_t retryAtMs = 0;

  int      lastHttp   = 0;
  uint32_t lastPostMs = 0;             // 0 = never
  uint32_t okCount = 0, failCount = 0;

  bool     relayOn    = false;
  bool     baseValid  = false;         // cumulative energy fetched from server
  float    baseKWh    = 0;

  bool     anomalyKnown = false;       // at least one ingest response parsed
  uint8_t  anomalyCount = 0;
  AnomalyInfo anomalies[MAX_ANOMALIES];
};

// ---- globals ----------------------------------------------------------------
static NetState          g_ns;
static SemaphoreHandle_t g_mtx = nullptr;
static Gateway           g_gateway = Gateway::GSM;
static QueueHandle_t     g_readingsQ = nullptr;      // length 1: latest snapshot
static volatile bool     g_forgetWifiReq = false;    // set by UI menu
static WiFiManager       g_wm;

HardwareSerial SerialAT(2);
TinyGsm modem(SerialAT);

// ---- shared-state helpers -----------------------------------------------------
template <typename F> static void nsSet(F f) {
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  f(g_ns);
  xSemaphoreGive(g_mtx);
}
static NetState nsGet() {
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  NetState c = g_ns;
  xSemaphoreGive(g_mtx);
  return c;
}
static void nsStep(const char* s) {
  nsSet([&](NetState& n) { strlcpy(n.step, s, sizeof(n.step)); });
}
static void netInit() { g_mtx = xSemaphoreCreateMutex(); }

// ---- relay ---------------------------------------------------------------------
static void relaySet(bool on) {
  digitalWrite(PIN_RELAY, (on == (RELAY_ACTIVE_HIGH != 0)) ? HIGH : LOW);
  nsSet([&](NetState& n) { n.relayOn = on; });
}

// meter.frequency_Hz is currently always 0 -> fall back to 50 Hz
static inline float freqOf(const EnergyReadings& r) { return r.frequency_Hz > 1.0f ? r.frequency_Hz : 50.0f; }

// =============================================================================
//  HTTP over either transport
// =============================================================================
struct HttpResp { int status = -100; String body; };

static HttpResp wifiHttp(bool post, const char* url, const char* body) {
  HttpResp r;
  WiFiClientSecure client;
  client.setInsecure();                       // same as authmode=0 on the modem
  HTTPClient http;
  http.setTimeout(15000);
  if (!http.begin(client, url)) { r.status = -1; return r; }
  http.addHeader("X-API-Key", API_KEY);
  if (post) {
    http.addHeader("Content-Type", "application/json");
    r.status = http.POST((uint8_t*)body, strlen(body));
  } else {
    r.status = http.GET();
  }
  if (r.status > 0) r.body = http.getString();
  http.end();
  return r;
}

static void gsmHttpTerm() { modem.sendAT("+HTTPTERM"); modem.waitResponse(1000); }

// SIM7600 / A7670 built-in HTTPS stack (same AT sequence as before, now GET + POST)
static HttpResp gsmHttp(bool post, const char* url, const char* body, int len) {
  HttpResp r;
  gsmHttpTerm();

  // TLS 1.2+, no cert check, SNI on (Vercel answers 421 without SNI)
  modem.sendAT("+CSSLCFG=\"sslversion\",0,4");  modem.waitResponse(2000);
  modem.sendAT("+CSSLCFG=\"authmode\",0,0");    modem.waitResponse(2000);
  modem.sendAT("+CSSLCFG=\"enableSNI\",0,1");   modem.waitResponse(2000);

  modem.sendAT("+HTTPINIT");
  if (modem.waitResponse(5000) != 1) { r.status = -1; return r; }

  modem.sendAT("+HTTPPARA=\"SSLCFG\",0");       modem.waitResponse(2000);

  modem.sendAT("+HTTPPARA=\"URL\",\"", url, "\"");
  if (modem.waitResponse(5000) != 1) { gsmHttpTerm(); r.status = -2; return r; }

  modem.sendAT("+HTTPPARA=\"CONTENT\",\"application/json\"");
  if (modem.waitResponse(5000) != 1) { gsmHttpTerm(); r.status = -3; return r; }

  modem.sendAT("+HTTPPARA=\"USERDATA\",\"X-API-Key: ", API_KEY, "\"");
  modem.waitResponse(2000);

  if (post) {
    modem.sendAT("+HTTPDATA=", len, ",10000");
    if (modem.waitResponse(5000, "DOWNLOAD") != 1) { gsmHttpTerm(); r.status = -4; return r; }
    SerialAT.write((const uint8_t*)body, len);
    if (modem.waitResponse(10000) != 1) { gsmHttpTerm(); r.status = -5; return r; }
  }

  modem.sendAT("+HTTPACTION=", post ? 1 : 0);
  if (modem.waitResponse(3000) != 1) { gsmHttpTerm(); r.status = -6; return r; }

  int respLen = 0;
  if (modem.waitResponse(30000L, "+HTTPACTION:") == 1) {
    String line = SerialAT.readStringUntil('\n');            // " 1,200,123"
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > c1) {
      r.status = line.substring(c1 + 1, c2).toInt();
      respLen  = line.substring(c2 + 1).toInt();
    } else {
      r.status = -8;
    }
  } else {
    r.status = -7;
  }

  if (r.status > 0 && respLen > 0) {
    modem.sendAT("+HTTPREAD=0,", respLen);
    if (modem.waitResponse(5000, "+HTTPREAD:") == 1) {
      String hdr = SerialAT.readStringUntil('\n');           // " DATA,123" or " 123"
      int n = hdr.substring(hdr.lastIndexOf(',') + 1).toInt();
      if (n <= 0 || n > respLen) n = respLen;
      if (n > 1024) n = 1024;
      char tmp[1025];
      SerialAT.setTimeout(3000);
      size_t got = SerialAT.readBytes(tmp, n);
      tmp[got] = 0;
      r.body = tmp;
    }
  }

  gsmHttpTerm();
  return r;
}

static HttpResp httpReq(bool post, const char* url, const char* body, int len) {
  return (g_gateway == Gateway::WIFI) ? wifiHttp(post, url, body) : gsmHttp(post, url, body, len);
}

// =============================================================================
//  Bringing the link up
// =============================================================================
static bool gsmLink(bool restartModem) {
  if (restartModem) {
    nsStep("Starting modem");
    if (!modem.restart()) { Serial.println("[gsm] modem not responding"); return false; }
  }
  nsStep("Finding network");
  if (!modem.waitForNetwork(60000L)) { Serial.println("[gsm] no network"); return false; }
  nsStep("Data session");
  if (!modem.gprsConnect(GSM_APN, GSM_USER, GSM_PASS)) { Serial.println("[gsm] APN failed"); return false; }
  return modem.isGprsConnected();
}

// Non-blocking WiFiManager: tries saved credentials, otherwise opens the
// provisioning AP and keeps servicing it until the user finishes (or forgets).
static bool wifiLink() {
  WiFi.mode(WIFI_STA);
  g_wm.setConfigPortalBlocking(false);
  g_wm.setConnectTimeout(WIFI_CONNECT_TIMEOUT_S);
  g_wm.setConfigPortalTimeout(0);                 // we manage the timeout ourselves

  for (;;) {
    nsStep(g_wm.getWiFiIsSaved() ? "Joining saved WiFi" : "WiFi setup needed");
    if (g_wm.autoConnect(WIFI_AP_NAME, WIFI_AP_PASS)) break;      // connected with saved creds

    // portal is now open
    nsSet([&](NetState& n) { n.stage = NetStage::Provisioning; });
    uint32_t t0 = millis();
    bool again = false;
    while (WiFi.status() != WL_CONNECTED) {
      g_wm.process();
      if (g_forgetWifiReq) {
        g_forgetWifiReq = false;
        g_wm.stopConfigPortal();
        g_wm.resetSettings();
        again = true;
        break;
      }
      // saved credentials exist but the router was away: give them another go
      if (g_wm.getWiFiIsSaved() && (millis() - t0) > WIFI_PORTAL_RETRY_MS) {
        g_wm.stopConfigPortal();
        return false;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!again) break;
  }
  WiFi.setAutoReconnect(true);
  nsSet([&](NetState& n) { n.stage = NetStage::Connecting; });
  return WiFi.status() == WL_CONNECTED;
}

// =============================================================================
//  Server protocol
// =============================================================================
static int buildJson(const EnergyReadings& r, float totalKWh, bool relayOn, char* buf, size_t n) {
  return snprintf(buf, n,
      "{\"meterId\":\"%s\",\"apiKey\":\"%s\","
      "\"voltage\":%.2f,\"current\":%.3f,\"activePower\":%.2f,"
      "\"apparentPower\":%.2f,\"reactivePower\":%.2f,"
      "\"powerFactor\":%.3f,\"phaseAngle\":%.1f,\"frequency\":%.2f,"
      "\"energy\":%.3f,\"relayState\":\"%s\"}",
      DEVICE_ID, API_KEY,
      r.voltage_V, r.current_A, r.active_power_W,
      r.apparent_power_VA, r.reactive_power_var,
      r.power_factor, r.phase_angle_deg, freqOf(r),
      totalKWh,
      relayOn ? "ON" : "OFF");
}

// GET /api/telemetry/energy -> cumulative energy so the meter carries on where it left off
static bool fetchBaseEnergy() {
  nsStep("Contacting server");
  HttpResp r = httpReq(false, ENERGY_URL, "", 0);
  Serial.printf("[srv] energy GET -> %d %s\n", r.status, r.body.c_str());
  if (r.status != 200) return false;

  JsonDocument doc;
  if (deserializeJson(doc, r.body)) return false;
  if (!(doc["success"] | false)) return false;

  float kwh = doc["energy"] | 0.0f;
  if (doc["energyWh"].is<float>()) kwh = (doc["energyWh"].as<float>()) / 1000.0f;   // Wh is the more precise field

  nsSet([&](NetState& n) { n.baseKWh = kwh; n.baseValid = true; });
  return true;
}

static void handleIngestResponse(const String& body) {
  JsonDocument doc;
  if (deserializeJson(doc, body)) return;

#if APPLY_SERVER_RELAY
  const char* rs = doc["relayState"] | (const char*)nullptr;
  if (rs) relaySet(strcmp(rs, "ON") == 0);
#endif

  const bool detected = doc["anomalyDetected"] | false;
  nsSet([&](NetState& n) {
    n.anomalyKnown = true;
    n.anomalyCount = 0;
    if (!detected) return;
    for (JsonObject a : doc["anomalies"].as<JsonArray>()) {
      if (n.anomalyCount >= MAX_ANOMALIES) break;
      AnomalyInfo& d = n.anomalies[n.anomalyCount++];
      strlcpy(d.type,     a["type"]     | "UNKNOWN", sizeof(d.type));
      strlcpy(d.severity, a["severity"] | "",        sizeof(d.severity));
      strlcpy(d.title,    a["title"]    | "",        sizeof(d.title));
    }
    if (n.anomalyCount == 0) {                       // flagged but no details
      AnomalyInfo& d = n.anomalies[n.anomalyCount++];
      strlcpy(d.type, "UNKNOWN", sizeof(d.type));
      d.severity[0] = 0;
      strlcpy(d.title, "Anomaly detected", sizeof(d.title));
    }
  });
}

// =============================================================================
//  The network task
// =============================================================================
static void refreshLink(bool& linkOk) {
  uint8_t bars = 0;
  char ssid[33] = "";
  if (g_gateway == Gateway::WIFI) {
    linkOk = (WiFi.status() == WL_CONNECTED);
    int rssi = WiFi.RSSI();
    bars = rssi > -55 ? 4 : rssi > -65 ? 3 : rssi > -75 ? 2 : rssi > -85 ? 1 : 0;
    strlcpy(ssid, WiFi.SSID().c_str(), sizeof(ssid));
  } else {
    linkOk = modem.isGprsConnected();
    int csq = modem.getSignalQuality();                    // 0..31, 99 = unknown
    bars = (csq <= 0 || csq == 99) ? 0 : (uint8_t)min(4, (csq + 7) / 8);
  }
  nsSet([&](NetState& n) { n.linkUp = linkOk; n.bars = bars; if (ssid[0]) strlcpy(n.ssid, ssid, sizeof(n.ssid)); });
}

static void failWait(const char* msg) {
  nsSet([&](NetState& n) {
    n.stage = NetStage::Failed;
    n.linkUp = false;
    strlcpy(n.failMsg, msg, sizeof(n.failMsg));
    n.retryAtMs = millis() + NET_RETRY_MS;
  });
  vTaskDelay(pdMS_TO_TICKS(NET_RETRY_MS));
}

static void netTask(void*) {
  EnergyReadings snap;
  bool have = false;
  uint8_t fails = 0;
  uint8_t failCycles = 0;
  bool gsmRestart = true;                                  // first GSM bring-up restarts the modem
  uint32_t lastSend = millis() - SEND_INTERVAL_MS;         // send first reading right away
  uint32_t lastLinkCheck = 0;

  for (;;) {
    // ---------------- 1. link ----------------
    nsSet([&](NetState& n) { n.stage = NetStage::Connecting; n.linkUp = false; });
    bool up = (g_gateway == Gateway::WIFI) ? wifiLink() : gsmLink(gsmRestart);
    gsmRestart = false;
    if (!up) { failWait(g_gateway == Gateway::WIFI ? "WiFi not connected" : "No cellular data"); continue; }

    bool linkOk = true;
    refreshLink(linkOk);

    // ---------------- 2. server handshake (once per boot) ----------------
    if (!nsGet().baseValid) {
      if (!fetchBaseEnergy()) { failWait("Server unreachable"); continue; }
      relaySet(true);    // relay ON once the server has been reached
    }
    nsSet([&](NetState& n) { n.stage = NetStage::Online; });
    Serial.println("[net] online");

    // ---------------- 3. steady state ----------------
    while (linkOk) {
      if (g_forgetWifiReq && g_gateway == Gateway::WIFI) {  // "Forget WiFi" from the menu
        g_forgetWifiReq = false;
        g_wm.resetSettings();
        WiFi.disconnect(true, true);
        break;                                              // -> wifiLink() opens the portal
      }

      if (xQueueReceive(g_readingsQ, &snap, pdMS_TO_TICKS(500)) == pdTRUE) have = true;

      uint32_t now = millis();
      if (now - lastLinkCheck > 5000) { lastLinkCheck = now; refreshLink(linkOk); if (!linkOk) break; }

      if (!have || (uint32_t)(now - lastSend) < SEND_INTERVAL_MS) continue;

      NetState ns = nsGet();
      float total = ns.baseKWh + snap.energy_import_Wh / 1000.0f;
      char body[640];
      int n = buildJson(snap, total, ns.relayOn, body, sizeof(body));
      if (n <= 0 || n >= (int)sizeof(body)) continue;

      Serial.printf("[srv] POST %s\n", body);
      HttpResp r = httpReq(true, INGEST_URL, body, n);
      bool ok = (r.status >= 200 && r.status < 300);
      Serial.printf("[srv] -> %d %s\n", r.status, r.body.c_str());

      nsSet([&](NetState& s) {
        s.lastHttp = r.status;
        if (ok) { s.lastPostMs = millis(); s.okCount++; } else { s.failCount++; }
      });

      if (ok) {
        fails = 0; failCycles = 0;
        lastSend = millis();
        handleIngestResponse(r.body);
      } else {
        lastSend = millis() - (SEND_INTERVAL_MS - 5000UL);   // retry in 5 s
        if (++fails >= 3) {                                  // give the link a kick
          fails = 0;
          gsmRestart = (++failCycles >= 2);                  // 2nd cycle in a row: full modem restart
          break;
        }
      }
    }
    // loop back to step 1 (reconnect). Energy base is NOT re-fetched.
  }
}

static void netStart(Gateway g) {
  g_gateway = g;
  nsSet([&](NetState& n) { n.gateway = g; });
  if (g == Gateway::GSM) SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  g_readingsQ = xQueueCreate(1, sizeof(EnergyReadings));
  xTaskCreatePinnedToCore(netTask, "net", 16384, nullptr, 1, nullptr, tskNO_AFFINITY);   // TLS + WiFiManager need the big stack
}