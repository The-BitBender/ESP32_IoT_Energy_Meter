#pragma once
#include <Arduino.h>

// =============================================================================
//  Device / server
// =============================================================================
#define DEVICE_ID         "MTR-0011"
#define API_KEY           "VP-HL5Y8Z26PA8FZ"

#define SERVER_BASE       "https://smart-energy-meter-mu.vercel.app"
#define INGEST_URL        SERVER_BASE "/api/telemetry/ingest"
#define ENERGY_URL        SERVER_BASE "/api/telemetry/energy?meterId=" DEVICE_ID

#define SEND_INTERVAL_MS  10000UL      // upload period (cellular data is costly)
#define NET_RETRY_MS      5000UL       // wait between failed connection attempts

// =============================================================================
//  Cellular (SIM7600 / A7670)
// =============================================================================
#define GSM_APN           "web.gprs.mtnnigeria.net"  // your carrier's APN
#define GSM_USER          ""
#define GSM_PASS          ""
#define MODEM_RX          12           // ESP32 RX  <- modem TX
#define MODEM_TX          13
#define MODEM_BAUD        115200

#define TINY_GSM_MODEM_SIM7600         // must be defined BEFORE TinyGsmClient.h
#define TINY_GSM_RX_BUFFER 1024

// =============================================================================
//  WiFi provisioning (WiFiManager, access-point mode)
// =============================================================================
#define WIFI_AP_NAME            "EnergyMeter-Setup"
#define WIFI_AP_PASS            "meter1234"          // 8+ chars, or "" for an open AP
#define WIFI_AP_IP_TEXT         "192.168.4.1"        // WiFiManager default portal address
#define WIFI_CONNECT_TIMEOUT_S  20
#define WIFI_PORTAL_RETRY_MS    180000UL             // portal open this long with saved creds -> retry them

// =============================================================================
//  Pins
// =============================================================================
#define PIN_BUTTON        4           // button between this pin and GND (internal pull-up used)
#define PIN_RELAY         15
#define RELAY_ACTIVE_HIGH 1            // 0 for active-low relay boards

// =============================================================================
//  Display / UI behaviour
// =============================================================================
#define OLED_IS_SH1106          0      // many "1.3 inch SSD1306" modules are really SH1106: set 1 if the picture is shifted 2px / has garbage on the edge
#define BOOT_SELECT_TIMEOUT_MS  10000UL // gateway prompt auto-selects the last used gateway after this long (0 = wait forever)
#define LONG_PRESS_MS           700
#define AUTO_JUMP_ON_ANOMALY    1      // jump to the anomaly page when a new anomaly appears
#define APPLY_SERVER_RELAY      1      // obey "relayState" returned by the ingest endpoint