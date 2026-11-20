#include <Arduino.h>
#include <Preferences.h>

#include "config.h"          // settings + TINY_GSM defines (must come before NetLink.h)
#include "EnergyMeter.h"
#include "NetLink.h"
#include "Button.h"
#include "Ui.h"

// =============================================================================
//  Globals
// =============================================================================
EnergyMeter meter;
Button      button;
Ui          ui;
Preferences prefs;

static EnergyReadings lastReadings = {};

// =============================================================================
//  Meter calibration data  (paste the block printed by the calibration wizard)
// =============================================================================
static void loadCalibration(EnergyMeterCalibration& cal) {
  cal.meter_constant = 3200;      // imp/kWh
  cal.mmode          = 0x7422;    // L line PGA gain bits [15:13]: 1xx=x1 000=x4 001=x8 010=x16 011=x24
  cal.pl_const_h     = 0x00D3;
  cal.pl_const_l     = 0xEB1C;
  cal.lgain          = 0x0000;
  cal.lphi           = 0x0000;
  cal.ugain          = 0x74A6;
  cal.igain_l        = 0x0B9A;
  cal.sag_th         = 0x1B0F;
  cal.pstart_th      = 0x0958;
  cal.qstart_th      = 0x0BAE;
  cal.ioffset_l      = 0x0000;
  cal.poffset_l      = 0x0000;
  cal.qoffset_l      = 0x0000;
}

static void meterTick() { meter.update(); }

// =============================================================================
//  Arduino entry points
// =============================================================================
static uint32_t lastErrCount = 0;
static EnergyMeter::State lastState = EnergyMeter::State::Off;

void setup() {
  Serial.begin(115200);
  delay(200);

  netInit();                                   // shared-state mutex
  pinMode(PIN_RELAY, OUTPUT);
  relaySet(false);                             // relay stays OFF until the server is reached

  button.begin(PIN_BUTTON);
  ui.begin();
  ui.splash();

  // ---- Meter ----
  EnergyMeterConfig cfg;
  cfg.pins = {18, 19, 23, 5};                  // SCK, MISO, MOSI, CS
  cfg.poll_interval_ms = 1000;
  loadCalibration(cfg.cal);
  if (!meter.begin(cfg)) {
    Serial.printf("EnergyMeter begin failed: %s\n", meter.lastError());
  }
  meter.enableCalibration(Serial);             // type "cal" in the serial monitor to calibrate

  // ---- Internet gateway: ask, defaulting to the last one used ----
  prefs.begin("meter", false);
  Gateway last = (Gateway)prefs.getUChar("gw", (uint8_t)Gateway::GSM);
  Gateway gw   = ui.selectGateway(button, last, meterTick);
  if (gw != last) prefs.putUChar("gw", (uint8_t)gw);
  prefs.end();

  netStart(gw);                                // starts the network task
}

void loop() {
  meter.update();                              // never blocks; neither the modem nor the OLED can stall it

  if (meter.newData() && !meter.calibrating()) {
    lastReadings = meter.readings();
    Serial.printf("V=%.2f I=%.3f P=%.0fW PF=%.3f f=%.2fHz E+=%.3fWh\n",
                  lastReadings.voltage_V, lastReadings.current_A, lastReadings.active_power_W,
                  lastReadings.power_factor, lastReadings.frequency_Hz, lastReadings.energy_import_Wh);
    xQueueOverwrite(g_readingsQ, &lastReadings);   // hand the latest snapshot to the uploader
  }

  ui.frame(button.poll(), nsGet(), lastReadings, meter.calibrating());

    // Report state changes and errors to the serial monitor
  // if (!meter.calibrating() && meter.state() != lastState) {
  //   lastState = meter.state();
  //   Serial.printf("[meter] state -> %s\n", meter.stateName());
  // }

  // if (!meter.calibrating() && meter.errorCount() != lastErrCount) {
  //   lastErrCount = meter.errorCount();
  //   Serial.printf("[meter] error #%lu: %s\n", (unsigned long)lastErrCount, meter.lastError());
  // }
}