#pragma once
#include <Arduino.h>
#include "M90E26.h"

/**
 * Calibration data produced by the built-in UART calibration wizard
 * (EnergyMeter::enableCalibration). Fill it in main.cpp (loadCalibration())
 * with the block the wizard prints. Nothing is stored in flash.
 *
 * The initial values are the M90E26 datasheet power-on values, NOT calibrated
 * for your hardware (note: power-on L line PGA gain is x1).
 */
struct EnergyMeterCalibration {
  uint16_t meter_constant = 3200;   // imp/kWh, must match PL_Constant
  uint16_t mmode = 0x9422;          // MMode[15:13] L line PGA gain: 1xx=x1 000=x4 001=x8 010=x16 011=x24
  uint16_t pl_const_h = 0x0015;     // PL_Constant high word (AN46102 3.2.2)
  uint16_t pl_const_l = 0xD174;     // PL_Constant low word
  uint16_t lgain = 0x0000;          // L line energy gain, two's complement, +-50 % (AN46102 3.2.4.2)
  uint16_t lphi = 0x0000;           // L line phase angle, SIGN-MAGNITUDE: bit15 = sign, bits 9:0 = value (3.2.4.3)
  uint16_t ugain = 0x6720;          // voltage rms gain (bit 15 must be 0)
  uint16_t igain_l = 0x7A13;        // L line current rms gain
  uint16_t sag_th = 0x1D6A;         // voltage sag threshold
  uint16_t pstart_th = 0x08BD;      // active startup power threshold (3.2.4.4)
  uint16_t qstart_th = 0x0AEC;      // reactive startup power threshold
  uint16_t ioffset_l = 0x0000;      // L line current offset (3.2.4.5)
  uint16_t poffset_l = 0x0000;      // L line active power offset, two's complement (3.2.4.1)
  uint16_t qoffset_l = 0x0000;      // L line reactive power offset, two's complement
};

struct EnergyMeterConfig {
  // ---- Hardware ---------------------------------------------------------------
  M90E26::Pins pins = {18, 19, 23, 5};        // SCK, MISO, MOSI, CS
  spi_host_device_t spi_host = SPI2_HOST;
  // Datasheet SPI timing (tCLH >= 30T+10 ns, tCLL >= 16T+10 ns, T = 122 ns) limits
  // a 50 % duty clock to about 136 kHz, so stay at or below that.
  uint32_t spi_hz = 100000;

  // ---- Timing -----------------------------------------------------------------
  uint32_t poll_interval_ms = 500;            // time between measurement cycles
  uint32_t retry_delay_ms = 1000;             // wait before re-initialising after a fault

  // ---- Calibration result (see EnergyMeterCalibration) --------------------------
  EnergyMeterCalibration cal;

  // ---- Other configuration registers --------------------------------------------
  uint16_t func_en = 0x0030;
  uint16_t ngain = 0x0000;                    // N line unused (no CT on N)
  uint16_t nphi = 0x0000;
  uint16_t pnol_th = 0x0000;
  uint16_t qnol_th = 0x0000;
  uint16_t igain_n = 0x7530;
  uint16_t uoffset = 0x0000;                  // not applicable for a self-powered meter
  uint16_t ioffset_n = 0x0000;
  uint16_t poffset_n = 0x0000;
  uint16_t qoffset_n = 0x0000;
  uint16_t small_p_mod = 0x0000;              // 0xA987 = small-power mode (used only by calibration)

  // ---- Software trims applied on top of the converted values --------------------
  float v_trim = 1.0f;
  float i_trim = 1.0f;
  float p_trim = 1.0f;                        // applies to P, Q and S
};

struct EnergyReadings {
  float voltage_V;
  float current_A;
  float active_power_W;        // negative = export (reverse)
  float reactive_power_var;
  float apparent_power_VA;
  float power_factor;
  float phase_angle_deg;
  float frequency_Hz;

  double energy_import_Wh;     // forward active, accumulated since begin()/resetEnergy()
  double energy_export_Wh;     // reverse active
  double reactive_fwd_varh;
  double reactive_rev_varh;

  uint16_t sys_status;         // raw SysStatus
  uint16_t en_status;          // raw EnStatus
  uint32_t timestamp_ms;       // millis() when this snapshot completed
};