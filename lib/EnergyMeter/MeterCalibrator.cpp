#include "MeterCalibrator.h"
#include "EnergyMeter.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

// UART calibration wizard for the M90E26 (procedure and formulas per AN46102).
// Internal to the EnergyMeter module: EnergyMeter owns it and drives it from update().

namespace {

const uint32_t kStepTimeoutMs = 40000;
const uint8_t kSamples = 10;                  // snapshots averaged per rms measurement
const uint32_t kCalPollMs = 300;              // faster polling while calibrating

// Energy-error window. AN46102 3.2.4.2 / 3.2.4.3 take the error from an energy
// measurement ("calibration bench"), so Lgain/Lphi are derived from energy, not Pmean.
const uint32_t kEwMinMs = 20000;
const int64_t kEwMinCounts = 500;             // 0.1-pulse LSBs -> about +-0.2 % quantisation
const uint32_t kEwMaxMs = 300000;
const int64_t kEwAbortCounts = 50;

const double kPlNumerator = 838860800.0;      // 3.2.2
const double kPhaseK = 3763.74;               // 3.2.4.3 (50 Hz, PF 0.5L)
const double kStartK = 93.2067556;            // 3.2.4.4
const double kStartKsActive = 0.004;          // 0.4 % of Ib -> datasheet default PStartTh (GL=24, VL=1, VU=250)
const double kStartKsReactive = 0.005;        // 0.5 % of Ib -> datasheet default QStartTh
const uint16_t kSmallPowerKey = 0xA987;       // 3.2.4.1
const long kLgainLimit = 16384;               // +-50 % (3.2.5.1)
const long kLphiLimit = 1023;                 // bits 9:0, sign in bit 15

// MMode[15:13] current channel gain (datasheet): 1xx = x1, 000 = x4, 001 = x8, 010 = x16, 011 = x24
int glFromMMode(uint16_t m) {
  static const int table[4] = {4, 8, 16, 24};
  uint8_t c = (m >> 13) & 7;
  if (c & 4) return 1;
  return table[c & 3];
}
int gainCode(int gl) {
  switch (gl) {
    case 1:  return 4;   // 100b
    case 4:  return 0;   // 000b
    case 8:  return 1;   // 001b
    case 16: return 2;   // 010b
    case 24: return 3;   // 011b
  }
  return -1;
}

// Lphi is "signed, MSB is the sign bit": sign-magnitude, 10-bit magnitude.
int lphiDecode(uint16_t r) {
  int m = r & 0x03FF;
  return (r & 0x8000) ? -m : m;
}
uint16_t lphiEncode(long v) {
  long m = (v < 0) ? -v : v;
  if (m > kLphiLimit) m = kLphiLimit;
  return (uint16_t)((v < 0 ? 0x8000 : 0) | m);
}

}  // namespace

void MeterCalibrator::begin(EnergyMeter& meter, Stream& io) {
  _m = &meter;
  _io = &io;
  _io->println("[cal] type 'cal' to start UART calibration, 'help' for commands");
}

void MeterCalibrator::update() {
  if (!_m) return;

  uint32_t limit = (_cs == CalState::Sampling && _smpEnergy) ? (kEwMaxMs + 30000UL) : kStepTimeoutMs;
  if ((_cs == CalState::WaitReady || _cs == CalState::Sampling) && (millis() - _csEnterMs) > limit) {
    cancel("meter not responding (check SPI wiring / meter errors)");
    return;
  }

  double v = 0;
  bool yes = false;
  String s;

  switch (_cs) {
    case CalState::Idle:
      handleIdle();
      break;

    // ---- Hardware parameters (AN46102 3.2.2) --------------------------------
    case CalState::AskUn:
      if (askNumber("Reference line voltage Un in volts (voltage at which Vu is measured)",
                    230.0, 50, 480, v)) {
        _hwUn = v;
        go(CalState::AskIb);
      }
      break;

    case CalState::AskIb:
      if (askNumber("Basic current Ib in amps (the current you will calibrate at)",
                    NAN, 0.05, 100, v)) {
        _hwIb = v;
        go(CalState::AskMC);
      }
      break;

    case CalState::AskMC:
      if (askNumber("Meter constant MC in imp/kWh", (double)_cc.cal.meter_constant,
                    100, 65535, v)) {
        _hwMc = floor(v + 0.5);
        go(CalState::AskGL);
      }
      break;

    case CalState::AskGL:
      if (askNumber("L line current PGA gain GL (1, 4, 8, 16 or 24)",
                    (double)glFromMMode(_cc.cal.mmode), 1, 24, v)) {
        int gl = (int)lround(v);
        if (gainCode(gl) < 0) {
          _io->println("  GL must be 1, 4, 8, 16 or 24");
          _prompted = false;
        } else {
          _hwGl = gl;
          go(CalState::AskVi);
        }
      }
      break;

    case CalState::AskVi:
      if (askNumber("Current sampling voltage Vi in mV rms (voltage at the IC's current input "
                    "at Ib, i.e. CT secondary current x burden resistor)",
                    NAN, 0.001, 5000, v)) {
        _hwVi = v;
        go(CalState::AskVu);
      }
      break;

    case CalState::AskVu:
      if (askNumber("Voltage sampling voltage Vu in mV rms (voltage at the IC's voltage input at Un)",
                    NAN, 1, 5000, v)) {
        _hwVu = v;
        applyHardwareParams();
      }
      break;

    // ---- Generic wait / sample states -----------------------------------------
    case CalState::WaitReady:
      if (_m->running()) go(_csAfterReady);
      break;

    case CalState::Sampling:
      if (sampleStep()) go(_csAfterSample);
      break;

    // ---- Step 1: voltage / current rms gain (3.2.4.5) ---------------------------
    case CalState::RmsAskV:
      if (askNumber("Actual line voltage in V (reference meter)", NAN, 50, 480, v)) {
        _refV = v;
        go(CalState::RmsAskI);
      }
      break;

    case CalState::RmsAskI:
      if (askNumber("Actual load current in A (reference meter)", NAN, 0.01, 100, v)) {
        _refI = v;
        if (_refI < 0.2 * _hwIb)
          _io->println("  warning: far below Ib - calibrate at Ib for best accuracy");
        if (_refI > 1.2 * _hwIb)
          _io->println("  warning: above Ib - the PL_Constant/Ib assumption no longer holds");
        _io->print("Sampling");
        armSampling(CalState::RmsApply, false);
        go(CalState::Sampling);
      }
      break;

    case CalState::RmsApply:
      rmsApply();
      break;

    case CalState::RmsConfirm:
      if (!_prompted) {
        _io->println("  After correction (reference vs IC):");
        _io->printf("    Voltage : %.2f V vs %.2f V  (%+.2f %%)\n", _refV, _avgV, (_avgV - _refV) / _refV * 100.0);
        _io->printf("    Current : %.3f A vs %.3f A  (%+.2f %%)\n", _refI, _avgI, (_avgI - _refI) / _refI * 100.0);
        _io->print("Enter = accept, r = apply another correction, q = cancel: ");
        _prompted = true;
      }
      if (!readLine(s)) break;
      s.toLowerCase();
      if (s == "q") cancel("cancelled by user");
      else if (s == "r") go(CalState::RmsApply);
      else if (s.length() == 0) go(CalState::LgAskP);
      else _prompted = false;
      break;

    // ---- Step 2: Lgain from the ENERGY error at PF = 1 (3.2.4.2) -----------------
    case CalState::LgAskP:
      if (askNumber("Actual active power in W (reference wattmeter, PF = 1)",
                    floor(_refV * _refI * 10.0 + 0.5) / 10.0, 1, 100000, v)) {
        _refP = v;
        _ewRefP = v;
        printEnergyEstimate(v);
        armSampling(CalState::LgApply, true);
        go(CalState::Sampling);
      }
      break;

    case CalState::LgApply:
      lgApply();
      break;

    case CalState::LgConfirm:
      if (!_prompted) {
        _io->printf("  Energy error after correction: %+.3f %%  (%.0f s, %lld counts)\n",
                    _epsE * 100.0, _ewDt / 1000.0, (long long)_ewCounts);
        _io->printf("  Mean-power register: %.1f W vs reference %.1f W (%+.2f %%)\n",
                    _avgP, _refP, (_avgP - _refP) / _refP * 100.0);
        _io->print("Enter = accept, r = apply another correction, q = cancel: ");
        _prompted = true;
      }
      if (!readLine(s)) break;
      s.toLowerCase();
      if (s == "q") cancel("cancelled by user");
      else if (s == "r") go(CalState::LgApply);
      else if (s.length() == 0) go(CalState::PhaseAsk);
      else _prompted = false;
      break;

    // ---- Step 3: Lphi from the ENERGY error at PF = 0.5L (3.2.4.3), optional -----
    case CalState::PhaseAsk:
      if (!_prompted) {
        _io->println("\n=== Step 3: phase angle calibration (optional) ===");
        _io->println("Needs a 0.5 inductive load (PF = 0.5L) at about Ib and 50 Hz.");
        _io->println("Recommended with a CT (it adds phase error). Skip with n if you have no such load.");
      }
      if (askYesNo("Run phase calibration now? (y/n)", yes)) go(yes ? CalState::PhaseAskV : CalState::OffAsk);
      break;

    case CalState::PhaseAskV:
      if (askNumber("Actual line voltage in V (reference meter)", _refV > 0 ? _refV : NAN, 50, 480, v)) {
        _refV = v;
        go(CalState::PhaseAskI);
      }
      break;

    case CalState::PhaseAskI:
      if (askNumber("Actual load current in A (reference meter)", _refI > 0 ? _refI : NAN, 0.01, 100, v)) {
        _refI = v;
        go(CalState::PhaseAskP);
      }
      break;

    case CalState::PhaseAskP:
      if (askNumber("Actual active power in W at PF 0.5L (reference wattmeter)",
                    floor(_refV * _refI * 0.5 * 10.0 + 0.5) / 10.0, 1, 100000, v)) {
        _refP = v;
        _ewRefP = v;
        _phSign = 1;
        _phFlipTried = false;
        _phUseStored = false;
        printEnergyEstimate(v);
        armSampling(CalState::PhaseApply, true);
        go(CalState::Sampling);
      }
      break;

    case CalState::PhaseApply:
      phaseApply();
      break;

    case CalState::PhaseConfirm: {
      bool worse = !_phFlipTried && fabs(_epsE) > 1.2 * fabs(_phEpsBefore) && fabs(_epsE) > 0.005;
      if (!_prompted) {
        _io->printf("  Energy error at PF 0.5L: before %+.3f %%, after %+.3f %%  (%.0f s, %lld counts, %.2f Hz)\n",
                    _phEpsBefore * 100.0, _epsE * 100.0, _ewDt / 1000.0, (long long)_ewCounts, _avgF);
        if (fabs(_avgF - 50.0) > 1.0)
          _io->println("  warning: the Lphi formula in AN46102 is specified for 50 Hz");
        if (worse) _io->println("  The error got worse: the correction may have gone the wrong way.");
        _io->print(worse ? "Enter = keep, f = flip sign and retry, r = repeat, q = cancel: "
                         : "Enter = accept, r = apply another correction, q = cancel: ");
        _prompted = true;
      }
      if (!readLine(s)) break;
      s.toLowerCase();
      if (s == "q") cancel("cancelled by user");
      else if (s == "r") { _phUseStored = false; go(CalState::PhaseApply); }
      else if (s == "f" && worse) {
        _phFlipTried = true;
        _phSign = -_phSign;
        _cc.cal.lphi = lphiEncode(_phOld);     // undo, recompute from the original error
        _phUseStored = true;
        go(CalState::PhaseApply);
      }
      else if (s.length() == 0) go(CalState::OffAsk);
      else _prompted = false;
      break;
    }

    // ---- Step 4: no-load offsets (3.2.4.1, 3.2.4.5), optional ---------------------
    case CalState::OffAsk:
      if (!_prompted) {
        _io->println("\n=== Step 4: no-load offset calibration (optional) ===");
        _io->println("Removes the small current/power the IC reads with NO load; it matters for low-current");
        _io->println("accuracy. Keep the line voltage applied. With a CT only switch the LOAD off:");
        _io->println("NEVER open the CT secondary circuit.");
      }
      if (askYesNo("Run no-load offset calibration now? (y/n)", yes)) go(yes ? CalState::OffWait : CalState::Finish);
      break;

    case CalState::OffWait:
      if (!_prompted) {
        _io->print("Switch the load OFF (voltage stays on), then press Enter: ");
        _prompted = true;
      }
      if (!readLine(s)) break;
      s.toLowerCase();
      if (s == "q") cancel("cancelled by user");
      else { _offStep = 0; go(CalState::OffRun); }
      break;

    case CalState::OffRun:
      offRun();
      break;

    case CalState::OffConfirm:
      if (!_prompted) {
        _io->printf("  No-load readings with the offsets applied: I = %.0f mA, P = %.0f W, Q = %.0f var\n",
                    _avgI * 1000.0, _avgP, _avgQ);
        _io->print("Enter = accept, r = redo, q = cancel: ");
        _prompted = true;
      }
      if (!readLine(s)) break;
      s.toLowerCase();
      if (s == "q") cancel("cancelled by user");
      else if (s == "r") { _offStep = 0; go(CalState::OffWait); }
      else if (s.length() == 0) go(CalState::Finish);
      else _prompted = false;
      break;

    case CalState::Finish:
      finish();
      break;
  }
}

// ---------------------------------------------------------------------------
// Idle commands
// ---------------------------------------------------------------------------

void MeterCalibrator::handleIdle() {
  String s;
  if (!readLine(s)) return;
  s.trim();
  s.toLowerCase();
  if (s == "cal") start();
  else if (s == "cal show") {
    _io->println("[cal] calibration values currently in use:");
    printBlock(_m->config().cal, false);
  }
  else if (s == "help" || s == "cal help") printHelp();
}

void MeterCalibrator::printHelp() {
  _io->println("Commands:");
  _io->println("  cal        start the calibration wizard");
  _io->println("  cal show   print the calibration values currently in use");
  _io->println("During the wizard: Enter accepts the [default], q cancels without changes.");
}

// Prints the values as C++ statements ready to paste into loadCalibration() in main.cpp.
void MeterCalibrator::printBlock(const EnergyMeterCalibration& c, bool withHardwareNote) {
  _io->println("------------------ copy from here ------------------");
  if (withHardwareNote) {
    _io->printf("  // Un=%g V  Ib=%g A  MC=%g imp/kWh  GL=x%g  Vi=%g mV  Vu=%g mV\n",
                _hwUn, _hwIb, _hwMc, _hwGl, _hwVi, _hwVu);
  }
  _io->printf("  cal.meter_constant = %u;\n", c.meter_constant);
  _io->printf("  cal.mmode          = 0x%04X;\n", c.mmode);
  _io->printf("  cal.pl_const_h     = 0x%04X;\n", c.pl_const_h);
  _io->printf("  cal.pl_const_l     = 0x%04X;\n", c.pl_const_l);
  _io->printf("  cal.lgain          = 0x%04X;\n", c.lgain);
  _io->printf("  cal.lphi           = 0x%04X;\n", c.lphi);
  _io->printf("  cal.ugain          = 0x%04X;\n", c.ugain);
  _io->printf("  cal.igain_l        = 0x%04X;\n", c.igain_l);
  _io->printf("  cal.sag_th         = 0x%04X;\n", c.sag_th);
  _io->printf("  cal.pstart_th      = 0x%04X;\n", c.pstart_th);
  _io->printf("  cal.qstart_th      = 0x%04X;\n", c.qstart_th);
  _io->printf("  cal.ioffset_l      = 0x%04X;\n", c.ioffset_l);
  _io->printf("  cal.poffset_l      = 0x%04X;\n", c.poffset_l);
  _io->printf("  cal.qoffset_l      = 0x%04X;\n", c.qoffset_l);
  _io->println("------------------ copy to here --------------------");
}

// ---------------------------------------------------------------------------
// Wizard control
// ---------------------------------------------------------------------------

void MeterCalibrator::start() {
  if (!_m->running()) {
    _io->println("[cal] meter is not running yet - fix the SPI/meter errors first");
    return;
  }
  _ccBackup = _m->config();
  _cc = _ccBackup;
  _cc.v_trim = _cc.i_trim = _cc.p_trim = 1.0f;
  _cc.small_p_mod = 0;
  _origPoll = _cc.poll_interval_ms;
  _cc.poll_interval_ms = kCalPollMs;
  _phSign = 1;
  _phFlipTried = false;

  _io->println("\n=========== M90E26 calibration (AN46102) ===========");
  _io->println("You will need: true-RMS voltmeter, ammeter and wattmeter (reference),");
  _io->println("a resistive load near Ib, and the sampling voltages at the IC pins.");
  _io->println("Enter accepts the [default]; q cancels and restores the previous registers.");
  _io->println("N line (no CT) is left untouched. Nothing is written to flash.\n");
  go(CalState::AskUn);
}

void MeterCalibrator::go(CalState s) {
  _cs = s;
  _prompted = false;
  _csEnterMs = millis();
  if (s == CalState::Sampling) startSampling();
  if (s == CalState::RmsAskV) {
    _io->println("\n=== Step 1: voltage / current rms gain ===");
    _io->println("Connect a resistive load (PF = 1) drawing about Ib, with reference");
    _io->println("instruments (true-RMS voltmeter, ammeter, wattmeter) on the same circuit.");
  }
  if (s == CalState::LgAskP) {
    _io->println("\n=== Step 2: energy gain (Lgain) at unity power factor ===");
    _io->println("Keep the same resistive load. The error is measured from the IC's ENERGY");
    _io->println("registers against your reference power over a time window (AN46102 3.2.4.2).");
  }
}

void MeterCalibrator::cancel(const char* why) {
  _io->printf("\n[cal] CANCELLED: %s\n", why);
  _io->println("[cal] previous registers restored, no values to copy");
  _m->reconfigure(_ccBackup);
  _m->resetEnergy();
  go(CalState::Idle);
}

void MeterCalibrator::apply(CalState next) {
  _m->reconfigure(_cc);
  _csAfterReady = next;
  go(CalState::WaitReady);
}

void MeterCalibrator::armSampling(CalState after, bool energyWindow) {
  _csAfterSample = after;
  _smpEnergy = energyWindow;
}

// ---------------------------------------------------------------------------
// Hardware parameters: PL_Constant (3.2.2), MMode gain (3.2.3), start thresholds (3.2.4.4)
//   PL_Constant = int( 838860800 * GL * VL * VU / (MC * Un * Ib) ), multiple of 4
//   PStartTh    = int( 93.2067556 * GL * VL * VU * KS )
// ---------------------------------------------------------------------------

void MeterCalibrator::applyHardwareParams() {
  // ADC input ranges (datasheet): current channel 600 mV / GL rms, voltage channel 600 mV rms.
  double iMax = 600.0 / _hwGl;
  double iHead = iMax / _hwVi;
  double uHead = 600.0 / _hwVu;
  _io->printf("\nCurrent channel: %.3f mV at Ib, ADC limit %.1f mV rms -> clips above about %.1f x Ib\n",
              _hwVi, iMax, iHead);
  _io->printf("Voltage channel: %.1f mV at Un, ADC limit 600 mV rms -> clips above about %.0f V\n",
              _hwVu, _hwUn * uHead);
  if (iHead < 1.0) {
    cancel("Vi exceeds the current-channel ADC range at this gain - pick a lower GL or a smaller burden");
    return;
  }
  if (uHead < 1.0) {
    cancel("Vu exceeds the 600 mV voltage-channel ADC range - increase the divider ratio");
    return;
  }
  if (iHead < 2.0) _io->println("  warning: little current headroom - the ADC clips just above Ib");
  if (uHead < 1.3) _io->println("  warning: little voltage headroom - the ADC clips on a mild over-voltage");

  double pl = kPlNumerator * _hwGl * _hwVi * _hwVu / (_hwMc * _hwUn * _hwIb);
  if (pl < 4.0 || pl > 4294967295.0) {
    cancel("PL_Constant out of 32-bit range - re-check Vi, Vu, Un, Ib and MC");
    return;
  }
  uint32_t plv = (uint32_t)pl;
  plv &= ~3UL;   // datasheet: set PL_Constant as a multiple of 4

  double ps = kStartK * _hwGl * _hwVi * _hwVu * kStartKsActive;
  double qs = kStartK * _hwGl * _hwVi * _hwVu * kStartKsReactive;
  if (ps > 65535.0 || qs > 65535.0) {
    cancel("start-up threshold out of range - re-check Vi, Vu and GL");
    return;
  }

  _cc.cal.pl_const_h = (uint16_t)(plv >> 16);
  _cc.cal.pl_const_l = (uint16_t)(plv & 0xFFFF);
  _cc.cal.meter_constant = (uint16_t)_hwMc;
  _cc.cal.mmode = (uint16_t)((_cc.cal.mmode & 0x1FFF) | ((uint16_t)gainCode((int)_hwGl) << 13));
  _cc.cal.pstart_th = (uint16_t)lround(ps);
  _cc.cal.qstart_th = (uint16_t)lround(qs);

  _io->printf("PL_Constant = %lu (0x%08lX)  ->  PLconstH 0x%04X, PLconstL 0x%04X\n",
              (unsigned long)plv, (unsigned long)plv, _cc.cal.pl_const_h, _cc.cal.pl_const_l);
  _io->printf("MMode = 0x%04X (L line gain x%d)   PStartTh 0x%04X   QStartTh 0x%04X\n",
              _cc.cal.mmode, (int)_hwGl, _cc.cal.pstart_th, _cc.cal.qstart_th);
  _io->println("Applying to the IC...");
  apply(CalState::RmsAskV);
}

// ---------------------------------------------------------------------------
// Step 1: rms gains (3.2.4.5)
//   Ugain = Ugain_old * Un_actual / Vol_mea      (power-on 26400; bit 15 must stay 0)
//   Igain = Igain_old * Ib_actual / Cur_mea      (power-on 31251)
//   SagTh = Un*100*sqrt(2)*0.78 / (4*Ugain/2^15) (2.3.2, 78 % of Un)
// ---------------------------------------------------------------------------

void MeterCalibrator::rmsApply() {
  if (_avgV < 5.0 || _avgI < 0.005) {
    cancel("IC reads ~0 V / A - check the load, CT and sensing connections");
    return;
  }
  double ug = floor(_cc.cal.ugain * _refV / _avgV + 0.5);
  if (ug < 1.0 || ug > 32767.0) {
    cancel("Ugain would exceed 0x7FFF - increase the voltage divider attenuation");
    return;
  }
  double ig = floor(_cc.cal.igain_l * _refI / _avgI + 0.5);
  if (ig < 1.0 || ig > 65535.0) {
    cancel("IgainL out of range - revisit the CT burden / PGA gain");
    return;
  }
  uint16_t oldU = _cc.cal.ugain, oldI = _cc.cal.igain_l;
  _cc.cal.ugain = (uint16_t)ug;
  _cc.cal.igain_l = (uint16_t)ig;

  double sag = _hwUn * 100.0 * sqrt(2.0) * 0.78 / (4.0 * _cc.cal.ugain / 32768.0);
  _cc.cal.sag_th = (uint16_t)(sag > 65535.0 ? 65535.0 : (double)lround(sag));

  _io->printf("  Measured %.2f V, %.3f A\n", _avgV, _avgI);
  _io->printf("  Ugain  0x%04X -> 0x%04X\n  IgainL 0x%04X -> 0x%04X\n  SagTh  -> 0x%04X\n",
              oldU, _cc.cal.ugain, oldI, _cc.cal.igain_l, _cc.cal.sag_th);
  _io->println("Applying and verifying...");
  _io->print("Sampling");
  armSampling(CalState::RmsConfirm, false);
  apply(CalState::Sampling);
}

// ---------------------------------------------------------------------------
// Step 2: Lgain (3.2.4.2)
//   eps = (E_meter - E_true) / E_true  (energy from the IC's registers, E_true = P_ref * time)
//   L_RATIO = -eps / (1 + eps);  Lgain = L_RATIO * 2^15, two's complement, limited to +-50 %
//   applied as a correction on the current Lgain so repeats converge:
//   G_new = G_old / (1 + eps)
// ---------------------------------------------------------------------------

void MeterCalibrator::lgApply() {
  double eps = _epsE;
  if (1.0 + eps < 0.05) {
    cancel("energy error is extreme - re-check the PL_Constant inputs and the reference power");
    return;
  }
  double gOld = 1.0 + (double)(int16_t)_cc.cal.lgain / 32768.0;
  double gNew = gOld / (1.0 + eps);
  long lg = lround((gNew - 1.0) * 32768.0);
  if (lg < -kLgainLimit || lg > kLgainLimit) {
    cancel("Lgain would exceed the +-50 % range of AN46102 - re-check GL, Vi, Vu, Un, Ib, MC and the reference power");
    return;
  }
  uint16_t oldL = _cc.cal.lgain;
  _cc.cal.lgain = (uint16_t)(int16_t)lg;

  _io->printf("  Energy error: %+.3f %%  (%.0f s, %lld counts)   Lgain 0x%04X -> 0x%04X\n",
              eps * 100.0, _ewDt / 1000.0, (long long)_ewCounts, oldL, _cc.cal.lgain);
  _io->println("Applying and verifying with a second energy window...");
  printEnergyEstimate(_ewRefP);
  armSampling(CalState::LgConfirm, true);
  apply(CalState::Sampling);
}

// ---------------------------------------------------------------------------
// Step 3: Lphi (3.2.4.3) at PF 0.5L, Ib, 50 Hz
//   Lphi = eps_L * 3763.74, sign-magnitude (MSB = sign); added to the existing value on repeats
// ---------------------------------------------------------------------------

void MeterCalibrator::phaseApply() {
  double eps = _phUseStored ? _phEpsBefore : _epsE;
  _phUseStored = false;
  _phEpsBefore = eps;
  _phOld = lphiDecode(_cc.cal.lphi);

  long nv = (long)_phOld + lround(_phSign * eps * kPhaseK);
  if (nv < -kLphiLimit || nv > kLphiLimit) {
    cancel("Lphi out of range - check the load power factor, the CT and the reference power");
    return;
  }
  uint16_t oldReg = _cc.cal.lphi;
  _cc.cal.lphi = lphiEncode(nv);
  _io->printf("  Energy error at PF 0.5L: %+.3f %%   Lphi 0x%04X -> 0x%04X (%+ld)\n",
              eps * 100.0, oldReg, _cc.cal.lphi, nv);
  _io->println("Applying and verifying with a second energy window...");
  printEnergyEstimate(_ewRefP);
  armSampling(CalState::PhaseConfirm, true);
  apply(CalState::Sampling);
}

// ---------------------------------------------------------------------------
// Step 4: no-load offsets (3.2.4.1 / 3.2.4.5)
//   PoffsetL / QoffsetL = complement of the mean Pmean / Qmean read in small-power mode (0xA987)
//   IoffsetL = complement of ( Irms_reading * IgainL / 2^16 * 2^8 )
// ---------------------------------------------------------------------------

void MeterCalibrator::offRun() {
  switch (_offStep) {
    case 0:   // baseline: offsets cleared, normal mode, measure the no-load current
      _cc.cal.ioffset_l = _cc.cal.poffset_l = _cc.cal.qoffset_l = 0;
      _cc.small_p_mod = 0;
      _offStep = 1;
      _io->print("Measuring no-load current");
      armSampling(CalState::OffRun, false);
      apply(CalState::Sampling);
      break;

    case 1:   // store Irms, then measure power in small-power mode
      _offIraw = lround(_avgI * 1000.0);
      _cc.small_p_mod = kSmallPowerKey;
      _offStep = 2;
      _io->printf("\n  no-load Irms = %ld mA\nMeasuring power in small-power mode", _offIraw);
      armSampling(CalState::OffRun, false);
      apply(CalState::Sampling);
      break;

    case 2: { // compute offsets, return to normal mode, verify
      long po = -lround(_avgP);
      long qo = -lround(_avgQ);
      long io = (_offIraw >= 1) ? -lround((double)_offIraw * _cc.cal.igain_l / 256.0) : 0;
      if (po < -32768 || po > 32767 || qo < -32768 || qo > 32767 || io < -32768 || io > 32767) {
        cancel("offset out of range - is the load really off (current circuit idle)?");
        return;
      }
      _cc.cal.poffset_l = (uint16_t)(int16_t)po;
      _cc.cal.qoffset_l = (uint16_t)(int16_t)qo;
      _cc.cal.ioffset_l = (uint16_t)(int16_t)io;
      _cc.small_p_mod = 0;
      _io->printf("  PoffsetL 0x%04X (%+ld)  QoffsetL 0x%04X (%+ld)  IoffsetL 0x%04X (%+ld)\n",
                  _cc.cal.poffset_l, po, _cc.cal.qoffset_l, qo, _cc.cal.ioffset_l, io);
      _io->print("Applying and verifying");
      _offStep = 3;
      armSampling(CalState::OffConfirm, false);
      apply(CalState::Sampling);
      break;
    }

    default:
      go(CalState::Finish);
      break;
  }
}

// ---------------------------------------------------------------------------
// Finish: keep the values live until reboot and print them for main.cpp
// ---------------------------------------------------------------------------

void MeterCalibrator::finish() {
  _cc.small_p_mod = 0;
  _cc.poll_interval_ms = _origPoll;
  _m->reconfigure(_cc);
  _m->resetEnergy();                       // energy counted during calibration is meaningless

  _io->println("\n=========== Calibration complete ===========");
  _io->println("Paste this block into loadCalibration() in main.cpp, replacing the template values.");
  printBlock(_cc.cal, true);
  _io->println("The values are active now but are NOT stored: after a reboot the values in");
  _io->println("main.cpp are used, so paste them there and re-flash.\n");
  go(CalState::Idle);
}

// ---------------------------------------------------------------------------
// Sampling: rms averages (10 snapshots) or an energy-error window
// ---------------------------------------------------------------------------

void MeterCalibrator::printEnergyEstimate(double refP) {
  double whPerLsb = 100.0 / (double)_cc.cal.meter_constant;      // 0.1 pulse per LSB
  double est = (refP > 0) ? ((double)kEwMinCounts * whPerLsb / refP * 3600.0) : 0;
  if (est < kEwMinMs / 1000.0) est = kEwMinMs / 1000.0;
  _io->printf("Measuring energy for about %.0f s - keep the load steady", est);
  if (est > kEwMaxMs / 1000.0)
    _io->printf("\n  warning: the load is small for this meter constant; accuracy will be limited");
}

void MeterCalibrator::startSampling() {
  _smpCnt = 0;
  _smpSkip = 2;                            // let RMS/power registers settle after a reconfigure
  _sumV = _sumI = _sumP = _sumQ = _sumF = 0;
  _ewStarted = false;
  _ewDots = 0;
  _smpLastTs = _m->readings().timestamp_ms;
}

bool MeterCalibrator::sampleStep() {
  if (!_m->running()) return false;
  const EnergyReadings& r = _m->readings();
  if (r.timestamp_ms == _smpLastTs) return false;
  _smpLastTs = r.timestamp_ms;
  if (_smpSkip) { _smpSkip--; return false; }

  int64_t net = (int64_t)_m->_cntAP - (int64_t)_m->_cntAN;     // active energy, 0.1-pulse LSBs

  if (_smpEnergy && !_ewStarted) {         // the window opens at this cycle boundary
    _ewStarted = true;
    _ewT0 = r.timestamp_ms;
    _ewC0 = net;
    return false;
  }

  _sumV += r.voltage_V;
  _sumI += r.current_A;
  _sumP += r.active_power_W;
  _sumQ += r.reactive_power_var;
  _sumF += r.frequency_Hz;
  _smpCnt++;

  if (!_smpEnergy) {
    _io->print('.');
    if (_smpCnt < kSamples) return false;
  } else {
    uint32_t dt = r.timestamp_ms - _ewT0;
    int64_t counts = net - _ewC0;
    if (dt / 5000UL > _ewDots) { _ewDots = dt / 5000UL; _io->print('.'); }
    bool enough = (dt >= kEwMinMs) && (counts >= kEwMinCounts);
    if (!enough && dt < kEwMaxMs) return false;
    if (counts < kEwAbortCounts) {
      cancel("not enough energy counted - increase the load, or check PL_Constant / MC / the CT connection");
      return false;
    }
    double eTrue = _ewRefP * (double)dt / 3600000.0;                              // Wh
    double eMeter = (double)counts * 100.0 / (double)_m->config().cal.meter_constant;  // Wh
    _epsE = eMeter / eTrue - 1.0;
    _ewDt = dt;
    _ewCounts = counts;
  }

  _avgV = _sumV / _smpCnt;
  _avgI = _sumI / _smpCnt;
  _avgP = _sumP / _smpCnt;
  _avgQ = _sumQ / _smpCnt;
  _avgF = _sumF / _smpCnt;
  _io->println();
  return true;
}

// ---------------------------------------------------------------------------
// Input helpers
// ---------------------------------------------------------------------------

// Non-blocking line reader; handles CR, LF and CRLF. An empty line (Enter) is a valid result.
bool MeterCalibrator::readLine(String& out) {
  while (_io->available()) {
    char c = (char)_io->read();
    if (c == '\n' && _lastCR) { _lastCR = false; continue; }
    _lastCR = (c == '\r');
    if (c == '\r' || c == '\n') {
      out = _line;
      _line = "";
      out.trim();
      return true;
    }
    if (_line.length() < 48) _line += c;
  }
  return false;
}

// Returns true when a valid number has been entered. Enter accepts def (if it is not NAN).
bool MeterCalibrator::askNumber(const char* text, double def, double lo, double hi, double& out) {
  if (!_prompted) {
    if (isnan(def)) _io->printf("%s: ", text);
    else _io->printf("%s [%g]: ", text, def);
    _prompted = true;
  }
  String s;
  if (!readLine(s)) return false;

  String low = s;
  low.toLowerCase();
  if (low == "q") { cancel("cancelled by user"); return false; }

  double v;
  if (s.length() == 0) {
    if (isnan(def)) { _io->println("  a value is required"); _prompted = false; return false; }
    v = def;
  } else {
    char* end = nullptr;
    v = strtod(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0') { _io->println("  not a number"); _prompted = false; return false; }
  }
  if (v < lo || v > hi) {
    _io->printf("  out of range (%g .. %g)\n", lo, hi);
    _prompted = false;
    return false;
  }
  _io->printf("  -> %g\n", v);
  out = v;
  return true;
}

bool MeterCalibrator::askYesNo(const char* text, bool& yes) {
  if (!_prompted) {
    _io->printf("%s: ", text);
    _prompted = true;
  }
  String s;
  if (!readLine(s)) return false;
  s.toLowerCase();
  if (s == "q") { cancel("cancelled by user"); return false; }
  if (s == "y") { yes = true; return true; }
  if (s == "n") { yes = false; return true; }
  _prompted = false;
  return false;
}