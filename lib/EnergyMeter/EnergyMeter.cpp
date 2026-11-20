#include "EnergyMeter.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

using namespace M90E26Reg;
using Result = M90E26::Result;

namespace {

// Order of registers read in each measurement cycle.
enum PollIdx : uint8_t {
  P_URMS, P_IRMS, P_PMEAN, P_QMEAN, P_SMEAN, P_FREQ, P_PF, P_PANGLE,
  P_AP, P_AN, P_RP, P_RN, P_ENSTATUS, P_SYSSTATUS, P_COUNT
};

const uint8_t kPollRegs[P_COUNT] = {
  Urms, Irms, Pmean, Qmean, Smean, Freq, PowerF, Pangle,
  APenergy, ANenergy, RPenergy, RNenergy, EnStatus, SysStatus
};

// SysStatus[15:14] = CalErr, [13:12] = AdjErr; 0b11 = checksum error.
inline bool calFault(uint16_t sys) {
  return ((sys >> 14) & 0x3) == 0x3 || ((sys >> 12) & 0x3) == 0x3;
}

inline int16_t s16(uint16_t v) { return (int16_t)v; }                 // "Complement" registers (Pmean, Qmean, Smean)
inline int16_t sm16(uint16_t v) {                                    // "Signed, MSB is the sign bit" (PowerF, Pangle)
  int16_t m = (int16_t)(v & 0x7FFF);
  return (v & 0x8000) ? (int16_t)-m : m;
}

}  // namespace

static_assert((int)P_COUNT == (int)EnergyMeter::kPollCount, "poll table size mismatch");

// ===========================================================================
// Meter: public API
// ===========================================================================

bool EnergyMeter::begin(const EnergyMeterConfig& cfg) {
  _cfg = cfg;
  if (_cfg.cal.meter_constant == 0) {
    _lastError = "meter_constant must be > 0";
    return false;
  }
  if (!_drv.begin(_cfg.pins, _cfg.spi_host, _cfg.spi_hz)) {
    _lastError = "SPI bus init failed";
    return false;
  }
  memset(&_r, 0, sizeof(_r));
  memset(_raw, 0, sizeof(_raw));
  _cntAP = _cntAN = _cntRP = _cntRN = 0;
  _newData = false;
  buildInitSequence();
  startInit();
  return true;
}

void EnergyMeter::enableCalibration(Stream& io) { _calib.begin(*this, io); }

void EnergyMeter::update() {
  _calib.update();

  switch (_state) {
    case State::Off:
      break;

    case State::Backoff:
      if ((int32_t)(millis() - _wakeAtMs) >= 0) startInit();
      break;

    case State::Init:
      runInit();
      break;

    case State::Verify:
      runVerify();
      break;

    case State::Idle:
      if ((uint32_t)(millis() - _cycleStartMs) >= _cfg.poll_interval_ms) {
        _cycleStartMs = millis();
        _pollIdx = 0;
        _opPending = false;
        _state = State::Polling;
      }
      break;

    case State::Polling:
      runPoll();
      break;
  }
}

void EnergyMeter::resetEnergy() {
  _cntAP = _cntAN = _cntRP = _cntRN = 0;
  _r.energy_import_Wh = _r.energy_export_Wh = 0;
  _r.reactive_fwd_varh = _r.reactive_rev_varh = 0;
}

const char* EnergyMeter::stateName() const {
  switch (_state) {
    case State::Off:     return "Off";
    case State::Init:    return "Init";
    case State::Verify:  return "Verify";
    case State::Idle:    return "Idle";
    case State::Polling: return "Polling";
    case State::Backoff: return "Backoff";
  }
  return "?";
}

// Swap in a new register set and re-run the init sequence (non-blocking).
// Accumulated energy totals are kept.
void EnergyMeter::reconfigure(const EnergyMeterConfig& cfg) {
  if (_state == State::Off) return;
  // Let any in-flight SPI transfer (~150 us) finish before rebuilding the sequence.
  uint32_t t0 = micros();
  while (_drv.busy() && (uint32_t)(micros() - t0) < 3000) _drv.poll();
  _cfg = cfg;
  if (_cfg.cal.meter_constant == 0) _cfg.cal.meter_constant = 1;
  buildInitSequence();
  _newData = false;
  startInit();
}

// ===========================================================================
// Meter: init sequence (table-driven; executed one SPI write at a time)
// ===========================================================================

void EnergyMeter::addStep(uint8_t reg, uint16_t value, uint16_t delay_ms) {
  if (_seqLen < kMaxInitSteps) {
    _seq[_seqLen].reg = reg;
    _seq[_seqLen].value = value;
    _seq[_seqLen].delay_ms = delay_ms;
    _seqLen++;
  }
}

void EnergyMeter::buildInitSequence() {
  const EnergyMeterConfig& c = _cfg;
  _seqLen = 0;

  addStep(SoftReset, SOFT_RESET_KEY, 20);
  addStep(FuncEn, c.func_en);
  addStep(SagTh, c.cal.sag_th);
  addStep(SmallPMod, c.small_p_mod);        // 0xA987 only while calibrating offsets

  // ---- Metering calibration start-up registers 0x21..0x2B + CS1 ----
  addStep(CalStart, CAL_ENTER);
  const uint16_t calRegs[11] = {
    c.cal.pl_const_h, c.cal.pl_const_l, c.cal.lgain, c.cal.lphi, c.ngain, c.nphi,
    c.cal.pstart_th, c.pnol_th, c.cal.qstart_th, c.qnol_th, c.cal.mmode
  };
  for (uint8_t i = 0; i < 11; i++) addStep(PLconstH + i, calRegs[i]);
  addStep(CS1, M90E26::checksum(calRegs, 11));

  // ---- Measurement calibration registers 0x31..0x3A + CS2 ----
  addStep(AdjStart, CAL_ENTER);
  const uint16_t adjRegs[10] = {
    c.cal.ugain, c.cal.igain_l, c.igain_n, c.uoffset, c.cal.ioffset_l, c.ioffset_n,
    c.cal.poffset_l, c.cal.qoffset_l, c.poffset_n, c.qoffset_n
  };
  for (uint8_t i = 0; i < 10; i++) addStep(Ugain + i, adjRegs[i]);
  addStep(CS2, M90E26::checksum(adjRegs, 10));

  // ---- Leave calibration mode -> metering enabled ----
  addStep(CalStart, CAL_EXIT);
  addStep(AdjStart, CAL_EXIT, 20);  // let the IC validate checksums before verify
}

void EnergyMeter::startInit() {
  _step = 0;
  _verifyStep = 0;
  _opPending = false;
  _gateUntilMs = millis();
  _state = State::Init;
}

void EnergyMeter::runInit() {
  if (_opPending) {
    Result r = _drv.poll();
    if (r == Result::Busy) return;
    _opPending = false;
    if (r != Result::Done) {
      fault("SPI error during init");
      return;
    }
    _gateUntilMs = millis() + _seq[_step - 1].delay_ms;
  }

  if ((int32_t)(millis() - _gateUntilMs) < 0) return;  // inter-step delay

  if (_step >= _seqLen) {
    _verifyStep = 0;
    _state = State::Verify;
    return;
  }

  const InitStep& s = _seq[_step];
  if (!_drv.startWrite(s.reg, s.value)) {
    fault("SPI queue error during init");
    return;
  }
  _opPending = true;
  _step++;
}

void EnergyMeter::runVerify() {
  if (!_opPending) {
    uint8_t reg = (_verifyStep == 0) ? SagTh : SysStatus;
    if (!_drv.startRead(reg)) {
      fault("SPI queue error during verify");
      return;
    }
    _opPending = true;
    return;
  }

  uint16_t v = 0;
  Result r = _drv.poll(&v);
  if (r == Result::Busy) return;
  _opPending = false;
  if (r != Result::Done) {
    fault("SPI error during verify");
    return;
  }

  if (_verifyStep == 0) {
    // Write/readback sanity check: catches wiring, SPI-mode and power problems.
    if (v != _cfg.cal.sag_th) {
      fault("SPI readback mismatch (check wiring / SPI-mode select / power)");
      return;
    }
    _verifyStep = 1;
  } else {
    _r.sys_status = v;
    if (calFault(v)) {
      fault("IC reports calibration/adjust checksum error");
      return;
    }
    _cycleStartMs = millis();
    _state = State::Idle;
  }
}

// ===========================================================================
// Meter: measurement cycle
// ===========================================================================

void EnergyMeter::runPoll() {
  if (!_opPending) {
    if (!_drv.startRead(kPollRegs[_pollIdx])) {
      fault("SPI queue error during poll");
      return;
    }
    _opPending = true;
    return;
  }

  uint16_t v = 0;
  Result r = _drv.poll(&v);
  if (r == Result::Busy) return;
  _opPending = false;
  if (r != Result::Done) {
    fault("SPI error during poll");
    return;
  }

  _raw[_pollIdx++] = v;
  if (_pollIdx >= kPollCount) {
    finishCycle();
    return;
  }

  // Back-to-back: queue the next register immediately (still non-blocking).
  if (!_drv.startRead(kPollRegs[_pollIdx])) {
    fault("SPI queue error during poll");
    return;
  }
  _opPending = true;
}

void EnergyMeter::finishCycle() {
  // Floating MISO reads as all ones.
  if (_raw[P_URMS] == 0xFFFF && _raw[P_IRMS] == 0xFFFF && _raw[P_FREQ] == 0xFFFF) {
    fault("no response from IC (all 0xFFFF)");
    return;
  }
  // Config lost (brown-out, noise glitch)? Re-initialise.
  if (calFault(_raw[P_SYSSTATUS])) {
    fault("checksum error at runtime, re-initialising");
    return;
  }

  const EnergyMeterConfig& c = _cfg;

  _r.voltage_V           = _raw[P_URMS] * 0.01f * c.v_trim;
  _r.current_A           = _raw[P_IRMS] * 0.001f * c.i_trim;
  _r.active_power_W      = s16(_raw[P_PMEAN]) * 1.0f * c.p_trim;
  _r.reactive_power_var  = s16(_raw[P_QMEAN]) * 1.0f * c.p_trim;
  _r.apparent_power_VA   = s16(_raw[P_SMEAN]) * 1.0f * c.p_trim;
  _r.power_factor        = sm16(_raw[P_PF]) * 0.001f;
  _r.phase_angle_deg     = sm16(_raw[P_PANGLE]) * 0.1f;
  _r.frequency_Hz        = _raw[P_FREQ] * 0.01f;
  _r.en_status           = _raw[P_ENSTATUS];
  _r.sys_status          = _raw[P_SYSSTATUS];

  // Energy registers are read-and-clear: accumulate raw counts, convert on demand.
  _cntAP += _raw[P_AP];
  _cntAN += _raw[P_AN];
  _cntRP += _raw[P_RP];
  _cntRN += _raw[P_RN];
  const double whPerLsb = 100.0 / (double)c.cal.meter_constant;  // 0.1 pulse per LSB
  _r.energy_import_Wh   = (double)_cntAP * whPerLsb;
  _r.energy_export_Wh   = (double)_cntAN * whPerLsb;
  _r.reactive_fwd_varh  = (double)_cntRP * whPerLsb;
  _r.reactive_rev_varh  = (double)_cntRN * whPerLsb;

  _r.timestamp_ms = millis();
  _newData = true;
  _state = State::Idle;
}

void EnergyMeter::fault(const char* why) {
  _lastError = why;
  _errCount++;
  _opPending = false;
  _wakeAtMs = millis() + _cfg.retry_delay_ms;
  _state = State::Backoff;
}