#pragma once
#include <Arduino.h>
#include "M90E26.h"
#include "EnergyMeterTypes.h"   // EnergyMeterConfig, EnergyMeterCalibration, EnergyReadings
#include "MeterCalibrator.h"    // internal: the UART calibration wizard (not used by main.cpp)

class EnergyMeter {
 public:
  enum class State : uint8_t { Off, Init, Verify, Idle, Polling, Backoff };
  enum : uint8_t { kPollCount = 14, kMaxInitSteps = 40 };

  // Sets up SPI and starts the init sequence. Returns false only if SPI setup fails.
  bool begin(const EnergyMeterConfig& cfg);

  // Call every loop(). Never blocks; each call advances the state machines.
  void update();

  bool running() const { return _state == State::Idle || _state == State::Polling; }
  State state() const { return _state; }
  const char* stateName() const;

  // True once per completed measurement cycle.
  bool newData() {
    bool f = _newData;
    _newData = false;
    return f;
  }
  const EnergyReadings& readings() const { return _r; }
  const EnergyMeterConfig& config() const { return _cfg; }

  void resetEnergy();
  uint32_t errorCount() const { return _errCount; }
  const char* lastError() const { return _lastError; }

  // ---- Built-in UART calibration (AN46102) ------------------------------------------
  // Call once after begin(). Type "cal" on the given stream to start; results are
  // printed as a block to paste into main.cpp. Nothing is written to flash.
  void enableCalibration(Stream& io);
  bool calibrating() const { return _calib.active(); }  // true while the wizard runs

 private:
  friend class MeterCalibrator;   // the wizard re-applies register sets via reconfigure()

  struct InitStep {
    uint8_t reg;
    uint16_t value;
    uint16_t delay_ms;
  };

  void buildInitSequence();
  void addStep(uint8_t reg, uint16_t value, uint16_t delay_ms = 0);
  void startInit();
  void runInit();
  void runVerify();
  void runPoll();
  void finishCycle();
  void fault(const char* why);
  void reconfigure(const EnergyMeterConfig& cfg);

  EnergyMeterConfig _cfg;
  M90E26 _drv;
  State _state = State::Off;

  InitStep _seq[kMaxInitSteps];
  uint8_t _seqLen = 0;
  uint8_t _step = 0;
  uint8_t _verifyStep = 0;
  uint8_t _pollIdx = 0;
  bool _opPending = false;

  uint32_t _gateUntilMs = 0;
  uint32_t _wakeAtMs = 0;
  uint32_t _cycleStartMs = 0;

  uint16_t _raw[kPollCount];
  uint64_t _cntAP = 0, _cntAN = 0, _cntRP = 0, _cntRN = 0;  // energy LSB counters

  EnergyReadings _r;
  bool _newData = false;
  uint32_t _errCount = 0;
  const char* _lastError = "";

  MeterCalibrator _calib;
};