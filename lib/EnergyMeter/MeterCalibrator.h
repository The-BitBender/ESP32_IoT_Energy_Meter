#pragma once
#include <Arduino.h>
#include "EnergyMeterTypes.h"

class EnergyMeter;

/**
 * UART calibration wizard for the M90E26, following AN46102 section 3.
 *
 * INTERNAL to the EnergyMeter module: EnergyMeter owns an instance and drives it
 * from EnergyMeter::update(). Application code never includes or uses this class;
 * it only calls EnergyMeter::enableCalibration() and EnergyMeter::calibrating().
 *
 * Nothing is stored in flash: the results are printed as a block to paste into
 * the calibration template in main.cpp.
 *
 * Steps:  hardware parameters -> PL_Constant, PStartTh/QStartTh, MMode (3.2.2-3.2.4.4)
 *         1. Urms / Irms gain (3.2.4.5)         at PF = 1
 *         2. Lgain from ENERGY error (3.2.4.2)  at PF = 1
 *         3. Lphi  from ENERGY error (3.2.4.3)  at PF = 0.5 inductive (optional)
 *         4. no-load offsets (3.2.4.1, 3.2.4.5) (optional)
 */
class MeterCalibrator {
 public:
  void begin(EnergyMeter& meter, Stream& io);
  void update();                                     // non-blocking
  bool active() const { return _cs != CalState::Idle; }

 private:
  enum class CalState : uint8_t {
    Idle,
    AskUn, AskIb, AskMC, AskGL, AskVi, AskVu,
    WaitReady, Sampling,
    RmsAskV, RmsAskI, RmsApply, RmsConfirm,
    LgAskP, LgApply, LgConfirm,
    PhaseAsk, PhaseAskV, PhaseAskI, PhaseAskP, PhaseApply, PhaseConfirm,
    OffAsk, OffWait, OffRun, OffConfirm,
    Finish
  };

  void handleIdle();
  void start();
  void go(CalState s);
  void cancel(const char* why);
  void apply(CalState next);
  void armSampling(CalState after, bool energyWindow);
  void startSampling();
  bool sampleStep();
  bool readLine(String& out);
  bool askNumber(const char* text, double def, double lo, double hi, double& out);
  bool askYesNo(const char* text, bool& yes);
  void applyHardwareParams();
  void rmsApply();
  void lgApply();
  void phaseApply();
  void offRun();
  void finish();
  void printEnergyEstimate(double refP);
  void printBlock(const EnergyMeterCalibration& c, bool withHardwareNote);
  void printHelp();

  EnergyMeter* _m = nullptr;
  Stream* _io = nullptr;
  CalState _cs = CalState::Idle;
  CalState _csAfterReady = CalState::Idle;
  CalState _csAfterSample = CalState::Idle;
  uint32_t _csEnterMs = 0;
  bool _prompted = false;

  EnergyMeterConfig _cc;        // working copy being calibrated
  EnergyMeterConfig _ccBackup;  // restored on cancel
  uint32_t _origPoll = 1000;

  String _line;
  bool _lastCR = false;

  // hardware parameters (AN46102 3.2.2)
  double _hwUn = 230, _hwIb = 0, _hwMc = 3200, _hwGl = 24, _hwVi = 0, _hwVu = 0;
  // reference readings typed by the user
  double _refV = 0, _refI = 0, _refP = 0;

  // sampling (rms averages, or an energy-error window)
  bool _smpEnergy = false;
  uint8_t _smpCnt = 0, _smpSkip = 0;
  uint32_t _smpLastTs = 0;
  double _sumV = 0, _sumI = 0, _sumP = 0, _sumQ = 0, _sumF = 0;
  double _avgV = 0, _avgI = 0, _avgP = 0, _avgQ = 0, _avgF = 0;

  // energy window
  bool _ewStarted = false;
  uint32_t _ewT0 = 0, _ewDt = 0, _ewDots = 0;
  int64_t _ewC0 = 0, _ewCounts = 0;
  double _ewRefP = 0;
  double _epsE = 0;             // energy error of the last window: E_meter / E_true - 1

  // phase step bookkeeping
  double _phEpsBefore = 0, _phSign = 1;
  int _phOld = 0;
  bool _phUseStored = false, _phFlipTried = false;

  // offset step bookkeeping
  uint8_t _offStep = 0;
  long _offIraw = 0;
};