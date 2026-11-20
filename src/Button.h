#pragma once
#include "config.h"

enum class BtnEvent : uint8_t { None, Next, Select };

// One button, two gestures:
//   short press (released before LONG_PRESS_MS) -> Next
//   hold (fires while still held)               -> Select
class Button {
 public:
  void begin(uint8_t pin) {
    _pin = pin;
    pinMode(_pin, INPUT_PULLUP);
  }

  BtnEvent poll() {
    const uint32_t now = millis();
    const bool raw = (digitalRead(_pin) == LOW);

    if (raw != _raw) { _raw = raw; _edgeAt = now; }

    if (raw != _stable && (now - _edgeAt) >= DEBOUNCE_MS) {
      _stable = raw;
      if (_stable) {
        _downAt = now;
        _longFired = false;
      } else if (!_longFired) {
        return BtnEvent::Next;              // released before the hold time
      }
    }

    if (_stable && !_longFired && (now - _downAt) >= LONG_PRESS_MS) {
      _longFired = true;
      return BtnEvent::Select;
    }
    return BtnEvent::None;
  }

 private:
  static constexpr uint32_t DEBOUNCE_MS = 25;
  uint8_t  _pin = 0;
  bool     _raw = false, _stable = false, _longFired = false;
  uint32_t _edgeAt = 0, _downAt = 0;
};