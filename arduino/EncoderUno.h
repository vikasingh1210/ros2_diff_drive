#pragma once
#include <Arduino.h>

// Quadrature encoder reader for Arduino UNO using external interrupts on pin A.
// Works best when pinA is 2 or 3 (INT0/INT1). pinB can be any digital pin.
// Counts +/- 1 per A transition (x2 decoding). Direction is derived from B.
class EncoderUNO {
public:
  EncoderUNO(uint8_t pinA, uint8_t pinB)
  : _pinA(pinA), _pinB(pinB) {}

  void begin() {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    noInterrupts();
    if (!_inst0) { _inst0 = this; attachInterrupt(digitalPinToInterrupt(_pinA), _isr0, CHANGE); }
    else if (!_inst1) { _inst1 = this; attachInterrupt(digitalPinToInterrupt(_pinA), _isr1, CHANGE); }
    interrupts();
  }

  inline long ticks() const {
    noInterrupts();
    long t = _ticks;
    interrupts();
    return t;
  }

  inline void reset() {
    noInterrupts();
    _ticks = 0;
    interrupts();
  }

private:
  volatile long _ticks = 0;
  uint8_t _pinA, _pinB;

  void _handle() {
    // Read both channels and decide direction.
    // On A transition: if A == B → -1 else +1 (CW/CCW depends on wiring).
    int a = digitalRead(_pinA);
    int b = digitalRead(_pinB);
    int8_t dir = (a == b) ? -1 : +1;
    _ticks += dir;
  }

  static EncoderUNO* _inst0;
  static EncoderUNO* _inst1;

  static void _isr0() { if (_inst0) _inst0->_handle(); }
  static void _isr1() { if (_inst1) _inst1->_handle(); }
};

// static storage
EncoderUNO* EncoderUNO::_inst0 = nullptr;
EncoderUNO* EncoderUNO::_inst1 = nullptr;
