#pragma once
#include <Arduino.h>

// Minimal L298 2-pin driver (sign-magnitude).
// Use two PWM-capable pins (e.g., 5/6, 9/10 on UNO).
// If pwmOnIn1=true: IN1 gets PWM for forward, IN2 LOW; reverse = IN2 gets PWM, IN1 LOW.
class L298 {
public:
  L298(uint8_t in1, uint8_t in2, bool pwmOnIn1 = true, uint8_t maxPwm = 255)
  : _in1(in1), _in2(in2), _pwmOnIn1(pwmOnIn1), _max(maxPwm) {}

  void begin() {
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    stop();
  }

  inline void stop() {
    analogWrite(_in1, 0);
    analogWrite(_in2, 0);
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
  }

  // Signed PWM: -255..+255
  void drive(int16_t pwm) {
    if (pwm == 0) { stop(); return; }
    int s = (pwm > 0) ? 1 : -1;
    int mag = abs(pwm);
    if (mag > _max) mag = _max;

    if (s > 0) {
      // forward
      if (_pwmOnIn1) {
        analogWrite(_in1, mag);
        analogWrite(_in2, 0);
        digitalWrite(_in2, LOW);
      } else {
        analogWrite(_in2, 0);
        digitalWrite(_in2, LOW);
        analogWrite(_in1, mag);
      }
    } else {
      // reverse
      if (_pwmOnIn1) {
        analogWrite(_in1, 0);
        digitalWrite(_in1, LOW);
        analogWrite(_in2, mag);
      } else {
        analogWrite(_in2, mag);
        analogWrite(_in1, 0);
        digitalWrite(_in1, LOW);
      }
    }
  }

private:
  uint8_t _in1, _in2;
  bool    _pwmOnIn1;
  uint8_t _max;
};
