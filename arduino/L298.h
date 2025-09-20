#pragma once
#include <Arduino.h>

// L298 with enable pins tied HIGH; drive via IN1/IN2 pairs.
// We PWM one leg and drive the other LOW/HIGH to set direction.
class L298 {
public:
  L298(uint8_t in1, uint8_t in2, bool pwmOnIn1 = true, uint8_t pwmMax = 255)
  : in1_(in1), in2_(in2), pwmOnIn1_(pwmOnIn1), pwmMax_(pwmMax) {}

  void begin() {
    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);
    stop();
  }

  // pwm: -255..+255
  void drive(int16_t pwm) {
    pwm = constrain(pwm, -(int)pwmMax_, (int)pwmMax_);
    uint8_t duty = (uint8_t)abs(pwm);

    if (pwm > 0) {
      if (pwmOnIn1_) { analogWrite(in1_, duty); digitalWrite(in2_, LOW);  }
      else           { digitalWrite(in1_, HIGH); analogWrite(in2_, 255 - duty); }
    } else if (pwm < 0) {
      if (pwmOnIn1_) { digitalWrite(in1_, LOW);  analogWrite(in2_, duty); }
      else           { analogWrite(in1_, 255 - duty); digitalWrite(in2_, HIGH); }
    } else {
      brakeFast();
    }
  }

  void stop()      { analogWrite(in1_, 0); analogWrite(in2_, 0); }
  void brakeFast() { digitalWrite(in1_, HIGH); digitalWrite(in2_, HIGH); }

private:
  uint8_t in1_, in2_; bool pwmOnIn1_; uint8_t pwmMax_;
};
