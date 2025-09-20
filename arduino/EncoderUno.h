#pragma once
#include <Arduino.h>
#include <PinChangeInterrupt.h>

// Encoders: A on D2/D3 (INT0/INT1), B on any PCINT (e.g., D4, D12)
class EncoderUNO {
public:
  EncoderUNO(uint8_t pinA, uint8_t pinB, bool invert=false)
  : pinA_(pinA), pinB_(pinB), invert_(invert) {}

  void begin() {
    pinMode(pinA_, INPUT_PULLUP);
    pinMode(pinB_, INPUT_PULLUP);

    if (!slot0_) slot0_ = this;
    else if (!slot1_) slot1_ = this;

    if (pinA_ == 2)      attachInterrupt(digitalPinToInterrupt(2), isrA_2, CHANGE);
    else if (pinA_ == 3) attachInterrupt(digitalPinToInterrupt(3), isrA_3, CHANGE);
    else                 attachPCINT(digitalPinToPCINT(pinA_), isrA_PCINT, CHANGE);

    if (this == slot0_) attachPCINT(digitalPinToPCINT(pinB_), isrB_0, CHANGE);
    else                attachPCINT(digitalPinToPCINT(pinB_), isrB_1, CHANGE);
  }

  inline long ticks() const { noInterrupts(); long t = ticks_; interrupts(); return t; }
  inline void reset()       { noInterrupts(); ticks_ = 0;    interrupts(); }

  float rpmFromDelta(long dTicks, float dt, long ticks_per_rev) const {
    return (float)dTicks * (60.0f / (ticks_per_rev * dt));
  }

private:
  volatile long ticks_ = 0;
  uint8_t pinA_, pinB_; bool invert_;

  inline void edgeA() {
    bool A = digitalRead(pinA_), B = digitalRead(pinB_);
    int dir = (A == B) ? +1 : -1;
    if (invert_) dir = -dir;
    ticks_ += dir;
  }
  inline void edgeB() {
    bool A = digitalRead(pinA_), B = digitalRead(pinB_);
    int dir = (A != B) ? +1 : -1;
    if (invert_) dir = -dir;
    ticks_ += dir;
  }

  static EncoderUNO* slotForPinA(uint8_t pin) {
    if (slot0_ && slot0_->pinA_ == pin) return slot0_;
    if (slot1_ && slot1_->pinA_ == pin) return slot1_;
    return nullptr;
  }
  static EncoderUNO* slot0_; static EncoderUNO* slot1_;

  static void isrA_2() { if (auto s = slotForPinA(2)) s->edgeA(); }
  static void isrA_3() { if (auto s = slotForPinA(3)) s->edgeA(); }
  static void isrA_PCINT() { if (slot0_) slot0_->edgeA(); if (slot1_) slot1_->edgeA(); }
  static void isrB_0() { if (slot0_) slot0_->edgeB(); }
  static void isrB_1() { if (slot1_) slot1_->edgeB(); }
};

inline EncoderUNO* EncoderUNO::slot0_ = nullptr;
inline EncoderUNO* EncoderUNO::slot1_ = nullptr;
