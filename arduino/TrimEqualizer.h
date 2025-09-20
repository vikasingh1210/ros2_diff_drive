#pragma once
#include <Arduino.h>

class TrimEqualizer {
public:
  // --- Tunables (constructor args) ---
  // trimLimit: max extra PWM added to each wheel (>=0)
  // trimKi   : integral step to grow trims (0.05..0.30 typical)
  // deadbTck : tick deadband per loop (ignore tiny differences)
  // minMoveT : ignore noise when neither wheel really moves
  // minCmd   : treat tiny commanded PWM as zero
  TrimEqualizer(
    int16_t trimLimit = 80,
    float   trimKi    = 0.15f,
    long    deadbTck  = 3,
    int16_t minMoveT  = 1,
    int16_t minCmd    = 10)
  : TRIM_LIMIT_(trimLimit), TRIM_KI_(trimKi),
    TICKS_DEADB_(deodbClamp(deadbTck)), MIN_MOVE_TCK_(minMoveT), MIN_CMD_PWM_(minCmd) {}

  // Call when a new PWM command is received
  // Returns true if direction sign of any wheel changed (trims auto-reset)
  bool onCommand(int16_t pwmL, int16_t pwmR) {
    baseL_ = (abs(pwmL) < MIN_CMD_PWM_) ? 0 : pwmL;
    baseR_ = (abs(pwmR) < MIN_CMD_PWM_) ? 0 : pwmR;

    int8_t sL = sgn(baseL_), sR = sgn(baseR_);
    bool signChanged = (sL != lastSignL_ && lastSignL_ != 0) ||
                       (sR != lastSignR_ && lastSignR_ != 0);
    if (signChanged) resetTrims();

    lastSignL_ = sL; lastSignR_ = sR;
    return signChanged;
  }

  // Update trims using measured tick deltas over the loop interval.
  // dL/dR = signed tick increments since previous loop.
  // Returns true if trims changed this call.
  bool update(long dL, long dR) {
    long sL = labs(dL), sR = labs(dR);
    int  aL = abs(baseL_), aR = abs(baseR_);

    bool moving = (aL > 0 || aR > 0) && (sL > MIN_MOVE_TCK_ || sR > MIN_MOVE_TCK_);
    bool changed = false;

    if (moving) {
      if (aL == 0 && aR > 0) {
        // Only right commanded — bleed left trim
        changed |= bleedOne_(trimL_);
      } else if (aR == 0 && aL > 0) {
        // Only left commanded — bleed right trim
        changed |= bleedOne_(trimR_);
      } else if (aL > 0 && aR > 0) {
        // Ratio target: sL/sR ≈ aL/aR  =>  aR*sL ? aL*sR
        long lhs = (long)aR * sL;
        long rhs = (long)aL * sR;
        long err = rhs - lhs; // >0 => left is slower than desired ratio

        if (err > TICKS_DEADB_) {
          changed |= grow_(trimL_, (int)(TRIM_KI_ * err / max(1, aR)));
        } else if (err < -TICKS_DEADB_) {
          long e = -err;
          changed |= grow_(trimR_, (int)(TRIM_KI_ * e / max(1, aL)));
        }
      }
    } else {
      // Not moving — bleed both trims gently toward zero
      changed |= bleedOne_(trimL_);
      changed |= bleedOne_(trimR_);
    }
    return changed;
  }

  // Compute signed outputs by applying trim magnitudes with base sign
  // outL/outR are constrained to [-255, 255]
  void apply(int16_t& outL, int16_t& outR) const {
    int8_t sL = sgn(baseL_), sR = sgn(baseR_);
    long oL = (long)baseL_ + (long)sL * trimL_;
    long oR = (long)baseR_ + (long)sR * trimR_;
    outL = (int16_t)constrain(oL, -255, 255);
    outR = (int16_t)constrain(oR, -255, 255);
  }

  // Accessors
  inline int16_t baseL() const { return baseL_; }
  inline int16_t baseR() const { return baseR_; }
  inline int16_t trimL() const { return trimL_; }  // magnitude (>=0)
  inline int16_t trimR() const { return trimR_; }
  inline void    resetTrims()  { trimL_ = 0; trimR_ = 0; }
  inline void    stop()        { baseL_ = baseR_ = 0; resetTrims(); }

private:
  static inline int8_t sgn(int16_t v){ return (v > 0) - (v < 0); }
  static inline long deodbClamp(long v){ return v < 0 ? 0 : v; }

  bool grow_(int16_t& t, int delta){
    int16_t before = t;
    long v = (long)t + (long)delta;
    if (v < 0) v = 0;
    if (v > TRIM_LIMIT_) v = TRIM_LIMIT_;
    t = (int16_t)v;
    return t != before;
  }
  bool bleedOne_(int16_t& t){
    if (t == 0) return false;
    t += (t > 0) ? -1 : +1;
    return true;
  }

  // Tunables
  const int16_t TRIM_LIMIT_;
  const float   TRIM_KI_;
  const long    TICKS_DEADB_;
  const int16_t MIN_MOVE_TCK_;
  const int16_t MIN_CMD_PWM_;

  // State
  int16_t baseL_ = 0, baseR_ = 0;     // commanded base PWM
  int16_t trimL_ = 0, trimR_ = 0;     // trim magnitudes (>=0)
  int8_t  lastSignL_ = 0, lastSignR_ = 0;
};
