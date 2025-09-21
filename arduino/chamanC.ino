#include <Arduino.h>
#include "EncoderUno.h"   // class: EncoderUNO
#include "L298.h"         // .begin(), .drive(int16_t)

/* ---------------- Pinout (same as your repo) ---------------- */
const uint8_t L_IN1 = 5;
const uint8_t L_IN2 = 6;
const uint8_t R_IN3 = 9;
const uint8_t R_IN4 = 10;
const uint8_t L_ENC_A = 2, L_ENC_B = 4;
const uint8_t R_ENC_A = 3, R_ENC_B = 7;

/* ---------------- Direction & polarity (your current settings) ---------------- */
int8_t MOTOR_L_SIGN   =  1;
int8_t MOTOR_R_SIGN   =  1;
int8_t ENCODER_L_SIGN = -1;
int8_t ENCODER_R_SIGN =  1;

/* ---------------- Motion & control tuning ---------------- */
static const uint16_t CONTROL_MS     = 20;      // ~50 Hz
static const int16_t  MAX_PWM        = 140;     // top speed (leave 200; L298 clamps to 255 internally)
static const int16_t  MIN_PWM        = 45;      // to overcome stiction
static const long     RAMP_TICKS_HI  = 2200;    // far from goal -> use MAX
static const long     RAMP_TICKS_LO  = 800;     // close to goal -> use MIN
static const long     STOP_DEADBAND  = 10;      // a little larger deadband helps

// --- velocity loop (inner) ---
static const float    TPR            = 2048.0f; // set your true ticks/rev
static const int16_t  RPM_MAX_CMD    = 220;
static const float    RPM_ALPHA      = 0.5f;    // more smoothing (0.4→0.5)
static const float    KI_VEL_L       = 0.8f;    // slightly gentler I (0.9→0.8)
static const float    KI_VEL_R       = 0.8f;
static const int16_t  TRIM_CAP       = 200;     // cap trims within MAX_PWM range

// feed-forward: PWM ≈ offs + slope*|RPM|
static const float    SLOPE_FWD_L    = 0.25f, SLOPE_FWD_R = 0.25f;
static const float    SLOPE_REV_L    = 0.26f, SLOPE_REV_R = 0.18f;
static const int16_t  OFFS_FWD_L     = 10,     OFFS_FWD_R = 10;
static const int16_t  OFFS_REV_L     = 12,     OFFS_REV_R = 14;

// straightness equalization
static const int      STRAIGHT_TOL_RPM = 20;
static const float    K_EQUALIZE       = 0.10f;

// direction-change helpers
static const int16_t  KICK_PWM_DIRCHG  = 12;
static const int      FREEZE_TICKS     = 3;

// --- position loop (outer, for TICKS mode) ---
static const float    K_POS_TO_RPM    = 0.04f;  // RPM per tick error
static const int16_t  POS_RPM_MAX     = 80;
static const int16_t  POS_RPM_MIN     = 25;

/* ---------------- Objects ---------------- */
EncoderUNO encL(L_ENC_A, L_ENC_B);
EncoderUNO encR(R_ENC_A, R_ENC_B);
L298       motorL(L_IN1, L_IN2, /*pwmOnIn1=*/true, 255);
L298       motorR(R_IN3, R_IN4, /*pwmOnIn1=*/true, 255);

/* ---------------- State ---------------- */
enum Mode { MODE_IDLE=0, MODE_RPM=1, MODE_TICKS=2 };
Mode mode = MODE_IDLE;

unsigned long lastCtl = 0;

// logical tick history (after ENCODER_*_SIGN)
long lastTickL = 0, lastTickR = 0;

// TICKS mode
long startL = 0, startR = 0;
long goalL  = 0, goalR  = 0;
bool movingL = false, movingR = false;
long prevRemL = 0, prevRemR = 0;

// inner loop state
float rpmCmdL = 0.f, rpmCmdR = 0.f;
float rpmMeasL_f = 0.f, rpmMeasR_f = 0.f;
int16_t trimL = 0, trimR = 0;
int freezeCtr = 0;
int lastCmdSign = 0; // +1 fwd, -1 rev, 0 idle

/* ---------------- Helpers ---------------- */
static inline int16_t clamp255(int v) { return (v>255)?255:((v<-255)?-255:v); }
static inline int16_t pwm_for_remaining(long rem_abs) {
  if (rem_abs <= RAMP_TICKS_LO) return MIN_PWM;
  if (rem_abs >= RAMP_TICKS_HI) return MAX_PWM;
  long span = RAMP_TICKS_HI - RAMP_TICKS_LO;
  long off  = rem_abs - RAMP_TICKS_LO;
  long val  = MIN_PWM + (off * (MAX_PWM - MIN_PWM)) / (span > 0 ? span : 1);
  if (val < MIN_PWM) val = MIN_PWM;
  if (val > MAX_PWM) val = MAX_PWM;
  return (int16_t)val;
}
static inline int sgnl(long x){ return (x>0) - (x<0); }
static inline int sgnf(float x){ return (x>0.f) - (x<0.f); }

int16_t basePwmForSide(float rpm, bool left){
  if (rpm == 0.f) return 0;
  const bool fwd = rpm > 0.f;
  const float mag = fabs(rpm);
  float slope = left ? (fwd? SLOPE_FWD_L : SLOPE_REV_L)
                     : (fwd? SLOPE_FWD_R : SLOPE_REV_R);
  int16_t offs = left ? (fwd? OFFS_FWD_L : OFFS_REV_L)
                      : (fwd? OFFS_FWD_R : OFFS_REV_R);
  int s = fwd ? 1 : -1;
  int16_t pwm = (int16_t)(s*offs + slope*mag);
  if (abs(pwm) < MIN_PWM) pwm = s*MIN_PWM;
  return clamp255(pwm);
}

void stopAll() {
  movingL = movingR = false;
  rpmCmdL = rpmCmdR = 0.f;
  trimL = trimR = 0;
  motorL.drive(0);
  motorR.drive(0);
  mode = MODE_IDLE;
}

void hardResetTicksAndGoals() {
  encL.reset();
  encR.reset();
  long tL_raw = encL.ticks();
  long tR_raw = encR.ticks();
  lastTickL = ENCODER_L_SIGN * tL_raw;
  lastTickR = ENCODER_R_SIGN * tR_raw;
  startL = startR = 0;
  goalL  = goalR  = 0;
  prevRemL = prevRemR = 0;
}

/* ---------------- Parsing ---------------- */
bool parseTicksLine(const String& line, long& dL, long& dR) {
  if (!(line.startsWith("TICKS:") || line.startsWith("ticks:") ||
        line.startsWith("Ticks:") || line.startsWith("TiCkS:"))) return false;
  int c = line.indexOf(':'); if (c < 0) return false;
  String rest = line.substring(c + 1); rest.trim();
  int sp = rest.indexOf(' '); if (sp < 0) return false;
  dL = rest.substring(0, sp).toInt();
  dR = rest.substring(sp + 1).toInt();
  return true;
}
bool parseRpmLine(const String& line, long& rL, long& rR){
  if (!(line.startsWith("RPM:") || line.startsWith("rpm:") ||
        line.startsWith("SPD:") || line.startsWith("spd:"))) return false;
  int c = line.indexOf(':'); if (c<0) return false;
  String rest = line.substring(c+1); rest.trim();
  int sp = rest.indexOf(' '); if (sp<0) return false;
  rL = rest.substring(0, sp).toInt();
  rR = rest.substring(sp+1).toInt();
  return true;
}

/* ---------------- Setup ---------------- */
void setup() {
  Serial.begin(115200);
  encL.begin(); encR.begin();
  motorL.begin(); motorR.begin();
  hardResetTicksAndGoals();
  Serial.println("READY: TICKS+RPM (cmd-sign enforce, anti-windup, crossing stop) @115200");
  Serial.println("Cmds: TICKS:<L> <R> | RPM:<L> <R> | STOP | RST | PWM:<L> <R>");
}

/* ---------------- Loop ---------------- */
void loop() {
  // 1) Commands
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    if (line.equalsIgnoreCase("STOP")) {
      stopAll();
      hardResetTicksAndGoals();
      Serial.println("OK:STOP+RST");
      continue;
    }
    if (line.equalsIgnoreCase("RST")) {
      stopAll(); hardResetTicksAndGoals(); Serial.println("OK:RST"); continue;
    }

    long dL, dR;
    if (parseTicksLine(line, dL, dR)) {
      long tL = ENCODER_L_SIGN * encL.ticks();
      long tR = ENCODER_R_SIGN * encR.ticks();
      startL = tL; startR = tR;
      goalL  = tL + dL;
      goalR  = tR + dR;
      lastTickL = tL; lastTickR = tR;
      prevRemL = prevRemR = 0;
      movingL = (dL != 0);
      movingR = (dR != 0);
      rpmCmdL = rpmCmdR = 0.f;
      mode = MODE_TICKS;
      Serial.print("GOAL:"); Serial.print(goalL); Serial.print(' '); Serial.println(goalR);
      continue;
    }

    long rL, rR;
    if (parseRpmLine(line, rL, rR)){
      rpmCmdL = (float) constrain((int)rL, -RPM_MAX_CMD, RPM_MAX_CMD);
      rpmCmdR = (float) constrain((int)rR, -RPM_MAX_CMD, RPM_MAX_CMD);
      movingL = movingR = false;
      mode = MODE_RPM;
      int signNow = ((rpmCmdL + rpmCmdR) > 1.f ? +1 : ((rpmCmdL + rpmCmdR) < -1.f ? -1 : 0));
      if (signNow != lastCmdSign){ freezeCtr = FREEZE_TICKS; lastCmdSign = signNow; }
      Serial.println("OK:RPM");
      continue;
    }

    if (line.startsWith("PWM:") || line.startsWith("pwm:")) {
      int c = line.indexOf(':'); String rest = line.substring(c+1); rest.trim();
      int sp = rest.indexOf(' ');
      int pL = rest.substring(0, sp).toInt();
      int pR = rest.substring(sp+1).toInt();
      motorL.drive(clamp255(MOTOR_L_SIGN * pL));
      motorR.drive(clamp255(MOTOR_R_SIGN * pR));
      movingL = movingR = false; mode = MODE_IDLE;
      prevRemL = prevRemR = 0;
      Serial.println("OK:PWM");
      continue;
    }

    Serial.println("ERR:UNKNOWN_CMD");
  }

  // 2) Control loop
  unsigned long now = millis();
  if (now - lastCtl < CONTROL_MS) return;
  float dt = (now - lastCtl) * 0.001f;
  lastCtl = now;

  long tL_raw = encL.ticks();
  long tR_raw = encR.ticks();
  long tL = ENCODER_L_SIGN * tL_raw;
  long tR = ENCODER_R_SIGN * tR_raw;

  long dL_tick = tL - lastTickL;
  long dR_tick = tR - lastTickR;
  lastTickL = tL; lastTickR = tR;

  float rpmL = (dt>0.f) ? ((dL_tick / TPR) / dt * 60.f) : 0.f;
  float rpmR = (dt>0.f) ? ((dR_tick / TPR) / dt * 60.f) : 0.f;
  rpmMeasL_f += RPM_ALPHA * (rpmL - rpmMeasL_f);
  rpmMeasR_f += RPM_ALPHA * (rpmR - rpmMeasR_f);

  // ---- Outer: TICKS -> RPM setpoints ----
  if (mode == MODE_TICKS){
    long remL = goalL - tL;
    long remR = goalR - tR;

    // stop if within deadband OR if we crossed the target
    bool crossedL = (prevRemL != 0 && ((remL > 0) != (prevRemL > 0)));
    bool crossedR = (prevRemR != 0 && ((remR > 0) != (prevRemR > 0)));
    bool doneL = (labs(remL) <= STOP_DEADBAND) || crossedL;
    bool doneR = (labs(remR) <= STOP_DEADBAND) || crossedR;

    if (!doneL){
      float v = K_POS_TO_RPM * (float)remL;
      if (fabs(v) < POS_RPM_MIN) v = (v>=0.f)? POS_RPM_MIN : -POS_RPM_MIN;
      rpmCmdL = constrain(v, -POS_RPM_MAX, POS_RPM_MAX);
    } else rpmCmdL = 0.f;

    if (!doneR){
      float v = K_POS_TO_RPM * (float)remR;
      if (fabs(v) < POS_RPM_MIN) v = (v>=0.f)? POS_RPM_MIN : -POS_RPM_MIN;
      rpmCmdR = constrain(v, -POS_RPM_MAX, POS_RPM_MAX);
    } else rpmCmdR = 0.f;

    prevRemL = remL; prevRemR = remR;
    if (doneL && doneR){
      stopAll();
    }
  }

  // ---- Inner: RPM -> PWM ----
  int16_t baseL = basePwmForSide(rpmCmdL, true);
  int16_t baseR = basePwmForSide(rpmCmdR, false);

  float eq = 0.f;
  bool sameSign = (rpmCmdL * rpmCmdR) > 0.f;
  bool similar  = (abs((int)rpmCmdL - (int)rpmCmdR) <= STRAIGHT_TOL_RPM);
  if ((mode==MODE_RPM || mode==MODE_TICKS) && sameSign && similar)
    eq = K_EQUALIZE * (rpmMeasL_f - rpmMeasR_f);

  // direction-kick & freeze
  int16_t kickL = 0, kickR = 0;
  if (freezeCtr > 0){
    int cmdSign = ((rpmCmdL + rpmCmdR) > 1.f ? +1 : ((rpmCmdL + rpmCmdR) < -1.f ? -1 : 0));
    kickL = (cmdSign>0 ? +KICK_PWM_DIRCHG : (cmdSign<0 ? -KICK_PWM_DIRCHG : 0));
    kickR = kickL; freezeCtr--;
  }

  // I-only trims with simple anti-windup: integrate only if not saturating
  // OR if the integration would reduce saturation.
  auto update_trim = [](int16_t& trim, float KI, float err, int16_t base, int16_t kick){
    int16_t pre = base + trim + kick;
    bool sat = (abs(pre) >= MAX_PWM - 2);
    if (!sat || ( (pre > 0 && err < 0) || (pre < 0 && err > 0) )) {
      long t = (long)trim + (long)(KI * err);
      if (t >  TRIM_CAP) t =  TRIM_CAP;
      if (t < -TRIM_CAP) t = -TRIM_CAP;
      trim = (int16_t)t;
    }
  };

  float eL = (rpmCmdL - rpmMeasL_f) - eq;
  float eR = (rpmCmdR - rpmMeasR_f) + eq;
  if (mode==MODE_RPM || mode==MODE_TICKS){
    update_trim(trimL, KI_VEL_L, eL, baseL, kickL);
    update_trim(trimR, KI_VEL_R, eR, baseR, kickR);
  } else { trimL = trimR = 0; }

  int16_t outL = baseL + trimL + kickL;
  int16_t outR = baseR + trimR + kickR;

  // honor minimum PWM when commanding non-zero RPM
  if (mode!=MODE_IDLE){
    if (rpmCmdL!=0.f && abs(outL)<MIN_PWM) outL = (outL>=0)? MIN_PWM : -MIN_PWM;
    if (rpmCmdR!=0.f && abs(outR)<MIN_PWM) outR = (outR>=0)? MIN_PWM : -MIN_PWM;
  } else { outL = outR = 0; }

  // ---- ENFORCE COMMAND SIGN IN TICKS MODE ----
  // Always push toward the goal: set command sign = sign(remaining)
  if (mode == MODE_TICKS){
    long remL = goalL - tL;
    long remR = goalR - tR;
    int sL = sgnl(remL);
    int sR = sgnl(remR);
    outL = (sL == 0) ? 0 : (sL * abs(outL));
    outR = (sR == 0) ? 0 : (sR * abs(outR));
  }

  // ---- Drive motors ----
  motorL.drive(clamp255(MOTOR_L_SIGN * outL));
  motorR.drive(clamp255(MOTOR_R_SIGN * outR));

  // 3) Telemetry
  Serial.print("TRW:"); Serial.print(tL_raw); Serial.print(' '); Serial.println(tR_raw);
  Serial.print("TCK:"); Serial.print(tL);     Serial.print(' '); Serial.println(tR);
  Serial.print("RPM:"); Serial.print(rpmMeasL_f,1); Serial.print(' '); Serial.println(rpmMeasR_f,1);
  Serial.print("TRM:"); Serial.print(trimL); Serial.print(' '); Serial.println(trimR);
  if (mode==MODE_TICKS && (movingL || movingR)) {
    long remL = goalL - tL;
    long remR = goalR - tR;
    Serial.print("REM:"); Serial.print(remL); Serial.print(' '); Serial.println(remR);
  }
}
