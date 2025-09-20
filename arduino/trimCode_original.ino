#include <Arduino.h>
#include "EncoderUNO.h"
#include "L298.h"
#include "Protocol.h"
#include "TrimEqualizer.h"  // <— NEW

// ======= L298 pins =======
const uint8_t L_IN1 = 5;
const uint8_t L_IN2 = 6;
const uint8_t R_IN3 = 9;
const uint8_t R_IN4 = 10;

// ======= Encoders =======
const uint8_t L_ENC_A = 2, L_ENC_B = 4;
const uint8_t R_ENC_A = 3, R_ENC_B = 7;

// Invert flags as you validated earlier
EncoderUNO encL(L_ENC_A, L_ENC_B, false);
EncoderUNO encR(R_ENC_A, R_ENC_B, true);

L298 motorL(L_IN1, L_IN2, true, 255);
L298 motorR(R_IN3, R_IN4, true, 255);

// Loop timing
static const unsigned LOOP_MS = 200;

// Trim equalizer instance (you can tweak tunables here)
TrimEqualizer trimEq(/*trimLimit=*/80, /*trimKi=*/0.15f,
                     /*deadbTck=*/3, /*minMoveT=*/1, /*minCmd=*/10);

String line;
long prevTicksL = 0, prevTicksR = 0;

static inline void driveNow() {
  int16_t outL, outR;
  trimEq.apply(outL, outR);
  motorL.drive(outL);
  motorR.drive(outR);
}

// Parse "PWM:<L> <R>"
bool parsePwmLine(const String& s, int16_t& outL, int16_t& outR) {
  if (!s.startsWith("PWM:")) return false;
  int sp = s.indexOf(' ');
  if (sp < 0) return false;
  long l = s.substring(4, sp).toInt();
  long r = s.substring(sp + 1).toInt();
  outL = (int16_t)constrain(l, -255, 255);
  outR = (int16_t)constrain(r, -255, 255);
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  encL.begin(); encR.begin();
  motorL.begin(); motorR.begin();

  prevTicksL = encL.ticks();
  prevTicksR = encR.ticks();

  Serial.println("OK: PWM passthrough + ratio-based trim (separate file).");
  Serial.println("Commands:");
  Serial.println("  PWM:<L> <R>   (−255..255)");
  Serial.println("  STOP");
}

void loop() {
  // --- Serial parsing ---
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line.trim();
      if (line.length()) {
        if (line.equalsIgnoreCase("STOP")) {
          trimEq.stop();
          motorL.brakeFast();
          motorR.brakeFast();
          Serial.println("ACK:STOP");
        } else {
          int16_t pl, pr;
          if (parsePwmLine(line, pl, pr)) {
            bool flipped = trimEq.onCommand(pl, pr);
            if (flipped) Serial.println("INFO:dir_change -> trims reset");
            Serial.print("ACK:PWM "); Serial.print(pl); Serial.print(" ");
            Serial.println(pr);
            driveNow();
          } else {
            Serial.print("ERR:bad_cmd '"); Serial.print(line); Serial.println("'");
          }
        }
      }
      line = "";
    } else {
      if (line.length() < 64) line += c; else line = ""; // guard
    }
  }

  // --- 200 ms loop ---
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last >= LOOP_MS) {
    last = now;

    long tL = encL.ticks(), tR = encR.ticks();
    long dL = tL - prevTicksL, dR = tR - prevTicksR;

    trimEq.update(dL, dR);
    driveNow();

    // Telemetry
    printTicks(Serial, tL, tR);
    Serial.print("TRM:"); Serial.print(trimEq.trimL()); Serial.print(" ");
    Serial.println(trimEq.trimR());

    prevTicksL = tL; prevTicksR = tR;
  }
}
