#pragma once
#include <Arduino.h>

struct SpeedCmd {
  int16_t rpm_l = 0;
  int16_t rpm_r = 0;
};

// Parse "SPD:<L> <R>" (case-insensitive). Returns true if parsed.
inline bool parseSpeedLine(const String& line, SpeedCmd& out) {
  out = {};
  if (line.length() < 5) return false;

  // Case-insensitive startsWith for "SPD:"
  bool ok = false;
  if (line.startsWith("SPD:") || line.startsWith("spd:") ||
      line.startsWith("Spd:") || line.startsWith("SpD:")) {
    ok = true;
  }
  if (!ok) return false;

  int sep = line.indexOf(':');
  if (sep < 0) return false;
  String rest = line.substring(sep + 1);
  rest.trim();

  int sp = rest.indexOf(' ');
  if (sp < 0) return false;

  out.rpm_l = (int16_t)rest.substring(0, sp).toInt();

  String r = rest.substring(sp + 1);
  r.trim();
  out.rpm_r = (int16_t)r.toInt();
  return true;
}

inline void printTicks(Stream& s, long tL, long tR) {
  s.print("TCK:"); s.print(tL); s.print(' '); s.println(tR);
}
