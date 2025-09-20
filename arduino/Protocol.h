#pragma once
#include <Arduino.h>

// In : "SPD:<leftRPM> <rightRPM>\n"  or "STOP\n"
// Out: "TCK:<leftTicks> <rightTicks>\n"

struct SpeedCmd {
  long rpm_l;
  long rpm_r;
  SpeedCmd() : rpm_l(0), rpm_r(0) {}
  SpeedCmd(long L, long R) : rpm_l(L), rpm_r(R) {}
};

inline bool parseSpeedLine(const String& s, SpeedCmd& out) {
  if (!s.startsWith("SPD:")) return false;
  int sp = s.indexOf(' ');
  if (sp < 0) return false;
  out.rpm_l = s.substring(4, sp).toInt();
  out.rpm_r = s.substring(sp + 1).toInt();
  return true;
}

inline void printTicks(Stream& io, long tl, long tr) {
  io.print("TCK:"); io.print(tl); io.print(" "); io.println(tr);
}
