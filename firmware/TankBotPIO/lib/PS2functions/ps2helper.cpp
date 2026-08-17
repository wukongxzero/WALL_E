#include "ps2helper.h"
#include "Arduino.h"

float clampf(float &x, float &lo, float &hi) {
  /*
  if (x < lo)
    return lo;
  if (x > hi)
    return hi;
  return x;
*/
  return (max(min(x, hi), lo));
}

int clampInt(int &v, const int &lo, const int &hi) {
  /*if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  */
  // Optimization added
  return (max(min(v, hi), lo));
}

int applyDeadzone(int &v, const int &dz) { return (abs(v) <= dz) ? 0 : v; }

void rampToward(int &curIO, int &tgt, int &step) {
  /* if (cur < tgt)
     return min(cur + step, tgt);
   if (cur > tgt)
     return max(cur - step, tgt);
  */
  int error = curIO - tgt;
  // branchless programming approach
  curIO = max(min(curIO + ((error > 0) - (error < 0)) * step, tgt), tgt);
}

const char *dirStr(int c) {
  if (c > 0)
    return "FWD";
  if (c < 0)
    return "REV";
  return "STOP";
  // this down here does not work...I tried
  // return (c > 0) * "FWD" + (c < 0) * "BWD" + (c == 0) * "STOP";
}
