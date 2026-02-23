#ifndef HELPER_FUNCTIONS
#define HELPER_FUNCTIONS

#include "Arduino.h"
// namespace TankBotFunctions {
template <typename T> T clamp(T &x, T &lo, T &hi) {
  return max(min(x, hi), lo);
}
//}
//
/*
#ifdef __cplusplus
extern "C" {
#endif
*/
float clampf(float &x, const float &lo, const float &hi);

int clampInt(int &v, const int &lo, const int &hi);

int applyDeadzone(int &v, const int &dz);

void rampToward(int &cur, int &tgt, int &step);

const char *dirStr(int &c);
/*
#ifdef __cplusplus
}
#endif
*/

#endif
