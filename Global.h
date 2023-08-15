#ifndef GLOBAL_H
#define GLOBAL_H

void clamp(int16_t &val, int16_t minVal, int16_t maxVal) {
  if (val > maxVal) {
    val = maxVal;
  } else if (val < minVal) {
    val = minVal;
  }
}

double doubleMap(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// TODO make this <utils.h>

#endif
