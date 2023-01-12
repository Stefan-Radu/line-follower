#ifndef PID_CONTROLLER
#define PID_CONTROLLER

#include "Global.h"

int16_t pidCenterLine(double e, bool debug = false) {
  static const int16_t maxCorrection = MAX_MOTOR_SPEED * 2;
  static const double kp = 650, kd = 2000, ki = 0.15;
  
  static double lastError = 0, cummulativeError = 0;

  int16_t correction = kp * e + kd * (e - lastError) + ki * cummulativeError;
  clamp(correction, -maxCorrection, maxCorrection);

  if (debug) {
    Serial.println(correction);
  }
  
  lastError = e;
  cummulativeError += e;

  return correction;
}

#endif
