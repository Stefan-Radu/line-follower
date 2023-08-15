#ifndef PID_CONTROLLER
#define PID_CONTROLLER

#include "Global.h"

#define MOTOR_RUN_POWER 150

int16_t pidCenterLine(double e, bool debug = false) {
  static const int16_t maxCorrection = MOTOR_RUN_POWER * 2;
  static const double kp = 650, kd = 2000, ki = 0.15;
  
  static double lastError = 0, cummulativeError = 0;

  int16_t correction = kp * e + kd * (e - lastError) + ki * cummulativeError;
  clamp(correction, -maxCorrection, maxCorrection);

#ifdef DEBUG
  //Serial.print("PID Correction: ");
  //Serial.println(correction);
#endif
  
  lastError = e;
  cummulativeError += e;

  return correction;
}

#endif
