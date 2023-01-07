#ifndef PID_CONTROLLER
#define PID_CONTROLLER

#include "Global.h"

motorSpeed pidCenterLine(int16_t e, debug = false) {
  static const maxCorrection = MAX_MOTOR_SPEED; // at most stop a motor, but don't turn is back
  
  static const int16_t kp = 10;
  static const double kd = 0, ki = 0;
  
  static double lastError = 0, cummulativeError = 0;

  int16_t correction = kp * e + kd * (e - lastError) + ki * cummulativeError;
  clamp(correction, -maxCorrection, maxCorrection);

  if (debug) {
    Serial.println(correction);
  }
  
  lastError = e;
  cummulativeError += e;

  int16_t correctedMotorSpeed = MAX_MOTOR_SPEED - abs(correction);
  if (correction < 0) {
    // left correction? TODO
    return {correctedMotorSpeed, MAX_MOTOR_SPEED};  
  } else {
    // right correction? TODO
    return {MAX_MOTOR_SPEED, correctedMotorSpeed};
  }
}

#endif
