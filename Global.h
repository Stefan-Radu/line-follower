#ifndef GLOBAL_H
#define GLOBAL_H

struct motorSpeed {
  int16_t leftMotorSpeed;
  int16_t rightMotorSpeed;
};

int16_t clamp(int16_t val, int16_t minVal, int16_t maxVal) {
  if (val > maxVal) {
    return maxVal;
  } else if (val < minVal) {
    return minVal;
  }
  return val;
}

#define LEFT_MOTOR_PLUS_PIN 5
#define LEFT_MOTOR_MINUS_PIN 6
#define RIGHT_MOTOR_PLUS_PIN 10
#define RIGHT_MOTOR_MINUS_PIN 9

#define SENSOR_LEFT_EXTREME_READING -100
#define SENSOR_RIGHT_EXTREME_READING 100

#define MAX_MOTOR_SPEED 255

#endif
