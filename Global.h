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

#define RIGHT_INPUT_PIN_1 7
#define RIGHT_INPUT_PIN_2 6
#define RIGHT_ENABLE_PIN 11

#define LEFT_INPUT_PIN_1 5
#define LEFT_INPUT_PIN_2 4
#define LEFT_ENABLE_PIN 10

#define LINE_POS_FAR_LEFT -1.0
#define LINE_POS_FAR_RIGHT 1.0
#define SENSOR_MAX_VALUE 5000.0
#define SENSOR_MIN_VALUE 0.0

#define CALIBRATION_OUTER_THRESHOLD 0.95
#define CALIBRATION_INNER_THRESHOLD 0.1

#define MAX_MOTOR_SPEED 255

#define TIME_ONE_WHITE_TO_STOP 1000 // millis

#define EEPROM_CALIBRATION_DATA_ADDRESS 0

#define BUTTON_PIN 3
#define BUTTON_DEBOUNCE_DURATION 50

#endif
