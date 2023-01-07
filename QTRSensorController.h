#ifndef QTR_SENSOR_CONTROLLER_H
#define QTR_SENSOR_CONTROLLER_H

#include <QTRSensors.h>
#include "Global.h"

QTRSensors qtr;

static const uint8_t sensorCount = 6;
static uint16_t sensorValues[sensorCount];

void qtrInit() {
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, sensorCount);
  delay(500);
  initilized = true;
  Serial.println("QTR Sensor initialized");
}

void qtrCalibrate() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  delay(500);
  Serial.println("QTR Sensor calibrated");
}

/*
 * count how many of the sensors 
 * detect a dark color
 * this is used to determine the driving state
 */
int8_t qtrGetBlackSensorCount() {
  static const uint16_t blackThreshold = 400;

  int8_t count = 0;
  for (int8_t i = 0; i < sensorCount; ++i) {
    if (sensorValues[i] > blackthreshold) {
      count += 1;
    }
  }

  return count;
}

// TODO update this to be able to return linepos, turns, intersections
int16_t qtrGetBlackLinePosition(bool debug = false) {
  /* 
   * according to qtr documentation: 
   * res is between 0 and 5000 
   * I want to map it to a better interval 
   */
  int16_t res = qtr.readLineBlack(sensorValues);
  int16_t linePos = map(res, 0, 5000, SENSOR_LEFT_EXTREME_READING, SENSOR_RIGHT_EXTREME_READING);

  if (debug) {
    for (uint8_t i = 0; i < sensorCount; ++i) {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.print(">> ");
    Serial.print(linePos);
    Serial.print(" <<");
    Serial.print('\n');
  }
  
  return linePos;
}

#endif
