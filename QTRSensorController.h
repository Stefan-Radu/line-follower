#ifndef QTR_SENSOR_CONTROLLER_H
#define QTR_SENSOR_CONTROLLER_H

#include <QTRSensors.h>
#include <EEPROM.h>
#include "MotorController.h"
#include "Global.h"

QTRSensors qtr;

const uint8_t sensorCount = 6;
uint16_t sensorValues[sensorCount];

void qtrInit() {
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, sensorCount);

  delay(500);
  Serial.println("QTR Initialized");
}

void saveCalibrationData() {
  int16_t address = EEPROM_CALIBRATION_DATA_ADDRESS;
  for (int i = 0; i < sensorCount; ++i) {
    EEPROM.put(address, qtr.calibrationOn.minimum[i]);
    address += sizeof(qtr.calibrationOn.minimum[i]);
  }
  for (int i = 0; i < sensorCount; ++i) {
    EEPROM.put(address, qtr.calibrationOn.maximum[i]);
    address += sizeof(qtr.calibrationOn.maximum[i]);
  }

  Serial.println("Calibration Data saved");
}

void loadCalibrationData() {
  int16_t address = EEPROM_CALIBRATION_DATA_ADDRESS;
  
  if (qtr.calibrationOn.minimum == NULL) {
    qtr.calibrationOn.minimum = (uint16_t*) calloc(sensorCount, sizeof(uint16_t));
  }
  for (int8_t i = 0; i < sensorCount; ++i) {
    EEPROM.get(address, qtr.calibrationOn.minimum[i]);
    address += sizeof(qtr.calibrationOn.minimum[i]);
  }

  if (qtr.calibrationOn.maximum == NULL) {
    qtr.calibrationOn.maximum = (uint16_t*) calloc(sensorCount, sizeof(uint16_t));
  }
  for (int8_t i = 0; i < sensorCount; ++i) {
    EEPROM.get(address, qtr.calibrationOn.maximum[i]);
    address += sizeof(qtr.calibrationOn.maximum[i]);
  }

  qtr.calibrationOn.initialized = true;
  Serial.println("Calibration Data loaded");
}

/*
 * count how many of the sensors 
 * detect a dark color
 * this is used to determine the driving state
 */
int8_t qtrGetBlackSensorCount() {
  static const uint16_t blackThreshold = 400;

  qtr.readCalibrated(sensorValues); 
  
//  for (uint8_t i = 0; i < sensorCount; ++i) {
//    Serial.print(sensorValues[i]);
//    Serial.print('\t');
//  }
//  delay(500);

  int8_t count = 0;
  for (int8_t i = 0; i < sensorCount; ++i) {
    if (sensorValues[i] > blackThreshold) {
      count += 1;
    }
  }

  return count;
}

// TODO update this to be able to return linepos, turns, intersections
/*
 * return black line position as a number between -1 and 1
 * where -1 is far left
 * and 1 is far right
 */
double qtrGetBlackLinePosition(bool debug = false) {
  /* 
   * according to qtr documentation: 
   * res is between 0 and 5000 
   * I want to map it to a better interval 
   */
  int16_t res = qtr.readLineBlack(sensorValues);
  double linePos = doubleMap(res, SENSOR_MIN_VALUE, SENSOR_MAX_VALUE, LINE_POS_FAR_LEFT, LINE_POS_FAR_RIGHT);

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

void qtrCalibrate() {
  Serial.println("QTR Sensor calibrating...");
  
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  int16_t motorPower = MAX_MOTOR_SPEED;
  
  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    double linePos = qtrGetBlackLinePosition();
    /*
     * oscilate between the left part of the line
     * and the right side of the line
     */
    if ((linePos < -CALIBRATION_OUTER_THRESHOLD && motorPower > 0) ||
        (linePos > CALIBRATION_OUTER_THRESHOLD && motorPower < 0)) {
      motorPower *= -1;
    }
    motorRun(leftMotor, motorPower);
    motorRun(rightMotor, -motorPower);
  }

  /*
   * do this to somewhat center the robot on
   * the line after callibration
   */
  motorPower *= -1;
  motorRun(leftMotor, motorPower);
  motorRun(rightMotor, -motorPower);
  while (1) {
    double linePos = qtrGetBlackLinePosition();
    if (linePos > -CALIBRATION_INNER_THRESHOLD && linePos < CALIBRATION_INNER_THRESHOLD) {
      break;
    }
  }

  motorStop(leftMotor);
  motorStop(rightMotor);
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  
  delay(500);
  Serial.println("QTR Sensor calibrated");

  saveCalibrationData();
}

#endif
