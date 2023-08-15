#ifndef TESTS_H
#define TESTS_H

#include "MotorController.h"
#include "ButtonConroller.h"
#include "QTRSensorController.h"

/* 
 * run both motors forwards and backwards
 * with delays between all operations
 */
void motorAll() {
  delay(2000);
  motorRun(leftMotor, MAX_MOTOR_SPEED);
  delay(1000);
  motorStop(leftMotor);
  delay(1000);
  motorRun(leftMotor, -MAX_MOTOR_SPEED);
  delay(1000);
  motorStop(leftMotor);
  delay(1000);
  motorRun(rightMotor, MAX_MOTOR_SPEED);
  delay(1000);
  motorStop(rightMotor);
  delay(1000);
  motorRun(rightMotor, -MAX_MOTOR_SPEED);
  delay(1000);
  motorStop(rightMotor);
}

/*
 * Run both motors forwards after X seconds
 * and stop after Y seconds
 */
void motorForward() {
  delay(3000);
  motorRun(leftMotor, MAX_MOTOR_SPEED);
  motorRun(rightMotor, MAX_MOTOR_SPEED);
  delay(2500);
  motorStop(leftMotor);
  motorStop(rightMotor);
}

/*
 * Show button presses in the serial monitor
 */
void testButton() {
  Button b;
  buttonInit(b, BUTTON_PIN);

  while (1) {
    bool p = buttonDetectPress(b);
    
    if (p == true)
      Serial.println(p);
  }
}

void testQtrSensors() {
    qtrReadCalibrated();
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.print('\n');
}

#endif
