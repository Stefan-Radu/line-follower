#include "DriveController.h"
#include "QTRSensorController.h"
#include "ButtonController.h"

/*
 * useful for testing direction and which motor is which
 * expected behaviour
 * wait 2 sec
 * turn left motor forward for 1 sec
 * wait 1 sec
 * turn left motor backward for 1 sec
 * wait 1 sec
 * do the same with right motor
 */

void motorTest1() {
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

void motorTest2() {
  delay(5000);
  motorRun(leftMotor, MAX_MOTOR_SPEED);
  motorRun(rightMotor, MAX_MOTOR_SPEED);
  delay(2500);
  motorStop(leftMotor);
  motorStop(rightMotor);
}

void testButton() {
  Button b;
  buttonInit(b, BUTTON_PIN);

  while (1) {
    bool p = buttonDetectPress(b);
    
    if (p == true)
      Serial.println(p);
  }
}

void setup(){
  Serial.begin(9600);
  driveInit();
  qtrInit();
  loadCalibrationData();  
  delay(1000);
}

void loop(){
  driveQTRCalibrateOnButtonPress();
  driveStateUpdate();
  driveController();
}
