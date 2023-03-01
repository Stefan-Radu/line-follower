#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H

#include "ButtonController.h"
#include "MotorController.h"
#include "Global.h"
#include "QTRSensorController.h"
#include "PidController.h"

enum DriveState {
  STOP,
  FORWARD,
  TURN_LEFT,
  TURN_RIGHT,
  TURN_AROUND,
};

enum DriveMode {
  FOLLOW_LINE,
  SOLVE_MAZE,
};

DriveMode driveMode = SOLVE_MAZE;
DriveState state = STOP;
Button button;

void driveInit() {
  motorInit(leftMotor);
  motorInit(rightMotor);
  
  // used to recalibrate the sensor
  buttonInit(button, BUTTON_PIN);
  
  Serial.println("Driver Initialized");
}

void driveStateUpdate() {
  static int32_t lastTimeOnBlack = 0;

  int32_t timeNow = millis();
//  int8_t cnt = qtrGetBlackSensorCount();
  if (cnt == 0) {
    if (lastTimeOnBlack - timeNow > TIME_ONE_WHITE_TO_STOP) {
      state = STOP;
    }
  } else {
    lastTimeOnBlack = millis();
    state = FORWARD;
  }
}

void driveStop() {
  motorStop(leftMotor);
  motorStop(rightMotor);
}

void driveQTRCalibrateOnButtonPress() {
  static Button b;
  bool p = buttonDetectPress(button);
  if (p == 1) {
    driveStop();
    state = STOP;
    qtrCalibrate();
  }
}

void driveForward() {
  double error = qtrGetBlackLinePosition();
  int16_t correction = pidCenterLine(error);
  int16_t correctedMotorSpeed = MAX_MOTOR_SPEED - abs(correction);
 
  if (correction < 0) {
    // left correction
    motorRun(leftMotor, correctedMotorSpeed);
    motorRun(rightMotor, MAX_MOTOR_SPEED);  
  } else {
    // right correction
    motorRun(leftMotor, MAX_MOTOR_SPEED);
    motorRun(rightMotor, correctedMotorSpeed);
  }
}

void simulateDriveForward() {
  /*
   * this is for testing only!! 
   * TODO remove when done
   */
  double error = qtrGetBlackLinePosition();
  Serial.print("Error ");
  Serial.println(error);
  
  int16_t correction = pidCenterLine(error, true);
  Serial.print("Correction: ");
  Serial.println(correction);
  int16_t correctedMotorSpeed = MAX_MOTOR_SPEED - abs(correction);
 
  if (correction < 0) {
    Serial.println("Left Correction");
  } else {
    Serial.println("Right Correction");
  }
  delay(500);
}

void driveController() {
  switch (state) {
    case STOP:
      driveStop();
      break;
    case FORWARD:
      driveForward();
      break;
  }
}

#endif
