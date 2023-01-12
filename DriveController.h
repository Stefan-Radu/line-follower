#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H

#include "MotorController.h"
#include "Global.h"
#include "QTRSensorController.h"
#include "PidController.h"

enum DriveState {
  STOP,
  FORWARD,
  SIDE_TURN,
  U_TURN,
};

const motor leftMotor {LEFT_INPUT_PIN_2, LEFT_INPUT_PIN_1, LEFT_ENABLE_PIN};
const motor rightMotor {RIGHT_INPUT_PIN_2, RIGHT_INPUT_PIN_1, RIGHT_ENABLE_PIN};

DriveState state = STOP;

void driveInit() {
  motorInit(leftMotor);
  motorInit(rightMotor);
  Serial.println("Driver Initialized");
}

void driveStateUpdate() {
  static int32_t lastTimeOnBlack = 0;

  int32_t timeNow = millis();
  int8_t cnt = qtrGetBlackSensorCount();
  if (cnt == 0) {
    if (lastTimeOnBlack - timeNow > TIME_ONE_WHITE_TO_STOP) {
      state = DriveState::STOP;
    }
  } else {
    lastTimeOnBlack = millis();
    state = DriveState::FORWARD;
  }
}

void driveStop() {
  motorStop(leftMotor);
  motorStop(rightMotor);
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
    case DriveState::STOP:
      driveStop();
      break;
    case DriveState::FORWARD:
      driveForward();
      break;
    case DriveState::SIDE_TURN:
      break;
    case DriveState::U_TURN:
      break;
  }
}

#endif
