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
  TURNING_LEFT,
  TURNING_RIGHT,
} driveState = STOP;

enum class DriveAction {
  STOP,
  FOLLOW_LINE,
  FORWARD,
  TURN_LEFT,
  TURN_RIGHT,
  TURN_AROUND,
};

enum DriveMode {
  FOLLOW_LINE,
  SOLVE_MAZE,
} driveMode = SOLVE_MAZE;

Button button;

bool isTurning = false;
bool actionLock = false;

#define TURN_OVERSHOOT_DURATION 150
#define U_TURN_OVERSHOOT_DURATION 150
#define BACK_ON_TRACK_DURATION 800
#define SKIP_INTERSECTION_DURATION 150

void driveInit() {
  motorInit(leftMotor);
  motorInit(rightMotor);
  
  // used to recalibrate the sensor
  buttonInit(button, BUTTON_PIN);
  
  Serial.println("Driver Initialized");
}

/*
 * if too much time is spent on white STOP
 */
void driveCheckToStop(const int &t) {
  if (t - millis() > TIME_ONE_WHITE_TO_STOP) {
    driveState = STOP;
  }
}

/*
 * TODO is it too much indirection?
 * should I have states change by themselves, instead of having a master changer?
 */
void driveStateUpdate(bool debug = false) {
  static int32_t lastTimeOnBlack = 0;
  static int32_t turnTimer = 0;

  if (driveMode == FOLLOW_LINE) {
    qtrBlackSensorCount cnt = qtrSmoothSensorBlackCount();
    if (cnt.total == 0) {
      driveCheckToStop(lastTimeOnBlack);
    } else {
      lastTimeOnBlack = millis();
      driveState = FORWARD;
    }
  } else if (driveMode == SOLVE_MAZE) {
    qtrSmoothUpdatePath();
    qtrSmoothPrintPath();

    PathType pathNow = smoothSensorInfo.currentPathType;
    if (driveState == FORWARD) {  
      if (pathNow == LEFT_TURN || pathNow == T_INTERSECTION) {
        driveState = TURNING_LEFT;
        turnTimer = millis();
      } else if (pathNow == RIGHT_TURN || pathNow == U_TURN) {
        driveState = TURNING_RIGHT;
        turnTimer = millis();
      }
    } else if (pathNow == LINE && millis() - turnTimer > 250) {
      driveState = FORWARD;
    }

    if (pathNow != EMPTY) {
      lastTimeOnBlack = millis();
      if (driveState == STOP) {
        driveState = FORWARD;
      }
    } else {
       // se oprea imediat asa ca am scos asta
//      driveCheckToStop(lastTimeOnBlack);
    }
  }
 
  if (debug) {
    Serial.println(driveState);
  }
}

bool driveIsTurn(const DriveAction &a) {
  return a == DriveAction::TURN_LEFT ||
         a == DriveAction::TURN_RIGHT ||
         a == DriveAction::TURN_AROUND;
}

DriveAction driveGetNextAction() {
  static uint32_t timer = 0;
  static DriveAction previousAction = DriveAction::STOP;
  static qtrPath pathA = {0, 1, 0}, pathB = {0, 0, 0};

  qtrPath newReading = qtrGetPath();
  if (newReading != pathA) {
    pathB = pathA;
    pathA = newReading;
  }

//  Serial.print(pathA.left);
//  Serial.print(" ");
//  Serial.print(pathA.front);
//  Serial.print(" ");
//  Serial.print(pathA.right);
//  Serial.print(" | ");
//  Serial.print(pathB.left);
//  Serial.print(" ");
//  Serial.print(pathB.front);
//  Serial.print(" ");
//  Serial.println(pathB.right);
  
  DriveAction ret;
  if (actionLock == true) {
    ret = previousAction;
  } else {
    if (driveIsTurn(previousAction)) {
      timer = millis();
      ret = DriveAction::FOLLOW_LINE;
    } else if (timer != 0) {
      ret = DriveAction::FOLLOW_LINE;
      if (millis() - timer > BACK_ON_TRACK_DURATION) {
        timer = 0;
      }
    } else if (pathB.left) {
      ret = DriveAction::TURN_LEFT;
      Serial.println("turn left");
//      delay(500);
    } else if (pathA.front) {
//      if (previousPath.right) {
//        ret = DriveAction::FORWARD;
//      } else {
        ret = DriveAction::FOLLOW_LINE;
//      }
      Serial.println("go straight");
    } else if (pathB.right) {
      ret = DriveAction::TURN_RIGHT;
      Serial.println("turn right");
    } else if (/*previousPath.front &&*/ pathA == qtrPath{0, 0, 0}) {
      ret = DriveAction::TURN_AROUND;
       // TODO doua goale la rand == U_TURN
      Serial.println("turn around");
    } else {
      ret = previousAction; 
    }
  }

  previousAction = ret;
//  previousPath = currentPath;
//  delay(100);
  return ret;
}

void driveStop() {
  motorStop(leftMotor);
  motorStop(rightMotor);
}

void driveMotorsTurn(const DriveAction &a) {
  switch (a) {
  case DriveAction::TURN_LEFT:
    motorRun(leftMotor, -TURN_MOTOR_SPEED); // TODO magic number
    motorRun(rightMotor, TURN_MOTOR_SPEED);
    break;
  case DriveAction::TURN_RIGHT:
    motorRun(leftMotor, TURN_MOTOR_SPEED);
    motorRun(rightMotor, -TURN_MOTOR_SPEED);// TODO magic number
    break;
  case DriveAction::TURN_AROUND:
    motorRun(leftMotor, TURN_MOTOR_SPEED);
    motorRun(rightMotor, -TURN_MOTOR_SPEED);
    break;
  }
}

void driveForw() {
  static uint32_t timer = 0;
  
  if (!actionLock) {
    actionLock = true;
    timer = millis();
  }

  // TODO IDEEE
//  driveStop();
  motorRun(leftMotor, MAX_MOTOR_SPEED);
  motorRun(rightMotor, MAX_MOTOR_SPEED);

  if (millis() - timer > SKIP_INTERSECTION_DURATION) {
    actionLock = false;
  }
}

enum class TurnStage {
  OVERSHOOT,
  FIND_EMPTY,
  TURN,
};

void driveTurn(const DriveAction &a) {
  static TurnStage ts;
  static uint32_t timer = 0;

  if (!actionLock) {
    actionLock = true;
    ts = TurnStage::OVERSHOOT;
  }

  qtrPath p = qtrGetPath();
  uint16_t timerTarget = (a == DriveAction::TURN_AROUND) ?
    U_TURN_OVERSHOOT_DURATION : TURN_OVERSHOOT_DURATION;

//  Serial.println(timerTarget);
//  delay(200);

  switch (ts) {
  case TurnStage::OVERSHOOT:
    if (!timer) {
      timer = millis();
      motorRun(leftMotor, MAX_MOTOR_SPEED);
      motorRun(rightMotor, MAX_MOTOR_SPEED);
    }

    if (millis() - timer > timerTarget) {
      timer = 0;
      ts = TurnStage::FIND_EMPTY;
    }
    break;
  case TurnStage::FIND_EMPTY:
    driveMotorsTurn(a);
    if (p == qtrPath{0, 0, 0}) {
      ts = TurnStage::TURN;
    }
    break;
  case TurnStage::TURN:
    if (p.front) {
      actionLock = false;
    }
    break;
  }
}

void driveForward(int16_t turnCompensation = 0) {
  double error = qtrGetBlackLinePosition();
  int16_t correction = pidCenterLine(error) + turnCompensation;
  
  clamp(correction, -MAX_MOTOR_SPEED * 2, MAX_MOTOR_SPEED * 2);
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

void driveTurn(bool left) {
  static bool overshooting = false;
  
  if (isTurning == false) {
    isTurning = true;
    overshooting = true;
  }

  PathType pathNow = smoothSensorInfo.currentPathType;
  if (overshooting) {
    driveForward();
    if (pathNow == LINE || pathNow == EMPTY) {
      overshooting = false;    
    }
  } else {
    if (left) {
      motorRun(leftMotor, -MAX_MOTOR_SPEED);
      motorRun(rightMotor, MAX_MOTOR_SPEED);  
    } else {
      motorRun(leftMotor, MAX_MOTOR_SPEED);
      motorRun(rightMotor, -MAX_MOTOR_SPEED);  
    }
    if (pathNow == LINE) {
      driveState = FORWARD;
      isTurning = false;
    }
  }
}

void driveController() {
  switch (driveState) {
    case STOP:
      driveStop();
      break;
    case FORWARD:
      driveForward();
      break;
    case TURNING_LEFT:
//      driveTurn(1);
      driveForward(-MAX_MOTOR_SPEED * 20);
      break;
    case TURNING_RIGHT:
      driveForward(MAX_MOTOR_SPEED * 20);
//      driveTurn(0);
      break;
  }
}

void driveHandleAction(const DriveAction &a) {
  switch (a) {
  case DriveAction::FOLLOW_LINE:
    driveForward(); // TODO rename this 
    break;
  case DriveAction::FORWARD:
    driveForw(); // TODO rename this
    break;
  case DriveAction::TURN_LEFT:
  case DriveAction::TURN_RIGHT:
  case DriveAction::TURN_AROUND:
    driveTurn(a);
    break;
  }
}

void driveQTRCalibrateOnButtonPress() {
  bool p = buttonDetectPress(button);
  if (p == 1) {
    driveState = STOP;
    driveStop();
    qtrCalibrate();
  }
}

#endif
