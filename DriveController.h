#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H

#include "ButtonController.h"
#include "MotorController.h"
#include "Global.h"
#include "QTRSensorController.h"
#include "PidController.h"

// TODO figure out wtf to do with driveState
// momentan pare ca folosesc doar werid state la ceva util

#define TIME_ONE_WHITE_TO_STOP 1000 // millis

// for calibration only these 3 varaibles
#define MOTOR_TURN_POWER 100
#define WEIRD_STATE_DURATION 100    // millis

enum class DriveState {
    STOPPED,
    TURN,
    WEIRD_STATE,
    FORWARD
} driveState = DriveState::STOPPED;

enum class TurnType {
    LEFT,
    RIGHT,
    AROUND,
    NO_TURN,
};

void driveInit() {
    motorInit(leftMotor);
    motorInit(rightMotor);
    // used to recalibrate the sensor
    buttonInit(button, BUTTON_PIN);
    // everything driving-related depends on this
    qtrInit();

    Serial.println("Driver Initialized");
}

void driveFollowLine(int16_t turnCompensation = 0) {
    double error = qtrGetBlackLinePosition();
    int16_t correction = pidCenterLine(error) + turnCompensation;

    clamp(correction, -MOTOR_RUN_POWER * 2, MOTOR_RUN_POWER * 2);
    int16_t correctedMotorSpeed = MOTOR_RUN_POWER - abs(correction);

    if (correction < 0) {
        // left correction
        motorRun(leftMotor, correctedMotorSpeed);
        motorRun(rightMotor, MOTOR_RUN_POWER);  
    } else {
        // right correction
        motorRun(leftMotor, MOTOR_RUN_POWER);
        motorRun(rightMotor, correctedMotorSpeed);
    }

    driveState = DriveState::FORWARD;
}

/*
 * run both motors based on power
 */
inline void driveRun(int16_t power) {
    // TODO account for PID compensation
    motorRun(leftMotor, power);
    motorRun(rightMotor, power);
    driveState = DriveState::FORWARD;
}

inline void driveRunLeft() {
    // TODO account for PID compensation
    motorRun(leftMotor, -MOTOR_TURN_POWER);
    motorRun(rightMotor, MOTOR_TURN_POWER);
    driveState = DriveState::TURN;
}

inline void driveRunRight() {
    // TODO account for PID compensation
    motorRun(leftMotor,   MOTOR_TURN_POWER);
    motorRun(rightMotor, -MOTOR_TURN_POWER);
    driveState = DriveState::TURN;
}

/*
 * stop both motors
 */
inline void driveStop() {
    motorStop(leftMotor);
    motorStop(rightMotor);
    driveState = DriveState::STOPPED;
}

/*
 * power motors on for d milliseconds
 * if power is positive go forward
 * if power is negative go backward
 * blocking!!
 */
// TODO dubious utility
void driveForMillis(int16_t power, int16_t d) {
    driveRun(power);
    delay(d);
    driveStop();
}

/*
 * if too much time is spent on white STOP
 */
//inline void driveCheckToStop(const int &t) {
    //if (t - millis() > TIME_ONE_WHITE_TO_STOP) {
        //driveState = DriveState::STOPPED;
    //}
//}

inline TurnType driveDetectTurn(const qtrPath &p) {
    if (p.leftPath) {
        return TurnType::LEFT;
    } else if (p.centerPath) {
        return TurnType::NO_TURN;
    } else if (p.rightPath) {
        return TurnType::RIGHT;
    } else {
        return TurnType::AROUND;
    }
}

// TODO can be made non blocking with a global driving 
// union state structure which holds all necessary details

/*
 * !!THIS IS A BLOCKING ACTION!!
 *
 * turning behavior
 * turn in the corresponding drirection until a line
 * is detected on the outer edge. then switch back to
 * line follow 
 *
 * !!THIS IS A BLOCKING ACTION!!
 */
inline void driveTurn(const TurnType &turnType) {
    if (turnType == TurnType::LEFT) {
        driveRunLeft();
    } else {
        driveRunRight();
    }

    while (true) {
        qtrReadCalibrated();
        if ((turnType == TurnType::LEFT && qtrDetectFarLeft())
                || (turnType != TurnType::LEFT && qtrDetectFarRight())) {
            break;
        }
    }
    driveFollowLine();
    driveState = DriveState::FORWARD;
}

inline void driveTurnHandle(const TurnType &turnType) {
    // TODO set return variable to turn character
    switch (turnType) {
        case TurnType::NO_TURN:
            driveFollowLine();
            break;
        case TurnType::LEFT:
        case TurnType::AROUND:
        case TurnType::RIGHT:
            driveTurn(turnType);
            break;
    }
    driveState = DriveState::FORWARD;
}

/*
 * I have two impelemntation ideas.
 * The first one is time based and drives forward past an intersection
 * for a certain amount of time.
 * The second one drives past an intersection until it doesn't see left or right
 * anymore and is time-based only for uturns
 *
 * TODO I'll have to test both, but I prefer the first one as it's easier to
 * implement
 * TODO ahah nvm le pot combina. pot avea un minimum time si check de st dr.
 * daca inca am stanga dreapta mai continui putin
 */
void drive() {
    // first variant
    static int32_t weirdStateStart = 0;
    static qtrPath cumulativePath = {0, 0, 0};

    if (driveState == DriveState::STOPPED) {
        return;
    }

    qtrPath pathNow = qtrGetPath();

    if (driveState == DriveState::WEIRD_STATE) {
        driveRun(MOTOR_TURN_POWER);
        cumulativePath |= pathNow;
        if (millis() - weirdStateStart > WEIRD_STATE_DURATION
                && !pathNow.leftPath && !pathNow.rightPath) {
            driveState = DriveState::TURN;
            TurnType turnType = driveDetectTurn(cumulativePath);
            // !! BLOCKING !!
            driveTurnHandle(turnType);
        }
    } else {
        if (!pathNow.centerPath 
                || pathNow.leftPath
                || pathNow.rightPath) {
            driveRun(MOTOR_TURN_POWER);
            cumulativePath = pathNow;
            weirdStateStart = millis();
            driveState = DriveState::WEIRD_STATE;
        } else {
            driveFollowLine();
            driveState = DriveState::FORWARD;
        }
    }
}

#endif
