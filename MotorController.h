#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Global.h"

struct motor {
  uint8_t inputPin1,
          inputPin2,
          enablePin;
};

motor leftMotor {LEFT_INPUT_PIN_2, LEFT_INPUT_PIN_1, LEFT_ENABLE_PIN};
motor rightMotor {RIGHT_INPUT_PIN_2, RIGHT_INPUT_PIN_1, RIGHT_ENABLE_PIN};

void motorInit(const motor &m) {
  pinMode(m.inputPin1, OUTPUT);
  pinMode(m.inputPin2, OUTPUT);
  pinMode(m.enablePin, OUTPUT);
}

// make motor turn at a certain speed
// TODO determine thresholds
void motorRun(const motor &m, int16_t power) {
  if (power >= 0) {
    // positive power => forward movement
    digitalWrite(m.inputPin1, HIGH);
    digitalWrite(m.inputPin2, LOW);
  } else {
    // negative power => backward movement
    digitalWrite(m.inputPin1, LOW);
    digitalWrite(m.inputPin2, HIGH);
  }
  analogWrite(m.enablePin, abs(power));
}

void motorStop(const motor &m) {
  digitalWrite(m.inputPin1, LOW);
  digitalWrite(m.inputPin2, LOW);
  analogWrite(m.enablePin, 0);
}

#endif
