#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#define RIGHT_INPUT_PIN_1 7
#define RIGHT_INPUT_PIN_2 6
#define RIGHT_ENABLE_PIN 11

#define LEFT_INPUT_PIN_1 5
#define LEFT_INPUT_PIN_2 4
#define LEFT_ENABLE_PIN 10

struct motor {
  uint8_t inputPin1,
          inputPin2,
          enablePin;
};

motor leftMotor {LEFT_INPUT_PIN_2, LEFT_INPUT_PIN_1, LEFT_ENABLE_PIN};
motor rightMotor {RIGHT_INPUT_PIN_2, RIGHT_INPUT_PIN_1, RIGHT_ENABLE_PIN};

/*
 * for motor operation 3 pins are needed:
 * INPUT1, INPUT2, ENABLE
 */
void motorInit(const motor &m) {
  pinMode(m.inputPin1, OUTPUT);
  pinMode(m.inputPin2, OUTPUT);
  pinMode(m.enablePin, OUTPUT);
}

/*
 * INPUT1 HIGH & INPUT2 LOW & power on ENABLE turns motor forward
 * INPUT1 LOW & INPUT2 HIGH & power on ENABLE turns motor backwards
 */
void motorRun(const motor &m, int16_t power) {
  if (power >= 0) {
    // positive power => forward movement
    digitalWrite(m.inputPin1, HIGH);
    digitalWrite(m.inputPin2, LOW);
    analogWrite(m.enablePin, power);
  } else {
    // negative power => backward movement
    digitalWrite(m.inputPin1, LOW);
    digitalWrite(m.inputPin2, HIGH);
    analogWrite(m.enablePin, -power);
  }
}

/*
 * INPUT1 LOW & INPUT2 LOW & 0 on ENABLE stops the motor
 */
void motorStop(const motor &m) {
  digitalWrite(m.inputPin1, LOW);
  digitalWrite(m.inputPin2, LOW);
  analogWrite(m.enablePin, 0);
}

#endif
