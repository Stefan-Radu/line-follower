#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

struct motor {
  uint8_t inputPin1,
          inputPin2,
          enablePin;
};

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
