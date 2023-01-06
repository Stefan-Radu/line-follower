#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

struct motor {
  int plusPin;
  int minusPin;
};


// make motor turn at a certain speed
// TODO determine thresholds
void runMotor(motor &m, bool forward, byte power) {
  if (forward) {
    analogWrite(m.plusPin, power);
    digitalWrite(m.minusPin, LOW);
  } else {
    analogWrite(m.plusPin, LOW);
    digitalWrite(m.minusPin, power);
  }
}

void stopMotor(motor &m) {
  digitalWrite(m.plusPin, LOW);
  digitalWrite(m.minusPin, LOW);
}

#endif
