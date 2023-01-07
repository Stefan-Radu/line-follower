#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

struct motor {
  uint8_t plusPin;
  uint8_t minusPin;
};

void initMotor(const motor &m) {
  pinMode(m.plusPin, OUTPUT);
  pinMode(m.minusPin, OUTPUT);
}

// make motor turn at a certain speed
// TODO determine thresholds
void runMotor(const motor &m, int16_t power) {
  if (power >= 0) {
    // positive power => forward movement
    analogWrite(m.plusPin, power);
    digitalWrite(m.minusPin, LOW);
  } else {
    // negative power => forward movement
    analogWrite(m.plusPin, LOW);
    digitalWrite(m.minusPin, -power);
  }
}

void stopMotor(const motor &m) {
  digitalWrite(m.plusPin, LOW);
  digitalWrite(m.minusPin, LOW);
}

#endif
