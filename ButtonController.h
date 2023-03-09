#ifndef BUTTON_CONTROLLER_H
#define BUTTON_CONTROLLER_H

#define DEBOUNCE_DURATION 50

struct Button {
  uint8_t pin;
  bool state;
};

bool buttonInit(Button &b, const uint8_t buttonPin) {
  pinMode(buttonPin, INPUT_PULLUP);
  b = {buttonPin, 1, // pullup so not pressed is 1
  };
}

void buttonUpdateState(Button &b) {
  static uint8_t lastReading = 1; // not pressed is 1
  static uint32_t lastDebounceTime = 0;
  
  uint8_t newReading = digitalRead(b.pin);
  if (newReading != lastReading) {
    lastDebounceTime = millis();
  }

  if (millis() - lastDebounceTime > DEBOUNCE_DURATION) {
    if (newReading != b.state) {
      b.state = newReading;
    }
  }

  lastReading = newReading;
}

bool buttonDetectPress(Button &b) {
  bool lastState = b.state;
  buttonUpdateState(b); // do this so state gets updated no matter the outcome

  if (lastState == 0) {
    // if it's already pressed => NO new press
    return 0;
  }
  // if it was not pressed => YES if we press it now; NO otherwise.
  return !b.state;
}

#endif
