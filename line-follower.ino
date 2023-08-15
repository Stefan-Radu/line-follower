#include "DriveController.h"
#include "QTRSensorController.h"
#include "ButtonController.h"

void buttonHandler();

// TODO change debug code everywhere
void setup() {
    Serial.begin(9600);

#ifdef DEBUG
    Serial.println("Running in debug mode");
#endif

    driveInit();
    delay(1000);
}

// TODO check qtrsensor big comment regarding sensor reads
void loop(){
    buttonHandler();
    drive();
}

void buttonHandler() {
    bool p = buttonDetectPress(button);
    if (p == 1) {
        if (driveState != DriveState::STOPPED) {
            driveStop();
            // !! BLOCKING !!
            qtrCalibrate(); 
        } else {
            driveFollowLine();
        }
    }
}
