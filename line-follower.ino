#include "MotorController.h"
#include "QTRSensorController.h"
#include "Global.h"

const motor leftMotor {LEFT_MOTOR_PLUS_PIN, LEFT_MOTOR_MINUS_PIN};
const motor rightMotor {RIGHT_MOTOR_PLUS_PIN, RIGHT_MOTOR_MINUS_PIN};

//This will run only one time.
void setup(){
  Serial.begin(9600);

//  initMotor(leftMotor);
//  initMotor(rightMotor);
//  
//  runMotor(leftMotor, true, 200);
//  runMotor(rightMotor, true, 200);
//  
//  delay(3000);
//  
//  stopMotor(rightMotor);
//  
//  delay(1000);
//  
//  runMotor(rightMotor, false, 255);
//  
//  stopMotor(leftMotor);
//  stopMotor(rightMotor);

  qtrInit();
  qtrCalibrate();
}


void loop(){
   qtrGetBlackLinePosition(true);
}
