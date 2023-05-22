#ifndef QTR_SENSOR_CONTROLLER_H
#define QTR_SENSOR_CONTROLLER_H

#include <QTRSensors.h>
#include <EEPROM.h>
#include "MotorController.h"
#include "Global.h"


#define BLACK_THRESHOLD 400
//#define READINGS_CNT_TO_UPDATE_PATH 2
#define SMOOTH_READINGS_CNT_TO_UPDATE 5
#define EXP_SMOOTH_ALPHA 0.6f // 0.05 at power 10

QTRSensors qtr;

const uint8_t sensorCount = 6;
uint16_t sensorValues[sensorCount];

// for 3 readings
//enum PathType: uint8_t {
//  EMPTY,
//  LINE,
//  LEFT_TURN,
//  RIGHT_TURN,
//  CROSS_INTERSECTION,
//  T_INTERSECTION,
//  LEFT_T_INTERSECTION,
//  RIGHT_T_INTERSECTION,
//  DEAD_END,
//  DESTINATION
//};

// for 2 readings
enum PathType: uint8_t {
  EMPTY,
  LINE,
  LEFT_TURN,
  RIGHT_TURN,
  T_INTERSECTION,
  U_TURN,
  DESTINATION
};

enum ReadingType: uint8_t {
  NOTHING,
  VERTICAL,
  FULL_HORIZONTAL,
  LEFT_HORIZONTAL,
  RIGHT_HORIZONTAL
};

struct qtrPath {
  uint8_t left  : 1,
          front : 1,
          right : 1;

  bool operator==(const qtrPath &other) const {
    return this->left == other.left && 
           this->front == other.front &&
           this->right == other.right;
  }

  bool operator!=(const qtrPath &other) const {
    return !(*this == other);
  }
};

struct qtrSmoothSensorInfo {
  uint8_t readings;
  uint16_t values[sensorCount];
  PathType currentPathType;
  ReadingType pastReadings[2];
//  ReadingType pastReadings[3];
} smoothSensorInfo;

struct qtrBlackSensorCount {
  uint8_t leftSide: 2,
          rightSide: 2,
          total: 4;
};

void qtrInit() {
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, sensorCount);

  smoothSensorInfo = {
    .readings = 0,
    .values = {0, 0, 0, 0, 0, 0},
    .currentPathType = EMPTY,
    .pastReadings = {NOTHING, NOTHING}
  };
  
  delay(500);
  Serial.println("QTR Initialized");
}

void saveCalibrationData() {
  int16_t address = EEPROM_CALIBRATION_DATA_ADDRESS;
  for (int i = 0; i < sensorCount; ++i) {
    EEPROM.put(address, qtr.calibrationOn.minimum[i]);
    address += sizeof(qtr.calibrationOn.minimum[i]);
  }
  for (int i = 0; i < sensorCount; ++i) {
    EEPROM.put(address, qtr.calibrationOn.maximum[i]);
    address += sizeof(qtr.calibrationOn.maximum[i]);
  }

  Serial.println("Calibration Data saved");
}

void loadCalibrationData() {
  int16_t address = EEPROM_CALIBRATION_DATA_ADDRESS;
  
  if (qtr.calibrationOn.minimum == NULL) {
    qtr.calibrationOn.minimum = (uint16_t*) calloc(sensorCount, sizeof(uint16_t));
  }
  for (int8_t i = 0; i < sensorCount; ++i) {
    EEPROM.get(address, qtr.calibrationOn.minimum[i]);
    address += sizeof(qtr.calibrationOn.minimum[i]);
  }

  if (qtr.calibrationOn.maximum == NULL) {
    qtr.calibrationOn.maximum = (uint16_t*) calloc(sensorCount, sizeof(uint16_t));
  }
  for (int8_t i = 0; i < sensorCount; ++i) {
    EEPROM.get(address, qtr.calibrationOn.maximum[i]);
    address += sizeof(qtr.calibrationOn.maximum[i]);
  }

  qtr.calibrationOn.initialized = true;
  Serial.println("Calibration Data loaded");
}

/*
 * return black line position as a number between -1 and 1
 * where -1 is far left
 * and 1 is far right
 */
double qtrGetBlackLinePosition(bool debug = false) {
  /* 
   * according to qtr documentation: 
   * res is between 0 and 5000 
   * I want to map it to a better interval 
   */
  int16_t res = qtr.readLineBlack(sensorValues);
  double linePos = doubleMap(res, SENSOR_MIN_VALUE, SENSOR_MAX_VALUE, LINE_POS_FAR_LEFT, LINE_POS_FAR_RIGHT);

  if (debug) {
    // TODO remove this when done with it
    for (uint8_t i = 0; i < sensorCount; ++i) {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.print(">> ");
    Serial.print(linePos);
    Serial.print(" <<");
    Serial.print('\n');
  }
  
  return linePos;
}

void qtrCalibrate() {
  Serial.println("QTR Sensor calibrating...");
  
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  int16_t motorPower = MAX_MOTOR_SPEED;
  
  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    double linePos = qtrGetBlackLinePosition();
    /*
     * TODO write oscialate() function
     * oscilate between the left part of the line
     * and the right side of the line
     */
    if ((linePos < -CALIBRATION_OUTER_THRESHOLD && motorPower > 0) ||
        (linePos > CALIBRATION_OUTER_THRESHOLD && motorPower < 0)) {
      motorPower *= -1;
    }
    motorRun(leftMotor, motorPower);
    motorRun(rightMotor, -motorPower);
  }

  /*
   * TODO fix this as it's brokend and throws the robot sideways
   * do this to somewhat center the robot on
   * the line after callibration
   */
  motorPower *= -1;
  motorRun(leftMotor, motorPower);
  motorRun(rightMotor, -motorPower);
  while (1) {
    double linePos = qtrGetBlackLinePosition();
    if (linePos > -CALIBRATION_INNER_THRESHOLD && linePos < CALIBRATION_INNER_THRESHOLD) {
      break;
    }
  }

  motorStop(leftMotor);
  motorStop(rightMotor);
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  
  delay(500);
  Serial.println("QTR Sensor calibrated");

  saveCalibrationData();
}

inline bool qtrIsBlack(uint16_t reading) {
  return reading > BLACK_THRESHOLD;
}

inline uint8_t qtrCountBlackSensors() {
  uint8_t ret = 0;
  for (uint8_t idx = 0; idx < sensorCount; ++idx) {
    if (qtrIsBlack(sensorValues[idx])) {
      ret += 1;
    }
  }

  return ret;
}

inline bool qtrBlackSensorsGTEOne() {
  for (uint8_t idx = 0; idx < sensorCount; ++idx) {
    if (qtrIsBlack(sensorValues[idx])) {
        return true;
    }
  }

  return false;
}

// TODO black count back to being just count
qtrBlackSensorCount qtrSmoothSensorBlackCount() {
  qtrBlackSensorCount ret = {0, 0, 0};
  for (int i = 0; i < sensorCount / 2; ++i) {
    ret.leftSide += qtrIsBlack(smoothSensorInfo.values[i]);
  }
  for (int i = sensorCount / 2; i < sensorCount; ++i) {
    ret.rightSide += qtrIsBlack(smoothSensorInfo.values[i]);
  }
  ret.total = ret.leftSide + ret.rightSide;
  return ret;
}

void qtrSmoothReadCalibrated() {
  /* 
   * do exponential smoothing to reduce errors
   * that means that the smooth calibrated value sv:
   * sv_n = sv_n * (1 - alpha) + alpha * reading_now 
   */
  qtr.readCalibrated(sensorValues);
  for (int i = 0; i < sensorCount; ++i) {
    smoothSensorInfo.values[i] = 1.0f * smoothSensorInfo.values[i] * (1 - EXP_SMOOTH_ALPHA) +
                                 EXP_SMOOTH_ALPHA * sensorValues[i];
  }
  smoothSensorInfo.readings += 1;
}

bool qtrSmoothUpdateReadings() {
  // return true if updated
  qtrSmoothReadCalibrated();

  if (smoothSensorInfo.readings == SMOOTH_READINGS_CNT_TO_UPDATE) {
    smoothSensorInfo.readings = 0;
   
    // shift down past readings to make room for new reading
//    smoothSensorInfo.pastReadings[2] = smoothSensorInfo.pastReadings[1];
    smoothSensorInfo.pastReadings[1] = smoothSensorInfo.pastReadings[0];
    
    qtrBlackSensorCount cnt = qtrSmoothSensorBlackCount();
    if (cnt.total < 2) {
      smoothSensorInfo.pastReadings[0] = NOTHING; 
    } else if (cnt.total <= 4) {
      smoothSensorInfo.pastReadings[0] = VERTICAL;
    } else if (cnt.total <= 5 && cnt.leftSide == 3) {
      smoothSensorInfo.pastReadings[0] = LEFT_HORIZONTAL;
    } else if (cnt.total <= 5 && cnt.rightSide == 3) {
      smoothSensorInfo.pastReadings[0] = RIGHT_HORIZONTAL;
    } else if (cnt.total == 6) {
      smoothSensorInfo.pastReadings[0] = FULL_HORIZONTAL;
    } else {
      smoothSensorInfo.pastReadings[0] = VERTICAL;
    }
    
    return 1;
  }

  return 0;
}

bool qtrUpdatedReadings() {
  return smoothSensorInfo.readings = 0;
}

void qtrSmoothUpdatePath() {
  bool updated = qtrSmoothUpdateReadings();
  if (!updated) {
    return;
  }

  if (smoothSensorInfo.pastReadings[0] == VERTICAL) {
    smoothSensorInfo.currentPathType = LINE;
  } else if (smoothSensorInfo.pastReadings[0] == LEFT_HORIZONTAL) {
    smoothSensorInfo.currentPathType = LEFT_TURN;
  } else if (smoothSensorInfo.pastReadings[0] == RIGHT_HORIZONTAL) {
    smoothSensorInfo.currentPathType = RIGHT_TURN;
  } else if (smoothSensorInfo.pastReadings[0] == FULL_HORIZONTAL) {
    smoothSensorInfo.currentPathType = T_INTERSECTION;
  } else if (smoothSensorInfo.pastReadings[0] == NOTHING) {
    if (smoothSensorInfo.pastReadings[1] == VERTICAL) {
      smoothSensorInfo.currentPathType = U_TURN;
    } else {
      smoothSensorInfo.currentPathType = EMPTY;
    }
  } else {
    smoothSensorInfo.currentPathType = LINE;
  }
}

qtrSmoothPrintPath() {
  String s = "";
  switch (smoothSensorInfo.currentPathType) {
    case EMPTY:
      s = "EMPTY";
      break;
    case LINE:
      s = "LINE";
      break;
    case LEFT_TURN:
      s = "LEFT_TURN";
      break;
    case RIGHT_TURN:
      s = "RIGHT_TURN";
      break;
    case U_TURN:
      s = "U_TURN";
      break;
    case T_INTERSECTION:
      s = "T_INTERSECTION";
      break;
  }

  Serial.println(s);
}

qtrPath qtrInterpretValues(int16_t* sensorValues) {
  /*
   * Get sensor values and output qtrPath struct reflecting 
   * an interpretation of the previous reading:
   *  - line in the center
   *  - line to the left (left turn)
   *  - line to the right (right turn)
   *  - horizontal line (t-intersection)
   */

  bool isBlackFarLeft = qtrIsBlack(sensorValues[0]);
  bool isBlackFarRight = qtrIsBlack(sensorValues[sensorCount - 1]);

  qtrPath res = {0, 0, 0};
  if (!isBlackFarLeft && !isBlackFarRight) {
    if (qtrBlackSensorsGTEOne()) {
      res.front = 1;
    }
  } else {
    if (isBlackFarLeft) {
      res.left = 1;
    }
    if (isBlackFarRight) {
      res.right = 1;
    }
  }

  return res;
}

//#define DEBOUNCE_CNT_TARGET 4
#define DEBOUNCE_TIME_TARGET 20

qtrPath qtrGetPath() {
  static int16_t consistentReadingsCnt = 0;
  static qtrPath previousPath = {0, 1, 0}, result = {0, 1, 0};
//  static int32_t debounceCnt = 0;
  static int32_t debounceTime = 0;
  
  qtr.readCalibrated(sensorValues);
  qtrPath currentPath = qtrInterpretValues(sensorValues);

//  if (currentPath == qtrPath{0, 0, 0}) {
//    Serial.println("nimiiic");
//    delay(1000);
//  }
//  Serial.print(currentPath.left);
//  Serial.print(currentPath.front);
//  Serial.println(currentPath.right);

//  if (currentPath != previousPath) {
//    debounceCnt = 1;
//  } else {
//    debounceCnt += 1;
//  }
  if (currentPath != previousPath) {
    debounceTime = millis();
  }

//  if (debounceCnt >= DEBOUNCE_CNT_TARGET) {
//    result = currentPath;
//  }
  if (millis() - debounceTime > DEBOUNCE_TIME_TARGET) {
    result = currentPath;
  }
  previousPath = currentPath;
  
  return result;
}

#endif
