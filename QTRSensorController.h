#ifndef QTR_SENSOR_CONTROLLER_H
#define QTR_SENSOR_CONTROLLER_H

#include <QTRSensors.h>
#include <EEPROM.h>
#include "MotorController.h"
#include "Global.h"

#define SENSOR_COUNT 8
#define BLACK_THRESHOLD 400
#define EEPROM_CALIBRATION_DATA_ADDRESS 0

QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];

struct qtrPath {
    uint8_t leftPath   : 1,
            centerPath : 1,
            rightPath  : 1;

    bool operator==(const qtrPath &other) const {
        return this->leftPath   == other.leftPath
            && this->centerPath == other.centerPath
            && this->rightPath  == other.rightPath;
    }

    bool operator!=(const qtrPath &other) const {
        return !(*this == other);
    }

    void operator|=(const qtrPath &other) {
        this->leftPath   |= other.leftPath;
        this->centerPath |= other.centerPath;
        this->rightPath  |= other.rightPath;
    }
};

const qtrPath qtrNullPath = {0, 0, 0};

/*
 * just a wrapper to simplify the api
 */
inline void qtrReadCalibrated() {
    qtr.readCalibrated(sensorValues);
}

inline bool qtrIsBlack(uint16_t reading) {
    return reading > BLACK_THRESHOLD;
}

inline bool qtrDetectFarLeft() {
    return qtrIsBlack(sensorValues[0]);
}

inline bool qtrDetectFarRight() {
    return qtrIsBlack(sensorValues[SENSOR_COUNT - 1]);
}

/*
 * load data from past calibration
 * which was saved in EEPROM
 *
 * calibration data is defines as a minimum and maximum
 * threshold for each of the sensors
 */
void loadCalibrationData() {
    int16_t address = EEPROM_CALIBRATION_DATA_ADDRESS;

    if (qtr.calibrationOn.minimum == NULL) {
        qtr.calibrationOn.minimum = (uint16_t*) calloc(SENSOR_COUNT, sizeof(uint16_t));
    }
    for (int8_t i = 0; i < SENSOR_COUNT; ++i) {
        EEPROM.get(address, qtr.calibrationOn.minimum[i]);
        address += sizeof(qtr.calibrationOn.minimum[i]);
    }

    if (qtr.calibrationOn.maximum == NULL) {
        qtr.calibrationOn.maximum = (uint16_t*) calloc(SENSOR_COUNT, sizeof(uint16_t));
    }
    for (int8_t i = 0; i < SENSOR_COUNT; ++i) {
        EEPROM.get(address, qtr.calibrationOn.maximum[i]);
        address += sizeof(qtr.calibrationOn.maximum[i]);
    }

    qtr.calibrationOn.initialized = true;
    Serial.println("Calibration Data loaded");
}

/*
 * iterate all sensors and save minimum and maximum thresholds
 * to predefined addresses in the EEPROM
 * this is useful because the calibration can be done once
 * and reused later just by loading these values
 */
void saveCalibrationData() {
    int16_t address = EEPROM_CALIBRATION_DATA_ADDRESS;
    for (int i = 0; i < SENSOR_COUNT; ++i) {
        EEPROM.put(address, qtr.calibrationOn.minimum[i]);
        address += sizeof(qtr.calibrationOn.minimum[i]);
    }
    for (int i = 0; i < SENSOR_COUNT; ++i) {
        EEPROM.put(address, qtr.calibrationOn.maximum[i]);
        address += sizeof(qtr.calibrationOn.maximum[i]);
    }

    Serial.println("Calibration Data saved");
}

// TODO please test that the sensor orientation is as expected
/*
 * initialize qtr sensors and load previous calibration data from EEPROM
 */
void qtrInit() {
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SENSOR_COUNT);

    delay(500);
    Serial.println("QTR Initialized");

    loadCalibrationData();  
}

/*
 * return black line position as a number between -1 and 1
 * where -1 is far left and 1 is far right
 */
double qtrGetBlackLinePosition() {
    /* 
     * according to qtr documentation: 
     * res is between 0 and 1000 * SENSOR_COUNT
     */
    static const double linePosMinValue = 0;
    static const double linePosMaxValue = SENSOR_COUNT * 1000;

    static const double linePosMappedLow = -1.0;
    static const double linePosMappedHigh = 1.0;

    int16_t readLinePos = qtr.readLineBlack(sensorValues);
    double mappedLinePos = doubleMap(readLinePos, linePosMinValue,
            linePosMaxValue, linePosMappedLow, linePosMappedHigh);

#ifdef DEBUG
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.print(">> ");
    Serial.print(mappedLinePos);
    Serial.print(" <<");
    Serial.print('\n');
#endif

    return mappedLinePos;
}

/*
 * !! THIS IS BLOCKING !!
 * calibrate sensors
 * !! THIS IS BLOCKING !!
 */
void qtrCalibrate() {
    static const float calibrationOuterThreshold = 0.9;
    static const float calibrationInnerThreshold = 0.1;

    static const int16_t calibrationMotorPower = 100;

    int16_t motorPower = calibrationMotorPower; 

    Serial.println("QTR Sensor calibrating...");

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    // turn on Arduino's LED to indicate we are in calibration mode
    digitalWrite(LED_BUILTIN, HIGH);

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
        if ((linePos < -calibrationOuterThreshold && motorPower > 0) ||
                (linePos > calibrationOuterThreshold && motorPower < 0)) {
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
        if (linePos > -calibrationInnerThreshold && linePos < calibrationInnerThreshold) {
            break;
        }
    }

    motorStop(leftMotor);
    motorStop(rightMotor);

    // turn off Arduino's LED to indicate we are through with calibration
    digitalWrite(LED_BUILTIN, LOW);

    delay(500);
    Serial.println("QTR Sensor calibrated");

    // save calibration data to EEPROM
    saveCalibrationData();
}

inline uint8_t qtrCountBlackSensors() {
    uint8_t ret = 0;
    for (uint8_t idx = 0; idx < SENSOR_COUNT; ++idx) {
        if (qtrIsBlack(sensorValues[idx])) {
            ret += 1;
        }
    }

    return ret;
}

/*
 * return qtrPath struct reflecting 
 * where the sensor sees black
 * - far left
 * - far right
 * - center
 */
qtrPath qtrGetPath() {
    /*
     * TODO functia asta e apelata deja in qtrGetBlackLinePosition
     * care e apelata in functia de read follow line din drive
     * voi face o functie de qtrUpdate care trebuie apelata in fiecare
     * bucla si de care depinde resul codului
     * TREBUIE sa ma asigur ca alta citire de senzori nu are loc in alta parte
     * asta ar trebui sa minimizeze latency-ul foarte mult
     * TODO test pentru qtr read si print sensor data
     */
    qtrReadCalibrated(); // TODO vezi deasupra mare atentie la latency cu asta

    qtrPath ret { 
        .leftPath = qtrIsBlack(sensorValues[0]),
        .centerPath = qtrIsBlack(sensorValues[SENSOR_COUNT / 2 - 1])
            || qtrIsBlack(sensorValues[SENSOR_COUNT / 2]),
        .rightPath = qtrIsBlack(sensorValues[SENSOR_COUNT - 1])
    };

    return ret;
}

#endif
