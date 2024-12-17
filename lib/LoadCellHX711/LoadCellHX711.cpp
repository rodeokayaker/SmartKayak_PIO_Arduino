/**
 * @file LoadCellHX711.cpp
 * @brief Реализация библиотеки для работы с тензодатчиками через HX711
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#include "LoadCellHX711.h"
#include <Preferences.h>

constexpr byte VALID_CALIB_FLAG = 0x42;

LoadCellHX711::LoadCellHX711(const char* prefs_Name, uint8_t dout_pin, uint8_t sclk_pin)
    : scale(),
      calibValid(false),
      log_level(0),
      prefsName(prefs_Name),
      doutPin(dout_pin),
      sclkPin(sclk_pin),
      logStream(&Serial) {
    calibData.calibrationFactor = 1.0f;
    calibData.offset = 0.0f;
}

bool LoadCellHX711::begin() {

    scale.begin(doutPin, sclkPin);

    if(!scale.wait_ready_timeout(1000)) {
        logStream->println("HX711 not found!");
        return false;
    }

    resetCalibration();
    calibValid=readCalibrationData();

    return true;
}

void LoadCellHX711::saveCalibrationData() {
    Preferences prefs;
    prefs.begin(prefsName.c_str(), false);
    prefs.putBytes("calibData", &calibData, sizeof(LoadCellCalibData));
    prefs.end();
}

bool LoadCellHX711::readCalibrationData() {
    Preferences prefs;
    prefs.begin(prefsName.c_str(), true);
    if(prefs.isKey("calibData")) {
        prefs.getBytes("calibData", &calibData, sizeof(LoadCellCalibData));
        prefs.end();
        return true;
    }
    prefs.end();
    return false;
}

float LoadCellHX711::getForce() {
    float force = (scale.read()-calibData.offset)*calibData.calibrationFactor;
    

    return force;
}

void LoadCellHX711::calibrate() {
    logStream->printf("Calibrating load cell \'%s\'\n", prefsName.c_str());
    logStream->println("Remove all weight");
    resetCalibration();

    delay(3000);
    int32_t sum = 0;
    for(int i = 0; i < 20; i++) {
        sum += scale.read();

        delay(10);
    }
    calibData.offset = sum / 20;

    logStream->println("Tare done");
    
    logStream->println("Place 1kg weight and wait...");
    delay(3000);
    sum = 0;
    for(int i = 0; i < 30; i++) {
        sum += scale.read()-calibData.offset;
        delay(10);
    }

    calibData.calibrationFactor = 10000.0f / (sum/30);


    saveCalibrationData();
    calibValid = true;
    
//    if(log_level > 0) {
        logStream->printf("Calibration factor: %.3f\n", calibData.calibrationFactor);
        logStream->printf("Offset: %d\n", calibData.offset);
        logStream->printf("Calibration Done\n");
//    }
}

void LoadCellHX711::resetCalibration() {
    calibData.offset = 0;
    calibData.calibrationFactor = 1.0f;
}

bool LoadCellHX711::isCalibrationValid() {
    return calibValid;
} 

void LoadCellHX711::setLogStream(Stream* stream) {
    logStream = stream;
}