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
    
    if(readCalibrationData()) {
        scale.set_scale(calibData.calibrationFactor);
        scale.set_offset(calibData.offset);
        calibValid = true;
    } else {
        scale.set_scale();
        calibValid = false;
    }
    
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
    if(prefs.isKey(prefsName.c_str())) {
        prefs.getBytes("calibData", &calibData, sizeof(LoadCellCalibData));
        prefs.end();
        return true;
    }
    prefs.end();
    return false;
}

float LoadCellHX711::getForce() {
    float force = scale.get_units(1);
    
    if(log_level > 0) {
        Serial.printf("Raw force: %.2f\n", force);
    }
    
    return force;
}

void LoadCellHX711::calibrate() {
    logStream->printf("Calibrating load cell \'%s\'\n", prefsName.c_str());
    logStream->println("Remove all weight");
    scale.set_scale();
    delay(2000);
    logStream->println("Tare...");  
    scale.tare();
    logStream->println("Tare done");
    
    logStream->println("Place 1kg weight and wait...");
    delay(2000);

    logStream->println("Calibrate scale...");

    calibData.calibrationFactor = 1000.0f / scale.get_units(20);
    scale.set_scale(calibData.calibrationFactor);

    logStream->println("Calibrate scale done");
    calibData.offset = scale.get_offset();
    
    saveCalibrationData();
    calibValid = true;
    
    if(log_level > 0) {
        logStream->printf("Calibration factor: %.3f\n", calibData.calibrationFactor);
        logStream->printf("Offset: %.1f\n", calibData.offset);
    }
}

void LoadCellHX711::tare() {
    scale.tare();
    calibData.offset = scale.get_offset();
    if(calibValid) {
        saveCalibrationData();
    }
}

bool LoadCellHX711::isCalibrationValid() {
    return calibValid;
} 

void LoadCellHX711::setLogStream(Stream* stream) {
    logStream = stream;
}