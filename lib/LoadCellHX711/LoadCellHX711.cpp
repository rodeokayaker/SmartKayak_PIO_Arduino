/**
 * @file LoadCellHX711.cpp
 * @brief Реализация библиотеки для работы с тензодатчиками через HX711
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#include "LoadCellHX711.h"

constexpr byte VALID_CALIB_FLAG = 0x42;

LoadCellHX711::LoadCellHX711(HX711& scale, int eepromAddr, int calibFlagAddr)
    : scale(scale), 
      eepromAddr(eepromAddr),
      calibFlagAddr(calibFlagAddr),
      calibValid(false),
      log_level(0) {
    calibData.calibrationFactor = 1.0f;
    calibData.offset = 0.0f;
}

bool LoadCellHX711::begin() {
    if(!scale.wait_ready_timeout(1000)) {
        Serial.println("HX711 not found!");
        return false;
    }
    
    // Чтение калибровки из EEPROM
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
    EEPROM.begin(512);
    EEPROM.write(calibFlagAddr, VALID_CALIB_FLAG);
    EEPROM.put(eepromAddr, calibData);
    EEPROM.commit();
    EEPROM.end();
}

bool LoadCellHX711::readCalibrationData() {
    EEPROM.begin(512);
    if(EEPROM.read(calibFlagAddr) == VALID_CALIB_FLAG) {
        EEPROM.get(eepromAddr, calibData);
        EEPROM.end();
        return true;
    }
    EEPROM.end();
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
    scale.set_scale();
    tare();
    
    Serial.println("Place 1kg weight and wait...");
    delay(5000);
    
    float reading = scale.get_units(10);
    calibData.calibrationFactor = reading / 1000.0f; // 1kg = 1000g
    calibData.offset = scale.get_offset();
    
    scale.set_scale(calibData.calibrationFactor);
    saveCalibrationData();
    calibValid = true;
    
    if(log_level > 0) {
        Serial.printf("Calibration factor: %.3f\n", calibData.calibrationFactor);
        Serial.printf("Offset: %.1f\n", calibData.offset);
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