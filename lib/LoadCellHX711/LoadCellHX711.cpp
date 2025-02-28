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
      logStream(&Serial),
      readyToRead(false) {
    calibData.calibrationFactor = 1.0f;
    calibData.offset = 0.0f;
}

bool LoadCellHX711::begin(uint16_t freq) {

    scale.begin(doutPin, sclkPin);
    frequency = freq;

    if(!scale.wait_ready_timeout(1000)) {
        logStream->println("HX711 not found!");
        return false;
    }

    resetCalibration();
    calibValid=readCalibrationData();
    readyToRead = true;
    return true;
}

void LoadCellHX711::saveCalibrationData() {
    Preferences prefs;
    prefs.begin(prefsName.c_str(), false);
    prefs.putBytes("calibData", &calibData, sizeof(LoadCellCalibData));
    prefs.end();
    calibValid = true;
}

bool LoadCellHX711::readCalibrationData() {
    Preferences prefs;
    prefs.begin(prefsName.c_str(), true);
    if(prefs.isKey("calibData")) {
        prefs.getBytes("calibData", &calibData, sizeof(LoadCellCalibData));
        prefs.end();
        calibValid = true;
        return true;
    }
    prefs.end();
    return false;
}

float LoadCellHX711::getForce() {
    float force = calibData.calibrationFactor*(float)(lastReadData-calibData.offset);
    return force;
}

uint16_t LoadCellHX711::getFrequency() {
    return frequency;
}

int32_t LoadCellHX711::getRawForce() {
    return lastReadData;
}

bool LoadCellHX711::read() {
    if(!readyToRead) {
        return false;
    }
    lastReadData = scale.read();
    lastReadTime = millis();
    return true;
}

void LoadCellHX711::calibrate() {
    logStream->printf("Calibrating load cell \'%s\'\n", prefsName.c_str());
    logStream->println("Remove all weight");
    resetCalibration();

    delay(3000);
    int32_t sum = 0;
    for(int i = 0; i < 200; i++) {
        sum += scale.read();

        delay(10);
    }
    calibData.offset = sum / 200;

    logStream->println("Tare done");
    
    logStream->println("Place 1kg weight and wait...");
    delay(3000);
    sum = 0;
    for(int i = 0; i < 200; i++) {
        sum += scale.read()-calibData.offset;
        delay(10);
    }

    calibData.calibrationFactor = HX711_DEFAULT_CALIBRATION_FACTOR / ((float)sum/200);


    saveCalibrationData();
    
//    if(log_level > 0) {
        logStream->printf("Calibration factor: %.3f\n", calibData.calibrationFactor);
        logStream->printf("Offset: %d\n", calibData.offset);
        logStream->printf("Calibration Done\n");
//    }
}

void LoadCellHX711::calibrateScale(float weight) {
    int32_t sum = 0;
    for(int i = 0; i < 200; i++) {
        sum += scale.read()-calibData.offset;
        delay(10);
    }

    calibData.calibrationFactor = weight / ((float)sum/200);
    saveCalibrationData();
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

void LoadCellHX711::tare() {
    logStream->printf("Taring load cell \'%s\'\n", prefsName.c_str());
    logStream->println("Remove all weight");

    delay(3000);
    int32_t sum = 0;
    for(int i = 0; i < 50; i++) {
        sum += scale.read();

        delay(10);
    }
    logStream->println("Turn over");
    delay(3000);
    for(int i = 0; i < 50; i++) {
        sum += scale.read();

        delay(10);
    }
    logStream->println("Turn sideways");
    delay(3000);
    for(int i = 0; i < 50; i++) {
        sum += scale.read();
        delay(10);
    }
    logStream->println("Turn over");
    delay(3000);
    for(int i = 0; i < 50; i++) {
        sum += scale.read();
        delay(10);
    }

    calibData.offset = sum / 200;

    logStream->println("Tare done");
    saveCalibrationData();
}

void LoadCellHX711::reset(uint32_t delay_ms) {
    readyToRead = false;
    scale.power_down();
    vTaskDelay(pdMS_TO_TICKS(delay_ms));  // неблокирующая задержка 10 мс
    scale.power_up();
    readyToRead = true;
}