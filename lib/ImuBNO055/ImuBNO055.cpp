/**
 * @file ImuBNO055.cpp
 * @brief Implementation of BNO055 IMU sensor using new ImuSensor interface
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#include "ImuBNO055.h"
#include <Arduino.h>

// Cross-platform sleep implementation
#if defined(ARDUINO_ARCH_ESP32)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
static inline void sleepMs(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }
#else
static inline void sleepMs(uint32_t ms) { delay(ms); }
#endif

#if defined(ARDUINO_ARCH_ESP32)
TaskHandle_t ImuBNO055::_taskHandle = nullptr;
#endif

ImuBNO055::ImuBNO055(const char* name, uint8_t i2cAddr, int32_t sensorID, Stream* stream) 
    : IIMUSensor(stream),
      bno(sensorID, i2cAddr),
      prefsName(name),
      i2cAddress(i2cAddr),
      haveNewData(false),
      haveNewOrientation(false),
      calibrationSaved(false),
      lastSavedCalibrationTime(0),
      lastDataUpdate(0),
      lastOrientationUpdate(0)
{
    memset(&currentData, 0, sizeof(IMUData));
    currentData.q0 = 1.0f;
    memset(&currentOrientation, 0, sizeof(OrientationData));
    currentOrientation.q0 = 1.0f;
    memset(&sensorOffsets, 0, sizeof(adafruit_bno055_offsets_t));
    
    system_cal = gyro_cal = accel_cal = mag_cal = 0;
}

ImuBNO055::~ImuBNO055() {
    stopServices();
}

bool ImuBNO055::begin() {
    logStream->println("Initializing BNO055...");
    
    if (!bno.begin()) {
        logStream->println("No BNO055 detected. Check wiring or I2C address!");
        return false;
    }
    logStream->println("BNO055 initialized successfully");
    
    sleepMs(1000);
    
    bno.setExtCrystalUse(true);
    
    if (readCalibrationData()) {
        logStream->println("Loading saved calibration data...");
        setSensorCalibration();
        calibrationSaved = true;
    } else {
        logStream->println("No saved calibration data found");
        calibrationSaved = false;
    }
    
    bno.setMode(OPERATION_MODE_NDOF);
    sleepMs(100);
    
    sensor_t sensor;
    bno.getSensor(&sensor);
    logStream->println("BNO055 initialized successfully");
    logStream->printf("Sensor: %s\n", sensor.name);
    logStream->printf("Driver Ver: %d\n", sensor.version);
    logStream->printf("Unique ID: %d\n", sensor.sensor_id);
    logStream->printf("Max Value: %f\n", sensor.max_value);
    logStream->printf("Min Value: %f\n", sensor.min_value);
    logStream->printf("Resolution: %f\n", sensor.resolution);
    
    return true;
}

bool ImuBNO055::getOrientation(OrientationData& orientation) {
    updateOrientationData();
    orientation = currentOrientation;
    haveNewOrientation = false;
    return true;
}

bool ImuBNO055::getData(IMUData& data) {
    updateSensorData();
    data = currentData;
    haveNewData = false;
    return true;
}

bool ImuBNO055::orientationDataReady() {
    updateOrientationData();
    return haveNewOrientation;
}

bool ImuBNO055::imuDataReady() {
    updateSensorData();
    return haveNewData;
}

uint16_t ImuBNO055::calibrate() {
    logStream->println("BNO055 has automatic calibration. No manual calibration needed.");
    logStream->println("Move the sensor in figure-8 motion for magnetometer calibration.");
    logStream->println("Keep device stable for accelerometer and gyroscope calibration.");
    
    uint32_t startTime = millis();
    while (millis() - startTime < 30000) {
        getCalibrationStatus(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
        
        if (system_cal >= 2 && gyro_cal >= 2 && accel_cal >= 2 && mag_cal >= 2) {
            logStream->println("Calibration completed successfully!");
            break;
        }
        
        if ((millis() - startTime) % 2000 == 0) {
            printCalibrationStatus();
        }
        sleepMs(100);
    }
    
    return getCalibrationStatus();
}

uint16_t ImuBNO055::getCalibrationStatus() {
    getCalibrationStatus(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
    return system_cal + gyro_cal + accel_cal + mag_cal;
}

void ImuBNO055::getCalibrationData(void* data) {
    if (data) {
        getSensorCalibration();
        memcpy(data, &sensorOffsets, sizeof(adafruit_bno055_offsets_t));
    }
}

void ImuBNO055::setCalibrationData(void* data) {
    if (data) {
        memcpy(&sensorOffsets, data, sizeof(adafruit_bno055_offsets_t));
        setSensorCalibration();
    }
}

uint16_t ImuBNO055::calibrationDataSize() {
    return sizeof(adafruit_bno055_offsets_t);
}

bool ImuBNO055::setOrientationFrequency(uint16_t frequency, bool emulate) {
    orientationFrequency = frequency;
    if (emulate) {
        logStream->println("BNO055: Orientation frequency emulation enabled");
        return true;
    }
    return false;
}

bool ImuBNO055::setIMUFrequency(uint16_t frequency, bool emulate) {
    imuFrequency = frequency;
    if (emulate) {
        logStream->println("BNO055: IMU frequency emulation enabled");
        return true;
    }
    return false;
}

void ImuBNO055::startServices() {
    #if defined(ARDUINO_ARCH_ESP32)
    if (_taskHandle == nullptr) {
        xTaskCreatePinnedToCore(taskEntry, "imu_bno055_task", 4096, this, 5, &_taskHandle, tskNO_AFFINITY);
        logStream->println("BNO055 polling task started");
    }
    #endif
}

void ImuBNO055::stopServices() {
    #if defined(ARDUINO_ARCH_ESP32)
    if (_taskHandle) {
        vTaskDelete(_taskHandle);
        _taskHandle = nullptr;
        logStream->println("BNO055 polling task stopped");
    }
    #endif
}

#if defined(ARDUINO_ARCH_ESP32)
void ImuBNO055::taskEntry(void* arg) {
    ImuBNO055* imu = (ImuBNO055*)arg;
    
    TickType_t last = xTaskGetTickCount();
    TickType_t period = pdMS_TO_TICKS(1000UL / (imu->imuFrequency == 0 ? 100 : imu->imuFrequency));
    
    for (;;) {
        vTaskDelayUntil(&last, period);
        
        if (imu->updateSensorData()) {
            imu->haveNewData = true;
            if (imu->imuDataCb) {
                imu->imuDataCb(imu->currentData);
            }
        }
        
        if (imu->updateOrientationData()) {
            imu->haveNewOrientation = true;
            if (imu->orientationCb) {
                imu->orientationCb(imu->currentOrientation);
            }
        }
        
        imu->checkAndSaveCalibration();
    }
}
#endif

void ImuBNO055::getCalibrationStatus(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
    bno.getCalibration(sys, gyro, accel, mag);
    system_cal = *sys;
    gyro_cal = *gyro;
    accel_cal = *accel;
    mag_cal = *mag;
}

bool ImuBNO055::isFullyCalibrated() {
    getCalibrationStatus(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
    return (system_cal == 3 && gyro_cal == 3 && accel_cal == 3 && mag_cal == 3);
}

void ImuBNO055::printCalibrationStatus() {
    getCalibrationStatus(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
    
    logStream->printf("Calibration Status: Sys=%d Gyro=%d Accel=%d Mag=%d\n",
                     system_cal, gyro_cal, accel_cal, mag_cal);
    
    if (system_cal < 3) logStream->println("- System calibration needed");
    if (gyro_cal < 3) logStream->println("- Gyroscope calibration needed (keep device stable)");
    if (accel_cal < 3) logStream->println("- Accelerometer calibration needed (place in different orientations)");
    if (mag_cal < 3) logStream->println("- Magnetometer calibration needed (move in figure-8 pattern)");
}

float ImuBNO055::getTemperature() {
    return bno.getTemp();
}

void ImuBNO055::resetCalibration() {
    memset(&sensorOffsets, 0, sizeof(adafruit_bno055_offsets_t));
    calibrationSaved = false;
    
    Preferences prefs;
    if (prefs.begin(prefsName.c_str(), false)) {
        prefs.remove("bno_calib");
        prefs.putBool("calib_valid", false);
        prefs.end();
    }
    
    logStream->println("BNO055 calibration data reset");
    
    bno.setMode(OPERATION_MODE_CONFIG);
    sleepMs(25);
    bno.setMode(OPERATION_MODE_NDOF);
    sleepMs(100);
}

void ImuBNO055::checkAndSaveCalibration() {
    if (calibrationSaved && ((CALIBRATION_SAVE_INTERVAL < 0) || 
        (millis() - lastSavedCalibrationTime < CALIBRATION_SAVE_INTERVAL))) {
        return;
    }
    
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    
    if (isFullyCalibrated()) {
        logStream->println("Calibration is fully calibrated");
        saveCalibrationData();
        lastSavedCalibrationTime = millis();
        calibrationSaved = true;
    }
}

bool ImuBNO055::readCalibrationData() {
    Preferences prefs;
    if (!prefs.begin(prefsName.c_str(), true)) {
        return false;
    }
    
    size_t dataSize = prefs.getBytesLength("bno_calib");
    if (dataSize != sizeof(adafruit_bno055_offsets_t)) {
        prefs.end();
        return false;
    }
    
    size_t readSize = prefs.getBytes("bno_calib", &sensorOffsets, sizeof(adafruit_bno055_offsets_t));
    bool validFlag = prefs.getBool("calib_valid", false);
    
    prefs.end();
    
    bool success = (readSize == sizeof(adafruit_bno055_offsets_t) && validFlag);
    
    return success;
}

void ImuBNO055::saveCalibrationData() {
    bool getSuccess = bno.getSensorOffsets(sensorOffsets);
    if (!getSuccess) {
        return;
    }
    
    Preferences prefs;
    if (!prefs.begin(prefsName.c_str(), false)) {
        return;
    }
    
    size_t written = prefs.putBytes("bno_calib", &sensorOffsets, sizeof(adafruit_bno055_offsets_t));
    prefs.putBool("calib_valid", true);
    
    prefs.end();
    logStream->printf("Calibration data saved to %s\n", prefsName.c_str());
}

void ImuBNO055::getSensorCalibration() {
    bno.getSensorOffsets(sensorOffsets);
}

void ImuBNO055::setSensorCalibration() {
    bno.setMode(OPERATION_MODE_CONFIG);
    sleepMs(25);
    
    bno.setSensorOffsets(sensorOffsets);
    bno.setMode(OPERATION_MODE_NDOF);
    sleepMs(100);
}

bool ImuBNO055::updateSensorData() {
    uint32_t now = millis();
    
    uint32_t interval = imuFrequency > 0 ? (1000 / imuFrequency) : 10;
    if (now - lastDataUpdate < interval) {
        return false;
    }
    
    lastDataUpdate = now;
    
    imu::Quaternion quat = bno.getQuat();
    currentData.q0 = quat.w();
    currentData.q1 = quat.x();
    currentData.q2 = quat.y();
    currentData.q3 = quat.z();
    
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    currentData.ax = accel.x();
    currentData.ay = accel.y();
    currentData.az = accel.z();
    
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    currentData.gx = gyro.x() * DEG_TO_RAD;
    currentData.gy = gyro.y() * DEG_TO_RAD;
    currentData.gz = gyro.z() * DEG_TO_RAD;
    
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    currentData.mx = mag.x();
    currentData.my = mag.y();
    currentData.mz = mag.z();
    
    currentData.mag_x = (int16_t)mag.x();
    currentData.mag_y = (int16_t)mag.y();
    currentData.mag_z = (int16_t)mag.z();
    
    currentData.timestamp = now;
    
    return true;
}

bool ImuBNO055::updateOrientationData() {
    uint32_t now = millis();
    
    uint32_t interval = orientationFrequency > 0 ? (1000 / orientationFrequency) : 10;
    if (now - lastOrientationUpdate < interval) {
        return false;
    }
    
    lastOrientationUpdate = now;
    
    currentOrientation.q0 = currentData.q0;
    currentOrientation.q1 = currentData.q1;
    currentOrientation.q2 = currentData.q2;
    currentOrientation.q3 = currentData.q3;
    currentOrientation.timestamp = now;
    
    return true;
}
