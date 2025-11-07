/**
 * @file ImuBNO055.h
 * @brief BNO055 IMU sensor implementation using new ImuSensor interface
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#ifndef IMU_BNO055_H
#define IMU_BNO055_H

#include <Arduino.h>
#include "../Core/Interfaces/IIMUSensor.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Preferences.h>

/**
 * @class ImuBNO055
 * @brief BNO055 IMU sensor implementation using IIMUSensor interface
 */
class ImuBNO055 : public IIMUSensor {
private:
    // BNO055 sensor instance
    Adafruit_BNO055 bno;
    
    // Configuration
    String prefsName;
    uint8_t i2cAddress;
    
    // Data storage
    IMUData currentData;
    OrientationData currentOrientation;
    bool haveNewData;
    bool haveNewOrientation;
    
    // Calibration data
    adafruit_bno055_offsets_t sensorOffsets;
    bool calibrationSaved;
    uint32_t lastSavedCalibrationTime;
    static const uint32_t CALIBRATION_SAVE_INTERVAL = 600000;
    
    // Calibration status
    uint8_t system_cal, gyro_cal, accel_cal, mag_cal;
    
    // Timing for polling mode
    uint32_t lastDataUpdate;
    uint32_t lastOrientationUpdate;
    
    // Private methods
    void checkAndSaveCalibration();
    bool readCalibrationData();
    void saveCalibrationData();
    void setSensorCalibration();
    void getSensorCalibration();
    bool updateSensorData();
    bool updateOrientationData();

public:
    // Constructor & Destructor
    ImuBNO055(const char* name, uint8_t i2cAddr = BNO055_ADDRESS_B, 
              int32_t sensorID = -1, Stream* stream = &Serial);
    ~ImuBNO055();

    // Implementation of pure virtual methods from IIMUSensor
    bool getOrientation(OrientationData& orientation) override;
    bool getData(IMUData& data) override;
    bool orientationDataReady() override;
    bool imuDataReady() override;
    uint16_t calibrate() override;
    uint16_t getCalibrationStatus() override;
    void getCalibrationData(void* data) override;
    void setCalibrationData(void* data) override;
    uint16_t calibrationDataSize() override;

    // Overridden virtual methods
    bool setOrientationFrequency(uint16_t frequency, bool emulate = false) override;
    bool setIMUFrequency(uint16_t frequency, bool emulate = false) override;
    void startServices() override;
    void stopServices() override;

    // BNO055 specific methods
    bool begin();
    void getCalibrationStatus(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
    bool isFullyCalibrated();
    void printCalibrationStatus();
    float getTemperature();
    void resetCalibration();

    // FreeRTOS task handling (ESP32 only)
    #if defined(ARDUINO_ARCH_ESP32)
    static TaskHandle_t _taskHandle;
    static void taskEntry(void* arg);
    #endif
};

#endif
