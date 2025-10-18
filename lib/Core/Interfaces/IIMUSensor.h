/**
 * @file IIMUSensor.h
 * @brief Unified interface for IMU sensors
 * 
 * This is the single IMU interface to be used across the project.
 * Based on the improved ImuSensor design with callback support.
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#ifndef CORE_I_IMU_SENSOR_H
#define CORE_I_IMU_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include "../Types.h"

/**
 * @class IIMUSensor
 * @brief Abstract base class for IMU sensor implementations
 */
class IIMUSensor {
protected:
    // Logging stream for debug output
    Stream* logStream;
    
    // Sensor frequency settings
    uint16_t orientationFrequency;
    uint16_t imuFrequency;

    // Event callback functions
    void (*orientationCb)(const OrientationData&);
    void (*imuDataCb)(const IMUData&);

public:
    /**
     * @brief Constructor
     * @param stream Pointer to Stream object for logging (default: Serial)
     */
    IIMUSensor(Stream* stream = &Serial) : 
        logStream(stream), 
        orientationFrequency(0), 
        imuFrequency(0),
        orientationCb(nullptr),
        imuDataCb(nullptr)
    {}
    
    // ========================================================================
    // PURE VIRTUAL METHODS - Must be implemented by derived classes
    // ========================================================================
    
    /**
     * @brief Get current orientation data
     * @param orientation Reference to OrientationData structure to fill
     * @return true if data was successfully retrieved, false otherwise
     */
    virtual bool getOrientation(OrientationData& orientation) = 0;
    
    /**
     * @brief Get current IMU sensor data
     * @param data Reference to IMUData structure to fill
     * @return true if data was successfully retrieved, false otherwise
     */
    virtual bool getData(IMUData& data) = 0;
    
    /**
     * @brief Check if new orientation data is available
     * @return true if new orientation data is ready, false otherwise
     */
    virtual bool orientationDataReady() = 0;
    
    /**
     * @brief Check if new IMU data is available
     * @return true if new IMU data is ready, false otherwise
     */
    virtual bool imuDataReady() = 0;

    /**
     * @brief Start sensor calibration process
     * @return Calibration status code (0 = not calibrated, higher = better calibration)
     */
    virtual uint16_t calibrate() = 0;
    
    /**
     * @brief Get calibration data from sensor
     * @param data Pointer to buffer to store calibration data
     */
    virtual void getCalibrationData(void* data) = 0;
    
    /**
     * @brief Set calibration data to sensor
     * @param data Pointer to calibration data buffer
     */
    virtual void setCalibrationData(void* data) = 0;

    // ========================================================================
    // VIRTUAL METHODS - Can be overridden by derived classes
    // ========================================================================
    
    /**
     * @brief Get current calibration status
     * @return Calibration status (0 = not calibrated, higher = better calibration)
     */
    virtual uint16_t getCalibrationStatus() { return 0; }
    
    /**
     * @brief Get size of calibration data in bytes
     * @return Size of calibration data buffer in bytes
     */
    virtual uint16_t calibrationDataSize() { return 0; }

    /**
     * @brief Set orientation update frequency
     * @param frequency Desired frequency in Hz
     * @param emulate If true, emulate the frequency using software timing
     * @return true if frequency was set successfully, false otherwise
     */
    virtual bool setOrientationFrequency(uint16_t frequency, bool emulate = false) { 
        orientationFrequency = frequency; 
        return false;
    }
    
    /**
     * @brief Set IMU data update frequency
     * @param frequency Desired frequency in Hz
     * @param emulate If true, emulate the frequency using software timing
     * @return true if frequency was set successfully, false otherwise
     */
    virtual bool setIMUFrequency(uint16_t frequency, bool emulate = false) { 
        imuFrequency = frequency; 
        return false;
    }

    /**
     * @brief Start sensor services (FreeRTOS tasks, interrupts, etc.)
     */
    virtual void startServices() {}
    
    /**
     * @brief Stop sensor services
     */
    virtual void stopServices() {}

    // ========================================================================
    // PUBLIC METHODS
    // ========================================================================
    
    /**
     * @brief Set logging stream for debug output
     * @param stream Pointer to Stream object for logging
     */
    void setLogStream(Stream* stream = &Serial) { logStream = stream; }

    /**
     * @brief Set callback function for orientation data
     * @param cb Function pointer to orientation data callback
     */
    void onOrientation(void (*cb)(const OrientationData&)) { orientationCb = cb; }
    
    /**
     * @brief Set callback function for IMU data
     * @param cb Function pointer to IMU data callback
     */
    void onIMUData(void (*cb)(const IMUData&)) { imuDataCb = cb; }

    /**
     * @brief Virtual destructor
     */
    virtual ~IIMUSensor() = default;
};

#endif // CORE_I_IMU_SENSOR_H

