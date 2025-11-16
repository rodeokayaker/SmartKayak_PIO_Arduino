/**
 * @file test_imu_bno055.cpp
 * @brief Test file for new ImuBNO055 library
 * 
 * This file demonstrates how to use the new ImuBNO055 library with callback
 * support and automatic calibration handling.
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#include <Arduino.h>
#include "sensor-imu/ImuBNO055.h"

// Create IMU instance
ImuBNO055 imu("bno055_test", BNO055_ADDRESS_A, -1, &Serial);

// Callback functions
void onOrientationData(const OrientationData& orientation) {
    Serial.printf("Orientation Callback - Q: [%.3f, %.3f, %.3f, %.3f] Time: %lu\n",
                  orientation.q0, orientation.q1, orientation.q2, orientation.q3,
                  orientation.timestamp);
}

void onIMUData(const IMUData& data) {
    Serial.printf("IMU Callback - Accel: [%.3f, %.3f, %.3f] Gyro: [%.3f, %.3f, %.3f] Mag: [%.3f, %.3f, %.3f] Time: %lu\n",
                  data.ax, data.ay, data.az,
                  data.gx, data.gy, data.gz,
                  data.mx, data.my, data.mz,
                  data.timestamp);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== BNO055 Test with New Interface ===");
    
    // Initialize IMU
    if (!imu.begin()) {
        Serial.println("Failed to initialize BNO055!");
        while(1) {
            delay(1000);
        }
    }
    
    // Set frequencies
    imu.setOrientationFrequency(50, true);  // 50Hz with emulation
    imu.setIMUFrequency(100, true);         // 100Hz with emulation
    
    // Set callbacks
    imu.onOrientation(onOrientationData);
    imu.onIMUData(onIMUData);
    
    // Start services (FreeRTOS task for ESP32)
    imu.startServices();
    
    Serial.println("BNO055 initialized successfully!");
    Serial.println("Starting calibration monitoring...");
}

void loop() {
    static unsigned long lastPrint = 0;
    static unsigned long lastCalibCheck = 0;
    
    unsigned long now = millis();
    
    // Print calibration status every 2 seconds
    if (now - lastCalibCheck > 2000) {
        lastCalibCheck = now;
        
        uint8_t sys, gyro, accel, mag;
        imu.getCalibrationStatus(&sys, &gyro, &accel, &mag);
        
        Serial.printf("Calibration Status: Sys=%d Gyro=%d Accel=%d Mag=%d\n", 
                     sys, gyro, accel, mag);
        
        if (imu.isFullyCalibrated()) {
            Serial.println("*** FULLY CALIBRATED! ***");
        }
    }
    
    // Manual data reading (polling mode) - for demonstration
    if (now - lastPrint > 1000) {  // Every 1 second
        lastPrint = now;
        
        // Check if new data is available
        if (imu.orientationDataReady()) {
            OrientationData orientation;
            if (imu.getOrientation(orientation)) {
                Serial.printf("Manual Orientation - Q: [%.3f, %.3f, %.3f, %.3f]\n",
                             orientation.q0, orientation.q1, orientation.q2, orientation.q3);
            }
        }
        
        if (imu.imuDataReady()) {
            IMUData data;
            if (imu.getData(data)) {
                Serial.printf("Manual IMU - Accel: [%.3f, %.3f, %.3f] Gyro: [%.3f, %.3f, %.3f]\n",
                             data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
            }
        }
        
        // Print temperature
        float temp = imu.getTemperature();
        Serial.printf("Temperature: %.1f°C\n", temp);
    }
    
    delay(10);
}
