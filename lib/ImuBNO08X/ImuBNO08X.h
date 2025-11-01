/**
 * @file ImuBNO08X.h
 * @brief BNO08X IMU sensor implementation using SH2 protocol
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#ifndef IMU_BNO085_H
#define IMU_BNO085_H

#include <Arduino.h>
#include "../Core/Interfaces/IIMUSensor.h"
#include <Wire.h>
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

/**
 * @class ImuBNO08X
 * @brief BNO08X IMU sensor implementation using SH2 protocol
 */
class ImuBNO08X : public IIMUSensor {
private:
    // Configuration
    String prefsName;

    // Static data storage (shared across all instances)
    static IMUData currentData;
    static OrientationData currentOrientation;
    static int accuracy[6];

    // Hardware configuration
    static int8_t _interruptPin;
    static uint8_t _i2cAddress;
    static int8_t _rstPin;

    // SH2 protocol and I2C handling
    static TwoWire* _i2c;
    static sh2_Hal_t _hal;
    static volatile bool _haveEvent;
    static volatile bool _haveOrientation;
    static volatile bool _haveIMU;
    static sh2_SensorValue_t _value;
    static const size_t kI2CMax = 32;
    static bool started;

    // FreeRTOS task & interrupt handling (ESP32 only)
    #if defined(ARDUINO_ARCH_ESP32)
    static TaskHandle_t _taskHandle;
    static void taskEntry(void* arg);
    static void IRAM_ATTR intISR();
    #endif

    // SH2 HAL methods
    static bool waitForIntIfNeeded();
    static void hwReset();
    static uint32_t hal_getTimeUs(sh2_Hal_t* hal);
    static int i2c_open(sh2_Hal_t* hal);
    static void i2c_close(sh2_Hal_t* hal);
    static bool i2c_write_bytes(const uint8_t* buf, size_t len, bool sendStop);
    static bool i2c_read_bytes(uint8_t* buf, size_t len, bool sendStop);
    static int i2c_write(sh2_Hal_t* hal, uint8_t* pBuffer, unsigned len);
    static int i2c_read(sh2_Hal_t* hal, uint8_t* pBuffer, unsigned len, uint32_t* t_us);
    static void hal_event(void* cookie, sh2_AsyncEvent_t* e);
    static void sensor_cb(void* cookie, sh2_SensorEvent_t* ev);
    static bool enableReport(sh2_SensorId_t id, uint32_t interval_us, uint32_t sensorSpecific = 0);
    void setReports();

public:
    // Constructor & Destructor
    ImuBNO08X(const char* name, Stream* stream = &Serial) : 
        IIMUSensor(stream), prefsName(name) {}
    ~ImuBNO08X() {}

    // Implementation of pure virtual methods from IIMUSensor
    bool getOrientation(OrientationData& orientation) override;
    bool getData(IMUData& data) override;
    bool orientationDataReady() override;
    bool imuDataReady() override;
    uint16_t calibrate() override { return 0; }
    uint16_t getCalibrationStatus() override;
    void getCalibrationData(void* data) override;
    void setCalibrationData(void* data) override;
    uint16_t calibrationDataSize() override { return 0; }

    // BNO08X specific methods
    void setInterruptPin(int interruptPin) { this->_interruptPin = interruptPin; }
    int8_t getInterruptPin() { return _interruptPin; }
    int begin(TwoWire* i2c = &Wire, uint8_t i2cAddress = 0, int8_t interruptPin = -1, int8_t rstPin = -1);
    void startServices() override;
    void stopServices() override;
    bool setOrientationFrequency(uint16_t frequency, bool emulate = false) override;
    bool setIMUFrequency(uint16_t frequency, bool emulate = false) override;
    
    // Дополнительные методы для диагностики и восстановления
    bool forceReset();
    bool scanI2CDevices();
    bool testConnection();
};

#endif