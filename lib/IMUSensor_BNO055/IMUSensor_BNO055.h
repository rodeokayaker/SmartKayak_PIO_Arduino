/**
 * @file IMUSensor_BNO055.h
 * @brief Библиотека для работы с IMU модулем BNO055
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#ifndef IMUSensor_BNO055_h
#define IMUSensor_BNO055_h

#include "InterfaceIMU.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>

#define BNO055_IMU_DEFAULT_FREQUENCY 100
#define BNO055_MAG_DEFAULT_FREQUENCY 20
#define BNO055_CALIBRATION_SAVE_INTERVAL 600000 // 10 minutes

class IMUSensor_BNO055 : public IIMU {
private:
    Adafruit_BNO055 bno;                  // BNO055 sensor
    
    IMUData currentData;          // Текущие данные с датчиков
    OrientationData currentOrientation; // Текущая ориентация
    IMUCalibData calibData;       // Калибровочные данные (для совместимости)
    Stream* logStream;            // Поток для логирования
    std::string prefsName;        // Имя раздела в Preferences (для совместимости)
    uint16_t imuFrequency;        // Частота обновления IMU
    uint16_t magFrequency;        // Частота магнитометра
    int interruptPin;             // Пин прерывания (не используется)
    
    bool sensorReady;             // Готовность сенсора
    uint8_t i2cAddress;           // I2C адрес
    bool calibrationSaved;        // Флаг сохранения калибровки
    uint32_t lastSavedCalibrationTime;    // Время последнего сохранения калибровки

    // Калибровочные статусы BNO055
    uint8_t system_cal, gyro_cal, accel_cal, mag_cal;
    
    
    // Калибровочные данные BNO055
    adafruit_bno055_offsets_t sensorOffsets;
    
    // Дополнительные калибровочные параметры для BNO055
    int16_t accelRadius;
    int16_t magRadius;
    
    // Внутренние методы
    void checkAndSaveCalibration();
    void getSensorCalibration();
    void setSensorCalibration();


public:
    IMUSensor_BNO055(const char* prefsName, uint8_t i2cAddr = BNO055_ADDRESS_A, 
                     int32_t sensorID = -1, Stream* logStream = &Serial);
    
    // Реализация интерфейса IIMU
    bool begin(uint16_t imuFreq, 
               uint16_t magFreq);
    virtual void begin() override { begin(BNO055_IMU_DEFAULT_FREQUENCY, BNO055_MAG_DEFAULT_FREQUENCY); };
    void calibrate() override;                    // Заглушка для BNO055
    void calibrateCompass() override;             // Заглушка для BNO055
    IMUData readData() override;
    IMUData getData() override;
    OrientationData updateOrientation() override;
    OrientationData getOrientation() override;
    bool isCalibrationValid() override;
    uint16_t magnetometerFrequency() override { return magFrequency; };
    bool DMPEnabled() override { return true; };  // BNO055 всегда имеет внутренний процессор
    int8_t interruptPIN() override { return interruptPin; };
    void magnetometerUpdate() override;           // Заглушка для BNO055
    uint16_t getFrequency() override { return imuFrequency; };
    void setFrequency(uint16_t frequency) { imuFrequency = frequency; };
    void setLogStream(Stream* stream = &Serial) override { logStream = stream; };
    void setCalibrationData(const IMUCalibData data, bool save = false) override;
    IMUCalibData getCalibrationData() override;
    bool DMPValid() override { return sensorReady; };
    int8_t getIntStatus() override { return 0; }; // BNO055 не использует прерывания

    // Дополнительные методы для BNO055
    void getCalibrationStatus(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
    bool isFullyCalibrated();
    void printCalibrationStatus();
    float getTemperature();
    
    // Методы калибровки
    void resetCalibration();
    bool readCalibrationData();
    void saveCalibrationData();

    void setInterruptPin(int interruptPin) { this->interruptPin = -1;}; // не используется
    
    ~IMUSensor_BNO055();
};

#endif