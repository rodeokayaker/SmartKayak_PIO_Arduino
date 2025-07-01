/**
 * @file IMUSensor_ICM20948.h
 * @brief Библиотека для работы с IMU модулем ICM-20948
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#ifndef IMUSensor_ICM20948_h
#define IMUSensor_ICM20948_h

#include "InterfaceIMU.h"
#include <Wire.h>
#include "ICM_20948.h" // SparkFun ICM-20948 library

#define ICM20948_IMU_DEFAULT_FREQUENCY 100
#define ICM20948_MAG_DEFAULT_FREQUENCY 20
#define ICM20948_CALIBRATION_SAVE_INTERVAL 600000 // 10 minutes
#define ICM20948_I2C_ADDRESS 0x69

class IMUSensor_ICM20948 : public IIMU {
private:
    ICM_20948_I2C icm;                    // ICM-20948 sensor object
    
    IMUData currentData;                  // Текущие данные с датчиков
    OrientationData currentOrientation;   // Текущая ориентация
    IMUCalibData calibData;               // Калибровочные данные
    Stream* logStream;                    // Поток для логирования
    std::string prefsName;                // Имя раздела в Preferences
    uint16_t imuFrequency;                // Частота обновления IMU
    uint16_t magFrequency;                // Частота магнитометра
    int interruptPin;                     // Пин прерывания
    
    bool sensorReady;                     // Готовность сенсора
    uint8_t i2cAddress;                   // I2C адрес
    bool calibrationSaved;                // Флаг сохранения калибровки
    uint32_t lastSavedCalibrationTime;    // Время последнего сохранения калибровки
    bool dmpEnabled;                      // DMP включен
    bool dmpValid;                        // DMP данные валидны
    
    // Калибровочные статусы
    bool gyroCalibrated;
    bool accelCalibrated;
    bool magCalibrated;
    bool systemCalibrated;
    
    // Переменные для отслеживания калибровки DMP
    bool calibrationInProgress;
    uint32_t calibrationStartTime;
    
    // Калибровочные данные ICM-20948
    struct ICM20948CalibData {
        int16_t gyroBias[3];
        int16_t accelBias[3];
        float magBias[3];
        float magScale[3];
    } icmCalibData;
    
    // Внутренние методы
    void checkAndSaveCalibration();
    void applySensorCalibration();
    void performAutoCalibration();
    bool initializeDMP();
    void updateFromDMP();
    void readRawSensorData();
    void scaleAndCalibrateData();

public:
    IMUSensor_ICM20948(const char* prefsName, uint8_t i2cAddr = ICM20948_I2C_ADDRESS, 
                       int interruptPin = -1, Stream* logStream = &Serial);
    
    bool begin(uint16_t imuFreq = ICM20948_IMU_DEFAULT_FREQUENCY, 
               uint16_t magFreq = ICM20948_MAG_DEFAULT_FREQUENCY);

    // Реализация интерфейса IIMU
    virtual void begin() override { begin(imuFrequency, magFrequency); };
    void calibrate() override;
    void calibrateCompass() override;
    IMUData readData() override;
    IMUData getData() override;
    OrientationData updateOrientation() override;
    OrientationData getOrientation() override;
    bool isCalibrationValid() override;
    uint16_t magnetometerFrequency() override { return magFrequency; };
    bool DMPEnabled() override { return dmpEnabled; };
    int8_t interruptPIN() override { return interruptPin; };
    void magnetometerUpdate() override;
    uint16_t getFrequency() override { return imuFrequency; };
    void setFrequency(uint16_t frequency) { imuFrequency = frequency; };
    void setLogStream(Stream* stream = &Serial) override { logStream = stream; };
    void setCalibrationData(const IMUCalibData data, bool save = false) override;
    IMUCalibData getCalibrationData() override;
    bool DMPValid() override { return dmpEnabled && dmpValid; };
    int8_t getIntStatus() override;

    // Дополнительные методы для ICM-20948
    void getCalibrationStatus(bool* sys, bool* gyro, bool* accel, bool* mag);
    bool isFullyCalibrated();
    void printCalibrationStatus();
    float getTemperature();
    void updateCalibrationStatus();
    
    // Методы калибровки
    void resetCalibration();
    bool readCalibrationData();
    void saveCalibrationData();
    void enableAutoCalibration(bool enable = true);
    bool isAutoCalibrationEnabled();

    void setInterruptPin(int pin) { interruptPin = pin; };
    
    ~IMUSensor_ICM20948();
};

#endif 