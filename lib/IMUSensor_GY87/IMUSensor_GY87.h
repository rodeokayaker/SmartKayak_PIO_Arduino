/**
 * @file IMUSensor_GY87.h
 * @brief Библиотека для работы с IMU модулем GY-87
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#ifndef IMUSensor_GY87_h
#define IMUSensor_GY87_h

#include "InterfaceIMU.h"
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MechaQMC5883.h>
#include <iarduino_Pressure_BMP.h>

#define GY87_IMU_DEFAULT_FREQUENCY 100
#define GY87_MAG_DEFAULT_FREQUENCY 40

class IMUSensor_GY87 : public IIMU {
private:
    MPU6050_6Axis_MotionApps20 mpu;                  // MPU6050 (акселерометр + гироскоп)
    iarduino_Pressure_BMP baro;   // Барометр
    MechaQMC5883 mag;           ///< Магнитометр
    
    IMUData currentData;          // Текущие данные с датчиков
    int calibValid;              // Флаг валидности калибровки
    IMUCalibData calibData;       // Калибровочные данные
    Stream* logStream;            // Поток для логирования
    std::string prefsName;         // Имя раздела в Preferences для хранения калибровки
    int16_t imuFrequency;          // Частота обновления IMU
    int interruptPin;
    uint32_t autoCalibCount;  ///< Счетчик итераций автокалибровки
    OrientationData currentOrientation; ///< Текущая ориентация
    uint16_t magFrequency;
    class OrientationEstimator* orientationEstimator;
    class Calibration* calibrator;

    // DMP переменные
    bool dmpReady;
    uint8_t devStatus;
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];
    bool useDMP;
    bool dmpValid;
    bool EKF_started;

    // Переменные ориентации DMP
    Quaternion q;

    // Дополнительные данные
    float pressure;    // Давление в Па
    float temperature; // Температура в °C
    float altitude;    // Высота в метрах

    int mag_call_count;


    // Методы калибровки
    void initialCalibrateMagnetometer(bool ransac = true, bool geometric = true);
    void getSmoothedReadings(int16_t* readings, int samples = 1000);
    bool isStable(int16_t* readings1, int16_t* readings2, int tolerance);

    void calculateCalibratedMagnetometer(int* mx, int* my, int* mz, float* mx_cal, float* my_cal, float* mz_cal);

public:
    IMUSensor_GY87(const char* prefsName, bool use_dmp=true, uint8_t interPin=-1, Stream* logStream = &Serial);
    
    // Методы для работы с калибровкой
    void resetCalibration();
    void setDefaultCalibration();
    void saveCalibrationData();
    bool readCalibrationData();
    void setInterruptPin(int pin) {interruptPin = pin;};
    
    // Реализация интерфейса IIMU
    bool begin( uint16_t imuFreq=GY87_IMU_DEFAULT_FREQUENCY, uint16_t magFreq=GY87_MAG_DEFAULT_FREQUENCY);
    void calibrate() override;
    void calibrateCompass() override;
    IMUData readData() override;
    IMUData getData() override;
    OrientationData updateOrientation() override;
    bool isCalibrationValid() override;
    uint16_t magnetometerFrequency() override {return magFrequency;};
    bool DMPEnabled() override {return useDMP && dmpReady;};
    int8_t interruptPIN() override {return interruptPin;};
    void magnetometerUpdate() override;
    
    // Дополнительные методы
    float getPressure() { return pressure; }
    float getTemperature() { return temperature; }
    float getAltitude() { return altitude; }
    
    // Получение ориентации
    void getYawPitchRoll(float* ypr);
    void getQuaternion(float* quat);
    ~IMUSensor_GY87();
    OrientationData getOrientation() override;
    uint16_t getFrequency() override {return imuFrequency;};
    void setFrequency(uint16_t frequency);

    void setLogStream(Stream* stream = &Serial) override {logStream = stream;};

    void setCalibrationData(const IMUCalibData data, bool save = false) override;
    IMUCalibData getCalibrationData() override;

    void PrintCalibrationData(Stream* stream = &Serial);
    void PrintChipsInfo(Stream* stream = &Serial);

    bool DMPValid() override {return useDMP && dmpReady && dmpValid;};
    int8_t getIntStatus() override {return mpu.getIntStatus();};
};

#endif