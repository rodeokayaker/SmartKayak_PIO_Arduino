/**
 * @file IMUSensor_GY87.h
 * @brief Библиотека для работы с IMU модулем GY-87
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 * 
 * Обеспечивает работу с IMU модулем GY-87, включая:
 * - Инициализацию и калибровку датчиков
 * - Чтение и обработку данных
 * - Сохранение калибровочных данных в EEPROM
 */

#ifndef IMUSensor_GY87_h
#define IMUSensor_GY87_h

#include "SmartPaddle.h"
#include <EEPROM.h>
#include <Wire.h>
#include <iarduino_Pressure_BMP.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <QMC5883LCompass.h>
#include <MadgwickAHRS.h>

// Структура для хранения калибровочных данных IMU
struct IMUCalibData {
    // Калибровочные данные MPU6050
    int16_t accelOffset[3];
    int16_t gyroOffset[3];
    
    // Калибровочные данные QMC5883L
    float magOffset[3];
    float magScale[3];
};

class IMUSensor : public IIMU {
private:
    MPU6050 mpu;                  // MPU6050 (акселерометр + гироскоп)
    QMC5883LCompass compass;      // Магнитометр
    iarduino_Pressure_BMP baro;   // Барометр
    Madgwick filter;              // Фильтр Madgwick
    
    IMUData currentData;          // Текущие данные с датчиков
    bool calibValid;              // Флаг валидности калибровки
    IMUCalibData calibData;       // Калибровочные данные
    const int imuCalibAddr;       // Адрес в EEPROM для калибровки
    const int imuValidFlagAddr;   // Адрес в EEPROM для флага валидности
    int8_t log_imu;              // Уровень логирования
    
    // DMP переменные
    bool dmpReady;
    uint8_t mpuIntStatus;
    uint8_t devStatus;
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];
    
    // Переменные ориентации
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
    
    // Преобразование сырых данных в физические величины
    float convertRawAcceleration(int16_t aRaw);
    float convertRawGyro(int16_t gRaw);
    float convertRawCompass(int mag);
    
public:
    IMUSensor(int imuAddr, int imuValidFlagAddr);
    
    // Методы для работы с калибровкой
    IMUCalibData& getCalibrationData();
    void resetCalibration();
    void setDefaultCalibration();
    void saveCalibrationData();
    bool readCalibrationData();
    
    // Реализация интерфейса IIMU
    bool begin() override;
    void calibrate() override;
    void getData(IMUData& data) override;
    IMUData getData() override;
    bool isCalibrationValid() override;
    
    void setLogLevel(int8_t level) { log_imu = level; }
};

#endif