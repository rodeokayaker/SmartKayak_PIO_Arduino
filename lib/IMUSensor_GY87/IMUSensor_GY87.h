/**
 * @file IMUSensor_GY87.h
 * @brief Библиотека для работы с IMU модулем GY-87
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#ifndef IMUSensor_GY87_h
#define IMUSensor_GY87_h

#include "SmartPaddle.h"
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
//#include <QMC5883LCompass.h>
#include <MechaQMC5883.h>
#include <iarduino_Pressure_BMP.h>
#include <MadgwickAHRS.h>

#define GY87_IMU_DEFAULT_FREQUENCY 100

class IMUSensor_GY87 : public IIMU {
private:
    MPU6050_6Axis_MotionApps20 mpu;                  // MPU6050 (акселерометр + гироскоп)
//    QMC5883LCompass compass;      // Магнитометр
    iarduino_Pressure_BMP baro;   // Барометр
    MechaQMC5883 mag;           ///< Магнитометр
    
    IMUData currentData;          // Текущие данные с датчиков
    bool calibValid;              // Флаг валидности калибровки
    IMUCalibData calibData;       // Калибровочные данные
    int8_t log_imu;              // Уровень логирования
    Stream* logStream;            // Поток для логирования
    std::string prefsName;         // Имя раздела в Preferences для хранения калибровки
    int16_t imuFrequency;          // Частота обновления IMU
    Madgwick madgwick;
    int interruptPin;
    uint32_t autoCalibCount;  ///< Счетчик итераций автокалибровки

    OrientationData currentOrientation; ///< Текущая ориентация

    // DMP переменные
    bool dmpReady;
    uint8_t devStatus;
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];
    bool autoCalibrateMag;

    // Переменные ориентации
    Quaternion q;
    VectorFloat gravity;

    // Дополнительные данные
    float pressure;    // Давление в Па
    float temperature; // Температура в °C
    float altitude;    // Высота в метрах
    

    // Методы калибровки
    void adaptiveCalibrateMagnetometer(float* q);   
    void initialCalibrateMagnetometer(bool ransac = true, bool geometric = true);
    
public:
    IMUSensor_GY87(const char* prefsName, Stream* logStream = &Serial);
    
    // Методы для работы с калибровкой
    void resetCalibration();
    void setDefaultCalibration();
    void saveCalibrationData();
    bool readCalibrationData();
    void setInterruptPin(int pin) {interruptPin = pin;};
    
    // Реализация интерфейса IIMU
    bool begin() override;
    void calibrate() override;
    void getData(IMUData& data) override;
    IMUData getData() override;
    bool isCalibrationValid() override;
    
    // Дополнительные методы
    void setLogLevel(int8_t level) { log_imu = level; }
    float getPressure() { return pressure; }
    float getTemperature() { return temperature; }
    float getAltitude() { return altitude; }
    
    // Получение ориентации
    void getYawPitchRoll(float* ypr);
    void getQuaternion(float* quat);
    ~IMUSensor_GY87();
    void update() override;
    OrientationData getOrientation() override;
    int16_t getFrequency();
    void setFrequency(int16_t frequency);

    void setLogStream(Stream* stream = &Serial) override {logStream = stream;};

    void getSmoothedReadings(int16_t* readings, int samples = 1000);
    bool isStable(int16_t* readings1, int16_t* readings2, int tolerance);
    void setAutoCalibrateMag(bool enable = true) { autoCalibrateMag = enable; }
    void setCalibrationData(const IMUCalibData data, bool save = false) override;
    IMUCalibData getCalibrationData() override;
};

#endif
