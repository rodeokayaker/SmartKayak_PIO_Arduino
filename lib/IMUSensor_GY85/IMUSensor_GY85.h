/**
 * @file IMUSensor_GY85.h
 * @brief Библиотека для работы с IMU модулем GY-85
 * 
 * Обеспечивает работу с IMU модулем GY-85, включая:
 * - Инициализацию и калибровку датчиков
 * - Чтение данных с акселерометра, гироскопа и магнитометра
 * - Вычисление ориентации с помощью фильтра Madgwick
 * - Сохранение/загрузку калибровочных данных
 * - Логирование данных
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#ifndef IMUSensor_GY85_h
#define IMUSensor_GY85_h

#include "SmartPaddle.h"
#include <ITG3200.h>
#include <MechaQMC5883.h>
#include <ADXL345.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

/** @brief Частота обновления IMU по умолчанию (Гц) */
#define GY85_IMU_DEFAULT_FREQUENCY 98


/**
 * @brief Класс для работы с IMU модулем GY-85
 */
class IMUSensor_GY85 : public IIMU {
private:
    // Датчики
    ADXL345 accel;              ///< Акселерометр
    ITG3200 gyro;               ///< Гироскоп
    MechaQMC5883 mag;           ///< Магнитометр
    
    // Данные и состояние
    IMUData currentData;         ///< Текущие данные с датчиков
    bool calibValid;             ///< Флаг валидности калибровки
    IMUCalibData calibData;      ///< Калибровочные данные
    uint32_t autoCalibCount;  ///< Счетчик итераций автокалибровки

    
    // Обработка ориентации
    Madgwick madgwick;          ///< Фильтр Madgwick
    int16_t imuFrequency;       ///< Частота обновления данных
    OrientationData currentOrientation; ///< Текущая ориентация
    
    // Логирование
    int8_t log_imu;             ///< Уровень логирования (0 - отключено)
    Stream* logStream;           ///< Поток для логирования
    std::string prefsName;       ///< Имя для сохранения настроек

    // Методы калибровки


    void adaptiveCalibrateMagnetometer(float* q);
    void initialCalibrateMagnetometer(bool ransac = true, bool geometric = true);

public:
    /**
     * @brief Конструктор класса IMUSensor_GY85
     * @param prefs_Name Имя для сохранения настроек
     * @param logStream Поток для логирования (по умолчанию Serial)
     */
    IMUSensor_GY85(const char* prefs_Name, Stream* logStream = nullptr);

    // Методы для работы с калибровкой
    void resetCalibration();               ///< Сбросить калибровку
    void setDefaultCalibration();          ///< Установить калибровку по умолчанию
    void saveCalibrationData();            ///< Сохранить калибровку
    bool readCalibrationData();            ///< Загрузить калибровку
    void setCalibrationData(const IMUCalibData data, bool save = false) override;
    IMUCalibData getCalibrationData() override;

    // Реализация интерфейса IIMU
    bool begin() override;                 
    void calibrate() override;             
    void calibrateCompass() override;
    void getData(IMUData& data) override;  
    IMUData getData() override;            
    bool isCalibrationValid() override;    
    void update();                
    OrientationData getOrientation() override; 
    void setLogStream(Stream* stream = &Serial) override;

    // Методы настройки
    void setLogLevel(int8_t level);        
    void setFrequency(int16_t frequency);  
    int16_t getFrequency();                

    // Деструктор
    ~IMUSensor_GY85();
};

#endif