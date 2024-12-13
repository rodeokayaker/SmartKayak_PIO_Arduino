/**
 * @file IMUSensor_GY85.h
 * @brief Библиотека для работы с IMU модулем GY-85
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 * 
 * Обеспечивает работу с IMU модулем GY-85, включая:
 * - Инициализацию и калибровку датчиков
 * - Чтение и обработку данных
 * - Сохранение калибровочных данных в EEPROM
 */

#ifndef IMUSensor_GY85_h
#define IMUSensor_GY85_h

#include "SmartPaddle.h"
#include <ITG3200.h>
#include <MechaQMC5883.h>
#include <ADXL345.h>
#include <Wire.h>

/**
 * @brief Структура для хранения калибровочных данных IMU датчиков
 * 
 * Содержит масштабирующие коэффициенты и смещения для всех осей
 * каждого из датчиков IMU модуля GY-85:
 * - ADXL345 (акселерометр)
 * - ITG3200 (гироскоп)
 * - QMC5883L (магнитометр)
 */
struct IMUCalibData {
    // Калибровочные данные акселерометра
    int16_t accelOffset[3];  // Смещения нуля по трем осям
    float accelScale[3];     // Масштабирующие коэффициенты для приведения к м/с²
    
    // Калибровочные данные гироскопа
    int16_t gyroOffset[3];   // Смещения нуля по трем осям
    float gyroScale[3];      // Масштабирующие коэффициенты для приведения к рад/с
    
    // Калибровочные данные магнитометра
    int16_t magOffset[3];    // Смещения нуля по трем осям
    float magScale[3];       // Масштабирующие коэффициенты для нормализации
};

/**
 * @brief Класс для работы с IMU модулем GY-85
 * 
 * Обеспечивает:
 * - Инициализацию всех датчиков
 * - Калибровку датчиков
 * - Чтение данных с учетом калибровки
 * - Сохранение/загрузку калибровочных данных в/из EEPROM
 * - Логирование данных
 */
class IMUSensor_GY85 : public IIMU {
private:
    ADXL345& accel;              // Акселерометр
    ITG3200& gyro;               // Гироскоп
    MechaQMC5883& mag;           // Магнитометр
    IMUData currentData;          // Текущие данные с датчиков
    bool calibValid;              // Флаг валидности калибровки
    IMUCalibData calibData;       // Калибровочные данные
    int8_t log_imu;              // Уровень логирования (0 - отключено)
    Stream* logStream;            // Поток для логирования
     
public:
    /**
     * @brief Конструктор класса IMUSensor
     * 
     * @param a Ссылка на объект акселерометра
     * @param g Ссылка на объект гироскопа
     * @param m Ссылка на объект магнитометра
     * @param imuAddr Адрес в EEPROM для хранения калибровочных данных
     * @param imuValidFlagAddr Адрес в EEPROM для флага валидности калибровки
     */
    IMUSensor_GY85(ADXL345& a, ITG3200& g, MechaQMC5883& m, 
               Stream* logStream = nullptr);

    // Методы для работы с калибровкой
    IMUCalibData& getCalibrationData();    // Получить текущие калибровочные данные
    void resetCalibration();               // Сбросить калибровку
    void setDefaultCalibration();          // Установить калибровку по умолчанию
    void saveCalibrationData();            // Сохранить калибровку в EEPROM
    bool readCalibrationData();            // Загрузить калибровку из EEPROM

    // Реализация интерфейса IIMU
    bool begin() override;                 // Инициализация датчиков
    void calibrate() override;             // Выполнить калибровку
    void getData(IMUData& data) override;  // Получить данные с датчиков
    IMUData getData() override;            // Получить данные (альтернативный метод)
    bool isCalibrationValid() override;    // Проверить валидность калибровки

    /**
     * @brief Установить уровень логирования
     * 
     * @param level Уровень логирования:
     *             0 - логирование отключено
     *             1 - базовое логирование
     *             2 - расширенное логирование
     */
    void setLogLevel(int8_t level) {
        log_imu = level;
    };

    void setLogStream(Stream* stream = nullptr) override {
        if (stream) logStream = stream; else logStream = &Serial;
    };
};

#endif