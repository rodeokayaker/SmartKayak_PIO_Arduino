/**
 * @file LoadCellHX711.h
 * @brief Библиотека для работы с тензодатчиками через HX711
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 * 
 * Обеспечивает работу с тензодатчиками через HX711, включая:
 * - Инициализацию и калибровку датчиков
 * - Чтение данных с учетом калибровки
 * - Сохранение/загрузку калибровочных данных в/из EEPROM
 */

#ifndef LoadCellHX711_h
#define LoadCellHX711_h

#include "SmartPaddle.h"
#include <HX711.h>

/**
 * @brief Структура для хранения калибровочных данных тензодатчика
 */
struct LoadCellCalibData {

    float calibrationFactor;  // Коэффициент калибровки
    int32_t offset;            // Смещение нуля
};

/**
 * @brief Класс для работы с тензодатчиком через HX711
 */
class LoadCellHX711 : public ILoadCell {
private:
    HX711 scale;                 // Объект HX711
    LoadCellCalibData calibData;  // Калибровочные данные

    bool calibValid;              // Флаг валидности калибровки
    int8_t log_level;            // Уровень логирования
    Stream* logStream;            // Поток для логирования
    uint8_t doutPin;
    uint8_t sclkPin;

    std::string prefsName;
    
    // Сохранение/загрузка калибровки
    void saveCalibrationData();
    bool readCalibrationData();
    
public:
    /**
     * @brief Конструктор класса LoadCellHX711
     * 
     * @param scale Ссылка на объект HX711
     * @param eepromAddr Адрес в EEPROM для хранения калибровки
     * @param calibFlagAddr Адрес в EEPROM для флага калибровки
     */
    LoadCellHX711(const char* prefs_Name, uint8_t dout_pin, uint8_t sclk_pin);
    
    // Реализация интерфейса ILoadCell
    bool begin() override;
    float getForce() override;
    void calibrate() override;
    bool isCalibrationValid() override;
    
    // Дополнительные методы
    void setLogLevel(int8_t level) { log_level = level; }
    LoadCellCalibData& getCalibrationData() { return calibData; }
    void resetCalibration();  // Установка нуля

    virtual void setLogStream(Stream* stream=&Serial) override;
    void setPins(uint8_t dout_pin, uint8_t sclk_pin) { doutPin = dout_pin; sclkPin = sclk_pin; }
};

#endif 