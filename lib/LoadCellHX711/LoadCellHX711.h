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

#define HX711_DEFAULT_FREQUENCY 10
#define HX711_DEFAULT_CALIBRATION_FACTOR 1000.0f


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
    uint16_t frequency;
    int8_t log_level;            // Уровень логирования
    Stream* logStream;            // Поток для логирования
    uint8_t doutPin;
    uint8_t sclkPin;
    int32_t lastReadData;
    uint32_t lastReadTime;
    std::string prefsName;
    bool readyToRead;
    
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
    bool begin(uint16_t freq=HX711_DEFAULT_FREQUENCY);
    
    // Реализация интерфейса ILoadCell
    bool read() override;
    float getForce() override;
    int32_t getRawForce() override;
    void calibrate() override;
    bool isCalibrationValid() override;
    uint16_t getFrequency() override;
    void calibrateScale(float weight) override;
    
    // Дополнительные методы
    void setLogLevel(int8_t level) { log_level = level; }
    LoadCellCalibData& getCalibrationData() { return calibData; }
    void resetCalibration();  // Установка нуля

    virtual void setLogStream(Stream* stream=&Serial) override;
    void setPins(uint8_t dout_pin, uint8_t sclk_pin) { doutPin = dout_pin; sclkPin = sclk_pin; }
    void tare() override;
    void reset(uint32_t delay_ms=10) override;
    bool isDataReady() override { return readyToRead&&scale.is_ready(); }
    uint8_t getDRDYPin() override { return doutPin; }
    
};

#endif 