/**
 * @file IMUSensor_BNO085.h
 * @brief Библиотека для работы с IMU модулем BNO085
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#ifndef IMUSensor_BNO085_h
#define IMUSensor_BNO085_h

#include "InterfaceIMU.h"
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Preferences.h>

#define BNO085_IMU_DEFAULT_FREQUENCY 100
#define BNO085_MAG_DEFAULT_FREQUENCY 100
#define BNO085_CALIBRATION_SAVE_INTERVAL 30000 // 30 секунд
#define BNO085_CALIBRATION_CHECK_INTERVAL 5000 // 5 секунд

// Структура для хранения калибровочных данных BNO085
struct BNO085CalibrationData {
    float quatAccuracy;         // Точность кватерниона в радианах
    uint8_t qualityLevel;       // Уровень качества (0-3)
    uint32_t timestamp;         // Время сохранения
    bool isValid;               // Флаг валидности данных
    uint32_t measurementCount;  // Количество измерений при этой точности
};

class IMUSensor_BNO085 : public IIMU {
private:
    BNO08x myIMU;                           // BNO085 sensor object
    
    IMUData currentData;                    // Текущие данные с датчиков
    OrientationData currentOrientation;     // Текущая ориентация
    IMUCalibData calibData;                 // Калибровочные данные (для совместимости)
    Stream* logStream;                      // Поток для логирования
    std::string prefsName;                  // Имя раздела в Preferences
    uint16_t imuFrequency;                  // Частота обновления IMU
    uint16_t magFrequency;                  // Частота магнитометра
    int8_t interruptPin;                    // Пин прерывания
    
    bool sensorReady;                       // Готовность сенсора
    uint8_t i2cAddress;                     // I2C адрес
    bool calibrationSaved;                  // Флаг сохранения калибровки
    uint32_t lastSavedCalibrationTime;      // Время последнего сохранения
    uint32_t lastCalibrationCheck;          // Время последней проверки калибровки
    
    // Калибровочные данные BNO085
    BNO085CalibrationData savedCalibration; // Сохраненная калибровка
    BNO085CalibrationData currentCalibration; // Текущая калибровка
    uint8_t calibrationCounter;             // Счетчик стабильных измерений
    
    // Счетчики данных
    uint32_t quaternionCount;
    uint32_t accelCount;
    uint32_t gyroCount; 
    uint32_t magCount;
    

    
    // Внутренние методы
    void checkAndSaveCalibration();
    bool readCalibrationData();
    void saveCalibrationData(); 
    void setReports();
    uint8_t evaluateCalibrationQuality(float accuracy);
    static void IRAM_ATTR dataReadyISR();
    
public:
    IMUSensor_BNO085(const char* prefsName, uint8_t i2cAddr = 0x4A, 
                     int8_t interruptPin = -1, Stream* logStream = &Serial);
    
    // Реализация интерфейса IIMU
    bool begin(uint16_t imuFreq, uint16_t magFreq);
    virtual void begin() override { begin(BNO085_IMU_DEFAULT_FREQUENCY, BNO085_MAG_DEFAULT_FREQUENCY); }
    void calibrate() override;
    void calibrateCompass() override;
    bool readData() override;
    IMUData getData() override;
    OrientationData updateOrientation() override;
    OrientationData getOrientation() override;
    bool isCalibrationValid() override;
    uint16_t magnetometerFrequency() override { return magFrequency; }
    bool DMPEnabled() override { return true; }  // BNO085 всегда имеет внутренний процессор
    int8_t interruptPIN() override { return interruptPin; }
    void magnetometerUpdate() override {};
    uint16_t getFrequency() override { return imuFrequency; }
    void setFrequency(uint16_t frequency) override {imuFrequency = frequency;};
    void setLogStream(Stream* stream = &Serial) override { logStream = stream; }
    void setCalibrationData(const IMUCalibData data, bool save = false) override;
    IMUCalibData getCalibrationData() override;
    bool DMPValid() override { return sensorReady; }
    int8_t getIntStatus() override { return 0; }
    
    // Дополнительные методы для BNO085
    void getCalibrationStatus(uint8_t* quality, float* accuracy);
    bool isFullyCalibrated();
    void printCalibrationStatus();
    float getQuaternionAccuracy();
    uint8_t getCalibrationQuality();
    
    // Методы управления калибровкой
    void resetCalibration();
    bool hasImprovedCalibration();
    void forceSaveCalibration();

    
    // Статистика
    void printStatistics();
    uint32_t getQuaternionCount() { return quaternionCount; }
    uint32_t getAccelCount() { return accelCount; }
    uint32_t getGyroCount() { return gyroCount; }
    uint32_t getMagCount() { return magCount; }

    void softReset();
    bool isReady();

    void setInterruptPin(int interruptPin) { this->interruptPin = interruptPin;};
    
    virtual ~IMUSensor_BNO085() {};
};

#endif 