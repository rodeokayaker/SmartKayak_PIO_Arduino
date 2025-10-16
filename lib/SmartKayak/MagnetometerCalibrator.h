#ifndef MAGNETOMETER_CALIBRATOR_H
#define MAGNETOMETER_CALIBRATOR_H

#include <Arduino.h>
#include "InterfaceIMU.h"

/**
 * Класс для автоматической калибровки магнитометра
 * Компенсирует Hard Iron и Soft Iron искажения
 * Использует алгоритм эллипсоидной подгонки
 */
class MagnetometerCalibrator {
public:
    struct CalibrationResult {
        float offset[3];           // Hard iron offset (смещение)
        float scale[3];            // Soft iron scale (масштаб по осям)
        float rotation[3];         // Soft iron rotation (углы поворота)
        float quality;             // Качество калибровки [0..1]
        bool valid;                // Валидность калибровки
        int sampleCount;           // Количество использованных образцов
    };
    
    struct CalibrationSettings {
        int minSamples;            // Минимум образцов для калибровки
        int maxSamples;            // Максимум образцов (буфер)
        float expectedMagnitude;   // Ожидаемая магнитуда поля Земли (μT)
        float qualityThreshold;    // Порог качества для принятия калибровки
        bool autoCalibrate;        // Автоматическая калибровка в фоне
        
        CalibrationSettings() :
            minSamples(200),
            maxSamples(500),
            expectedMagnitude(50.0f),  // ~50 μT для средних широт
            qualityThreshold(0.8f),
            autoCalibrate(true) {}
    };

private:
    CalibrationSettings settings;
    CalibrationResult result;
    
    // Буфер для сбора образцов
    static const int MAX_SAMPLES = 500;
    float samples[MAX_SAMPLES][3];
    int sampleIndex;
    int sampleCount;
    
    bool isCalibrating;
    unsigned long calibrationStartTime;
    
    // Внутренние переменные для алгоритма
    float minVals[3];
    float maxVals[3];
    
public:
    MagnetometerCalibrator();
    
    // Запуск/остановка калибровки
    void startCalibration();
    void stopCalibration();
    bool isCalibrationActive() const { return isCalibrating; }
    
    // Добавление образца
    void addSample(const IMUData& imuData);
    void addSample(float mx, float my, float mz);
    
    // Вычисление калибровки
    CalibrationResult computeCalibration();
    
    // Применение калибровки к данным
    void applyCalibration(float& mx, float& my, float& mz) const;
    void applyCalibration(IMUData& imuData) const;
    
    // Получение результатов
    CalibrationResult getCalibrationResult() const { return result; }
    bool isCalibrationValid() const { return result.valid; }
    
    // Настройки
    void setSettings(const CalibrationSettings& newSettings) { settings = newSettings; }
    CalibrationSettings getSettings() const { return settings; }
    
    // Диагностика
    void printCalibrationResult(Stream* stream = &Serial) const;
    float getCalibrationProgress() const;
    
    // Сохранение/загрузка калибровки (в EEPROM или файл)
    void saveCalibration();
    void loadCalibration();
    
private:
    // Алгоритм эллипсоидной подгонки
    void ellipsoidFit();
    
    // Простая калибровка min/max (для начала)
    void simpleMinMaxCalibration();
    
    // Оценка качества калибровки
    float evaluateCalibrationQuality();
    
    // Проверка покрытия пространства образцами
    bool checkSampleCoverage();
    
    // Вспомогательные математические функции
    void computeCentroid(float centroid[3]);
    void computeCovariance(float cov[3][3]);
};

/**
 * Умный детектор магнитных помех в реальном времени
 * Анализирует данные и определяет источники помех
 */
class MagneticInterferenceDetector {
public:
    enum InterferenceType {
        NONE,               // Нет помех
        HARD_IRON,          // Постоянные магниты (смещение)
        SOFT_IRON,          // Ферромагнитные материалы (искажение)
        DYNAMIC,            // Динамические помехи (моторы, провода)
        SEVERE              // Критические помехи
    };
    
    struct InterferenceReport {
        InterferenceType type;
        float severity;         // [0..1]
        float reliability;      // Надежность детекции [0..1]
        bool actionRequired;    // Требуется действие
        String description;     // Описание проблемы
    };
    
private:
    static const int WINDOW_SIZE = 30;
    float magHistory[WINDOW_SIZE][3];
    int historyIndex;
    int historyCount;
    
    float baselineMagnitude;
    bool baselineEstablished;
    
public:
    MagneticInterferenceDetector();
    
    // Анализ текущих данных
    InterferenceReport analyze(const IMUData& imuData);
    
    // Установка базовой линии (чистые условия)
    void setBaseline(const IMUData& imuData);
    void resetBaseline();
    
    // Диагностика
    void printReport(const InterferenceReport& report, Stream* stream = &Serial);

private:
    // Детекция типов помех
    bool detectHardIron(float deviation);
    bool detectSoftIron(float stdDev[3]);
    bool detectDynamicInterference(float deltaRate);
    
    // Вычисления
    void calculateStatistics(float mean[3], float stdDev[3], float& magnitude);
    float calculateMagnitude(float mx, float my, float mz);
};

#endif // MAGNETOMETER_CALIBRATOR_H
