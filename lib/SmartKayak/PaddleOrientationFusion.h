#ifndef PADDLE_ORIENTATION_FUSION_H
#define PADDLE_ORIENTATION_FUSION_H

#include "../Math/SP_Quaternion.h"
#include "SmartPaddle.h"
#include <Arduino.h>

/**
 * Система адаптивного определения относительной ориентации весла
 * 
 * Использует:
 * 1. DMP кватернионы (гироскоп + акселерометр) как основной источник
 * 2. Магнитометр для коррекции дрифта (только когда данные надежны)
 * 3. Циклические паттерны гребли для валидации и предсказания
 * 4. Адаптивный Калман-подобный фильтр для подавления помех
 */

// Структура для хранения паттерна гребка
struct StrokePattern {
    float shaftRotationRange[2];    // Диапазон поворота шафта [min, max]
    float shaftTiltRange[2];         // Диапазон наклона [min, max]
    float bladeRotationRange[2];     // Диапазон поворота лопасти [min, max]
    unsigned long avgDuration;       // Средняя длительность гребка (мс)
    bool isValid;                    // Валидность паттерна
    
    StrokePattern() : avgDuration(0), isValid(false) {
        shaftRotationRange[0] = shaftRotationRange[1] = 0;
        shaftTiltRange[0] = shaftTiltRange[1] = 0;
        bladeRotationRange[0] = bladeRotationRange[1] = 0;
    }
};

// Состояние фазы гребка
enum class StrokePhase {
    RECOVERY,      // Возврат весла
    CATCH,         // Захват воды
    DRIVE,         // Гребок
    RELEASE,       // Выход из воды
    UNKNOWN
};

// Качество данных магнитометра
enum class MagnetometerQuality {
    EXCELLENT,     // Отличное качество, можно использовать для коррекции
    GOOD,          // Хорошее, использовать с осторожностью
    POOR,          // Плохое, игнорировать
    INVALID        // Невалидные данные
};

class PaddleOrientationFusion {
private:
    // DMP кватернионы (основной источник)
    SP_Math::Quaternion kayakQuatDMP;
    SP_Math::Quaternion paddleQuatDMP;
    
    // Скорректированные кватернионы
    SP_Math::Quaternion kayakQuatCorrected;
    SP_Math::Quaternion paddleQuatCorrected;
    
    // Относительная ориентация
    SP_Math::Quaternion relativeOrientation;
    
    // Паттерн гребка
    StrokePattern leftStrokePattern;
    StrokePattern rightStrokePattern;
    StrokePattern* currentPattern;
    
    // Фаза гребка
    StrokePhase currentPhase;
    unsigned long phaseStartTime;
    unsigned long lastStrokeStartTime;
    
    // Параметры магнитометра
    float magVarianceThreshold;      // Порог дисперсии для определения помех
    float magDeviationThreshold;     // Порог отклонения от паттерна
    int magSampleBufferSize;         // Размер буфера для анализа
    
    // Буферы данных магнитометра
    static const int MAG_BUFFER_SIZE = 10;
    struct MagSample {
        int16_t x, y, z;
        unsigned long timestamp;
    };
    MagSample kayakMagBuffer[MAG_BUFFER_SIZE];
    MagSample paddleMagBuffer[MAG_BUFFER_SIZE];
    int magBufferIndex;
    
    // Коррекция дрифта
    float yawDriftRate;              // Скорость дрифта по yaw (град/сек)
    unsigned long lastDriftCorrectionTime;
    
    // Весовые коэффициенты для фильтра
    float dmpWeight;                 // Вес DMP данных (0.95-0.98)
    float magWeight;                 // Вес магнитометра (0.02-0.05)
    float patternWeight;             // Вес паттерна гребли (0.0-0.3)
    
    // Флаги состояния
    bool magnetometerCalibrated;
    bool patternsLearned;
    int strokeCount;                 // Счетчик гребков для обучения
    
    // Методы анализа магнитометра
    MagnetometerQuality assessMagnetometerQuality(const IMUData& kayakIMU, const IMUData& paddleIMU);
    float calculateMagVariance(const MagSample* buffer, int size);
    float calculateMagDeviation(int16_t mx, int16_t my, int16_t mz, const StrokePattern& pattern);
    bool detectMagneticAnomaly(const IMUData& imuData);
    
    // Методы работы с паттернами
    void updateStrokePattern(StrokePattern& pattern, float shaftRotation, float shaftTilt, float bladeRotation);
    StrokePhase detectStrokePhase(float shaftTiltAngle, float bladeForce, const SP_Math::Quaternion& relQuat);
    bool validateAgainstPattern(const SP_Math::Quaternion& relQuat, const StrokePattern& pattern);
    
    // Методы коррекции
    SP_Math::Quaternion applyDriftCorrection(const SP_Math::Quaternion& quat, float deltaTime);
    SP_Math::Quaternion applyMagneticCorrection(const SP_Math::Quaternion& dmpQuat, 
                                                  const IMUData& imuData, 
                                                  MagnetometerQuality quality);
    SP_Math::Quaternion applyPatternCorrection(const SP_Math::Quaternion& quat, 
                                                 const StrokePattern& pattern,
                                                 StrokePhase phase);
    
    // Фильтрация
    SP_Math::Quaternion complementaryFilter(const SP_Math::Quaternion& dmpQuat,
                                             const SP_Math::Quaternion& magCorrected,
                                             const SP_Math::Quaternion& patternPredicted,
                                             MagnetometerQuality magQuality);

public:
    PaddleOrientationFusion();
    
    /**
     * Основной метод обновления относительной ориентации
     * 
     * @param kayakIMU - данные IMU каяка
     * @param paddleIMU - данные IMU весла
     * @param kayakOrientation - кватернион ориентации каяка
     * @param paddleOrientation - кватернион ориентации весла
     * @param bladeForce - текущая сила на лопасти
     * @return относительная ориентация весла относительно каяка
     */
    SP_Math::Quaternion update(const IMUData& kayakIMU,
                                const IMUData& paddleIMU,
                                const OrientationData& kayakOrientation,
                                const OrientationData& paddleOrientation,
                                float bladeForce);
    
    /**
     * Получить текущую относительную ориентацию
     */
    SP_Math::Quaternion getRelativeOrientation() const { return relativeOrientation; }
    
    /**
     * Получить фазу гребка
     */
    StrokePhase getStrokePhase() const { return currentPhase; }
    
    /**
     * Получить качество магнитометра
     */
    MagnetometerQuality getCurrentMagQuality(const IMUData& kayakIMU, const IMUData& paddleIMU) {
        return assessMagnetometerQuality(kayakIMU, paddleIMU);
    }
    
    /**
     * Настройка параметров фильтра
     */
    void setFilterWeights(float dmp, float mag, float pattern) {
        dmpWeight = dmp;
        magWeight = mag;
        patternWeight = pattern;
    }
    
    /**
     * Настройка порогов магнитометра
     */
    void setMagnetometerThresholds(float varianceThreshold, float deviationThreshold) {
        magVarianceThreshold = varianceThreshold;
        magDeviationThreshold = deviationThreshold;
    }
    
    /**
     * Получить статистику обучения
     */
    void getStats(int& strokes, bool& learned, float& driftRate) const {
        strokes = strokeCount;
        learned = patternsLearned;
        driftRate = yawDriftRate;
    }
    
    /**
     * Сброс паттернов для переобучения
     */
    void resetPatterns() {
        leftStrokePattern.isValid = false;
        rightStrokePattern.isValid = false;
        patternsLearned = false;
        strokeCount = 0;
    }
};

#endif // PADDLE_ORIENTATION_FUSION_H
