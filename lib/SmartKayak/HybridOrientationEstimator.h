#ifndef HYBRID_ORIENTATION_ESTIMATOR_H
#define HYBRID_ORIENTATION_ESTIMATOR_H

#include "SP_Quaternion.h"
#include "InterfaceIMU.h"
#include <Arduino.h>

// Структура для хранения истории гребков
struct StrokePhase {
    enum Phase {
        CATCH,           // Захват воды (весло входит)
        PULL,            // Проводка (основная фаза)
        RELEASE,         // Выход весла из воды
        RECOVERY,        // Возврат весла
        UNKNOWN
    };
    
    Phase phase;
    float shaftRotationAngle;  // Угол поворота шафта
    float shaftTiltAngle;      // Угол наклона
    float bladeRotationAngle;  // Поворот лопасти
    unsigned long timestamp;
};

// Параметры для детекции магнитных помех
struct MagneticAnomalyDetector {
    float varianceThreshold;      // Порог вариации для детекции помех
    float magnitudeMin;           // Минимальная приемлемая магнитуда
    float magnitudeMax;           // Максимальная приемлемая магнитуда
    int windowSize;               // Размер окна для анализа
    float smoothingFactor;        // Коэффициент сглаживания
    
    MagneticAnomalyDetector() :
        varianceThreshold(50.0f),  // μT²
        magnitudeMin(20.0f),       // μT (примерно половина земного поля)
        magnitudeMax(80.0f),       // μT (примерно удвоенное земное поле)
        windowSize(10),
        smoothingFactor(0.9f) {}
};

// Основной класс гибридной оценки ориентации
class HybridOrientationEstimator {
private:
    // Буферы для хранения истории
    static const int HISTORY_SIZE = 20;
    float magHistory[HISTORY_SIZE][3];  // История магнитометра
    int historyIndex;
    int historyCount;
    
    // История гребков для паттерн-матчинга
    static const int STROKE_HISTORY_SIZE = 10;
    StrokePhase strokeHistory[STROKE_HISTORY_SIZE];
    int strokeHistoryIndex;
    int strokeHistoryCount;
    
    // Детектор магнитных аномалий
    MagneticAnomalyDetector anomalyDetector;
    
    // Состояние системы
    float magReliability;           // Текущая надежность магнитометра [0..1]
    float expectedMagMagnitude;     // Ожидаемая магнитуда поля
    bool magCalibrated;
    
    // Фильтр для DMP кватерниона (гиро+акселерометр)
    SP_Math::Quaternion filteredDMPQuat;
    
    // Комплементарный кватернион с учетом магнитометра
    SP_Math::Quaternion hybridQuat;
    
    // Относительная ориентация весла к каяку
    SP_Math::Quaternion relativeQuat;
    SP_Math::Quaternion kayakQuat;
    
    // Коррекция дрифта на основе паттернов гребли
    SP_Math::Quaternion driftCorrection;
    
    // Временные метки
    unsigned long lastUpdateTime;
    
    // Параметры фильтра
    float gyroWeight;               // Вес гироскопа в фильтре
    float accelWeight;              // Вес акселерометра
    float magWeight;                // Вес магнитометра (адаптивный)
    float patternWeight;            // Вес коррекции по паттерну гребли
    
    // Детекция фазы гребка
    StrokePhase::Phase currentPhase;
    float phaseConfidence;
    
public:
    HybridOrientationEstimator();
    
    // Основной метод обновления ориентации
    SP_Math::Quaternion updateRelativeOrientation(
        const IMUData& paddleIMU,           // Данные IMU весла
        const OrientationData& paddleDMP,   // DMP кватернион весла
        const IMUData& kayakIMU,            // Данные IMU каяка  
        const OrientationData& kayakDMP     // DMP кватернион каяка
    );
    
    // Детекция магнитных помех
    float detectMagneticAnomaly(const IMUData& imuData);
    
    // Калибровка магнитометра "на лету"
    void calibrateMagnetometer(const IMUData& imuData);
    
    // Определение фазы гребка
    StrokePhase::Phase detectStrokePhase(
        float shaftRotationAngle,
        float shaftTiltAngle,
        float bladeForce
    );
    
    // Коррекция дрифта на основе паттернов
    SP_Math::Quaternion correctDriftByPattern(
        const SP_Math::Quaternion& currentQuat,
        StrokePhase::Phase phase
    );
    
    // Получение надежности магнитометра
    float getMagReliability() const { return magReliability; }
    
    // Получение текущей фазы гребка
    StrokePhase::Phase getCurrentPhase() const { return currentPhase; }
    float getPhaseConfidence() const { return phaseConfidence; }
    
    // Настройка параметров
    void setAnomalyDetector(const MagneticAnomalyDetector& detector) {
        anomalyDetector = detector;
    }
    
    void setFilterWeights(float gyro, float accel, float mag, float pattern) {
        gyroWeight = gyro;
        accelWeight = accel;
        magWeight = mag;
        patternWeight = pattern;
    }
    
    // Сброс истории для перекалибровки
    void resetHistory();
    
    // Получение относительной ориентации с адаптивной фузией
    SP_Math::Quaternion getRelativeOrientation() const { return relativeQuat; }
    
    // Диагностика
    void printDiagnostics(Stream* stream = &Serial);

private:
    // Внутренние методы
    void addToMagHistory(const IMUData& imuData);
    float calculateMagVariance();
    float calculateMagMagnitude(const IMUData& imuData);
    
    // Комплементарная фильтрация с адаптивными весами
    SP_Math::Quaternion adaptiveFusion(
        const SP_Math::Quaternion& dmpQuat,
        const IMUData& imuData,
        float magReliability
    );
    
    // Извлечение yaw из магнитометра
    float extractMagneticYaw(const IMUData& imuData);
    
    // Коррекция yaw дрифта
    SP_Math::Quaternion correctYawDrift(
        const SP_Math::Quaternion& quat,
        float magneticYaw,
        float reliability
    );
    
    // Матчинг паттернов гребли
    float matchStrokePattern(
        float currentAngle,
        StrokePhase::Phase phase
    );
    
    // Добавление в историю гребков
    void addToStrokeHistory(const StrokePhase& phase);
    
    // Вычисление относительной ориентации с учетом горизонта каяка
    SP_Math::Quaternion calculateRelativeOrientation(
        const SP_Math::Quaternion& paddleQuat,
        const SP_Math::Quaternion& kayakQuat
    );
};

#endif // HYBRID_ORIENTATION_ESTIMATOR_H
