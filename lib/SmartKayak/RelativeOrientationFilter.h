#ifndef RELATIVE_ORIENTATION_FILTER_H
#define RELATIVE_ORIENTATION_FILTER_H

#include "SP_Quaternion.h"
#include "Arduino.h"
#include <vector>

// Состояние фазы гребка
enum class StrokePhase {
    CATCH,          // Захват воды (весло входит в воду)
    PULL,           // Тяга (основная фаза)
    RELEASE,        // Выход (весло выходит из воды)
    RECOVERY,       // Проводка (весло в воздухе)
    UNKNOWN
};

// Статистика для детекции качества данных магнитометра
struct MagnetometerStats {
    float magnitude;           // Величина магнитного поля
    float changeRate;          // Скорость изменения
    float consistency;         // Консистентность между измерениями
    bool isReliable;          // Флаг надежности
    unsigned long lastUpdate;
};

// Параметры фильтра
struct FilterParameters {
    float gyroTrust = 0.98f;           // Доверие гироскопу (высокое для DMP)
    float magTrustMax = 0.02f;         // Максимальное доверие магнитометру
    float magTrustMin = 0.0f;          // Минимальное (при помехах)
    
    // Параметры для детекции магнитных помех
    float nominalMagMagnitude = 50.0f; // μT для вашего региона
    float magTolerancePercent = 30.0f; // ±30% от номинала
    float maxMagChangeRate = 100.0f;   // μT/s максимальная скорость изменения
    
    // Параметры циклической коррекции
    bool useStrokePhaseCorrection = true;
    float strokePhaseWeight = 0.1f;    // Вес коррекции по фазе гребка
};

class RelativeOrientationFilter {
private:
    FilterParameters params;
    
    // Состояние фильтра
    SP_Math::Quaternion filteredRelativeQuat;
    SP_Math::Quaternion lastKayakQuat;
    SP_Math::Quaternion lastPaddleQuat;
    
    // Статистика магнитометров
    MagnetometerStats kayakMagStats;
    MagnetometerStats paddleMagStats;
    
    // История для детекции паттернов
    struct MagSample {
        float x, y, z;
        unsigned long timestamp;
    };
    std::vector<MagSample> kayakMagHistory;
    std::vector<MagSample> paddleMagHistory;
    static const size_t MAX_HISTORY = 20;
    
    // Фазы гребка
    StrokePhase currentPhase;
    StrokePhase lastPhase;
    
    // Ожидаемые относительные положения для каждой фазы (для коррекции)
    SP_Math::Quaternion expectedCatchQuat;
    SP_Math::Quaternion expectedReleaseQuat;
    
    // Дрифт yaw (накопленная ошибка по рысканию)
    float yawDriftCorrection;
    
public:
    RelativeOrientationFilter();
    
    /**
     * Основной метод обновления фильтра
     * @param kayakQuat - кватернион каяка из DMP
     * @param paddleQuat - кватернион весла из DMP
     * @param kayakMag - данные магнитометра каяка (x,y,z)
     * @param paddleMag - данные магнитометра весла (x,y,z)
     * @param shaftAngle - текущий угол поворота весла (для определения фазы)
     * @param bladeForce - сила на лопасти (для определения фазы)
     * @return отфильтрованный относительный кватернион
     */
    SP_Math::Quaternion update(
        const SP_Math::Quaternion& kayakQuat,
        const SP_Math::Quaternion& paddleQuat,
        const SP_Math::Vector& kayakMag,
        const SP_Math::Vector& paddleMag,
        float shaftAngle,
        float bladeForce
    );
    
    /**
     * Определение фазы гребка
     */
    StrokePhase detectStrokePhase(float shaftAngle, float shaftTilt, float bladeForce);
    
    /**
     * Оценка надежности данных магнитометра
     */
    float evaluateMagnetometerTrust(MagnetometerStats& stats, const SP_Math::Vector& magData);
    
    /**
     * Обновление статистики магнитометра
     */
    void updateMagStats(MagnetometerStats& stats, const SP_Math::Vector& magData, 
                       std::vector<MagSample>& history);
    
    /**
     * Вычисление базовой относительной ориентации (как в оригинальном коде)
     */
    SP_Math::Quaternion computeBasicRelativeOrientation(
        const SP_Math::Quaternion& kayakQuat,
        const SP_Math::Quaternion& paddleQuat
    );
    
    /**
     * Коррекция yaw на основе магнитометра
     */
    SP_Math::Quaternion correctYawWithMagnetometer(
        const SP_Math::Quaternion& relativeQuat,
        const SP_Math::Vector& kayakMag,
        const SP_Math::Vector& paddleMag,
        float trust
    );
    
    /**
     * Коррекция на основе известной фазы гребка
     */
    SP_Math::Quaternion correctWithStrokePhase(
        const SP_Math::Quaternion& relativeQuat,
        StrokePhase phase
    );
    
    /**
     * Калибровка ожидаемых положений для фаз гребка
     */
    void calibrateStrokePhaseQuaternions(
        StrokePhase phase,
        const SP_Math::Quaternion& observedQuat
    );
    
    // Геттеры
    StrokePhase getCurrentPhase() const { return currentPhase; }
    float getYawDriftCorrection() const { return yawDriftCorrection; }
    bool isKayakMagReliable() const { return kayakMagStats.isReliable; }
    bool isPaddleMagReliable() const { return paddleMagStats.isReliable; }
    SP_Math::Quaternion getFilteredQuaternion() const { return filteredRelativeQuat; }
    MagnetometerStats getKayakMagStats() const { return kayakMagStats; }
    MagnetometerStats getPaddleMagStats() const { return paddleMagStats; }
    
    // Настройка параметров
    void setParameters(const FilterParameters& p) { params = p; }
    FilterParameters& getParameters() { return params; }
};

#endif // RELATIVE_ORIENTATION_FILTER_H
