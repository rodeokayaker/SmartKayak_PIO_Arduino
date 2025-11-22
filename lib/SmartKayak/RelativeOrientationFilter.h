#ifndef RELATIVE_ORIENTATION_FILTER_H
#define RELATIVE_ORIENTATION_FILTER_H

#include "SP_Quaternion.h"
#include "SP_Types.h"
#include "Arduino.h"

// Конфигурация фильтра
#define MAG_VARIANCE_WINDOW 20          // Окно для расчета дисперсии магнитометра
#define MAG_VARIANCE_THRESHOLD 800.0f   // Порог дисперсии для детекции помех
#define MAG_TRUST_FACTOR_MIN 0.0f       // Минимальный уровень доверия к магнитометру
#define MAG_TRUST_FACTOR_MAX 1.0f       // Максимальный уровень доверия
#define STROKE_PATTERN_MEMORY 10        // Количество запоминаемых циклов гребли
#define YAW_DRIFT_CORRECTION_RATE 0.02f // Скорость коррекции дрифта yaw
#define GYRO_INTEGRATION_WEIGHT 0.98f   // Вес интеграции гироскопа в фильтре

// Структура для хранения истории магнитометра
struct MagnetometerHistory {
    float mag_x[MAG_VARIANCE_WINDOW];
    float mag_y[MAG_VARIANCE_WINDOW];
    float mag_z[MAG_VARIANCE_WINDOW];
    uint8_t index;
    bool filled;
    
    MagnetometerHistory() : index(0), filled(false) {
        memset(mag_x, 0, sizeof(mag_x));
        memset(mag_y, 0, sizeof(mag_y));
        memset(mag_z, 0, sizeof(mag_z));
    }
};

// Структура для паттерна гребка (для коррекции ориентации)
struct StrokeOrientationPattern {
    float expected_yaw_change;      // Ожидаемое изменение yaw за гребок
    float shaft_rotation_start;     // Угол поворота шафта в начале гребка
    float shaft_rotation_end;       // Угол поворота шафта в конце гребка
    BladeSideType blade_side;       // Сторона лопасти
    uint32_t duration_ms;           // Длительность гребка
    float confidence;               // Уверенность в паттерне
    bool is_valid;
    
    StrokeOrientationPattern() : 
        expected_yaw_change(0), 
        shaft_rotation_start(0),
        shaft_rotation_end(0),
        blade_side(BladeSideType::ALL_BLADES),
        duration_ms(0),
        confidence(0),
        is_valid(false) {}
};

// Структура для текущего гребка
struct CurrentStrokeState {
    bool in_progress;
    uint32_t start_time;
    float start_yaw;
    float start_shaft_rotation;
    BladeSideType blade_side;
    StrokeOrientationPattern* matched_pattern;
    
    CurrentStrokeState() : 
        in_progress(false),
        start_time(0),
        start_yaw(0),
        start_shaft_rotation(0),
        blade_side(BladeSideType::ALL_BLADES),
        matched_pattern(nullptr) {}
};

/**
 * Фильтр для точного определения относительной ориентации весла
 * 
 * Основные принципы:
 * 1. DMP кватернионы (гироскоп + акселерометр) - основа ориентации
 * 2. Магнитометр - коррекция дрифта yaw ТОЛЬКО при отсутствии помех
 * 3. Паттерны гребли - дополнительная коррекция на основе цикличности
 * 4. Комплементарный фильтр для объединения источников
 */
class RelativeOrientationFilter {
private:
    // История магнитометра для детекции помех
    MagnetometerHistory kayak_mag_history;
    MagnetometerHistory paddle_mag_history;
    
    // Паттерны гребков для коррекции
    StrokeOrientationPattern stroke_patterns[STROKE_PATTERN_MEMORY];
    uint8_t pattern_count;
    
    // Текущее состояние гребка
    CurrentStrokeState current_stroke;
    
    // Фильтрованная ориентация
    SP_Math::Quaternion filtered_kayak_orientation;
    SP_Math::Quaternion filtered_paddle_orientation;
    SP_Math::Quaternion filtered_relative_orientation;
    
    // Уровни доверия
    float kayak_mag_trust;
    float paddle_mag_trust;
    
    // Референсное направление (север) при калибровке
    SP_Math::Vector reference_north_kayak;
    SP_Math::Vector reference_north_paddle;
    bool reference_calibrated;
    
    // Интегрированный yaw из гироскопа
    float integrated_yaw_kayak;
    float integrated_yaw_paddle;
    uint32_t last_update_time;
    
    // Статистика
    uint32_t mag_interference_count;
    uint32_t stroke_correction_count;

public:
    RelativeOrientationFilter();
    
    /**
     * Основной метод обновления фильтра
     * @param dmp_kayak_quat - DMP кватернион каяка (гироскоп + акселерометр)
     * @param dmp_paddle_quat - DMP кватернион весла (гироскоп + акселерометр)
     * @param kayak_mag - данные магнитометра каяка
     * @param paddle_mag - данные магнитометра весла
     * @param kayak_gyro - данные гироскопа каяка
     * @param paddle_gyro - данные гироскопа весла
     * @param dt - время с последнего обновления (секунды)
     * @return отфильтрованная относительная ориентация весла
     */
    SP_Math::Quaternion update(
        const SP_Math::Quaternion& dmp_kayak_quat,
        const SP_Math::Quaternion& dmp_paddle_quat,
        const SP_Math::Vector& kayak_mag,
        const SP_Math::Vector& paddle_mag,
        const SP_Math::Vector& kayak_gyro,
        const SP_Math::Vector& paddle_gyro,
        float dt
    );
    
    /**
     * Обновление состояния гребка для паттерн-коррекции
     */
    void updateStrokeState(
        bool stroke_active,
        BladeSideType blade_side,
        float shaft_rotation_angle,
        float blade_force
    );
    
    /**
     * Калибровка референсного направления (север)
     * Вызывать когда каяк и весло стабильны и нет магнитных помех
     */
    void calibrateReference();
    
    /**
     * Получить текущую относительную ориентацию
     */
    SP_Math::Quaternion getRelativeOrientation() const {
        return filtered_relative_orientation;
    }
    
    /**
     * Получить уровни доверия к магнитометрам
     */
    void getMagnetometerTrust(float& kayak_trust, float& paddle_trust) const {
        kayak_trust = kayak_mag_trust;
        paddle_trust = paddle_mag_trust;
    }
    
    /**
     * Получить статистику
     */
    void getStats(uint32_t& interference_count, uint32_t& correction_count) const {
        interference_count = mag_interference_count;
        correction_count = stroke_correction_count;
    }
    
    /**
     * Сброс статистики
     */
    void resetStats() {
        mag_interference_count = 0;
        stroke_correction_count = 0;
    }

private:
    /**
     * Детектор магнитных помех
     * Анализирует вариацию магнитного поля для определения помех
     */
    float detectMagneticInterference(
        const SP_Math::Vector& mag_reading,
        MagnetometerHistory& history
    );
    
    /**
     * Вычисление дисперсии магнитометра
     */
    float calculateMagVariance(const MagnetometerHistory& history);
    
    /**
     * Коррекция yaw на основе магнитометра
     */
    SP_Math::Quaternion correctYawWithMagnetometer(
        const SP_Math::Quaternion& dmp_quat,
        const SP_Math::Vector& mag_reading,
        const SP_Math::Vector& reference_north,
        float trust_factor
    );
    
    /**
     * Интеграция гироскопа для yaw
     */
    float integrateGyroYaw(float current_yaw, float gyro_z, float dt);
    
    /**
     * Комплементарный фильтр для объединения источников
     */
    SP_Math::Quaternion complementaryFilter(
        const SP_Math::Quaternion& dmp_quat,
        const SP_Math::Quaternion& mag_corrected_quat,
        float mag_trust
    );
    
    /**
     * Поиск подходящего паттерна гребка
     */
    StrokeOrientationPattern* findMatchingStrokePattern(
        BladeSideType blade_side,
        float shaft_rotation_start
    );
    
    /**
     * Добавление нового паттерна гребка
     */
    void addStrokePattern(const StrokeOrientationPattern& pattern);
    
    /**
     * Коррекция на основе паттерна гребка
     */
    SP_Math::Quaternion correctWithStrokePattern(
        const SP_Math::Quaternion& current_quat,
        float stroke_progress
    );
    
    /**
     * Вычисление относительной ориентации с учетом только направления каяка
     */
    SP_Math::Quaternion computeRelativeOrientation(
        const SP_Math::Quaternion& kayak_quat,
        const SP_Math::Quaternion& paddle_quat
    );
    
    /**
     * Извлечение yaw из кватерниона
     */
    float extractYaw(const SP_Math::Quaternion& quat);
    
    /**
     * Создание кватерниона только с yaw вращением
     */
    SP_Math::Quaternion createYawQuaternion(float yaw_angle);
};

#endif // RELATIVE_ORIENTATION_FILTER_H
