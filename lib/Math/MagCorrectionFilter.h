#pragma once
#include "SP_Math.h"
#include <vector>

namespace SP_Filters {

class DMP_MagFusion {
public:
    struct Config {
        float alpha = 0.98f;              // Базовый коэффициент доверия DMP
        float mag_trust_base = 0.02f;     // Базовый коэффициент доверия магнитометру
        float magnetic_declination = 0.0f; // Магнитное склонение
        int buffer_size = 50;             // Размер буфера анализа
        float variance_threshold = 0.1f;   // Порог вариации магнитного поля
        float dip_angle_threshold = 0.2f;  // Допустимое отклонение угла наклона
        float min_field_strength = 0.7f;   // Минимальная допустимая величина поля
        float max_field_strength = 1.3f;   // Максимальная допустимая величина поля
        float gyro_threshold = 1.0f;       // Порог "быстрого движения" для гироскопа
    };

    DMP_MagFusion(const Config& config);

    SP_Math::Quaternion update(
        const SP_Math::Quaternion& dmp_quat,
        const SP_Math::Vector& mag,
        const SP_Math::Vector& gyro
    );

    void setConfig(const Config& config);
    float getMagTrust() const { return current_mag_trust; }
    float getMagVariance() const { return current_variance; }

private:
    Config config;
    SP_Math::Quaternion last_quat;
    SP_Math::Quaternion last_valid_correction;  // Сохраняем последнюю достоверную коррекцию
    bool has_valid_correction = false;          // Флаг наличия достоверной коррекции
    std::vector<SP_Math::Vector> mag_buffer;
    float current_mag_trust = 1.0f;
    float current_variance = 0.0f;

    struct MagAnalysis {
        float variance;      // Вариация величины поля
        float dip_error;     // Ошибка угла наклона
        float field_strength;// Величина магнитного поля
        bool is_reliable;    // Общая надежность
        float trust_factor;  // Коэффициент доверия [0-1]
    };

    MagAnalysis analyzeMagField(
        const SP_Math::Vector& mag,
        const SP_Math::Quaternion& dmp_quat
    );

    float calculateAdaptiveAlpha(
        const MagAnalysis& analysis,
        const SP_Math::Vector& gyro
    );

    SP_Math::Quaternion calculateYawCorrection(
        const SP_Math::Quaternion& dmp_quat,
        const SP_Math::Vector& mag,
        float trust_factor
    );

    SP_Math::Vector calculateMeanVector();
    float calculateVariance(const SP_Math::Vector& mean_mag);
    float calculateTrustFactor(const MagAnalysis& analysis);
};



} // namespace SP_Filters
