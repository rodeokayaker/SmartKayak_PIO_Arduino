#include "MagCorrectionFilter.h"
#include <cmath>
#include <Arduino.h>
namespace SP_Filters {




DMP_MagFusion::DMP_MagFusion(const DMP_MagFusion::Config& config) 
    : config(config), 
      last_quat(1.0f, 0.0f, 0.0f, 0.0f),
      mag_buffer(config.buffer_size) {
}

SP_Math::Quaternion DMP_MagFusion::update(
    const SP_Math::Quaternion& dmp_quat,
    const SP_Math::Vector& mag,
    const SP_Math::Vector& gyro
) {
     // Нормализуем входные данные
    SP_Math::Vector mag_norm = mag;

//    Serial.printf("Accel,%f,%f,%f\n",mag.x(),mag.y(),mag.z());
     
    // Анализируем магнитное поле
 //   MagAnalysis analysis = analyzeMagField(mag_norm, dmp_quat);
    
    //if (analysis.is_reliable) {
        // При достоверных данных вычисляем новую коррекцию курса
        SP_Math::Vector mag_local = dmp_quat.rotate(mag_norm);
        

  //      Serial.printf("Mag,%f,%f,%f\n",mag_local.x(),mag_local.y(),mag_local.z());
        mag_local.z() = 0.0f;  // Проецируем на горизонтальную плоскость
        mag_local = mag_local.normalize();

        // Вычисляем новую коррекцию курса
        float yaw_correction = std::atan2(mag_local.y(), mag_local.x()) +  
                              config.magnetic_declination;

//        Serial.printf("Yaw correction: %f\n", yaw_correction);

        SP_Math::Quaternion new_correction(
            SP_Math::Vector(0.0f, 0.0f, 1.0f),  // поворот вокруг оси Z
            yaw_correction
        );

  //      Serial.printf("New correction: %f, %f, %f, %f\n", new_correction[0], new_correction[1], new_correction[2], new_correction[3]);

        if (has_valid_correction) {
            // Плавно переходим к новой коррекции
            float blend_factor = 0.05f; // Коэффициент плавности (меньше - плавнее)
            last_valid_correction = last_valid_correction.slerp(new_correction, blend_factor);
            last_valid_correction = new_correction;
        } else {
            // Первая коррекция - применяем сразу
            last_valid_correction = new_correction;
            has_valid_correction = true;
        }
    //}

    // Всегда применяем последнюю достоверную коррекцию, если она есть
    if (has_valid_correction) {
//        Serial.printf("Applying last valid correction: %f, %f, %f, %f\n", last_valid_correction[0], last_valid_correction[1], last_valid_correction[2], last_valid_correction[3]);
        return dmp_quat * last_valid_correction;
    }

//    Serial.println("Applying no correction");

    return dmp_quat;
}

DMP_MagFusion::MagAnalysis DMP_MagFusion::analyzeMagField(
    const SP_Math::Vector& mag,
    const SP_Math::Quaternion& dmp_quat
) {
    MagAnalysis analysis;
    
    // Обновляем буфер с защитой от переполнения
    static size_t buffer_idx = 0;
    mag_buffer[buffer_idx] = mag;
    buffer_idx = (buffer_idx + 1) % config.buffer_size;

    // Вычисляем вариацию магнитного поля
    SP_Math::Vector mean_mag = calculateMeanVector();
    analysis.variance = calculateVariance(mean_mag);
    current_variance = analysis.variance;

    // Проверяем величину поля
    analysis.field_strength = mag.length();
    bool field_strength_ok = (analysis.field_strength >= config.min_field_strength && 
                            analysis.field_strength <= config.max_field_strength);

    // Проверяем угол наклона магнитного поля
    SP_Math::Vector mag_local = dmp_quat.conjugate().rotate(mag);
    analysis.dip_error = std::abs(std::asin(mag_local.y()));

    // Вычисляем коэффициент доверия
    analysis.trust_factor = calculateTrustFactor(analysis);

    // Определяем надежность данных
    analysis.is_reliable = 
        field_strength_ok &&
        analysis.variance < config.variance_threshold &&
        analysis.dip_error < config.dip_angle_threshold &&
        analysis.trust_factor > 0.3f;

    return analysis;
}

float DMP_MagFusion::calculateTrustFactor(const MagAnalysis& analysis) {
    // Вычисляем отдельные факторы доверия
    float variance_trust = 1.0f - std::min(1.0f, analysis.variance / config.variance_threshold);
    float dip_trust = 1.0f - std::min(1.0f, analysis.dip_error / config.dip_angle_threshold);
    float field_trust = 1.0f - std::min(1.0f, 
        std::abs(analysis.field_strength - 1.0f) / (config.max_field_strength - 1.0f));

    // Комбинируем факторы с весами
    return variance_trust * 0.3f + dip_trust * 0.4f + field_trust * 0.3f;
}

float DMP_MagFusion::calculateAdaptiveAlpha(
    const MagAnalysis& analysis,
    const SP_Math::Vector& gyro
) {
    float alpha = config.alpha;

    // Увеличиваем доверие к DMP при быстром движении
    float gyro_magnitude = gyro.length();
    if (gyro_magnitude > config.gyro_threshold) {
        float gyro_factor = std::min(1.0f, (gyro_magnitude - config.gyro_threshold) * 0.1f);
        alpha = std::min(0.99f, alpha + gyro_factor);
    }

    // Уменьшаем доверие к магнитометру при высокой вариации
    float variance_factor = analysis.variance / config.variance_threshold;
    alpha = std::min(0.99f, alpha + variance_factor * 0.1f);

    return alpha;
}

SP_Math::Quaternion DMP_MagFusion::calculateYawCorrection(
    const SP_Math::Quaternion& dmp_quat,
    const SP_Math::Vector& mag,
    float trust_factor
) {
    // Получаем локальное направление магнитного поля
    SP_Math::Vector mag_local = dmp_quat.conjugate().rotate(mag);
    
    // Проецируем на горизонтальную плоскость
    mag_local.y() = 0.0f;
    mag_local = mag_local.normalize();

    // Вычисляем угол коррекции с учетом магнитного склонения
    float yaw_correction = std::atan2(mag_local.y(), mag_local.x()) + 
                          config.magnetic_declination;

    // Создаем кватернион коррекции только по оси Z
    SP_Math::Quaternion correction(
        SP_Math::Vector(0.0f, 0.0f, 1.0f),
        yaw_correction * trust_factor
    );

    return dmp_quat * correction;
}

void DMP_MagFusion::setConfig(const Config& new_config) {
    config = new_config;
    mag_buffer.resize(config.buffer_size);
}

SP_Math::Vector DMP_MagFusion::calculateMeanVector() {
    SP_Math::Vector mean(0.0f, 0.0f, 0.0f);
    int valid_samples = 0;

    // Суммируем все векторы в буфере
    for (const auto& mag : mag_buffer) {
        if (mag.length() > 0.1f) {  // Проверка на валидность вектора
            mean += mag;
            valid_samples++;
        }
    }

    // Защита от деления на ноль
    if (valid_samples > 0) {
        mean /= static_cast<float>(valid_samples);
    }

    return mean;
}

float DMP_MagFusion::calculateVariance(const SP_Math::Vector& mean) {
    float variance = 0.0f;
    int valid_samples = 0;

    // Вычисляем сумму квадратов отклонений от среднего
    for (const auto& mag : mag_buffer) {
        if (mag.length() > 0.1f) {  // Проверка на валидность вектора
            SP_Math::Vector diff = mag - mean;
            variance += diff.lengthSquared();  // Используем квадрат длины разности векторов
            valid_samples++;
        }
    }

    // Защита от деления на ноль
    if (valid_samples > 1) {  // Нужно минимум 2 сэмпла для вариации
        variance /= static_cast<float>(valid_samples - 1);  // Несмещенная оценка
    }

    return variance;
}

} // namespace SP_Filters
