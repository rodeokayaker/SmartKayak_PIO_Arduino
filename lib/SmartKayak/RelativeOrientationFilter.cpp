#include "RelativeOrientationFilter.h"
#include <cmath>

RelativeOrientationFilter::RelativeOrientationFilter() :
    pattern_count(0),
    kayak_mag_trust(0.5f),
    paddle_mag_trust(0.5f),
    reference_calibrated(false),
    integrated_yaw_kayak(0),
    integrated_yaw_paddle(0),
    last_update_time(0),
    mag_interference_count(0),
    stroke_correction_count(0)
{
    filtered_kayak_orientation = SP_Math::Quaternion(1, 0, 0, 0);
    filtered_paddle_orientation = SP_Math::Quaternion(1, 0, 0, 0);
    filtered_relative_orientation = SP_Math::Quaternion(1, 0, 0, 0);
    
    // Инициализация паттернов
    for (int i = 0; i < STROKE_PATTERN_MEMORY; i++) {
        stroke_patterns[i].is_valid = false;
    }
}

SP_Math::Quaternion RelativeOrientationFilter::update(
    const SP_Math::Quaternion& dmp_kayak_quat,
    const SP_Math::Quaternion& dmp_paddle_quat,
    const SP_Math::Vector& kayak_mag,
    const SP_Math::Vector& paddle_mag,
    const SP_Math::Vector& kayak_gyro,
    const SP_Math::Vector& paddle_gyro,
    float dt
) {
    // 1. Детекция магнитных помех
    kayak_mag_trust = detectMagneticInterference(kayak_mag, kayak_mag_history);
    paddle_mag_trust = detectMagneticInterference(paddle_mag, paddle_mag_history);
    
    // 2. Интеграция гироскопа для yaw (резервный источник)
    integrated_yaw_kayak = integrateGyroYaw(integrated_yaw_kayak, kayak_gyro.z(), dt);
    integrated_yaw_paddle = integrateGyroYaw(integrated_yaw_paddle, paddle_gyro.z(), dt);
    
    // 3. Коррекция DMP кватернионов магнитометром (если нет помех)
    SP_Math::Quaternion kayak_mag_corrected = dmp_kayak_quat;
    SP_Math::Quaternion paddle_mag_corrected = dmp_paddle_quat;
    
    if (reference_calibrated) {
        if (kayak_mag_trust > 0.3f) {
            kayak_mag_corrected = correctYawWithMagnetometer(
                dmp_kayak_quat, kayak_mag, reference_north_kayak, kayak_mag_trust
            );
        }
        
        if (paddle_mag_trust > 0.3f) {
            paddle_mag_corrected = correctYawWithMagnetometer(
                dmp_paddle_quat, paddle_mag, reference_north_paddle, paddle_mag_trust
            );
        }
    }
    
    // 4. Комплементарный фильтр: объединение DMP и магнитометра
    filtered_kayak_orientation = complementaryFilter(
        dmp_kayak_quat, kayak_mag_corrected, kayak_mag_trust
    );
    
    filtered_paddle_orientation = complementaryFilter(
        dmp_paddle_quat, paddle_mag_corrected, paddle_mag_trust
    );
    
    // 5. Вычисление относительной ориентации
    filtered_relative_orientation = computeRelativeOrientation(
        filtered_kayak_orientation, filtered_paddle_orientation
    );
    
    // 6. Коррекция на основе паттерна гребка (если активен гребок)
    if (current_stroke.in_progress && current_stroke.matched_pattern != nullptr) {
        uint32_t current_time = millis();
        uint32_t elapsed = current_time - current_stroke.start_time;
        float stroke_progress = (float)elapsed / (float)current_stroke.matched_pattern->duration_ms;
        
        if (stroke_progress <= 1.0f) {
            filtered_relative_orientation = correctWithStrokePattern(
                filtered_relative_orientation, stroke_progress
            );
            stroke_correction_count++;
        }
    }
    
    last_update_time = millis();
    
    return filtered_relative_orientation;
}

float RelativeOrientationFilter::detectMagneticInterference(
    const SP_Math::Vector& mag_reading,
    MagnetometerHistory& history
) {
    // Добавляем новое измерение в историю
    history.mag_x[history.index] = mag_reading.x();
    history.mag_y[history.index] = mag_reading.y();
    history.mag_z[history.index] = mag_reading.z();
    
    history.index++;
    if (history.index >= MAG_VARIANCE_WINDOW) {
        history.index = 0;
        history.filled = true;
    }
    
    // Если недостаточно данных, возвращаем среднее доверие
    if (!history.filled) {
        return 0.5f;
    }
    
    // Вычисляем дисперсию
    float variance = calculateMagVariance(history);
    
    // Проверяем аномальные изменения магнитуды
    float current_magnitude = std::sqrt(
        mag_reading.x() * mag_reading.x() + 
        mag_reading.y() * mag_reading.y() + 
        mag_reading.z() * mag_reading.z()
    );
    
    // Вычисляем среднюю магнитуду из истории
    float sum_magnitude = 0;
    for (int i = 0; i < MAG_VARIANCE_WINDOW; i++) {
        sum_magnitude += std::sqrt(
            history.mag_x[i] * history.mag_x[i] + 
            history.mag_y[i] * history.mag_y[i] + 
            history.mag_z[i] * history.mag_z[i]
        );
    }
    float avg_magnitude = sum_magnitude / MAG_VARIANCE_WINDOW;
    
    // Проверка резкого изменения магнитуды (признак помехи)
    float magnitude_change_ratio = std::abs(current_magnitude - avg_magnitude) / avg_magnitude;
    
    // Расчет фактора доверия
    float trust = MAG_TRUST_FACTOR_MAX;
    
    // Уменьшаем доверие при высокой дисперсии
    if (variance > MAG_VARIANCE_THRESHOLD) {
        trust *= (MAG_VARIANCE_THRESHOLD / variance);
        mag_interference_count++;
    }
    
    // Уменьшаем доверие при резком изменении магнитуды (>20%)
    if (magnitude_change_ratio > 0.2f) {
        trust *= (0.2f / magnitude_change_ratio);
        mag_interference_count++;
    }
    
    // Ограничиваем диапазон
    trust = std::max(MAG_TRUST_FACTOR_MIN, std::min(MAG_TRUST_FACTOR_MAX, trust));
    
    return trust;
}

float RelativeOrientationFilter::calculateMagVariance(const MagnetometerHistory& history) {
    // Вычисляем средние значения
    float mean_x = 0, mean_y = 0, mean_z = 0;
    for (int i = 0; i < MAG_VARIANCE_WINDOW; i++) {
        mean_x += history.mag_x[i];
        mean_y += history.mag_y[i];
        mean_z += history.mag_z[i];
    }
    mean_x /= MAG_VARIANCE_WINDOW;
    mean_y /= MAG_VARIANCE_WINDOW;
    mean_z /= MAG_VARIANCE_WINDOW;
    
    // Вычисляем дисперсию
    float var_x = 0, var_y = 0, var_z = 0;
    for (int i = 0; i < MAG_VARIANCE_WINDOW; i++) {
        float dx = history.mag_x[i] - mean_x;
        float dy = history.mag_y[i] - mean_y;
        float dz = history.mag_z[i] - mean_z;
        var_x += dx * dx;
        var_y += dy * dy;
        var_z += dz * dz;
    }
    var_x /= MAG_VARIANCE_WINDOW;
    var_y /= MAG_VARIANCE_WINDOW;
    var_z /= MAG_VARIANCE_WINDOW;
    
    // Общая дисперсия
    return var_x + var_y + var_z;
}

SP_Math::Quaternion RelativeOrientationFilter::correctYawWithMagnetometer(
    const SP_Math::Quaternion& dmp_quat,
    const SP_Math::Vector& mag_reading,
    const SP_Math::Vector& reference_north,
    float trust_factor
) {
    // Преобразуем магнитометр в систему координат тела
    SP_Math::Vector mag_body = dmp_quat.conjugate().rotate(mag_reading);
    
    // Проецируем на горизонтальную плоскость
    SP_Math::Vector mag_horizontal(mag_body.x(), mag_body.y(), 0);
    mag_horizontal.normalize();
    
    // Вычисляем yaw из магнитометра
    float mag_yaw = std::atan2(mag_horizontal.y(), mag_horizontal.x());
    
    // Вычисляем референсный yaw
    float ref_yaw = std::atan2(reference_north.y(), reference_north.x());
    
    // Коррекция yaw
    float yaw_correction = mag_yaw - ref_yaw;
    
    // Применяем коррекцию с учетом фактора доверия
    float corrected_yaw = extractYaw(dmp_quat) + yaw_correction * trust_factor * YAW_DRIFT_CORRECTION_RATE;
    
    // Создаем новый кватернион с исправленным yaw
    SP_Math::Quaternion yaw_quat = createYawQuaternion(corrected_yaw);
    
    // Сохраняем pitch и roll из DMP
    float roll, pitch, yaw_dmp;
    // Извлекаем euler углы из DMP
    dmp_quat.toEuler(roll, pitch, yaw_dmp);
    
    // Создаем кватернион с исходным roll и pitch, но исправленным yaw
    SP_Math::Quaternion corrected = SP_Math::Quaternion::fromEuler(roll, pitch, corrected_yaw);
    
    return corrected;
}

float RelativeOrientationFilter::integrateGyroYaw(float current_yaw, float gyro_z, float dt) {
    float new_yaw = current_yaw + gyro_z * dt;
    
    // Нормализация угла в диапазон [-π, π]
    while (new_yaw > M_PI) new_yaw -= 2 * M_PI;
    while (new_yaw < -M_PI) new_yaw += 2 * M_PI;
    
    return new_yaw;
}

SP_Math::Quaternion RelativeOrientationFilter::complementaryFilter(
    const SP_Math::Quaternion& dmp_quat,
    const SP_Math::Quaternion& mag_corrected_quat,
    float mag_trust
) {
    // Комплементарный фильтр с адаптивным весом
    // Высокий вес гироскопа (DMP) для краткосрочной точности
    // Магнитометр корректирует долгосрочный дрифт
    
    float gyro_weight = GYRO_INTEGRATION_WEIGHT + (1.0f - mag_trust) * (1.0f - GYRO_INTEGRATION_WEIGHT);
    float mag_weight = 1.0f - gyro_weight;
    
    // SLERP интерполяция между кватернионами
    SP_Math::Quaternion result = dmp_quat.slerp(mag_corrected_quat, mag_weight);
    
    return result.normalize();
}

void RelativeOrientationFilter::updateStrokeState(
    bool stroke_active,
    BladeSideType blade_side,
    float shaft_rotation_angle,
    float blade_force
) {
    uint32_t current_time = millis();
    
    if (stroke_active && !current_stroke.in_progress) {
        // Начало нового гребка
        current_stroke.in_progress = true;
        current_stroke.start_time = current_time;
        current_stroke.start_yaw = extractYaw(filtered_relative_orientation);
        current_stroke.start_shaft_rotation = shaft_rotation_angle;
        current_stroke.blade_side = blade_side;
        
        // Ищем подходящий паттерн
        current_stroke.matched_pattern = findMatchingStrokePattern(
            blade_side, shaft_rotation_angle
        );
        
    } else if (!stroke_active && current_stroke.in_progress) {
        // Конец гребка - сохраняем новый паттерн
        StrokeOrientationPattern new_pattern;
        new_pattern.expected_yaw_change = extractYaw(filtered_relative_orientation) - current_stroke.start_yaw;
        new_pattern.shaft_rotation_start = current_stroke.start_shaft_rotation;
        new_pattern.shaft_rotation_end = shaft_rotation_angle;
        new_pattern.blade_side = blade_side;
        new_pattern.duration_ms = current_time - current_stroke.start_time;
        new_pattern.confidence = (kayak_mag_trust + paddle_mag_trust) / 2.0f;
        new_pattern.is_valid = true;
        
        // Добавляем паттерн если он достаточно длинный и уверенный
        if (new_pattern.duration_ms > 400 && new_pattern.duration_ms < 3000 && 
            new_pattern.confidence > 0.5f) {
            addStrokePattern(new_pattern);
        }
        
        // Сбрасываем состояние гребка
        current_stroke.in_progress = false;
        current_stroke.matched_pattern = nullptr;
    }
}

StrokeOrientationPattern* RelativeOrientationFilter::findMatchingStrokePattern(
    BladeSideType blade_side,
    float shaft_rotation_start
) {
    StrokeOrientationPattern* best_match = nullptr;
    float best_similarity = 0;
    
    for (int i = 0; i < STROKE_PATTERN_MEMORY; i++) {
        if (!stroke_patterns[i].is_valid) continue;
        if (stroke_patterns[i].blade_side != blade_side) continue;
        
        // Вычисляем схожесть по начальному углу шафта
        float angle_diff = std::abs(stroke_patterns[i].shaft_rotation_start - shaft_rotation_start);
        if (angle_diff > 180) angle_diff = 360 - angle_diff; // Нормализация
        
        float similarity = 1.0f - (angle_diff / 180.0f); // [0, 1]
        similarity *= stroke_patterns[i].confidence;
        
        if (similarity > best_similarity && similarity > 0.6f) {
            best_similarity = similarity;
            best_match = &stroke_patterns[i];
        }
    }
    
    return best_match;
}

void RelativeOrientationFilter::addStrokePattern(const StrokeOrientationPattern& pattern) {
    // Ищем самый старый или наименее уверенный паттерн для замены
    int replace_idx = 0;
    float min_confidence = 1.0f;
    
    for (int i = 0; i < STROKE_PATTERN_MEMORY; i++) {
        if (!stroke_patterns[i].is_valid) {
            replace_idx = i;
            break;
        }
        if (stroke_patterns[i].confidence < min_confidence) {
            min_confidence = stroke_patterns[i].confidence;
            replace_idx = i;
        }
    }
    
    // Заменяем паттерн
    stroke_patterns[replace_idx] = pattern;
    
    // Обновляем счетчик
    if (!stroke_patterns[replace_idx].is_valid || pattern_count < STROKE_PATTERN_MEMORY) {
        pattern_count++;
    }
}

SP_Math::Quaternion RelativeOrientationFilter::correctWithStrokePattern(
    const SP_Math::Quaternion& current_quat,
    float stroke_progress
) {
    if (current_stroke.matched_pattern == nullptr) {
        return current_quat;
    }
    
    // Вычисляем ожидаемое изменение yaw на текущем прогрессе
    float expected_yaw_change = current_stroke.matched_pattern->expected_yaw_change * stroke_progress;
    float expected_yaw = current_stroke.start_yaw + expected_yaw_change;
    
    // Текущий yaw
    float current_yaw = extractYaw(current_quat);
    
    // Мягкая коррекция (не резкая)
    float correction_strength = current_stroke.matched_pattern->confidence * 0.1f; // 10% коррекции
    float corrected_yaw = current_yaw + (expected_yaw - current_yaw) * correction_strength;
    
    // Сохраняем roll и pitch
    float roll, pitch, yaw;
    current_quat.toEuler(roll, pitch, yaw);
    
    return SP_Math::Quaternion::fromEuler(roll, pitch, corrected_yaw);
}

SP_Math::Quaternion RelativeOrientationFilter::computeRelativeOrientation(
    const SP_Math::Quaternion& kayak_quat,
    const SP_Math::Quaternion& paddle_quat
) {
    // Получаем направление оси X каяка в мировой системе
    SP_Math::Vector x_k = kayak_quat.rotate(SP_Math::Vector(1, 0, 0));
    
    // Проецируем на горизонтальную плоскость
    float norm_sq = x_k[0]*x_k[0] + x_k[1]*x_k[1];
    
    SP_Math::Vector X_new(0, 0, 0);
    if (norm_sq < 1e-10f) {
        X_new[0] = 1;
        X_new[1] = 0;
        X_new[2] = 0;
    } else {
        float inv_norm = 1.0f / std::sqrt(norm_sq);
        X_new[0] = x_k[0] * inv_norm;
        X_new[1] = x_k[1] * inv_norm;
        X_new[2] = 0;
    }
    
    // Строим кватернион поворота от [1,0,0] к X_new
    float cos_half_theta = std::sqrt(0.5f * (1 + X_new[0]));
    float sin_half_theta = (X_new[1] >= 0 ? 1 : -1) * std::sqrt(0.5f * (1 - X_new[0]));
    SP_Math::Quaternion q_new(cos_half_theta, 0, 0, sin_half_theta);
    
    // Вычисляем относительную ориентацию
    SP_Math::Quaternion q_relative = q_new.conjugate() * paddle_quat;
    
    return q_relative.normalize();
}

float RelativeOrientationFilter::extractYaw(const SP_Math::Quaternion& quat) {
    float roll, pitch, yaw;
    quat.toEuler(roll, pitch, yaw);
    return yaw;
}

SP_Math::Quaternion RelativeOrientationFilter::createYawQuaternion(float yaw_angle) {
    // Кватернион поворота вокруг оси Z
    float half_yaw = yaw_angle * 0.5f;
    return SP_Math::Quaternion(
        std::cos(half_yaw),  // w
        0,                   // x
        0,                   // y
        std::sin(half_yaw)   // z
    );
}

void RelativeOrientationFilter::calibrateReference() {
    // Сохраняем текущие магнитометрические направления как референс
    // Вызывать когда каяк и весло стабильны без помех
    
    // Берем последние значения из истории
    int idx = (kayak_mag_history.index > 0) ? kayak_mag_history.index - 1 : MAG_VARIANCE_WINDOW - 1;
    reference_north_kayak = SP_Math::Vector(
        kayak_mag_history.mag_x[idx],
        kayak_mag_history.mag_y[idx],
        kayak_mag_history.mag_z[idx]
    );
    
    idx = (paddle_mag_history.index > 0) ? paddle_mag_history.index - 1 : MAG_VARIANCE_WINDOW - 1;
    reference_north_paddle = SP_Math::Vector(
        paddle_mag_history.mag_x[idx],
        paddle_mag_history.mag_y[idx],
        paddle_mag_history.mag_z[idx]
    );
    
    reference_north_kayak.normalize();
    reference_north_paddle.normalize();
    
    reference_calibrated = true;
    
    Serial.println("✅ Магнитометр откалиброван - референсное направление сохранено");
}
