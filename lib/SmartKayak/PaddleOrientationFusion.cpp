#include "PaddleOrientationFusion.h"
#include <math.h>

PaddleOrientationFusion::PaddleOrientationFusion() 
    : kayakQuatDMP(1, 0, 0, 0)
    , paddleQuatDMP(1, 0, 0, 0)
    , kayakQuatCorrected(1, 0, 0, 0)
    , paddleQuatCorrected(1, 0, 0, 0)
    , relativeOrientation(1, 0, 0, 0)
    , currentPattern(nullptr)
    , currentPhase(StrokePhase::UNKNOWN)
    , phaseStartTime(0)
    , lastStrokeStartTime(0)
    , magVarianceThreshold(500.0f)      // Порог дисперсии магнитометра
    , magDeviationThreshold(100.0f)     // Порог отклонения от паттерна
    , magSampleBufferSize(MAG_BUFFER_SIZE)
    , magBufferIndex(0)
    , yawDriftRate(0.0f)
    , lastDriftCorrectionTime(0)
    , dmpWeight(0.97f)                  // 97% вес DMP (гироскоп+акселерометр)
    , magWeight(0.03f)                  // 3% вес магнитометра (при хорошем качестве)
    , patternWeight(0.0f)               // 0% изначально, растет с обучением
    , magnetometerCalibrated(false)
    , patternsLearned(false)
    , strokeCount(0)
{
    // Инициализация буферов магнитометра
    for (int i = 0; i < MAG_BUFFER_SIZE; i++) {
        kayakMagBuffer[i] = {0, 0, 0, 0};
        paddleMagBuffer[i] = {0, 0, 0, 0};
    }
}

SP_Math::Quaternion PaddleOrientationFusion::update(
    const IMUData& kayakIMU,
    const IMUData& paddleIMU,
    const OrientationData& kayakOrientation,
    const OrientationData& paddleOrientation,
    float bladeForce)
{
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastDriftCorrectionTime) / 1000.0f; // в секундах
    
    // 1. Загружаем DMP кватернионы (основной источник - гироскоп + акселерометр)
    kayakQuatDMP = SP_Math::Quaternion(kayakOrientation.q0, kayakOrientation.q1, 
                                        kayakOrientation.q2, kayakOrientation.q3);
    paddleQuatDMP = SP_Math::Quaternion(paddleOrientation.q0, paddleOrientation.q1, 
                                         paddleOrientation.q2, paddleOrientation.q3);
    
    // 2. Обновляем буфер магнитометра для анализа качества
    kayakMagBuffer[magBufferIndex] = {kayakIMU.mag_x, kayakIMU.mag_y, kayakIMU.mag_z, currentTime};
    paddleMagBuffer[magBufferIndex] = {paddleIMU.mag_x, paddleIMU.mag_y, paddleIMU.mag_z, currentTime};
    magBufferIndex = (magBufferIndex + 1) % MAG_BUFFER_SIZE;
    
    // 3. Оцениваем качество данных магнитометра
    MagnetometerQuality magQuality = assessMagnetometerQuality(kayakIMU, paddleIMU);
    
    // 4. Вычисляем базовую относительную ориентацию из DMP
    // Проецируем на горизонталь (убираем pitch и roll каяка)
    SP_Math::Vector x_k = kayakQuatDMP.rotate(SP_Math::Vector(1, 0, 0));
    float norm_sq = x_k[0]*x_k[0] + x_k[1]*x_k[1];
    
    SP_Math::Vector X_new(0, 0, 0);
    if (norm_sq < 1e-10f) {
        X_new = SP_Math::Vector(1, 0, 0);
    } else {
        float inv_norm = 1.0f / std::sqrt(norm_sq);
        X_new = SP_Math::Vector(x_k[0] * inv_norm, x_k[1] * inv_norm, 0);
    }
    
    // Кватернион поворота от [1,0,0] к X_new
    float cos_half_theta = std::sqrt(0.5f * (1 + X_new[0]));
    float sin_half_theta = (X_new[1] >= 0 ? 1 : -1) * std::sqrt(0.5f * (1 - X_new[0]));
    SP_Math::Quaternion q_kayak_horizontal(cos_half_theta, 0, 0, sin_half_theta);
    
    // Относительная ориентация (только DMP)
    SP_Math::Quaternion relQuat_DMP = q_kayak_horizontal.conjugate() * paddleQuatDMP;
    relQuat_DMP.normalize();
    
    // 5. Определяем фазу гребка и обновляем паттерны
    float shaftRotation, shaftTilt, bladeRotation;
    getPaddleAngles(relQuat_DMP, shaftRotation, shaftTilt, bladeRotation);
    
    StrokePhase newPhase = detectStrokePhase(shaftTilt, bladeForce, relQuat_DMP);
    if (newPhase != currentPhase) {
        if (newPhase == StrokePhase::CATCH) {
            // Начало нового гребка
            lastStrokeStartTime = currentTime;
            strokeCount++;
            
            // Обновляем паттерн для текущей стороны
            if (currentPattern) {
                updateStrokePattern(*currentPattern, shaftRotation, shaftTilt, bladeRotation);
                
                // После 5 гребков начинаем использовать паттерн
                if (strokeCount >= 5) {
                    patternsLearned = true;
                    patternWeight = 0.1f; // 10% вес паттерна
                }
            }
        }
        currentPhase = newPhase;
        phaseStartTime = currentTime;
    }
    
    // 6. Применяем коррекцию дрифта (только по yaw, не затрагивает pitch/roll)
    SP_Math::Quaternion relQuat_DriftCorrected = relQuat_DMP;
    if (deltaTime > 0.01f) { // Минимум 10мс между коррекциями
        relQuat_DriftCorrected = applyDriftCorrection(relQuat_DMP, deltaTime);
        lastDriftCorrectionTime = currentTime;
    }
    
    // 7. Применяем магнитную коррекцию (только если качество хорошее)
    SP_Math::Quaternion relQuat_MagCorrected = relQuat_DriftCorrected;
    if (magQuality == MagnetometerQuality::EXCELLENT || magQuality == MagnetometerQuality::GOOD) {
        // Коррекция каяка
        SP_Math::Quaternion kayakCorrected = applyMagneticCorrection(kayakQuatDMP, kayakIMU, magQuality);
        // Коррекция весла
        SP_Math::Quaternion paddleCorrected = applyMagneticCorrection(paddleQuatDMP, paddleIMU, magQuality);
        
        // Пересчет относительной ориентации
        SP_Math::Vector x_k_corr = kayakCorrected.rotate(SP_Math::Vector(1, 0, 0));
        float norm_sq_corr = x_k_corr[0]*x_k_corr[0] + x_k_corr[1]*x_k_corr[1];
        
        if (norm_sq_corr >= 1e-10f) {
            float inv_norm_corr = 1.0f / std::sqrt(norm_sq_corr);
            SP_Math::Vector X_new_corr(x_k_corr[0] * inv_norm_corr, x_k_corr[1] * inv_norm_corr, 0);
            
            float cos_half = std::sqrt(0.5f * (1 + X_new_corr[0]));
            float sin_half = (X_new_corr[1] >= 0 ? 1 : -1) * std::sqrt(0.5f * (1 - X_new_corr[0]));
            SP_Math::Quaternion q_corr(cos_half, 0, 0, sin_half);
            
            relQuat_MagCorrected = q_corr.conjugate() * paddleCorrected;
            relQuat_MagCorrected.normalize();
        }
    }
    
    // 8. Применяем коррекцию на основе паттерна (если обучены)
    SP_Math::Quaternion relQuat_PatternCorrected = relQuat_MagCorrected;
    if (patternsLearned && currentPattern && currentPattern->isValid) {
        relQuat_PatternCorrected = applyPatternCorrection(relQuat_MagCorrected, *currentPattern, currentPhase);
    }
    
    // 9. Комплементарный фильтр - объединяем все источники
    relativeOrientation = complementaryFilter(
        relQuat_DriftCorrected,    // Базовый DMP с коррекцией дрифта
        relQuat_MagCorrected,      // С магнитной коррекцией
        relQuat_PatternCorrected,  // С коррекцией по паттерну
        magQuality
    );
    
    return relativeOrientation;
}

// ============================================================================
// Оценка качества магнитометра
// ============================================================================

MagnetometerQuality PaddleOrientationFusion::assessMagnetometerQuality(
    const IMUData& kayakIMU, 
    const IMUData& paddleIMU)
{
    // 1. Проверка на аномалии (резкие скачки)
    if (detectMagneticAnomaly(kayakIMU) || detectMagneticAnomaly(paddleIMU)) {
        return MagnetometerQuality::INVALID;
    }
    
    // 2. Расчет дисперсии за последние N измерений
    float kayakVariance = calculateMagVariance(kayakMagBuffer, MAG_BUFFER_SIZE);
    float paddleVariance = calculateMagVariance(paddleMagBuffer, MAG_BUFFER_SIZE);
    
    // 3. Проверка соответствия паттерну (если паттерн известен)
    float patternDeviation = 0;
    if (currentPattern && currentPattern->isValid) {
        // Упрощенная проверка - в реальности нужно преобразовать mag в углы
        patternDeviation = 0; // TODO: реализовать
    }
    
    // 4. Классификация качества
    float maxVariance = max(kayakVariance, paddleVariance);
    
    if (maxVariance < magVarianceThreshold * 0.3f) {
        return MagnetometerQuality::EXCELLENT;
    } else if (maxVariance < magVarianceThreshold * 0.7f) {
        return MagnetometerQuality::GOOD;
    } else if (maxVariance < magVarianceThreshold) {
        return MagnetometerQuality::POOR;
    } else {
        return MagnetometerQuality::INVALID;
    }
}

float PaddleOrientationFusion::calculateMagVariance(const MagSample* buffer, int size) {
    if (size < 2) return 0;
    
    // Расчет среднего
    float mean_x = 0, mean_y = 0, mean_z = 0;
    for (int i = 0; i < size; i++) {
        mean_x += buffer[i].x;
        mean_y += buffer[i].y;
        mean_z += buffer[i].z;
    }
    mean_x /= size;
    mean_y /= size;
    mean_z /= size;
    
    // Расчет дисперсии
    float var_x = 0, var_y = 0, var_z = 0;
    for (int i = 0; i < size; i++) {
        float dx = buffer[i].x - mean_x;
        float dy = buffer[i].y - mean_y;
        float dz = buffer[i].z - mean_z;
        var_x += dx * dx;
        var_y += dy * dy;
        var_z += dz * dz;
    }
    
    // Возвращаем суммарную дисперсию
    return (var_x + var_y + var_z) / (size - 1);
}

bool PaddleOrientationFusion::detectMagneticAnomaly(const IMUData& imuData) {
    // Проверка на нулевые или слишком большие значения
    if (imuData.mag_x == 0 && imuData.mag_y == 0 && imuData.mag_z == 0) {
        return true; // Нулевые данные
    }
    
    float magnitude = sqrt(imuData.mag_x * imuData.mag_x + 
                          imuData.mag_y * imuData.mag_y + 
                          imuData.mag_z * imuData.mag_z);
    
    // Магнитное поле Земли ~25-65 µT (в сырых единицах MPU ~300-600)
    if (magnitude < 100 || magnitude > 1000) {
        return true; // Аномальная величина
    }
    
    return false;
}

// ============================================================================
// Определение фазы гребка
// ============================================================================

StrokePhase PaddleOrientationFusion::detectStrokePhase(
    float shaftTiltAngle, 
    float bladeForce, 
    const SP_Math::Quaternion& relQuat)
{
    const float CATCH_TILT_THRESHOLD = -10.0f;  // Угол захвата воды
    const float RELEASE_TILT_THRESHOLD = 10.0f; // Угол выхода из воды
    const float FORCE_THRESHOLD = 400.0f;       // Порог силы
    
    // Определяем фазу по углу наклона и силе
    if (shaftTiltAngle < CATCH_TILT_THRESHOLD && abs(bladeForce) < FORCE_THRESHOLD) {
        // Весло опускается, но силы еще нет - захват
        return StrokePhase::CATCH;
    } else if (shaftTiltAngle < 0 && abs(bladeForce) > FORCE_THRESHOLD) {
        // Весло наклонено и есть сила - фаза гребка
        return StrokePhase::DRIVE;
    } else if (shaftTiltAngle > RELEASE_TILT_THRESHOLD && abs(bladeForce) < FORCE_THRESHOLD) {
        // Весло поднимается, силы нет - выход
        return StrokePhase::RELEASE;
    } else if (abs(bladeForce) < FORCE_THRESHOLD) {
        // Нет силы, весло в воздухе - возврат
        return StrokePhase::RECOVERY;
    }
    
    return currentPhase; // Сохраняем текущую фазу если непонятно
}

void PaddleOrientationFusion::updateStrokePattern(
    StrokePattern& pattern,
    float shaftRotation, 
    float shaftTilt, 
    float bladeRotation)
{
    const float LEARNING_RATE = 0.1f; // Скорость обучения паттерна
    
    if (!pattern.isValid) {
        // Первая инициализация
        pattern.shaftRotationRange[0] = pattern.shaftRotationRange[1] = shaftRotation;
        pattern.shaftTiltRange[0] = pattern.shaftTiltRange[1] = shaftTilt;
        pattern.bladeRotationRange[0] = pattern.bladeRotationRange[1] = bladeRotation;
        pattern.avgDuration = 0;
        pattern.isValid = true;
    } else {
        // Обновление диапазонов с экспоненциальным сглаживанием
        pattern.shaftRotationRange[0] = pattern.shaftRotationRange[0] * (1 - LEARNING_RATE) + 
                                        min(shaftRotation, pattern.shaftRotationRange[0]) * LEARNING_RATE;
        pattern.shaftRotationRange[1] = pattern.shaftRotationRange[1] * (1 - LEARNING_RATE) + 
                                        max(shaftRotation, pattern.shaftRotationRange[1]) * LEARNING_RATE;
        
        pattern.shaftTiltRange[0] = pattern.shaftTiltRange[0] * (1 - LEARNING_RATE) + 
                                    min(shaftTilt, pattern.shaftTiltRange[0]) * LEARNING_RATE;
        pattern.shaftTiltRange[1] = pattern.shaftTiltRange[1] * (1 - LEARNING_RATE) + 
                                    max(shaftTilt, pattern.shaftTiltRange[1]) * LEARNING_RATE;
        
        pattern.bladeRotationRange[0] = pattern.bladeRotationRange[0] * (1 - LEARNING_RATE) + 
                                        min(bladeRotation, pattern.bladeRotationRange[0]) * LEARNING_RATE;
        pattern.bladeRotationRange[1] = pattern.bladeRotationRange[1] * (1 - LEARNING_RATE) + 
                                        max(bladeRotation, pattern.bladeRotationRange[1]) * LEARNING_RATE;
    }
}

// ============================================================================
// Коррекция дрифта
// ============================================================================

SP_Math::Quaternion PaddleOrientationFusion::applyDriftCorrection(
    const SP_Math::Quaternion& quat, 
    float deltaTime)
{
    // Оценка дрифта по yaw (только вокруг оси Z)
    // В идеале, во время фазы RECOVERY (возврат весла) yaw должен оставаться стабильным
    
    if (currentPhase == StrokePhase::RECOVERY && deltaTime > 0) {
        // Вычисляем текущий yaw
        // Для кватерниона [w, x, y, z]: yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                         1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
        
        // Обновляем оценку скорости дрифта (очень медленно)
        const float DRIFT_LEARNING_RATE = 0.001f;
        yawDriftRate = yawDriftRate * (1 - DRIFT_LEARNING_RATE) + 
                      (yaw / deltaTime) * DRIFT_LEARNING_RATE;
    }
    
    // Применяем компенсацию дрифта
    float driftAngle = -yawDriftRate * deltaTime * 0.01f; // Малая коррекция
    
    // Кватернион коррекции вокруг Z
    float half_angle = driftAngle * 0.5f;
    SP_Math::Quaternion driftCorrection(
        cos(half_angle),
        0,
        0,
        sin(half_angle)
    );
    
    return (driftCorrection * quat).normalize();
}

// ============================================================================
// Магнитная коррекция
// ============================================================================

SP_Math::Quaternion PaddleOrientationFusion::applyMagneticCorrection(
    const SP_Math::Quaternion& dmpQuat,
    const IMUData& imuData,
    MagnetometerQuality quality)
{
    if (quality == MagnetometerQuality::POOR || quality == MagnetometerQuality::INVALID) {
        return dmpQuat; // Не используем плохие данные
    }
    
    // Вычисляем yaw из магнитометра
    float mag_x = imuData.mag_x;
    float mag_y = imuData.mag_y;
    
    // Компенсация наклона (tilt compensation)
    // Поворачиваем вектор магнитного поля в горизонтальную плоскость
    SP_Math::Vector magVector(mag_x, mag_y, imuData.mag_z);
    SP_Math::Vector magHorizontal = dmpQuat.conjugate().rotate(magVector);
    
    float yaw_mag = atan2(magHorizontal.y(), magHorizontal.x());
    
    // Вычисляем yaw из DMP
    float yaw_dmp = atan2(2.0f * (dmpQuat.w() * dmpQuat.z() + dmpQuat.x() * dmpQuat.y()),
                         1.0f - 2.0f * (dmpQuat.y() * dmpQuat.y() + dmpQuat.z() * dmpQuat.z()));
    
    // Вычисляем ошибку yaw
    float yaw_error = yaw_mag - yaw_dmp;
    
    // Нормализация угла [-π, π]
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
    
    // Применяем коррекцию с весом в зависимости от качества
    float correction_weight = (quality == MagnetometerQuality::EXCELLENT) ? magWeight : magWeight * 0.5f;
    float yaw_correction = yaw_error * correction_weight;
    
    // Кватернион коррекции вокруг Z
    float half_corr = yaw_correction * 0.5f;
    SP_Math::Quaternion correction(
        cos(half_corr),
        0,
        0,
        sin(half_corr)
    );
    
    return (correction * dmpQuat).normalize();
}

// ============================================================================
// Коррекция по паттерну
// ============================================================================

SP_Math::Quaternion PaddleOrientationFusion::applyPatternCorrection(
    const SP_Math::Quaternion& quat,
    const StrokePattern& pattern,
    StrokePhase phase)
{
    // Паттерн-коррекция применяется только в предсказуемых фазах
    if (phase != StrokePhase::DRIVE && phase != StrokePhase::RECOVERY) {
        return quat;
    }
    
    // Извлекаем углы из кватерниона
    float shaftRotation, shaftTilt, bladeRotation;
    getPaddleAngles(quat, shaftRotation, shaftTilt, bladeRotation);
    
    // Проверяем выход за границы паттерна
    bool outOfBounds = false;
    float correction = 0;
    
    // Коррекция shaft rotation
    if (shaftRotation < pattern.shaftRotationRange[0]) {
        correction = pattern.shaftRotationRange[0] - shaftRotation;
        outOfBounds = true;
    } else if (shaftRotation > pattern.shaftRotationRange[1]) {
        correction = pattern.shaftRotationRange[1] - shaftRotation;
        outOfBounds = true;
    }
    
    if (outOfBounds) {
        // Мягкая коррекция к границам паттерна
        float corr_angle = correction * patternWeight * DEG_TO_RAD;
        float half_angle = corr_angle * 0.5f;
        
        SP_Math::Quaternion patternCorr(
            cos(half_angle),
            0,
            0,
            sin(half_angle)
        );
        
        return (patternCorr * quat).normalize();
    }
    
    return quat;
}

// ============================================================================
// Комплементарный фильтр
// ============================================================================

SP_Math::Quaternion PaddleOrientationFusion::complementaryFilter(
    const SP_Math::Quaternion& dmpQuat,
    const SP_Math::Quaternion& magCorrected,
    const SP_Math::Quaternion& patternPredicted,
    MagnetometerQuality magQuality)
{
    // Адаптивные веса в зависимости от качества магнитометра
    float w_dmp = dmpWeight;
    float w_mag = 0;
    float w_pattern = 0;
    
    switch (magQuality) {
        case MagnetometerQuality::EXCELLENT:
            w_mag = magWeight;
            w_pattern = patternsLearned ? patternWeight * 0.5f : 0;
            break;
        case MagnetometerQuality::GOOD:
            w_mag = magWeight * 0.5f;
            w_pattern = patternsLearned ? patternWeight : 0;
            break;
        case MagnetometerQuality::POOR:
            w_mag = 0;
            w_pattern = patternsLearned ? patternWeight * 1.5f : 0;
            break;
        case MagnetometerQuality::INVALID:
            w_mag = 0;
            w_pattern = patternsLearned ? patternWeight * 2.0f : 0;
            break;
    }
    
    // Нормализация весов
    float total_weight = w_dmp + w_mag + w_pattern;
    w_dmp /= total_weight;
    w_mag /= total_weight;
    w_pattern /= total_weight;
    
    // SLERP (Spherical Linear Interpolation) для кватернионов
    // Сначала объединяем DMP и магнитную коррекцию
    SP_Math::Quaternion intermediate = dmpQuat.slerp(magCorrected, w_mag / (w_dmp + w_mag));
    
    // Затем добавляем коррекцию по паттерну
    SP_Math::Quaternion result = intermediate.slerp(patternPredicted, w_pattern);
    
    return result.normalize();
}

// Вспомогательная функция для извлечения углов (используем из SmartKayak.cpp)
extern void getPaddleAngles(const SP_Math::Quaternion& relativePaddleQ, 
                            float& shaftRotationAngle,
                            float& shaftTiltAngle,
                            float& bladeRotationAngle);
