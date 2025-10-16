#include "RelativeOrientationFilter.h"
#include <cmath>

RelativeOrientationFilter::RelativeOrientationFilter() 
    : filteredRelativeQuat(1, 0, 0, 0)
    , lastKayakQuat(1, 0, 0, 0)
    , lastPaddleQuat(1, 0, 0, 0)
    , currentPhase(StrokePhase::UNKNOWN)
    , lastPhase(StrokePhase::UNKNOWN)
    , expectedCatchQuat(1, 0, 0, 0)
    , expectedReleaseQuat(1, 0, 0, 0)
    , yawDriftCorrection(0.0f)
{
    kayakMagStats = {0, 0, 0, false, 0};
    paddleMagStats = {0, 0, 0, false, 0};
    kayakMagHistory.reserve(MAX_HISTORY);
    paddleMagHistory.reserve(MAX_HISTORY);
}

SP_Math::Quaternion RelativeOrientationFilter::update(
    const SP_Math::Quaternion& kayakQuat,
    const SP_Math::Quaternion& paddleQuat,
    const SP_Math::Vector& kayakMag,
    const SP_Math::Vector& paddleMag,
    float shaftAngle,
    float bladeForce
) {
    unsigned long now = millis();
    
    // 1. Обновляем статистику магнитометров
    updateMagStats(kayakMagStats, kayakMag, kayakMagHistory);
    updateMagStats(paddleMagStats, paddleMag, paddleMagHistory);
    
    // 2. Оцениваем доверие к магнитометрам
    float kayakMagTrust = evaluateMagnetometerTrust(kayakMagStats, kayakMag);
    float paddleMagTrust = evaluateMagnetometerTrust(paddleMagStats, paddleMag);
    
    // Общее доверие - минимум из двух (оба должны быть надежны)
    float magTrust = std::min(kayakMagTrust, paddleMagTrust);
    
    // 3. Вычисляем базовую относительную ориентацию из DMP кватернионов
    SP_Math::Quaternion baseRelativeQuat = computeBasicRelativeOrientation(kayakQuat, paddleQuat);
    
    // 4. Применяем коррекцию yaw от магнитометра (если надежен)
    SP_Math::Quaternion magCorrectedQuat = baseRelativeQuat;
    if (magTrust > 0.01f) {
        magCorrectedQuat = correctYawWithMagnetometer(baseRelativeQuat, kayakMag, paddleMag, magTrust);
    }
    
    // 5. Определяем фазу гребка
    float shaftTilt = 0; // Можно передать как параметр если есть
    currentPhase = detectStrokePhase(shaftAngle, shaftTilt, bladeForce);
    
    // 6. Применяем коррекцию на основе фазы гребка
    SP_Math::Quaternion phaseCorrectedQuat = magCorrectedQuat;
    if (params.useStrokePhaseCorrection) {
        phaseCorrectedQuat = correctWithStrokePhase(magCorrectedQuat, currentPhase);
        
        // Обучаем ожидаемые положения в критических фазах
        if (currentPhase == StrokePhase::CATCH && lastPhase != StrokePhase::CATCH) {
            calibrateStrokePhaseQuaternions(StrokePhase::CATCH, phaseCorrectedQuat);
        }
        if (currentPhase == StrokePhase::RELEASE && lastPhase != StrokePhase::RELEASE) {
            calibrateStrokePhaseQuaternions(StrokePhase::RELEASE, phaseCorrectedQuat);
        }
    }
    
    // 7. Комплементарный фильтр для сглаживания
    // Используем высокое доверие к DMP (98%) и низкое к коррекциям (2%)
    float alpha = params.gyroTrust;
    filteredRelativeQuat = filteredRelativeQuat.slerp(phaseCorrectedQuat, 1.0f - alpha);
    filteredRelativeQuat.normalize();
    
    // Сохраняем состояние
    lastKayakQuat = kayakQuat;
    lastPaddleQuat = paddleQuat;
    lastPhase = currentPhase;
    
    return filteredRelativeQuat;
}

StrokePhase RelativeOrientationFilter::detectStrokePhase(float shaftAngle, float shaftTilt, float bladeForce) {
    // Определяем фазу на основе угла, наклона и силы
    
    // CATCH: весло входит в воду (наклон отрицательный, начинается сила)
    if (shaftTilt < -20.0f && abs(bladeForce) > 300) {
        return StrokePhase::CATCH;
    }
    
    // PULL: основная тяга (высокая сила)
    if (abs(bladeForce) > 600) {
        return StrokePhase::PULL;
    }
    
    // RELEASE: выход из воды (наклон положительный, сила падает)
    if (shaftTilt > 20.0f && abs(bladeForce) < 300 && lastPhase == StrokePhase::PULL) {
        return StrokePhase::RELEASE;
    }
    
    // RECOVERY: проводка в воздухе (малая сила)
    if (abs(bladeForce) < 200) {
        return StrokePhase::RECOVERY;
    }
    
    return StrokePhase::UNKNOWN;
}

float RelativeOrientationFilter::evaluateMagnetometerTrust(MagnetometerStats& stats, const SP_Math::Vector& magData) {
    if (!stats.isReliable) {
        return 0.0f;
    }
    
    float trust = params.magTrustMax;
    
    // 1. Проверка величины поля (должна быть близка к номинальной)
    float magnitudeDeviation = abs(stats.magnitude - params.nominalMagMagnitude) / params.nominalMagMagnitude;
    if (magnitudeDeviation > params.magTolerancePercent / 100.0f) {
        trust *= 0.5f; // Снижаем доверие
    }
    
    // 2. Проверка скорости изменения (не должна быть слишком высокой)
    if (stats.changeRate > params.maxMagChangeRate) {
        trust *= 0.3f; // Сильно снижаем доверие при быстрых изменениях
    }
    
    // 3. Проверка консистентности (должна быть высокой)
    if (stats.consistency < 0.7f) {
        trust *= 0.5f;
    }
    
    return std::max(params.magTrustMin, trust);
}

void RelativeOrientationFilter::updateMagStats(MagnetometerStats& stats, const SP_Math::Vector& magData, 
                                               std::vector<MagSample>& history) {
    unsigned long now = millis();
    
    // Добавляем в историю
    MagSample sample = {magData.x(), magData.y(), magData.z(), now};
    history.push_back(sample);
    if (history.size() > MAX_HISTORY) {
        history.erase(history.begin());
    }
    
    // Вычисляем величину
    stats.magnitude = std::sqrt(magData.x()*magData.x() + magData.y()*magData.y() + magData.z()*magData.z());
    
    // Вычисляем скорость изменения
    if (history.size() >= 2) {
        const MagSample& prev = history[history.size() - 2];
        float dx = magData.x() - prev.x;
        float dy = magData.y() - prev.y;
        float dz = magData.z() - prev.z;
        float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        float dt = (now - prev.timestamp) / 1000.0f; // в секундах
        stats.changeRate = (dt > 0) ? (distance / dt) : 0;
    }
    
    // Вычисляем консистентность (стандартное отклонение величины)
    if (history.size() >= 10) {
        float sumMag = 0;
        float sumMagSq = 0;
        for (const auto& s : history) {
            float mag = std::sqrt(s.x*s.x + s.y*s.y + s.z*s.z);
            sumMag += mag;
            sumMagSq += mag * mag;
        }
        float meanMag = sumMag / history.size();
        float variance = (sumMagSq / history.size()) - (meanMag * meanMag);
        float stdDev = std::sqrt(variance);
        
        // Консистентность высокая если std dev низкое
        stats.consistency = 1.0f - std::min(1.0f, stdDev / params.nominalMagMagnitude);
    } else {
        stats.consistency = 0.5f; // Недостаточно данных
    }
    
    // Определяем надежность
    stats.isReliable = (
        abs(stats.magnitude - params.nominalMagMagnitude) < params.nominalMagMagnitude * params.magTolerancePercent / 100.0f &&
        stats.changeRate < params.maxMagChangeRate &&
        stats.consistency > 0.6f
    );
    
    stats.lastUpdate = now;
}

SP_Math::Quaternion RelativeOrientationFilter::computeBasicRelativeOrientation(
    const SP_Math::Quaternion& kayakQuat,
    const SP_Math::Quaternion& paddleQuat
) {
    // Используем упрощенный метод из оригинального кода
    // Получаем направление X каяка и проецируем на горизонталь
    SP_Math::Vector x_k = kayakQuat.rotate(SP_Math::Vector(1, 0, 0));
    
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
    SP_Math::Quaternion q_relative = q_new.conjugate() * paddleQuat;
    return q_relative.normalize();
}

SP_Math::Quaternion RelativeOrientationFilter::correctYawWithMagnetometer(
    const SP_Math::Quaternion& relativeQuat,
    const SP_Math::Vector& kayakMag,
    const SP_Math::Vector& paddleMag,
    float trust
) {
    // Вычисляем heading (yaw) из магнитометров
    float kayakYaw = atan2(kayakMag.y(), kayakMag.x());
    float paddleYaw = atan2(paddleMag.y(), paddleMag.x());
    
    // Относительный yaw
    float relativeYaw = paddleYaw - kayakYaw;
    
    // Нормализуем в диапазон [-π, π]
    while (relativeYaw > M_PI) relativeYaw -= 2*M_PI;
    while (relativeYaw < -M_PI) relativeYaw += 2*M_PI;
    
    // Извлекаем текущий yaw из кватерниона
    float currentYaw = atan2(2.0f * (relativeQuat.w() * relativeQuat.z() + relativeQuat.x() * relativeQuat.y()),
                             1.0f - 2.0f * (relativeQuat.y() * relativeQuat.y() + relativeQuat.z() * relativeQuat.z()));
    
    // Вычисляем коррекцию
    float yawError = relativeYaw - currentYaw;
    while (yawError > M_PI) yawError -= 2*M_PI;
    while (yawError < -M_PI) yawError += 2*M_PI;
    
    // Применяем коррекцию пропорционально доверию
    float correction = yawError * trust;
    yawDriftCorrection += correction;
    
    // Создаем кватернион коррекции вокруг Z
    float halfAngle = correction / 2.0f;
    SP_Math::Quaternion correctionQuat(cos(halfAngle), 0, 0, sin(halfAngle));
    
    // Применяем коррекцию
    SP_Math::Quaternion corrected = relativeQuat * correctionQuat;
    return corrected.normalize();
}

SP_Math::Quaternion RelativeOrientationFilter::correctWithStrokePhase(
    const SP_Math::Quaternion& relativeQuat,
    StrokePhase phase
) {
    // Коррекция работает только в определенных фазах с известным положением
    SP_Math::Quaternion targetQuat = relativeQuat;
    float weight = 0.0f;
    
    switch (phase) {
        case StrokePhase::CATCH:
            // В момент захвата весло должно быть в определенном положении
            if (expectedCatchQuat.w() != 1.0f || expectedCatchQuat.x() != 0.0f) {
                targetQuat = expectedCatchQuat;
                weight = params.strokePhaseWeight * 0.3f; // Небольшая коррекция
            }
            break;
            
        case StrokePhase::RELEASE:
            // В момент выхода также известное положение
            if (expectedReleaseQuat.w() != 1.0f || expectedReleaseQuat.x() != 0.0f) {
                targetQuat = expectedReleaseQuat;
                weight = params.strokePhaseWeight * 0.3f;
            }
            break;
            
        case StrokePhase::RECOVERY:
            // В фазе проводки можно сделать более сильную коррекцию
            // так как весло в воздухе и положение более предсказуемо
            weight = params.strokePhaseWeight * 0.5f;
            break;
            
        default:
            weight = 0.0f;
            break;
    }
    
    if (weight > 0.0f) {
        // Интерполируем между текущим и целевым
        return relativeQuat.slerp(targetQuat, weight);
    }
    
    return relativeQuat;
}

void RelativeOrientationFilter::calibrateStrokePhaseQuaternions(
    StrokePhase phase,
    const SP_Math::Quaternion& observedQuat
) {
    // Обучаем ожидаемые положения с экспоненциальным сглаживанием
    float learningRate = 0.05f; // Медленно обучаемся
    
    switch (phase) {
        case StrokePhase::CATCH:
            if (expectedCatchQuat.w() == 1.0f && expectedCatchQuat.x() == 0.0f) {
                // Первое наблюдение
                expectedCatchQuat = observedQuat;
            } else {
                // Обновляем с сглаживанием
                expectedCatchQuat = expectedCatchQuat.slerp(observedQuat, learningRate);
                expectedCatchQuat.normalize();
            }
            break;
            
        case StrokePhase::RELEASE:
            if (expectedReleaseQuat.w() == 1.0f && expectedReleaseQuat.x() == 0.0f) {
                expectedReleaseQuat = observedQuat;
            } else {
                expectedReleaseQuat = expectedReleaseQuat.slerp(observedQuat, learningRate);
                expectedReleaseQuat.normalize();
            }
            break;
            
        default:
            break;
    }
}
