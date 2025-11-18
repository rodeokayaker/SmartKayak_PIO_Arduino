#include "HybridOrientationEstimator.h"
#include <cmath>

HybridOrientationEstimator::HybridOrientationEstimator() :
    historyIndex(0),
    historyCount(0),
    strokeHistoryIndex(0),
    strokeHistoryCount(0),
    magReliability(0.0f),
    expectedMagMagnitude(50.0f),  // Примерная величина земного магнитного поля
    magCalibrated(false),
    gyroWeight(0.98f),
    accelWeight(0.02f),
    magWeight(0.0f),  // Начинаем без магнитометра
    patternWeight(0.0f),
    currentPhase(StrokePhase::UNKNOWN),
    phaseConfidence(0.0f),
    lastUpdateTime(0),
    filteredDMPQuat(1, 0, 0, 0),
    hybridQuat(1, 0, 0, 0),
    relativeQuat(1, 0, 0, 0),
    kayakQuat(1, 0, 0, 0),
    driftCorrection(1, 0, 0, 0)
{
    // Инициализация истории нулями
    for (int i = 0; i < HISTORY_SIZE; i++) {
        magHistory[i][0] = 0;
        magHistory[i][1] = 0;
        magHistory[i][2] = 0;
    }
}

SP_Math::Quaternion HybridOrientationEstimator::updateRelativeOrientation(
    const IMUData& paddleIMU,
    const OrientationData& paddleDMP,
    const IMUData& kayakIMU,
    const OrientationData& kayakDMP)
{
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f;
    if (lastUpdateTime == 0) dt = 0.001f;  // Первая итерация
    lastUpdateTime = currentTime;
    
    // 1. ДЕТЕКЦИЯ МАГНИТНЫХ ПОМЕХ
    float paddleMagReliability = detectMagneticAnomaly(paddleIMU);
    float kayakMagReliability = detectMagneticAnomaly(kayakIMU);
    
    // Используем минимальную надежность
    magReliability = std::min(paddleMagReliability, kayakMagReliability);
    
    // 2. АДАПТИВНАЯ ФУЗИЯ ДЛЯ ВЕСЛА
    SP_Math::Quaternion paddleQuat(paddleDMP.q0, paddleDMP.q1, paddleDMP.q2, paddleDMP.q3);
    SP_Math::Quaternion fusedPaddleQuat = adaptiveFusion(paddleQuat, paddleIMU, paddleMagReliability);
    
    // 3. АДАПТИВНАЯ ФУЗИЯ ДЛЯ КАЯКА
    kayakQuat = SP_Math::Quaternion(kayakDMP.q0, kayakDMP.q1, kayakDMP.q2, kayakDMP.q3);
    SP_Math::Quaternion fusedKayakQuat = adaptiveFusion(kayakQuat, kayakIMU, kayakMagReliability);
    
    // 4. ВЫЧИСЛЕНИЕ ОТНОСИТЕЛЬНОЙ ОРИЕНТАЦИИ
    relativeQuat = calculateRelativeOrientation(fusedPaddleQuat, fusedKayakQuat);
    
    // 5. КОРРЕКЦИЯ ПО ПАТТЕРНУ ГРЕБЛИ (если надежность магнитометра низкая)
    if (magReliability < 0.5f && strokeHistoryCount > 3) {
        SP_Math::Quaternion patternCorrected = correctDriftByPattern(relativeQuat, currentPhase);
        
        // Смешиваем с коррекцией по паттерну
        float blendFactor = (0.5f - magReliability) * 2.0f * patternWeight;  // 0..1 когда mag<0.5
        relativeQuat = relativeQuat.slerp(patternCorrected, blendFactor);
    }
    
    return relativeQuat;
}

float HybridOrientationEstimator::detectMagneticAnomaly(const IMUData& imuData) {
    // Добавляем текущие данные в историю
    addToMagHistory(imuData);
    
    if (historyCount < 3) {
        return 0.0f;  // Недостаточно данных
    }
    
    // 1. Проверка магнитуды поля
    float magnitude = calculateMagMagnitude(imuData);
    
    if (!magCalibrated) {
        // Калибруем ожидаемую магнитуду
        if (historyCount >= anomalyDetector.windowSize) {
            float avgMag = 0;
            for (int i = 0; i < historyCount; i++) {
                avgMag += std::sqrt(
                    magHistory[i][0] * magHistory[i][0] +
                    magHistory[i][1] * magHistory[i][1] +
                    magHistory[i][2] * magHistory[i][2]
                );
            }
            expectedMagMagnitude = avgMag / historyCount;
            magCalibrated = true;
        }
    }
    
    // 2. Проверка отклонения от ожидаемой магнитуды
    float magnitudeDeviation = std::abs(magnitude - expectedMagMagnitude) / expectedMagMagnitude;
    float magnitudeScore = 1.0f - std::min(magnitudeDeviation * 2.0f, 1.0f);
    
    // 3. Проверка вариации (стабильности)
    float variance = calculateMagVariance();
    float varianceScore = 1.0f - std::min(variance / anomalyDetector.varianceThreshold, 1.0f);
    
    // 4. Проверка диапазона
    float rangeScore = 1.0f;
    if (magnitude < anomalyDetector.magnitudeMin || magnitude > anomalyDetector.magnitudeMax) {
        rangeScore = 0.0f;
    }
    
    // 5. Комбинированная оценка надежности
    float reliability = (magnitudeScore * 0.3f + varianceScore * 0.5f + rangeScore * 0.2f);
    
    // Сглаживание оценки надежности
    static float smoothedReliability = 0.5f;
    smoothedReliability = smoothedReliability * anomalyDetector.smoothingFactor + 
                          reliability * (1.0f - anomalyDetector.smoothingFactor);
    
    return smoothedReliability;
}

StrokePhase::Phase HybridOrientationEstimator::detectStrokePhase(
    float shaftRotationAngle,
    float shaftTiltAngle,
    float bladeForce)
{
    StrokePhase::Phase detectedPhase = StrokePhase::UNKNOWN;
    float confidence = 0.0f;
    
    // Определяем фазу по углам и силе
    if (std::abs(bladeForce) > 400) {
        // Есть нагрузка - это фаза проводки
        if (shaftTiltAngle < -5.0f) {
            detectedPhase = StrokePhase::PULL;
            confidence = 0.9f;
        } else {
            detectedPhase = StrokePhase::CATCH;
            confidence = 0.7f;
        }
    } else {
        // Нет нагрузки
        if (shaftTiltAngle < -15.0f) {
            // Весло глубоко наклонено - выход
            detectedPhase = StrokePhase::RELEASE;
            confidence = 0.7f;
        } else {
            // Весло над водой - возврат
            detectedPhase = StrokePhase::RECOVERY;
            confidence = 0.8f;
        }
    }
    
    // Валидация по истории
    if (strokeHistoryCount > 0) {
        StrokePhase::Phase lastPhase = strokeHistory[(strokeHistoryIndex - 1 + STROKE_HISTORY_SIZE) % STROKE_HISTORY_SIZE].phase;
        
        // Проверяем логическую последовательность фаз
        bool validTransition = false;
        if (lastPhase == StrokePhase::CATCH && detectedPhase == StrokePhase::PULL) validTransition = true;
        if (lastPhase == StrokePhase::PULL && detectedPhase == StrokePhase::RELEASE) validTransition = true;
        if (lastPhase == StrokePhase::RELEASE && detectedPhase == StrokePhase::RECOVERY) validTransition = true;
        if (lastPhase == StrokePhase::RECOVERY && detectedPhase == StrokePhase::CATCH) validTransition = true;
        if (lastPhase == detectedPhase) validTransition = true;  // Оставаемся в той же фазе
        
        if (!validTransition) {
            confidence *= 0.5f;  // Снижаем уверенность при нелогичном переходе
        }
    }
    
    currentPhase = detectedPhase;
    phaseConfidence = confidence;
    
    // Сохраняем в историю
    StrokePhase phase;
    phase.phase = detectedPhase;
    phase.shaftRotationAngle = shaftRotationAngle;
    phase.shaftTiltAngle = shaftTiltAngle;
    phase.bladeRotationAngle = 0;  // Можно добавить если нужно
    phase.timestamp = millis();
    addToStrokeHistory(phase);
    
    return detectedPhase;
}

SP_Math::Quaternion HybridOrientationEstimator::correctDriftByPattern(
    const SP_Math::Quaternion& currentQuat,
    StrokePhase::Phase phase)
{
    if (strokeHistoryCount < 5) {
        return currentQuat;  // Недостаточно истории
    }
    
    // Ищем похожие фазы в истории
    float avgRotationAngle = 0;
    int matchCount = 0;
    
    for (int i = 0; i < strokeHistoryCount; i++) {
        if (strokeHistory[i].phase == phase) {
            avgRotationAngle += strokeHistory[i].shaftRotationAngle;
            matchCount++;
        }
    }
    
    if (matchCount > 0) {
        avgRotationAngle /= matchCount;
        
        // Получаем текущий yaw из кватерниона
        float currentYaw = currentQuat.yaw();
        
        // Вычисляем ожидаемый yaw на основе паттерна
        float expectedYaw = avgRotationAngle * DEGREES_TO_RADIANS;
        
        // Вычисляем коррекцию
        float yawCorrection = expectedYaw - currentYaw;
        
        // Ограничиваем коррекцию (не более 15 градусов)
        yawCorrection = std::max(-15.0f * DEGREES_TO_RADIANS, 
                                std::min(15.0f * DEGREES_TO_RADIANS, yawCorrection));
        
        // Применяем коррекцию как поворот вокруг оси Z
        SP_Math::Quaternion correction = SP_Math::Quaternion::fromAxisAngle(
            SP_Math::Vector(0, 0, 1), yawCorrection
        );
        
        return correction * currentQuat;
    }
    
    return currentQuat;
}

void HybridOrientationEstimator::addToMagHistory(const IMUData& imuData) {
    magHistory[historyIndex][0] = imuData.mx;
    magHistory[historyIndex][1] = imuData.my;
    magHistory[historyIndex][2] = imuData.mz;
    
    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    if (historyCount < HISTORY_SIZE) {
        historyCount++;
    }
}

float HybridOrientationEstimator::calculateMagVariance() {
    if (historyCount < 2) return 0;
    
    // Вычисляем среднее
    float mean[3] = {0, 0, 0};
    for (int i = 0; i < historyCount; i++) {
        mean[0] += magHistory[i][0];
        mean[1] += magHistory[i][1];
        mean[2] += magHistory[i][2];
    }
    mean[0] /= historyCount;
    mean[1] /= historyCount;
    mean[2] /= historyCount;
    
    // Вычисляем дисперсию
    float variance = 0;
    for (int i = 0; i < historyCount; i++) {
        float diff[3] = {
            magHistory[i][0] - mean[0],
            magHistory[i][1] - mean[1],
            magHistory[i][2] - mean[2]
        };
        variance += diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2];
    }
    variance /= historyCount;
    
    return variance;
}

float HybridOrientationEstimator::calculateMagMagnitude(const IMUData& imuData) {
    return std::sqrt(
        imuData.mx * imuData.mx +
        imuData.my * imuData.my +
        imuData.mz * imuData.mz
    );
}

SP_Math::Quaternion HybridOrientationEstimator::adaptiveFusion(
    const SP_Math::Quaternion& dmpQuat,
    const IMUData& imuData,
    float magReliability)
{
    // DMP уже содержит фузию гироскопа и акселерометра
    // Мы добавляем коррекцию yaw от магнитометра, если он надежен
    
    if (magReliability < 0.3f) {
        // Магнитометр ненадежен - используем только DMP
        return dmpQuat;
    }
    
    // Извлекаем yaw из магнитометра
    float magneticYaw = extractMagneticYaw(imuData);
    
    // Корректируем yaw дрифт
    return correctYawDrift(dmpQuat, magneticYaw, magReliability);
}

float HybridOrientationEstimator::extractMagneticYaw(const IMUData& imuData) {
    // Простая оценка yaw из магнитометра
    // Предполагаем, что устройство относительно горизонтально
    float yaw = std::atan2(imuData.my, imuData.mx);
    return yaw;
}

SP_Math::Quaternion HybridOrientationEstimator::correctYawDrift(
    const SP_Math::Quaternion& quat,
    float magneticYaw,
    float reliability)
{
    // Получаем текущий yaw из кватерниона
    float currentYaw = quat.yaw();
    
    // Вычисляем разницу
    float yawDiff = magneticYaw - currentYaw;
    
    // Нормализуем разницу в диапазон [-π, π]
    while (yawDiff > M_PI) yawDiff -= 2 * M_PI;
    while (yawDiff < -M_PI) yawDiff += 2 * M_PI;
    
    // Применяем коррекцию пропорционально надежности
    float correction = yawDiff * reliability * magWeight;
    
    // Создаем кватернион коррекции вокруг оси Z
    SP_Math::Quaternion correctionQuat = SP_Math::Quaternion::fromAxisAngle(
        SP_Math::Vector(0, 0, 1), correction
    );
    
    // Применяем коррекцию
    return correctionQuat * quat;
}

void HybridOrientationEstimator::addToStrokeHistory(const StrokePhase& phase) {
    strokeHistory[strokeHistoryIndex] = phase;
    strokeHistoryIndex = (strokeHistoryIndex + 1) % STROKE_HISTORY_SIZE;
    if (strokeHistoryCount < STROKE_HISTORY_SIZE) {
        strokeHistoryCount++;
    }
}

SP_Math::Quaternion HybridOrientationEstimator::calculateRelativeOrientation(
    const SP_Math::Quaternion& paddleQuat,
    const SP_Math::Quaternion& kayakQuat)
{
    // Получаем направление оси X каяка в мировой системе координат
    SP_Math::Vector x_k = kayakQuat.rotate(SP_Math::Vector(1, 0, 0));
    
    // Проецируем на горизонтальную плоскость (убираем наклон каяка)
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

void HybridOrientationEstimator::resetHistory() {
    historyCount = 0;
    historyIndex = 0;
    strokeHistoryCount = 0;
    strokeHistoryIndex = 0;
    magCalibrated = false;
    magReliability = 0.0f;
}

void HybridOrientationEstimator::printDiagnostics(Stream* stream) {
    stream->println("=== Hybrid Orientation Estimator Diagnostics ===");
    stream->printf("Mag Reliability: %.2f\n", magReliability);
    stream->printf("Expected Mag Magnitude: %.2f μT\n", expectedMagMagnitude);
    stream->printf("Current Phase: %d (confidence: %.2f)\n", (int)currentPhase, phaseConfidence);
    stream->printf("Stroke History Count: %d\n", strokeHistoryCount);
    stream->printf("Mag History Count: %d\n", historyCount);
    stream->printf("Filter Weights - Gyro: %.2f, Accel: %.2f, Mag: %.2f, Pattern: %.2f\n",
                   gyroWeight, accelWeight, magWeight, patternWeight);
    stream->println("================================================");
}

