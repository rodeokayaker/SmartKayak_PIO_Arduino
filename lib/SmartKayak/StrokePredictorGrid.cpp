#include "StrokePredictorGrid.h"
#include <cmath>

StrokePredictorGrid::StrokePredictorGrid() 
    : omegaIndex(0), omegaHistoryFilled(false) {
    // Инициализация истории
    for (int i = 0; i < 5; i++) {
        omegaHistory[i] = SP_Math::Vector(0, 0, 0);
        omegaTimestamps[i] = 0;
    }
}

StrokePredictorGrid::~StrokePredictorGrid() {
    // Очистка не требуется - unordered_map очистится автоматически
}

//=============================================================================
// ОСНОВНЫЕ ФУНКЦИИ
//=============================================================================

void StrokePredictorGrid::learn(
    const SP_Math::Quaternion& paddleQuat,
    const SP_Math::Quaternion& kayakQuat,
    const SP_Math::Vector& angularVelocity,
    float currentForce,
    bool isForward
) {
    // Обновляем историю угловых скоростей
    updateOmegaHistory(angularVelocity);
    
    // Проверяем порог силы для обучения
    // Если сила меньше порога И ячейка еще не заполнена, не обучаемся
    if (fabs(currentForce) < config.forceThreshold) {
        // Получаем относительную ориентацию
        SP_Math::Quaternion relativeQuat = getRelativeOrientation(paddleQuat, kayakQuat);
        
        // Извлекаем углы
        float shaftRotation, shaftTilt, bladeRotation;
        getPaddleAngles(relativeQuat, shaftRotation, shaftTilt, bladeRotation);
        
        // Вычисляем индексы
        int idxRot, idxTilt, idxBlade;
        float fracRot, fracTilt, fracBlade;
        angleToIndices(shaftRotation, shaftTilt, bladeRotation, 
                      idxRot, idxTilt, idxBlade,
                      fracRot, fracTilt, fracBlade);
        
        // Проверяем существование хотя бы одной вершины куба
        uint32_t keys[8];
        get8NeighborKeys(idxRot, idxTilt, idxBlade, keys);
        
        bool hasAnyVertex = false;
        for (int i = 0; i < 8; i++) {
            if (sparseGrid.find(keys[i]) != sparseGrid.end()) {
                hasAnyVertex = true;
                break;
            }
        }
        
        // Если нет ни одной вершины - не обучаемся на малых силах
        if (!hasAnyVertex) {
            return;
        }
    }
    
    // Получаем относительную ориентацию
    SP_Math::Quaternion relativeQuat = getRelativeOrientation(paddleQuat, kayakQuat);
    
    // Извлекаем углы
    float shaftRotation, shaftTilt, bladeRotation;
    getPaddleAngles(relativeQuat, shaftRotation, shaftTilt, bladeRotation);
    
    // Обновляем сетку
    updateGrid(shaftRotation, shaftTilt, bladeRotation, currentForce, isForward);
}

float StrokePredictorGrid::predict(
    const SP_Math::Quaternion& paddleQuat,
    const SP_Math::Quaternion& kayakQuat,
    const SP_Math::Vector& angularVelocity,
    float deltaT,
    bool isForward,
    bool useAcceleration,
    float* confidence
) {
    // Вычисляем угловое ускорение если нужно
    SP_Math::Vector angularAcceleration(0, 0, 0);
    if (useAcceleration && omegaHistoryFilled) {
        angularAcceleration = calculateAngularAcceleration();
    }
    
    // Экстраполируем ориентацию весла на deltaT вперед
    SP_Math::Quaternion predictedPaddleQuat = extrapolateOrientation(
        paddleQuat, angularVelocity, angularAcceleration, 
        deltaT / 1000.0f, useAcceleration
    );
    
    // Получаем относительную ориентацию предсказанного положения
    SP_Math::Quaternion predictedRelativeQuat = getRelativeOrientation(
        predictedPaddleQuat, kayakQuat
    );
    
    // Извлекаем углы из предсказанного положения
    float shaftRotation, shaftTilt, bladeRotation;
    getPaddleAngles(predictedRelativeQuat, shaftRotation, shaftTilt, bladeRotation);
    
    // Интерполируем значение из сетки
    float outConfidence = 0.0f;
    float predictedForce = interpolateFromGrid(
        shaftRotation, shaftTilt, bladeRotation, 
        isForward, &outConfidence
    );
    
    // Возвращаем уверенность если запрошено
    if (confidence != nullptr) {
        *confidence = outConfidence;
    }
    
    return predictedForce;
}

//=============================================================================
// ИСТОРИЯ УГЛОВЫХ СКОРОСТЕЙ
//=============================================================================

void StrokePredictorGrid::updateOmegaHistory(const SP_Math::Vector& omega) {
    omegaHistory[omegaIndex] = omega;
    omegaTimestamps[omegaIndex] = millis();
    
    omegaIndex++;
    if (omegaIndex >= config.omegaHistorySize) {
        omegaIndex = 0;
        omegaHistoryFilled = true;
    }
}

SP_Math::Vector StrokePredictorGrid::calculateAngularAcceleration() const {
    if (!omegaHistoryFilled && omegaIndex < 2) {
        return SP_Math::Vector(0, 0, 0);
    }
    
    // Используем линейную регрессию по последним 5 точкам
    // alpha = d(omega)/dt
    
    int numSamples = omegaHistoryFilled ? config.omegaHistorySize : omegaIndex;
    if (numSamples < 2) {
        return SP_Math::Vector(0, 0, 0);
    }
    
    // Простой метод: разница между последним и первым
    int lastIdx = (omegaIndex - 1 + config.omegaHistorySize) % config.omegaHistorySize;
    int firstIdx = (omegaIndex) % config.omegaHistorySize;
    
    SP_Math::Vector deltaOmega = omegaHistory[lastIdx] - omegaHistory[firstIdx];
    float deltaTime = (omegaTimestamps[lastIdx] - omegaTimestamps[firstIdx]) / 1000.0f; // в секундах
    
    if (deltaTime > 0.001f) {
        return deltaOmega / deltaTime;
    }
    
    return SP_Math::Vector(0, 0, 0);
}

//=============================================================================
// ЭКСТРАПОЛЯЦИЯ ОРИЕНТАЦИИ
//=============================================================================

SP_Math::Quaternion StrokePredictorGrid::extrapolateOrientation(
    const SP_Math::Quaternion& currentQuat,
    const SP_Math::Vector& omega,
    const SP_Math::Vector& alpha,
    float deltaT,
    bool useAcceleration
) const {
    // Вычисляем угловую скорость в будущем (если используем ускорение)
    SP_Math::Vector omegaFuture = omega;
    if (useAcceleration) {
        omegaFuture = omega + alpha * deltaT;
    }
    
    // Вычисляем угол поворота
    float angleChange = omegaFuture.length() * deltaT;
    
    // Если угол слишком мал - возвращаем текущий кватернион
    if (angleChange < 1e-6f) {
        return currentQuat;
    }
    
    // Нормализованная ось вращения
    SP_Math::Vector axis = omegaFuture / omegaFuture.length();
    
    // Создаем кватернион поворота через экспоненциальную карту
    SP_Math::Quaternion deltaQ = SP_Math::Quaternion::fromAxisAngle(axis, angleChange);
    
    // Применяем поворот
    SP_Math::Quaternion predictedQuat = currentQuat * deltaQ;
    
    return predictedQuat.normalize();
}

//=============================================================================
// ИЗВЛЕЧЕНИЕ УГЛОВ
//=============================================================================

void StrokePredictorGrid::getPaddleAngles(
    const SP_Math::Quaternion& relativePaddleQ,
    float& shaftRotationAngle,
    float& shaftTiltAngle,
    float& bladeRotationAngle
) const {
    SP_Math::Vector paddleY(0, 1, 0);  // ось шафта
    SP_Math::Vector globalZ(0, 0, 1);  // вертикальный вектор
    
    // Поворачиваем векторы с использованием относительной ориентации
    SP_Math::Vector currentShaftDir = relativePaddleQ.rotate(paddleY);
    SP_Math::Vector currentZinPaddle = relativePaddleQ.conjugate().rotate(globalZ);
    
    // Угол поворота шафта вокруг Z (в плоскости XY)
    SP_Math::Vector sideDir(0, 1, 0);
    
    float cosAngle = sideDir.dot(currentShaftDir);
    float crossZ = sideDir.x() * currentShaftDir.y() - sideDir.y() * currentShaftDir.x();
    
    shaftRotationAngle = atan2(crossZ, cosAngle) * 57.295779513082320876798154814105f; // RAD_TO_DEG
    
    // Угол наклона шафта относительно плоскости XY
    shaftTiltAngle = asin(currentShaftDir.z()) * 57.295779513082320876798154814105f;
    
    // Угол поворота лопасти вокруг оси шафта
    bladeRotationAngle = atan2(currentZinPaddle.x(), currentZinPaddle.z()) * 57.295779513082320876798154814105f;
}

SP_Math::Quaternion StrokePredictorGrid::getRelativeOrientation(
    const SP_Math::Quaternion& paddleQuat,
    const SP_Math::Quaternion& kayakQuat
) const {
    // Получаем направление оси X каяка в мировой системе координат
    SP_Math::Vector x_k = kayakQuat.rotate(SP_Math::Vector(1, 0, 0));
    
    // Проецируем на горизонтальную плоскость (убираем наклон каяка)
    float norm_sq = x_k[0] * x_k[0] + x_k[1] * x_k[1];
    
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
    
    // Вычисляем ориентацию весла относительно горизонтального направления каяка
    SP_Math::Quaternion q_relative = q_new.conjugate() * paddleQuat;
    
    return q_relative.normalize();
}

//=============================================================================
// РАБОТА С СЕТКОЙ
//=============================================================================

uint32_t StrokePredictorGrid::computeGridKey(int idxRot, int idxTilt, int idxBlade) const {
    // Ключ: (idxRot * 18 + idxTilt) * 36 + idxBlade
    // Диапазон: 0..23327
    return (uint32_t)((idxRot * 18 + idxTilt) * 36 + idxBlade);
}

void StrokePredictorGrid::angleToIndices(
    float shaftRotation,
    float shaftTilt,
    float bladeRotation,
    int& idxRot,
    int& idxTilt,
    int& idxBlade,
    float& fracRot,
    float& fracTilt,
    float& fracBlade
) const {
    // Квантуем углы в индексы
    float step = config.gridStep;
    
    // Rotation: -180..+180 -> 0..35
    float rotNormalized = shaftRotation + 180.0f;
    idxRot = (int)(rotNormalized / step);
    fracRot = (rotNormalized - idxRot * step) / step;
    
    // Tilt: -90..+90 -> 0..17
    float tiltNormalized = shaftTilt + 90.0f;
    idxTilt = (int)(tiltNormalized / step);
    fracTilt = (tiltNormalized - idxTilt * step) / step;
    
    // Blade: -180..+180 -> 0..35
    float bladeNormalized = bladeRotation + 180.0f;
    idxBlade = (int)(bladeNormalized / step);
    fracBlade = (bladeNormalized - idxBlade * step) / step;
    
    // Ограничиваем индексы
    if (idxRot < 0) idxRot = 0;
    if (idxRot > 35) idxRot = 35;
    if (idxTilt < 0) idxTilt = 0;
    if (idxTilt > 17) idxTilt = 17;
    if (idxBlade < 0) idxBlade = 0;
    if (idxBlade > 35) idxBlade = 35;
    
    // Ограничиваем дробные части
    if (fracRot < 0) fracRot = 0;
    if (fracRot > 1) fracRot = 1;
    if (fracTilt < 0) fracTilt = 0;
    if (fracTilt > 1) fracTilt = 1;
    if (fracBlade < 0) fracBlade = 0;
    if (fracBlade > 1) fracBlade = 1;
}

int StrokePredictorGrid::wrapAngleIndex(int idx, int maxIdx) const {
    if (idx < 0) return idx + maxIdx;
    if (idx >= maxIdx) return idx - maxIdx;
    return idx;
}

void StrokePredictorGrid::get8NeighborKeys(
    int idxRot, int idxTilt, int idxBlade,
    uint32_t keys[8]
) const {
    // 8 вершин куба
    // Для rotation и blade используем wrap (циклические)
    // Для tilt НЕ используем wrap (не циклический)
    
    int idxRot1 = wrapAngleIndex(idxRot + 1, 36);
    int idxTilt1 = idxTilt + 1;
    if (idxTilt1 > 17) idxTilt1 = 17; // Ограничиваем, не wrap
    int idxBlade1 = wrapAngleIndex(idxBlade + 1, 36);
    
    keys[0] = computeGridKey(idxRot,  idxTilt,  idxBlade);
    keys[1] = computeGridKey(idxRot1, idxTilt,  idxBlade);
    keys[2] = computeGridKey(idxRot,  idxTilt1, idxBlade);
    keys[3] = computeGridKey(idxRot1, idxTilt1, idxBlade);
    keys[4] = computeGridKey(idxRot,  idxTilt,  idxBlade1);
    keys[5] = computeGridKey(idxRot1, idxTilt,  idxBlade1);
    keys[6] = computeGridKey(idxRot,  idxTilt1, idxBlade1);
    keys[7] = computeGridKey(idxRot1, idxTilt1, idxBlade1);
}

float StrokePredictorGrid::getTrilinearWeight(float fx, float fy, float fz, int cornerIdx) const {
    // Вес для trilinear интерполяции
    // cornerIdx: 0-7 соответствует 8 вершинам куба
    // fx, fy, fz: дробные части [0..1]
    
    float wx = (cornerIdx & 1) ? fx : (1.0f - fx);
    float wy = (cornerIdx & 2) ? fy : (1.0f - fy);
    float wz = (cornerIdx & 4) ? fz : (1.0f - fz);
    
    return wx * wy * wz;
}

float StrokePredictorGrid::interpolateFromGrid(
    float shaftRotation,
    float shaftTilt,
    float bladeRotation,
    bool isForward,
    float* outConfidence
) const {
    // Получаем индексы и дробные части
    int idxRot, idxTilt, idxBlade;
    float fracRot, fracTilt, fracBlade;
    angleToIndices(shaftRotation, shaftTilt, bladeRotation,
                  idxRot, idxTilt, idxBlade,
                  fracRot, fracTilt, fracBlade);
    
    // Получаем 8 ключей соседних вершин
    uint32_t keys[8];
    get8NeighborKeys(idxRot, idxTilt, idxBlade, keys);
    
    // Интерполируем по заполненным вершинам
    float totalWeight = 0.0f;
    float interpolatedValue = 0.0f;
    float totalConfidence = 0.0f;
    int filledVertices = 0;
    
    for (int i = 0; i < 8; i++) {
        auto it = sparseGrid.find(keys[i]);
        if (it != sparseGrid.end() && it->second.confidence > 0) {
            float weight = getTrilinearWeight(fracRot, fracTilt, fracBlade, i);
            float value = isForward ? it->second.force_forward : it->second.force_backward;
            
            interpolatedValue += value * weight;
            totalWeight += weight;
            totalConfidence += it->second.confidence * weight;
            filledVertices++;
        }
    }
    
    // Проверяем достаточность данных
    if (totalWeight > 0.5f) {
        if (outConfidence != nullptr) {
            *outConfidence = totalConfidence / totalWeight / 100.0f; // Нормализуем
            if (*outConfidence > 1.0f) *outConfidence = 1.0f;
        }
        return interpolatedValue / totalWeight;
    }
    
    // Недостаточно данных
    if (outConfidence != nullptr) {
        *outConfidence = 0.0f;
    }
    return 0.0f;
}

void StrokePredictorGrid::updateGrid(
    float shaftRotation,
    float shaftTilt,
    float bladeRotation,
    float forceValue,
    bool isForward
) {
    // Получаем индексы и дробные части
    int idxRot, idxTilt, idxBlade;
    float fracRot, fracTilt, fracBlade;
    angleToIndices(shaftRotation, shaftTilt, bladeRotation,
                  idxRot, idxTilt, idxBlade,
                  fracRot, fracTilt, fracBlade);
    
    if (learningMethod == LearningMethod::METHOD_A) {
        // Метод A: Корректировка по ошибке
        // Сначала получаем предсказание
        float predictedForce = interpolateFromGrid(shaftRotation, shaftTilt, bladeRotation, isForward);
        updateGridMethodA(idxRot, idxTilt, idxBlade, fracRot, fracTilt, fracBlade,
                         predictedForce, forceValue, isForward);
    } else {
        // Метод B: Прямое обновление
        updateGridMethodB(idxRot, idxTilt, idxBlade, fracRot, fracTilt, fracBlade,
                         forceValue, isForward);
    }
}

void StrokePredictorGrid::updateGridMethodA(
    int idxRot, int idxTilt, int idxBlade,
    float fracRot, float fracTilt, float fracBlade,
    float predictedForce,
    float realForce,
    bool isForward
) {
    // Вычисляем ошибку
    float error = realForce - predictedForce;
    
    // Получаем 8 ключей
    uint32_t keys[8];
    get8NeighborKeys(idxRot, idxTilt, idxBlade, keys);
    
    // Обновляем все 8 вершин пропорционально их весу
    for (int i = 0; i < 8; i++) {
        float weight = getTrilinearWeight(fracRot, fracTilt, fracBlade, i);
        float correction = error * weight * config.alphaEMA;
        
        // Получаем или создаем ячейку
        GridCell& cell = sparseGrid[keys[i]];
        
        if (cell.confidence == 0) {
            // Первое обновление - устанавливаем значение напрямую
            if (isForward) {
                cell.force_forward = realForce;
            } else {
                cell.force_backward = realForce;
            }
        } else {
            // Применяем коррекцию
            if (isForward) {
                cell.force_forward += correction;
            } else {
                cell.force_backward += correction;
            }
        }
        
        cell.confidence++;
    }
}

void StrokePredictorGrid::updateGridMethodB(
    int idxRot, int idxTilt, int idxBlade,
    float fracRot, float fracTilt, float fracBlade,
    float realForce,
    bool isForward
) {
    // Получаем 8 ключей
    uint32_t keys[8];
    get8NeighborKeys(idxRot, idxTilt, idxBlade, keys);
    
    // Обновляем все 8 вершин взвешенным EMA
    for (int i = 0; i < 8; i++) {
        float weight = getTrilinearWeight(fracRot, fracTilt, fracBlade, i);
        
        // Получаем или создаем ячейку
        GridCell& cell = sparseGrid[keys[i]];
        
        if (cell.confidence == 0) {
            // Первое обновление - устанавливаем значение напрямую
            if (isForward) {
                cell.force_forward = realForce;
            } else {
                cell.force_backward = realForce;
            }
        } else {
            // Взвешенное EMA
            float alpha = config.alphaEMA * weight;
            if (isForward) {
                cell.force_forward = cell.force_forward * (1.0f - alpha) + realForce * alpha;
            } else {
                cell.force_backward = cell.force_backward * (1.0f - alpha) + realForce * alpha;
            }
        }
        
        cell.confidence++;
    }
}

//=============================================================================
// МЕТОДЫ ОТЛАДКИ
//=============================================================================

uint32_t StrokePredictorGrid::getFilledCellsCount() const {
    return sparseGrid.size();
}

float StrokePredictorGrid::getAverageConfidence() const {
    if (sparseGrid.empty()) {
        return 0.0f;
    }
    
    uint32_t totalConfidence = 0;
    for (const auto& pair : sparseGrid) {
        totalConfidence += pair.second.confidence;
    }
    
    return (float)totalConfidence / sparseGrid.size();
}

void StrokePredictorGrid::printGridStatistics() const {
    Serial.println("=== Grid Statistics ===");
    Serial.printf("Filled cells: %u\n", getFilledCellsCount());
    Serial.printf("Average confidence: %.2f\n", getAverageConfidence());
    Serial.printf("Grid step: %.1f degrees\n", config.gridStep);
    Serial.printf("Alpha EMA: %.2f\n", config.alphaEMA);
    Serial.printf("Force threshold: %.1f\n", config.forceThreshold);
    Serial.printf("Learning method: %s\n", 
                 learningMethod == LearningMethod::METHOD_A ? "METHOD_A" : "METHOD_B");
    
    // Статистика по заполненности
    if (!sparseGrid.empty()) {
        float minForward = 1e9, maxForward = -1e9;
        float minBackward = 1e9, maxBackward = -1e9;
        uint32_t minConf = UINT32_MAX, maxConf = 0;
        
        for (const auto& pair : sparseGrid) {
            const GridCell& cell = pair.second;
            if (cell.force_forward < minForward) minForward = cell.force_forward;
            if (cell.force_forward > maxForward) maxForward = cell.force_forward;
            if (cell.force_backward < minBackward) minBackward = cell.force_backward;
            if (cell.force_backward > maxBackward) maxBackward = cell.force_backward;
            if (cell.confidence < minConf) minConf = cell.confidence;
            if (cell.confidence > maxConf) maxConf = cell.confidence;
        }
        
        Serial.printf("Force forward range: %.1f .. %.1f\n", minForward, maxForward);
        Serial.printf("Force backward range: %.1f .. %.1f\n", minBackward, maxBackward);
        Serial.printf("Confidence range: %u .. %u\n", minConf, maxConf);
    }
    Serial.println("=======================");
}

void StrokePredictorGrid::clearGrid() {
    sparseGrid.clear();
    Serial.println("Grid cleared!");
}

