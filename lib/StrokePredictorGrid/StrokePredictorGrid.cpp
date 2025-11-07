#include "StrokePredictorGrid.h"
#include "OrientationUtils.h"
#include <cmath>

#ifdef ARDUINO
#include "Arduino.h"
#else
#define Serial stderr
#endif

StrokePredictorGrid::StrokePredictorGrid() 
    : lastLearningCellIdx{0, 0, 0}, 
      lastLearningCellValue(0),
      lastLearningCellFrac{0, 0, 0},
      learningCellTimes(1),
      lastLearningCellDirection(true) 
{

    sparseGrid.reserve(1500);

}

StrokePredictorGrid::~StrokePredictorGrid() {
    // Очистка не требуется - unordered_map очистится автоматически
}

//=============================================================================
// ОСНОВНЫЕ ФУНКЦИИ
//=============================================================================

void StrokePredictorGrid::learn(
    PaddleOrientationCalculator* relativeOrientation,
    float currentForce,
    bool isForward
) {

    // Извлекаем углы
    float shaftRotation, shaftTilt, bladeRotation;
    relativeOrientation->getPaddleAngles(shaftRotation, shaftTilt, bladeRotation);

    // Вычисляем индексы
    int idxRot, idxTilt, idxBlade;
    float fracRot, fracTilt, fracBlade;
    angleToIndices(shaftRotation, shaftTilt, bladeRotation, 
                    idxRot, idxTilt, idxBlade,
                    fracRot, fracTilt, fracBlade);

    // Проверяем, если та же ячейка, то просто обновляем обучаемое значение

    if (lastLearningCellIdx[0] == idxRot && lastLearningCellIdx[1] == idxTilt && lastLearningCellIdx[2] == idxBlade && lastLearningCellDirection == isForward) {
        lastLearningCellValue += currentForce;
        lastLearningCellFrac[0] += fracRot;
        lastLearningCellFrac[1] += fracTilt;
        lastLearningCellFrac[2] += fracBlade;
        learningCellTimes++;
        return;
    }

    // Обучаемся по последней обучаемой ячейке

    lastLearningCellFrac[0] /= learningCellTimes;
    lastLearningCellFrac[1] /= learningCellTimes;
    lastLearningCellFrac[2] /= learningCellTimes;
    lastLearningCellValue /= learningCellTimes;
//    Serial.printf("Learning cell: %d, %d, %d, Value: %f, Frac: %f, %f, %f\n", lastLearningCellIdx[0], lastLearningCellIdx[1], lastLearningCellIdx[2], lastLearningCellValue, lastLearningCellFrac[0], lastLearningCellFrac[1], lastLearningCellFrac[2]);

    if (learningMethod == LearningMethod::METHOD_A) {

        // Метод A: Корректировка по ошибке
        // Сначала получаем предсказание
        float predictedForce = interpolateFromGrid(shaftRotation, shaftTilt, bladeRotation, isForward);
        updateGridMethodA(lastLearningCellIdx[0], lastLearningCellIdx[1], lastLearningCellIdx[2], lastLearningCellFrac[0], lastLearningCellFrac[1], lastLearningCellFrac[2], predictedForce, lastLearningCellValue, isForward);
    } else {
        // Метод B: Прямое обновление
        updateGridMethodB(lastLearningCellIdx[0], lastLearningCellIdx[1], lastLearningCellIdx[2], lastLearningCellFrac[0], lastLearningCellFrac[1], lastLearningCellFrac[2], lastLearningCellValue, isForward);
    }

    // Обновляем последнюю обучаемую ячейку
    lastLearningCellIdx[0] = idxRot;
    lastLearningCellIdx[1] = idxTilt;
    lastLearningCellIdx[2] = idxBlade;
    lastLearningCellValue = currentForce;
    lastLearningCellFrac[0] = fracRot;
    lastLearningCellFrac[1] = fracTilt;
    lastLearningCellFrac[2] = fracBlade;
    learningCellTimes = 1;
}

float StrokePredictorGrid::predict(
    PaddleOrientationCalculator* relativeOrientation,
    float deltaT,
    bool isForward,
    float* confidence
) {

    // Экстраполируем ориентацию весла на deltaT вперед
    SP_Math::Quaternion predictedPaddleQuat = extrapolateOrientation(
        relativeOrientation->getPaddleRelativeOrientation(), 
        relativeOrientation->getAngularVelocity(),
        relativeOrientation->getAngularAcceleration(), 
        deltaT / 1000.0f, true
    );
    
    
    // Извлекаем углы из предсказанного положения
    float shaftRotation, shaftTilt, bladeRotation;
    relativeOrientation->getPaddleAngles(predictedPaddleQuat, shaftRotation, shaftTilt, bladeRotation);
    
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

    if (predictedForce > 0) {
//        Serial.printf("Predicted force: %f, Confidence: %f\n", predictedForce, outConfidence);
    }
    
    return predictedForce;
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
    SP_Math::Vector omegaAverage = omega;
    if (useAcceleration) {
        omegaAverage = omega + alpha * deltaT/2;
    }
    
    // Вычисляем угол поворота
    float omegaAverageLength = omegaAverage.length();
    float angleChange = omegaAverageLength * deltaT;
    
    // Если угол слишком мал - возвращаем текущий кватернион
    if (angleChange < 1e-6f) {
        return currentQuat;
    }
    
    // Нормализованная ось вращения
    SP_Math::Vector axis = omegaAverage / omegaAverageLength;
    
    // Создаем кватернион поворота через экспоненциальную карту
    SP_Math::Quaternion deltaQ = SP_Math::Quaternion::fromAxisAngle(axis, angleChange);
    
    // Применяем поворот
    SP_Math::Quaternion predictedQuat = currentQuat * deltaQ;
    
    return predictedQuat.normalize();
}

//=============================================================================
// ИЗВЛЕЧЕНИЕ УГЛОВ
//=============================================================================
/*
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

    // Вычисляем углы поворота шафта
    // Проекция оси шафта на плоскость XY каяка
    SP_Math::Vector shaftProjectionXY(currentShaftDir.x(), currentShaftDir.y(), 0);
    shaftProjectionXY.normalize();
    
    // Угол поворота шафта вокруг Z (в плоскости XY)
    SP_Math::Vector sideDir(0, 1, 0);
    
    float cosAngle = sideDir.dot(shaftProjectionXY);
    float crossZ = sideDir.x() * shaftProjectionXY.y() - sideDir.y() * shaftProjectionXY.x();
    
    shaftRotationAngle = atan2(crossZ, cosAngle) * 57.295779513082320876798154814105f; // RAD_TO_DEG
    
    // Угол наклона шафта относительно плоскости XY
    shaftTiltAngle = asin(currentShaftDir.z()) * 57.295779513082320876798154814105f;
    
    // Угол поворота лопасти вокруг оси шафта
    bladeRotationAngle = atan2(currentZinPaddle.x(), currentZinPaddle.z()) * 57.295779513082320876798154814105f;
}
    */


//=============================================================================
// РАБОТА С СЕТКОЙ
//=============================================================================

uint32_t StrokePredictorGrid::computeGridKey(int idxRot, int idxTilt, int idxBlade) const {
    // Ключ: (idxRot * 18 + idxTilt) * 36 + idxBlade
    // Диапазон: 0..23327
    if ((idxRot < 0) || (idxTilt < 0) || (idxBlade < 0)) {
        Serial.printf("idxRot: %d, idxTilt: %d, idxBlade: %d\n", idxRot, idxTilt, idxBlade);
        return 0;
    }
    if ((idxRot > 35) || (idxTilt > 17) || (idxBlade > 35)) {
        Serial.printf("idxRot: %d, idxTilt: %d, idxBlade: %d\n", idxRot, idxTilt, idxBlade);
        return 0;
    }
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
            *outConfidence = totalConfidence / totalWeight / 10.0f; // Нормализуем
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
    // Ограничиваем размер grid для предотвращения нехватки памяти
    const size_t MAX_GRID_SIZE = 2000;  // Максимум 2000 ячеек (~125KB памяти)
    
    // Вычисляем ошибку
    float error = realForce - predictedForce;
    
    // Получаем 8 ключей
    uint32_t keys[8];
    get8NeighborKeys(idxRot, idxTilt, idxBlade, keys);
    
    // Обновляем все 8 вершин пропорционально их весу
    for (int i = 0; i < 8; i++) {

        // Если сила меньше порога, то не обучаемся на малых силах если ячейка не существует
        if (fabs(realForce) < config.forceThreshold) {
            if (sparseGrid.find(keys[i]) == sparseGrid.end()) {
                continue;
            }
        }

        // Проверка переполнения grid перед созданием новой ячейки
        bool cellExists = (sparseGrid.find(keys[i]) != sparseGrid.end());
        if (!cellExists && sparseGrid.size() >= MAX_GRID_SIZE) {
            // Grid переполнен, новую ячейку не создаем
            continue;
        }

        float weight = getTrilinearWeight(fracRot, fracTilt, fracBlade, i);
        float correction = error * weight * config.alphaEMA;
        
        // Получаем или создаем ячейку (теперь безопасно)
        try {
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
            
        } catch (const std::bad_alloc& e) {
            // Нехватка памяти при создании ячейки grid
            Serial.printf("⚠️ Grid memory allocation failed (size:%u)\n", sparseGrid.size());
            break;  // Прерываем цикл обновления
        } catch (...) {
            Serial.println("⚠️ Unknown exception in grid update");
            break;
        }
    }
}

void StrokePredictorGrid::updateGridMethodB(
    int idxRot, int idxTilt, int idxBlade,
    float fracRot, float fracTilt, float fracBlade,
    float realForce,
    bool isForward
) {
    // Ограничиваем размер grid для предотвращения нехватки памяти
    const size_t MAX_GRID_SIZE = 800;  // Максимум 800 ячеек (~50KB памяти)
    
    // Получаем 8 ключей
    uint32_t keys[8];
    get8NeighborKeys(idxRot, idxTilt, idxBlade, keys);
    
    // Обновляем все 8 вершин взвешенным EMA
    for (int i = 0; i < 8; i++) {
        
        // Если сила меньше порога, то не обучаемся на малых силах если ячейка не существует
        if (fabs(realForce) < config.forceThreshold) {
            if (sparseGrid.find(keys[i]) == sparseGrid.end()) {
                continue;
            }
        }

        // Проверка переполнения grid перед созданием новой ячейки
        bool cellExists = (sparseGrid.find(keys[i]) != sparseGrid.end());
        if (!cellExists && sparseGrid.size() >= MAX_GRID_SIZE) {
            // Grid переполнен, новую ячейку не создаем
            continue;
        }

        // Получаем или создаем ячейку (теперь безопасно)
        try {
            GridCell& cell = sparseGrid[keys[i]];
        
            float weight = getTrilinearWeight(fracRot, fracTilt, fracBlade, i);

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
            
        } catch (const std::bad_alloc& e) {
            // Нехватка памяти при создании ячейки grid
            Serial.printf("⚠️ Grid memory allocation failed (size:%u)\n", sparseGrid.size());
            break;  // Прерываем цикл обновления
        } catch (...) {
            Serial.println("⚠️ Unknown exception in grid update");
            break;
        }
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

