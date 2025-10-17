#ifndef STROKEPREDICTORGRID_H
#define STROKEPREDICTORGRID_H

#include "Arduino.h"
#include "SP_Quaternion.h"
#include "SP_Vector.h"
#include <unordered_map>

// Конфигурация сетки
struct GridConfig {
    float gridStep = 10.0f;              // Шаг сетки в градусах
    float alphaEMA = 0.2f;               // Коэффициент EMA
    float forceThreshold = 400.0f;       // Порог силы для обучения
    float predictionTime = 200.0f;       // Время предсказания по умолчанию (мс)
    uint8_t omegaHistorySize = 5;        // Размер истории для вычисления ускорения
};

// Ячейка сетки
struct GridCell {
    float force_forward;      // Усилие для гребка вперед
    float force_backward;     // Усилие для гребка назад
    uint16_t confidence;      // Количество обновлений
    
    GridCell() : force_forward(0.0f), force_backward(0.0f), confidence(0) {}
};

// Методы обучения
enum class LearningMethod {
    METHOD_A,  // Корректировка по ошибке
    METHOD_B   // Прямое взвешенное обновление
};

class StrokePredictorGrid {
public:
    StrokePredictorGrid();
    ~StrokePredictorGrid();
    
    // Основные функции
    void learn(
        const SP_Math::Quaternion& paddleQuat,        // Текущая ориентация весла
        const SP_Math::Quaternion& kayakQuat,         // Текущая ориентация каяка
        const SP_Math::Vector& angularVelocity,       // Угловая скорость (рад/с)
        float currentForce,                            // Текущая сила на тензодатчиках
        bool isForward                                 // Направление гребка
    );
    
    float predict(
        const SP_Math::Quaternion& paddleQuat,        // Текущая ориентация весла
        const SP_Math::Quaternion& kayakQuat,         // Текущая ориентация каяка
        const SP_Math::Vector& angularVelocity,       // Угловая скорость
        float deltaT,                                  // Время предсказания (мс)
        bool isForward,                                // Направление гребка
        bool useAcceleration = true,                   // Использовать ли ускорение
        float* confidence = nullptr                    // Выходной параметр уверенности
    );
    
    // Конфигурация
    GridConfig config;
    LearningMethod learningMethod = LearningMethod::METHOD_B;
    
    // Методы отладки
    uint32_t getFilledCellsCount() const;
    float getAverageConfidence() const;
    void printGridStatistics() const;
    void clearGrid();
    
private:
    // Разреженная сетка
    std::unordered_map<uint32_t, GridCell> sparseGrid;
    
    // История угловых скоростей
    SP_Math::Vector omegaHistory[5];
    uint32_t omegaTimestamps[5];
    uint8_t omegaIndex;
    bool omegaHistoryFilled;
    
    // Вспомогательные методы
    void updateOmegaHistory(const SP_Math::Vector& omega);
    SP_Math::Vector calculateAngularAcceleration() const;
    
    SP_Math::Quaternion extrapolateOrientation(
        const SP_Math::Quaternion& currentQuat,
        const SP_Math::Vector& omega,
        const SP_Math::Vector& alpha,
        float deltaT,
        bool useAcceleration
    ) const;
    
    void getPaddleAngles(
        const SP_Math::Quaternion& relativePaddleQ,
        float& shaftRotationAngle,
        float& shaftTiltAngle,
        float& bladeRotationAngle
    ) const;
    
    SP_Math::Quaternion getRelativeOrientation(
        const SP_Math::Quaternion& paddleQuat,
        const SP_Math::Quaternion& kayakQuat
    ) const;
    
    // Работа с сеткой
    uint32_t computeGridKey(int idxRot, int idxTilt, int idxBlade) const;
    void angleToIndices(
        float shaftRotation,
        float shaftTilt,
        float bladeRotation,
        int& idxRot,
        int& idxTilt,
        int& idxBlade,
        float& fracRot,
        float& fracTilt,
        float& fracBlade
    ) const;
    
    int wrapAngleIndex(int idx, int maxIdx) const;
    
    float interpolateFromGrid(
        float shaftRotation,
        float shaftTilt,
        float bladeRotation,
        bool isForward,
        float* outConfidence = nullptr
    ) const;
    
    void updateGrid(
        float shaftRotation,
        float shaftTilt,
        float bladeRotation,
        float forceValue,
        bool isForward
    );
    
    void updateGridMethodA(
        int idxRot, int idxTilt, int idxBlade,
        float fracRot, float fracTilt, float fracBlade,
        float predictedForce,
        float realForce,
        bool isForward
    );
    
    void updateGridMethodB(
        int idxRot, int idxTilt, int idxBlade,
        float fracRot, float fracTilt, float fracBlade,
        float realForce,
        bool isForward
    );
    
    float getTrilinearWeight(float fx, float fy, float fz, int cornerIdx) const;
    void get8NeighborKeys(
        int idxRot, int idxTilt, int idxBlade,
        uint32_t keys[8]
    ) const;
};

#endif

