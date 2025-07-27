#include "StrokePredictor.h"

StrokePredictor::StrokePredictor() {
    // Базовая инициализация
}

StrokePredictor::~StrokePredictor() {
    // Базовая очистка
}

float StrokePredictor::PredictForce(float shaft_z, float blade_z, BladeSideType bladeSide, 
                                   float current_force, float& coefPredict) {
    // Базовая реализация - просто возвращаем 0
    coefPredict = 0.0f;
    return 0.0f;
} 