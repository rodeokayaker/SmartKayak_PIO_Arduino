#include "AngularAccelerationCalculator.h"

AngularAccelerationCalculator::AngularAccelerationCalculator() : omegaIndex(0) {
    for (int i = 0; i < OMEGA_HISTORY_SIZE; i++) {
        omegaHistory[i] = SP_Math::Vector(0, 0, 0);
        omegaTimestamps[i] = 0;
    }
}

void AngularAccelerationCalculator::updateAngularVelocity(float gx, float gy, float gz, uint32_t timestamp) {
    omegaHistory[omegaIndex][0] = gx;
    omegaHistory[omegaIndex][1] = gy;
    omegaHistory[omegaIndex][2] = gz;
    omegaTimestamps[omegaIndex] = timestamp;
    omegaIndex++;
    if (omegaIndex >= OMEGA_HISTORY_SIZE) {
        omegaIndex = 0;
    }
}

void AngularAccelerationCalculator::updateAngularVelocity(const SP_Math::Vector& omega, uint32_t timestamp) {
    omegaHistory[omegaIndex] = omega;
    omegaTimestamps[omegaIndex] = timestamp;
    omegaIndex++;
    if (omegaIndex >= OMEGA_HISTORY_SIZE) {
        omegaIndex = 0;
    }
}

void AngularAccelerationCalculator::updateAngularVelocity(const IMUData& imuData) {
    updateAngularVelocity(SP_Math::Vector(imuData.gx, imuData.gy, imuData.gz), imuData.timestamp);
}

SP_Math::Vector AngularAccelerationCalculator::calculateAngularAcceleration() const {
        
    // Вычисляем среднее время
    float t_mean = 0;
    for (int i = 0; i < OMEGA_HISTORY_SIZE; i++) {
        t_mean += omegaTimestamps[i];
    }
    t_mean /= OMEGA_HISTORY_SIZE;
    
    SP_Math::Vector alpha(0, 0, 0);
    
    // Для каждой компоненты (x, y, z)
    for (int axis = 0; axis < 3; axis++) {
        float numerator = 0, denominator = 0;
        
        for (int i = 0; i < OMEGA_HISTORY_SIZE; i++) {
            float t_diff = (omegaTimestamps[i] - t_mean) / 1000.0f;
            numerator += t_diff * omegaHistory[i][axis];
            denominator += t_diff * t_diff;
        }
        
        if (denominator > 1e-10f) {
            alpha[axis] = numerator / denominator;
        }
    }
    
    return alpha;
}