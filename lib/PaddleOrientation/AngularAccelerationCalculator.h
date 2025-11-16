#ifndef ANGULARACCELERATIONCALCULATOR_H
#define ANGULARACCELERATIONCALCULATOR_H

#include "../Math/SP_Vector.h"
#include "../Core/Types.h"

#define OMEGA_HISTORY_SIZE 4


class AngularAccelerationCalculator {
    private:

        // История угловых скоростей
        SP_Math::Vector omegaHistory[OMEGA_HISTORY_SIZE];
        uint32_t omegaTimestamps[OMEGA_HISTORY_SIZE];
        uint8_t omegaIndex;

    public:
        AngularAccelerationCalculator();

        void updateAngularVelocity(const SP_Math::Vector& omega, uint32_t timestamp);
        void updateAngularVelocity(const IMUData& imuData);
        void updateAngularVelocity(float gx, float gy, float gz, uint32_t timestamp);

        SP_Math::Vector calculateAngularAcceleration() const;
};

#endif