#include "LoadCellCalibrator.h"

LoadCellCalibrator::LoadCellCalibrator(float paddleLength, float bladeWeight, 
                                     float bladeCenter, float bladeMomentInertia)
    : paddleLength(paddleLength), 
      bladeWeight(bladeWeight), 
      bladeCenter(bladeCenter),
      bladeMomentInertia(bladeMomentInertia) {}

void LoadCellCalibrator::setPaddleParameters(float length, float weight, 
                                           float center, float momentInertia) {
    paddleLength = length;
    bladeWeight = weight;
    bladeCenter = center;
    bladeMomentInertia = momentInertia;
}

float LoadCellCalibrator::calculateInertialEffects(const IMUData& imuData,
                                                 const BladeOrientation& bladeOrientation,
                                                 bool isRightBlade) const {
    float leverArm = isRightBlade ? 
        (paddleLength/2 - bladeCenter) : 
        (-paddleLength/2 + bladeCenter);

    // Получаем нормаль лопасти
    const float* bladeNormal = isRightBlade ? 
        bladeOrientation.rightBladeVector : 
        bladeOrientation.leftBladeVector;

    // Линейные ускорения
    float linearForce = bladeWeight * 
        sqrt(imuData.ax * imuData.ax + 
             imuData.ay * imuData.ay + 
             imuData.az * imuData.az);

    // Центробежная сила от вращения
    float angularVelocity[3] = {imuData.gx, imuData.gy, imuData.gz};
    float centrifugalForce = bladeWeight * 
        (angularVelocity[0] * angularVelocity[0] + 
         angularVelocity[1] * angularVelocity[1] + 
         angularVelocity[2] * angularVelocity[2]) * leverArm;

    // Тангенциальное ускорение от изменения угловой скорости
    static float prevAngularVelocity[3] = {0, 0, 0};
    static uint32_t prevTime = 0;
    uint32_t currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0f;

    float tangentialForce = 0;
    if (dt > 0) {
        float angularAcceleration[3] = {
            (angularVelocity[0] - prevAngularVelocity[0]) / dt,
            (angularVelocity[1] - prevAngularVelocity[1]) / dt,
            (angularVelocity[2] - prevAngularVelocity[2]) / dt
        };

        tangentialForce = bladeMomentInertia * 
            sqrt(angularAcceleration[0] * angularAcceleration[0] +
                 angularAcceleration[1] * angularAcceleration[1] +
                 angularAcceleration[2] * angularAcceleration[2]) / leverArm;

        memcpy(prevAngularVelocity, angularVelocity, sizeof(prevAngularVelocity));
        prevTime = currentTime;
    }

    // Суммарная сила
    float totalForce = linearForce + centrifugalForce + tangentialForce;

    // Проекция на направление измерения
    float projectionFactor = bladeNormal[0] * leverArm / 
        sqrt(leverArm * leverArm + bladeNormal[1] * bladeNormal[1]);

    return totalForce * projectionFactor;
}

float LoadCellCalibrator::calculateGyroscopicEffect(const IMUData& imuData,
                                                  const BladeOrientation& bladeOrientation,
                                                  bool isRightBlade) const {
    // Гироскопический момент возникает при пересечении осей вращения
    float angularVelocity[3] = {imuData.gx, imuData.gy, imuData.gz};
    
    // Момент импульса вдоль оси весла (Y)
    float angularMomentumY = bladeMomentInertia * angularVelocity[1];
    
    // Поперечная угловая скорость (в плоскости XZ)
    float transverseOmega = sqrt(angularVelocity[0] * angularVelocity[0] + 
                                angularVelocity[2] * angularVelocity[2]);
    
    // Гироскопический момент пропорционален векторному произведению
    float gyroscopicMoment = angularMomentumY * transverseOmega;
    
    float leverArm = isRightBlade ? 
        (paddleLength/2 - bladeCenter) : 
        (-paddleLength/2 + bladeCenter);
        
    return gyroscopicMoment / leverArm;
}

void LoadCellCalibrator::updateTare(bool isRightBlade, double leftForce, double rightForce,
                                  const IMUData& imuData, const BladeOrientation& bladeOrientation) {
    if (isRightBlade) {
        leftTare.samples++;
        float compensation = calculateInertialEffects(imuData, bladeOrientation, false) +
                           calculateGyroscopicEffect(imuData, bladeOrientation, false);
        leftTare.sum += (leftForce - compensation);
        
        if (leftTare.samples > SAMPLES_THRESHOLD) {
            double avg = leftTare.sum / leftTare.samples;
            leftTare.average = leftTare.average * (1 - ALPHA_LEFT) + avg * ALPHA_LEFT;
            leftTare.samples = 0;
            leftTare.sum = 0;
        }
    } else {
        rightTare.samples++;
        float compensation = calculateInertialEffects(imuData, bladeOrientation, true) +
                           calculateGyroscopicEffect(imuData, bladeOrientation, true);
        rightTare.sum += (rightForce - compensation);
        
        if (rightTare.samples > SAMPLES_THRESHOLD) {
            double avg = rightTare.sum / rightTare.samples;
            rightTare.average = rightTare.average * (1 - ALPHA_RIGHT) + avg * ALPHA_RIGHT;
            rightTare.samples = 0;
            rightTare.sum = 0;
        }
    }
}

double LoadCellCalibrator::getCalibratedForce(bool isRightBlade, double rawForce,
                                            const IMUData& imuData,
                                            const BladeOrientation& bladeOrientation) const {
    float inertialEffects = calculateInertialEffects(imuData, bladeOrientation, isRightBlade);
    float gyroscopicEffect = calculateGyroscopicEffect(imuData, bladeOrientation, isRightBlade);
    double tare = isRightBlade ? rightTare.average : leftTare.average;
    
    return rawForce - tare - inertialEffects - gyroscopicEffect;
} 