#include "LoadCellCalibrator.h"

LoadCellCalibrator::LoadCellCalibrator(float paddleLength, float imuDistance, float bladeWeight, 
                                     float bladeCenter, float bladeMomentInertia)
    : paddleLength(paddleLength), 
      imuDistance(imuDistance),
      bladeWeight(bladeWeight), 
      bladeCenter(bladeCenter),
      bladeMomentInertia(bladeMomentInertia) {}

void LoadCellCalibrator::setPaddleParameters(float length, float imuDist, float weight, 
                                           float center, float momentInertia) {
    paddleLength = length;
    imuDistance = imuDist;
    bladeWeight = weight;
    bladeCenter = center;
    bladeMomentInertia = momentInertia;
}

float LoadCellCalibrator::calculateInertialEffects(const IMUData& imuData,
                                                 const BladeOrientation& bladeOrientation,
                                                 bool isRightBlade) const {
    // Функция рассчитывает инерциальные силы в граммах, действующие на лопасть
    // в направлении нормали к лопасти от ускорений и вращений весла
    //
    // Входные единицы:
    // - bladeWeight: килограммы
    // - imuData.ax/ay/az: м/с² (линейные ускорения)
    // - imuData.gx/gy/gz: рад/с (угловые скорости)
    // - leverArm: метры (расстояние от IMU до центра лопасти)
    //
    // ВРЕМЕННО ОТКЛЮЧЕНО ДЛЯ ОТЛАДКИ
//    return 0.0f;
    
    // ИСХОДНЫЙ КОД:
    float leverArm = isRightBlade ? 
        (paddleLength/2 - imuDistance - bladeCenter)*bladeOrientation.YAxisDirection : 
        (-paddleLength/2 - imuDistance + bladeCenter)*bladeOrientation.YAxisDirection;

    // Получаем нормаль лопасти
    const float* bladeNormal = isRightBlade ? 
        bladeOrientation.rightBladeVector : 
        bladeOrientation.leftBladeVector;

    // Проекция ускорения на нормаль лопасти (скалярное произведение)
    float normalAcceleration = imuData.ax * bladeNormal[0] + 
                              imuData.ay * bladeNormal[1] + 
                              imuData.az * bladeNormal[2];
    
    // Инерциальная сила направлена противоположно ускорению (F = -ma)
    float linearForce = -bladeWeight * normalAcceleration;

    float centrifugalForce = 0;


/*
    // Центробежная сила от вращения
    float angularVelocity[3] = {imuData.gx, imuData.gy, imuData.gz};
    
    // Полная угловая скорость (модуль)
    float omegaSquared = angularVelocity[0] * angularVelocity[0] + 
                        angularVelocity[1] * angularVelocity[1] + 
                        angularVelocity[2] * angularVelocity[2];
    
    // Центробежная сила направлена радиально от оси вращения
    // F = m * ω² * r, где r - расстояние от оси вращения
    float centrifugalForceMagnitude = bladeWeight * omegaSquared * fabs(leverArm);
    
    // Проекция центробежной силы на нормаль лопасти (упрощенно)
    // Предполагаем, что центробежная сила в основном перпендикулярна веслу
    float centrifugalForce = centrifugalForceMagnitude * 0.1f; // малая компонента
    */

    // Тангенциальное ускорение от изменения угловой скорости
    static float prevAngularVelocity[3] = {0, 0, 0};
    static uint32_t prevTime = 0;
    uint32_t currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0f;

    float tangentialForce = 0;
    if (dt > 0) {
        float angularAcceleration[3] = {
            (imuData.gx - prevAngularVelocity[0]) / dt,
            (imuData.gy - prevAngularVelocity[1]) / dt,
            (imuData.gz - prevAngularVelocity[2]) / dt
        };

        // Проекция углового ускорения на нормаль лопасти
        float normalAngularAcceleration = angularAcceleration[0] * bladeNormal[0] + 
                                        angularAcceleration[1] * bladeNormal[1] + 
                                        angularAcceleration[2] * bladeNormal[2];
        
        // Тангенциальная сила от углового ускорения: F = m * α * r
        // Но здесь используется момент инерции, поэтому: F = (I * α) / r
        tangentialForce = -bladeMomentInertia * normalAngularAcceleration / fabs(leverArm);

        prevAngularVelocity[0] = imuData.gx;
        prevAngularVelocity[1] = imuData.gy;
        prevAngularVelocity[2] = imuData.gz;

        prevTime = currentTime;
    }

    // Суммарная сила
    float totalForce = linearForce + centrifugalForce + tangentialForce;
//    Serial.printf("%s Linear force: %d, Centrifugal force: %d, Tangential force: %d, Total force: %d\n", isRightBlade ? "Right" : "Left", (int)(linearForce*1000), (int)(centrifugalForce*1000), (int)(tangentialForce*1000), (int)(totalForce*1000));

    // Отладочная информация (можно закомментировать для производительности)
    /*
    Serial.printf("Inertial Forces [%s blade]: Linear=%.2f, Centrifugal=%.2f, Tangential=%.2f, Total=%.2f g\n",
                  isRightBlade ? "Right" : "Left", 
                  linearForce, centrifugalForce, tangentialForce, totalForce);
    */

    // Возвращаем суммарную силу в граммах (проекция уже учтена в расчетах выше)
    return totalForce * 1000/9.81;
    
}

float LoadCellCalibrator::calculateGyroscopicEffect(const IMUData& imuData,
                                                  const BladeOrientation& bladeOrientation,
                                                  bool isRightBlade) const {
    // ВРЕМЕННО ОТКЛЮЧЕНО ДЛЯ ОТЛАДКИ
    return 0.0f;
    
    // ИСХОДНЫЙ КОД:
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
        (paddleLength/2 - imuDistance - bladeCenter)*bladeOrientation.YAxisDirection : 
        (-paddleLength/2 - imuDistance + bladeCenter)*bladeOrientation.YAxisDirection;
        
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
//        Serial.printf("Right tare\n");
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
//    float inertialEffects = calculateInertialEffects(imuData, bladeOrientation, isRightBlade);
//    float gyroscopicEffect = calculateGyroscopicEffect(imuData, bladeOrientation, isRightBlade);
    double tare = isRightBlade ? rightTare.average : leftTare.average;
    
    return rawForce - tare;// - inertialEffects - gyroscopicEffect;
} 