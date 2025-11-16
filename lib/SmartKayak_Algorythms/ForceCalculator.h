#ifndef FORCECALCULATOR_H
#define FORCECALCULATOR_H

#include "../Core/Types.h"
#include "SP_Math.h"
#include "OrientationUtils.h"


float calculateForce()
{
    int force = 0;
    int borderLoadForce = 600;

    loadData loads = paddle->getLoadData();
    if (LOADCELL_SMOOTHING_FACTOR > 0) {
        currentLoadCellData.forceR = (currentLoadCellData.forceR * (1 - LOADCELL_SMOOTHING_FACTOR) + loads.forceR * LOADCELL_SMOOTHING_FACTOR);
        currentLoadCellData.forceL = (currentLoadCellData.forceL * (1 - LOADCELL_SMOOTHING_FACTOR) + loads.forceL * LOADCELL_SMOOTHING_FACTOR);
        currentLoadCellData.forceR_raw = (currentLoadCellData.forceR_raw * (1 - LOADCELL_SMOOTHING_FACTOR) + loads.forceR_raw * LOADCELL_SMOOTHING_FACTOR);
        currentLoadCellData.forceL_raw = (currentLoadCellData.forceL_raw * (1 - LOADCELL_SMOOTHING_FACTOR) + loads.forceL_raw * LOADCELL_SMOOTHING_FACTOR);
        currentLoadCellData.timestamp = loads.timestamp;
        loads = currentLoadCellData;
    } 

    OrientationData paddleOrientation = paddle->getOrientationData();
    IMUData paddleIMUData = paddle->getIMUData();
    if (paddleOrientation.q0 == 0) {
//        Serial.printf("Paddle orientation is not valid: %f, %f, %f, %f\n", paddleOrientation.q0, paddleOrientation.q1, paddleOrientation.q2, paddleOrientation.q3);
        return; 
    }

    SP_Math::Quaternion currentPaddleQ(paddleOrientation.q0,paddleOrientation.q1,paddleOrientation.q2,paddleOrientation.q3);
    SP_Math::Quaternion paddleRelativeQuat = getRelativeOrientation(currentPaddleQ,kayakOrientationQuat);
    //Determine which blade is lower
    BladeSideType bladeSide = getLowerBladeSide(currentPaddleQ, paddle->getSpecs().axisDirectionSign);
    currentBladeSide = bladeSide;

    if (bladeSide == BladeSideType::ALL_BLADES) {
        motorDriver->stop();
        return; 
    }

    // Обновляем калибровку с учетом IMU
    loadCellCalibrator.updateTare(
        (bladeSide == BladeSideType::RIGHT_BLADE),
        loads.forceL,
        loads.forceR,
        paddleIMUData,
        paddle->getBladeAngles(),
        paddle->getSpecs().axisDirectionSign
    );
    
    // Получаем откалиброванное значение силы с учетом гравитации
    float bladeForce = loadCellCalibrator.getCalibratedForce(
        (bladeSide == BladeSideType::RIGHT_BLADE),
        (bladeSide == BladeSideType::RIGHT_BLADE) ? loads.forceR : loads.forceL,
        paddleIMUData,
        paddle->getBladeAngles(),
        paddle->getSpecs().axisDirectionSign
    );
    
 
    SP_Math::Vector paddleNormal( 
        (bladeSide == BladeSideType::RIGHT_BLADE) ? 
        paddle->getBladeAngles().rightBladeVector : 
        paddle->getBladeAngles().leftBladeVector);
    

    SP_Math::Vector kayakPaddleCorrectedNormal = paddleRelativeQuat.rotate(paddleNormal);
     
    float shaftRotationAngle;  // поворот вокруг оси Z каяка
    float shaftTiltAngle;      // наклон вокруг оси X каяка
    float bladeRotationAngle;  // поворот вокруг оси Y весла
    getPaddleAngles(paddleRelativeQuat, shaftRotationAngle, shaftTiltAngle, bladeRotationAngle);

//    shaftRotationAngle = 0;
    int shaftRotationAngleInt = (int)shaftRotationAngle;
    int shaftTiltAngleInt = (int)shaftTiltAngle;
    int bladeRotationAngleInt = (int)bladeRotationAngle;

    kayakPaddleCorrectedNormal.normalize();
    float cosAngle = kayakPaddleCorrectedNormal.x();

    float fForce = bladeForce*cosAngle;
    float predictedForce = 0;
    float confidence = 0;

    if (predictorMode == 1 ) {
        SP_Math::Vector angularVelocity(
            paddleIMUData.gx,
            paddleIMUData.gy,
            paddleIMUData.gz
        );
        predictedForce = strokePredictor->predict(currentPaddleQ, kayakOrientationQuat, angularVelocity, strokePredictor->config.predictionTime, forceAdapter.goingForward(), true, &confidence);

        if (modeSwitch->getMode() != MOTOR_OFF) {
            // Обучаем предиктор только если мотор работает

            float forceToLearn = fForce;

            if (fabs(fForce) < strokePredictor->config.forceThreshold) {
                forceToLearn = 0;
            }

            strokePredictor->learn(currentPaddleQ, kayakOrientationQuat, angularVelocity, paddleIMUData.timestamp, forceToLearn, forceAdapter.goingForward());
        }
    } 

    if ((bladeForce < borderLoadForce) && (bladeForce > -borderLoadForce)) {
        fForce = 0;
    } else {
        predictedForceUsed = false;
    }
    
    usingPredictedForce = false;
    
    if (predictorMode == 1) {
        if (fForce == 0 && (!predictedForceUsed || (millis() - predictedTimeUsed < strokePredictor->config.predictionTime+100))) {

            if (fabs(predictedForce) > borderLoadForce && confidence > 0.5) {
                if (!predictedForceUsed) {
                    predictedTimeUsed = millis();
                    predictedForceUsed = true;
                }
                fForce = predictedForce;
                usingPredictedForce = true;
            }
        }
    }

    currentForceGramms = (int)fForce;

}

#endif