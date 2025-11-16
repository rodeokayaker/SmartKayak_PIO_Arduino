#include "PredictedPaddle.h"
#include "LoadCellCalibrator.h"
#include "StrokePredictorGrid.h"
#include "PaddleOrientationCalculator.h"
//#include "SP_CoordinateTransform.h"

PredictedPaddle::PredictedPaddle(SmartPaddle* sp) :  paddle(sp) {
    currentLoadCellData.forceR = 0;
    currentLoadCellData.forceL = 0;
    currentLoadCellData.forceR_raw = 0;
    currentLoadCellData.forceL_raw = 0;
    currentLoadCellData.timestamp = 0;

    loadCellCalibrator = new LoadCellCalibrator();
    strokePredictor = new StrokePredictorGrid();
    relativeOrientation = new PaddleOrientationCalculator(sp);
    sp->setEventHandler(this);
    calibrateLoads = false;
    if (sp->specsValid()) {
        onUpdateSpecs(sp->getSpecs(), sp);
    }
    if (sp->bladeOrientationValid()) {
        onUpdateBladeAngle(sp->getBladeAngles(), sp);
    }

    strokePredictor->config.gridStep = DEFAULT_GRID_STEP;
    strokePredictor->config.alphaEMA = DEFAULT_ALPHA_EMA;
    strokePredictor->config.forceThreshold = DEFAULT_FORCE_THRESHOLD;
    strokePredictor->config.predictionTime = DEFAULT_PREDICTION_TIME;
    strokePredictor->learningMethod = LearningMethod::METHOD_B;

}

PredictedPaddle::~PredictedPaddle() {

    if (paddle) {
        paddle->removeEventHandler(this);
    }
    delete loadCellCalibrator;
    delete strokePredictor;
    delete relativeOrientation;
}

void PredictedPaddle::onUpdateLoad(const loadData& loadData, SmartPaddle* paddle) {


    currentLoadCellData.forceR = (currentLoadCellData.forceR * (LOADCELL_SMOOTHING_FACTOR) + loadData.forceR * (1-LOADCELL_SMOOTHING_FACTOR));
    currentLoadCellData.forceL = (currentLoadCellData.forceL * (LOADCELL_SMOOTHING_FACTOR) + loadData.forceL * (1-LOADCELL_SMOOTHING_FACTOR));
    currentLoadCellData.forceR_raw = loadData.forceR_raw ;
    currentLoadCellData.forceL_raw = loadData.forceL_raw ;
    currentLoadCellData.timestamp = loadData.timestamp;


    if (calibrateLoads) {
        BladeSideType bladeSide = relativeOrientation->getLowerBladeSide();

        
        if ((bladeSide == RIGHT_BLADE && !paddle->getSpecs().hasLeftBlade)|| (bladeSide == LEFT_BLADE && !paddle->getSpecs().hasRightBlade)) {
//            Serial.printf("Calibrating SUP!");
            double shaftVectorZ = relativeOrientation->shaftVectorZ();
            if (fabs(shaftVectorZ) < 0.4) {
                loadCellCalibrator->updateTare(
                    bladeSide == LEFT_BLADE,
                    loadData,
                    paddle->getIMUData(),
                    relativeOrientation->getAngularAcceleration(),
                    paddle->getBladeAngles()
                );
            }
        } else {
            loadCellCalibrator->updateTare(
                bladeSide != LEFT_BLADE,
                loadData,
                paddle->getIMUData(),
                relativeOrientation->getAngularAcceleration(),
                paddle->getBladeAngles()
            );

        }
    }
}

void PredictedPaddle::onUpdateOrientation(const OrientationData& orientationData, SmartPaddle* paddle) {
            
        
    // Преобразование кватерниона ориентации в стандартную систему координат
//    SP_CoordinateTransform::transformOrientationData(orientationData, paddle->getSpecs().axisDirection);
    relativeOrientation->updatePaddleOrientation(orientationData);
}

void PredictedPaddle::onUpdateIMU(const IMUData& imuData, SmartPaddle* paddle) {
    // Преобразование координат IMU в стандартную систему (Y вдоль шафта)
    // независимо от реальной ориентации датчика
//    SP_CoordinateTransform::transformIMUData(imuData, paddle->getSpecs().axisDirection);
    relativeOrientation->updatePaddleIMU(imuData);

}

void PredictedPaddle::onUpdateBladeAngle(const BladeOrientation& bladeOrientation, SmartPaddle* paddle)
{
//    Serial.printf("PredictedPaddle::onUpdateBladeAngle: %f, %f, %f, %f,%f,%f\n", bladeOrientation.leftBladeVector[0], bladeOrientation.leftBladeVector[1], bladeOrientation.leftBladeVector[2], bladeOrientation.rightBladeVector[0], bladeOrientation.rightBladeVector[1], bladeOrientation.rightBladeVector[2]);
    leftBladeVector = SP_Math::Vector(bladeOrientation.leftBladeVector[0], bladeOrientation.leftBladeVector[1], bladeOrientation.leftBladeVector[2]);
    rightBladeVector = SP_Math::Vector(bladeOrientation.rightBladeVector[0], bladeOrientation.rightBladeVector[1], bladeOrientation.rightBladeVector[2]);
    // Преобразование векторов нормали лопастей в стандартную систему координат
//    SP_CoordinateTransform::transformBladeOrientation(rightBladeVector, leftBladeVector, paddle->getSpecs().axisDirection);
    rightBladeVector.normalize();
    leftBladeVector.normalize();
    calibrateLoads = true;

}

void PredictedPaddle::onUpdateSpecs(const PaddleSpecs& specs, SmartPaddle* paddle) {
//    Serial.printf("PredictedPaddle::onUpdateSpecs: %d\n", specs.axisDirection);
    relativeOrientation->setPaddleAxisDirection(specs.axisDirection);
    loadCellCalibrator->setPaddleParameters(specs.length, specs.imuDistance, specs.bladeWeight, specs.bladeCenter, specs.bladeMomentInertia);
}

void PredictedPaddle::onDisconnect(SmartPaddle* paddle) {
    loadCellCalibrator->resetTare();
}

void PredictedPaddle::updateKayakOrientation(const OrientationData& kayakOrientation) {
    relativeOrientation->updateKayakOrientation(kayakOrientation);
}

void PredictedPaddle::updateKayakIMU(const IMUData& kayakIMU) {

    relativeOrientation->updateKayakIMU(kayakIMU);
}

void PredictedPaddle::teachStrokePredictor(float force, bool isForward) {
    strokePredictor->learn(relativeOrientation, force, isForward);
}

float PredictedPaddle::predictStroke(bool isForward, float deltaT, float* confidence) {
    return strokePredictor->predict(relativeOrientation, deltaT, isForward, confidence);
}

SP_Math::Vector PredictedPaddle::getLeftBladeVector() {
    return relativeOrientation->getPaddleRelativeOrientation().rotate(leftBladeVector);
}

SP_Math::Vector PredictedPaddle::getRightBladeVector() {
    return relativeOrientation->getPaddleRelativeOrientation().rotate(rightBladeVector);
}

float PredictedPaddle::getAxialForce(BladeSideType bladeSide) {
    if (bladeSide == ALL_BLADES) {
        bladeSide = relativeOrientation->getLowerBladeSide();
    }
    if (bladeSide == LEFT_BLADE) {
        return - getLeftBladeVector().x() *loadCellCalibrator->getCalibratedForce(false, currentLoadCellData.forceL);
    } else {
        return - getRightBladeVector().x() * loadCellCalibrator->getCalibratedForce(true, currentLoadCellData.forceR);
    }
}