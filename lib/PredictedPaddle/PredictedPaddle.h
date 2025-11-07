#ifndef PREDICTEDPADDLE_H
#define PREDICTEDPADDLE_H

#include "SmartPaddle.h"
#include "AngularAccelerationCalculator.h"
#include "PaddleOrientationCalculator.h"
#include "SP_Vector.h"
#include "LoadCellCalibrator.h"
#define LOADCELL_SMOOTHING_FACTOR 0.5f

#define DEFAULT_PREDICTION_TIME 300.0f
#define DEFAULT_GRID_STEP 10.0f
#define DEFAULT_ALPHA_EMA 0.2f
#define DEFAULT_FORCE_THRESHOLD 400.0f

class PP_EventHandler;
class StrokePredictorGrid;
class PaddleOrientationCalculator;
class AngularAccelerationCalculator;

class PredictedPaddle: public SP_EventHandler {

    private:
        SmartPaddle* paddle;
        LoadCellCalibrator* loadCellCalibrator;
        StrokePredictorGrid* strokePredictor;
        SP_Math::Vector leftBladeVector;
        SP_Math::Vector rightBladeVector;
        bool calibrateLoads;

        loadData currentLoadCellData;


    public:
        PaddleOrientationCalculator* relativeOrientation;

        PredictedPaddle(SmartPaddle* sp);
        ~PredictedPaddle();

        void updateKayakOrientation(const OrientationData& kayakOrientation);
        void updateKayakIMU(const IMUData& kayakIMU);

        void onUpdateLoad(loadData& loadData, SmartPaddle* paddle) override;
        void onUpdateOrientation(OrientationData& orientationData, SmartPaddle* paddle) override;
        void onUpdateIMU(IMUData& imuData, SmartPaddle* paddle) override;
        void onUpdateBladeAngle(BladeOrientation& bladeOrientation, SmartPaddle* paddle) override;
        void onUpdateSpecs(PaddleSpecs& specs, SmartPaddle* paddle) override;
        void onDisconnect(SmartPaddle* paddle) override;

        void getRelativeAngles(float& shaftRotation, float& shaftTilt, float& bladeRotation, bool& isRightBlade)
        {
            relativeOrientation->getPaddleAngles(shaftRotation, shaftTilt, bladeRotation);
            isRightBlade = relativeOrientation->getLowerBladeSide() == BladeSideType::RIGHT_BLADE;
//            Serial.printf("Axis direction: %d\n", paddle->getSpecs().axisDirection);
        }


        void teachStrokePredictor(float force, bool isForward);
        float predictStroke(bool isForward, float deltaT,  float* confidence = nullptr);

        SP_Math::Vector getLeftBladeVector();
        SP_Math::Vector getRightBladeVector();

        void startLoadCalibration() {calibrateLoads = true;};
        void stopLoadCalibration() {calibrateLoads = false;};

        loadData getLoadData() {return currentLoadCellData;};
        float getLeftTare() {return loadCellCalibrator->getLeftTare();};
        float getRightTare() {return loadCellCalibrator->getRightTare();};
        float getLeftForce() {return loadCellCalibrator->getCalibratedForce(false, currentLoadCellData.forceL);};
        float getRightForce() {return loadCellCalibrator->getCalibratedForce(true, currentLoadCellData.forceR);};
        float getForce(BladeSideType bladeSide) {return loadCellCalibrator->getCalibratedForce(bladeSide == BladeSideType::RIGHT_BLADE, bladeSide == BladeSideType::RIGHT_BLADE ? currentLoadCellData.forceR : currentLoadCellData.forceL);};
    
        float getAxialForce(BladeSideType bladeSide = BladeSideType::ALL_BLADES);  // получаем осевую силу на лопасти

        BladeSideType getBladeSide() {return relativeOrientation->getLowerBladeSide();};
        bool operating() {return paddle->operating();};
        uint8_t status() {return paddle->status();};
        void setKayakSpecs(const KayakSpecs& specs) {
            relativeOrientation->setKayakAxisDirection(specs.axisDirection);
        }
};

#endif