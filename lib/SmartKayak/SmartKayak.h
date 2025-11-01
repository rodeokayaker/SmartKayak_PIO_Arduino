#ifndef SMARTKAYAK_H
#define SMARTKAYAK_H

#include "SmartPaddle.h"
#include "Peripherals.h"
#include "../Core/Interfaces/ILogger.h"
#include "SP_Quaternion.h"
#include "ForceAdapter.h"
#include "PredictedPaddle.h"

#include <Wire.h>

#include "../Core/Interfaces/IMotor.h"
#include "../Core/Interfaces/IDisplay.h"
#include "LoadCellCalibrator.h"


#define SMARTKAYAK_LOG_FORCE 1
#define LOADCELL_SMOOTHING_FACTOR 0.5f



class SmartKayak{
    friend class SmartKayakRTOS;
    friend  void  dmpDataReady();

    SP_Math::Quaternion kayakOrientationQuat;

    SmartPaddle* paddle;
    PredictedPaddle* predictedPaddle;

    IMotorDriver* motorDriver;
    IModeSwitch* modeSwitch;
    IIMUSensor* imu;

    BladeSideType currentBladeSide;
    int currentForceGramms;

    ForceAdapter forceAdapter;
    KayakDisplay* display;
    
    TaskHandle_t imuTaskHandle;
    TaskHandle_t magnetometerTaskHandle;

    int8_t predictorMode;  // 0 - не используется, 1 - используется

    uint32_t predictedTimeUsed;
    bool predictedForceUsed;
    bool usingPredictedForce;

    KayakSpecs specs;
    String prefsName;

public:
    SmartKayak(String prefs_name);
    void begin();

    void loadSpecs();
    void saveSpecs();

    void setSpecs(const KayakSpecs& s, bool save = true);
    KayakSpecs getSpecs() const { return specs; }

    void startTasks();
    void update();
    void setPaddle(SmartPaddle* paddle);
    void setMotorDriver(IMotorDriver* motorDriver);
    void setModeSwitch(IModeSwitch* modeSwitch);
    void setIMU(IIMUSensor* imu, uint32_t frequency);
    void setDisplay(KayakDisplay* newDisplay);
    void enableGridPredictor(){ predictorMode = 1; };
    void disablePredictor(){ predictorMode = 0; };
    int8_t getPredictorMode(){ return predictorMode; };
    void setPredictorMode(int8_t mode){ predictorMode = mode; };
    bool isUsingPredictedForce(){ return usingPredictedForce; };

    void logCall(ILogInterface* logger, LogMode logMode, int* loadCell = nullptr, int* externalForce = nullptr);



private:
//    SP_Math::Quaternion getRelativeOrientation(SP_Math::Quaternion& orientation);

};




#endif
