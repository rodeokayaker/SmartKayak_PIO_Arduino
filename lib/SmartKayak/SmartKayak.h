#ifndef SMARTKAYAK_H
#define SMARTKAYAK_H

#include "SmartPaddle.h"
#include "Peripherals.h"
#include "../Core/Interfaces/ILogger.h"
#include "SP_Quaternion.h"
#include "ForceAdapter.h"

#include <Wire.h>

#include "../Core/Interfaces/IMotor.h"
#include "../Core/Interfaces/IDisplay.h"
#include "LoadCellCalibrator.h"


#define SMARTKAYAK_LOG_FORCE 1
#define LOADCELL_SMOOTHING_FACTOR 0.5f

class SmartKayak;

struct AutoTareData {
    double samples;
    float sum;
    float average;
};


class SmartKayak{
    friend class SmartKayakRTOS;
    friend  void  dmpDataReady();

    //SP_Math::Vector paddleNullVector;
    //float paddleShaftAngle;
    SP_Math::Quaternion kayakOrientationQuat;
    SmartPaddle* paddle;
    IMotorDriver* motorDriver;
    IModeSwitch* modeSwitch;
    IIMUSensor* imu;
    //int nullLoadLeft;
    //int nullLoadRight;

    BladeSideType currentBladeSide;
    int currentForceGramms;
    loadData currentLoadCellData;

    LoadCellCalibrator loadCellCalibrator;
    ForceAdapter forceAdapter;
    KayakDisplay* display;
    KayakDisplayData displayData;
    
    TaskHandle_t imuTaskHandle;
    TaskHandle_t magnetometerTaskHandle;


public:
    SmartKayak();
    void begin();

    void update();

    void setPaddle(SmartPaddle* paddle);
    void setMotorDriver(IMotorDriver* motorDriver);
    void setModeSwitch(IModeSwitch* modeSwitch);
    void setIMU(IIMUSensor* imu, uint32_t frequency);

    void logCall(ILogInterface* logger, LogMode logMode, int* loadCell=nullptr, int* externalForce=nullptr);

    void setDisplay(KayakDisplay* newDisplay) {
        display = newDisplay;
    }

    void startTasks();

private:
    SP_Math::Quaternion getRelativeOrientation(SP_Math::Quaternion& orientation, SmartPaddle* paddle = nullptr);

};




#endif
