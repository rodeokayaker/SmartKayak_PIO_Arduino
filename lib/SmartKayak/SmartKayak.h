#ifndef SMARTKAYAK_H
#define SMARTKAYAK_H

#include "SmartPaddle.h"
#include "Peripherals.h"
#include "LogInterface.h"
#include "SP_Quaternion.h"

#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#include "InterfaceMotor.h"
#include "LoadCellCalibrator.h"
#include "DisplayInterface.h"


#define SMARTKAYAK_LOG_FORCE 1

class SmartKayak;

struct AutoTareData {
    double samples;
    float sum;
    float average;
};


class SmartKayak{
    friend class SmartKayakRTOS;
    friend  void  dmpDataReady();

    SP_Math::Vector paddleNullVector;
    float paddleShaftAngle;
    SP_Math::Quaternion paddleCalibQuaternion;
    SmartPaddle* paddle;
    IMotorDriver* motorDriver;
    IModeSwitch* modeSwitch;
    IIMU* imu;
    uint32_t imuFrequency;
    int nullLoadLeft;
    int nullLoadRight;

    LoadCellCalibrator loadCellCalibrator;
    KayakDisplay* display;
    KayakDisplayData displayData;


    TaskHandle_t imuTaskHandle;
    TaskHandle_t magnetometerTaskHandle;
    public:
    SmartKayak();
    void begin();

    void update();
    void updateIMU();

    void setPaddle(SmartPaddle* paddle);
    void setMotorDriver(IMotorDriver* motorDriver);
    void setModeSwitch(IModeSwitch* modeSwitch);
    void setIMU(IIMU* imu, uint32_t frequency);
    void logState(ILogInterface* logger);

    void calibratePaddle();
    void logVizualizeSerial();
    void logVizualizeMag();

    void logCall(ILogInterface* logger, LogMode logMode, int* loadCell=nullptr, int* externalForce=nullptr);

    void setDisplay(KayakDisplay* newDisplay) {
        display = newDisplay;
    }

    void startTasks();




};




#endif
