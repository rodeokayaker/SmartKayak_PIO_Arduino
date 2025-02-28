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


class SmartKayak {
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
    uint32_t log_level;
    uint32_t lastPrintLCD1;
    uint32_t lastPrintLCD2;

    hd44780* textLCD1;
    hd44780* textLCD2;
    LoadCellCalibrator loadCellCalibrator;
    KayakDisplay* display;
    KayakDisplayData displayData;
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
    void setTextLCD(hd44780* lcd1=nullptr, hd44780* lcd2=nullptr) {
        textLCD1=lcd1; 
        textLCD2=lcd2;
    }

    void calibratePaddle();
    void logVizualizeSerial();
    void logVizualizeMag();

    void setLogLevel(uint32_t level) {log_level = level;};
    void andLogLevel(uint32_t level) {log_level = log_level & level;};
    void orLogLevel(uint32_t level) {log_level = log_level | level;};
    void xorLogLevel(uint32_t level) {log_level = log_level ^ level;};

    void offLogLevel(uint32_t level) {log_level = log_level & ~level;};
    void toggleLogLevel(uint32_t level) {log_level = log_level ^ level;};
    void onLogLevel(uint32_t level) {log_level = log_level | level;};

    void setDisplay(KayakDisplay* newDisplay) {
        display = newDisplay;
        if (display) display->begin();
    }




};




#endif
