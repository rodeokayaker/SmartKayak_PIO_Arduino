#ifndef SMARTKAYAK_H
#define SMARTKAYAK_H

#include "SmartPaddle.h"
#include "Peripherals.h"
#include "LogInterface.h"
#include "SP_Quaternion.h"

#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#define SMARTKAYAK_LOG_FORCE 1

class SmartKayak;

enum MotorDirection {
    REVERSE = -1,
    BRAKE = 0,
    FORWARD = 1
};

enum MotorPowerMode {
    MOTOR_OFF = 0,
    MOTOR_LOW_POWER = 1,
    MOTOR_MEDIUM_POWER = 2,
    MOTOR_HIGH_POWER = 3
};

class IMotorDriver {
    public:
    virtual void setForce(int speed)=0;
    virtual int getForce()=0;
    virtual bool stop() { setForce(0); return true;};
    virtual void begin()=0;
    virtual ~IMotorDriver() = default;
};

class IModeSwitch {
    public:
    virtual MotorPowerMode getMode()=0;
    virtual ~IModeSwitch() = default;
};

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
    AutoTareData leftTare;
    AutoTareData rightTare;
    uint32_t imuFrequency;
    int nullLoadLeft;
    int nullLoadRight;
    uint32_t log_level;
    uint32_t lastPrintLCD1;
    uint32_t lastPrintLCD2;

    hd44780* textLCD1;
    hd44780* textLCD2;
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




};




#endif
