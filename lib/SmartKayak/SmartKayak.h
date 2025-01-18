#ifndef SMARTKAYAK_H
#define SMARTKAYAK_H

#include "SmartPaddle.h"
#include "Peripherals.h"
#include "LogInterface.h"
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
    virtual void setSpeed(int speed)=0;
    virtual void setDirection(int direction)=0;
    virtual void setBrake(bool brake)=0;
    virtual void begin()=0;
    virtual ~IMotorDriver() = default;
};

class IModeSwitch {
    public:
    virtual MotorPowerMode getMode()=0;
    virtual ~IModeSwitch() = default;
};


class SmartKayak {
    SmartPaddle* paddle;
    IMotorDriver* motorDriver;
    IModeSwitch* modeSwitch;
    IIMU* imu;
    uint32_t imuFrequency;
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

};




#endif
