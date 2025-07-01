#ifndef InterfaceMotor_h
#define InterfaceMotor_h

enum MotorDirection {
    REVERSE = -1,
    BRAKE = 0,
    FORWARD = 1
};

enum MotorPowerMode {
    MOTOR_OFF = 0,
    MOTOR_LOW_POWER = 1,
    MOTOR_MEDIUM_POWER = 2,
    MOTOR_HIGH_POWER = 3,
    MOTOR_DEBUG = 4
};

class IMotorDriver {
    public:
    virtual void setForce(int grams)=0;
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

#endif