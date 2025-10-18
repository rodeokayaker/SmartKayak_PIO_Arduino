/**
 * @file IMotor.h
 * @brief Motor driver interface
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#ifndef CORE_I_MOTOR_H
#define CORE_I_MOTOR_H

#include "../Types.h"

class IMotorDriver {
public:
    virtual void setForce(int grams) = 0;
    virtual int getForce() = 0;
    virtual int getForceGramms() = 0;
    virtual bool stop() { setForce(0); return true; }
    virtual void begin() = 0;
    virtual ~IMotorDriver() = default;
};

class IModeSwitch {
public:
    virtual MotorPowerMode getMode() = 0;
    virtual ~IModeSwitch() = default;
};

#endif // CORE_I_MOTOR_H

