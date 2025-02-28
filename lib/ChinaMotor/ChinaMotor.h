#ifndef ChinaMotor_h
#define ChinaMotor_h

#include "InterfaceMotor.h"
#include "Arduino.h"
#include <ESP32Servo.h>

#define IDLE_TIME 500
#define STOP_TIME 1000

class ChinaMotor: public IMotorDriver {

    int currentForce;
    Servo servo;
    int motor_pin;
    uint32_t motor_idle_start_time;

    uint32_t stop_time;
    uint32_t force_change_time;


    public:
    ChinaMotor(int pin);

    void begin() override;

    void setForce(int speed) override;

    int getForce() override;

    bool stop() override;

    void runRaw(int speed);

    ~ChinaMotor();
};

#endif