#ifndef ChinaMotor_h
#define ChinaMotor_h

#include "InterfaceMotor.h"
#include "Arduino.h"
#include <ESP32Servo.h>

#define IDLE_TIME 200
#define STOP_TIME 500

class ChinaMotor: public IMotorDriver {

    int currentForce;
    Servo servo;
    int motor_pin;
    uint32_t motor_idle_start_time;
    float forceGramms;

    uint32_t stop_time;
    uint32_t force_change_time;


    const int STOP_SIGNAL=1500;
    const int FULL_FORWARD_SIGNAL=2000;
    const int FULL_REVERSE_SIGNAL=1000;

    // Константы модели из ИТОГОВАЯ_МОДЕЛЬ.md
    const float BREAKPOINT_1=1601.8f;  // Граница мертвой и рабочей зоны
    const float BREAKPOINT_2=1873.1f;  // Граница рабочей зоны и насыщения
    const int MAX_THRUST=7710;      // Максимальная тяга в граммах
    
    // Коэффициенты для рабочей зоны
    // F_кг = ((-2901.2*M + 4,787,786) - 140,692) × (-9.8×10⁻⁶)
    // Упрощенно: F_кг = (4,647,094 - 2901.2*M) × (-9.8×10⁻⁶)
    const float INTERCEPT=4647094.0;    // 4,787,786 - 140,692
    const float SLOPE=-2901.2;
    const float SCALE=-9.8e-3;
    

    public:
    ChinaMotor(int pin);

    void begin() override;

    void setForce(int grams) override;

    int getForce() override;

    bool stop() override;

    void runRaw(int speed);

    int getForceGramms() override {return forceGramms;}

    // Преобразование желаемой тяги в килограммах в управляющий сигнал мотора
    int getSignalFromForce(int desired_thrust_gr);
    
    // Установка тяги в килограммах (удобный метод)
    void setThrustKg(int desired_thrust_gr);

    ~ChinaMotor();
};

#endif