#include "ChinaMotor.h"


ChinaMotor::ChinaMotor(int pin): 
    currentForce(0),
    motor_pin(pin),
     motor_idle_start_time(0),
     force_change_time(0),
     stop_time(0)
{
}

void ChinaMotor::begin() {
  
        // Initialize and arm the MOTOR
        servo.attach(motor_pin);
        servo.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
        delay(3000); // delay to allow the ESC to recognize the stopped signal.
          
        Serial.printf("ChinaMotor begin\n");
}

void ChinaMotor::setForce(int speed) {
        if (speed == 0 && currentForce == 0) {
            return;
        }
        if ((currentForce >0 && speed<0) || (currentForce <0 && speed>0)) {
            // if force is changing direction, start idle time
            //Serial.printf("Changing direction, starting idle time\n");
            motor_idle_start_time = millis();
            servo.writeMicroseconds(1500);
            stop_time = millis();
            force_change_time = millis();
            currentForce = 0;
            return;
        }
        if (millis() - motor_idle_start_time < IDLE_TIME) {
            // keep motor idle for IDLE_TIME milliseconds
            servo.writeMicroseconds(1500);
            stop_time = millis();
            force_change_time = millis();
            currentForce = 0;
            return;
        }


        int forceApprox = 0;
        if (stop_time>millis()) {
            forceApprox = (int)((float)currentForce*(float)(stop_time-millis())/(float)(stop_time-force_change_time));
        }
        if (speed>0) {
            currentForce = max(forceApprox, speed);
        }
        if (speed<0) {
            currentForce = min(forceApprox, speed);
        }
        if (speed==0) {
            currentForce = forceApprox;
        }
        if (currentForce==speed) {
            stop_time = millis()+STOP_TIME;
        }
//        Serial.printf("currentForce: %d, speed: %d, forceApprox: %d\n", currentForce, speed, forceApprox);
        force_change_time = millis();
//        Serial.printf("ChinaMotor setForce: %d\n", speed);
        int PWM1_DutyCycle = map(currentForce, -1000, 1000, 1000, 2000);
        if (PWM1_DutyCycle < 1000) {
            PWM1_DutyCycle = 1000;
        }
        if (PWM1_DutyCycle > 2000) {
            PWM1_DutyCycle = 2000;
        }
        servo.writeMicroseconds(PWM1_DutyCycle);
}

void ChinaMotor::runRaw(int speed) {
    servo.writeMicroseconds(speed);
}

bool ChinaMotor::stop() {
    currentForce = 0;
    servo.writeMicroseconds(1500);
    stop_time = millis();
    force_change_time = millis();
    return true;
}

int ChinaMotor::getForce() {
        return currentForce;
}

ChinaMotor::~ChinaMotor() {
    servo.detach();
}