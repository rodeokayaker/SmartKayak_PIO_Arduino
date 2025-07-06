#include "ChinaMotor.h"


ChinaMotor::ChinaMotor(int pin): 
    currentForce(STOP_SIGNAL),
    motor_pin(pin),
     motor_idle_start_time(0),
     force_change_time(0),
     stop_time(0), 
     forceGramms(0)
{

    
}

void ChinaMotor::begin() {
  
        // Initialize and arm the MOTOR
        servo.attach(motor_pin);
        servo.writeMicroseconds(STOP_SIGNAL); // send "stop" signal to ESC. Also necessary to arm the ESC.
        delay(3000); // delay to allow the ESC to recognize the stopped signal.
          
        Serial.printf("ChinaMotor begin\n");
}

int ChinaMotor::getSignalFromForce(int desired_thrust_gr) {

    // Проверка входных данных - отрицательная тяга не поддерживается
    if (desired_thrust_gr < 0) {
        Serial.println("⚠️ Внимание: Отрицательная тяга (реверс) не поддерживается.");
        Serial.println("   Модель работает только для прямого хода (0-7.71 кг)");
        return STOP_SIGNAL;  // Остановка
    }
    
    // Мертвая зона: тяга ≤ 0 кг



    if (desired_thrust_gr == 0) {
        return STOP_SIGNAL;  // Стандартное значение остановки
    }
    
 /*   // Зона насыщения: тяга ≥ максимальной
    if (desired_thrust_gr >= MAX_THRUST) {
        return FULL_FORWARD_SIGNAL;  // Максимальный сигнал
    }*/
    
    // Рабочая зона: линейная зависимость
    // Обратная формула: M = (INTERCEPT - desired_thrust_kg / SCALE) / (-SLOPE)
    float motor_signal = (INTERCEPT - (float)desired_thrust_gr / SCALE) / (-SLOPE);
    
    // Округляем до целого и ограничиваем диапазон
    int result = (int)round(motor_signal);
//    result = max((int)(BREAKPOINT_1) + 1, min(result, (int)BREAKPOINT_2));
    result = max(STOP_SIGNAL, min(result, FULL_FORWARD_SIGNAL));    // TODO: remove this
    return result;
}

void ChinaMotor::setThrustKg(int desired_thrust_gr) {
    int motor_signal = getSignalFromForce(desired_thrust_gr);
    servo.writeMicroseconds(motor_signal);
    Serial.printf("Установлена тяга: %.2f гр, сигнал мотора: %d\n", desired_thrust_gr, motor_signal);
}

void ChinaMotor::setForce(int force) {
        forceGramms = force;
        if (force == 0 && currentForce == STOP_SIGNAL) {
            return;
        }

        if (millis() - motor_idle_start_time < IDLE_TIME) {
            // keep motor idle for IDLE_TIME milliseconds
            servo.writeMicroseconds(STOP_SIGNAL);
            stop_time = millis();
            force_change_time = millis();
            currentForce = STOP_SIGNAL;
            return;
        }
        if ((currentForce >STOP_SIGNAL && force<0) || (currentForce <STOP_SIGNAL && force>0)) {
            // if force is changing direction, start idle time
            //Serial.printf("Changing direction, starting idle time\n");
            if (millis() - last_force_change_time < FORCE_CHANGE_TIME) {
                force=0;
            } else {
                motor_idle_start_time = millis();
                servo.writeMicroseconds(STOP_SIGNAL);
                stop_time = millis();
                force_change_time = millis();
                currentForce = STOP_SIGNAL;
                return;
            }
        }

        if (force!=0) last_force_change_time = millis();

        int forceApprox = STOP_SIGNAL;
        if (stop_time>millis()) {
            forceApprox = STOP_SIGNAL+(int)((float)(currentForce-STOP_SIGNAL)*(float)(stop_time-millis())/(float)(stop_time-force_change_time));
        }
        int signal;
        if (force>0) {
            signal = getSignalFromForce(force);
        }
        if (force<0) {
            signal = STOP_SIGNAL*2-getSignalFromForce(-force);
        }
        
        if (force>0) {
            signal = max(forceApprox, signal);
        }
        if (force<0) {
            signal = min(forceApprox, signal);
        }
        if (force==0) {
            signal = forceApprox;
        }

        if (signal!=forceApprox) {
            stop_time = millis()+STOP_TIME;
        }
        currentForce = signal;

        force_change_time = millis();

        if (currentForce<FULL_REVERSE_SIGNAL) {
            currentForce = FULL_REVERSE_SIGNAL;
        }
        if (currentForce>FULL_FORWARD_SIGNAL) {
            currentForce = FULL_FORWARD_SIGNAL;
        }
        

        servo.writeMicroseconds(currentForce);
}

void ChinaMotor::runRaw(int speed) {
    servo.writeMicroseconds(speed);
}

bool ChinaMotor::stop() {
    forceGramms = 0;
    currentForce = STOP_SIGNAL;
    servo.writeMicroseconds(STOP_SIGNAL);
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