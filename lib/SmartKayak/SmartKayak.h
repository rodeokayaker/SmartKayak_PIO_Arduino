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

// Добавляем enum для состояний предвосхищения
enum class AnticipationState {
    IDLE,                    // Ожидание
    ANTICIPATION_TRIGGERED,  // Сработало предвосхищение 
    MOTOR_ACTIVE,           // Мотор активен
    FORCE_CONFIRMED         // Сила подтверждена
};


class SmartKayak;

struct AutoTareData {
    double samples;
    float sum;
    float average;
};

// Структура для настроек предвосхищения
struct AnticipationSettings {
    float triggerPitchAngle;      // Угол срабатывания (-12.2°)
    float hysteresis;             // Гистерезис (2°)
    unsigned long anticipationTime; // Время предвосхищения (460 мс)
    unsigned long timeoutTime;     // Таймаут сброса (2000 мс)
    int borderLoadForce;          // Пороговая сила (400)
    float minMotorPower;          // Минимальная мощность мотора (0.5)
    float maxMotorPower;          // Максимальная мощность мотора (1.0)
    
    AnticipationSettings() :
        triggerPitchAngle(-12.2f),
        hysteresis(2.0f),
        anticipationTime(460),
        timeoutTime(2000),
        borderLoadForce(400),
        minMotorPower(0.5f),
        maxMotorPower(1.0f) {}
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

    // Добавляем переменные для системы предвосхищения
    AnticipationState anticipationState;
    AnticipationSettings anticipationSettings;
    unsigned long anticipationStartTime;
    unsigned long lastStateChangeTime;
    float lastShaftTiltAngle;
    int anticipatedForce;
    bool anticipationEnabled;
    
    // Счетчики для статистики
    unsigned long anticipationTriggerCount;
    unsigned long successfulAnticipationCount;
    unsigned long falsePositiveCount;
    


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

        // Новые методы для управления предвосхищением
    void enableAnticipation(bool enable = true) { anticipationEnabled = enable; }
    void setAnticipationSettings(const AnticipationSettings& settings) { anticipationSettings = settings; }
    AnticipationState getAnticipationState() const { return anticipationState; }
    void getAnticipationStats(unsigned long& triggers, unsigned long& successful, unsigned long& falsePositive) const {
        triggers = anticipationTriggerCount;
        successful = successfulAnticipationCount;
        falsePositive = falsePositiveCount;
    }
    void resetAnticipationStats() {
        anticipationTriggerCount = 0;
        successfulAnticipationCount = 0;
        falsePositiveCount = 0;
    }


private:
    // Внутренние методы для предвосхищения
    void updateAnticipationLogic(float shaftTiltAngle, float bladeForce, int& force);
    void transitionToState(AnticipationState newState);
    int calculateAnticipatedForce(float bladeForce, float anticipationFactor = 0.63f);




};




#endif
