/**
 * @file IDisplay.h
 * @brief Display interfaces
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#ifndef CORE_I_DISPLAY_H
#define CORE_I_DISPLAY_H

#include "../Types.h"
#include "IMotor.h"
#include "ILogger.h"

// Base display interface
class IDisplay {
public:
    virtual void begin() = 0;
    virtual void clear() = 0;
    virtual void update() = 0;
    virtual ~IDisplay() = default;
};

// Text display interface
class ITextDisplay : public IDisplay {
public:
    virtual void setCursor(int x, int y) = 0;
    virtual void print(const char* text) = 0;
    virtual void printf(const char* format, ...) = 0;
    virtual int getColumns() const = 0;
    virtual int getRows() const = 0;
};

// Kayak-specific display base class
class PredictedPaddle;
#define N_PADDLES 1

class KayakDisplay {
protected:
    uint32_t updateInterval;
    uint32_t lastUpdate;
    ILogSwitch* logSwitch;
    IModeSwitch* motorSwitch;
    IMotorDriver* motorDriver;
    PredictedPaddle* predictedPaddle[N_PADDLES];
    int nPaddles;
public:
    explicit KayakDisplay(uint32_t interval = 100) 
        : updateInterval(interval), lastUpdate(0), 
          logSwitch(nullptr), motorSwitch(nullptr), motorDriver(nullptr), nPaddles(0) {
            for (int i = 0; i < N_PADDLES; i++) {
                predictedPaddle[i] = nullptr;
            }
          }
    
    virtual void begin() = 0;
    virtual void update() {
        if (millis() - lastUpdate < updateInterval) {
            return;
        }
        updateDisplay();
        lastUpdate = millis();
    }
    
    virtual void setLogSwitch(ILogSwitch* logSwitch) {
        this->logSwitch = logSwitch;
    }
    virtual void setMotorSwitch(IModeSwitch* motorSwitch) {
        this->motorSwitch = motorSwitch;
    }
    virtual void setMotorDriver(IMotorDriver* motorDriver) {
        this->motorDriver = motorDriver;
    }

    virtual int addPredictedPaddle(PredictedPaddle* ppaddle){
        if (nPaddles < N_PADDLES) {
            predictedPaddle[nPaddles] = ppaddle;
            nPaddles++;
        }
        return nPaddles;
    }    
    

    virtual void setDebugData(int force, int load, bool scn = false) {}
    virtual void switchDebugScreen(bool on) {}
    
protected:
    virtual void updateDisplay() = 0;
};

#endif // CORE_I_DISPLAY_H

