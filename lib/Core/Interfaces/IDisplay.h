/**
 * @file IDisplay.h
 * @brief Display interfaces
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#ifndef CORE_I_DISPLAY_H
#define CORE_I_DISPLAY_H

#include <Arduino.h>
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
class KayakDisplay {
protected:
    KayakDisplayData currentData;
    uint32_t updateInterval;
    uint32_t lastUpdate;
    ILogSwitch* logSwitch;
    IModeSwitch* motorSwitch;
    IMotorDriver* motorDriver;

public:
    explicit KayakDisplay(uint32_t interval = 100) 
        : updateInterval(interval), lastUpdate(0), 
          logSwitch(nullptr), motorSwitch(nullptr), motorDriver(nullptr) {}
    
    virtual void begin() = 0;
    virtual void update(const KayakDisplayData& data) {
        if (millis() - lastUpdate < updateInterval) {
            return;
        }
        currentData = data;
        updateDisplay();
        lastUpdate = millis();
    }
    
    void paddleConnected(bool connected) {
        currentData.isPaddleConnected = connected;
    }

    KayakDisplayData getCurrentDisplayData() {
        return currentData;
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

    virtual void setDebugData(int force, int load, bool scn = false) {}
    virtual void switchDebugScreen(bool on) {}
    
protected:
    virtual void updateDisplay() = 0;
};

#endif // CORE_I_DISPLAY_H

