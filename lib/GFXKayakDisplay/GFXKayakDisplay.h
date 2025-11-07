#pragma once
#include "../Core/Interfaces/IDisplay.h"
#include <Arduino_GFX_Library.h>

class GFXKayakDisplay : public KayakDisplay {

    Arduino_GFX* gfx;       // Invoke custom library
    uint16_t width;
    uint16_t height;
    bool landscape;
    bool initialized;
    int mode;
    TaskHandle_t updateGFXTaskHandle;
    SemaphoreHandle_t gfxMutex;
    class SmartKayak* kayak;
    bool firstShow;

    struct PPaddleData {
        int oldLineCoord[4];
        int lastShaftRotation;
        int lastShaftTilt;
        int lastBladeRotation;
        bool lastIsRightBlade;
        int lastLeftTare;
        int lastRightTare;
        int lastLeftForce;
        int lastRightForce;
        uint8_t lastStatus;
    } ppaddleData[N_PADDLES];

    struct ModeData {
        int motorMode;
        int LogMode;
        int predictorMode;
    } modeData;

    struct DebugData {
        int force;
        int load;
        bool firstShow;
        bool scn;
    } debugData;

    struct MotorData {
        int signal;
        float force;
    } motorData;

    bool debugScreen;

    int ledPin;

public:
    GFXKayakDisplay(Arduino_GFX* gfx, int ledPin = -1, int frequency = 5);
    void begin() override;
    void setRotation(int rotation);
    void setKayak(SmartKayak* kayak) {
        this->kayak = kayak;
    }
    void updateDisplay() override ;
    void clear();
    void setMode(int mode) {
        this->mode = mode;
    }
    void end();
    ~GFXKayakDisplay();
    void startTasks();
    void setDebugData(int force, int load, bool scn) override;
    void switchDebugScreen(bool on) override;
    
private:
    static void updateGFXTask(void* pvParameters);
    void showStatusLines();
    void showMotorForceScreen();
    void showModeLines();
    void showOrientationScreen();
    void showDebugScreen();

};
    