#pragma once
#include "../Core/Interfaces/IDisplay.h"
#include <TFT_eSPI.h>

class TFTSmallDisplay : public KayakDisplay {

    TFT_eSPI tft;       // Invoke custom library
    int mode;
    TaskHandle_t updateTFTTaskHandle;
    SemaphoreHandle_t tftMutex;
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

    bool debugScreen;

public:
    TFTSmallDisplay(int frequency = 5);
    void begin() override;
    void setKayak(SmartKayak* kayak) {
        this->kayak = kayak;
    }
    void updateDisplay() override ;
//    void updateDisplayTask();
    void clear();
    void setMode(int mode) {
        this->mode = mode;
    }
    void end();
    ~TFTSmallDisplay();
    void startTasks();
    void setDebugData(int force, int load, bool scn) override;
    void switchDebugScreen(bool on) override;
    
private:
    static void updateTFTTask(void* pvParameters);
    void showStatusLines();
    void showMotorForceScreen();
    void showModeLines();
    void showOrientationScreen();
    void showDebugScreen();

};
    