#pragma once
#include "../Core/Interfaces/IDisplay.h"
#include <TFT_eSPI.h>

class TFTSmallDisplay : public KayakDisplay {

    TFT_eSPI tft;       // Invoke custom library
    int mode;
    TaskHandle_t updateTFTTaskHandle;
    SemaphoreHandle_t tftMutex;
    KayakDisplayData lastData;
    bool firstShow;
    int oldLineCoord[4];

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
    void update(const KayakDisplayData& data) override;
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
    