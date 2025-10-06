#ifndef LOADCELLSETHX711_H
#define LOADCELLSETHX711_H

#include "LoadCellSet.h"
#include <HX711.h>

class LoadCellSetHX711 : public LoadCellSet {
    private:
    HX711 scaleR;  // Right blade scale
    HX711 scaleL;  // Left blade scale
    
    uint8_t doutR_pin;
    uint8_t sclkR_pin;
    uint8_t doutL_pin;
    uint8_t sclkL_pin;
    
    loadData currentData;
    bool rightUpdated;
    bool leftUpdated;
    bool readyToRead;
    
    const char* prefs_Name;
    loadCellSetCalibrationData calibrationData;
    
    // FreeRTOS task handle
    static TaskHandle_t _taskHandle;
    
    // Private methods
    void calibrateBlade(BladeSideType bladeSide, float weight);
    void tareBlade(BladeSideType bladeSide);
    void saveCalibrationData();
    bool readCalibrationData();
    void resetCalibration();
    
    // FreeRTOS task and interrupt handlers
    static void taskEntry(void* arg);
    static void IRAM_ATTR intISR_R();
    static void IRAM_ATTR intISR_L();
    
    public:
    LoadCellSetHX711(const char* prefsName, PaddleType paddleType=TWO_BLADES, BladeSideType bladeSide=ALL_BLADES);
    bool begin(uint8_t doutR_pin, uint8_t sclkR_pin, uint8_t doutL_pin, uint8_t sclkL_pin);
    void startServices() override;
    void stopServices() override;
    void calibrate(BladeSideType bladeSide, float weight) override;
    void tare(BladeSideType bladeSide) override;
    bool isCalibrationValid(BladeSideType bladeSide) override;
    void reset() override;
    bool isDataReady(BladeSideType bladeSide) override;
    void setFrequency(uint16_t frequency) override;
    bool getData(loadData& data) override;
    void updateTare(BladeSideType bladeSide, float weight) override;
    
    // Additional methods
    void setPins(uint8_t doutR_pin, uint8_t sclkR_pin, uint8_t doutL_pin, uint8_t sclkL_pin);
    bool isDataReady() { return readyToRead && (scaleR.is_ready() || scaleL.is_ready()); }
};

#endif
