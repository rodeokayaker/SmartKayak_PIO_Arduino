#ifndef LOADCELLSETADS1220_H
#define LOADCELLSETADS1220_H

#include "LoadCellSet.h"
#include <SPI.h>
#include "Protocentral_ADS1220.h"




class LoadCellSetADS1220 : public LoadCellSet {
    private:
    static Protocentral_ADS1220 pc_ads1220;
    uint8_t sclk_pin;
    uint8_t miso_pin;
    uint8_t mosi_pin;
    uint8_t cs_pin;
    uint8_t drdy_pin;

    loadData currentData;
    bool rightUpdated;
    bool leftUpdated;
    bool firstRight;
    bool firstChanelSelected;
    const char* prefs_Name;

    static TaskHandle_t _taskHandle;
    loadCellSetCalibrationData calibrationData;

    int getFreqID();
    uint16_t requestedFreq() {
        if (bladeSide == ALL_BLADES)
            return frequency*2;
        return frequency;
    }
    static void taskEntry(void* arg);
    static void IRAM_ATTR intISR();


    void calibrateBlade(BladeSideType bladeSide, float weight);
    void tareBlade(BladeSideType bladeSide);

    public:
    LoadCellSetADS1220(const char* prefsName, PaddleType paddleType=TWO_BLADES, BladeSideType bladeSide=ALL_BLADES);
    bool begin(uint8_t sclk_pin, uint8_t miso_pin, uint8_t mosi_pin, uint8_t cs_pin, uint8_t drdy_pin);
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
    
    // Дополнительные методы для калибровки
    void saveCalibrationData();
    bool readCalibrationData();
    void resetCalibration(); 
};

#endif