#ifndef LOADCELLSET_H
#define LOADCELLSET_H

#include <Arduino.h>
#include "SP_Types.h"

struct loadData {
    float forceR;
    float forceL;
    int32_t forceR_raw;
    int32_t forceL_raw;
    uint32_t timestamp;
};

struct loadCellSetCalibrationData {
    float scale[2]; // scale for right and left load cells
    float offset[2]; // offset for right and left load cells
};

class LoadCellSet {
    protected:
        BladeSideType bladeSide;
        Stream* logStream;
    
        uint16_t frequency;        

        void (*loadDataCb)(const loadData&, BladeSideType);
        bool singleNotify;
    public:
    LoadCellSet(PaddleType paddleType=TWO_BLADES, BladeSideType bladeSide=ALL_BLADES):
    singleNotify(false), logStream(&Serial), frequency(100), loadDataCb(nullptr)
    {
        if (paddleType == TWO_BLADES)
            this->bladeSide = ALL_BLADES;
        else
            this->bladeSide = bladeSide;
    }
    
    void setLogStream(Stream* logStream) { this->logStream = logStream; }    
    void onLoadData(void (*loadDataCb)(const loadData&, BladeSideType), bool singleNotify=false) { this->loadDataCb = loadDataCb; this->singleNotify = singleNotify; }
    virtual bool getData(loadData& data) = 0;
    virtual void startServices() = 0;
    virtual void stopServices() = 0;

    virtual void calibrate(BladeSideType bladeSide, float weight) = 0;
    virtual void tare(BladeSideType bladeSide) = 0;
    virtual bool isCalibrationValid(BladeSideType bladeSide) = 0;
    virtual void updateTare(BladeSideType bladeSide, float weight) = 0;

    virtual void reset() = 0;
    virtual bool isDataReady(BladeSideType bladeSide) = 0;
    virtual void setFrequency(uint16_t frequency) { this->frequency = frequency; }
    virtual uint16_t getFrequency() { return frequency; };

    };

#endif