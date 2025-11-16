/**
 * @file ILoadCell.h
 * @brief Interfaces for load cell sensors
 * 
 * This file contains two interfaces:
 * - ILoadCell: for single load cell
 * - ILoadCellSet: for set of load cells (typically left + right)
 * 
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#ifndef CORE_I_LOAD_CELL_H
#define CORE_I_LOAD_CELL_H

#include "../Types.h"

/**
 * @class ILoadCell
 * @brief Interface for single load cell sensor
 */
class ILoadCell {
public:
    virtual float getForce() = 0;
    virtual int32_t getRawForce() = 0;
    virtual bool read() = 0;
    virtual void calibrate() = 0;
    virtual void calibrateScale(float weight) = 0;
    virtual void tare() = 0;
    virtual bool isCalibrationValid() = 0;
    virtual void setLogStream(Stream* stream = &Serial) = 0;
    virtual uint16_t getFrequency() = 0;
    virtual void reset(uint32_t delay_ms = 10) = 0;
    virtual uint8_t getDRDYPin() = 0;
    virtual bool isDataReady() = 0;
    virtual ~ILoadCell() = default;
};

/**
 * @class ILoadCellSet
 * @brief Interface for set of load cells (left + right) with FreeRTOS support
 * 
 * This interface is designed for dual load cell systems typically used in paddle applications.
 * Implementations use FreeRTOS tasks for asynchronous data acquisition.
 */
class ILoadCellSet {
protected:
    BladeSideType bladeSide;
    Stream* logStream;
    uint16_t frequency;
    void (*loadDataCb)(const loadData&, BladeSideType);
    bool singleNotify;
    
public:
    ILoadCellSet(PaddleType paddleType = TWO_BLADES, BladeSideType bladeSide = ALL_BLADES):
        singleNotify(false), logStream(&Serial), frequency(100), loadDataCb(nullptr)
    {
        if (paddleType == TWO_BLADES)
            this->bladeSide = ALL_BLADES;
        else {
            this->bladeSide = bladeSide;
            singleNotify = true;
        }
    }
    
    // Data acquisition
    virtual bool getData(loadData& data) = 0;
    
    // Service control (FreeRTOS tasks)
    virtual void startServices() = 0;
    virtual void stopServices() = 0;
    
    // Calibration
    virtual void calibrate(BladeSideType bladeSide, float weight) = 0;
    virtual void tare(BladeSideType bladeSide) = 0;
    virtual bool isCalibrationValid(BladeSideType bladeSide) = 0;
    virtual void updateTare(BladeSideType bladeSide, float weight) = 0;
    
    // Hardware control
    virtual void reset() = 0;
    virtual bool isDataReady(BladeSideType bladeSide) = 0;
    
    // Configuration
    virtual void setFrequency(uint16_t frequency) { this->frequency = frequency; }
    virtual uint16_t getFrequency() { return frequency; }
    
    void setLogStream(Stream* logStream) { this->logStream = logStream; }
    void onLoadData(void (*loadDataCb)(const loadData&, BladeSideType), bool singleNotify = false) { 
        this->loadDataCb = loadDataCb; 
        if (bladeSide == ALL_BLADES)
            this->singleNotify = singleNotify;
        else
            this->singleNotify = true; 
    }
    
    virtual ~ILoadCellSet() = default;
};

#endif // CORE_I_LOAD_CELL_H

