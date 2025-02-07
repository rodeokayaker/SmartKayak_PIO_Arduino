#ifndef INTERFACE_LOAD_CELL_H
#define INTERFACE_LOAD_CELL_H

#include <Arduino.h>

struct loadData {
    float forceR;
    float forceL;
    int32_t forceR_raw;
    int32_t forceL_raw;
    uint32_t timestamp;
};

class ILoadCell {
public:
    virtual float getForce() = 0;
    virtual int32_t getRawForce() = 0;
    virtual void read() = 0;
    virtual void calibrate() = 0;
    virtual void calibrateScale(float weight) = 0;
    virtual void tare() = 0;
    virtual bool isCalibrationValid() = 0;
    virtual void setLogStream(Stream* stream = &Serial) = 0;
    virtual uint16_t getFrequency() = 0;
    virtual ~ILoadCell() = default; // Деструктор по умолчанию
};

#endif