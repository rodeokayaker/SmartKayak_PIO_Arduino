#ifndef INTERFACE_LOAD_CELL_H
#define INTERFACE_LOAD_CELL_H

#include <Arduino.h>

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
    virtual void reset(uint32_t delay_ms=10) = 0;
    virtual uint8_t getDRDYPin() = 0;
    virtual bool isDataReady() = 0;
    virtual ~ILoadCell() = default; // Деструктор по умолчанию
};

#endif