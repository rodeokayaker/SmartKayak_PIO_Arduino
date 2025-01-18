#ifndef INTERFACE_LOAD_CELL_H
#define INTERFACE_LOAD_CELL_H

#include <Arduino.h>

struct loadData {
    int32_t forceL;
    int32_t forceR;
    uint32_t timestamp;
};

class ILoadCell {
public:
    virtual float getForce() = 0;
    virtual void calibrate() = 0;
    virtual bool isCalibrationValid() = 0;
    virtual bool begin() = 0;
    virtual void setLogStream(Stream* stream = &Serial) = 0;
    virtual ~ILoadCell() = default; // Деструктор по умолчанию
};

#endif