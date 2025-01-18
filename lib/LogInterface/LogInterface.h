#ifndef LOG_INTERFACE_H
#define LOG_INTERFACE_H

#include <Stream.h>
#include <Arduino.h>
#include "InterfaceLoadCell.h"
#include "InterfaceIMU.h"
class ILogInterface: public Stream{
    public:

//Log interface
    virtual void logQuaternion(const float* q){};
    virtual void logLoads(const float* loads){};
    virtual void logIMU(const float* imu){};
    // Stream implementation
/*    virtual int available() override 
    {
        return Serial.available();
    };
    virtual int read() override
    {
        return Serial.read();
    };
    virtual size_t readBytes(uint8_t* buffer, size_t bufferSize) override
    {
        return Serial.readBytes(buffer, bufferSize);
    };
    virtual int peek() override
    {
        return Serial.peek();
    };
    virtual size_t write(uint8_t byte) override
    {
        return Serial.write(byte);
    };
    virtual size_t write(const uint8_t* buffer, size_t bufferSize) override
    {
        return Serial.write(buffer, bufferSize);
    };
    virtual void flush() override
    {
        Serial.flush();
    };    */
    virtual ~ILogInterface()=default;
};

#endif