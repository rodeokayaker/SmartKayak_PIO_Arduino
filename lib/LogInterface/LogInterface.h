#ifndef LOG_INTERFACE_H
#define LOG_INTERFACE_H

#include <Stream.h>
#include <Arduino.h>
#include "InterfaceLoadCell.h"
//#include "InterfaceIMU.h"
#include "ImuSensor.h"


enum LogMode {
    LOG_MODE_OFF = 0,
    LOG_MODE_PADDLE_MAG = 1,
    LOG_MODE_KAYAK_MAG = 2,
    LOG_MODE_DEBUG = 3,
    LOG_MODE_ALL = 4
};

static const int nLogModes = 5;

static const char* logModeNames[] = {
    "OFF",
    "PMAG",
    "KMAG",
    "DBG",
    "ALL"
};

class ILogSwitch {
    public:
    virtual LogMode getLogMode()=0;
    virtual bool getLogStarted()=0;
    virtual ~ILogSwitch()=default;
};

class ILogInterface: public Stream{
    public:

    virtual bool StartLog(const char* logName);
    virtual bool StopLog()=0;

//Log interface
    virtual void logQuaternion(const float* q){};
    virtual void logLoads(const loadData& loads){};
    virtual void logIMU(const IMUData& imu){};
    virtual void logOrientation(const OrientationData& orientation){};
    virtual bool Started(){return false;};
    virtual bool Opened(){return true;};
    virtual bool Open(){return true;};
    virtual bool Close(){return true;};
    virtual ~ILogInterface()=default;
};

#endif