/**
 * @file ILogger.h
 * @brief Logging interface
 * @author Ivan Rybnikov
 * @copyright Copyright (c) 2024
 */

#ifndef CORE_I_LOGGER_H
#define CORE_I_LOGGER_H

#include <Stream.h>
#include <Arduino.h>
#include "../Types.h"

class ILogSwitch {
public:
    virtual LogMode getLogMode() = 0;
    virtual bool getLogStarted() = 0;
    virtual ~ILogSwitch() = default;
};

class ILogInterface : public Stream {
public:
    virtual bool StartLog(const char* logName);
    virtual bool StopLog() = 0;

    // Log interface methods
    virtual void logQuaternion(const float* q) {}
    virtual void logLoads(const loadData& loads) {}
    virtual void logIMU(const IMUData& imu) {}
    virtual void logOrientation(const OrientationData& orientation) {}
    virtual bool Started() { return false; }
    virtual bool Opened() { return true; }
    virtual bool Open() { return true; }
    virtual bool Close() { return true; }
    virtual ~ILogInterface() = default;
};

#endif // CORE_I_LOGGER_H

