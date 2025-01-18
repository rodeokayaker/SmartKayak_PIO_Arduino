#ifndef AMPERIKA_CR_LOG_H
#define AMPERIKA_CR_LOG_H

#include "LogInterface.h"
#include <SD.h>

class AmperikaCRLog : public ILogInterface{
    private:
    File dataFile;
    String filename;
    bool fileOpened;

    uint8_t cs_pin;    // CS пин
    uint8_t sck_pin;  // SCK пин
    uint8_t miso_pin; // MISO пин
    uint8_t mosi_pin; // MOSI пин

    private:

    public:

    AmperikaCRLog(uint8_t cs_pin, uint8_t sck_pin, uint8_t miso_pin, uint8_t mosi_pin);

    void begin();
    void begin(const char* filename);
    bool openFile();
    bool closeFile();
    bool clearFile();
    void setFilename(const char* filename);

    void logQuaternion(const float* q) override;
    void logLoads(const float* loads) override;
    void logIMU(const float* imu) override;

    //Stream metods implementation
    virtual int available() override;
    virtual int read() override;
    virtual size_t readBytes(uint8_t* buffer, size_t bufferSize) override;
    virtual int peek() override;
    virtual size_t write(uint8_t byte) override;
    virtual size_t write(const uint8_t* buffer, size_t bufferSize) override;
    virtual void flush() override;

};

#endif