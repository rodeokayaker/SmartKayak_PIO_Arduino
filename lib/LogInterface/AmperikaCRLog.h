#ifndef AMPERIKA_CR_LOG_H
#define AMPERIKA_CR_LOG_H

#include "../Core/Interfaces/ILogger.h"
#include <SD.h>

class AmperikaCRLog : public ILogInterface{
    private:
    File dataFile;
    String filename;
    bool fileOpened;
    String pref_name_str;
    String dir_name;

    uint8_t cs_pin;    // CS пин
    uint8_t sck_pin;  // SCK пин
    uint8_t miso_pin; // MISO пин
    uint8_t mosi_pin; // MOSI пин

    bool started;

    private:

    public:

    AmperikaCRLog(uint8_t cs_pin, uint8_t sck_pin, uint8_t miso_pin, uint8_t mosi_pin);

    bool begin(const char* filename=nullptr);
    bool openFile();
    bool closeFile();
    bool clearFile();
    bool Started() override;
    void setFilename(const char* pref_name);
    void newFile(const char* suffix);
    bool StartLog(const char* logName="LOG") override;
    bool StopLog() override;
    void logQuaternion(const float* q) override;
    void logLoads(const loadData& loads) override;
    void logIMU(const IMUData& imu) override;
    void logOrientation(const OrientationData& orientation) override;
    //Stream metods implementation
    virtual int available() override;
    virtual int read() override;
    virtual size_t readBytes(uint8_t* buffer, size_t bufferSize) override;
    virtual int peek() override;
    virtual size_t write(uint8_t byte) override;
    virtual size_t write(const uint8_t* buffer, size_t bufferSize) override;
    virtual void flush() override;
    void createDir(const char* dir);
    virtual bool Opened() override {return fileOpened;};
    virtual bool Open() override {return openFile();};
    virtual bool Close() override {return closeFile();};
};

#endif