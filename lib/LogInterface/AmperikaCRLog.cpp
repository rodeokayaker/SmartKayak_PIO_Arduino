#ifdef USE_SD_LOG
#include "AmperikaCRLog.h"
#include <SPI.h>
#include <SD.h>

AmperikaCRLog::AmperikaCRLog(uint8_t cs_pin, uint8_t sck_pin, uint8_t miso_pin, uint8_t mosi_pin):
    cs_pin(cs_pin), sck_pin(sck_pin), miso_pin(miso_pin), mosi_pin(mosi_pin),
    fileOpened(false)
{
}

bool AmperikaCRLog::begin(const char* filename) {
    if (filename) {
        setFilename(filename);
    }
   // Настройка пинов
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH); // Отключаем карту по умолчанию
    Serial.println("SD Card initialization started");

    SPISettings spiSettings(10000000, MSBFIRST, SPI_MODE0); // 10MHz, стандартный режим

    Serial.println("SPI initialization started");
    // Инициализация SPI для SD карты
    SPI.begin(sck_pin, miso_pin, mosi_pin);
    Serial.println("SPI initialization finished");
    Serial.println("SPI initialization finished");

//    SPI.beginTransaction(spiSettings);
    Serial.println("SPI transaction started");
    // Инициализация SD карты
    if (!SD.begin(cs_pin)) {
        Serial.println("SD Card initialization failed!");
        return false;
    }
    Serial.println("SD Card initialized successfully");
    return true;
}

void AmperikaCRLog::setFilename(const char* filename) {
    if (filename) this->filename = filename;
}

bool AmperikaCRLog::openFile() {
    if (fileOpened) closeFile();
    dataFile = SD.open(filename, FILE_APPEND);
    if (dataFile) {
        fileOpened = true;
        return true;
    }
    return false;
}

bool AmperikaCRLog::closeFile() {
    if (fileOpened) {
        dataFile.close();
        fileOpened = false;
        return true;
    }
    return false;
}

bool AmperikaCRLog::clearFile() {
    if (fileOpened) {
        dataFile.close();
        dataFile = SD.open(filename, FILE_WRITE);
        return true;
    } else {
        dataFile = SD.open(filename, FILE_WRITE);
        dataFile.close();
        return true;
    }
}


int AmperikaCRLog::available() {
    return false;
}

int AmperikaCRLog::read() {
    return -1;
}

size_t AmperikaCRLog::readBytes(uint8_t* buffer, size_t bufferSize) {
    return 0;
}

int AmperikaCRLog::peek() {
    return -1;
}

size_t AmperikaCRLog::write(uint8_t byte) {
    if (fileOpened) { 
        return dataFile.write(byte); 
    } else {
        dataFile=SD.open(filename, FILE_APPEND);
        size_t ret = dataFile.write(byte);
        dataFile.close();
        return ret;
    }
}

size_t AmperikaCRLog::write(const uint8_t* buffer, size_t bufferSize) {
    if (fileOpened) { 
        return dataFile.write(buffer, bufferSize); 
    } else {
        dataFile=SD.open(filename, FILE_APPEND);
        size_t ret = dataFile.write(buffer, bufferSize);
        dataFile.close();
        return ret;
    }
}

void AmperikaCRLog::flush() {
    if (fileOpened) { 
        dataFile.write('\n');
        dataFile.flush(); 
    } else {
        dataFile=SD.open(filename, FILE_APPEND);
        dataFile.write('\n');
        dataFile.close();
    }
}

void AmperikaCRLog::logQuaternion(const float* q){
    this->printf("%f; %f; %f; %f;", q[0], q[1], q[2], q[3]);
}

void AmperikaCRLog::logLoads(const loadData& loads){
    this->printf("%d; %d; %d;", loads.timestamp, loads.forceR, loads.forceL);
}

void AmperikaCRLog::logIMU(const IMUData& imu){
    this->printf("%lu; %f; %f; %f; %f; %f; %f; %d; %d; %d; %f; %f; %f; %f; %f; %f; %f;",
        imu.timestamp,
        imu.ax, imu.ay, imu.az,
        imu.gx, imu.gy, imu.gz,
        imu.mag_x, imu.mag_y, imu.mag_z,
        imu.mx, imu.my, imu.mz,
        imu.q0, imu.q1, imu.q2, imu.q3
    );
}

void AmperikaCRLog::logOrientation(const OrientationData& orientation){
    this->printf("%lu; %f; %f; %f; %f;",
        orientation.timestamp,
        orientation.q0, orientation.q1, orientation.q2, orientation.q3
    );
}


#endif



