//#ifdef USE_SD_LOG
#include "AmperikaCRLog.h"
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>

AmperikaCRLog::AmperikaCRLog(uint8_t cs_pin, uint8_t sck_pin, uint8_t miso_pin, uint8_t mosi_pin):
    cs_pin(cs_pin), sck_pin(sck_pin), miso_pin(miso_pin), mosi_pin(mosi_pin),
    fileOpened(false), started(false)
{
}

//SPIClass spi2(HSPI);

bool AmperikaCRLog::begin(const char* pref_name) {

   // Настройка пинов
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH); // Отключаем карту по умолчанию
 //   Serial.println("SD Card initialization started");
//    SPISettings spiSettings(10000000, MSBFIRST, SPI_MODE0); // 10MHz, стандартный режим
//    SPIClass* vspi = new SPIClass(3);
    Serial.printf("SPI initialization %d, %d, %d, %d \n", sck_pin, miso_pin, mosi_pin, cs_pin);

//    spi2.begin(sck_pin, miso_pin, mosi_pin, cs_pin);
    SPI.begin(sck_pin, miso_pin, mosi_pin, cs_pin);
//    spi2.setFrequency(200000); // Уменьшаем частоту SPI до 200 кГц
//    Serial.println("SPI initialization finished");
    delay(100);
    
//    SPI.beginTransaction(spiSettings);    
//    Serial.println("SPI transaction started");
//    delay(100);
    // Инициализация SD карты (явно задаём шину и низкую частоту)
//    Serial.println("Проверка напряжения...");
//    Serial.printf("Напряжение на CS: %d\n", digitalRead(cs_pin));
//    Serial.printf("SD.begin(%d)\n", cs_pin);
//    if (!SD.begin(cs_pin, spi2, 10000000, "/sd", 5, false)) {
    if (!SD.begin(cs_pin)) {
        Serial.println("SD Card initialization failed!");
        return false;
    }
    Serial.println("SD Card initialized successfully");
    started = true;
    if (!pref_name) {
        dir_name="/log";
    } else {
        Preferences prefs;
        prefs.begin(pref_name, false);
        dir_name = prefs.getString("home_log_dir", "/log");
        Serial.printf("Home log dir: %s\n", dir_name.c_str());
        createDir(dir_name.c_str());
        int dir_number = prefs.getInt("log_dir_number", 0);
        dir_name=dir_name+"/"+String(dir_number);
        prefs.putInt("log_dir_number", dir_number+1);
        prefs.end();
    }
    createDir(dir_name.c_str());
    filename = dir_name+"/"+String(millis())+".csv";
    return true;
}

void AmperikaCRLog::setFilename(const char* filename) {
    if (filename) {
        this->filename = filename;
        Serial.printf("Log file: %s\n", filename);
        if (fileOpened){
            dataFile.close();
            dataFile = SD.open(filename, FILE_APPEND);
            if (!dataFile) {
                fileOpened = false;
            }
        }
    }
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
        dataFile.flush(); 
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

void AmperikaCRLog::createDir(const char* dir){
    if (!SD.exists(dir)) {
        SD.mkdir(dir);
    }
}

void AmperikaCRLog::newFile(const char* suffix){
    // Используем фиксированный буфер вместо String конкатенации для экономии стека
    char filename_buf[128];
    snprintf(filename_buf, sizeof(filename_buf), "%s/%lu_%s.csv", dir_name.c_str(), millis(), suffix);
    setFilename(filename_buf);
}

bool AmperikaCRLog::StartLog(const char* logName){
    closeFile();
    newFile(logName);
//    openFile();
    return true;
}

bool AmperikaCRLog::StopLog(){
    closeFile();
    newFile("NONE");
    return true;
}

bool AmperikaCRLog::Started(){
    return started;
}


//#endif



