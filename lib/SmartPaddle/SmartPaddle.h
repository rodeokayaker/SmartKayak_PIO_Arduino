#ifndef SmartPaddle_h
#define SmartPaddle_h
#include <Arduino.h>
#include <MadgwickAHRS.h>
#include <BLEServer.h>
#include <BLEClient.h>
#include "SP_BLESerial.h"
#include "InterfaceIMU.h"
#include "InterfaceLoadCell.h"
#include "OverwritingQueue.h"
#define SENSOR_QUEUE_SIZE 10


// UUID для BLE сервисов и характеристик
namespace SmartPaddleUUID {
    extern const char* SERVICE_UUID;
    extern const char* FORCE_UUID;
    extern const char* IMU_UUID;
    extern const char* STATUS_UUID;
    extern const char* SPECS_UUID;
    extern const char* ORIENTATION_UUID;
    extern const char* BLADE_UUID;    
}

enum PaddleType{
    ONE_BLADE,
    TWO_BLADES
};

enum BladeSideType{
    RIGHT_BLADE,
    LEFT_BLADE,
    ALL_BLADES
};




struct BladeData {
    BladeSideType bladeSide;
    float quaternion[4];
    float force;
    uint32_t timestamp;
};


// Интерфейсы для датчиков


struct PaddleSpecs{
    uint32_t PaddleID;                // Paddle ID
    PaddleType paddleType;                // Blade type (TWO_BLADES or ONE_BLADE)
    float leftFeatheredQuaternion[4];     // Feathered orientation
    float rightFeatheredQuaternion[4];     // Feathered orientation
};

struct PaddleStatus{
    int8_t batteryLevel;
    int8_t temperature;
};



class SP_BLESerial;

// Class for smart paddle unified interface
class SmartPaddle {
    protected:
    PaddleSpecs specs;
    uint32_t KayakID;                // Kayak ID
    PaddleStatus status;
    bool isConnected;                  // Paddle connection status
    bool run_madgwick;
    SP_BLESerial* serial;
    BLESerialMessageHandler* messageHandler;
    
    public:
    SmartPaddle():
        specs(),
        KayakID(0),
        status(),
        isConnected(false),
        run_madgwick(false),
        serial(nullptr),
        messageHandler(nullptr){}
    virtual void begin(const char* deviceName)=0;
    virtual void setPaddleID(uint32_t id)=0;
    virtual void setFilterFrequency(uint32_t frequency)=0;
    virtual void setRunMadgwick(bool run){run_madgwick=run;}
    
    virtual void updateIMU()=0;
    virtual void updateLoads()=0;
    virtual void updateBLE()=0;
    virtual uint32_t paddleMillis()=0;

    virtual IMUData getIMUData()=0;
    virtual loadData getLoadData()=0;
    virtual OrientationData getOrientationData()=0;
 
    virtual bool connect()=0;                   // Connect to paddle
    virtual void disconnect()=0;               // Disconnect paddle
    bool connected() {return isConnected;}
    virtual void startPairing()=0;
    virtual SP_BLESerial* getSerial() {return serial;}
    virtual void calibrateIMU()=0;
    virtual void calibrateLoads(BladeSideType blade_side)=0;
    virtual ~SmartPaddle();

 
};

extern std::map<void*, SmartPaddle*> PaddleMap;
extern int BLEMTU;

#endif

