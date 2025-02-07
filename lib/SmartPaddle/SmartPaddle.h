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
    String PaddleID;                // Paddle ID
    PaddleType paddleType;          // Blade type (TWO_BLADES or ONE_BLADE)
    float length;                   // in meters
    float bladePower;
    uint16_t firmwareVersion;
    String paddleModel;
};

struct BladeOrientation{
    signed char YAxisDirection;     // 1 if Y axis directs right, -1 if Y axis directs left
    float rightBladeAngle;          // in radians
    float rightBladeVector[3];      // normal vector in paddleOrientation
    float leftBladeAngle;           // in radians
    float leftBladeVector[3];      // normal vector in paddleOrientation
};

struct PaddleStatus{
    int8_t batteryLevel;
    int8_t temperature;
};

class SP_Event_Handler {
    public:
    virtual void onUpdateIMU(IMUData& imuData, SmartPaddle* paddle) {};
    virtual void onUpdateOrientation(OrientationData& orientationData, SmartPaddle* paddle) {};
    virtual void onUpdateLoad(loadData& loadData, SmartPaddle* paddle) {};
    virtual void onUpdateBlade(BladeData& bladeData, SmartPaddle* paddle) {};
    virtual void onUpdateSpecs(PaddleSpecs& specsData, SmartPaddle* paddle) {};
    virtual void onUpdateStatus(PaddleStatus& statusData, SmartPaddle* paddle) {};
    virtual void onUpdateBladeAngle(BladeOrientation& bladeOrientation, SmartPaddle* paddle) {};
    virtual void onConnect(SmartPaddle* paddle) {};
    virtual void onDisconnect(SmartPaddle* paddle) {};
    virtual void onPairing(SmartPaddle* paddle) {};
    virtual void onPairingDone(SmartPaddle* paddle, BLEAddress* address) {};
    virtual void onUnpair(SmartPaddle* paddle) {};
    virtual void onShutdown(SmartPaddle* paddle) {};
    virtual void onError(SmartPaddle* paddle) {};
    virtual void onUpdate(SmartPaddle* paddle) {};
};



class SP_BLESerial;

// Class for smart paddle unified interface
class SmartPaddle {
    protected:
    PaddleSpecs specs;
    PaddleStatus status;
    BladeOrientation bladeOrientation;
    bool isConnected;                  // Paddle connection status
    SP_BLESerial* serial;
    BLESerialMessageHandler* messageHandler;
    static int BLEMTU;
    SP_Event_Handler* eventHandler;
    public:
    SmartPaddle():
        specs(),
        status(),
        isConnected(false),
        serial(nullptr),
        messageHandler(nullptr),
        eventHandler(nullptr){}
    virtual void begin(const char* deviceName)=0;
    virtual void setFilterFrequency(uint32_t frequency)=0;
    
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
    virtual void calibrateBladeAngle(BladeSideType blade_side)=0;
    virtual BladeOrientation getBladeAngles() {return bladeOrientation;}
    virtual PaddleSpecs getSpecs() {return specs;}
    virtual PaddleStatus getStatus() {return status;}
    virtual ~SmartPaddle();
    void setEventHandler(SP_Event_Handler* handler) {eventHandler = handler;}
 
};

#endif

