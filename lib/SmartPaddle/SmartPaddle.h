/**
 * @file SmartPaddle.h
 * @brief Базовый класс SmartPaddle
 */
#ifndef SmartPaddle_h
#define SmartPaddle_h

#include <Arduino.h>
#include <BLEServer.h>
#include <BLEClient.h>
#include "SP_BLESerial.h"
#include "InterfaceIMU.h"
#include "InterfaceLoadCell.h"
#include "OverwritingQueue.h"
#include "SP_Types.h"
#include "SP_EventHandler.h"

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

class SP_BLESerial;

// Class for smart paddle unified interface
class SmartPaddle {
    protected:
    PaddleSpecs specs;
    PaddleStatus status;
    BladeOrientation bladeOrientation;
    bool isConnected;                  // Paddle connection status
    SP_BLESerial* serial;
    SP_MessageHandler* messageHandler;
    static int BLEMTU;
    SP_EventHandler* eventHandler;
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
    void setEventHandler(SP_EventHandler* handler) {eventHandler = handler;}
 
};

#endif

