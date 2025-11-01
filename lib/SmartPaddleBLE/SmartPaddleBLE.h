#ifndef SMARTPADDLEBLE_H
#define SMARTPADDLEBLE_H

#include <Arduino.h>
#include "SP_BLESerial.h"
#include "SmartPaddle.h"

#define SENSOR_QUEUE_SIZE 10 // Queue size for sensor data


class SP_BLESerial;


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


class SmartPaddleBLE: public SmartPaddle {
    protected:
        PaddleStatus status;
        bool isConnected;                  // Paddle connection status
        SP_BLESerial* serial;
        SP_MessageHandler* messageHandler;
        static int BLEMTU;


    public:
        SmartPaddleBLE(): SmartPaddle(),
        status(),
        isConnected(false),
        serial(nullptr),
        messageHandler(nullptr)
        {}
        virtual void begin(const char* deviceName)=0;    

        virtual void disconnect()=0;               // Disconnect paddle
        bool connected() {return isConnected;}
        virtual void startPairing()=0;
        virtual SP_BLESerial* getSerial() {return serial;}
        
        virtual PaddleStatus getStatus() {return status;}
        virtual bool operating() override {return connected();}

        virtual ~SmartPaddleBLE();
};

#endif
