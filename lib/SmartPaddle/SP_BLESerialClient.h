/**
 * @file SP_BLESerialClient.h
 * @brief Клиентская часть BLE Serial для SmartPaddle
 */

#pragma once
#include "SP_BLESerial.h"
#include "SmartPaddleClient.h"



class SP_BLESerialClient : public SP_BLESerial, public BLECharacteristicCallbacks {

private:
    BLERemoteService* serialService;
    BLERemoteCharacteristic* txChar;
    BLERemoteCharacteristic* rxChar;
    std::function<void(BLERemoteCharacteristic*, uint8_t*, size_t, bool)> notifyCallback;


    void notifySerialCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
//       SmartPaddle* paddle = PaddleMap[(void*)(pChar->getRemoteService()->getClient())];
//        if (paddle == nullptr) return;
 //       SP_BLESerialClient* client = (SP_BLESerialClient*)paddle->getSerial();
        SP_BLESerialClient* client = this;
        if(!client || !client->started) return;

        // Добавляем полученные данные в буфер
        for(size_t i = 0; i < length; i++) {
//            Serial.print((char)pData[i]);
            client->receiveBuffer.add(pData[i]);
        }
    }

    bool setupSerialService() {
        BLEClient* client = static_cast<SmartPaddleBLEClient*>(paddle)->getBLEClient();
        if(!client) return false;

        // Получаем сервис
        serialService = client->getService(SPSerialUUID::SERVICE_UUID);
        if(!serialService) return false;

        // Получаем характеристики
        txChar = serialService->getCharacteristic(SPSerialUUID::TX_UUID);
        rxChar = serialService->getCharacteristic(SPSerialUUID::RX_UUID);

        if(!txChar || !rxChar) return false;

        // Устанавливаем callback для уведомлений
        if(txChar->canNotify()) {
            if (!notifyCallback)
                notifyCallback = std::bind(&SP_BLESerialClient::notifySerialCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
            txChar->registerForNotify(notifyCallback);
            const uint8_t indicationOn[] = {0x1, 0x0};
            const uint8_t indicationOff[] = {0x0, 0x0};            
            txChar->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)indicationOn, 2, true);

        }

        return true;
    }

    void disconnect() {
     }

public:
    SP_BLESerialClient(SmartPaddle* p) : 
        SP_BLESerial(p)
         { }


    void flush() override {
        if(!started || !paddle->connected()) return;
        if (transmitBufferLength ==0) {lastFlushTime=millis(); return;}
        
        rxChar->writeValue(transmitBuffer, transmitBufferLength);
        
        transmitBufferLength = 0;
        lastFlushTime = millis();
    }

    // Методы BLE
    void begin() override {
        if(started) return;
        if(setupSerialService()) {
            started = true;
            startJsonProcessTask();
        }
    }

    void end() override {
        if(!started) return;
        flush();
        disconnect();
        started = false;
        stopJsonProcessTask();
    }
};