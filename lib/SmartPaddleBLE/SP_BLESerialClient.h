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
//            Serial.printf("Received data: %d\n", pData[i]);
            client->receiveBuffer.add(pData[i]);
        }
    }

    bool setupSerialService() {
        Serial.println("Setting up serial service...");
        BLEClient* client = static_cast<SmartPaddleBLEClient*>(paddle)->getBLEClient();
        if(!client){
            Serial.println("Failed to get BLE client");
            return false;
        }

        // Получаем сервис
        serialService = client->getService(SPSerialUUID::SERVICE_UUID);
        if(!serialService){
            Serial.println("Failed to get serial service");
            return false;
        }
        delay(10);

        // Получаем характеристики
        txChar = serialService->getCharacteristic(SPSerialUUID::TX_UUID);
        delay(10);
        rxChar = serialService->getCharacteristic(SPSerialUUID::RX_UUID);
        delay(10);
        if(!txChar || !rxChar) {
            Serial.println("Failed to get serial characteristics");
            return false;
        }

        // Устанавливаем callback для уведомлений
        if(txChar->canNotify()) {
            if (!notifyCallback)
                notifyCallback = std::bind(&SP_BLESerialClient::notifySerialCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
            txChar->registerForNotify(notifyCallback);
            const uint8_t indicationOn[] = {0x1, 0x0};
            const uint8_t indicationOff[] = {0x0, 0x0};            
            txChar->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)indicationOn, 2, true);

        } else {
            Serial.println("Failed to register notify for serial characteristic");
            return false;
        }

        return true;
    }

    void disconnect() {
     }

public:
    SP_BLESerialClient(SmartPaddleBLE* p) : 
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
            // Создаем JSON задачу только при первом запуске
            // При последующих подключениях она будет переиспользоваться
            startJsonProcessTask();
        }
    }

    void end() override {
        if(!started) return;
        flush();
        disconnect();
        started = false;
        // НЕ останавливаем JSON задачу - она будет работать при переподключении
        // Задача удаляется только в деструкторе ~SP_BLESerial()
    }
};